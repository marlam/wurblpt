/*
 * Copyright (C) 2019, 2020, 2021, 2022
 * Computer Graphics Group, University of Siegen (written by Martin Lambers)
 * Copyright (C) 2022, 2023
 * Martin Lambers <marlam@marlam.de>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include <omp.h>

#include <cstdio>
#include <cstdlib>

#include <vector>
#include <algorithm>

#include "aabb.hpp"
#include "animation.hpp"
#include "hitable.hpp"


namespace WurblPT {

class Ray;
class Prng;

class AABBSorter
{
private:
    const std::vector<AABB>& _aabbs;
    const int _axis;

public:
    AABBSorter(const std::vector<AABB>& aabbs, int axis) :
        _aabbs(aabbs), _axis(axis)
    {
    }

    bool operator() (unsigned int i, unsigned int j)
    {
        return (_aabbs[i].center()[_axis] < _aabbs[j].center()[_axis]);
    }
};

class BVHNode
{
private:
    AABB _aabb;
    union {
        BVHNode* _children[2];
        const Hitable* _hitable;
    };
    bool _isLeaf;

    float sah(const std::vector<float>& areas0, const std::vector<float>& areas1,
            size_t subset_offset, size_t subset_size, int i)
    {
        return i * areas0[subset_offset + i - 1] + (subset_size - i) * areas1[subset_offset + i];
    }

public:
    BVHNode() : _isLeaf(true)
    {
    }

    ~BVHNode()
    {
        if (!_isLeaf) {
            delete _children[0];
            delete _children[1];
        }
    }

    void build(const std::vector<const Hitable*>& hitables,
            const std::vector<AABB>& aabbs,
            std::vector<unsigned int>& subset,
            std::vector<float>& areas0, std::vector<float>& areas1,
            size_t subset_offset, size_t subset_size,
            unsigned int treeLevel = 0)
    {
        assert(subset_size > 0);
        if (subset_size == 1) {
            _aabb = aabbs[subset[subset_offset + 0]];
            _hitable = hitables[subset[subset_offset + 0]];
            _isLeaf = true;
        } else {
            // find the AABB for the complete subset, determine its longest
            // axis, and sort the hitables along that axis
            int n = subset_size; // convenient to have it as an int below
            _aabb = aabbs[subset[subset_offset + 0]];
            for (int i = 1; i < n; i++)
                _aabb = merge(_aabb, aabbs[subset[subset_offset + i]]);
            AABBSorter sorter(aabbs, _aabb.longestAxis());
            std::sort(subset.begin() + subset_offset, subset.begin() + subset_offset + subset_size, sorter);
            // The splitIndex is the index of the first hitable that goes to
            // _children[1]; all before that go to _children[0]. It must be in
            // [1, n-1] so that both children get at least one hitable.
            // That means we never need areas0[n-1] or areas1[0], which is why
            // we don't compute them in the loops below.
            // Note: parallelizing the computation of areas0 and areas1 is not worth the effort.
            AABB box0 = aabbs[subset[subset_offset + 0]];
            areas0[subset_offset + 0] = box0.surfaceArea();
            for (int i = 1; i < n - 1; i++) {
                box0 = merge(box0, aabbs[subset[subset_offset + i]]);
                areas0[subset_offset + i] = box0.surfaceArea();
            }
            AABB box1 = aabbs[subset[subset_offset + n - 1]];
            areas1[subset_offset + n - 1] = box1.surfaceArea();
            for (int i = n - 2; i > 0; i--) {
                box1 = merge(box1, aabbs[subset[subset_offset + i]]);
                areas1[subset_offset + i] = box1.surfaceArea();
            }
            // find the optimal splitIndex according to SAH
            int splitIndex = 1;
            float minSAH = sah(areas0, areas1, subset_offset, subset_size, 1);
            for (int i = 2; i < n; i++) {
                float SAH = sah(areas0, areas1, subset_offset, subset_size, i);
                if (SAH < minSAH) {
                    minSAH = SAH;
                    splitIndex = i;
                }
            }
            // divide hitables into two new sets
            size_t subset0_offset = subset_offset;
            size_t subset0_size = splitIndex;
            size_t subset1_offset = subset_offset + splitIndex;
            size_t subset1_size = subset_size - subset0_size;
            _children[0] = new BVHNode;
            _children[1] = new BVHNode;
            constexpr unsigned int parallelizationThreshold = 16384;
            if (subset0_size >= parallelizationThreshold && subset1_size >= parallelizationThreshold) {
                #pragma omp parallel sections num_threads(2)
                {
                    #pragma omp section
                    _children[0]->build(hitables, aabbs, subset, areas0, areas1, subset0_offset, subset0_size, treeLevel + 1);
                    #pragma omp section
                    _children[1]->build(hitables, aabbs, subset, areas0, areas1, subset1_offset, subset1_size, treeLevel + 1);
                }
            } else {
                _children[0]->build(hitables, aabbs, subset, areas0, areas1, subset0_offset, subset0_size, treeLevel + 1);
                _children[1]->build(hitables, aabbs, subset, areas0, areas1, subset1_offset, subset1_size, treeLevel + 1);
            }
            _isLeaf = false;
        }
    }

    static const BVHNode* buildBVH(size_t maxTreeDepth,
            const std::vector<const Hitable*> hitables,
            AnimationCache& animationCacheT0, AnimationCache& animationCacheT1)
    {
        fprintf(stderr, "Building bounding volume hierarchy for %zu hitables for %.3fs-%.3fs\n",
                hitables.size(), animationCacheT0.t(), animationCacheT1.t());
        BVHNode* rootNode = nullptr;
        if (hitables.size() > 0) {
            rootNode = new BVHNode;
            std::vector<AABB> aabbs(hitables.size());
            std::vector<unsigned int> subset(hitables.size());
            for (size_t i = 0; i < hitables.size(); i++) {
                aabbs[i] = hitables[i]->aabb(animationCacheT0, animationCacheT1);
                subset[i] = i;
            }
            std::vector<float> areas0(hitables.size());
            std::vector<float> areas1(hitables.size());
            // build the tree with nested parallelization
            assert(size_t(omp_get_supported_active_levels()) >= maxTreeDepth);
            int max_active_levels_bak = omp_get_max_active_levels();
            omp_set_max_active_levels(maxTreeDepth);
            rootNode->build(hitables, aabbs, subset, areas0, areas1, 0, subset.size());
            omp_set_max_active_levels(max_active_levels_bak);
        }
        return rootNode;
    }

    void measure(size_t& nodeCount, size_t& maxDepth, size_t depth = 1) const
    {
        size_t nc1 = 0;
        size_t md1 = depth;
        size_t nc2 = 0;
        size_t md2 = depth;
        if (!_isLeaf) {
            _children[0]->measure(nc1, md1, depth + 1);
            _children[1]->measure(nc2, md2, depth + 1);
        }
        nodeCount = 1 + nc1 + nc2;
        maxDepth = max(md1, md2);
    }

    friend class BVH;
};

#ifdef __GNUC__
# define WURBLPT_ATTRIB_ALIGN32 __attribute__ ((aligned (32)))
#else
# define WURBLPT_ATTRIB_ALIGN32 __declspec(align(32))
#endif
class WURBLPT_ATTRIB_ALIGN32 BVHNodeLinear
{
public:
    AABB aabb;
    union {
        size_t child2IndexMulTwoPlusOne;
        const Hitable* hitable;
    };
};

class BVH : public Hitable
{
private:
    static const size_t maxTreeDepth = 128;
    std::vector<BVHNodeLinear> _nodes;

    size_t flatten(const BVHNode* node, size_t* offset)
    {
        size_t myOffset = (*offset)++;
        BVHNodeLinear* linearNode = &(_nodes[myOffset]);
        linearNode->aabb = node->_aabb;
        if (node->_isLeaf) {
            linearNode->hitable = node->_hitable;
        } else {
            flatten(node->_children[0], offset);
            size_t child2Index = flatten(node->_children[1], offset);
            linearNode->child2IndexMulTwoPlusOne = child2Index * 2 + 1;
        }
        return myOffset;
    }

public:
    BVH()
    {
        static_assert(sizeof(BVHNodeLinear) == 32);
    }

    void build(const std::vector<const Hitable*> hitables,
            AnimationCache& animationCacheT0, AnimationCache& animationCacheT1)
    {
        const BVHNode* root = BVHNode::buildBVH(maxTreeDepth, hitables, animationCacheT0, animationCacheT1);
        size_t nodeCount, maxDepth;
        root->measure(nodeCount, maxDepth);
        if (maxDepth > maxTreeDepth) {
            fprintf(stderr, "Too many levels\n");
            abort();
        }
        fprintf(stderr, "Linearizing bounding volume hierarchy with %zu nodes on %zu levels\n", nodeCount, maxDepth);
        _nodes.resize(nodeCount);
        //assert(reinterpret_cast<size_t>(_nodes.data()) % 32 == 0);
        size_t offset = 0;
        flatten(root, &offset);
        delete root;
    }

    virtual AABB aabb(AnimationCache&, AnimationCache&) const override
    {
        return _nodes[0].aabb;
    }

    virtual HitRecord hit(const Ray& ray, const RayIntersectionHelper& rayHelper,
            float amin, float amax, float minHitDistance,
            AnimationCache& animationCache,
            Prng& prng) const override
    {
        HitRecord hr; // initialized with haveHit=false
        size_t toVisitOffset = 0;
        size_t currentNodeIndex = 0;
        size_t nodesToVisit[maxTreeDepth];
        for (;;) {
            const BVHNodeLinear& node = _nodes[currentNodeIndex];
            if (node.aabb.mayHit(ray, amin, amax, rayHelper.invDirection)) {
                if (node.child2IndexMulTwoPlusOne % 2 == 0) {
                    HitRecord currentHr = node.hitable->hit(ray, rayHelper, amin, amax, minHitDistance,
                            animationCache, prng);
                    if (currentHr.haveHit) {
                        hr = currentHr;
                        amax = hr.a;
                    }
                    if (toVisitOffset == 0)
                        break;
                    currentNodeIndex = nodesToVisit[--toVisitOffset];
                } else {
                    nodesToVisit[toVisitOffset++] = node.child2IndexMulTwoPlusOne / 2; // child 2
                    currentNodeIndex++; // child 1
                }
            } else {
                if (toVisitOffset == 0)
                    break;
                currentNodeIndex = nodesToVisit[--toVisitOffset];
            }
        }

        return hr;
    }
};

}
