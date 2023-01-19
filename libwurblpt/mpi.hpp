/*
 * Copyright (C) 2023
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

#include <cassert>

#include <vector>
#include <thread>

#ifdef WURBLPT_WITH_MPI
# include <mpi.h>
#endif


namespace WurblPT {

constexpr int MPITagGetBlockRequest = 1;
constexpr int MPITagGetBlockAnswer = 2;
constexpr int MPITagSubmitBlockRequest = 3;

class MPICoordinatorMain
{
private:
    unsigned int _blockCount;
    unsigned int _blockSize;
    unsigned int _lastBlockSize;
    float* _pixelData;
    unsigned int _componentCount;
    unsigned int _nextBlockIndex;
    unsigned int _finishedRanksCount;
    std::vector<unsigned int> _rankBlockIndices;

public:
    MPICoordinatorMain(unsigned int worldSize,
            unsigned int blockCount, unsigned int blockSize, unsigned int lastBlockSize,
            float* pixelData, unsigned int componentCount) :
        _blockCount(blockCount),
        _blockSize(blockSize),
        _lastBlockSize(lastBlockSize),
        _pixelData(pixelData),
        _componentCount(componentCount),
        _nextBlockIndex(0),
        _finishedRanksCount(0),
        _rankBlockIndices(worldSize)
    {
    }

    std::thread spawn()
    {
        return std::thread(&MPICoordinatorMain::mainLoop, this);
    }

    void mainLoop()
    {
#ifdef WURBLPT_WITH_MPI
        while (_finishedRanksCount < _rankBlockIndices.size()) {
            MPI_Status status;
            MPI_Probe(MPI_ANY_SOURCE, MPI_ANY_TAG, MPI_COMM_WORLD, &status);
            if (status.MPI_TAG == MPITagGetBlockRequest) {
                MPI_Recv(nullptr, 0, MPI_UNSIGNED, status.MPI_SOURCE, MPITagGetBlockRequest, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
                unsigned int blockIndex;
                unsigned int blockInfo[2]; // start and size
                if (_nextBlockIndex >= _blockCount) {
                    blockIndex = 0;
                    blockInfo[0] = 0;
                    blockInfo[1] = 0;
                } else {
                    blockIndex = _nextBlockIndex++;
                    blockInfo[0] = blockIndex * _blockSize;
                    blockInfo[1] = (blockIndex == _blockCount - 1) ? _lastBlockSize : _blockSize;
                }
                MPI_Send(blockInfo, 2, MPI_UNSIGNED, status.MPI_SOURCE, MPITagGetBlockAnswer, MPI_COMM_WORLD);
                _rankBlockIndices[status.MPI_SOURCE] = blockIndex;
                if (blockInfo[1] == 0)
                    _finishedRanksCount++;
            } else if (status.MPI_TAG == MPITagSubmitBlockRequest) {
                unsigned int blockStart = _rankBlockIndices[status.MPI_SOURCE] * _blockSize;
                int count = 0;
                MPI_Get_count(&status, MPI_FLOAT, &count);
                MPI_Recv(_pixelData + blockStart * _componentCount,
                        count, MPI_FLOAT, status.MPI_SOURCE, MPITagSubmitBlockRequest,
                        MPI_COMM_WORLD, MPI_STATUS_IGNORE);
            }
        }
#endif
    }
};

class MPICoordinatorChild
{
private:
    int _mpiRank;
    float* _pixelData;
    unsigned int _componentCount;

public:
    MPICoordinatorChild(int mpiRank, float* pixelData, unsigned int componentCount) :
        _mpiRank(mpiRank),
        _pixelData(pixelData),
        _componentCount(componentCount)
    {
    }

    void getBlock(unsigned int* blockStart, unsigned int* blockSize)
    {
#ifdef WURBLPT_WITH_MPI
        MPI_Send(nullptr, 0, MPI_UNSIGNED, 0, MPITagGetBlockRequest, MPI_COMM_WORLD);
        unsigned int blockInfo[2];
        MPI_Recv(blockInfo, 2, MPI_UNSIGNED, 0, MPITagGetBlockAnswer, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
        *blockStart = blockInfo[0];
        *blockSize = blockInfo[1];
#else
        (void)blockStart;
        (void)blockSize;
#endif
    }

    void submitBlock(unsigned int blockStart, unsigned int blockSize)
    {
#ifdef WURBLPT_WITH_MPI
        MPI_Send(_pixelData + blockStart * _componentCount,
                blockSize * _componentCount, MPI_FLOAT,
                0, MPITagSubmitBlockRequest, MPI_COMM_WORLD);
#else
        (void)blockStart;
        (void)blockSize;
#endif
    }
};

class MPICoordinator
{
private:
    unsigned int _blockSize;
    int _mpiWorldSize;
    int _mpiWorldRank;
    char _processId[16];
    bool _singleBlockCaseDone;
    unsigned int _pixelCount;
    MPICoordinatorMain* _main;
    std::thread _mainThread;
    MPICoordinatorChild* _child;

public:
    /* Constructor.
     * The default block size is chosen so that
     * - A block is typically large enough so that the OpenMP-parallelized loop
     *   over its pixels can make use of all available processor cores
     * - A block is typically small enough so that each MPI rank gets to
     *   compute enough blocks for dynamic load balancing to make sense
     * - The block size is neither too small nor too large so that communication
     *   overhead (requesting a new block, submitting a finished blocks) does
     *   not result in wasting CPU time
     * Nevertheless, for special cases (e.g. testing, benchmarking), the user
     * may want to chose a different block size.
     */
    MPICoordinator(unsigned int blockSize = 4096) :
        _blockSize(blockSize),
        _mpiWorldSize(1),
        _mpiWorldRank(0),
        _processId { 'm', 'a', 'i', 'n', '\0' },
        _singleBlockCaseDone(false),
        _pixelCount(0),
        _main(nullptr),
        _child(nullptr)
    {
#ifdef WURBLPT_WITH_MPI
        int provided = MPI_THREAD_SINGLE;
        MPI_Init_thread(nullptr, nullptr, MPI_THREAD_MULTIPLE, &provided);
        if (provided < MPI_THREAD_MULTIPLE) {
            fprintf(stderr, "This MPI implementation does not support MPI_THREAD_MULTIPLE\n");
            abort();
        }
        MPI_Comm_size(MPI_COMM_WORLD, &_mpiWorldSize);
        MPI_Comm_rank(MPI_COMM_WORLD, &_mpiWorldRank);
        if (_mpiWorldSize > 1) {
            std::snprintf(_processId, sizeof(_processId), "MPI rank %d", _mpiWorldRank);
        }
        char hostname[256];
        gethostname(hostname, sizeof(hostname));
        fprintf(stderr, "MPI rank %d/%d runs on %s\n", _mpiWorldRank, _mpiWorldSize, hostname);
#endif
    }

    ~MPICoordinator()
    {
        assert(!_main);
        assert(!_child);
#ifdef WURBLPT_WITH_MPI
        MPI_Finalize();
#endif
    }

    const char* processId() const
    {
        return _processId;
    }

    void init(unsigned int width, unsigned int height,
            float* pixelData, unsigned int componentCount)
    {
        assert(!_main);
        assert(!_child);
        _pixelCount = width * height;
        if (_mpiWorldSize > 1) {
            if (_mpiWorldRank == 0) {
                unsigned int blockCount = _pixelCount / _blockSize;
                if (blockCount * _blockSize != _pixelCount)
                    blockCount++;
                unsigned int lastBlockSize = _pixelCount - (blockCount - 1) * _blockSize;
                _main = new MPICoordinatorMain(_mpiWorldSize, blockCount, _blockSize, lastBlockSize, pixelData, componentCount);
                _mainThread = _main->spawn();
            }
            _child = new MPICoordinatorChild(_mpiWorldRank, pixelData, componentCount);
        } else {
            _singleBlockCaseDone = false;
        }
    }

    void getBlock(unsigned int* blockStart, unsigned int* blockSize)
    {
        if (_mpiWorldSize > 1) {
            _child->getBlock(blockStart, blockSize);
        } else {
            *blockStart = 0;
            if (_singleBlockCaseDone) {
                *blockSize = 0;
            } else {
                *blockSize = _pixelCount;
                _singleBlockCaseDone = true;
            }
        }
    }

    void submitBlock(unsigned int blockStart, unsigned int blockSize)
    {
        if (_mpiWorldSize > 1) {
            _child->submitBlock(blockStart, blockSize);
        }
    }

    void finish()
    {
#ifdef WURBLPT_WITH_MPI
        MPI_Barrier(MPI_COMM_WORLD);
#endif
        if (_mpiWorldSize > 1) {
            assert(_child);
            delete _child;
            _child = nullptr;
            if (_main) {
                _mainThread.join();
                delete _main;
                _main = nullptr;
            }
        }
    }

    bool mainProcess() const
    {
        return _mpiWorldRank == 0;
    }

    int worldSize() const
    {
        return _mpiWorldSize;
    }
};

}
