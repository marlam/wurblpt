/*
 * Copyright (C) 2017, 2018, 2019, 2020, 2021, 2022
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

#include "gvm.hpp"


namespace WurblPT {

/**
 * \brief Defines a pose, consisting of translation, rotation, and scaling.
 * This is intended to be used as a replacement for a 4x4 affine transformation matrix.
 * You can do the same things with it (translate, rotate, scale), but since the
 * three components are separated, transformations can also be cleanly interpolated
 * and you can always look at each part separately.
 *
 * Applying a transformation to a vector (using operator*) is ca. 40% more expensive
 * than applying the same transformation via matrix multiplication (on a 12th Gen Intel
 * i5 with -Ofast).
 * If you need to transform large amounts of data with the same transformation, it is
 * therefore advisable to first generate a 4x4 matrix (and/or 3x3 normal matrix) from
 * the transformation.
 */
class Transformation
{
public:
    /*! \brief Constructor */
    Transformation(const vec3& t = vec3(0.0f), const quat& r = quat::null(), const vec3& s = vec3(1.0f)) :
        translation(t), rotation(r), scaling(s)
    {
    }

    /*! \brief Translation */
    vec3 translation;
    /*! \brief Rotation */
    quat rotation;
    /*! \brief Scaling */
    vec3 scaling;

    /*! \brief Returns whether this is the identity transformation */
    bool isIdentity() const
    {
        return (all(equal(translation, vec3(0.0f)))
                && rotation.w >= 1.0f
                && all(equal(scaling, vec3(1.0f))));
    }

    /*! \brief Returns whether two transformations are identical */
    bool operator==(const Transformation& t) const
    {
        return (all(equal(translation, t.translation))
                && rotation == t.rotation
                && all(equal(scaling, t.scaling)));
    }

    /*! \brief Applies this transformation to a vector */
    vec3 operator*(const vec3& v) const
    {
        return translation + (rotation * (v * scaling));
    }

    /*! \brief Applies a translation */
    void translate(const vec3& v)
    {
        translation += rotation * (v * scaling);
    }

    /*! \brief Applies a rotation */
    void rotate(const quat& q)
    {
        rotation *= q;
    }

    /*! \brief Applies a scaling */
    void scale(const vec3& s)
    {
        scaling *= s;
    }

    /*! \brief Returns this transformation as a 4x4 matrix. Useful for transforming a lot of data
     * since matrix multiplication is cheaper than the transformation operator*. */
    mat4 toMat4() const
    {
        mat4 M(vec4(1.0f, 0.0f, 0.0f, 0.0f),
               vec4(0.0f, 1.0f, 0.0f, 0.0f),
               vec4(0.0f, 0.0f, 1.0f, 0.0f),
               vec4(translation, 1.0f));
        M *= WurblPT::toMat4(rotation);
        M.scale(scaling);
        return M;
    }

    /*! \brief Returns the 3x3 normal matrix for this transformation (i.e. the matrix to transform normals with).
     * Note that we don't need to bother with inverse(transpose(toMat4())) since we can return
     * just the rotation part. */
    mat3 toNormalMatrix() const
    {
        return toMat3(rotation);
    }

    /*! \brief Create a transformation from a common definition of a view matrix */
    static Transformation fromLookAt(const vec3& eye, const vec3& center, const vec3& up = vec3(0.0f, 1.0f, 0.0f))
    {
        // See documentation of gluLookAt for the meaning of f,s,u:
        // https://www.khronos.org/registry/OpenGL-Refpages/gl2.1/xhtml/gluLookAt.xml
        vec3 f = normalize(center - eye);
        vec3 s = normalize(cross(f, up));
        vec3 u = cross(s, f);
        // Instead of building a matrix, we need the rotation as a quaternion
        quat rot0 = toQuat(vec3(0.0f, 0.0f, -1.0f), f);
        quat rot1 = toQuat(rot0 * vec3(0.0f, 1.0f, 0.0f), u);
        return Transformation(eye, rot1 * rot0);
    }

    /*! \brief Return the "look from" (or origin) point of this transformation */
    vec3 lookFrom() const
    {
        return translation;
    }

    /*! \brief Return the "look at" vector of this transformation */
    vec3 lookAt() const
    {
        return translation + rotation * vec3(0.0f, 0.0f, -1.0f);
    }

    /*! \brief Return the "up" vector of this transformation */
    vec3 up() const
    {
        return rotation * vec3(0.0f, 1.0f, 0.0f);
    }
};

inline Transformation inverse(const Transformation& T)
{
    vec3 invT = -T.translation;
    quat invR = -T.rotation;
    vec3 invS = 1.0f / T.scaling;
    return Transformation(invR * (invT * invS), invR, invS);
}

inline Transformation translate(const Transformation& T, const vec3& v)
{
    Transformation R = T;
    R.translate(v);
    return R;
}

inline Transformation rotate(const Transformation& T, const quat& q)
{
    Transformation R = T;
    R.rotate(q);
    return R;
}

inline Transformation scale(const Transformation& T, const vec3& s)
{
    Transformation R = T;
    R.scale(s);
    return R;
}

inline Transformation operator*(const Transformation& S, const Transformation& T)
{
    return scale(rotate(translate(S, T.translation), T.rotation), T.scaling);
}

inline Transformation& operator*=(Transformation& S, const Transformation& T)
{
    return S = S * T;
}

/*! \brief Interpolate two transformations. The value of \a alpha should be in [0,1],
 * where 0 results in \a T0, and 1 results in \a T1. Positions are interpolated
 * linearly and rotations are interpolated via spherical linear interpolation (slerp). */
inline Transformation mix(const Transformation& T0, const Transformation& T1, float alpha)
{
    return Transformation(
            mix(T0.translation, T1.translation, alpha),
            slerp(T0.rotation, T1.rotation, alpha),
            mix(T0.scaling, T1.scaling, alpha));
}

}
