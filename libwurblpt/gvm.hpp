/*
 * Copyright (C) 2008, 2009, 2010, 2011, 2012, 2013, 2014, 2015, 2019, 2020,
 * 2021, 2022, 2023
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

/*
 * These are vector and matrix classes that resemble the GLSL types vec2, vec3,
 * vec4, mat2, mat3, mat4, mat2x3, mat3x2, mat2x4, mat4x2, mat3x4, mat4x3 (and
 * the variants bvec, ivec, dvec, dmat).
 * Additionally, there is a quaternion class (quat and dquat) and a frustum class
 * (frust and dfrust).
 *
 * Vector elements are called (x,y,z,w) and (r,g,b,a). Swizzling is supported
 * only via function calls because it otherwise becomes an unholy mess even in
 * modern C++.
 *
 * The data in matrices is column-major, like OpenGL. Use transpose() to exchange
 * data with row-major libraries.
 */

#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <bit>


namespace WurblPT
{
    /* Basic numeric constants, inspired by the ones in good old math.h, but using
     * variable templates like C++20 <numbers> does, however with <float> as the default type. */

    template<typename T> constexpr T e_v = T(2.7182818284590452353602874713526625L);
    inline constexpr float e = e_v<float>;
    template<typename T> constexpr T log2e_v = T(1.4426950408889634073599246810018921L);
    inline constexpr float log2e = log2e_v<float>;
    template<typename T> constexpr T log10e_v = T(0.4342944819032518276511289189166051L);
    inline constexpr float log10e = log10e_v<float>;
    template<typename T> constexpr T ln2_v = T(0.6931471805599453094172321214581766L);
    inline constexpr float ln2 = ln2_v<float>;
    template<typename T> constexpr T ln10_v = T(2.3025850929940456840179914546843642L);
    inline constexpr float ln10 = ln10_v<float>;
    template<typename T> constexpr T pi_v = T(3.1415926535897932384626433832795029L);
    inline constexpr float pi = pi_v<float>;
    template<typename T> constexpr T pi_2_v = T(1.5707963267948966192313216916397514L);
    inline constexpr float pi_2 = pi_2_v<float>;
    template<typename T> constexpr T pi_4_v = T(0.7853981633974483096156608458198757L);
    inline constexpr float pi_4 = pi_4_v<float>;
    template<typename T> constexpr T inv_pi_v = T(0.3183098861837906715377675267450287L);
    inline constexpr float inv_pi = inv_pi_v<float>;
    template<typename T> constexpr T sqrt2_v = T(1.4142135623730950488016887242096981L);
    inline constexpr float sqrt2 = sqrt2_v<float>;
    template<typename T> constexpr T inv_sqrt2_v = T(0.7071067811865475244008443621048490L);
    inline constexpr float inv_sqrt2 = inv_sqrt2_v<float>;

    template<typename T> constexpr T maxval_v = std::numeric_limits<T>::max();
    inline constexpr float maxval = maxval_v<float>;
    template<typename T> constexpr T minval_v = std::numeric_limits<T>::lowest();
    inline constexpr float minval = minval_v<float>;
    template<typename T> constexpr T epsilon_v = std::numeric_limits<T>::epsilon();
    inline constexpr float epsilon = epsilon_v<float>;
    // epsilon is 1e-19 for long double, 1e-16 for double and 1e-7 for float


    /* GLSL functions for the subset of the base data types for which the function makes sense. */

    template<typename T> constexpr T min(T x, T y) requires (std::is_arithmetic_v<T>) { return (x < y ? x : y); }
    // Bonus: min() for 3 and 4 arguments
    template<typename T> constexpr T min(T x, T y, T z) { return min(min(x, y), z); }
    template<typename T> constexpr T min(T x, T y, T z, T w) { return min(min(min(x, y), z), w); }

    template<typename T> constexpr T max(T x, T y) requires (std::is_arithmetic_v<T>) { return (x > y ? x : y); }
    // Bonus: max() for 3 and 4 arguments
    template<typename T> constexpr T max(T x, T y, T z) { return max(max(x, y), z); }
    template<typename T> constexpr T max(T x, T y, T z, T w) { return max(max(max(x, y), z), w); }

    template<typename T> constexpr T clamp(T x, T minval, T maxval) requires (std::is_arithmetic_v<T>) { return min(maxval, max(minval, x)); }

    template<typename T> constexpr T step(T edge, T x) requires (std::is_arithmetic_v<T>) { return (x < edge ? T(0) : T(1)); }

    template<typename T> constexpr T mod(T x, T y) requires (std::is_integral_v<T>) { return x - (x / y) * y; }
    template<typename T> constexpr T mod(T x, T y) requires (std::is_floating_point_v<T>) { return x - std::floor(x / y) * y; }

    template<typename T> constexpr T sign(T x) requires (std::is_arithmetic_v<T>) { return (x < T(0) ? T(-1) : x > T(0) ? T(1) : T(0)); }

    template<typename T> constexpr bool signbit(T x) requires (std::is_floating_point_v<T>) { return std::signbit(x); }

    using std::abs;

    template<typename T> constexpr T deg2rad_v = pi_v<T> / T(180.0L);
    template<typename T> constexpr T rad2deg_v = T(180.0L) / pi_v<T>;
    template<typename T> constexpr T radians(T x) requires (std::is_floating_point_v<T>) { return x * deg2rad_v<T>; }
    template<typename T> constexpr T degrees(T x) requires (std::is_floating_point_v<T>) { return x * rad2deg_v<T>; }

    using std::sin;

    using std::cos;

    using std::tan;

    using std::asin;

    using std::acos;

    using std::atan;

    template<typename T> constexpr T atan(T y, T x) requires (std::is_floating_point_v<T>) { return std::atan2(y, x); }

    using std::pow;

    using std::exp;

    using std::exp2;

    using std::log;

    using std::log2;

    using std::log10;

    template<typename T> constexpr T sqr(T x) requires (std::is_arithmetic_v<T>) { return x * x; }

    using std::sqrt;

    template<typename T> constexpr T inversesqrt(T x) requires (std::is_floating_point_v<T>) { return T(1) / std::sqrt(x); }

    using std::cbrt;

    using std::round;

    using std::floor;

    using std::ceil;

    template<typename T> constexpr T fract(T x) requires (std::is_floating_point_v<T>) { return x - std::floor(x); }

    using std::isfinite;

    using std::isnan;

    using std::isinf;

    using std::isnormal;

    template<typename T> constexpr T mix(T x, T y, T alpha) requires (std::is_floating_point_v<T>) { return x + alpha * (y - x); }

    template<typename T> constexpr T smoothstep(T e0, T e1, T x) requires (std::is_floating_point_v<T>)
    {
        T t = clamp((x - e0) / (e1 - e0), T(0), T(1));
        return t * t * (T(3) - T(2) * t);
    }

    template<typename T> constexpr bool greaterThan(T x, T y) requires (std::is_arithmetic_v<T>) { return x > y; }

    template<typename T> constexpr bool greaterThanEqual(T x, T y) requires (std::is_arithmetic_v<T>) { return x >= y; }

    template<typename T> constexpr bool lessThan(T x, T y) requires (std::is_arithmetic_v<T>) { return x < y; }

    template<typename T> constexpr bool lessThanEqual(T x, T y) requires (std::is_arithmetic_v<T>) { return x <= y; }

    template<typename T> constexpr bool equal(T x, T y) requires (std::is_arithmetic_v<T>) { return x == y; }

    template<typename T> constexpr bool notEqual(T x, T y) requires (std::is_arithmetic_v<T>) { return x != y; }

    constexpr bool any(bool a) { return a; }

    constexpr bool all(bool a) { return a; }

    constexpr bool negate(bool a) { return !a; }

    // Bonus: popcount for unsigned integers
    template<typename T> constexpr int popcount(T x) requires (std::is_unsigned_v<T>) { return std::popcount(x); }

    // Bonus: power-of-two check for positive integers
    template<typename T> constexpr bool is_pow2(T x) requires (std::is_unsigned_v<T>)
    {
#if __cpp_lib_int_pow2 >= 202002
        return std::has_single_bit(x);
#else
        return (x > T(0) && (x & (x - T(1))) == T(0));
#endif
    }

    // Bonus: return the next power of two, or x itself if it already is a power of two
    template<typename T> constexpr bool next_pow2(T x) requires (std::is_unsigned_v<T>)
    {
#if __cpp_lib_int_pow2 >= 202002
        return std::bit_ceil(x);
#else
        return (x < T(1) ? T(1) : (x & (x - T(1))) == T(0) ? x : T(1) << (log2(x) + T(1)));
#endif
    }

    // Bonus: return the next multiple of b (> 0) that is greater than or equal to a (>= 0).
    template<typename T> constexpr T next_multiple(T a, T b) requires (std::is_integral_v<T>) { return ((a / b) + (a % b == T(0) ? T(0) : T(1))) * b; }


    /* Vector */

    template<typename T, int N> class vector
    {
    public:
        T values[N];

        /* Common constructors */

        vector() {}

        vector(T x)
        {
            for (int i = 0; i < N; i++)
                values[i] = x;
        }

        explicit vector(const T* v)
        {
            for (int i = 0; i < N; i++)
                values[i] = v[i];
        }

        template<typename U> explicit vector(const vector<U, 2>& v)
        {
            for (int i = 0; i < N; i++)
                values[i] = v[i];
        }

        /* Constructors for N=2,3,4 */

        explicit vector(T x, T y) requires (N == 2) : values { x, y } {}

        explicit vector(T x, T y, T z) requires (N == 3) : values { x, y, z } {}
        explicit vector(const vector<T, 2>& xy, T z) requires (N == 3) : values { xy[0], xy[1], z } {}
        explicit vector(T x, const vector<T, 2>& yz) requires (N == 3) : values { x, yz[0], yz[1] } {}

        explicit vector(T x, T y, T z, T w) requires (N == 4) : values { x, y, z, w } {}
        explicit vector(const vector<T, 2>& xy, T z, T w) requires (N == 4) : values { xy[0], xy[1], z, w } {}
        explicit vector(const vector<T, 2>& xy, const vector<T, 2>& zw) requires (N == 4) : values { xy[0], xy[1], zw[0], zw[1] } {}
        explicit vector(T x, const vector<T, 2>& yz, T w) requires (N == 4) : values { x, yz[0], yz[1], w } {}
        explicit vector(T x, T y, const vector<T, 2>& zw) requires (N == 4) : values { x, y, zw[0], zw[1] } {}
        explicit vector(const vector<T, 3>& xyz, T w) requires (N == 4) : values { xyz[0], xyz[1], xyz[2], w } {}
        explicit vector(T x, const vector<T, 3>& yzw) requires (N == 4) : values { x, yzw[0], yzw[1], yzw[2] } {}

        /* Common data access */

        operator const T*() const { return values; }
        T& operator[](std::ptrdiff_t i) { return values[i]; }
        T operator[](std::ptrdiff_t i) const { return values[i]; }

        /* Data access for N=2,3,4 */

        T& x()       requires (N >= 1 && N <= 4) { return values[0]; }
        T  x() const requires (N >= 1 && N <= 4) { return values[0]; }
        T& y()       requires (N >= 2 && N <= 4) { return values[1]; }
        T  y() const requires (N >= 2 && N <= 4) { return values[1]; }
        T& z()       requires (N >= 3 && N <= 4) { return values[2]; }
        T  z() const requires (N >= 3 && N <= 4) { return values[2]; }
        T& w()       requires (N == 4          ) { return values[3]; }
        T  w() const requires (N == 4          ) { return values[3]; }

        T& r()       requires (N >= 1 && N <= 4) { return values[0]; }
        T  r() const requires (N >= 1 && N <= 4) { return values[0]; }
        T& g()       requires (N >= 2 && N <= 4) { return values[1]; }
        T  g() const requires (N >= 2 && N <= 4) { return values[1]; }
        T& b()       requires (N >= 3 && N <= 4) { return values[2]; }
        T  b() const requires (N >= 3 && N <= 4) { return values[2]; }
        T& a()       requires (N == 4          ) { return values[3]; }
        T  a() const requires (N == 4          ) { return values[3]; }

        T& s()       requires (N >= 1 && N <= 4) { return values[0]; }
        T  s() const requires (N >= 1 && N <= 4) { return values[0]; }
        T& t()       requires (N >= 2 && N <= 4) { return values[1]; }
        T  t() const requires (N >= 2 && N <= 4) { return values[1]; }
        T& p()       requires (N >= 3 && N <= 4) { return values[2]; }
        T  p() const requires (N >= 3 && N <= 4) { return values[2]; }
        T& q()       requires (N == 4          ) { return values[3]; }
        T  q() const requires (N == 4          ) { return values[3]; }

        vector<T, 2> xx() const requires (N >= 1 && N <= 4) { return vector<T, 2>(values[0], values[0]); }
        vector<T, 2> xy() const requires (N >= 2 && N <= 4) { return vector<T, 2>(values[0], values[1]); }
        vector<T, 2> xz() const requires (N >= 3 && N <= 4) { return vector<T, 2>(values[0], values[2]); }
        vector<T, 2> xw() const requires (N >= 4 && N <= 4) { return vector<T, 2>(values[0], values[3]); }
        vector<T, 2> yx() const requires (N >= 2 && N <= 4) { return vector<T, 2>(values[1], values[0]); }
        vector<T, 2> yy() const requires (N >= 2 && N <= 4) { return vector<T, 2>(values[1], values[1]); }
        vector<T, 2> yz() const requires (N >= 3 && N <= 4) { return vector<T, 2>(values[1], values[2]); }
        vector<T, 2> yw() const requires (N >= 4 && N <= 4) { return vector<T, 2>(values[1], values[3]); }
        vector<T, 2> zx() const requires (N >= 3 && N <= 4) { return vector<T, 2>(values[2], values[0]); }
        vector<T, 2> zy() const requires (N >= 3 && N <= 4) { return vector<T, 2>(values[2], values[1]); }
        vector<T, 2> zz() const requires (N >= 3 && N <= 4) { return vector<T, 2>(values[2], values[2]); }
        vector<T, 2> zw() const requires (N >= 4 && N <= 4) { return vector<T, 2>(values[2], values[3]); }
        vector<T, 2> wx() const requires (N >= 4 && N <= 4) { return vector<T, 2>(values[3], values[0]); }
        vector<T, 2> wy() const requires (N >= 4 && N <= 4) { return vector<T, 2>(values[3], values[1]); }
        vector<T, 2> wz() const requires (N >= 4 && N <= 4) { return vector<T, 2>(values[3], values[2]); }
        vector<T, 2> ww() const requires (N >= 4 && N <= 4) { return vector<T, 2>(values[3], values[3]); }

        vector<T, 2> rr() const requires (N >= 1 && N <= 4) { return vector<T, 2>(values[0], values[0]); }
        vector<T, 2> rg() const requires (N >= 2 && N <= 4) { return vector<T, 2>(values[0], values[1]); }
        vector<T, 2> rb() const requires (N >= 3 && N <= 4) { return vector<T, 2>(values[0], values[2]); }
        vector<T, 2> ra() const requires (N >= 4 && N <= 4) { return vector<T, 2>(values[0], values[3]); }
        vector<T, 2> gr() const requires (N >= 2 && N <= 4) { return vector<T, 2>(values[1], values[0]); }
        vector<T, 2> gg() const requires (N >= 2 && N <= 4) { return vector<T, 2>(values[1], values[1]); }
        vector<T, 2> gb() const requires (N >= 3 && N <= 4) { return vector<T, 2>(values[1], values[2]); }
        vector<T, 2> ga() const requires (N >= 4 && N <= 4) { return vector<T, 2>(values[1], values[3]); }
        vector<T, 2> br() const requires (N >= 3 && N <= 4) { return vector<T, 2>(values[2], values[0]); }
        vector<T, 2> bg() const requires (N >= 3 && N <= 4) { return vector<T, 2>(values[2], values[1]); }
        vector<T, 2> bb() const requires (N >= 3 && N <= 4) { return vector<T, 2>(values[2], values[2]); }
        vector<T, 2> ba() const requires (N >= 4 && N <= 4) { return vector<T, 2>(values[2], values[3]); }
        vector<T, 2> ar() const requires (N >= 4 && N <= 4) { return vector<T, 2>(values[3], values[0]); }
        vector<T, 2> ag() const requires (N >= 4 && N <= 4) { return vector<T, 2>(values[3], values[1]); }
        vector<T, 2> ab() const requires (N >= 4 && N <= 4) { return vector<T, 2>(values[3], values[2]); }
        vector<T, 2> aa() const requires (N >= 4 && N <= 4) { return vector<T, 2>(values[3], values[3]); }

        vector<T, 3> xxx() const requires (N >= 1 && N <= 4) { return vector<T, 3>(values[0], values[0], values[0]); }
        vector<T, 3> xxy() const requires (N >= 2 && N <= 4) { return vector<T, 3>(values[0], values[0], values[1]); }
        vector<T, 3> xxz() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[0], values[0], values[2]); }
        vector<T, 3> xxw() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[0], values[0], values[3]); }
        vector<T, 3> xyx() const requires (N >= 2 && N <= 4) { return vector<T, 3>(values[0], values[1], values[0]); }
        vector<T, 3> xyy() const requires (N >= 2 && N <= 4) { return vector<T, 3>(values[0], values[1], values[1]); }
        vector<T, 3> xyz() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[0], values[1], values[2]); }
        vector<T, 3> xyw() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[0], values[1], values[3]); }
        vector<T, 3> xzx() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[0], values[2], values[0]); }
        vector<T, 3> xzy() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[0], values[2], values[1]); }
        vector<T, 3> xzz() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[0], values[2], values[2]); }
        vector<T, 3> xzw() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[0], values[2], values[3]); }
        vector<T, 3> xwx() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[0], values[3], values[0]); }
        vector<T, 3> xwy() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[0], values[3], values[1]); }
        vector<T, 3> xwz() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[0], values[3], values[2]); }
        vector<T, 3> xww() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[0], values[3], values[3]); }
        vector<T, 3> yxx() const requires (N >= 2 && N <= 4) { return vector<T, 3>(values[1], values[0], values[0]); }
        vector<T, 3> yxy() const requires (N >= 2 && N <= 4) { return vector<T, 3>(values[1], values[0], values[1]); }
        vector<T, 3> yxz() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[1], values[0], values[2]); }
        vector<T, 3> yxw() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[1], values[0], values[3]); }
        vector<T, 3> yyx() const requires (N >= 2 && N <= 4) { return vector<T, 3>(values[1], values[1], values[0]); }
        vector<T, 3> yyy() const requires (N >= 2 && N <= 4) { return vector<T, 3>(values[1], values[1], values[1]); }
        vector<T, 3> yyz() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[1], values[1], values[2]); }
        vector<T, 3> yyw() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[1], values[1], values[3]); }
        vector<T, 3> yzx() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[1], values[2], values[0]); }
        vector<T, 3> yzy() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[1], values[2], values[1]); }
        vector<T, 3> yzz() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[1], values[2], values[2]); }
        vector<T, 3> yzw() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[1], values[2], values[3]); }
        vector<T, 3> ywx() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[1], values[3], values[0]); }
        vector<T, 3> ywy() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[1], values[3], values[1]); }
        vector<T, 3> ywz() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[1], values[3], values[2]); }
        vector<T, 3> yww() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[1], values[3], values[3]); }
        vector<T, 3> zxx() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[2], values[0], values[0]); }
        vector<T, 3> zxy() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[2], values[0], values[1]); }
        vector<T, 3> zxz() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[2], values[0], values[2]); }
        vector<T, 3> zxw() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[2], values[0], values[3]); }
        vector<T, 3> zyx() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[2], values[1], values[0]); }
        vector<T, 3> zyy() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[2], values[1], values[1]); }
        vector<T, 3> zyz() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[2], values[1], values[2]); }
        vector<T, 3> zyw() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[2], values[1], values[3]); }
        vector<T, 3> zzx() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[2], values[2], values[0]); }
        vector<T, 3> zzy() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[2], values[2], values[1]); }
        vector<T, 3> zzz() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[2], values[2], values[2]); }
        vector<T, 3> zzw() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[2], values[2], values[3]); }
        vector<T, 3> zwx() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[2], values[3], values[0]); }
        vector<T, 3> zwy() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[2], values[3], values[1]); }
        vector<T, 3> zwz() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[2], values[3], values[2]); }
        vector<T, 3> zww() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[2], values[3], values[3]); }
        vector<T, 3> wxx() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[3], values[0], values[0]); }
        vector<T, 3> wxy() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[3], values[0], values[1]); }
        vector<T, 3> wxz() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[3], values[0], values[2]); }
        vector<T, 3> wxw() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[3], values[0], values[3]); }
        vector<T, 3> wyx() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[3], values[1], values[0]); }
        vector<T, 3> wyy() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[3], values[1], values[1]); }
        vector<T, 3> wyz() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[3], values[1], values[2]); }
        vector<T, 3> wyw() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[3], values[1], values[3]); }
        vector<T, 3> wzx() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[3], values[2], values[0]); }
        vector<T, 3> wzy() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[3], values[2], values[1]); }
        vector<T, 3> wzz() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[3], values[2], values[2]); }
        vector<T, 3> wzw() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[3], values[2], values[3]); }
        vector<T, 3> wwx() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[3], values[3], values[0]); }
        vector<T, 3> wwy() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[3], values[3], values[1]); }
        vector<T, 3> wwz() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[3], values[3], values[2]); }
        vector<T, 3> www() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[3], values[3], values[3]); }

        vector<T, 3> rrr() const requires (N >= 1 && N <= 4) { return vector<T, 3>(values[0], values[0], values[0]); }
        vector<T, 3> rrg() const requires (N >= 2 && N <= 4) { return vector<T, 3>(values[0], values[0], values[1]); }
        vector<T, 3> rrb() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[0], values[0], values[2]); }
        vector<T, 3> rra() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[0], values[0], values[3]); }
        vector<T, 3> rgr() const requires (N >= 2 && N <= 4) { return vector<T, 3>(values[0], values[1], values[0]); }
        vector<T, 3> rgg() const requires (N >= 2 && N <= 4) { return vector<T, 3>(values[0], values[1], values[1]); }
        vector<T, 3> rgb() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[0], values[1], values[2]); }
        vector<T, 3> rga() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[0], values[1], values[3]); }
        vector<T, 3> rbr() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[0], values[2], values[0]); }
        vector<T, 3> rbg() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[0], values[2], values[1]); }
        vector<T, 3> rbb() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[0], values[2], values[2]); }
        vector<T, 3> rba() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[0], values[2], values[3]); }
        vector<T, 3> rar() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[0], values[3], values[0]); }
        vector<T, 3> rag() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[0], values[3], values[1]); }
        vector<T, 3> rab() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[0], values[3], values[2]); }
        vector<T, 3> raa() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[0], values[3], values[3]); }
        vector<T, 3> grr() const requires (N >= 2 && N <= 4) { return vector<T, 3>(values[1], values[0], values[0]); }
        vector<T, 3> grg() const requires (N >= 2 && N <= 4) { return vector<T, 3>(values[1], values[0], values[1]); }
        vector<T, 3> grb() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[1], values[0], values[2]); }
        vector<T, 3> gra() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[1], values[0], values[3]); }
        vector<T, 3> ggr() const requires (N >= 2 && N <= 4) { return vector<T, 3>(values[1], values[1], values[0]); }
        vector<T, 3> ggg() const requires (N >= 2 && N <= 4) { return vector<T, 3>(values[1], values[1], values[1]); }
        vector<T, 3> ggb() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[1], values[1], values[2]); }
        vector<T, 3> gga() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[1], values[1], values[3]); }
        vector<T, 3> gbr() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[1], values[2], values[0]); }
        vector<T, 3> gbg() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[1], values[2], values[1]); }
        vector<T, 3> gbb() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[1], values[2], values[2]); }
        vector<T, 3> gba() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[1], values[2], values[3]); }
        vector<T, 3> gar() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[1], values[3], values[0]); }
        vector<T, 3> gag() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[1], values[3], values[1]); }
        vector<T, 3> gab() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[1], values[3], values[2]); }
        vector<T, 3> gaa() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[1], values[3], values[3]); }
        vector<T, 3> brr() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[2], values[0], values[0]); }
        vector<T, 3> brg() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[2], values[0], values[1]); }
        vector<T, 3> brb() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[2], values[0], values[2]); }
        vector<T, 3> bra() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[2], values[0], values[3]); }
        vector<T, 3> bgr() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[2], values[1], values[0]); }
        vector<T, 3> bgg() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[2], values[1], values[1]); }
        vector<T, 3> bgb() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[2], values[1], values[2]); }
        vector<T, 3> bga() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[2], values[1], values[3]); }
        vector<T, 3> bbr() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[2], values[2], values[0]); }
        vector<T, 3> bbg() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[2], values[2], values[1]); }
        vector<T, 3> bbb() const requires (N >= 3 && N <= 4) { return vector<T, 3>(values[2], values[2], values[2]); }
        vector<T, 3> bba() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[2], values[2], values[3]); }
        vector<T, 3> bar() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[2], values[3], values[0]); }
        vector<T, 3> bag() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[2], values[3], values[1]); }
        vector<T, 3> bab() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[2], values[3], values[2]); }
        vector<T, 3> baa() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[2], values[3], values[3]); }
        vector<T, 3> arr() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[3], values[0], values[0]); }
        vector<T, 3> arg() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[3], values[0], values[1]); }
        vector<T, 3> arb() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[3], values[0], values[2]); }
        vector<T, 3> ara() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[3], values[0], values[3]); }
        vector<T, 3> agr() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[3], values[1], values[0]); }
        vector<T, 3> agg() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[3], values[1], values[1]); }
        vector<T, 3> agb() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[3], values[1], values[2]); }
        vector<T, 3> aga() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[3], values[1], values[3]); }
        vector<T, 3> abr() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[3], values[2], values[0]); }
        vector<T, 3> abg() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[3], values[2], values[1]); }
        vector<T, 3> abb() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[3], values[2], values[2]); }
        vector<T, 3> aba() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[3], values[2], values[3]); }
        vector<T, 3> aar() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[3], values[3], values[0]); }
        vector<T, 3> aag() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[3], values[3], values[1]); }
        vector<T, 3> aab() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[3], values[3], values[2]); }
        vector<T, 3> aaa() const requires (N >= 4 && N <= 4) { return vector<T, 3>(values[3], values[3], values[3]); }

        vector<T, 4> xxxx() const requires (N >= 1 && N <= 4) { return vector<T, 4>(values[0], values[0], values[0], values[0]); }
        vector<T, 4> xxxy() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[0], values[0], values[0], values[1]); }
        vector<T, 4> xxxz() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[0], values[0], values[2]); }
        vector<T, 4> xxxw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[0], values[0], values[3]); }
        vector<T, 4> xxyx() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[0], values[0], values[1], values[0]); }
        vector<T, 4> xxyy() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[0], values[0], values[1], values[1]); }
        vector<T, 4> xxyz() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[0], values[0], values[1], values[2]); }
        vector<T, 4> xxyw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[0], values[1], values[3]); }
        vector<T, 4> xxzx() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[0], values[2], values[0]); }
        vector<T, 4> xxzy() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[0], values[2], values[1]); }
        vector<T, 4> xxzz() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[0], values[2], values[2]); }
        vector<T, 4> xxzw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[0], values[2], values[3]); }
        vector<T, 4> xxwx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[0], values[3], values[0]); }
        vector<T, 4> xxwy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[0], values[3], values[1]); }
        vector<T, 4> xxwz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[0], values[3], values[2]); }
        vector<T, 4> xxww() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[0], values[3], values[3]); }
        vector<T, 4> xyxx() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[0], values[1], values[0], values[0]); }
        vector<T, 4> xyxy() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[0], values[1], values[0], values[1]); }
        vector<T, 4> xyxz() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[1], values[0], values[2]); }
        vector<T, 4> xyxw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[1], values[0], values[3]); }
        vector<T, 4> xyyx() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[0], values[1], values[1], values[0]); }
        vector<T, 4> xyyy() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[0], values[1], values[1], values[1]); }
        vector<T, 4> xyyz() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[1], values[1], values[2]); }
        vector<T, 4> xyyw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[1], values[1], values[3]); }
        vector<T, 4> xyzx() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[1], values[2], values[0]); }
        vector<T, 4> xyzy() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[1], values[2], values[1]); }
        vector<T, 4> xyzz() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[1], values[2], values[2]); }
        vector<T, 4> xyzw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[1], values[2], values[3]); }
        vector<T, 4> xywx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[1], values[3], values[0]); }
        vector<T, 4> xywy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[1], values[3], values[1]); }
        vector<T, 4> xywz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[1], values[3], values[2]); }
        vector<T, 4> xyww() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[1], values[3], values[3]); }
        vector<T, 4> xzxx() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[2], values[0], values[0]); }
        vector<T, 4> xzxy() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[2], values[0], values[1]); }
        vector<T, 4> xzxz() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[2], values[0], values[2]); }
        vector<T, 4> xzxw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[2], values[0], values[3]); }
        vector<T, 4> xzyx() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[2], values[1], values[0]); }
        vector<T, 4> xzyy() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[2], values[1], values[1]); }
        vector<T, 4> xzyz() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[2], values[1], values[2]); }
        vector<T, 4> xzyw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[2], values[1], values[3]); }
        vector<T, 4> xzzx() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[2], values[2], values[0]); }
        vector<T, 4> xzzy() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[2], values[2], values[1]); }
        vector<T, 4> xzzz() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[2], values[2], values[2]); }
        vector<T, 4> xzzw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[2], values[2], values[3]); }
        vector<T, 4> xzwx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[2], values[3], values[0]); }
        vector<T, 4> xzwy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[2], values[3], values[1]); }
        vector<T, 4> xzwz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[2], values[3], values[2]); }
        vector<T, 4> xzww() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[2], values[3], values[3]); }
        vector<T, 4> xwxx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[3], values[0], values[0]); }
        vector<T, 4> xwxy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[3], values[0], values[1]); }
        vector<T, 4> xwxz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[3], values[0], values[2]); }
        vector<T, 4> xwxw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[3], values[0], values[3]); }
        vector<T, 4> xwyx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[3], values[1], values[0]); }
        vector<T, 4> xwyy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[3], values[1], values[1]); }
        vector<T, 4> xwyz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[3], values[1], values[2]); }
        vector<T, 4> xwyw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[3], values[1], values[3]); }
        vector<T, 4> xwzx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[3], values[2], values[0]); }
        vector<T, 4> xwzy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[3], values[2], values[1]); }
        vector<T, 4> xwzz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[3], values[2], values[2]); }
        vector<T, 4> xwzw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[3], values[2], values[3]); }
        vector<T, 4> xwwx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[3], values[3], values[0]); }
        vector<T, 4> xwwy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[3], values[3], values[1]); }
        vector<T, 4> xwwz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[3], values[3], values[2]); }
        vector<T, 4> xwww() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[3], values[3], values[3]); }
        vector<T, 4> yxxx() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[1], values[0], values[0], values[0]); }
        vector<T, 4> yxxy() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[1], values[0], values[0], values[1]); }
        vector<T, 4> yxxz() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[0], values[0], values[2]); }
        vector<T, 4> yxxw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[0], values[0], values[3]); }
        vector<T, 4> yxyx() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[1], values[0], values[1], values[0]); }
        vector<T, 4> yxyy() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[1], values[0], values[1], values[1]); }
        vector<T, 4> yxyz() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[0], values[1], values[2]); }
        vector<T, 4> yxyw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[0], values[1], values[3]); }
        vector<T, 4> yxzx() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[1], values[0], values[2], values[0]); }
        vector<T, 4> yxzy() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[1], values[0], values[2], values[1]); }
        vector<T, 4> yxzz() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[0], values[2], values[2]); }
        vector<T, 4> yxzw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[0], values[2], values[3]); }
        vector<T, 4> yxwx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[0], values[3], values[0]); }
        vector<T, 4> yxwy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[0], values[3], values[1]); }
        vector<T, 4> yxwz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[0], values[3], values[2]); }
        vector<T, 4> yxww() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[0], values[3], values[3]); }
        vector<T, 4> yyxx() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[1], values[1], values[0], values[0]); }
        vector<T, 4> yyxy() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[1], values[1], values[0], values[1]); }
        vector<T, 4> yyxz() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[1], values[0], values[2]); }
        vector<T, 4> yyxw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[1], values[0], values[3]); }
        vector<T, 4> yyyx() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[1], values[1], values[1], values[0]); }
        vector<T, 4> yyyy() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[1], values[1], values[1], values[1]); }
        vector<T, 4> yyyz() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[1], values[1], values[2]); }
        vector<T, 4> yyyw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[1], values[1], values[3]); }
        vector<T, 4> yyzx() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[1], values[2], values[0]); }
        vector<T, 4> yyzy() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[1], values[2], values[1]); }
        vector<T, 4> yyzz() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[1], values[2], values[2]); }
        vector<T, 4> yyzw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[1], values[2], values[3]); }
        vector<T, 4> yywx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[1], values[3], values[0]); }
        vector<T, 4> yywy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[1], values[3], values[1]); }
        vector<T, 4> yywz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[1], values[3], values[2]); }
        vector<T, 4> yyww() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[1], values[3], values[3]); }
        vector<T, 4> yzxx() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[2], values[0], values[0]); }
        vector<T, 4> yzxy() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[2], values[0], values[1]); }
        vector<T, 4> yzxz() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[2], values[0], values[2]); }
        vector<T, 4> yzxw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[2], values[0], values[3]); }
        vector<T, 4> yzyx() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[2], values[1], values[0]); }
        vector<T, 4> yzyy() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[2], values[1], values[1]); }
        vector<T, 4> yzyz() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[2], values[1], values[2]); }
        vector<T, 4> yzyw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[2], values[1], values[3]); }
        vector<T, 4> yzzx() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[2], values[2], values[0]); }
        vector<T, 4> yzzy() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[2], values[2], values[1]); }
        vector<T, 4> yzzz() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[2], values[2], values[2]); }
        vector<T, 4> yzzw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[2], values[2], values[3]); }
        vector<T, 4> yzwx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[2], values[3], values[0]); }
        vector<T, 4> yzwy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[2], values[3], values[1]); }
        vector<T, 4> yzwz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[2], values[3], values[2]); }
        vector<T, 4> yzww() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[2], values[3], values[3]); }
        vector<T, 4> ywxx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[3], values[0], values[0]); }
        vector<T, 4> ywxy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[3], values[0], values[1]); }
        vector<T, 4> ywxz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[3], values[0], values[2]); }
        vector<T, 4> ywxw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[3], values[0], values[3]); }
        vector<T, 4> ywyx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[3], values[1], values[0]); }
        vector<T, 4> ywyy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[3], values[1], values[1]); }
        vector<T, 4> ywyz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[3], values[1], values[2]); }
        vector<T, 4> ywyw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[3], values[1], values[3]); }
        vector<T, 4> ywzx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[3], values[2], values[0]); }
        vector<T, 4> ywzy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[3], values[2], values[1]); }
        vector<T, 4> ywzz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[3], values[2], values[2]); }
        vector<T, 4> ywzw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[3], values[2], values[3]); }
        vector<T, 4> ywwx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[3], values[3], values[0]); }
        vector<T, 4> ywwy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[3], values[3], values[1]); }
        vector<T, 4> ywwz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[3], values[3], values[2]); }
        vector<T, 4> ywww() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[3], values[3], values[3]); }
        vector<T, 4> zxxx() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[0], values[0], values[0]); }
        vector<T, 4> zxxy() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[0], values[0], values[1]); }
        vector<T, 4> zxxz() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[0], values[0], values[2]); }
        vector<T, 4> zxxw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[0], values[0], values[3]); }
        vector<T, 4> zxyx() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[0], values[1], values[0]); }
        vector<T, 4> zxyy() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[0], values[1], values[1]); }
        vector<T, 4> zxyz() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[0], values[1], values[2]); }
        vector<T, 4> zxyw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[0], values[1], values[3]); }
        vector<T, 4> zxzx() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[0], values[2], values[0]); }
        vector<T, 4> zxzy() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[0], values[2], values[1]); }
        vector<T, 4> zxzz() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[0], values[2], values[2]); }
        vector<T, 4> zxzw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[0], values[2], values[3]); }
        vector<T, 4> zxwx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[0], values[3], values[0]); }
        vector<T, 4> zxwy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[0], values[3], values[1]); }
        vector<T, 4> zxwz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[0], values[3], values[2]); }
        vector<T, 4> zxww() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[0], values[3], values[3]); }
        vector<T, 4> zyxx() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[1], values[0], values[0]); }
        vector<T, 4> zyxy() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[1], values[0], values[1]); }
        vector<T, 4> zyxz() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[1], values[0], values[2]); }
        vector<T, 4> zyxw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[1], values[0], values[3]); }
        vector<T, 4> zyyx() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[1], values[1], values[0]); }
        vector<T, 4> zyyy() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[1], values[1], values[1]); }
        vector<T, 4> zyyz() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[1], values[1], values[2]); }
        vector<T, 4> zyyw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[1], values[1], values[3]); }
        vector<T, 4> zyzx() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[1], values[2], values[0]); }
        vector<T, 4> zyzy() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[1], values[2], values[1]); }
        vector<T, 4> zyzz() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[1], values[2], values[2]); }
        vector<T, 4> zyzw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[1], values[2], values[3]); }
        vector<T, 4> zywx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[1], values[3], values[0]); }
        vector<T, 4> zywy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[1], values[3], values[1]); }
        vector<T, 4> zywz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[1], values[3], values[2]); }
        vector<T, 4> zyww() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[1], values[3], values[3]); }
        vector<T, 4> zzxx() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[2], values[0], values[0]); }
        vector<T, 4> zzxy() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[2], values[0], values[1]); }
        vector<T, 4> zzxz() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[2], values[0], values[2]); }
        vector<T, 4> zzxw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[2], values[0], values[3]); }
        vector<T, 4> zzyx() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[2], values[1], values[0]); }
        vector<T, 4> zzyy() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[2], values[1], values[1]); }
        vector<T, 4> zzyz() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[2], values[1], values[2]); }
        vector<T, 4> zzyw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[2], values[1], values[3]); }
        vector<T, 4> zzzx() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[2], values[2], values[0]); }
        vector<T, 4> zzzy() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[2], values[2], values[1]); }
        vector<T, 4> zzzz() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[2], values[2], values[2]); }
        vector<T, 4> zzzw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[2], values[2], values[3]); }
        vector<T, 4> zzwx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[2], values[3], values[0]); }
        vector<T, 4> zzwy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[2], values[3], values[1]); }
        vector<T, 4> zzwz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[2], values[3], values[2]); }
        vector<T, 4> zzww() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[2], values[3], values[3]); }
        vector<T, 4> zwxx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[3], values[0], values[0]); }
        vector<T, 4> zwxy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[3], values[0], values[1]); }
        vector<T, 4> zwxz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[3], values[0], values[2]); }
        vector<T, 4> zwxw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[3], values[0], values[3]); }
        vector<T, 4> zwyx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[3], values[1], values[0]); }
        vector<T, 4> zwyy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[3], values[1], values[1]); }
        vector<T, 4> zwyz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[3], values[1], values[2]); }
        vector<T, 4> zwyw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[3], values[1], values[3]); }
        vector<T, 4> zwzx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[3], values[2], values[0]); }
        vector<T, 4> zwzy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[3], values[2], values[1]); }
        vector<T, 4> zwzz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[3], values[2], values[2]); }
        vector<T, 4> zwzw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[3], values[2], values[3]); }
        vector<T, 4> zwwx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[3], values[3], values[0]); }
        vector<T, 4> zwwy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[3], values[3], values[1]); }
        vector<T, 4> zwwz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[3], values[3], values[2]); }
        vector<T, 4> zwww() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[3], values[3], values[3]); }
        vector<T, 4> wxxx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[0], values[0], values[0]); }
        vector<T, 4> wxxy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[0], values[0], values[1]); }
        vector<T, 4> wxxz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[0], values[0], values[2]); }
        vector<T, 4> wxxw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[0], values[0], values[3]); }
        vector<T, 4> wxyx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[0], values[1], values[0]); }
        vector<T, 4> wxyy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[0], values[1], values[1]); }
        vector<T, 4> wxyz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[0], values[1], values[2]); }
        vector<T, 4> wxyw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[0], values[1], values[3]); }
        vector<T, 4> wxzx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[0], values[2], values[0]); }
        vector<T, 4> wxzy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[0], values[2], values[1]); }
        vector<T, 4> wxzz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[0], values[2], values[2]); }
        vector<T, 4> wxzw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[0], values[2], values[3]); }
        vector<T, 4> wxwx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[0], values[3], values[0]); }
        vector<T, 4> wxwy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[0], values[3], values[1]); }
        vector<T, 4> wxwz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[0], values[3], values[2]); }
        vector<T, 4> wxww() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[0], values[3], values[3]); }
        vector<T, 4> wyxx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[1], values[0], values[0]); }
        vector<T, 4> wyxy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[1], values[0], values[1]); }
        vector<T, 4> wyxz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[1], values[0], values[2]); }
        vector<T, 4> wyxw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[1], values[0], values[3]); }
        vector<T, 4> wyyx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[1], values[1], values[0]); }
        vector<T, 4> wyyy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[1], values[1], values[1]); }
        vector<T, 4> wyyz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[1], values[1], values[2]); }
        vector<T, 4> wyyw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[1], values[1], values[3]); }
        vector<T, 4> wyzx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[1], values[2], values[0]); }
        vector<T, 4> wyzy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[1], values[2], values[1]); }
        vector<T, 4> wyzz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[1], values[2], values[2]); }
        vector<T, 4> wyzw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[1], values[2], values[3]); }
        vector<T, 4> wywx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[1], values[3], values[0]); }
        vector<T, 4> wywy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[1], values[3], values[1]); }
        vector<T, 4> wywz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[1], values[3], values[2]); }
        vector<T, 4> wyww() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[1], values[3], values[3]); }
        vector<T, 4> wzxx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[2], values[0], values[0]); }
        vector<T, 4> wzxy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[2], values[0], values[1]); }
        vector<T, 4> wzxz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[2], values[0], values[2]); }
        vector<T, 4> wzxw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[2], values[0], values[3]); }
        vector<T, 4> wzyx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[2], values[1], values[0]); }
        vector<T, 4> wzyy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[2], values[1], values[1]); }
        vector<T, 4> wzyz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[2], values[1], values[2]); }
        vector<T, 4> wzyw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[2], values[1], values[3]); }
        vector<T, 4> wzzx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[2], values[2], values[0]); }
        vector<T, 4> wzzy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[2], values[2], values[1]); }
        vector<T, 4> wzzz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[2], values[2], values[2]); }
        vector<T, 4> wzzw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[2], values[2], values[3]); }
        vector<T, 4> wzwx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[2], values[3], values[0]); }
        vector<T, 4> wzwy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[2], values[3], values[1]); }
        vector<T, 4> wzwz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[2], values[3], values[2]); }
        vector<T, 4> wzww() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[2], values[3], values[3]); }
        vector<T, 4> wwxx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[3], values[0], values[0]); }
        vector<T, 4> wwxy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[3], values[0], values[1]); }
        vector<T, 4> wwxz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[3], values[0], values[2]); }
        vector<T, 4> wwxw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[3], values[0], values[3]); }
        vector<T, 4> wwyx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[3], values[1], values[0]); }
        vector<T, 4> wwyy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[3], values[1], values[1]); }
        vector<T, 4> wwyz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[3], values[1], values[2]); }
        vector<T, 4> wwyw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[3], values[1], values[3]); }
        vector<T, 4> wwzx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[3], values[2], values[0]); }
        vector<T, 4> wwzy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[3], values[2], values[1]); }
        vector<T, 4> wwzz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[3], values[2], values[2]); }
        vector<T, 4> wwzw() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[3], values[2], values[3]); }
        vector<T, 4> wwwx() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[3], values[3], values[0]); }
        vector<T, 4> wwwy() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[3], values[3], values[1]); }
        vector<T, 4> wwwz() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[3], values[3], values[2]); }
        vector<T, 4> wwww() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[3], values[3], values[3]); }

        vector<T, 4> rrrr() const requires (N >= 1 && N <= 4) { return vector<T, 4>(values[0], values[0], values[0], values[0]); }
        vector<T, 4> rrrg() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[0], values[0], values[0], values[1]); }
        vector<T, 4> rrrb() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[0], values[0], values[2]); }
        vector<T, 4> rrra() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[0], values[0], values[3]); }
        vector<T, 4> rrgr() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[0], values[0], values[1], values[0]); }
        vector<T, 4> rrgg() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[0], values[0], values[1], values[1]); }
        vector<T, 4> rrgb() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[0], values[0], values[1], values[2]); }
        vector<T, 4> rrga() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[0], values[1], values[3]); }
        vector<T, 4> rrbr() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[0], values[2], values[0]); }
        vector<T, 4> rrbg() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[0], values[2], values[1]); }
        vector<T, 4> rrbb() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[0], values[2], values[2]); }
        vector<T, 4> rrba() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[0], values[2], values[3]); }
        vector<T, 4> rrar() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[0], values[3], values[0]); }
        vector<T, 4> rrag() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[0], values[3], values[1]); }
        vector<T, 4> rrab() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[0], values[3], values[2]); }
        vector<T, 4> rraa() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[0], values[3], values[3]); }
        vector<T, 4> rgrr() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[0], values[1], values[0], values[0]); }
        vector<T, 4> rgrg() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[0], values[1], values[0], values[1]); }
        vector<T, 4> rgrb() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[1], values[0], values[2]); }
        vector<T, 4> rgra() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[1], values[0], values[3]); }
        vector<T, 4> rggr() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[0], values[1], values[1], values[0]); }
        vector<T, 4> rggg() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[0], values[1], values[1], values[1]); }
        vector<T, 4> rggb() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[1], values[1], values[2]); }
        vector<T, 4> rgga() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[1], values[1], values[3]); }
        vector<T, 4> rgbr() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[1], values[2], values[0]); }
        vector<T, 4> rgbg() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[1], values[2], values[1]); }
        vector<T, 4> rgbb() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[1], values[2], values[2]); }
        vector<T, 4> rgba() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[1], values[2], values[3]); }
        vector<T, 4> rgar() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[1], values[3], values[0]); }
        vector<T, 4> rgag() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[1], values[3], values[1]); }
        vector<T, 4> rgab() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[1], values[3], values[2]); }
        vector<T, 4> rgaa() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[1], values[3], values[3]); }
        vector<T, 4> rbrr() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[2], values[0], values[0]); }
        vector<T, 4> rbrg() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[2], values[0], values[1]); }
        vector<T, 4> rbrb() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[2], values[0], values[2]); }
        vector<T, 4> rbra() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[2], values[0], values[3]); }
        vector<T, 4> rbgr() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[2], values[1], values[0]); }
        vector<T, 4> rbgg() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[2], values[1], values[1]); }
        vector<T, 4> rbgb() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[2], values[1], values[2]); }
        vector<T, 4> rbga() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[2], values[1], values[3]); }
        vector<T, 4> rbbr() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[2], values[2], values[0]); }
        vector<T, 4> rbbg() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[2], values[2], values[1]); }
        vector<T, 4> rbbb() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[0], values[2], values[2], values[2]); }
        vector<T, 4> rbba() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[2], values[2], values[3]); }
        vector<T, 4> rbar() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[2], values[3], values[0]); }
        vector<T, 4> rbag() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[2], values[3], values[1]); }
        vector<T, 4> rbab() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[2], values[3], values[2]); }
        vector<T, 4> rbaa() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[2], values[3], values[3]); }
        vector<T, 4> rarr() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[3], values[0], values[0]); }
        vector<T, 4> rarg() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[3], values[0], values[1]); }
        vector<T, 4> rarb() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[3], values[0], values[2]); }
        vector<T, 4> rara() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[3], values[0], values[3]); }
        vector<T, 4> ragr() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[3], values[1], values[0]); }
        vector<T, 4> ragg() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[3], values[1], values[1]); }
        vector<T, 4> ragb() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[3], values[1], values[2]); }
        vector<T, 4> raga() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[3], values[1], values[3]); }
        vector<T, 4> rabr() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[3], values[2], values[0]); }
        vector<T, 4> rabg() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[3], values[2], values[1]); }
        vector<T, 4> rabb() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[3], values[2], values[2]); }
        vector<T, 4> raba() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[3], values[2], values[3]); }
        vector<T, 4> raar() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[3], values[3], values[0]); }
        vector<T, 4> raag() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[3], values[3], values[1]); }
        vector<T, 4> raab() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[3], values[3], values[2]); }
        vector<T, 4> raaa() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[0], values[3], values[3], values[3]); }
        vector<T, 4> grrr() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[1], values[0], values[0], values[0]); }
        vector<T, 4> grrg() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[1], values[0], values[0], values[1]); }
        vector<T, 4> grrb() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[0], values[0], values[2]); }
        vector<T, 4> grra() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[0], values[0], values[3]); }
        vector<T, 4> grgr() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[1], values[0], values[1], values[0]); }
        vector<T, 4> grgg() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[1], values[0], values[1], values[1]); }
        vector<T, 4> grgb() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[0], values[1], values[2]); }
        vector<T, 4> grga() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[0], values[1], values[3]); }
        vector<T, 4> grbr() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[1], values[0], values[2], values[0]); }
        vector<T, 4> grbg() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[1], values[0], values[2], values[1]); }
        vector<T, 4> grbb() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[0], values[2], values[2]); }
        vector<T, 4> grba() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[0], values[2], values[3]); }
        vector<T, 4> grar() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[0], values[3], values[0]); }
        vector<T, 4> grag() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[0], values[3], values[1]); }
        vector<T, 4> grab() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[0], values[3], values[2]); }
        vector<T, 4> graa() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[0], values[3], values[3]); }
        vector<T, 4> ggrr() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[1], values[1], values[0], values[0]); }
        vector<T, 4> ggrg() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[1], values[1], values[0], values[1]); }
        vector<T, 4> ggrb() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[1], values[0], values[2]); }
        vector<T, 4> ggra() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[1], values[0], values[3]); }
        vector<T, 4> gggr() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[1], values[1], values[1], values[0]); }
        vector<T, 4> gggg() const requires (N >= 2 && N <= 4) { return vector<T, 4>(values[1], values[1], values[1], values[1]); }
        vector<T, 4> gggb() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[1], values[1], values[2]); }
        vector<T, 4> ggga() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[1], values[1], values[3]); }
        vector<T, 4> ggbr() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[1], values[2], values[0]); }
        vector<T, 4> ggbg() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[1], values[2], values[1]); }
        vector<T, 4> ggbb() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[1], values[2], values[2]); }
        vector<T, 4> ggba() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[1], values[2], values[3]); }
        vector<T, 4> ggar() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[1], values[3], values[0]); }
        vector<T, 4> ggag() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[1], values[3], values[1]); }
        vector<T, 4> ggab() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[1], values[3], values[2]); }
        vector<T, 4> ggaa() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[1], values[3], values[3]); }
        vector<T, 4> gbrr() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[2], values[0], values[0]); }
        vector<T, 4> gbrg() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[2], values[0], values[1]); }
        vector<T, 4> gbrb() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[2], values[0], values[2]); }
        vector<T, 4> gbra() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[2], values[0], values[3]); }
        vector<T, 4> gbgr() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[2], values[1], values[0]); }
        vector<T, 4> gbgg() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[2], values[1], values[1]); }
        vector<T, 4> gbgb() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[2], values[1], values[2]); }
        vector<T, 4> gbga() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[2], values[1], values[3]); }
        vector<T, 4> gbbr() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[2], values[2], values[0]); }
        vector<T, 4> gbbg() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[2], values[2], values[1]); }
        vector<T, 4> gbbb() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[1], values[2], values[2], values[2]); }
        vector<T, 4> gbba() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[2], values[2], values[3]); }
        vector<T, 4> gbar() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[2], values[3], values[0]); }
        vector<T, 4> gbag() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[2], values[3], values[1]); }
        vector<T, 4> gbab() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[2], values[3], values[2]); }
        vector<T, 4> gbaa() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[2], values[3], values[3]); }
        vector<T, 4> garr() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[3], values[0], values[0]); }
        vector<T, 4> garg() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[3], values[0], values[1]); }
        vector<T, 4> garb() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[3], values[0], values[2]); }
        vector<T, 4> gara() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[3], values[0], values[3]); }
        vector<T, 4> gagr() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[3], values[1], values[0]); }
        vector<T, 4> gagg() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[3], values[1], values[1]); }
        vector<T, 4> gagb() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[3], values[1], values[2]); }
        vector<T, 4> gaga() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[3], values[1], values[3]); }
        vector<T, 4> gabr() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[3], values[2], values[0]); }
        vector<T, 4> gabg() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[3], values[2], values[1]); }
        vector<T, 4> gabb() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[3], values[2], values[2]); }
        vector<T, 4> gaba() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[3], values[2], values[3]); }
        vector<T, 4> gaar() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[3], values[3], values[0]); }
        vector<T, 4> gaag() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[3], values[3], values[1]); }
        vector<T, 4> gaab() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[3], values[3], values[2]); }
        vector<T, 4> gaaa() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[1], values[3], values[3], values[3]); }
        vector<T, 4> brrr() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[0], values[0], values[0]); }
        vector<T, 4> brrg() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[0], values[0], values[1]); }
        vector<T, 4> brrb() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[0], values[0], values[2]); }
        vector<T, 4> brra() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[0], values[0], values[3]); }
        vector<T, 4> brgr() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[0], values[1], values[0]); }
        vector<T, 4> brgg() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[0], values[1], values[1]); }
        vector<T, 4> brgb() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[0], values[1], values[2]); }
        vector<T, 4> brga() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[0], values[1], values[3]); }
        vector<T, 4> brbr() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[0], values[2], values[0]); }
        vector<T, 4> brbg() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[0], values[2], values[1]); }
        vector<T, 4> brbb() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[0], values[2], values[2]); }
        vector<T, 4> brba() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[0], values[2], values[3]); }
        vector<T, 4> brar() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[0], values[3], values[0]); }
        vector<T, 4> brag() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[0], values[3], values[1]); }
        vector<T, 4> brab() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[0], values[3], values[2]); }
        vector<T, 4> braa() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[0], values[3], values[3]); }
        vector<T, 4> bgrr() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[1], values[0], values[0]); }
        vector<T, 4> bgrg() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[1], values[0], values[1]); }
        vector<T, 4> bgrb() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[1], values[0], values[2]); }
        vector<T, 4> bgra() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[1], values[0], values[3]); }
        vector<T, 4> bggr() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[1], values[1], values[0]); }
        vector<T, 4> bggg() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[1], values[1], values[1]); }
        vector<T, 4> bggb() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[1], values[1], values[2]); }
        vector<T, 4> bgga() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[1], values[1], values[3]); }
        vector<T, 4> bgbr() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[1], values[2], values[0]); }
        vector<T, 4> bgbg() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[1], values[2], values[1]); }
        vector<T, 4> bgbb() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[1], values[2], values[2]); }
        vector<T, 4> bgba() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[1], values[2], values[3]); }
        vector<T, 4> bgar() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[1], values[3], values[0]); }
        vector<T, 4> bgag() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[1], values[3], values[1]); }
        vector<T, 4> bgab() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[1], values[3], values[2]); }
        vector<T, 4> bgaa() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[1], values[3], values[3]); }
        vector<T, 4> bbrr() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[2], values[0], values[0]); }
        vector<T, 4> bbrg() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[2], values[0], values[1]); }
        vector<T, 4> bbrb() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[2], values[0], values[2]); }
        vector<T, 4> bbra() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[2], values[0], values[3]); }
        vector<T, 4> bbgr() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[2], values[1], values[0]); }
        vector<T, 4> bbgg() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[2], values[1], values[1]); }
        vector<T, 4> bbgb() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[2], values[1], values[2]); }
        vector<T, 4> bbga() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[2], values[1], values[3]); }
        vector<T, 4> bbbr() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[2], values[2], values[0]); }
        vector<T, 4> bbbg() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[2], values[2], values[1]); }
        vector<T, 4> bbbb() const requires (N >= 3 && N <= 4) { return vector<T, 4>(values[2], values[2], values[2], values[2]); }
        vector<T, 4> bbba() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[2], values[2], values[3]); }
        vector<T, 4> bbar() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[2], values[3], values[0]); }
        vector<T, 4> bbag() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[2], values[3], values[1]); }
        vector<T, 4> bbab() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[2], values[3], values[2]); }
        vector<T, 4> bbaa() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[2], values[3], values[3]); }
        vector<T, 4> barr() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[3], values[0], values[0]); }
        vector<T, 4> barg() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[3], values[0], values[1]); }
        vector<T, 4> barb() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[3], values[0], values[2]); }
        vector<T, 4> bara() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[3], values[0], values[3]); }
        vector<T, 4> bagr() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[3], values[1], values[0]); }
        vector<T, 4> bagg() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[3], values[1], values[1]); }
        vector<T, 4> bagb() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[3], values[1], values[2]); }
        vector<T, 4> baga() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[3], values[1], values[3]); }
        vector<T, 4> babr() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[3], values[2], values[0]); }
        vector<T, 4> babg() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[3], values[2], values[1]); }
        vector<T, 4> babb() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[3], values[2], values[2]); }
        vector<T, 4> baba() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[3], values[2], values[3]); }
        vector<T, 4> baar() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[3], values[3], values[0]); }
        vector<T, 4> baag() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[3], values[3], values[1]); }
        vector<T, 4> baab() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[3], values[3], values[2]); }
        vector<T, 4> baaa() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[2], values[3], values[3], values[3]); }
        vector<T, 4> arrr() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[0], values[0], values[0]); }
        vector<T, 4> arrg() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[0], values[0], values[1]); }
        vector<T, 4> arrb() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[0], values[0], values[2]); }
        vector<T, 4> arra() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[0], values[0], values[3]); }
        vector<T, 4> argr() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[0], values[1], values[0]); }
        vector<T, 4> argg() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[0], values[1], values[1]); }
        vector<T, 4> argb() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[0], values[1], values[2]); }
        vector<T, 4> arga() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[0], values[1], values[3]); }
        vector<T, 4> arbr() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[0], values[2], values[0]); }
        vector<T, 4> arbg() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[0], values[2], values[1]); }
        vector<T, 4> arbb() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[0], values[2], values[2]); }
        vector<T, 4> arba() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[0], values[2], values[3]); }
        vector<T, 4> arar() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[0], values[3], values[0]); }
        vector<T, 4> arag() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[0], values[3], values[1]); }
        vector<T, 4> arab() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[0], values[3], values[2]); }
        vector<T, 4> araa() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[0], values[3], values[3]); }
        vector<T, 4> agrr() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[1], values[0], values[0]); }
        vector<T, 4> agrg() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[1], values[0], values[1]); }
        vector<T, 4> agrb() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[1], values[0], values[2]); }
        vector<T, 4> agra() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[1], values[0], values[3]); }
        vector<T, 4> aggr() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[1], values[1], values[0]); }
        vector<T, 4> aggg() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[1], values[1], values[1]); }
        vector<T, 4> aggb() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[1], values[1], values[2]); }
        vector<T, 4> agga() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[1], values[1], values[3]); }
        vector<T, 4> agbr() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[1], values[2], values[0]); }
        vector<T, 4> agbg() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[1], values[2], values[1]); }
        vector<T, 4> agbb() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[1], values[2], values[2]); }
        vector<T, 4> agba() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[1], values[2], values[3]); }
        vector<T, 4> agar() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[1], values[3], values[0]); }
        vector<T, 4> agag() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[1], values[3], values[1]); }
        vector<T, 4> agab() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[1], values[3], values[2]); }
        vector<T, 4> agaa() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[1], values[3], values[3]); }
        vector<T, 4> abrr() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[2], values[0], values[0]); }
        vector<T, 4> abrg() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[2], values[0], values[1]); }
        vector<T, 4> abrb() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[2], values[0], values[2]); }
        vector<T, 4> abra() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[2], values[0], values[3]); }
        vector<T, 4> abgr() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[2], values[1], values[0]); }
        vector<T, 4> abgg() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[2], values[1], values[1]); }
        vector<T, 4> abgb() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[2], values[1], values[2]); }
        vector<T, 4> abga() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[2], values[1], values[3]); }
        vector<T, 4> abbr() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[2], values[2], values[0]); }
        vector<T, 4> abbg() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[2], values[2], values[1]); }
        vector<T, 4> abbb() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[2], values[2], values[2]); }
        vector<T, 4> abba() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[2], values[2], values[3]); }
        vector<T, 4> abar() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[2], values[3], values[0]); }
        vector<T, 4> abag() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[2], values[3], values[1]); }
        vector<T, 4> abab() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[2], values[3], values[2]); }
        vector<T, 4> abaa() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[2], values[3], values[3]); }
        vector<T, 4> aarr() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[3], values[0], values[0]); }
        vector<T, 4> aarg() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[3], values[0], values[1]); }
        vector<T, 4> aarb() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[3], values[0], values[2]); }
        vector<T, 4> aara() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[3], values[0], values[3]); }
        vector<T, 4> aagr() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[3], values[1], values[0]); }
        vector<T, 4> aagg() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[3], values[1], values[1]); }
        vector<T, 4> aagb() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[3], values[1], values[2]); }
        vector<T, 4> aaga() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[3], values[1], values[3]); }
        vector<T, 4> aabr() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[3], values[2], values[0]); }
        vector<T, 4> aabg() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[3], values[2], values[1]); }
        vector<T, 4> aabb() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[3], values[2], values[2]); }
        vector<T, 4> aaba() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[3], values[2], values[3]); }
        vector<T, 4> aaar() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[3], values[3], values[0]); }
        vector<T, 4> aaag() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[3], values[3], values[1]); }
        vector<T, 4> aaab() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[3], values[3], values[2]); }
        vector<T, 4> aaaa() const requires (N >= 4 && N <= 4) { return vector<T, 4>(values[3], values[3], values[3], values[3]); }

        /* Operators */

        friend vector operator+(const vector& a, const vector& b) { vector r; for (int i = 0; i < N; i++) r[i] = a[i] + b[i]; return r; }
        friend vector operator-(const vector& a, const vector& b) { vector r; for (int i = 0; i < N; i++) r[i] = a[i] - b[i]; return r; }
        friend vector operator*(const vector& a, const vector& b) { vector r; for (int i = 0; i < N; i++) r[i] = a[i] * b[i]; return r; }
        friend vector operator/(const vector& a, const vector& b) { vector r; for (int i = 0; i < N; i++) r[i] = a[i] / b[i]; return r; }
        friend vector operator%(const vector& a, const vector& b) { vector r; for (int i = 0; i < N; i++) r[i] = a[i] % b[i]; return r; }

        friend vector operator+(const vector& a) { return a; }
        friend vector operator-(const vector& a) { vector r; for (int i = 0; i < N; i++) r[i] = -a[i]; return r; }

        friend vector operator*(T s, const vector& a) { vector r; for (int i = 0; i < N; i++) r[i] = s * a[i]; return r; }
        friend vector operator*(const vector& a, T s) { vector r; for (int i = 0; i < N; i++) r[i] = a[i] * s; return r; }
        friend vector operator/(T s, const vector& a) { vector r; for (int i = 0; i < N; i++) r[i] = s / a[i]; return r; }
        friend vector operator/(const vector& a, T s) { vector r; for (int i = 0; i < N; i++) r[i] = a[i] / s; return r; }
        friend vector operator%(T s, const vector& a) { vector r; for (int i = 0; i < N; i++) r[i] = s % a[i]; return r; }
        friend vector operator%(const vector& a, T s) { vector r; for (int i = 0; i < N; i++) r[i] = a[i] % s; return r; }

        friend bool operator==(const vector& a, const vector& b)
        {
            bool ret = true;
            for (int i = 0; i < N; i++) {
                if (!equal(a[i], b[i])) {
                    ret = false;
                    break;
                }
            }
            return ret;
        }

        friend bool operator!=(const vector& a, const vector& b)
        {
            bool ret = false;
            for (int i = 0; i < N; i++) {
                if (notEqual(a[i], b[i])) {
                    ret = true;
                    break;
                }
            }
            return ret;
        }

        /* Assignment operators */

        vector& operator+=(const vector& b) { for (int i = 0; i < N; i++) values[i] += b[i]; return *this; }
        vector& operator-=(const vector& b) { for (int i = 0; i < N; i++) values[i] -= b[i]; return *this; }
        vector& operator*=(const vector& b) { for (int i = 0; i < N; i++) values[i] *= b[i]; return *this; }
        vector& operator/=(const vector& b) { for (int i = 0; i < N; i++) values[i] /= b[i]; return *this; }
        vector& operator%=(const vector& b) { for (int i = 0; i < N; i++) values[i] %= b[i]; return *this; }

        /* Functions */

        friend vector sin        (const vector& v) { vector r; for (int i = 0; i < N; i++) r[i] = sin(v[i]);         return r; }
        friend vector cos        (const vector& v) { vector r; for (int i = 0; i < N; i++) r[i] = cos(v[i]);         return r; }
        friend vector tan        (const vector& v) { vector r; for (int i = 0; i < N; i++) r[i] = tan(v[i]);         return r; }
        friend vector asin       (const vector& v) { vector r; for (int i = 0; i < N; i++) r[i] = asin(v[i]);        return r; }
        friend vector acos       (const vector& v) { vector r; for (int i = 0; i < N; i++) r[i] = acos(v[i]);        return r; }
        friend vector atan       (const vector& v) { vector r; for (int i = 0; i < N; i++) r[i] = atan(v[i]);        return r; }
        friend vector radians    (const vector& v) { vector r; for (int i = 0; i < N; i++) r[i] = radians(v[i]);     return r; }
        friend vector degrees    (const vector& v) { vector r; for (int i = 0; i < N; i++) r[i] = degrees(v[i]);     return r; }
        friend vector exp        (const vector& v) { vector r; for (int i = 0; i < N; i++) r[i] = exp(v[i]);         return r; }
        friend vector exp2       (const vector& v) { vector r; for (int i = 0; i < N; i++) r[i] = exp2(v[i]);        return r; }
        friend vector log        (const vector& v) { vector r; for (int i = 0; i < N; i++) r[i] = log(v[i]);         return r; }
        friend vector log2       (const vector& v) { vector r; for (int i = 0; i < N; i++) r[i] = log2(v[i]);        return r; }
        friend vector log10      (const vector& v) { vector r; for (int i = 0; i < N; i++) r[i] = log10(v[i]);       return r; }
        friend vector sqr        (const vector& v) { vector r; for (int i = 0; i < N; i++) r[i] = sqr(v[i]);         return r; }
        friend vector sqrt       (const vector& v) { vector r; for (int i = 0; i < N; i++) r[i] = sqrt(v[i]);        return r; }
        friend vector inversesqrt(const vector& v) { vector r; for (int i = 0; i < N; i++) r[i] = inversesqrt(v[i]); return r; }
        friend vector cbrt       (const vector& v) { vector r; for (int i = 0; i < N; i++) r[i] = cbrt(v[i]);        return r; }
        friend vector sign       (const vector& v) { vector r; for (int i = 0; i < N; i++) r[i] = sign(v[i]);        return r; }
        friend vector abs        (const vector& v) { vector r; for (int i = 0; i < N; i++) r[i] = abs(v[i]);         return r; }
        friend vector floor      (const vector& v) { vector r; for (int i = 0; i < N; i++) r[i] = floor(v[i]);       return r; }
        friend vector ceil       (const vector& v) { vector r; for (int i = 0; i < N; i++) r[i] = ceil(v[i]);        return r; }
        friend vector round      (const vector& v) { vector r; for (int i = 0; i < N; i++) r[i] = round(v[i]);       return r; }
        friend vector fract      (const vector& v) { vector r; for (int i = 0; i < N; i++) r[i] = fract(v[i]);       return r; }

        friend vector pow(const vector& v, const T p) { vector r; for (int i = 0; i < N; i++) r[i] = pow(v[i], p); return r; }

        friend vector<bool, N> isfinite(const vector& v) { vector<bool, N> r; for (int i = 0; i < N; i++) r[i] = isfinite(v[i]); return r; }
        friend vector<bool, N> isinf   (const vector& v) { vector<bool, N> r; for (int i = 0; i < N; i++) r[i] = isinf(v[i]);    return r; }
        friend vector<bool, N> isnan   (const vector& v) { vector<bool, N> r; for (int i = 0; i < N; i++) r[i] = isnan(v[i]);    return r; }
        friend vector<bool, N> isnormal(const vector& v) { vector<bool, N> r; for (int i = 0; i < N; i++) r[i] = isnormal(v[i]); return r; }

        friend vector min(const vector& a, const vector& b)
        {
            vector r;
            for (int i = 0; i < N; i++)
                r[i] = min(a[i], b[i]);
            return r;
        }

        friend vector min(const vector& a, const T x)
        {
            vector r;
            for (int i = 0; i < N; i++)
                r[i] = min(a[i], x);
            return r;
        }

        friend vector max(const vector& a, const vector& b)
        {
            vector r;
            for (int i = 0; i < N; i++)
                r[i] = max(a[i], b[i]);
            return r;
        }

        friend vector max(const vector& a, const T x)
        {
            vector r;
            for (int i = 0; i < N; i++)
                r[i] = max(a[i], x);
            return r;
        }

        friend vector clamp(const vector& v, const T minval, const T maxval)
        {
            vector r;
            for (int i = 0; i < N; i++)
                r[i] = clamp(v[i], minval, maxval);
            return r;
        }

        friend vector clamp(const vector& v, const vector& minval, const vector& maxval)
        {
            vector r;
            for (int i = 0; i < N; i++)
                r[i] = clamp(v[i], minval[i], maxval[i]);
            return r;
        }

        friend vector mix(const vector& a, const vector& b, const T alpha)
        {
            vector r;
            for (int i = 0; i < N; i++)
                r[i] = mix(a[i], b[i], alpha);
            return r;
        }

        friend vector mix(const vector& a, const vector& b, const vector& alpha)
        {
            vector r;
            for (int i = 0; i < N; i++)
                r[i] = mix(a[i], b[i], alpha[i]);
            return r;
        }

        friend vector step(const vector& a, const T edge)
        {
            vector r;
            for (int i = 0; i < N; i++)
                r[i] = step(a[i], edge);
            return r;
        }

        friend vector step(const vector& a, const vector& edge)
        {
            vector r;
            for (int i = 0; i < N; i++)
                r[i] = step(a[i], edge[i]);
            return r;
        }

        friend vector smoothstep(const vector& a, const T edge0, const T edge1)
        {
            vector r;
            for (int i = 0; i < N; i++)
                r[i] = smoothstep(a[i], edge0, edge1);
            return r;
        }

        friend vector smoothstep(const vector & a, const vector& edge0, const vector& edge1)
        {
            vector r;
            for (int i = 0; i < N; i++)
                r[i] = smoothstep(a[i], edge0[i], edge1[i]);
            return r;
        }

        friend vector mod(const vector& a, const T y)
        {
            vector r;
            for (int i = 0; i < N; i++)
                r[i] = mod(a[i], y);
            return r;
        }

        friend vector mod(const vector& a, const vector& y)
        {
            vector r;
            for (int i = 0; i < N; i++)
                r[i] = mod(a[i], y[i]);
            return r;
        }

        /* Comparison functions */

        friend vector<bool, N> greaterThan     (const vector& a, const vector& b) { vector<bool, N> r; for (int i = 0; i < N; i++) r[i] = greaterThan(a[i], b[i]);      return r; }
        friend vector<bool, N> greaterThanEqual(const vector& a, const vector& b) { vector<bool, N> r; for (int i = 0; i < N; i++) r[i] = greaterThanEqual(a[i], b[i]); return r; }
        friend vector<bool, N> lessThan        (const vector& a, const vector& b) { vector<bool, N> r; for (int i = 0; i < N; i++) r[i] = lessThan(a[i], b[i]);         return r; }
        friend vector<bool, N> lessThanEqual   (const vector& a, const vector& b) { vector<bool, N> r; for (int i = 0; i < N; i++) r[i] = lessThanEqual(a[i], b[i]);    return r; }
        friend vector<bool, N> equal           (const vector& a, const vector& b) { vector<bool, N> r; for (int i = 0; i < N; i++) r[i] = equal(a[i], b[i]);            return r; }
        friend vector<bool, N> notEqual        (const vector& a, const vector& b) { vector<bool, N> r; for (int i = 0; i < N; i++) r[i] = notEqual(a[i], b[i]);         return r; }

        /* Geometric functions */

        friend T dot(const vector& a, const vector& b)
        {
            T d = T(0);
            for (int i = 0; i < N; i++)
                d += a[i] * b[i];
            return d;
        }

        friend T length(const vector& a)
        {
            return sqrt(dot(a, a));
        }

        friend T distance(const vector& a, const vector& b)
        {
            return length(a - b);
        }

        friend vector normalize(const vector& v)
        {
            return v / length(v);
        }

        /* Functions for N=3 */

        friend vector faceforward(const vector& v, const vector& I, const vector& Nref) requires (N == 3)
        {
            return dot(Nref, I) < T(0) ? v : -v;
        }

        friend vector reflect(const vector& i, const vector& n) requires (N == 3)
        {
            return i - T(2) * dot(n, i) * n;
        }

        friend vector refract(const vector& i, const vector& n, T eta) requires (N == 3)
        {
            const T d = dot(n, i);
            const T k = T(1) - eta * eta * (T(1) - d * d);
            return k <= T(0) ? vector(T(0)) : i * eta - n * (eta * d + sqrt(k));
        }

        friend vector cross(const vector& v, const vector& w) requires (N == 3)
        {
            return vector(
                    v[1] * w[2] - v[2] * w[1],
                    v[2] * w[0] - v[0] * w[2],
                    v[0] * w[1] - v[1] * w[0]);
        }

        /* Bonus functions. Note that log2 is already covered. */

        friend vector<bool, N> is_pow2(const vector& a)
        {
            vector<bool, N> r;
            for (int i = 0; i < N; i++)
                r[i] = is_pow2(a[i]);
            return r;
        }

        friend vector popcount(const vector& a)
        {
            vector r;
            for (int i = 0; i < N; i++)
                r[i] = popcount(a[i]);
            return r;
        }

        friend vector next_pow2(const vector& a)
        {
            vector r;
            for (int i = 0; i < N; i++)
                r[i] = next_pow2(a[i]);
            return r;
        }

        friend vector next_multiple(const vector& a, T y)
        {
            vector r;
            for (int i = 0; i < N; i++)
                r[i] = next_multiple(a[i], y);
            return r;
        }

        friend vector next_multiple(const vector& a, const vector& y)
        {
            vector r;
            for (int i = 0; i < N; i++)
                r[i] = next_multiple(a[i], y[i]);
            return r;
        }

        friend T min(const vector& a)
        {
            T r = a[0];
            for (int i = 1; i < N; i++)
                if (a[i] < r)
                    r = a[i];
            return r;
        }

        friend T max(const vector& a)
        {
            T r = a[0];
            for (int i = 1; i < N; i++)
                if (a[i] > r)
                    r = a[i];
            return r;
        }

        friend T average(const vector& a)
        {
            constexpr T inv_N = T(1) / T(N);
            T sum = 0;
            for (int i = 0; i < N; i++)
                sum += a[i];
            return inv_N * sum;
        }
    };

    template<int N> bool any(const vector<bool, N>& v)
    {
        bool ret = false;
        for (int i = 0; i < N; i++) {
            if (v[i]) {
                ret = true;
                break;
            }
        }
        return ret;
    }

    template<int N> bool all(const vector<bool, N>& v)
    {
        bool ret = true;
        for (int i = 0; i < N; i++) {
            if (!v[i]) {
                ret = false;
                break;
            }
        }
        return ret;
    }

    template<int N> vector<bool, N> negate(const vector<bool, N>& v)
    {
        vector<bool, N> r;
        for (int i = 0; i < N; i++)
            r[i] = !v[i];
        return r;
    }

    typedef vector<bool, 2> bvec2;
    typedef vector<bool, 3> bvec3;
    typedef vector<bool, 4> bvec4;
    typedef vector<int8_t, 2> ibvec2;
    typedef vector<int8_t, 3> ibvec3;
    typedef vector<int8_t, 4> ibvec4;
    typedef vector<uint8_t, 2> ubvec2;
    typedef vector<uint8_t, 3> ubvec3;
    typedef vector<uint8_t, 4> ubvec4;
    typedef vector<int16_t, 2> svec2;
    typedef vector<int16_t, 3> svec3;
    typedef vector<int16_t, 4> svec4;
    typedef vector<uint16_t, 2> usvec2;
    typedef vector<uint16_t, 3> usvec3;
    typedef vector<uint16_t, 4> usvec4;
    typedef vector<int32_t, 2> ivec2;
    typedef vector<int32_t, 3> ivec3;
    typedef vector<int32_t, 4> ivec4;
    typedef vector<uint32_t, 2> uvec2;
    typedef vector<uint32_t, 3> uvec3;
    typedef vector<uint32_t, 4> uvec4;
    typedef vector<int64_t, 2> i64vec2;
    typedef vector<int64_t, 3> i64vec3;
    typedef vector<int64_t, 4> i64vec4;
    typedef vector<uint64_t, 2> u64vec2;
    typedef vector<uint64_t, 3> u64vec3;
    typedef vector<uint64_t, 4> u64vec4;
    typedef vector<float, 2> vec2;
    typedef vector<float, 3> vec3;
    typedef vector<float, 4> vec4;
    typedef vector<double, 2> dvec2;
    typedef vector<double, 3> dvec3;
    typedef vector<double, 4> dvec4;


    /* Matrix */

    template<typename T, int C, int R> class matrix
    {
    public:
        T values[C * R];

        /* Constructors, Destructor */

        matrix() {}

        explicit matrix(const T* a)
        {
            for (int i = 0; i < C * R; i++)
                values[i] = a[i];
        }

        matrix(T x)
        {
            for (int i = 0; i < C; i++)
                for (int j = 0; j < R; j++)
                    values[i * R + j] = (i == j ? x : T(0));
        }

        matrix(T v0, T v1, T v2, T v3) requires (C * R == 4)
            : values { v0, v1, v2, v3 } {}

        matrix(T v0, T v1, T v2, T v3, T v4, T v5) requires (C * R == 6)
            : values { v0, v1, v2, v3, v4, v5 } {}

        matrix(T v0, T v1, T v2, T v3, T v4, T v5, T v6, T v7) requires (C * R == 8)
            : values { v0, v1, v2, v3, v4, v5, v6, v7 } {}

        matrix(T v0, T v1, T v2, T v3, T v4, T v5, T v6, T v7, T v8) requires (C * R == 9)
            : values { v0, v1, v2, v3, v4, v5, v6, v7, v8 } {}

        matrix(T v0, T v1, T v2, T v3, T v4, T v5, T v6, T v7, T v8, T v9, T v10, T v11) requires (C * R == 12)
            : values { v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11 } {}

        matrix(T v0, T v1, T v2, T v3, T v4, T v5, T v6, T v7, T v8, T v9, T v10, T v11, T v12, T v13, T v14, T v15) requires (C * R == 16)
            : values { v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15 } {}

        explicit matrix(const vector<T, R>& col0, const vector<T, R>& col1) requires (C == 2)
        {
            for (int i = 0; i < R; i++) values[0 * C + i] = col0[i];
            for (int i = 0; i < R; i++) values[1 * C + i] = col1[i];
        }

        explicit matrix(const vector<T, R>& col0, const vector<T, R>& col1, const vector<T, R>& col2) requires (C == 3)
        {
            for (int i = 0; i < R; i++) values[0 * C + i] = col0[i];
            for (int i = 0; i < R; i++) values[1 * C + i] = col1[i];
            for (int i = 0; i < R; i++) values[2 * C + i] = col2[i];
        }

        explicit matrix(const vector<T, R>& col0, const vector<T, R>& col1, const vector<T, R>& col2, const vector<T, R>& col3) requires (C == 4)
        {
            for (int i = 0; i < R; i++) values[0 * C + i] = col0[i];
            for (int i = 0; i < R; i++) values[1 * C + i] = col1[i];
            for (int i = 0; i < R; i++) values[2 * C + i] = col2[i];
            for (int i = 0; i < R; i++) values[3 * C + i] = col3[i];
        }

        template<int N>
        matrix(const matrix<T, N, N>& M) requires (C == R && C != N)
        {
            for (int j = 0; j < C; j++)
                for (int i = 0; i < R; i++)
                    values[j * R + i] = ((j >= N || i >= N) ? (j == i ? T(1) : T(0)) : M[j][i]);
        }

        /* Data access */

        operator const T*() const { return values; }

        float* operator[](std::ptrdiff_t j)
        {
            return values + j * R;
        }

        const float* operator[](std::ptrdiff_t j) const
        {
            return values + j * R;
        }

        friend vector<T, C> row(const matrix& m, int row)
        {
            vector<T, C> r;
            for (int i = 0; i < C; i++)
                r[i] = m[i][row];
            return r;
        }

        friend vector<T, R> col(const matrix& m, int col)
        {
            return vector<T, R>(m[col]);
        }

        /* Operators */

        friend matrix operator+(const matrix& a, const matrix& b) { matrix r; for (int i = 0; i < C * R; i++) r.values[i] = a.values[i] + b.values[i]; return r; }
        friend matrix operator-(const matrix& a, const matrix& b) { matrix r; for (int i = 0; i < C * R; i++) r.values[i] = a.values[i] - b.values[i]; return r; }

        friend matrix operator+(const matrix& a) { return a; }
        friend matrix operator-(const matrix& a) { matrix r; for (int i = 0; i < C * R; i++) r.values[i] = -a.values[i]; return r; }

        friend matrix operator*(T s, const matrix& a) { matrix r; for (int i = 0; i < C * R; i++) r.values[i] = s * a.values[i]; return r; }
        friend matrix operator*(const matrix& a, T s) { matrix r; for (int i = 0; i < C * R; i++) r.values[i] = a.values[i] * s; return r; }
        friend matrix operator/(const matrix& a, T s) { matrix r; for (int i = 0; i < C * R; i++) r.values[i] = a.values[i] / s; return r; }
        friend matrix operator%(const matrix& a, T s) { matrix r; for (int i = 0; i < C * R; i++) r.values[i] = a.values[i] % s; return r; }

        friend vector<T, R> operator*(const matrix& m, const vector<T, C>& w)
        {
            vector<T, R> r;
            for (int i = 0; i < R; i++) {
                r[i] = T(0);
                for (int j = 0; j < C; j++) {
                    r[i] += m[j][i] * w[j];
                }
            }
            return r;
        }

        friend vector<T, C> operator*(const vector<T, R>& w, const matrix& m)
        {
            vector<T, C> r;
            for (int i = 0; i < C; i++) {
                r[i] = T(0);
                for (int j = 0; j < R; j++) {
                    r[i] += m[i][j] * w[j];
                }
            }
            return r;
        }

        friend matrix<T, R, R> operator*(const matrix<T, C, R>& m, const matrix<T, R, C>& n)
        {
            matrix<T, R, R> r;
            for (int i = 0; i < R; i++) {
                for (int j = 0; j < R; j++) {
                    r[i][j] = T(0);
                    for (int k = 0; k < C; k++) {
                        r[i][j] += m[k][j] * n[i][k];
                    }
                }
            }
            return r;
        }

        friend bool operator==(const matrix& a, const matrix& b)
        {
            bool ret = true;
            for (int i = 0; i < C * R; i++) {
                if (!equal(a.values[i], b.values[i])) {
                    ret = false;
                    break;
                }
            }
            return ret;
        }

        friend bool operator!=(const matrix& a, const matrix& b)
        {
            bool ret = false;
            for (int i = 0; i < C * R; i++) {
                if (notEqual(a.values[i], b.values[i])) {
                    ret = true;
                    break;
                }
            }
            return ret;
        }

        /* Assignment operators */

        matrix& operator+=(const matrix& b) { for (int i = 0; i < C * R; i++) values[i] += b.values[i]; return *this; }
        matrix& operator-=(const matrix& b) { for (int i = 0; i < C * R; i++) values[i] -= b.values[i]; return *this; }
        matrix& operator*=(const matrix& b) { matrix r = *this * b; *this = r; return *this; }

        /* Functions */

        friend matrix<T, R, C> transpose(const matrix& m)
        {
            matrix<T, R, C> r;
            for (int j = 0; j < C; j++)
                for (int i = 0; i < R; i++)
                    r[j * R + i] = m[i][j];
            return r;
        }

        friend matrix outerProduct(const vector<T, R>& v, const vector<T, C>& w)
        {
            matrix m;
            for (int j = 0; j < R; j++)
                for (int i = 0; i < C; i++)
                    m.values[j * R + i] = v[j] * w[i];
            return m;
        }

        /* Functions for 4x4 matrices */

        void translate(const vector<T, 3>& t) requires (C == 4 && R == 4)
        {
            values[3 * R + 0] = dot(vector<T, 3>(values[0 * R + 0], values[1 * R + 0], values[2 * R + 0]), t) + values[3 * R + 0];
            values[3 * R + 1] = dot(vector<T, 3>(values[0 * R + 1], values[1 * R + 1], values[2 * R + 1]), t) + values[3 * R + 1];
            values[3 * R + 2] = dot(vector<T, 3>(values[0 * R + 2], values[1 * R + 2], values[2 * R + 2]), t) + values[3 * R + 2];
            values[3 * R + 3] = dot(vector<T, 3>(values[0 * R + 3], values[1 * R + 3], values[2 * R + 3]), t) + values[3 * R + 3];
        }

        friend matrix translate(const matrix& m, const vector<T, 3>& v) requires (C == 4 && R == 4)
        {
            matrix M = m;
            M.translate(v);
            return M;
        }

        void scale(const vector<T, 3>& v) requires (C == 4 && R == 4)
        {
            for (int j = 0; j < C - 1; j++)
                for (int i = 0; i < R; i++)
                    values[j * R + i] *= v[j];
        }

        friend matrix scale(const matrix& m, const vector<T, 3>& v) requires (C == 4 && R == 4)
        {
            matrix M = m;
            M.scale(v);
            return M;
        }

        void rotate(T angle, const vector<T, 3>& axis) requires (C == 4 && R == 4)
        {
            *this *= toMat4(angle, normalize(axis));
        }

        friend matrix rotate(const matrix& m, T angle, const vector<T, 3>& axis) requires (C == 4 && R == 4)
        {
            matrix M = m;
            M.rotate(angle, axis);
            return M;
        }
    };

    /* Determinants and inverses for 2x2, 3x3, 4x4 */

    template<typename T> T det(const matrix<T, 2, 2>& m)
    {
        return m[0][0] * m[1][1] - m[1][0] * m[0][1];
    }

    template<typename T> T det(const matrix<T, 3, 3>& m)
    {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
             + m[0][1] * (m[1][2] * m[2][0] - m[1][0] * m[2][2])
             + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }

    template<typename T> T det(const matrix<T, 4, 4>& m)
    {
        T d0 = m[1][1] * (m[2][2] * m[3][3] - m[3][2] * m[2][3])
             + m[2][1] * (m[3][2] * m[1][3] - m[1][2] * m[3][3])
             + m[3][1] * (m[1][2] * m[2][3] - m[2][2] * m[1][3]);
        T d1 = m[0][1] * (m[2][2] * m[3][3] - m[3][2] * m[2][3])
             + m[2][1] * (m[3][2] * m[0][3] - m[0][2] * m[3][3])
             + m[3][1] * (m[0][2] * m[2][3] - m[2][2] * m[0][3]);
        T d2 = m[0][1] * (m[1][2] * m[3][3] - m[3][2] * m[1][3])
             + m[1][1] * (m[3][2] * m[0][3] - m[0][2] * m[3][3])
             + m[3][1] * (m[0][2] * m[1][3] - m[1][2] * m[0][3]);
        T d3 = m[0][1] * (m[1][2] * m[2][3] - m[2][2] * m[1][3])
             + m[1][1] * (m[2][2] * m[0][3] - m[0][2] * m[2][3])
             + m[2][1] * (m[0][2] * m[1][3] - m[1][2] * m[0][3]);
        return m[0][0] * d0 - m[1][0] * d1 + m[2][0] * d2 - m[3][0] * d3;
    }

    template<typename T, int N> bool invertible(const matrix<T, N, N>& m, T eps = epsilon_v<T>) requires (N == 2 || N == 3 || N == 4)
    {
        T d = det(m);
        return (d > eps || d < -eps);
    }

    typedef matrix<float, 2, 2> mat2;
    typedef matrix<double, 2, 2> dmat2;
    typedef matrix<float, 3, 3> mat3;
    typedef matrix<double, 3, 3> dmat3;
    typedef matrix<float, 4, 4> mat4;
    typedef matrix<double, 4, 4> dmat4;


    /* Quaternion */

    template<typename T>
    class quaternion
    {
    public:
        T x, y, z, w;

        /* Constructors, Destructor */

        quaternion() {}
        quaternion(T x, T y, T z, T w) : x(x), y(y), z(z), w(w) {}
        template<typename U>
        explicit quaternion(const quaternion<U>& q) : quaternion(T(q.x), T(q.y), T(q.z), T(q.w)) {}

        constexpr static quaternion null() { return quaternion(T(0), T(0), T(0), T(1)); }

        /* Operators */

        quaternion operator+() const
        {
            return *this;
        }

        quaternion operator-() const
        {
            return conjugate(*this);
        }

        quaternion operator*(const quaternion& q) const
        {
            quaternion p;
            p.x = w * q.x + x * q.w + y * q.z - z * q.y;
            p.y = w * q.y + y * q.w + z * q.x - x * q.z;
            p.z = w * q.z + z * q.w + x * q.y - y * q.x;
            p.w = w * q.w - x * q.x - y * q.y - z * q.z;
            return p;
        }

        const quaternion& operator*=(const quaternion& q)
        {
            *this = *this * q;
            return *this;
        }

        friend bool operator==(const quaternion& a, const quaternion& b)
        {
            return (equal(a.x, b.x) && equal(a.y, b.y) && equal(a.z, b.z) && equal(a.w, b.w));
        }

        friend bool operator!=(const quaternion& a, const quaternion& b)
        {
            return (notEqual(a.x, b.x) || notEqual(a.y, b.y) || notEqual(a.z, b.z) || notEqual(a.w, b.w));
        }

        friend vector<T, 3> operator*(const quaternion& q, const vector<T, 3>& v)
        {
            // direct way: quaternion t = q * quaternion(v.x(), v.y(), v.z(), T(0)) * conjugate(q); return vector<T, 3>(t.x, t.y, t.z);
            // instead: see https://www.johndcook.com/blog/2021/06/16/faster-quaternion-rotations/
            vector<T, 3> s = vector<T, 3>(q.x, q.y, q.z);
            vector<T, 3> t = T(2) * cross(s, v);
            return v + q.w * t + cross(s, t);
        }

        friend vector<T, 3> operator*(const vector<T, 3>& v, const quaternion& q)
        {
            return conjugate(q) * v;
        }

        /* Functions */

        vector<T, 3> axis() const
        {
            T t = sqrt(x * x + y * y + z * z);
            if (t < epsilon_v<T>)
                return vector<T, 3>(T(0));
            else
                return vector<T, 3>(x / t, y / t, z / t);
        }

        T angle() const
        {
            T t = sqrt(x * x + y * y + z * z);
            return T(2) * atan(t, w);
        }

        friend T magnitude(const quaternion& q)
        {
            return sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        }

        friend quaternion normalize(const quaternion q)
        {
            T m = magnitude(q);
            return quaternion(q.x / m, q.y / m, q.z / m, q.w / m);
        }

        friend quaternion conjugate(const quaternion& q)
        {
            return quaternion(-q.x, -q.y, -q.z, q.w);
        }

        friend quaternion inverse(const quaternion& q)
        {
            return conjugate(q);
        }

        friend quaternion slerp(const quaternion& q, const quaternion& r, T alpha)
        {
            quaternion w = r;
            T cos_half_angle = q.x * r.x + q.y * r.y + q.z * r.z + q.w * r.w;
            if (cos_half_angle < T(0)) {
                // quat(x, y, z, w) and quat(-x, -y, -z, -w) represent the same rotation
                w.x = -w.x; w.y = -w.y; w.z = -w.z; w.w = -w.w;
                cos_half_angle = -cos_half_angle;
            }
            T tmp_q, tmp_w;
            if (cos_half_angle >= T(1)) {
                // angle is zero => rotations are identical
                tmp_q = T(1);
                tmp_w = T(0);
            } else {
                T half_angle = acos(cos_half_angle);
                T sin_half_angle = sqrt(T(1) - cos_half_angle * cos_half_angle);
                if (abs(sin_half_angle) < epsilon_v<T>){
                    // angle is 180 degrees => result is not clear
                    tmp_q = T(0.5);
                    tmp_w = T(0.5);
                } else {
                    tmp_q = sin((T(1) - alpha) * half_angle) / sin_half_angle;
                    tmp_w = sin(alpha * half_angle) / sin_half_angle;
                }
            }
            return quaternion(
                    q.x * tmp_q + w.x * tmp_w,
                    q.y * tmp_q + w.y * tmp_w,
                    q.z * tmp_q + w.z * tmp_w,
                    q.w * tmp_q + w.w * tmp_w);
        }
    };

    /* Interactions of quaternions and vectors / matrices */

    template<typename T> vector<T, 3>& operator*=(vector<T, 3>& v, const quaternion<T>& q)
    {
        return v = v * q;
    }

    template<typename T> matrix<T, 4, 4> rotate(const matrix<T, 4, 4>& m, const quaternion<T>& q)
    {
        return m * toMat4(q);
    }

    template<typename T> matrix<T, 3, 3> rotate(const matrix<T, 3, 3>& m, const quaternion<T>& q)
    {
        return m * toMat3(q);
    }

    typedef quaternion<float> quat;
    typedef quaternion<double> dquat;


    /* Conversions of various representations of rotations:
     * - angle / axis
     * - dir1 -> dir2
     * - Euler angles
     * - rotation matrix
     */

    /* Helper function: get some vector that is orthogonal to the given normalized vector */
    template<typename T> vector<T, 3> someTangentTo(const vector<T, 3>& v)
    {
        // From Duff et al. "Building an Orthonormal Basis, Revisited", JCGT vol 6 no 1, 2017
        T sign = std::copysign(T(1), v.z());
        T a = T(-1) / (sign + v.z());
        T b = v.x() * v.y() * a;
        return vector<T, 3>(T(1) + sign * v.x() * v.x() * a, sign * b, -sign * v.x());
    }

    /* Conversions between different representations of rotations:
     * from angle/axis, oldpoint/newpoint, euler angles, matrix3 to quat
     * from angle/axis, oldpoint/newpoint, euler angles, quat to matrix3
     * from angle/axis, oldpoint/newpoint, euler angles, quat to matrix4
     * from angle/axis, oldpoint/newpoint, matrix3, quat to euler angles
     * TODO: Conversions from euler angles, matrix3, quat to angle/axis.
     */

    template<typename T> quaternion<T> toQuat(T angle, const vector<T, 3>& axis)
    {
        // The normalization is necessary since even e.g. a cross product of two
        // nearly-normalized vector might not be precisely unit lenth.
        // This happens e.g. if this function is called from toQuat(dir1, dir2).
        vector<T, 3> axisNormalized = normalize(axis);
        T sin_a = sin(T(0.5l) * angle);
        T cos_a = cos(T(0.5l) * angle);
        return quaternion<T>(axisNormalized.x() * sin_a, axisNormalized.y() * sin_a, axisNormalized.z() * sin_a, cos_a);
    }

    template<typename T> quaternion<T> toQuat(const vector<T, 3>& dir1Normalized, const vector<T, 3>& dir2Normalized)
    {
        T cosAngle = dot(dir1Normalized, dir2Normalized);
        quaternion<T> q;
        if (cosAngle >= T(1) - epsilon_v<T>) {
            // angle is very close to zero
            q = quaternion<T>::null();
        } else if (cosAngle <= T(-1) + epsilon_v<T>) {
            // angle is very close to 180°
            vector<T, 3> axis = someTangentTo(dir1Normalized);
            q = quaternion<T>(axis.x(), axis.y(), axis.z(), T(0));
        } else {
            q = toQuat(acos(cosAngle), cross(dir1Normalized, dir2Normalized));
        }
        return q;
    }

    template<typename T> quaternion<T> toQuat(const vector<T, 3>& euler_rot)
    {
        T x2 = T(0.5l) * euler_rot.x();
        T y2 = T(0.5l) * euler_rot.y();
        T z2 = T(0.5l) * euler_rot.z();
        T cx2 = cos(x2);
        T sx2 = sin(x2);
        T cy2 = cos(y2);
        T sy2 = sin(y2);
        T cz2 = cos(z2);
        T sz2 = sin(z2);
        quaternion<T> q;
        q.x = sx2 * cy2 * cz2 - cx2 * sy2 * sz2;
        q.y = cx2 * sy2 * cz2 + sx2 * cy2 * sz2;
        q.z = cx2 * cy2 * sz2 - sx2 * sy2 * cz2;
        q.w = cx2 * cy2 * cz2 + sx2 * sy2 * sz2;
        return q;
    }

    template<typename T> quaternion<T> toQuat(const matrix<T, 3, 3>& rot_matrix)
    {
        quaternion<T> q;
        // From "Matrix and Quaternion FAQ", Q55
        T t = T(1) + rot_matrix[0][0] + rot_matrix[1][1] + rot_matrix[2][2];
        if (t > epsilon_v<T>)
        {
            T s = sqrt(t) * T(2);
            q.x = (rot_matrix[1][2] - rot_matrix[2][1]) / s;
            q.y = (rot_matrix[2][0] - rot_matrix[0][2]) / s;
            q.z = (rot_matrix[0][1] - rot_matrix[1][0]) / s;
            q.w = s / T(4);
        }
        else if (rot_matrix[0][0] > rot_matrix[1][1] && rot_matrix[0][0] > rot_matrix[2][2])
        {
            t = T(1) + rot_matrix[0][0] - rot_matrix[1][1] - rot_matrix[2][2];
            T s = sqrt(t) * T(2);
            q.x = s / T(4);
            q.y = (rot_matrix[0][1] + rot_matrix[1][0]) / s;
            q.z = (rot_matrix[2][0] + rot_matrix[0][2]) / s;
            q.w = (rot_matrix[1][2] - rot_matrix[2][1]) / s;
        }
        else if (rot_matrix[1][1] > rot_matrix[2][2])
        {
            t = T(1) + rot_matrix[1][1] - rot_matrix[0][0] - rot_matrix[2][2];
            T s = sqrt(t) * T(2);
            q.x = (rot_matrix[0][1] + rot_matrix[1][0]) / s;
            q.y = s / T(4);
            q.z = (rot_matrix[1][2] + rot_matrix[2][1]) / s;
            q.w = (rot_matrix[2][0] - rot_matrix[0][2]) / s;
        }
        else
        {
            t = T(1) + rot_matrix[2][2] - rot_matrix[0][0] - rot_matrix[1][1];
            T s = sqrt(t) * T(2);
            q.x = (rot_matrix[2][0] + rot_matrix[0][2]) / s;
            q.y = (rot_matrix[1][2] + rot_matrix[2][1]) / s;
            q.z = s / T(4);
            q.w = (rot_matrix[0][1] - rot_matrix[1][0]) / s;
        }
        return q;
    }

    template<typename T> matrix<T, 3, 3> toMat3(T angle, const vector<T, 3>& axisNormalized)
    {
        const vector<T, 3>& n = axisNormalized;
        T c = cos(angle);
        T s = sin(angle);
        T mc = T(1) - c;
        matrix<T, 3, 3> m;
        m[0][0] = n.x * n.x * mc + c;
        m[0][1] = n.y * n.x * mc + n.z * s;
        m[0][2] = n.x * n.z * mc - n.y * s;
        m[1][0] = n.x * n.y * mc - n.z * s;
        m[1][1] = n.y * n.y * mc + c;
        m[1][2] = n.y * n.z * mc + n.x * s;
        m[2][0] = n.x * n.z * mc + n.y * s;
        m[2][1] = n.y * n.z * mc - n.x * s;
        m[2][2] = n.z * n.z * mc + c;
        return m;
    }

    template<typename T> matrix<T, 3, 3> toMat3(const vector<T, 3>& dir1Normalized, const vector<T, 3>& dir2Normalized)
    {
        T cosAngle = dot(dir1Normalized, dir2Normalized);
        T angle;
        vector<T, 3> axis;
        if (cosAngle >= T(1) - epsilon_v<T>) {
            // angle is very close to zero
            angle = T(0);
            axis = vector<T, 3>(T(1), T(0), T(0));
        } else if (cosAngle <= T(-1) + epsilon_v<T>) {
            // angle is very close to 180°
            angle = pi_v<T>;
            axis = someTangentTo(dir1Normalized);
        } else {
            angle = acos(cosAngle);
            axis = cross(dir1Normalized, dir2Normalized);
        }
        return toMat3(angle, axis);
    }

    template<typename T> matrix<T, 3, 3> toMat3(const vector<T, 3>& euler_rot)
    {
        return toMat3(toQuat(euler_rot));
    }

    template<typename T> matrix<T, 3, 3> toMat3(const quaternion<T>& q)
    {
        matrix<T, 3, 3> m;
        T xx = q.x * q.x;
        T xy = q.x * q.y;
        T xz = q.x * q.z;
        T xw = q.x * q.w;
        T yy = q.y * q.y;
        T yz = q.y * q.z;
        T yw = q.y * q.w;
        T zz = q.z * q.z;
        T zw = q.z * q.w;
        m[0][0] = T(1) - T(2) * (yy + zz);
        m[0][1] = T(2) * (xy + zw);
        m[0][2] = T(2) * (xz - yw);
        m[1][0] = T(2) * (xy - zw);
        m[1][1] = T(1) - T(2) * (xx + zz);
        m[1][2] = T(2) * (yz + xw);
        m[2][0] = T(2) * (xz + yw);
        m[2][1] = T(2) * (yz - xw);
        m[2][2] = T(1) - T(2) * (xx + yy);
        return m;
    }

    template<typename T> matrix<T, 4, 4> toMat4(T angle, const vector<T, 3>& axis)
    {
        matrix<T, 3, 3> r = toMat3(angle, axis);
        matrix<T, 4, 4> m;
        m[0][0] = r[0][0];
        m[0][1] = r[0][1];
        m[0][2] = r[0][2];
        m[0][3] = T(0);
        m[1][0] = r[1][0];
        m[1][1] = r[1][1];
        m[1][2] = r[1][2];
        m[1][3] = T(0);
        m[2][0] = r[2][0];
        m[2][1] = r[2][1];
        m[2][2] = r[2][2];
        m[2][3] = T(0);
        m[3][0] = T(0);
        m[3][1] = T(0);
        m[3][2] = T(0);
        m[3][3] = T(1);
        return m;
    }

    template<typename T> matrix<T, 4, 4> toMat4(const vector<T, 3>& dir1Normalized, const vector<T, 3>& dir2Normalized)
    {
        matrix<T, 3, 3> r = toMat3(dir1Normalized, dir2Normalized);
        matrix<T, 4, 4> m;
        m[0][0] = r[0][0];
        m[0][1] = r[0][1];
        m[0][2] = r[0][2];
        m[0][3] = T(0);
        m[1][0] = r[1][0];
        m[1][1] = r[1][1];
        m[1][2] = r[1][2];
        m[1][3] = T(0);
        m[2][0] = r[2][0];
        m[2][1] = r[2][1];
        m[2][2] = r[2][2];
        m[2][3] = T(0);
        m[3][0] = T(0);
        m[3][1] = T(0);
        m[3][2] = T(0);
        m[3][3] = T(1);
        return m;
    }

    template<typename T> matrix<T, 4, 4> toMat4(const vector<T, 3>& euler_rot)
    {
        return toMat4(toQuat(euler_rot));
    }

    template<typename T> matrix<T, 4, 4> toMat4(const quaternion<T>& q)
    {
        matrix<T, 3, 3> r = toMat3(q);
        matrix<T, 4, 4> m;
        m[0][0] = r[0][0];
        m[0][1] = r[0][1];
        m[0][2] = r[0][2];
        m[0][3] = T(0);
        m[1][0] = r[1][0];
        m[1][1] = r[1][1];
        m[1][2] = r[1][2];
        m[1][3] = T(0);
        m[2][0] = r[2][0];
        m[2][1] = r[2][1];
        m[2][2] = r[2][2];
        m[2][3] = T(0);
        m[3][0] = T(0);
        m[3][1] = T(0);
        m[3][2] = T(0);
        m[3][3] = T(1);
        return m;
    }

    template<typename T> vector<T, 3> toEuler(T angle, const vector<T, 3>& axis)
    {
        return toEuler(toQuat(angle, axis));
    }

    template<typename T> vector<T, 3> toEuler(const vector<T, 3>& oldpoint, const vector<T, 3>& newpoint)
    {
        return toEuler(toQuat(oldpoint, newpoint));
    }

    template<typename T> matrix<T, 3, 3> toEuler(const matrix<T, 3, 3>& rot_matrix)
    {
        return toEuler(toQuat(rot_matrix));
    }

    template<typename T> vector<T, 3> toEuler(const quaternion<T>& q)
    {
        constexpr T singularity_test_val = T(0.5) - epsilon_v<T>;
        T singularity_test = q.x * q.y + q.z * q.w;
        vector<T, 3> result;
        if (singularity_test > T(+singularity_test_val))
        {
            // north pole
            result = vector<T, 3>(
                    T(2) * atan(q.x, q.w),
                    pi_2_v<T>,
                    T(0));
        }
        else if (singularity_test < T(-singularity_test_val))
        {
            // south pole
            result = vector<T, 3>(
                    -T(2) * atan(q.x, q.w),
                    -pi_2_v<T>,
                    T(0));
        }
        else
        {
            result = vector<T, 3>(
                    atan(T(2) * (q.w * q.x + q.y * q.z),
                        (T(1) - T(2) * (q.x * q.x + q.y * q.y))),
                    asin(T(2) * (q.w * q.y - q.x * q.z)),
                    atan(T(2) * (q.w * q.z + q.x * q.y),
                        (T(1) - T(2) * (q.y * q.y + q.z * q.z))));
        }
        return result;
    }
}
