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

#include <tgd/array.hpp>
#include <tgd/io.hpp>

#include <wurblpt/wurblpt.hpp>


using namespace WurblPT;

void print_matrix(const mat4& M)
{
    fprintf(stderr,
            "[ %.2g %.2g %.2g %.2g ]\n"
            "[ %.2g %.2g %.2g %.2g ]\n"
            "[ %.2g %.2g %.2g %.2g ]\n"
            "[ %.2g %.2g %.2g %.2g ]\n\n",
            M[0][0], M[1][0], M[2][0], M[3][0],
            M[0][1], M[1][1], M[2][1], M[3][1],
            M[0][2], M[1][2], M[2][2], M[3][2],
            M[0][3], M[1][3], M[2][3], M[3][3]);
}

float print_matrix_diff(const mat4& M0, const mat4& M1)
{
    float diff = 0.0f;
    for (int col = 0; col < 4; col++) {
        for (int row = 0; row < 4; row++) {
            diff += abs(M0[col][row] - M1[col][row]);
        }
    }
    fprintf(stderr, "diff: %g\n\n", diff);
    return diff;
}

int main(void)
{
    Transformation T1;
    mat4 M1(1.0f);

    T1 = translate(T1, vec3(1, 2, 3));
    T1 = rotate(T1, toQuat(radians(15.0f), vec3(1.0f, 1.0f, 0.0f)));
    T1 = scale(T1, vec3(0.5f));
    M1 = translate(M1, vec3(1, 2, 3));
    M1 = rotate(M1, toQuat(radians(15.0f), vec3(1.0f, 1.0f, 0.0f)));
    M1 = scale(M1, vec3(0.5f));

    print_matrix(T1.toMat4());
    print_matrix(M1);
    assert(print_matrix_diff(T1.toMat4(), M1) < 0.0001f);

    Transformation T2;
    mat4 M2(1.0f);

    T2 = scale(T2, vec3(0.4f));
    T2 = rotate(T2, toQuat(radians(27.0f), vec3(1.0f, 0.0f, 0.0f)));
    T2 = translate(T2, vec3(3, 2, 1));
    M2 = scale(M2, vec3(0.4f));
    M2 = rotate(M2, toQuat(radians(27.0f), vec3(1.0f, 0.0f, 0.0f)));
    M2 = translate(M2, vec3(3, 2, 1));

    print_matrix(T2.toMat4());
    print_matrix(M2);
    assert(print_matrix_diff(T2.toMat4(), M2) < 0.0001f);

    Transformation T3;
    mat4 M3(1.0f);

    T3 = translate(T3, vec3(3, 2, 1));
    T3 = scale(T3, vec3(0.4f));
    T3 = rotate(T3, toQuat(radians(27.0f), vec3(1.0f, 0.0f, 0.0f)));
    M3 = translate(M3, vec3(3, 2, 1));
    M3 = scale(M3, vec3(0.4f));
    M3 = rotate(M3, toQuat(radians(27.0f), vec3(1.0f, 0.0f, 0.0f)));

    print_matrix(T3.toMat4());
    print_matrix(M3);
    assert(print_matrix_diff(T3.toMat4(), M3) < 0.0001f);

    Transformation T4;
    mat4 M4(1.0f);

    T4 = rotate(T4, toQuat(radians(27.0f), vec3(1.0f, 0.0f, 0.0f)));
    T4 = translate(T4, vec3(3, 2, 1));
    T4 = scale(T4, vec3(0.4f));
    M4 = rotate(M4, toQuat(radians(27.0f), vec3(1.0f, 0.0f, 0.0f)));
    M4 = translate(M4, vec3(3, 2, 1));
    M4 = scale(M4, vec3(0.4f));

    print_matrix(T4.toMat4());
    print_matrix(M4);
    assert(print_matrix_diff(T4.toMat4(), M4) < 0.0001f);

    Transformation T5;
    mat4 M5(1.0f);

    T5 = rotate(T5, toQuat(radians(27.0f), vec3(1.0f, 0.0f, 0.0f)));
    T5 = scale(T5, vec3(0.4f));
    T5 = translate(T5, vec3(3, 2, 1));
    M5 = rotate(M5, toQuat(radians(27.0f), vec3(1.0f, 0.0f, 0.0f)));
    M5 = scale(M5, vec3(0.4f));
    M5 = translate(M5, vec3(3, 2, 1));

    print_matrix(T5.toMat4());
    print_matrix(M5);
    assert(print_matrix_diff(T5.toMat4(), M5) < 0.0001f);

    Transformation T6;
    mat4 M6(1.0f);

    T6 = scale(T6, vec3(0.4f));
    T6 = translate(T6, vec3(3, 2, 1));
    T6 = rotate(T6, toQuat(radians(27.0f), vec3(1.0f, 0.0f, 0.0f)));
    M6 = scale(M6, vec3(0.4f));
    M6 = translate(M6, vec3(3, 2, 1));
    M6 = rotate(M6, toQuat(radians(27.0f), vec3(1.0f, 0.0f, 0.0f)));

    print_matrix(T6.toMat4());
    print_matrix(M6);
    assert(print_matrix_diff(T6.toMat4(), M6) < 0.0001f);

    Transformation T7 = T1 * T2 * T3;
    mat4 M7 = M1 * M2 * M3;
    print_matrix(T7.toMat4());
    print_matrix(M7);
    assert(print_matrix_diff(T7.toMat4(), M7) < 0.0001f);

    Transformation T8 = T3 * T2 * T1;
    mat4 M8 = M3 * M2 * M1;
    print_matrix(T8.toMat4());
    print_matrix(M8);
    assert(print_matrix_diff(T8.toMat4(), M8) < 0.0001f);

    return 0;
}
