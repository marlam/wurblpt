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

/* This tool converts stereoscopic to monoscopic videos by extracting the left view. */

#include <tgd/array.hpp>
#include <tgd/io.hpp>


int main(int argc, char* argv[])
{
    if (argc != 3) {
        fprintf(stderr, "Usage: %s <input> <output>\n", argv[0]);
        return 1;
    }

    TGD::Error err = TGD::ErrorNone;
    TGD::Importer importer(argv[1]);
    TGD::Exporter exporter(argv[2]);
    for (;;) {
        if (!importer.hasMore(&err)) {
            if (err != TGD::ErrorNone) {
                fprintf(stderr, "%s: %s: %s\n", argv[0], importer.fileName().c_str(), TGD::strerror(err));
            }
            break;
        }
        TGD::ArrayContainer a = importer.readArray(&err);
        if (err != TGD::ErrorNone) {
            fprintf(stderr, "%s: %s: %s\n", argv[0], importer.fileName().c_str(), TGD::strerror(err));
            break;
        }

        TGD::ArrayContainer r({ a.dimension(0), a.dimension(1) / 2 }, a.componentCount(), a.componentType());
        std::memcpy(r.data(), a.get(a.elementCount() / 2), r.dataSize());

        err = exporter.writeArray(r);
        if (err != TGD::ErrorNone) {
            fprintf(stderr, "%s: %s: %s\n", argv[0], exporter.fileName().c_str(), TGD::strerror(err));
            break;
        }
    }

    return (err == TGD::ErrorNone ? 0 : 1);
}
