/*
 * Copyright © 2017 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#ifndef BRW_DEFINES_COMMON_H
#endif // BRW_DEFINES_COMMON_H

enum brw_pixel_shader_computed_depth_mode {
   BRW_PSCDEPTH_OFF   = 0, /* PS does not compute depth */
   BRW_PSCDEPTH_ON    = 1, /* PS computes depth; no guarantee about value */
   BRW_PSCDEPTH_ON_GE = 2, /* PS guarantees output depth >= source depth */
   BRW_PSCDEPTH_ON_LE = 3, /* PS guarantees output depth <= source depth */
};

enum brw_barycentric_mode {
   BRW_BARYCENTRIC_PERSPECTIVE_PIXEL       = 0,
   BRW_BARYCENTRIC_PERSPECTIVE_CENTROID    = 1,
   BRW_BARYCENTRIC_PERSPECTIVE_SAMPLE      = 2,
   BRW_BARYCENTRIC_NONPERSPECTIVE_PIXEL    = 3,
   BRW_BARYCENTRIC_NONPERSPECTIVE_CENTROID = 4,
   BRW_BARYCENTRIC_NONPERSPECTIVE_SAMPLE   = 5,
   BRW_BARYCENTRIC_MODE_COUNT              = 6
};
#define BRW_BARYCENTRIC_NONPERSPECTIVE_BITS \
   ((1 << BRW_BARYCENTRIC_NONPERSPECTIVE_PIXEL) | \
    (1 << BRW_BARYCENTRIC_NONPERSPECTIVE_CENTROID) | \
    (1 << BRW_BARYCENTRIC_NONPERSPECTIVE_SAMPLE))
