/*
 * Copyright © 2014 Intel Corporation
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
 *
 * Authors:
 *    Neil Roberts <neil@linux.intel.com>
 */

/** @file brw_conditional_render.c
 *
 * Support for conditional rendering based on query objects
 * (GL_NV_conditional_render, GL_ARB_conditional_render_inverted) on Gen7+.
 */

#include "main/imports.h"
#include "main/condrender.h"

#include "brw_context.h"
#include "brw_defines.h"
#include "intel_batchbuffer.h"

static void
set_predicate_enable(struct brw_context *brw,
                     bool value)
{
   if (value)
      brw->predicate.state = BRW_PREDICATE_STATE_RENDER;
   else
      brw->predicate.state = BRW_PREDICATE_STATE_DONT_RENDER;
}

static void
set_predicate_for_overflow_query(struct brw_context *brw,
                                 struct brw_query_object *query,
                                 int stream_start, int count)
{
   /* R3 = R4 - R3; generated vertices
    * R1 = R2 - R1; written vertices
    * R1 = R3 - R1; there was an overflow on this stream
    * R0 = R0 | R1; accumulate whether there was any overflow
    */
   static const uint32_t maths[] = {
      MI_MATH_ALU2(LOAD, SRCA, R4),
      MI_MATH_ALU2(LOAD, SRCB, R3),
      MI_MATH_ALU0(SUB),
      MI_MATH_ALU2(STORE, R3, ACCU),
      MI_MATH_ALU2(LOAD, SRCA, R2),
      MI_MATH_ALU2(LOAD, SRCB, R1),
      MI_MATH_ALU0(SUB),
      MI_MATH_ALU2(STORE, R1, ACCU),
      MI_MATH_ALU2(LOAD, SRCA, R3),
      MI_MATH_ALU2(LOAD, SRCB, R1),
      MI_MATH_ALU0(SUB),
      MI_MATH_ALU2(STORE, R1, ACCU),
      MI_MATH_ALU2(LOAD, SRCA, R1),
      MI_MATH_ALU2(LOAD, SRCB, R0),
      MI_MATH_ALU0(OR),
      MI_MATH_ALU2(STORE, R0, ACCU),
   };

   brw_load_register_imm64(brw, HSW_CS_GPR(0), 0ull);

   for (int i = stream_start; i < stream_start + count; i++) {
      int offset = 32 * i;
      brw_load_register_mem64(brw,
                              HSW_CS_GPR(1),
                              query->bo,
                              I915_GEM_DOMAIN_INSTRUCTION,
                              0, /* write domain */
                              offset);
      offset += 8;
      brw_load_register_mem64(brw,
                              HSW_CS_GPR(2),
                              query->bo,
                              I915_GEM_DOMAIN_INSTRUCTION,
                              0, /* write domain */
                              offset);
      offset += 8;
      brw_load_register_mem64(brw,
                              HSW_CS_GPR(3),
                              query->bo,
                              I915_GEM_DOMAIN_INSTRUCTION,
                              0, /* write domain */
                              offset);
      offset += 8;
      brw_load_register_mem64(brw,
                              HSW_CS_GPR(4),
                              query->bo,
                              I915_GEM_DOMAIN_INSTRUCTION,
                              0, /* write domain */
                              offset);

      BEGIN_BATCH(1 + ARRAY_SIZE(maths));
      OUT_BATCH(HSW_MI_MATH | (1 + ARRAY_SIZE(maths) - 2));

      for (int m = 0; m < ARRAY_SIZE(maths); m++)
         OUT_BATCH(maths[m]);

      ADVANCE_BATCH();
   }

   brw_load_register_reg64(brw, HSW_CS_GPR(0), MI_PREDICATE_SRC0);
   brw_load_register_imm64(brw, MI_PREDICATE_SRC1, 0ull);
}

static void
set_predicate_for_occlusion_query(struct brw_context *brw,
                                  struct brw_query_object *query)
{
   brw_load_register_mem64(brw,
                           MI_PREDICATE_SRC0,
                           query->bo,
                           I915_GEM_DOMAIN_INSTRUCTION,
                           0, /* write domain */
                           0 /* offset */);
   brw_load_register_mem64(brw,
                           MI_PREDICATE_SRC1,
                           query->bo,
                           I915_GEM_DOMAIN_INSTRUCTION,
                           0, /* write domain */
                           8 /* offset */);
}

static void
set_predicate_for_result(struct brw_context *brw,
                         struct brw_query_object *query,
                         bool inverted)
{

   int load_op;

   assert(query->bo != NULL);

   /* Needed to ensure the memory is coherent for the MI_LOAD_REGISTER_MEM
    * command when loading the values into the predicate source registers for
    * conditional rendering.
    */
   brw_emit_pipe_control_flush(brw, PIPE_CONTROL_FLUSH_ENABLE);

   switch (query->Base.Target) {
   case GL_TRANSFORM_FEEDBACK_STREAM_OVERFLOW_ARB:
      set_predicate_for_overflow_query(brw, query, 0, 1);
      break;
   case GL_TRANSFORM_FEEDBACK_OVERFLOW_ARB:
      set_predicate_for_overflow_query(brw, query, 0, MAX_VERTEX_STREAMS);
      break;
   default:
      set_predicate_for_occlusion_query(brw, query);
   }

   if (inverted)
      load_op = MI_PREDICATE_LOADOP_LOAD;
   else
      load_op = MI_PREDICATE_LOADOP_LOADINV;

   BEGIN_BATCH(1);
   OUT_BATCH(GEN7_MI_PREDICATE |
             load_op |
             MI_PREDICATE_COMBINEOP_SET |
             MI_PREDICATE_COMPAREOP_SRCS_EQUAL);
   ADVANCE_BATCH();

   brw->predicate.state = BRW_PREDICATE_STATE_USE_BIT;
}

static void
brw_begin_conditional_render(struct gl_context *ctx,
                             struct gl_query_object *q,
                             GLenum mode)
{
   struct brw_context *brw = brw_context(ctx);
   struct brw_query_object *query = (struct brw_query_object *) q;
   bool inverted;

   if (!brw->predicate.supported)
      return;

   switch (mode) {
   case GL_QUERY_WAIT:
   case GL_QUERY_NO_WAIT:
   case GL_QUERY_BY_REGION_WAIT:
   case GL_QUERY_BY_REGION_NO_WAIT:
      inverted = false;
      break;
   case GL_QUERY_WAIT_INVERTED:
   case GL_QUERY_NO_WAIT_INVERTED:
   case GL_QUERY_BY_REGION_WAIT_INVERTED:
   case GL_QUERY_BY_REGION_NO_WAIT_INVERTED:
      inverted = true;
      break;
   default:
      unreachable("Unexpected conditional render mode");
   }

   /* If there are already samples from a BLT operation or if the query object
    * is ready then we can avoid looking at the values in the buffer and just
    * decide whether to draw using the CPU without stalling.
    */
   if (query->Base.Result || query->Base.Ready)
      set_predicate_enable(brw, (query->Base.Result != 0) ^ inverted);
   else
      set_predicate_for_result(brw, query, inverted);
}

static void
brw_end_conditional_render(struct gl_context *ctx,
                           struct gl_query_object *q)
{
   struct brw_context *brw = brw_context(ctx);

   /* When there is no longer a conditional render in progress it should
    * always render.
    */
   brw->predicate.state = BRW_PREDICATE_STATE_RENDER;
}

void
brw_init_conditional_render_functions(struct dd_function_table *functions)
{
   functions->BeginConditionalRender = brw_begin_conditional_render;
   functions->EndConditionalRender = brw_end_conditional_render;
}

bool
brw_check_conditional_render(struct brw_context *brw)
{
   if (brw->predicate.supported) {
      /* In some cases it is possible to determine that the primitives should
       * be skipped without needing the predicate enable bit and still without
       * stalling.
       */
      return brw->predicate.state != BRW_PREDICATE_STATE_DONT_RENDER;
   } else if (brw->ctx.Query.CondRenderQuery) {
      perf_debug("Conditional rendering is implemented in software and may "
                 "stall.\n");
      return _mesa_check_conditional_render(&brw->ctx);
   } else {
      return true;
   }
}
