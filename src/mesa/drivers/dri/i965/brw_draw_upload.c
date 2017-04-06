/*
 * Copyright 2003 VMware, Inc.
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL VMWARE AND/OR ITS SUPPLIERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "main/bufferobj.h"
#include "main/context.h"
#include "main/enums.h"
#include "main/macros.h"
#include "main/glformats.h"

#include "brw_draw.h"
#include "brw_defines.h"
#include "brw_context.h"
#include "brw_state.h"

#include "intel_batchbuffer.h"
#include "intel_buffer_objects.h"

static void
brw_upload_indices(struct brw_context *brw)
{
   struct gl_context *ctx = &brw->ctx;
   const struct _mesa_index_buffer *index_buffer = brw->ib.ib;
   GLuint ib_size;
   struct brw_bo *old_bo = brw->ib.bo;
   struct gl_buffer_object *bufferobj;
   GLuint offset;
   GLuint ib_type_size;

   if (index_buffer == NULL)
      return;

   ib_type_size = index_buffer->index_size;
   ib_size = index_buffer->count ? ib_type_size * index_buffer->count :
                                   index_buffer->obj->Size;
   bufferobj = index_buffer->obj;

   /* Turn into a proper VBO:
    */
   if (!_mesa_is_bufferobj(bufferobj)) {
      /* Get new bufferobj, offset:
       */
      intel_upload_data(brw, index_buffer->ptr, ib_size, ib_type_size,
			&brw->ib.bo, &offset);
      brw->ib.size = brw->ib.bo->size;
   } else {
      offset = (GLuint) (unsigned long) index_buffer->ptr;

      /* If the index buffer isn't aligned to its element size, we have to
       * rebase it into a temporary.
       */
      if ((ib_type_size - 1) & offset) {
         perf_debug("copying index buffer to a temporary to work around "
                    "misaligned offset %d\n", offset);

         GLubyte *map = ctx->Driver.MapBufferRange(ctx,
                                                   offset,
                                                   ib_size,
                                                   GL_MAP_READ_BIT,
                                                   bufferobj,
                                                   MAP_INTERNAL);

         intel_upload_data(brw, map, ib_size, ib_type_size,
                           &brw->ib.bo, &offset);
         brw->ib.size = brw->ib.bo->size;

         ctx->Driver.UnmapBuffer(ctx, bufferobj, MAP_INTERNAL);
      } else {
         struct brw_bo *bo =
            intel_bufferobj_buffer(brw, intel_buffer_object(bufferobj),
                                   offset, ib_size);
         if (bo != brw->ib.bo) {
            brw_bo_unreference(brw->ib.bo);
            brw->ib.bo = bo;
            brw->ib.size = bufferobj->Size;
            brw_bo_reference(bo);
         }
      }
   }

   /* Use 3DPRIMITIVE's start_vertex_offset to avoid re-uploading
    * the index buffer state when we're just moving the start index
    * of our drawing.
    */
   brw->ib.start_vertex_offset = offset / ib_type_size;

   if (brw->ib.bo != old_bo)
      brw->ctx.NewDriverState |= BRW_NEW_INDEX_BUFFER;

   if (index_buffer->index_size != brw->ib.index_size) {
      brw->ib.index_size = index_buffer->index_size;
      brw->ctx.NewDriverState |= BRW_NEW_INDEX_BUFFER;
   }
}

const struct brw_tracked_state brw_indices = {
   .dirty = {
      .mesa = 0,
      .brw = BRW_NEW_BLORP |
             BRW_NEW_INDICES,
   },
   .emit = brw_upload_indices,
};

static void
brw_emit_index_buffer(struct brw_context *brw)
{
   const struct _mesa_index_buffer *index_buffer = brw->ib.ib;
   GLuint cut_index_setting;

   if (index_buffer == NULL)
      return;

   if (brw->prim_restart.enable_cut_index && !brw->is_haswell) {
      cut_index_setting = BRW_CUT_INDEX_ENABLE;
   } else {
      cut_index_setting = 0;
   }

   BEGIN_BATCH(3);
   OUT_BATCH(CMD_INDEX_BUFFER << 16 |
             cut_index_setting |
             brw_get_index_type(index_buffer->index_size) |
             1);
   OUT_RELOC(brw->ib.bo,
             I915_GEM_DOMAIN_VERTEX, 0,
             0);
   OUT_RELOC(brw->ib.bo,
             I915_GEM_DOMAIN_VERTEX, 0,
	     brw->ib.size - 1);
   ADVANCE_BATCH();
}

const struct brw_tracked_state brw_index_buffer = {
   .dirty = {
      .mesa = 0,
      .brw = BRW_NEW_BATCH |
             BRW_NEW_BLORP |
             BRW_NEW_INDEX_BUFFER,
   },
   .emit = brw_emit_index_buffer,
};
