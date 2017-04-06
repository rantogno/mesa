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

#include <assert.h>

#include "common/gen_device_info.h"
#include "genxml/gen_macros.h"

#include "brw_context.h"
#if GEN_GEN < 7
#include "brw_defines.h"
#endif
#include "brw_state.h"
#include "brw_wm.h"
#include "brw_util.h"

#include "intel_batchbuffer.h"
#include "intel_buffer_objects.h"
#include "intel_fbo.h"

#include "main/enums.h"
#include "main/fbobject.h"
#include "main/framebuffer.h"
#include "main/glformats.h"
#include "main/shaderapi.h"
#include "main/stencil.h"
#include "main/transformfeedback.h"
#include "main/viewport.h"

UNUSED static void *
emit_dwords(struct brw_context *brw, unsigned n)
{
   intel_batchbuffer_begin(brw, n, RENDER_RING);
   uint32_t *map = brw->batch.map_next;
   brw->batch.map_next += n;
   intel_batchbuffer_advance(brw);
   return map;
}

struct brw_address {
   drm_intel_bo *bo;
   uint32_t read_domains;
   uint32_t write_domain;
   uint32_t offset;
};

static uint64_t
emit_reloc(struct brw_context *brw,
           void *location, struct brw_address address, uint32_t delta)
{
   uint32_t offset = (char *) location - (char *) brw->batch.map;

   return intel_batchbuffer_reloc(&brw->batch, address.bo, offset,
                                  address.read_domains,
                                  address.write_domain,
                                  address.offset + delta);
}

#define __gen_address_type struct brw_address
#define __gen_user_data struct brw_context

static uint64_t
__gen_combine_address(struct brw_context *brw, void *location,
                      struct brw_address address, uint32_t delta)
{
   if (address.bo == NULL) {
      return address.offset + delta;
   } else {
      return emit_reloc(brw, location, address, delta);
   }
}

#include "genxml/genX_pack.h"

#define _brw_cmd_length(cmd) cmd ## _length
#define _brw_cmd_length_bias(cmd) cmd ## _length_bias
#define _brw_cmd_header(cmd) cmd ## _header
#define _brw_cmd_pack(cmd) cmd ## _pack

#define brw_batch_emit(brw, cmd, name)                  \
   for (struct cmd name = { _brw_cmd_header(cmd) },     \
        *_dst = emit_dwords(brw, _brw_cmd_length(cmd)); \
        __builtin_expect(_dst != NULL, 1);              \
        _brw_cmd_pack(cmd)(brw, (void *)_dst, &name),   \
        _dst = NULL)

#define brw_batch_emitn(brw, cmd, n, ...) ({                \
      uint32_t *_dw = emit_dwords(brw, n);             \
      struct cmd template = {                          \
         _brw_cmd_header(cmd),                         \
         .DWordLength = n - _brw_cmd_length_bias(cmd), \
         __VA_ARGS__                                   \
      };                                               \
      _brw_cmd_pack(cmd)(brw, _dw, &template);         \
      _dw + 1; /* Array starts at dw[1] */             \
   })

#define brw_state_emit(brw, cmd, align, offset, name)              \
   for (struct cmd name = { 0, },                                  \
        *_dst = brw_state_batch(brw, _brw_cmd_length(cmd) * 4,     \
                                align, offset);                    \
        __builtin_expect(_dst != NULL, 1);                         \
        _brw_cmd_pack(cmd)(brw, (void *)_dst, &name),              \
        _dst = NULL)

/**
 * Determine the appropriate attribute override value to store into the
 * 3DSTATE_SF structure for a given fragment shader attribute.  The attribute
 * override value contains two pieces of information: the location of the
 * attribute in the VUE (relative to urb_entry_read_offset, see below), and a
 * flag indicating whether to "swizzle" the attribute based on the direction
 * the triangle is facing.
 *
 * If an attribute is "swizzled", then the given VUE location is used for
 * front-facing triangles, and the VUE location that immediately follows is
 * used for back-facing triangles.  We use this to implement the mapping from
 * gl_FrontColor/gl_BackColor to gl_Color.
 *
 * urb_entry_read_offset is the offset into the VUE at which the SF unit is
 * being instructed to begin reading attribute data.  It can be set to a
 * nonzero value to prevent the SF unit from wasting time reading elements of
 * the VUE that are not needed by the fragment shader.  It is measured in
 * 256-bit increments.
 */
static void
genX(get_attr_override)(struct GENX(SF_OUTPUT_ATTRIBUTE_DETAIL) *attr,
                        const struct brw_vue_map *vue_map,
                        int urb_entry_read_offset, int fs_attr,
                        bool two_side_color, uint32_t *max_source_attr)
{
   /* Find the VUE slot for this attribute. */
   int slot = vue_map->varying_to_slot[fs_attr];

   /* Viewport and Layer are stored in the VUE header.  We need to override
    * them to zero if earlier stages didn't write them, as GL requires that
    * they read back as zero when not explicitly set.
    */
   if (fs_attr == VARYING_SLOT_VIEWPORT || fs_attr == VARYING_SLOT_LAYER) {
      attr->ComponentOverrideX = true;
      attr->ComponentOverrideW = true;
      attr->ConstantSource = CONST_0000;

      if (!(vue_map->slots_valid & VARYING_BIT_LAYER))
         attr->ComponentOverrideY = true;
      if (!(vue_map->slots_valid & VARYING_BIT_VIEWPORT))
         attr->ComponentOverrideZ = true;

      return;
   }

   /* If there was only a back color written but not front, use back
    * as the color instead of undefined
    */
   if (slot == -1 && fs_attr == VARYING_SLOT_COL0)
      slot = vue_map->varying_to_slot[VARYING_SLOT_BFC0];
   if (slot == -1 && fs_attr == VARYING_SLOT_COL1)
      slot = vue_map->varying_to_slot[VARYING_SLOT_BFC1];

   if (slot == -1) {
      /* This attribute does not exist in the VUE--that means that the vertex
       * shader did not write to it.  This means that either:
       *
       * (a) This attribute is a texture coordinate, and it is going to be
       * replaced with point coordinates (as a consequence of a call to
       * glTexEnvi(GL_POINT_SPRITE, GL_COORD_REPLACE, GL_TRUE)), so the
       * hardware will ignore whatever attribute override we supply.
       *
       * (b) This attribute is read by the fragment shader but not written by
       * the vertex shader, so its value is undefined.  Therefore the
       * attribute override we supply doesn't matter.
       *
       * (c) This attribute is gl_PrimitiveID, and it wasn't written by the
       * previous shader stage.
       *
       * Note that we don't have to worry about the cases where the attribute
       * is gl_PointCoord or is undergoing point sprite coordinate
       * replacement, because in those cases, this function isn't called.
       *
       * In case (c), we need to program the attribute overrides so that the
       * primitive ID will be stored in this slot.  In every other case, the
       * attribute override we supply doesn't matter.  So just go ahead and
       * program primitive ID in every case.
       */
      attr->ComponentOverrideW = true;
      attr->ComponentOverrideX = true;
      attr->ComponentOverrideY = true;
      attr->ComponentOverrideZ = true;
      attr->ConstantSource = PRIM_ID;
      return;
   }

   /* Compute the location of the attribute relative to urb_entry_read_offset.
    * Each increment of urb_entry_read_offset represents a 256-bit value, so
    * it counts for two 128-bit VUE slots.
    */
   int source_attr = slot - 2 * urb_entry_read_offset;
   assert(source_attr >= 0 && source_attr < 32);

   /* If we are doing two-sided color, and the VUE slot following this one
    * represents a back-facing color, then we need to instruct the SF unit to
    * do back-facing swizzling.
    */
   bool swizzling = two_side_color &&
      ((vue_map->slot_to_varying[slot] == VARYING_SLOT_COL0 &&
        vue_map->slot_to_varying[slot+1] == VARYING_SLOT_BFC0) ||
       (vue_map->slot_to_varying[slot] == VARYING_SLOT_COL1 &&
        vue_map->slot_to_varying[slot+1] == VARYING_SLOT_BFC1));

   /* Update max_source_attr.  If swizzling, the SF will read this slot + 1. */
   if (*max_source_attr < source_attr + swizzling)
      *max_source_attr = source_attr + swizzling;

   attr->SourceAttribute = source_attr;
   if (swizzling)
      attr->SwizzleSelect = INPUTATTR_FACING;
}


static void
genX(calculate_attr_overrides)(const struct brw_context *brw,
                               struct GENX(SF_OUTPUT_ATTRIBUTE_DETAIL) *attr_overrides,
                               uint32_t *point_sprite_enables,
                               uint32_t *urb_entry_read_length,
                               uint32_t *urb_entry_read_offset)
{
   /* BRW_NEW_FS_PROG_DATA */
   const struct brw_wm_prog_data *wm_prog_data =
      brw_wm_prog_data(brw->wm.base.prog_data);
   uint32_t max_source_attr = 0;

   *point_sprite_enables = 0;

   *urb_entry_read_offset = BRW_SF_URB_ENTRY_READ_OFFSET;

   /* BRW_NEW_FRAGMENT_PROGRAM
    *
    * If the fragment shader reads VARYING_SLOT_LAYER, then we need to pass in
    * the full vertex header.  Otherwise, we can program the SF to start
    * reading at an offset of 1 (2 varying slots) to skip unnecessary data:
    * - VARYING_SLOT_PSIZ and BRW_VARYING_SLOT_NDC on gen4-5
    * - VARYING_SLOT_{PSIZ,LAYER} and VARYING_SLOT_POS on gen6+
    */

   bool fs_needs_vue_header = brw->fragment_program->info.inputs_read &
      (VARYING_BIT_LAYER | VARYING_BIT_VIEWPORT);

   *urb_entry_read_offset = fs_needs_vue_header ? 0 : 1;

   /* From the Ivybridge PRM, Vol 2 Part 1, 3DSTATE_SBE,
    * description of dw10 Point Sprite Texture Coordinate Enable:
    *
    * "This field must be programmed to zero when non-point primitives
    * are rendered."
    *
    * The SandyBridge PRM doesn't explicitly say that point sprite enables
    * must be programmed to zero when rendering non-point primitives, but
    * the IvyBridge PRM does, and if we don't, we get garbage.
    *
    * This is not required on Haswell, as the hardware ignores this state
    * when drawing non-points -- although we do still need to be careful to
    * correctly set the attr overrides.
    *
    * _NEW_POLYGON
    * BRW_NEW_PRIMITIVE | BRW_NEW_GS_PROG_DATA | BRW_NEW_TES_PROG_DATA
    */
   bool drawing_points = brw_is_drawing_points(brw);

   for (int attr = 0; attr < VARYING_SLOT_MAX; attr++) {
      int input_index = wm_prog_data->urb_setup[attr];

      if (input_index < 0)
         continue;

      /* _NEW_POINT */
      bool point_sprite = false;
      if (drawing_points) {
         if (brw->ctx.Point.PointSprite &&
             (attr >= VARYING_SLOT_TEX0 && attr <= VARYING_SLOT_TEX7) &&
             (brw->ctx.Point.CoordReplace & (1u << (attr - VARYING_SLOT_TEX0)))) {
            point_sprite = true;
         }

         if (attr == VARYING_SLOT_PNTC)
            point_sprite = true;

         if (point_sprite)
            *point_sprite_enables |= (1 << input_index);
      }

      /* BRW_NEW_VUE_MAP_GEOM_OUT | _NEW_LIGHT | _NEW_PROGRAM */
      struct GENX(SF_OUTPUT_ATTRIBUTE_DETAIL) attribute;
      memset(&attribute, 0, sizeof(attribute));

      if (!point_sprite) {
         genX(get_attr_override)(&attribute,
                                 &brw->vue_map_geom_out,
                                 *urb_entry_read_offset, attr,
                                 brw->ctx.VertexProgram._TwoSideEnabled,
                                 &max_source_attr);
      }

      /* The hardware can only do the overrides on 16 overrides at a
       * time, and the other up to 16 have to be lined up so that the
       * input index = the output index.  We'll need to do some
       * tweaking to make sure that's the case.
       */
      if (input_index < 16)
         attr_overrides[input_index] = attribute;
      /* TODO: re-add this assert */
      /* else */
      /*    assert(attr_override == input_index); */
   }

   /* From the Sandy Bridge PRM, Volume 2, Part 1, documentation for
    * 3DSTATE_SF DWord 1 bits 15:11, "Vertex URB Entry Read Length":
    *
    * "This field should be set to the minimum length required to read the
    *  maximum source attribute.  The maximum source attribute is indicated
    *  by the maximum value of the enabled Attribute # Source Attribute if
    *  Attribute Swizzle Enable is set, Number of Output Attributes-1 if
    *  enable is not set.
    *  read_length = ceiling((max_source_attr + 1) / 2)
    *
    *  [errata] Corruption/Hang possible if length programmed larger than
    *  recommended"
    *
    * Similar text exists for Ivy Bridge.
    */
   *urb_entry_read_length = ALIGN(max_source_attr + 1, 2) / 2;
}

/* ---------------------------------------------------------------------- */

static void
genX(upload_depth_stencil_state)(struct brw_context *brw)
{
   struct gl_context *ctx = &brw->ctx;

   /* _NEW_BUFFERS */
   struct intel_renderbuffer *depth_irb =
      intel_get_renderbuffer(ctx->DrawBuffer, BUFFER_DEPTH);

   /* _NEW_DEPTH */
   struct gl_depthbuffer_attrib *depth = &ctx->Depth;

   /* _NEW_STENCIL */
   struct gl_stencil_attrib *stencil = &ctx->Stencil;
   const int b = stencil->_BackFace;

#if GEN_GEN >= 8
   brw_batch_emit(brw, GENX(3DSTATE_WM_DEPTH_STENCIL), wmds) {
#else
   uint32_t ds_offset;
   brw_state_emit(brw, GENX(DEPTH_STENCIL_STATE), 64, &ds_offset, wmds) {
#endif
      if (depth->Test && depth_irb) {
         wmds.DepthTestEnable = true;
         wmds.DepthBufferWriteEnable = brw_depth_writes_enabled(brw);
         wmds.DepthTestFunction = intel_translate_compare_func(depth->Func);
      }

      if (stencil->_Enabled) {
         wmds.StencilTestEnable = true;
         wmds.StencilWriteMask = stencil->WriteMask[0] & 0xff;
         wmds.StencilTestMask = stencil->ValueMask[0] & 0xff;

         wmds.StencilTestFunction =
            intel_translate_compare_func(stencil->Function[0]);
         wmds.StencilFailOp =
            intel_translate_stencil_op(stencil->FailFunc[0]);
         wmds.StencilPassDepthPassOp =
            intel_translate_stencil_op(stencil->ZPassFunc[0]);
         wmds.StencilPassDepthFailOp =
            intel_translate_stencil_op(stencil->ZFailFunc[0]);

         wmds.StencilBufferWriteEnable = stencil->_WriteEnabled;

         if (stencil->_TestTwoSide) {
            wmds.DoubleSidedStencilEnable = true;
            wmds.BackfaceStencilWriteMask = stencil->WriteMask[b] & 0xff;
            wmds.BackfaceStencilTestMask = stencil->ValueMask[b] & 0xff;

            wmds.BackfaceStencilTestFunction =
               intel_translate_compare_func(stencil->Function[b]);
            wmds.BackfaceStencilFailOp =
               intel_translate_stencil_op(stencil->FailFunc[b]);
            wmds.BackfaceStencilPassDepthPassOp =
               intel_translate_stencil_op(stencil->ZPassFunc[b]);
            wmds.BackfaceStencilPassDepthFailOp =
               intel_translate_stencil_op(stencil->ZFailFunc[b]);
         }

#if GEN_GEN >= 9
         wmds.StencilReferenceValue = _mesa_get_stencil_ref(ctx, 0);
         wmds.BackfaceStencilReferenceValue = _mesa_get_stencil_ref(ctx, b);
#endif
      }
   }

#if GEN_GEN == 6
   brw_batch_emit(brw, GENX(3DSTATE_CC_STATE_POINTERS), ptr) {
      ptr.PointertoDEPTH_STENCIL_STATE = ds_offset;
      ptr.DEPTH_STENCIL_STATEChange = true;
   }
#elif GEN_GEN == 7
   brw_batch_emit(brw, GENX(3DSTATE_DEPTH_STENCIL_STATE_POINTERS), ptr) {
      ptr.PointertoDEPTH_STENCIL_STATE = ds_offset;
   }
#endif
}

static const struct brw_tracked_state genX(depth_stencil_state) = {
   .dirty = {
      .mesa = _NEW_BUFFERS |
              _NEW_DEPTH |
              _NEW_STENCIL,
      .brw  = BRW_NEW_BLORP |
              (GEN_GEN >= 8 ? BRW_NEW_CONTEXT
                            : BRW_NEW_BLORP |
                              BRW_NEW_STATE_BASE_ADDRESS),
   },
   .emit = genX(upload_depth_stencil_state),
};

/* ---------------------------------------------------------------------- */

static void
genX(upload_clip_state)(struct brw_context *brw)
{
   struct gl_context *ctx = &brw->ctx;

   /* _NEW_BUFFERS */
   struct gl_framebuffer *fb = ctx->DrawBuffer;

   brw_batch_emit(brw, GENX(3DSTATE_CLIP), clip) {
      if (!brw->meta_in_progress)
         clip.StatisticsEnable = true;

      /* TODO: How to properly check for this?
       * For now, copying code form vulkan
       */
      if (brw_wm_prog_data(brw->wm.base.prog_data)->barycentric_interp_modes &
          0x38)
         clip.NonPerspectiveBarycentricEnable = true;

      clip.UserClipDistanceCullTestEnableBitmask =
         brw_vue_prog_data(brw->vs.base.prog_data)->cull_distance_mask;

#if GEN_GEN >= 7
      clip.EarlyCullEnable = true;
#endif

#if GEN_GEN == 7
      if (ctx->Polygon._FrontBit == _mesa_is_user_fbo(fb))
         clip.FrontWinding = 1;

      if (ctx->Polygon.CullFlag) {
         switch (ctx->Polygon.CullFaceMode) {
            case GL_FRONT:
               clip.CullMode = CULLMODE_FRONT;
               break;
            case GL_BACK:
               clip.CullMode = CULLMODE_BACK;
               break;
            case GL_FRONT_AND_BACK:
               clip.CullMode = CULLMODE_BOTH;
               break;
            default:
               unreachable("Should not get here: invalid CullFlag");
         }
      } else {
         clip.CullMode = CULLMODE_NONE;
      }
#endif

#if GEN_GEN < 8
      if (!ctx->Transform.DepthClamp)
         clip.ViewportZClipTestEnable = true;
#endif

      /* _NEW_LIGHT */
      if (ctx->Light.ProvokingVertex == GL_FIRST_VERTEX_CONVENTION) {
         clip.TriangleStripListProvokingVertexSelect = 0;
         clip.TriangleFanProvokingVertexSelect = 1;
         clip.LineStripListProvokingVertexSelect = 0;
      } else {
         clip.TriangleStripListProvokingVertexSelect = 2;
         clip.TriangleFanProvokingVertexSelect = 2;
         clip.LineStripListProvokingVertexSelect = 1;
      }

      /* _NEW_TRANSFORM */
      clip.UserClipDistanceClipTestEnableBitmask =
         ctx->Transform.ClipPlanesEnabled;

#if GEN_GEN >= 8
      clip.ForceUserClipDistanceClipTestEnableBitmask = true;
#endif

      if (ctx->Transform.ClipDepthMode == GL_ZERO_TO_ONE)
         clip.APIMode = APIMODE_D3D;
      else
         clip.APIMode = APIMODE_OGL;

      clip.GuardbandClipTestEnable = true;

      /* BRW_NEW_VIEWPORT_COUNT */
      const unsigned viewport_count = brw->clip.viewport_count;

      if (ctx->RasterDiscard) {
         clip.ClipMode = CLIPMODE_REJECT_ALL;
#if GEN_GEN == 6
         perf_debug("Rasterizer discard is currently implemented via the "
                    "clipper; having the GS not write primitives would "
                    "likely be faster.\n");
#endif
      } else {
         clip.ClipMode = CLIPMODE_NORMAL;
      }

      if (brw->primitive == _3DPRIM_RECTLIST)
         clip.ClipEnable = false;
      else
         clip.ClipEnable = true;

      /* _NEW_POLYGON,
       * BRW_NEW_GEOMETRY_PROGRAM | BRW_NEW_TES_PROG_DATA | BRW_NEW_PRIMITIVE
       */
      if (!brw_is_drawing_points(brw) && !brw_is_drawing_lines(brw))
         clip.ViewportXYClipTestEnable = true;

      clip.MinimumPointWidth = 0.125;
      clip.MaximumPointWidth = 255.875;
      clip.MaximumVPIndex = viewport_count - 1;
      if (_mesa_geometric_layers(fb) <= 0)
         clip.ForceZeroRTAIndexEnable = true;
   }
}

static const struct brw_tracked_state genX(clip_state) = {
   .dirty = {
      .mesa  = _NEW_BUFFERS |
               _NEW_LIGHT |
               _NEW_POLYGON |
               _NEW_TRANSFORM,
      .brw   = BRW_NEW_BLORP |
               BRW_NEW_CONTEXT |
               BRW_NEW_FS_PROG_DATA |
               BRW_NEW_GS_PROG_DATA |
               BRW_NEW_VS_PROG_DATA |
               BRW_NEW_META_IN_PROGRESS |
               BRW_NEW_PRIMITIVE |
               BRW_NEW_RASTERIZER_DISCARD |
               BRW_NEW_TES_PROG_DATA |
               BRW_NEW_VIEWPORT_COUNT,
   },
   .emit = genX(upload_clip_state),
};

/* ---------------------------------------------------------------------- */

#if GEN_GEN >= 8
static void
genX(upload_raster)(struct brw_context *brw)
{
   struct gl_context *ctx = &brw->ctx;

   /* _NEW_BUFFERS */
   bool render_to_fbo = _mesa_is_user_fbo(brw->ctx.DrawBuffer);

   /* _NEW_POLYGON */
   struct gl_polygon_attrib *polygon = &ctx->Polygon;

   /* _NEW_POINT */
   struct gl_point_attrib *point = &ctx->Point;

   brw_batch_emit(brw, GENX(3DSTATE_RASTER), raster) {
      if (polygon->_FrontBit == render_to_fbo)
         raster.FrontWinding = CounterClockwise;

      if (polygon->CullFlag) {
         switch (polygon->CullFaceMode) {
            case GL_FRONT:
               raster.CullMode = CULLMODE_FRONT;
               break;
            case GL_BACK:
               raster.CullMode = CULLMODE_BACK;
               break;
            case GL_FRONT_AND_BACK:
               raster.CullMode = CULLMODE_BOTH;
               break;
            default:
               unreachable("not reached");
         }
      } else {
         raster.CullMode = CULLMODE_NONE;
      }

      point->SmoothFlag = raster.SmoothPointEnable;

      raster.DXMultisampleRasterizationEnable =
         _mesa_is_multisample_enabled(ctx);

      raster.GlobalDepthOffsetEnableSolid = polygon->OffsetFill;
      raster.GlobalDepthOffsetEnableWireframe = polygon->OffsetLine;
      raster.GlobalDepthOffsetEnablePoint = polygon->OffsetPoint;

      switch (polygon->FrontMode) {
         case GL_FILL:
            raster.FrontFaceFillMode = FILL_MODE_SOLID;
            break;
         case GL_LINE:
            raster.FrontFaceFillMode = FILL_MODE_WIREFRAME;
            break;
         case GL_POINT:
            raster.FrontFaceFillMode = FILL_MODE_POINT;
            break;
         default:
            unreachable("not reached");
      }

      switch (polygon->BackMode) {
         case GL_FILL:
            raster.BackFaceFillMode = FILL_MODE_SOLID;
            break;
         case GL_LINE:
            raster.BackFaceFillMode = FILL_MODE_WIREFRAME;
            break;
         case GL_POINT:
            raster.BackFaceFillMode = FILL_MODE_POINT;
            break;
         default:
            unreachable("not reached");
      }

      /* _NEW_LINE */
      raster.AntialiasingEnable = ctx->Line.SmoothFlag;

      /* _NEW_SCISSOR */
      raster.ScissorRectangleEnable = ctx->Scissor.EnableFlags;

      /* _NEW_TRANSFORM */
      if (!ctx->Transform.DepthClamp) {
#if GEN_GEN >= 9
         raster.ViewportZFarClipTestEnable = true;
         raster.ViewportZNearClipTestEnable = true;
#else
         raster.ViewportZClipTestEnable = true;
#endif
      }

      /* BRW_NEW_CONSERVATIVE_RASTERIZATION */
#if GEN_GEN >= 9
      raster.ConservativeRasterizationEnable =
         ctx->IntelConservativeRasterization;
#endif

      raster.GlobalDepthOffsetClamp = polygon->OffsetClamp;
      raster.GlobalDepthOffsetScale = polygon->OffsetFactor;

      /* copied from gen4 */
      raster.GlobalDepthOffsetConstant = polygon->OffsetUnits * 2;
   }
}

static const struct brw_tracked_state genX(raster_state) = {
   .dirty = {
      .mesa  = _NEW_BUFFERS |
               _NEW_LINE |
               _NEW_MULTISAMPLE |
               _NEW_POINT |
               _NEW_POLYGON |
               _NEW_SCISSOR |
               _NEW_TRANSFORM,
      .brw   = BRW_NEW_BLORP |
               BRW_NEW_CONTEXT |
               BRW_NEW_CONSERVATIVE_RASTERIZATION,
   },
   .emit = genX(upload_raster),
};
#endif

/* ---------------------------------------------------------------------- */

static void
genX(upload_sf)(struct brw_context *brw)
{
   struct gl_context *ctx = &brw->ctx;
   float point_size;

#if GEN_GEN == 6
   /* BRW_NEW_FS_PROG_DATA */
   const struct brw_wm_prog_data *wm_prog_data =
      brw_wm_prog_data(brw->wm.base.prog_data);
   uint32_t num_outputs = wm_prog_data->num_varying_inputs;
#endif

#if GEN_GEN <= 7
   /* _NEW_BUFFERS */
   bool render_to_fbo = _mesa_is_user_fbo(ctx->DrawBuffer);
   const bool multisampled_fbo = _mesa_geometric_samples(ctx->DrawBuffer) > 1;
#endif

   brw_batch_emit(brw, GENX(3DSTATE_SF), sf) {
      sf.StatisticsEnable = true;
      sf.ViewportTransformEnable = brw->sf.viewport_transform_enable;

#if GEN_GEN <= 6
      sf.AttributeSwizzleEnable = true;
      sf.NumberofSFOutputAttributes = num_outputs;

      /*
       * Window coordinates in an FBO are inverted, which means point
       * sprite origin must be inverted, too.
       */
      if ((ctx->Point.SpriteOrigin == GL_LOWER_LEFT) != render_to_fbo) {
         sf.PointSpriteTextureCoordinateOrigin = LOWERLEFT;
      } else {
         sf.PointSpriteTextureCoordinateOrigin = UPPERLEFT;
      }
#endif

#if GEN_GEN == 7
      /* _NEW_BUFFERS */
      sf.DepthBufferSurfaceFormat = brw_depthbuffer_format(brw);
#endif

#if GEN_GEN <= 7
      /* _NEW_POLYGON */
      if (ctx->Polygon._FrontBit == render_to_fbo)
         sf.FrontWinding = 1;
      sf.GlobalDepthOffsetEnableSolid = ctx->Polygon.OffsetFill;
      sf.GlobalDepthOffsetEnableWireframe = ctx->Polygon.OffsetLine;
      sf.GlobalDepthOffsetEnablePoint = ctx->Polygon.OffsetPoint;

      switch (ctx->Polygon.FrontMode) {
         case GL_FILL:
            sf.FrontFaceFillMode = FILL_MODE_SOLID;
            break;
         case GL_LINE:
            sf.FrontFaceFillMode = FILL_MODE_WIREFRAME;
            break;
         case GL_POINT:
            sf.FrontFaceFillMode = FILL_MODE_POINT;
            break;
         default:
            unreachable("not reached");
      }

      switch (ctx->Polygon.BackMode) {
         case GL_FILL:
            sf.BackFaceFillMode = FILL_MODE_SOLID;
            break;
         case GL_LINE:
            sf.BackFaceFillMode = FILL_MODE_WIREFRAME;
            break;
         case GL_POINT:
            sf.BackFaceFillMode = FILL_MODE_POINT;
            break;
         default:
            unreachable("not reached");
      }

      sf.ScissorRectangleEnable = true;

      if (ctx->Polygon.CullFlag) {
         switch (ctx->Polygon.CullFaceMode) {
            case GL_FRONT:
               sf.CullMode = CULLMODE_FRONT;
               break;
            case GL_BACK:
               sf.CullMode = CULLMODE_BACK;
               break;
            case GL_FRONT_AND_BACK:
               sf.CullMode = CULLMODE_BOTH;
               break;
            default:
               unreachable("not reached");
         }
      } else {
         sf.CullMode = CULLMODE_NONE;
      }

#if GEN_IS_HASWELL
      sf.LineStippleEnable = ctx->Line.StippleFlag;
#endif

      if (multisampled_fbo && ctx->Multisample.Enabled)
         sf.MultisampleRasterizationMode = MSRASTMODE_ON_PATTERN;

      /* copied from gen4 */
      sf.GlobalDepthOffsetConstant = ctx->Polygon.OffsetUnits * 2;
      sf.GlobalDepthOffsetScale = ctx->Polygon.OffsetFactor;
      sf.GlobalDepthOffsetClamp = ctx->Polygon.OffsetClamp;
#endif

      /* _NEW_LINE */
      /* TODO: if (brw->is_cherryview) */
      float line_width = brw_get_line_width_float(brw);
      sf.LineWidth = line_width;

      if (ctx->Line.SmoothFlag) {
         sf.LineEndCapAntialiasingRegionWidth = _10pixels;
#if GEN_GEN <= 7
         sf.AntiAliasingEnable = true;
#endif
      }

      /* _NEW_POINT - Clamp to ARB_point_parameters user limits */
      point_size = CLAMP(ctx->Point.Size, ctx->Point.MinSize, ctx->Point.MaxSize);
      /* Clamp to the hardware limits */
      sf.PointWidth = CLAMP(point_size, 0.125f, 255.875f);

      /* _NEW_PROGRAM | _NEW_POINT, BRW_NEW_VUE_MAP_GEOM_OUT */
      if (use_state_point_size(brw))
         sf.PointWidthSource = State;

#if GEN_GEN >= 8
      /* _NEW_POINT | _NEW_MULTISAMPLE */
      if ((ctx->Point.SmoothFlag || _mesa_is_multisample_enabled(ctx)) &&
          !ctx->Point.PointSprite)
         sf.SmoothPointEnable = true;
#endif

#if GEN_GEN < 7
      if (ctx->Line.SmoothFlag)
#endif
         sf.AALineDistanceMode = AALINEDISTANCE_TRUE;

      /* _NEW_LIGHT */
      if (ctx->Light.ProvokingVertex != GL_FIRST_VERTEX_CONVENTION) {
         sf.TriangleStripListProvokingVertexSelect = 2;
         sf.TriangleFanProvokingVertexSelect = 2;
         sf.LineStripListProvokingVertexSelect = 1;
      } else {
         sf.TriangleFanProvokingVertexSelect = 1;
      }

#if GEN_GEN == 6
      /* BRW_NEW_VUE_MAP_GEOM_OUT | BRW_NEW_FRAGMENT_PROGRAM |
       * _NEW_POINT | _NEW_LIGHT | _NEW_PROGRAM | BRW_NEW_FS_PROG_DATA
       */
      uint32_t urb_entry_read_length;
      uint32_t urb_entry_read_offset;
      uint32_t point_sprite_enables;
      genX(calculate_attr_overrides)(brw, sf.Attribute, &point_sprite_enables,
                                     &urb_entry_read_length,
                                     &urb_entry_read_offset);
      sf.VertexURBEntryReadLength = urb_entry_read_length;
      sf.VertexURBEntryReadOffset = urb_entry_read_offset;
      sf.PointSpriteTextureCoordinateEnable = point_sprite_enables;
      sf.ConstantInterpolationEnable = wm_prog_data->flat_inputs;
#endif
   }
}

static const struct brw_tracked_state genX(sf_state) = {
   .dirty = {
      .mesa  = _NEW_LIGHT |
               _NEW_PROGRAM |
               _NEW_LINE |
               _NEW_MULTISAMPLE |
               _NEW_POINT |
               (GEN_GEN <= 7 ? _NEW_BUFFERS | _NEW_POLYGON : 0),
      .brw   = BRW_NEW_BLORP |
               BRW_NEW_CONTEXT |
               BRW_NEW_VUE_MAP_GEOM_OUT |
               (GEN_GEN <= 7 ? BRW_NEW_GS_PROG_DATA |
                               BRW_NEW_PRIMITIVE |
                               BRW_NEW_TES_PROG_DATA
                             : 0) |
               (GEN_GEN <= 6 ? BRW_NEW_FS_PROG_DATA |
                               BRW_NEW_FRAGMENT_PROGRAM
                             : 0),
   },
   .emit = genX(upload_sf),
};

/* ---------------------------------------------------------------------- */

#if GEN_GEN >= 7
static void
genX(upload_sbe)(struct brw_context *brw)
{
   struct gl_context *ctx = &brw->ctx;
   /* BRW_NEW_FS_PROG_DATA */
   const struct brw_wm_prog_data *wm_prog_data =
      brw_wm_prog_data(brw->wm.base.prog_data);
   uint32_t num_outputs = wm_prog_data->num_varying_inputs;
#if GEN_GEN >= 8
   struct GENX(SF_OUTPUT_ATTRIBUTE_DETAIL) attr_overrides[16];
#endif
   uint32_t urb_entry_read_length;
   uint32_t urb_entry_read_offset;
   uint32_t point_sprite_enables;

   brw_batch_emit(brw, GENX(3DSTATE_SBE), sbe) {
      sbe.AttributeSwizzleEnable = true;
      sbe.NumberofSFOutputAttributes = num_outputs;

      /* _NEW_BUFFERS */
      bool render_to_fbo = _mesa_is_user_fbo(ctx->DrawBuffer);

      /* _NEW_POINT
       *
       * Window coordinates in an FBO are inverted, which means point
       * sprite origin must be inverted.
       */
      if ((ctx->Point.SpriteOrigin == GL_LOWER_LEFT) != render_to_fbo)
         sbe.PointSpriteTextureCoordinateOrigin = LOWERLEFT;
      else
         sbe.PointSpriteTextureCoordinateOrigin = UPPERLEFT;

      /* _NEW_POINT | _NEW_LIGHT | _NEW_PROGRAM,
       * BRW_NEW_FS_PROG_DATA | BRW_NEW_FRAGMENT_PROGRAM |
       * BRW_NEW_GS_PROG_DATA | BRW_NEW_PRIMITIVE | BRW_NEW_TES_PROG_DATA |
       * BRW_NEW_VUE_MAP_GEOM_OUT
       */
#if GEN_GEN < 8
      genX(calculate_attr_overrides)(brw,
                                     sbe.Attribute,
                                     &point_sprite_enables,
                                     &urb_entry_read_length,
                                     &urb_entry_read_offset);
#else
      memset(attr_overrides, 0, sizeof(attr_overrides));
      genX(calculate_attr_overrides)(brw,
                                     attr_overrides,
                                     &point_sprite_enables,
                                     &urb_entry_read_length,
                                     &urb_entry_read_offset);
#endif

      /* Typically, the URB entry read length and offset should be programmed
       * in 3DSTATE_VS and 3DSTATE_GS; SBE inherits it from the last active
       * stage which produces geometry.  However, we don't know the proper
       * value until we call calculate_attr_overrides().
       *
       * To fit with our existing code, we override the inherited values and
       * specify it here directly, as we did on previous generations.
       */
      sbe.VertexURBEntryReadLength = urb_entry_read_length;
      sbe.VertexURBEntryReadOffset = urb_entry_read_offset;
      sbe.PointSpriteTextureCoordinateEnable = point_sprite_enables;
      sbe.ConstantInterpolationEnable = wm_prog_data->flat_inputs;

#if GEN_GEN >= 8
      sbe.ForceVertexURBEntryReadLength = true;
      sbe.ForceVertexURBEntryReadOffset = true;
#endif

#if GEN_GEN >= 9
      /* prepare the active component dwords */
      int input_index = 0;
      for (int attr = 0; attr < VARYING_SLOT_MAX; attr++) {
         if (!(brw->fragment_program->info.inputs_read &
               BITFIELD64_BIT(attr))) {
            continue;
         }

         assert(input_index < 32);

         /* TODO: Should this be in gen9.xml? GEN9_SBE_ACTIVE_COMPONENT_XYZW */
         sbe.AttributeActiveComponentFormat[input_index] = 3;
         ++input_index;
      }
#endif
   }

#if GEN_GEN >= 8
   brw_batch_emit(brw, GENX(3DSTATE_SBE_SWIZ), sbes) {
      for (int i = 0; i < 16; i++)
         sbes.Attribute[i] = attr_overrides[i];
   }
#endif
}

static const struct brw_tracked_state genX(sbe_state) = {
   .dirty = {
      .mesa  = _NEW_BUFFERS |
               _NEW_LIGHT |
               _NEW_POINT |
               _NEW_POLYGON |
               _NEW_PROGRAM,
      .brw   = BRW_NEW_BLORP |
               BRW_NEW_CONTEXT |
               BRW_NEW_FRAGMENT_PROGRAM |
               BRW_NEW_FS_PROG_DATA |
               BRW_NEW_GS_PROG_DATA |
               BRW_NEW_TES_PROG_DATA |
               BRW_NEW_VUE_MAP_GEOM_OUT |
               (GEN_GEN == 7 ? BRW_NEW_PRIMITIVE
                             : 0),
   },
   .emit = genX(upload_sbe),
};
#endif

/* ---------------------------------------------------------------------- */

#if GEN_GEN >= 7

/**
 * Outputs the 3DSTATE_SO_DECL_LIST command.
 *
 * The data output is a series of 64-bit entries containing a SO_DECL per
 * stream.  We only have one stream of rendering coming out of the GS unit, so
 * we only emit stream 0 (low 16 bits) SO_DECLs.
 */
static void
genX(upload_3dstate_so_decl_list)(struct brw_context *brw,
                                  const struct brw_vue_map *vue_map)
{
   struct gl_context *ctx = &brw->ctx;
   /* BRW_NEW_TRANSFORM_FEEDBACK */
   struct gl_transform_feedback_object *xfb_obj =
      ctx->TransformFeedback.CurrentObject;
   const struct gl_transform_feedback_info *linked_xfb_info =
      xfb_obj->program->sh.LinkedTransformFeedback;
   struct GENX(SO_DECL) so_decl[MAX_VERTEX_STREAMS][128];
   int buffer_mask[MAX_VERTEX_STREAMS] = {0, 0, 0, 0};
   int next_offset[MAX_VERTEX_STREAMS] = {0, 0, 0, 0};
   int decls[MAX_VERTEX_STREAMS] = {0, 0, 0, 0};
   int max_decls = 0;
   STATIC_ASSERT(ARRAY_SIZE(so_decl[0]) >= MAX_PROGRAM_OUTPUTS);

   memset(so_decl, 0, sizeof(so_decl));

   /* Construct the list of SO_DECLs to be emitted.  The formatting of the
    * command feels strange -- each dword pair contains a SO_DECL per stream.
    */
   for (unsigned i = 0; i < linked_xfb_info->NumOutputs; i++) {
      int buffer = linked_xfb_info->Outputs[i].OutputBuffer;
      struct GENX(SO_DECL) decl = {0};
      int varying = linked_xfb_info->Outputs[i].OutputRegister;
      const unsigned components = linked_xfb_info->Outputs[i].NumComponents;
      unsigned component_mask = (1 << components) - 1;
      unsigned stream_id = linked_xfb_info->Outputs[i].StreamId;
      unsigned decl_buffer_slot = buffer;
      assert(stream_id < MAX_VERTEX_STREAMS);

      /* gl_PointSize is stored in VARYING_SLOT_PSIZ.w
       * gl_Layer is stored in VARYING_SLOT_PSIZ.y
       * gl_ViewportIndex is stored in VARYING_SLOT_PSIZ.z
       */
      if (varying == VARYING_SLOT_PSIZ) {
         assert(components == 1);
         component_mask <<= 3;
      } else if (varying == VARYING_SLOT_LAYER) {
         assert(components == 1);
         component_mask <<= 1;
      } else if (varying == VARYING_SLOT_VIEWPORT) {
         assert(components == 1);
         component_mask <<= 2;
      } else {
         component_mask <<= linked_xfb_info->Outputs[i].ComponentOffset;
      }

      buffer_mask[stream_id] |= 1 << buffer;

      decl.OutputBufferSlot = decl_buffer_slot;
      if (varying == VARYING_SLOT_LAYER || varying == VARYING_SLOT_VIEWPORT) {
         decl.RegisterIndex = vue_map->varying_to_slot[VARYING_SLOT_PSIZ];
      } else {
         assert(vue_map->varying_to_slot[varying] >= 0);
         decl.RegisterIndex = vue_map->varying_to_slot[varying];
      }
      decl.ComponentMask = component_mask;

      /* Mesa doesn't store entries for gl_SkipComponents in the Outputs[]
       * array.  Instead, it simply increments DstOffset for the following
       * input by the number of components that should be skipped.
       *
       * Our hardware is unusual in that it requires us to program SO_DECLs
       * for fake "hole" components, rather than simply taking the offset
       * for each real varying.  Each hole can have size 1, 2, 3, or 4; we
       * program as many size = 4 holes as we can, then a final hole to
       * accommodate the final 1, 2, or 3 remaining.
       */
      int skip_components =
         linked_xfb_info->Outputs[i].DstOffset - next_offset[buffer];

      next_offset[buffer] += skip_components;

      while (skip_components >= 4) {
         struct GENX(SO_DECL) *d = &so_decl[stream_id][decls[stream_id]++];
         d->HoleFlag = 1;
         d->OutputBufferSlot = decl_buffer_slot;
         d->ComponentMask = 0xf;
         skip_components -= 4;
      }

      if (skip_components > 0) {
         struct GENX(SO_DECL) *d = &so_decl[stream_id][decls[stream_id]++];
         d->HoleFlag = 1;
         d->OutputBufferSlot = decl_buffer_slot;
         d->ComponentMask = (1 << skip_components) - 1;
      }

      assert(linked_xfb_info->Outputs[i].DstOffset == next_offset[buffer]);

      next_offset[buffer] += components;

      so_decl[stream_id][decls[stream_id]++] = decl;

      if (decls[stream_id] > max_decls)
         max_decls = decls[stream_id];
   }

   uint32_t *dw;
   dw = brw_batch_emitn(brw, GENX(3DSTATE_SO_DECL_LIST), 3 + 2 * max_decls,
                        .StreamtoBufferSelects0 = buffer_mask[0],
                        .StreamtoBufferSelects1 = buffer_mask[1],
                        .StreamtoBufferSelects2 = buffer_mask[2],
                        .StreamtoBufferSelects3 = buffer_mask[3],
                        .NumEntries0 = decls[0],
                        .NumEntries1 = decls[1],
                        .NumEntries2 = decls[2],
                        .NumEntries3 = decls[3]);

   for (int i = 0; i < max_decls; i++) {
      GENX(SO_DECL_ENTRY_pack)(
         brw, dw + 2 + i * 2,
         &(struct GENX(SO_DECL_ENTRY)) {
            .Stream0Decl = so_decl[0][i],
            .Stream1Decl = so_decl[1][i],
            .Stream2Decl = so_decl[2][i],
            .Stream3Decl = so_decl[3][i],
         });
   }
}

static void
genX(upload_3dstate_so_buffers)(struct brw_context *brw)
{
   struct gl_context *ctx = &brw->ctx;
   /* BRW_NEW_TRANSFORM_FEEDBACK */
   struct gl_transform_feedback_object *xfb_obj =
      ctx->TransformFeedback.CurrentObject;
#if GEN_GEN < 8
   const struct gl_transform_feedback_info *linked_xfb_info =
      xfb_obj->program->sh.LinkedTransformFeedback;
#else
   struct brw_transform_feedback_object *brw_obj =
      (struct brw_transform_feedback_object *) xfb_obj;
   /* Copy these values from brw_defines.h so we don't have to include the whole
    * file.
    */
#define SKL_MOCS_WB (2 << 1)
#define BDW_MOCS_WB 0x78
   uint32_t mocs_wb = brw->gen >= 9 ? SKL_MOCS_WB : BDW_MOCS_WB;
#endif

   /* Set up the up to 4 output buffers.  These are the ranges defined in the
    * gl_transform_feedback_object.
    */
   for (int i = 0; i < 4; i++) {
      struct intel_buffer_object *bufferobj =
         intel_buffer_object(xfb_obj->Buffers[i]);

      if (!bufferobj) {
         brw_batch_emit(brw, GENX(3DSTATE_SO_BUFFER), sob) {
            sob.SOBufferIndex = i;
         }
         continue;
      }

      uint32_t start = xfb_obj->Offset[i];
      assert(start % 4 == 0);
      uint32_t end = ALIGN(start + xfb_obj->Size[i], 4);
      drm_intel_bo *bo =
         intel_bufferobj_buffer(brw, bufferobj, start, end - start);
      assert(end <= bo->size);

      brw_batch_emit(brw, GENX(3DSTATE_SO_BUFFER), sob) {
         sob.SOBufferIndex = i;

         sob.SurfaceBaseAddress.bo = bo;
         sob.SurfaceBaseAddress.read_domains = I915_GEM_DOMAIN_RENDER;
         sob.SurfaceBaseAddress.write_domain = I915_GEM_DOMAIN_RENDER;
         sob.SurfaceBaseAddress.offset = start;
#if GEN_GEN < 8
         sob.SurfacePitch = linked_xfb_info->Buffers[i].Stride * 4;
         sob.SurfaceEndAddress.bo = bo;
         sob.SurfaceEndAddress.read_domains = I915_GEM_DOMAIN_RENDER;
         sob.SurfaceEndAddress.write_domain = I915_GEM_DOMAIN_RENDER;
         sob.SurfaceEndAddress.offset = end;
#else
         sob.SOBufferEnable = true;
         sob.StreamOffsetWriteEnable = true;
         sob.StreamOutputBufferOffsetAddressEnable = true;
         sob.MOCS = mocs_wb;

         sob.SurfaceSize = xfb_obj->Size[i] / 4;
         if (sob.SurfaceSize > 0)
            sob.SurfaceSize -= 1;
         sob.StreamOutputBufferOffsetAddress.bo = brw_obj->offset_bo;
         sob.StreamOutputBufferOffsetAddress.read_domains =
            I915_GEM_DOMAIN_INSTRUCTION;
         sob.StreamOutputBufferOffsetAddress.write_domain =
            I915_GEM_DOMAIN_INSTRUCTION;
         sob.StreamOutputBufferOffsetAddress.offset = i * sizeof(uint32_t);

         if (brw_obj->zero_offsets)
            /* Zero out the offset and write that to offset_bo */
            sob.StreamOffset = 0;
         else
            /* Use offset_bo as the "Stream Offset." */
            sob.StreamOffset = 0xFFFFFFFF;
#endif
      }
   }

#if GEN_GEN >= 8
   brw_obj->zero_offsets = false;
#endif
}

static bool
genX(query_active)(struct gl_query_object *q)
{
   return q && q->Active;
}

static void
genX(upload_3dstate_streamout)(struct brw_context *brw, bool active,
                               const struct brw_vue_map *vue_map)
{
   struct gl_context *ctx = &brw->ctx;
   /* BRW_NEW_TRANSFORM_FEEDBACK */
   struct gl_transform_feedback_object *xfb_obj =
      ctx->TransformFeedback.CurrentObject;

   brw_batch_emit(brw, GENX(3DSTATE_STREAMOUT), sos) {
      if (active) {
#if GEN_GEN >= 8
         const struct gl_transform_feedback_info *linked_xfb_info =
            xfb_obj->program->sh.LinkedTransformFeedback;
#endif
         int urb_entry_read_offset = 0;
         int urb_entry_read_length = (vue_map->num_slots + 1) / 2 -
            urb_entry_read_offset;

         sos.SOFunctionEnable = true;
         sos.SOStatisticsEnable = true;

         /* BRW_NEW_RASTERIZER_DISCARD */
         if (ctx->RasterDiscard) {
            if (!genX(query_active)(ctx->Query.PrimitivesGenerated[0])) {
               sos.RenderingDisable = true;
            } else {
               perf_debug("Rasterizer discard with a GL_PRIMITIVES_GENERATED "
                          "query active relies on the clipper.");
            }
         }

         /* _NEW_LIGHT */
         if (ctx->Light.ProvokingVertex != GL_FIRST_VERTEX_CONVENTION)
            sos.ReorderMode = TRAILING;

#if GEN_GEN < 8
         if (brw->gen < 8) {
            if (xfb_obj->Buffers[0])
               sos.SOBufferEnable0 = true;
            if (xfb_obj->Buffers[1])
               sos.SOBufferEnable1 = true;
            if (xfb_obj->Buffers[2])
               sos.SOBufferEnable2 = true;
            if (xfb_obj->Buffers[3])
               sos.SOBufferEnable3 = true;
         }
#endif

         /* We always read the whole vertex.  This could be reduced at some
          * point by reading less and offsetting the register index in the
          * SO_DECLs.
          */
         sos.Stream0VertexReadOffset = urb_entry_read_offset;
         sos.Stream0VertexReadLength = urb_entry_read_length - 1;
         sos.Stream1VertexReadOffset = urb_entry_read_offset;
         sos.Stream1VertexReadLength = urb_entry_read_length - 1;
         sos.Stream2VertexReadOffset = urb_entry_read_offset;
         sos.Stream2VertexReadLength = urb_entry_read_length - 1;
         sos.Stream3VertexReadOffset = urb_entry_read_offset;
         sos.Stream3VertexReadLength = urb_entry_read_length - 1;

#if GEN_GEN >= 8
         /* Set buffer pitches; 0 means unbound. */
         if (xfb_obj->Buffers[0])
            sos.Buffer0SurfacePitch = linked_xfb_info->Buffers[0].Stride * 4;
         if (xfb_obj->Buffers[1])
            sos.Buffer1SurfacePitch = linked_xfb_info->Buffers[1].Stride * 4;
         if (xfb_obj->Buffers[2])
            sos.Buffer2SurfacePitch = linked_xfb_info->Buffers[2].Stride * 4;
         if (xfb_obj->Buffers[3])
            sos.Buffer3SurfacePitch = linked_xfb_info->Buffers[3].Stride * 4;
#endif
      }
   }
}

static void
genX(upload_sol)(struct brw_context *brw)
{
   struct gl_context *ctx = &brw->ctx;
   /* BRW_NEW_TRANSFORM_FEEDBACK */
   bool active = _mesa_is_xfb_active_and_unpaused(ctx);

   if (active) {
      genX(upload_3dstate_so_buffers)(brw);

      /* BRW_NEW_VUE_MAP_GEOM_OUT */
      genX(upload_3dstate_so_decl_list)(brw, &brw->vue_map_geom_out);
   }

   /* Finally, set up the SOL stage.  This command must always follow updates to
    * the nonpipelined SOL state (3DSTATE_SO_BUFFER, 3DSTATE_SO_DECL_LIST) or
    * MMIO register updates (current performed by the kernel at each batch
    * emit).
    */
   genX(upload_3dstate_streamout)(brw, active, &brw->vue_map_geom_out);
}

static const struct brw_tracked_state genX(sol_state) = {
   .dirty = {
      .mesa  = _NEW_LIGHT,
      .brw   = BRW_NEW_BATCH |
               BRW_NEW_BLORP |
               BRW_NEW_RASTERIZER_DISCARD |
               BRW_NEW_VUE_MAP_GEOM_OUT |
               BRW_NEW_TRANSFORM_FEEDBACK,
   },
   .emit = genX(upload_sol),
};
#endif

/* ---------------------------------------------------------------------- */

#if GEN_GEN >= 7
static void
genX(upload_ps)(struct brw_context *brw)
{
   /* BRW_NEW_FS_PROG_DATA */
   const struct brw_wm_prog_data *prog_data =
      brw_wm_prog_data(brw->wm.base.prog_data);
   const struct brw_stage_state *stage_state = &brw->wm.base;
#if GEN_GEN < 8
   const struct gl_context *ctx = &brw->ctx;
   /* BRW_NEW_FS_PROG_DATA | _NEW_COLOR */
   const bool enable_dual_src_blend = prog_data->dual_src_blend &&
                                      (ctx->Color.BlendEnabled & 1) &&
                                      ctx->Color.Blend[0]._UsesDualSrc;
   const struct gen_device_info *devinfo = &brw->screen->devinfo;
#endif

   brw_batch_emit(brw, GENX(3DSTATE_PS), ps) {
      /* Initialize the execution mask with VMask.  Otherwise, derivatives are
       * incorrect for subspans where some of the pixels are unlit.  We believe
       * the bit just didn't take effect in previous generations.
       */
#if GEN_GEN >= 8
      ps.VectorMaskEnable = true;
#endif
      ps.SamplerCount =
         DIV_ROUND_UP(CLAMP(stage_state->sampler_count, 0, 16), 4);

      /* BRW_NEW_FS_PROG_DATA */
      ps.BindingTableEntryCount = prog_data->base.binding_table.size_bytes / 4;

      if (prog_data->base.use_alt_mode)
         ps.FloatingPointMode = Alternate;

      /* Haswell requires the sample mask to be set in this packet as well as
       * in 3DSTATE_SAMPLE_MASK; the values should match. */
      /* _NEW_BUFFERS, _NEW_MULTISAMPLE */
#if GEN_IS_HASWELL
      ps.SampleMask = gen6_determine_sample_mask(brw);
#endif

      /* 3DSTATE_PS expects the number of threads per PSD, which is always 64;
       * it implicitly scales for different GT levels (which have some # of
       * PSDs).
       *
       * In Gen8 the format is U8-2 whereas in Gen9 it is U8-1.
       */
#if GEN_GEN >= 9
      ps.MaximumNumberofThreadsPerPSD = 64 - 1;
#elif GEN_GEN >= 8
      ps.MaximumNumberofThreadsPerPSD = 64 - 2;
#else
      ps.MaximumNumberofThreads = devinfo->max_wm_threads - 1;
#endif

      if (prog_data->base.nr_params > 0)
         ps.PushConstantEnable = true;

#if GEN_GEN < 8
      /* From the IVB PRM, volume 2 part 1, page 287:
       * "This bit is inserted in the PS payload header and made available to
       * the DataPort (either via the message header or via header bypass) to
       * indicate that oMask data (one or two phases) is included in Render
       * Target Write messages. If present, the oMask data is used to mask off
       * samples."
       */
      ps.oMaskPresenttoRenderTarget = prog_data->uses_omask;

      /* The hardware wedges if you have this bit set but don't turn on any dual
       * source blend factors.
       */
      ps.DualSourceBlendEnable = enable_dual_src_blend;

      /* BRW_NEW_FS_PROG_DATA */
      ps.AttributeEnable = (prog_data->num_varying_inputs != 0);
#endif

      /* From the documentation for this packet:
       * "If the PS kernel does not need the Position XY Offsets to
       *  compute a Position Value, then this field should be programmed
       *  to POSOFFSET_NONE."
       *
       * "SW Recommendation: If the PS kernel needs the Position Offsets
       *  to compute a Position XY value, this field should match Position
       *  ZW Interpolation Mode to ensure a consistent position.xyzw
       *  computation."
       *
       * We only require XY sample offsets. So, this recommendation doesn't
       * look useful at the moment. We might need this in future.
       */
      if (prog_data->uses_pos_offset)
         ps.PositionXYOffsetSelect = POSOFFSET_SAMPLE;
      else
         ps.PositionXYOffsetSelect = POSOFFSET_NONE;

      ps.RenderTargetFastClearEnable = brw->wm.fast_clear_op;
      ps._8PixelDispatchEnable = !!prog_data->dispatch_8;
      ps._16PixelDispatchEnable = !!prog_data->dispatch_16;
      ps.DispatchGRFStartRegisterForConstantSetupData0 =
         prog_data->base.dispatch_grf_start_reg;
      ps.DispatchGRFStartRegisterForConstantSetupData2 =
         prog_data->dispatch_grf_start_reg_2;

      ps.KernelStartPointer0 = stage_state->prog_offset;
      ps.KernelStartPointer2 = stage_state->prog_offset +
         prog_data->prog_offset_2;

      if (prog_data->base.total_scratch) {
         ps.ScratchSpaceBasePointer.bo = stage_state->scratch_bo;
         ps.ScratchSpaceBasePointer.read_domains = I915_GEM_DOMAIN_RENDER;
         ps.ScratchSpaceBasePointer.write_domain = I915_GEM_DOMAIN_RENDER;
         ps.ScratchSpaceBasePointer.offset =
            ffs(stage_state->per_thread_scratch) - 11;
      }
   }
}

static const struct brw_tracked_state genX(ps_state) = {
   .dirty = {
      .mesa  = _NEW_MULTISAMPLE |
               (GEN_GEN < 8 ? _NEW_BUFFERS |
                              _NEW_COLOR
                            : 0),
      .brw   = BRW_NEW_BATCH |
               BRW_NEW_BLORP |
               BRW_NEW_FS_PROG_DATA,
   },
   .emit = genX(upload_ps),
};
#endif

/* ---------------------------------------------------------------------- */

static void
genX(upload_wm)(struct brw_context *brw)
{
   struct gl_context *ctx = &brw->ctx;

   /* BRW_NEW_FS_PROG_DATA */
   const struct brw_wm_prog_data *wm_prog_data =
      brw_wm_prog_data(brw->wm.base.prog_data);
#if GEN_GEN < 8
   /* TODO: Should it be BRW_PSCDEPTH_OFF ? */
   bool writes_depth = wm_prog_data->computed_depth_mode != 0;
   /* _NEW_BUFFERS */
   const bool multisampled_fbo = _mesa_geometric_samples(ctx->DrawBuffer) > 1;
#endif

#if GEN_GEN < 7
   const struct brw_stage_state *stage_state = &brw->wm.base;
   const bool enable_dual_src_blend = wm_prog_data->dual_src_blend &&
                                      (ctx->Color.BlendEnabled & 1) &&
                                      ctx->Color.Blend[0]._UsesDualSrc;
   const struct gen_device_info *devinfo = &brw->screen->devinfo;

   /* We can't fold this into gen6_upload_wm_push_constants(), because
    * according to the SNB PRM, vol 2 part 1 section 7.2.2
    * (3DSTATE_CONSTANT_PS [DevSNB]):
    *
    *     "[DevSNB]: This packet must be followed by WM_STATE."
    */
   brw_batch_emit(brw, GENX(3DSTATE_CONSTANT_PS), wmcp) {
      if (wm_prog_data->base.nr_params != 0) {
         wmcp.Buffer0Valid = true;
         /* Pointer to the WM constant buffer.  Covered by the set of
          * state flags from gen6_upload_wm_push_constants.
          */
         wmcp.PointertoPSConstantBuffer0 = stage_state->push_const_offset;
         wmcp.PSConstantBuffer0ReadLength = stage_state->push_const_size - 1;
      }
   }
#endif

   brw_batch_emit(brw, GENX(3DSTATE_WM), wm) {
      wm.StatisticsEnable = true;
      wm.LineAntialiasingRegionWidth = _10pixels;
      wm.LineEndCapAntialiasingRegionWidth = _05pixels;

#if GEN_GEN < 7
      if (wm_prog_data->base.use_alt_mode)
         wm.FloatingPointMode = Alternate;

      wm.SamplerCount |= ALIGN(stage_state->sampler_count, 4) / 4;
      wm.BindingTableEntryCount = wm_prog_data->base.binding_table.size_bytes / 4;
      wm.MaximumNumberofThreads = devinfo->max_wm_threads - 1;
      wm._8PixelDispatchEnable = !!wm_prog_data->dispatch_8;
      wm._16PixelDispatchEnable = !!wm_prog_data->dispatch_16;
      wm.DispatchGRFStartRegisterForConstantSetupData0 =
         wm_prog_data->base.dispatch_grf_start_reg;
      wm.DispatchGRFStartRegisterForConstantSetupData2 =
         wm_prog_data->dispatch_grf_start_reg_2;
      wm.KernelStartPointer0 = stage_state->prog_offset;
      wm.KernelStartPointer2 = stage_state->prog_offset +
         wm_prog_data->prog_offset_2;
      wm.DualSourceBlendEnable = enable_dual_src_blend;
      wm.oMaskPresenttoRenderTarget = wm_prog_data->uses_omask;
      wm.NumberofSFOutputAttributes = wm_prog_data->num_varying_inputs;

      /* From the SNB PRM, volume 2 part 1, page 281:
       * "If the PS kernel does not need the Position XY Offsets
       * to compute a Position XY value, then this field should be
       * programmed to POSOFFSET_NONE."
       *
       * "SW Recommendation: If the PS kernel needs the Position Offsets
       * to compute a Position XY value, this field should match Position
       * ZW Interpolation Mode to ensure a consistent position.xyzw
       * computation."
       * We only require XY sample offsets. So, this recommendation doesn't
       * look useful at the moment. We might need this in future.
       */
      if (wm_prog_data->uses_pos_offset)
         wm.PositionXYOffsetSelect = POSOFFSET_SAMPLE;
      else
         wm.PositionXYOffsetSelect = POSOFFSET_NONE;

      if (wm_prog_data->base.total_scratch) {
         wm.ScratchSpaceBasePointer.bo = stage_state->scratch_bo;
         wm.ScratchSpaceBasePointer.read_domains = I915_GEM_DOMAIN_RENDER;
         wm.ScratchSpaceBasePointer.write_domain = I915_GEM_DOMAIN_RENDER;
         wm.ScratchSpaceBasePointer.offset =
            ffs(stage_state->per_thread_scratch) - 11;
      }

      wm.PixelShaderComputedDepth = writes_depth;
#endif

#if GEN_GEN >= 8
      wm.PointRasterizationRule = RASTRULE_UPPER_RIGHT;
#endif

      /* _NEW_LINE */
      wm.LineStippleEnable = ctx->Line.StippleFlag;

      /* _NEW_POLYGON */
      wm.PolygonStippleEnable = ctx->Polygon.StippleFlag;
      wm.BarycentricInterpolationMode = wm_prog_data->barycentric_interp_modes;

#if GEN_GEN < 8
      wm.PixelShaderUsesSourceDepth = wm_prog_data->uses_src_depth;
      wm.PixelShaderUsesSourceW = wm_prog_data->uses_src_w;
      if (wm_prog_data->uses_kill ||
          _mesa_is_alpha_test_enabled(ctx) ||
          _mesa_is_alpha_to_coverage_enabled(ctx) ||
          wm_prog_data->uses_omask) {
         wm.PixelShaderKillsPixel = true;
      }

      /* _NEW_BUFFERS | _NEW_COLOR */
      if (brw_color_buffer_write_enabled(brw) || writes_depth ||
          wm_prog_data->has_side_effects || wm.PixelShaderKillsPixel) {
         wm.ThreadDispatchEnable = true;
      }
      if (multisampled_fbo) {
         /* _NEW_MULTISAMPLE */
         if (ctx->Multisample.Enabled)
            wm.MultisampleRasterizationMode = MSRASTMODE_ON_PATTERN;
         else
            wm.MultisampleRasterizationMode = MSRASTMODE_OFF_PIXEL;

         if (wm_prog_data->persample_dispatch)
            wm.MultisampleDispatchMode = MSDISPMODE_PERSAMPLE;
         else
            wm.MultisampleDispatchMode = MSDISPMODE_PERPIXEL;
      } else {
         wm.MultisampleRasterizationMode = MSRASTMODE_OFF_PIXEL;
         wm.MultisampleDispatchMode = MSDISPMODE_PERSAMPLE;
      }

#if GEN_GEN >= 7
      wm.PixelShaderComputedDepthMode = wm_prog_data->computed_depth_mode;
      wm.PixelShaderUsesInputCoverageMask = wm_prog_data->uses_sample_mask;
#endif

      /* The "UAV access enable" bits are unnecessary on HSW because they only
       * seem to have an effect on the HW-assisted coherency mechanism which we
       * don't need, and the rasterization-related UAV_ONLY flag and the
       * DISPATCH_ENABLE bit can be set independently from it.
       * C.f. gen8_upload_ps_extra().
       *
       * BRW_NEW_FRAGMENT_PROGRAM | BRW_NEW_FS_PROG_DATA | _NEW_BUFFERS |
       * _NEW_COLOR
       */
#if GEN_IS_HASWELL
      if (!(brw_color_buffer_write_enabled(brw) || writes_depth) &&
          wm_prog_data->has_side_effects)
         wm.PSUAVonly = ON;
#endif
#endif

#if GEN_GEN >= 7
      /* BRW_NEW_FS_PROG_DATA */
      if (wm_prog_data->early_fragment_tests)
         wm.EarlyDepthStencilControl = EDSC_PREPS;
      else if (wm_prog_data->has_side_effects)
         wm.EarlyDepthStencilControl = EDSC_PSEXEC;
#endif
   }
}

static const struct brw_tracked_state genX(wm_state) = {
   .dirty = {
      .mesa  = _NEW_LINE |
               _NEW_POLYGON |
               (GEN_GEN < 8 ? _NEW_BUFFERS |
                              _NEW_COLOR |
                              _NEW_MULTISAMPLE :
                              0) |
               (GEN_GEN < 7 ? _NEW_PROGRAM_CONSTANTS : 0),
      .brw   = BRW_NEW_BLORP |
               BRW_NEW_FS_PROG_DATA |
               (GEN_GEN < 7 ? BRW_NEW_PUSH_CONSTANT_ALLOCATION |
                              BRW_NEW_BATCH
                            : BRW_NEW_CONTEXT),
   },
   .emit = genX(upload_wm),
};

/* ---------------------------------------------------------------------- */

#if GEN_GEN >= 8
static void
genX(upload_ps_extra)(struct brw_context *brw)
{
   const struct brw_wm_prog_data *prog_data =
      brw_wm_prog_data(brw->wm.base.prog_data);

   brw_batch_emit(brw, GENX(3DSTATE_PS_EXTRA), pse) {
      pse.PixelShaderValid = true;
      pse.PixelShaderComputedDepthMode = prog_data->computed_depth_mode;
      pse.PixelShaderKillsPixel = prog_data->uses_kill;
      pse.AttributeEnable = prog_data->num_varying_inputs != 0;
      pse.PixelShaderUsesSourceDepth = prog_data->uses_src_depth;
      pse.PixelShaderUsesSourceW = prog_data->uses_src_w;
      pse.PixelShaderIsPerSample = prog_data->persample_dispatch;

      /* _NEW_MULTISAMPLE | BRW_NEW_CONSERVATIVE_RASTERIZATION */
      if (prog_data->uses_sample_mask) {
#if GEN_GEN >= 9
         struct gl_context *ctx = &brw->ctx;

         if (prog_data->post_depth_coverage)
            pse.InputCoverageMaskState = ICMS_DEPTH_COVERAGE;
         else if (prog_data->inner_coverage && ctx->IntelConservativeRasterization)
            pse.InputCoverageMaskState = ICMS_INNER_CONSERVATIVE;
         else
            pse.InputCoverageMaskState = ICMS_NORMAL;
#else
         pse.PixelShaderUsesInputCoverageMask = true;
#endif
      }

      pse.oMaskPresenttoRenderTarget = prog_data->uses_omask;
#if GEN_GEN >= 9
      pse.PixelShaderPullsBary = prog_data->pulls_bary;
#endif

      /* The stricter cross-primitive coherency guarantees that the hardware
       * gives us with the "Accesses UAV" bit set for at least one shader stage
       * and the "UAV coherency required" bit set on the 3DPRIMITIVE command
       * are redundant within the current image, atomic counter and SSBO GL
       * APIs, which all have very loose ordering and coherency requirements
       * and generally rely on the application to insert explicit barriers when
       * a shader invocation is expected to see the memory writes performed by
       * the invocations of some previous primitive.  Regardless of the value
       * of "UAV coherency required", the "Accesses UAV" bits will implicitly
       * cause an in most cases useless DC flush when the lowermost stage with
       * the bit set finishes execution.
       *
       * It would be nice to disable it, but in some cases we can't because on
       * Gen8+ it also has an influence on rasterization via the PS UAV-only
       * signal (which could be set independently from the coherency mechanism
       * in the 3DSTATE_WM command on Gen7), and because in some cases it will
       * determine whether the hardware skips execution of the fragment shader
       * or not via the ThreadDispatchEnable signal.  However if we know that
       * GEN8_PS_BLEND_HAS_WRITEABLE_RT is going to be set and
       * GEN8_PSX_PIXEL_SHADER_NO_RT_WRITE is not set it shouldn't make any
       * difference so we may just disable it here.
       *
       * Gen8 hardware tries to compute ThreadDispatchEnable for us but doesn't
       * take into account KillPixels when no depth or stencil writes are
       * enabled.  In order for occlusion queries to work correctly with no
       * attachments, we need to force-enable here.
       *
       * BRW_NEW_FS_PROG_DATA | BRW_NEW_FRAGMENT_PROGRAM | _NEW_BUFFERS |
       * _NEW_COLOR
       */
      if ((prog_data->has_side_effects || prog_data->uses_kill) &&
          !brw_color_buffer_write_enabled(brw))
         pse.PixelShaderHasUAV = true;;

      if (prog_data->computed_stencil) {
         assert(brw->gen >= 9);
#if GEN_GEN >= 9
         pse.PixelShaderComputesStencil = true;
#endif
      }
   }
}

const struct brw_tracked_state genX(ps_extra) = {
   .dirty = {
      .mesa  = _NEW_BUFFERS | _NEW_COLOR,
      .brw   = BRW_NEW_BLORP |
               BRW_NEW_CONTEXT |
               BRW_NEW_FRAGMENT_PROGRAM |
               BRW_NEW_FS_PROG_DATA |
               BRW_NEW_CONSERVATIVE_RASTERIZATION,
   },
   .emit = genX(upload_ps_extra),
};
#endif

/* ---------------------------------------------------------------------- */

#define INIT_THREAD_DISPATCH_FIELDS(pkt, prefix) \
   pkt.KernelStartPointer = stage_state->prog_offset;                     \
   pkt.SamplerCount       =                                               \
      DIV_ROUND_UP(CLAMP(stage_state->sampler_count, 0, 16), 4);          \
   pkt.BindingTableEntryCount =                                           \
      stage_prog_data->binding_table.size_bytes / 4;                      \
   pkt.FloatingPointMode  = stage_prog_data->use_alt_mode;                \
                                                                          \
   if (stage_prog_data->total_scratch) {                                  \
      pkt.ScratchSpaceBasePointer = (struct brw_address) {                \
         .bo = stage_state->scratch_bo,                                   \
         .offset = 0,                                                     \
         .read_domains = I915_GEM_DOMAIN_RENDER,                          \
         .write_domain = I915_GEM_DOMAIN_RENDER,                          \
      };                                                                  \
      pkt.PerThreadScratchSpace =                                         \
         ffs(stage_state->per_thread_scratch) - 11;                       \
   }                                                                      \
                                                                          \
   pkt.DispatchGRFStartRegisterForURBData =                               \
      stage_prog_data->dispatch_grf_start_reg;                            \
   pkt.prefix##URBEntryReadLength = vue_prog_data->urb_read_length;       \
   pkt.prefix##URBEntryReadOffset = 0;                                    \
                                                                          \
   pkt.StatisticsEnable = true;                                           \
   pkt.Enable           = true;


static void
genX(upload_vs_state)(struct brw_context *brw)
{
   const struct gen_device_info *devinfo = &brw->screen->devinfo;
   const struct brw_stage_state *stage_state = &brw->vs.base;

   /* BRW_NEW_VS_PROG_DATA */
   const struct brw_vue_prog_data *vue_prog_data =
      brw_vue_prog_data(brw->vs.base.prog_data);
   const struct brw_stage_prog_data *stage_prog_data = &vue_prog_data->base;

#if GEN_GEN >= 8
   /* _NEW_TRANSFORM */
   struct gl_context *ctx = &brw->ctx;
   const struct gl_transform_attrib *transform = &ctx->Transform;
#endif

   assert(vue_prog_data->dispatch_mode == DISPATCH_MODE_SIMD8 ||
          vue_prog_data->dispatch_mode == DISPATCH_MODE_4X2_DUAL_OBJECT);

#if GEN_GEN < 7
   brw_batch_emit(brw, GENX(3DSTATE_CONSTANT_VS), cvs) {
      if (stage_state->push_const_size != 0) {
         cvs.Buffer0Valid = true;
         cvs.PointertoVSConstantBuffer0 = stage_state->push_const_offset;
         cvs.VSConstantBuffer0ReadLength = stage_state->push_const_size - 1;
      }
   }
#endif

   if (devinfo->is_ivybridge)
      gen7_emit_vs_workaround_flush(brw);

   brw_batch_emit(brw, GENX(3DSTATE_VS), vs) {
      INIT_THREAD_DISPATCH_FIELDS(vs, Vertex);

      vs.MaximumNumberofThreads = devinfo->max_vs_threads - 1;

#if GEN_GEN >= 8
      vs.SIMD8DispatchEnable =
         vue_prog_data->dispatch_mode == DISPATCH_MODE_SIMD8;

      vs.UserClipDistanceClipTestEnableBitmask = transform->ClipPlanesEnabled;
      vs.UserClipDistanceCullTestEnableBitmask =
         vue_prog_data->cull_distance_mask;
#endif
   }

#if GEN_GEN < 7
   brw_emit_pipe_control_flush(brw,
                               PIPE_CONTROL_DEPTH_STALL |
                               PIPE_CONTROL_INSTRUCTION_INVALIDATE |
                               PIPE_CONTROL_STATE_CACHE_INVALIDATE);
#endif
}

static const struct brw_tracked_state genX(vs_state) = {
   .dirty = {
      .mesa  = _NEW_TRANSFORM |
               (GEN_GEN < 7 ? _NEW_PROGRAM_CONSTANTS : 0),
      .brw   = BRW_NEW_BATCH |
               BRW_NEW_BLORP |
               BRW_NEW_CONTEXT |
               BRW_NEW_VS_PROG_DATA |
               (GEN_GEN < 7 ? BRW_NEW_PUSH_CONSTANT_ALLOCATION |
                              BRW_NEW_VERTEX_PROGRAM
                            : 0),
   },
   .emit = genX(upload_vs_state),
};

#if GEN_GEN >= 7
static void
genX(upload_hs_state)(struct brw_context *brw)
{
   const struct gen_device_info *devinfo = &brw->screen->devinfo;
   struct brw_stage_state *stage_state = &brw->tcs.base;
   struct brw_stage_prog_data *stage_prog_data = stage_state->prog_data;
   const struct brw_vue_prog_data *vue_prog_data =
      brw_vue_prog_data(stage_prog_data);

   /* BRW_NEW_TES_PROG_DATA */
   struct brw_tcs_prog_data *tcs_prog_data =
      brw_tcs_prog_data(stage_prog_data);

   if (!tcs_prog_data) {
      brw_batch_emit(brw, GENX(3DSTATE_HS), hs);
   } else {
      brw_batch_emit(brw, GENX(3DSTATE_HS), hs) {
         INIT_THREAD_DISPATCH_FIELDS(hs, Vertex);

         hs.InstanceCount = tcs_prog_data->instances - 1;
         hs.IncludeVertexHandles = true;

        hs.MaximumNumberofThreads = devinfo->max_tcs_threads - 1;
      }
   }
}

static const struct brw_tracked_state genX(hs_state) = {
   .dirty = {
      .mesa  = 0,
      .brw   = BRW_NEW_BATCH |
               BRW_NEW_BLORP |
               BRW_NEW_TCS_PROG_DATA |
               BRW_NEW_TESS_PROGRAMS,
   },
   .emit = genX(upload_hs_state),
};

static void
genX(upload_ds_state)(struct brw_context *brw)
{
   const struct gen_device_info *devinfo = &brw->screen->devinfo;
   const struct brw_stage_state *stage_state = &brw->tes.base;
   struct brw_stage_prog_data *stage_prog_data = stage_state->prog_data;

   /* BRW_NEW_TES_PROG_DATA */
   const struct brw_tes_prog_data *tes_prog_data =
      brw_tes_prog_data(stage_prog_data);
   const struct brw_vue_prog_data *vue_prog_data =
      brw_vue_prog_data(stage_prog_data);

#if GEN_GEN >= 8
   /* _NEW_TRANSFORM */
   struct gl_context *ctx = &brw->ctx;
   const struct gl_transform_attrib *transform = &ctx->Transform;
#endif

   if (!tes_prog_data) {
      brw_batch_emit(brw, GENX(3DSTATE_DS), ds);
   } else {
      brw_batch_emit(brw, GENX(3DSTATE_DS), ds) {
         INIT_THREAD_DISPATCH_FIELDS(ds, Patch);

        ds.MaximumNumberofThreads = devinfo->max_tes_threads - 1;
        ds.ComputeWCoordinateEnable =
           tes_prog_data->domain == BRW_TESS_DOMAIN_TRI;

#if GEN_GEN >= 8
        if (vue_prog_data->dispatch_mode == DISPATCH_MODE_SIMD8)
           ds.DispatchMode = DISPATCH_MODE_SIMD8_SINGLE_PATCH;
        ds.UserClipDistanceClipTestEnableBitmask =
            transform->ClipPlanesEnabled;
        ds.UserClipDistanceCullTestEnableBitmask =
            vue_prog_data->cull_distance_mask;
#endif
      }
   }
}

static const struct brw_tracked_state genX(ds_state) = {
   .dirty = {
      .mesa  = (GEN_GEN < 8 ? _NEW_TRANSFORM : 0),
      .brw   = BRW_NEW_BATCH |
               BRW_NEW_BLORP |
               BRW_NEW_TESS_PROGRAMS |
               BRW_NEW_TES_PROG_DATA |
               (GEN_GEN < 8 ? BRW_NEW_CONTEXT : 0),
   },
   .emit = genX(upload_ds_state),
};
#endif

static void
genX(upload_gs_state)(struct brw_context *brw)
{
   const struct gen_device_info *devinfo = &brw->screen->devinfo;
   const struct brw_stage_state *stage_state = &brw->gs.base;
   /* BRW_NEW_GEOMETRY_PROGRAM */
   bool active = brw->geometry_program;

   /* BRW_NEW_GS_PROG_DATA */
   struct brw_stage_prog_data *stage_prog_data = stage_state->prog_data;
   const struct brw_vue_prog_data *vue_prog_data =
      brw_vue_prog_data(stage_prog_data);
#if GEN_GEN >= 7
   const struct brw_gs_prog_data *gs_prog_data =
      brw_gs_prog_data(stage_prog_data);
#endif

   /* _NEW_TRANSFORM */
#if GEN_GEN >= 8
   struct gl_context *ctx = &brw->ctx;
   const struct gl_transform_attrib *transform = &ctx->Transform;
#endif

#if GEN_GEN < 7
   brw_batch_emit(brw, GENX(3DSTATE_CONSTANT_GS), cgs) {
      if (active && stage_state->push_const_size != 0) {
         cgs.Buffer0Valid = true;
         cgs.PointertoGSConstantBuffer0 = stage_state->push_const_offset;
         cgs.GSConstantBuffer0ReadLength = stage_state->push_const_size - 1;
      }
   }
#endif

   if (active) {
      brw_batch_emit(brw, GENX(3DSTATE_GS), gs) {
         INIT_THREAD_DISPATCH_FIELDS(gs, Vertex);

#if GEN_GEN >= 7
         gs.OutputVertexSize = gs_prog_data->output_vertex_size_hwords * 2 - 1;
         gs.OutputTopology = gs_prog_data->output_topology;
         gs.ControlDataHeaderSize =
            gs_prog_data->control_data_header_size_hwords;

         gs.InstanceControl = gs_prog_data->invocations - 1;
         gs.DispatchMode = vue_prog_data->dispatch_mode;

         gs.IncludePrimitiveID = gs_prog_data->include_primitive_id;

         gs.ControlDataFormat = gs_prog_data->control_data_format;
#endif

         gs.ReorderMode = TRAILING;
         gs.MaximumNumberofThreads =
            GEN_GEN == 8 ? (devinfo->max_gs_threads / 2 - 1)
                         : (devinfo->max_gs_threads - 1);

#if GEN_GEN < 7
         gs.SOStatisticsEnable = true;
         gs.RenderingEnabled = 1;
         if (brw->geometry_program->info.has_transform_feedback_varyings)
            gs.SVBIPayloadEnable = true;

         /* GEN6_GS_SPF_MODE and GEN6_GS_VECTOR_MASK_ENABLE are enabled as it
          * was previously done for gen6.
          *
          * TODO: test with both disabled to see if the HW is behaving
          * as expected, like in gen7.
          */
         gs.SingleProgramFlow = true;
         gs.VectorMaskEnable = true;
#endif

#if GEN_GEN >= 8
         gs.ExpectedVertexCount = gs_prog_data->vertices_in;

         if (gs_prog_data->static_vertex_count != -1) {
            gs.StaticOutput = true;
            gs.StaticOutputVertexCount = gs_prog_data->static_vertex_count;
         }
         gs.IncludeVertexHandles = vue_prog_data->include_vue_handles;

         gs.UserClipDistanceClipTestEnableBitmask =
            transform->ClipPlanesEnabled;
         gs.UserClipDistanceCullTestEnableBitmask =
            vue_prog_data->cull_distance_mask;

         const int urb_entry_write_offset = 1;
         const uint32_t urb_entry_output_length =
            DIV_ROUND_UP(vue_prog_data->vue_map.num_slots, 2) -
            urb_entry_write_offset;

         gs.VertexURBEntryOutputReadOffset = urb_entry_write_offset;
         gs.VertexURBEntryOutputLength = MAX2(urb_entry_output_length, 1);
#endif
      }
#if GEN_GEN < 7
   } else if (brw->ff_gs.prog_active)  {
      /* In gen6, transform feedback for the VS stage is done with an ad-hoc GS
       * program. This function provides the needed 3DSTATE_GS for this.
       */
      upload_gs_state_for_tf(brw);
#endif
   } else {
      brw_batch_emit(brw, GENX(3DSTATE_GS), gs) {
         gs.StatisticsEnable = true;
#if GEN_GEN < 7
         gs.RenderingEnabled = true;
#endif

#if GEN_GEN < 8
         gs.DispatchGRFStartRegisterForURBData = 1;
#if GEN_GEN >= 7
         gs.IncludeVertexHandles = true;
#endif
#endif
      }
   }
#if GEN_GEN < 7
   brw->gs.enabled = active;
#endif
}

static const struct brw_tracked_state genX(gs_state) = {
   .dirty = {
      .mesa  = (GEN_GEN < 8 ? _NEW_TRANSFORM : 0) |
               (GEN_GEN < 7 ? _NEW_PROGRAM_CONSTANTS : 0),
      .brw   = BRW_NEW_BATCH |
               BRW_NEW_BLORP |
               BRW_NEW_CONTEXT |
               BRW_NEW_GEOMETRY_PROGRAM |
               BRW_NEW_GS_PROG_DATA |
               (GEN_GEN < 7 ? BRW_NEW_FF_GS_PROG_DATA |
                              BRW_NEW_PUSH_CONSTANT_ALLOCATION
                            : 0),
   },
   .emit = genX(upload_gs_state),
};

static void
genX(upload_sf_clip_viewport)(struct brw_context *brw)
{
   struct gl_context *ctx = &brw->ctx;
   float y_scale, y_bias;
   const struct gen_device_info *devinfo = &brw->screen->devinfo;

   /* BRW_NEW_VIEWPORT_COUNT */
   const unsigned viewport_count = brw->clip.viewport_count;

   /* _NEW_BUFFERS */
   const bool render_to_fbo = _mesa_is_user_fbo(ctx->DrawBuffer);
   const uint32_t fb_width = (float)_mesa_geometric_width(ctx->DrawBuffer);
   const uint32_t fb_height = (float)_mesa_geometric_height(ctx->DrawBuffer);

#if GEN_GEN >= 7
#define clv sfv
   struct GENX(SF_CLIP_VIEWPORT) sfv;
   uint32_t sf_clip_vp_offset;
   uint32_t *sf_clip_map = brw_state_batch(brw, 16 * 4 * viewport_count,
                                           64, &sf_clip_vp_offset);
#else
   struct GENX(SF_VIEWPORT) sfv;
   struct GENX(CLIP_VIEWPORT) clv;
   uint32_t *sf_map = brw_state_batch(brw, 8 * 4 * viewport_count,
                                      32, &brw->sf.vp_offset);
   uint32_t *clip_map = brw_state_batch(brw, 4 * 4 * viewport_count,
                                        32, &brw->clip.vp_offset);
#endif

   /* _NEW_BUFFERS */
   if (render_to_fbo) {
      y_scale = 1.0;
      y_bias = 0;
   } else {
      y_scale = -1.0;
      y_bias = (float)fb_height;
   }

   for (unsigned i = 0; i < brw->clip.viewport_count; i++) {
      /* _NEW_VIEWPORT: Guardband Clipping */
      float scale[3], translate[3], gb_xmin, gb_xmax, gb_ymin, gb_ymax;
      _mesa_get_viewport_xform(ctx, i, scale, translate);

      sfv.ViewportMatrixElementm00 = scale[0];
      sfv.ViewportMatrixElementm11 = scale[1] * y_scale,
      sfv.ViewportMatrixElementm22 = scale[2],
      sfv.ViewportMatrixElementm30 = translate[0],
      sfv.ViewportMatrixElementm31 = translate[1] * y_scale + y_bias,
      sfv.ViewportMatrixElementm32 = translate[2],
      brw_calculate_guardband_size(devinfo, fb_width, fb_height,
                                   sfv.ViewportMatrixElementm00,
                                   sfv.ViewportMatrixElementm11,
                                   sfv.ViewportMatrixElementm30,
                                   sfv.ViewportMatrixElementm31,
                                   &gb_xmin, &gb_xmax, &gb_ymin, &gb_ymax);


      clv.XMinClipGuardband = gb_xmin;
      clv.XMaxClipGuardband = gb_xmax;
      clv.YMinClipGuardband = gb_ymin;
      clv.YMaxClipGuardband = gb_ymax;

#if GEN_GEN >= 8
      /* _NEW_VIEWPORT | _NEW_BUFFERS: Screen Space Viewport
       * The hardware will take the intersection of the drawing rectangle,
       * scissor rectangle, and the viewport extents. We don't need to be
       * smart, and can therefore just program the viewport extents.
       */
      const float viewport_Xmax =
         ctx->ViewportArray[i].X + ctx->ViewportArray[i].Width;
      const float viewport_Ymax =
         ctx->ViewportArray[i].Y + ctx->ViewportArray[i].Height;

      if (render_to_fbo) {
         sfv.XMinViewPort = ctx->ViewportArray[i].X;
         sfv.XMaxViewPort = viewport_Xmax - 1;
         sfv.YMinViewPort = ctx->ViewportArray[i].Y;
         sfv.YMaxViewPort = viewport_Ymax - 1;
      } else {
         sfv.XMinViewPort = ctx->ViewportArray[i].X;
         sfv.XMaxViewPort = viewport_Xmax - 1;
         sfv.YMinViewPort = fb_height - viewport_Ymax;
         sfv.YMaxViewPort = fb_height - ctx->ViewportArray[i].Y - 1;
      }
#endif

#if GEN_GEN >= 7
      GENX(SF_CLIP_VIEWPORT_pack)(NULL, sf_clip_map, &sfv);
      sf_clip_map += 16;
#else
      GENX(SF_VIEWPORT_pack)(NULL, sf_map, &sfv);
      GENX(CLIP_VIEWPORT_pack)(NULL, clip_map, &clv);
      sf_map += 8;
      clip_map += 4;
#endif
   }

#if GEN_GEN >= 7
   brw_batch_emit(brw, GENX(3DSTATE_VIEWPORT_STATE_POINTERS_SF_CLIP), ptr) {
      ptr.SFClipViewportPointer = sf_clip_vp_offset;
   }
#else
   brw->ctx.NewDriverState |= BRW_NEW_SF_VP | BRW_NEW_CLIP_VP;
#endif
}

static const struct brw_tracked_state genX(sf_clip_viewport) = {
   .dirty = {
      .mesa = _NEW_BUFFERS |
              _NEW_VIEWPORT,
      .brw = BRW_NEW_BATCH |
             BRW_NEW_BLORP |
             BRW_NEW_VIEWPORT_COUNT,
   },
   .emit = genX(upload_sf_clip_viewport),
};

/* ---------------------------------------------------------------------- */

#define blend_factor(x) brw_translate_blend_factor(x)
#define blend_eqn(x) brw_translate_blend_equation(x)

static void
genX(upload_blend_state)(struct brw_context *brw)
{
   struct gl_context *ctx = &brw->ctx;
   int size;

   /* We need at least one BLEND_STATE written, because we might do
    * thread dispatch even if _NumColorDrawBuffers is 0 (for example
    * for computed depth or alpha test), which will do an FB write
    * with render target 0, which will reference BLEND_STATE[0] for
    * alpha test enable.
    */
   int nr_draw_buffers = ctx->DrawBuffer->_NumColorDrawBuffers;
   if (nr_draw_buffers == 0 && ctx->Color.AlphaEnabled)
      nr_draw_buffers = 1;

   size = GENX(BLEND_STATE_ENTRY_length) * 4 * nr_draw_buffers;
#if GEN_GEN >= 8
   size += GENX(BLEND_STATE_length) * 4;
#endif

   uint32_t *blend_map;
   blend_map = brw_state_batch(brw, size, 64, &brw->cc.blend_state_offset);
   memset(blend_map, 0, size);

#if GEN_GEN >= 8
   struct GENX(BLEND_STATE) blend;
   memset(&blend, 0, sizeof(blend));
   {
#else
   for (int i = 0; i < nr_draw_buffers; i++) {
      struct GENX(BLEND_STATE_ENTRY) entry;
      memset(&entry, 0, sizeof(entry));
#define blend entry
#endif
      /* OpenGL specification 3.3 (page 196), section 4.1.3 says:
       * "If drawbuffer zero is not NONE and the buffer it references has an
       * integer format, the SAMPLE_ALPHA_TO_COVERAGE and SAMPLE_ALPHA_TO_ONE
       * operations are skipped."
       */
      if (!(ctx->DrawBuffer->_IntegerBuffers & 0x1)) {
         /* _NEW_MULTISAMPLE */
         if (_mesa_is_multisample_enabled(ctx)) {
            if (ctx->Multisample.SampleAlphaToCoverage) {
               blend.AlphaToCoverageEnable = true;
               blend.AlphaToCoverageDitherEnable = true;
            }
            if (ctx->Multisample.SampleAlphaToOne)
               blend.AlphaToOneEnable = true;
         }

         /* _NEW_COLOR */
         if (ctx->Color.AlphaEnabled) {
            blend.AlphaTestEnable = true;
            blend.AlphaTestFunction =
               intel_translate_compare_func(ctx->Color.AlphaFunc);
         }

         if (ctx->Color.DitherFlag) {
            blend.ColorDitherEnable = true;
         }
      }

#if GEN_GEN >= 8
      for (int i = 0; i < nr_draw_buffers; i++) {
         struct GENX(BLEND_STATE_ENTRY) entry;
         memset(&entry, 0, sizeof(entry));
#else
      {
#endif

         /* _NEW_BUFFERS */
         struct gl_renderbuffer *rb = ctx->DrawBuffer->_ColorDrawBuffers[i];

         /* Used for implementing the following bit of GL_EXT_texture_integer:
          * "Per-fragment operations that require floating-point color
          *  components, including multisample alpha operations, alpha test,
          *  blending, and dithering, have no effect when the corresponding
          *  colors are written to an integer color buffer."
          */
         bool integer = ctx->DrawBuffer->_IntegerBuffers & (0x1 << i);

#if GEN_GEN < 8
         GLenum rb_type;
         /* bool is_buffer_zero_integer_format = false; */

         if (rb)
            rb_type = _mesa_get_format_datatype(rb->Format);
         else
            rb_type = GL_UNSIGNED_NORMALIZED;

         /* if(i == 0 && integer) */
         /*    is_buffer_zero_integer_format = true; */
#endif

         /* _NEW_COLOR */
         if (ctx->Color.ColorLogicOpEnabled) {
#if GEN_GEN < 8
            WARN_ONCE(ctx->Color.LogicOp != GL_COPY &&
                      rb_type != GL_UNSIGNED_NORMALIZED &&
                      rb_type != GL_FLOAT, "Ignoring %s logic op on %s "
                      "renderbuffer\n",
                      _mesa_enum_to_string(ctx->Color.LogicOp),
                      _mesa_enum_to_string(rb_type));
            if (rb_type == GL_UNSIGNED_NORMALIZED) {
#endif
            entry.LogicOpEnable = true;
            entry.LogicOpFunction =
               intel_translate_logic_op(ctx->Color.LogicOp);
#if GEN_GEN < 8
            }
#endif
         } else if (ctx->Color.BlendEnabled & (1 << i) && !integer &&
                    !ctx->Color._AdvancedBlendMode) {
            GLenum eqRGB = ctx->Color.Blend[i].EquationRGB;
            GLenum eqA = ctx->Color.Blend[i].EquationA;
            GLenum srcRGB = ctx->Color.Blend[i].SrcRGB;
            GLenum dstRGB = ctx->Color.Blend[i].DstRGB;
            GLenum srcA = ctx->Color.Blend[i].SrcA;
            GLenum dstA = ctx->Color.Blend[i].DstA;

            if (eqRGB == GL_MIN || eqRGB == GL_MAX)
               srcRGB = dstRGB = GL_ONE;

            if (eqA == GL_MIN || eqA == GL_MAX)
               srcA = dstA = GL_ONE;

            /* Due to hardware limitations, the destination may have information
             * in an alpha channel even when the format specifies no alpha
             * channel. In order to avoid getting any incorrect blending due to
             * that alpha channel, coerce the blend factors to values that will
             * not read the alpha channel, but will instead use the correct
             * implicit value for alpha.
             */
            if (rb && !_mesa_base_format_has_channel(rb->_BaseFormat,
                                                     GL_TEXTURE_ALPHA_TYPE)) {
               srcRGB = brw_fix_xRGB_alpha(srcRGB);
               srcA = brw_fix_xRGB_alpha(srcA);
               dstRGB = brw_fix_xRGB_alpha(dstRGB);
               dstA = brw_fix_xRGB_alpha(dstA);
            }

            entry.ColorBufferBlendEnable = true;
            entry.DestinationBlendFactor = blend_factor(dstRGB);
            entry.SourceBlendFactor = blend_factor(srcRGB);
            entry.DestinationAlphaBlendFactor = blend_factor(dstA);
            entry.SourceAlphaBlendFactor = blend_factor(srcA);
            entry.ColorBlendFunction = blend_eqn(eqRGB);
            entry.AlphaBlendFunction = blend_eqn(eqA);

            if (srcA != srcRGB || dstA != dstRGB || eqA != eqRGB)
               blend.IndependentAlphaBlendEnable = true;
         }

         /* See section 8.1.6 "Pre-Blend Color Clamping" of the
          * SandyBridge PRM Volume 2 Part 1 for HW requirements.
          *
          * We do our ARB_color_buffer_float CLAMP_FRAGMENT_COLOR
          * clamping in the fragment shader.  For its clamping of
          * blending, the spec says:
          *
          *     "RESOLVED: For fixed-point color buffers, the inputs and
          *      the result of the blending equation are clamped.  For
          *      floating-point color buffers, no clamping occurs."
          *
          * So, generally, we want clamping to the render target's range.
          * And, good news, the hardware tables for both pre- and
          * post-blend color clamping are either ignored, or any are
          * allowed, or clamping is required but RT range clamping is a
          * valid option.
          */
         entry.PreBlendColorClampEnable = true;
         entry.PostBlendColorClampEnable = true;
         entry.ColorClampRange = COLORCLAMP_RTFORMAT;

         if (!ctx->Color.ColorMask[i][0])
            entry.WriteDisableRed = true;
         if (!ctx->Color.ColorMask[i][1])
            entry.WriteDisableGreen = true;
         if (!ctx->Color.ColorMask[i][2])
            entry.WriteDisableBlue = true;
         if (!ctx->Color.ColorMask[i][3])
            entry.WriteDisableAlpha = true;

         /* From the BLEND_STATE docs, DWord 0, Bit 29 (AlphaToOne Enable):
          * "If Dual Source Blending is enabled, this bit must be disabled."
          */
         WARN_ONCE(ctx->Color.Blend[i]._UsesDualSrc &&
                   _mesa_is_multisample_enabled(ctx) &&
                   ctx->Multisample.SampleAlphaToOne,
                   "HW workaround: disabling alpha to one with dual src "
                   "blending\n");
         if (ctx->Color.Blend[i]._UsesDualSrc)
            blend.AlphaToOneEnable = false;
#if GEN_GEN >= 8
         GENX(BLEND_STATE_ENTRY_pack)(NULL, &blend_map[1 + i * 2], &entry);
#else
         GENX(BLEND_STATE_ENTRY_pack)(NULL, &blend_map[i * 2], &entry);
#endif
      }
   }

#if GEN_GEN >= 8
   GENX(BLEND_STATE_pack)(NULL, blend_map, &blend);
#endif

#if GEN_GEN < 7
   brw_batch_emit(brw, GENX(3DSTATE_CC_STATE_POINTERS), ptr) {
      ptr.PointertoBLEND_STATE = brw->cc.blend_state_offset;
      ptr.BLEND_STATEChange = true;
   }
#else
   brw_batch_emit(brw, GENX(3DSTATE_BLEND_STATE_POINTERS), ptr) {
      ptr.BlendStatePointer = brw->cc.blend_state_offset;
#if GEN_GEN >= 8
      ptr.BlendStatePointerValid = true;
#endif
   }
#endif
}

static const struct brw_tracked_state genX(blend_state) = {
   .dirty = {
      .mesa = _NEW_BUFFERS |
              _NEW_COLOR |
              _NEW_MULTISAMPLE,
      .brw = BRW_NEW_BATCH |
             BRW_NEW_BLORP |
             BRW_NEW_STATE_BASE_ADDRESS,
   },
   .emit = genX(upload_blend_state),
};

#if GEN_GEN >= 8
static void
genX(upload_ps_blend)(struct brw_context *brw)
{
   struct gl_context *ctx = &brw->ctx;

   /* _NEW_BUFFERS */
   struct gl_renderbuffer *rb = ctx->DrawBuffer->_ColorDrawBuffers[0];
   const bool buffer0_is_integer = ctx->DrawBuffer->_IntegerBuffers & 0x1;

   /* _NEW_COLOR */
   struct gl_colorbuffer_attrib *color = &ctx->Color;

   brw_batch_emit(brw, GENX(3DSTATE_PS_BLEND), pb) {
      /* BRW_NEW_FRAGMENT_PROGRAM | _NEW_BUFFERS | _NEW_COLOR */
      pb.HasWriteableRT = brw_color_buffer_write_enabled(brw);

      if (!buffer0_is_integer) {
         /* _NEW_MULTISAMPLE */
         pb.AlphaToCoverageEnable =
            _mesa_is_multisample_enabled(ctx) &&
            ctx->Multisample.SampleAlphaToCoverage;

         pb.AlphaTestEnable = color->AlphaEnabled;
      }

      /* Used for implementing the following bit of GL_EXT_texture_integer:
       * "Per-fragment operations that require floating-point color
       *  components, including multisample alpha operations, alpha test,
       *  blending, and dithering, have no effect when the corresponding
       *  colors are written to an integer color buffer."
       *
       * The OpenGL specification 3.3 (page 196), section 4.1.3 says:
       * "If drawbuffer zero is not NONE and the buffer it references has an
       *  integer format, the SAMPLE_ALPHA_TO_COVERAGE and SAMPLE_ALPHA_TO_ONE
       *  operations are skipped."
       */
      if (rb && !buffer0_is_integer && (color->BlendEnabled & 1)) {
         GLenum eqRGB = color->Blend[0].EquationRGB;
         GLenum eqA = color->Blend[0].EquationA;
         GLenum srcRGB = color->Blend[0].SrcRGB;
         GLenum dstRGB = color->Blend[0].DstRGB;
         GLenum srcA = color->Blend[0].SrcA;
         GLenum dstA = color->Blend[0].DstA;

         if (eqRGB == GL_MIN || eqRGB == GL_MAX)
            srcRGB = dstRGB = GL_ONE;

         if (eqA == GL_MIN || eqA == GL_MAX)
            srcA = dstA = GL_ONE;

         /* Due to hardware limitations, the destination may have information
          * in an alpha channel even when the format specifies no alpha
          * channel. In order to avoid getting any incorrect blending due to
          * that alpha channel, coerce the blend factors to values that will
          * not read the alpha channel, but will instead use the correct
          * implicit value for alpha.
          */
         if (!_mesa_base_format_has_channel(rb->_BaseFormat,
                                            GL_TEXTURE_ALPHA_TYPE)) {
            srcRGB = brw_fix_xRGB_alpha(srcRGB);
            srcA = brw_fix_xRGB_alpha(srcA);
            dstRGB = brw_fix_xRGB_alpha(dstRGB);
            dstA = brw_fix_xRGB_alpha(dstA);
         }

         pb.ColorBufferBlendEnable = true;
         pb.SourceAlphaBlendFactor = brw_translate_blend_factor(srcA);
         pb.DestinationAlphaBlendFactor = brw_translate_blend_factor(dstA);
         pb.SourceBlendFactor = brw_translate_blend_factor(srcRGB);
         pb.DestinationBlendFactor = brw_translate_blend_factor(dstRGB);

         pb.IndependentAlphaBlendEnable =
            srcA != srcRGB || dstA != dstRGB || eqA != eqRGB;
      }
   }
}

static const struct brw_tracked_state genX(ps_blend) = {
   .dirty = {
      .mesa = _NEW_BUFFERS |
              _NEW_COLOR |
              _NEW_MULTISAMPLE,
      .brw = BRW_NEW_BLORP |
             BRW_NEW_CONTEXT |
             BRW_NEW_FRAGMENT_PROGRAM,
   },
   .emit = genX(upload_ps_blend)
};
#endif

/* ---------------------------------------------------------------------- */

#if GEN_GEN >= 7
static void
upload_te_state(struct brw_context *brw)
{
   /* BRW_NEW_TESS_PROGRAMS */
   bool active = brw->tess_eval_program;

   const struct brw_tes_prog_data *tes_prog_data =
      brw_tes_prog_data(brw->tes.base.prog_data);

   if (active) {
      brw_batch_emit(brw, GENX(3DSTATE_TE), te) {
         te.Partitioning = tes_prog_data->partitioning;
         te.OutputTopology = tes_prog_data->output_topology;
         te.TEDomain = tes_prog_data->domain;
         te.TEEnable = true;
         te.MaximumTessellationFactorOdd = 63.0;
         te.MaximumTessellationFactorNotOdd = 64.0;
      }
   } else {
      brw_batch_emit(brw, GENX(3DSTATE_TE), te);
   }
}

static const struct brw_tracked_state genX(te_state) = {
   .dirty = {
      .mesa  = 0,
      .brw   = BRW_NEW_BLORP |
               BRW_NEW_CONTEXT |
               BRW_NEW_TES_PROG_DATA |
               BRW_NEW_TESS_PROGRAMS,
   },
   .emit = upload_te_state,
};
#endif

static void
genX(upload_scissor_state)(struct brw_context *brw)
{
   struct gl_context *ctx = &brw->ctx;
   const bool render_to_fbo = _mesa_is_user_fbo(ctx->DrawBuffer);
   struct GENX(SCISSOR_RECT) scissor;
   uint32_t scissor_state_offset;
   const unsigned int fb_width= _mesa_geometric_width(ctx->DrawBuffer);
   const unsigned int fb_height = _mesa_geometric_height(ctx->DrawBuffer);
   uint32_t *scissor_map;

   /* BRW_NEW_VIEWPORT_COUNT */
   const unsigned viewport_count = brw->clip.viewport_count;

   scissor_map = brw_state_batch(
      brw, GENX(SCISSOR_RECT_length) * sizeof(uint32_t) * viewport_count,
      32, &scissor_state_offset);

   /* _NEW_SCISSOR | _NEW_BUFFERS | _NEW_VIEWPORT */

   /* The scissor only needs to handle the intersection of drawable and
    * scissor rect.  Clipping to the boundaries of static shared buffers
    * for front/back/depth is covered by looping over cliprects in brw_draw.c.
    *
    * Note that the hardware's coordinates are inclusive, while Mesa's min is
    * inclusive but max is exclusive.
    */
   for (unsigned i = 0; i < viewport_count; i++) {
      int bbox[4];

      bbox[0] = MAX2(ctx->ViewportArray[i].X, 0);
      bbox[1] = MIN2(bbox[0] + ctx->ViewportArray[i].Width, fb_width);
      bbox[2] = MAX2(ctx->ViewportArray[i].Y, 0);
      bbox[3] = MIN2(bbox[2] + ctx->ViewportArray[i].Height, fb_height);
      _mesa_intersect_scissor_bounding_box(ctx, i, bbox);

      if (bbox[0] == bbox[1] || bbox[2] == bbox[3]) {
         /* If the scissor was out of bounds and got clamped to 0 width/height
          * at the bounds, the subtraction of 1 from maximums could produce a
          * negative number and thus not clip anything.  Instead, just provide
          * a min > max scissor inside the bounds, which produces the expected
          * no rendering.
          */
         scissor.ScissorRectangleXMin = 1;
         scissor.ScissorRectangleXMax = 0;
         scissor.ScissorRectangleYMin = 1;
         scissor.ScissorRectangleYMax = 0;
      } else if (render_to_fbo) {
         /* texmemory: Y=0=bottom */
         scissor.ScissorRectangleXMin = bbox[0];
         scissor.ScissorRectangleXMax = bbox[1] - 1;
         scissor.ScissorRectangleYMin = bbox[2];
         scissor.ScissorRectangleYMax = bbox[3] - 1;
      } else {
         /* memory: Y=0=top */
         scissor.ScissorRectangleXMin = bbox[0];
         scissor.ScissorRectangleXMax = bbox[1] - 1;
         scissor.ScissorRectangleYMin = fb_height - bbox[3];
         scissor.ScissorRectangleYMax = fb_height - bbox[2] - 1;
      }

      GENX(SCISSOR_RECT_pack)(NULL, scissor_map + i * 2, &scissor);
   }

   brw_batch_emit(brw, GENX(3DSTATE_SCISSOR_STATE_POINTERS), ptr) {
      ptr.ScissorRectPointer = scissor_state_offset;
   }
}

static const struct brw_tracked_state genX(scissor_state) = {
   .dirty = {
      .mesa = _NEW_BUFFERS |
              _NEW_SCISSOR |
              _NEW_VIEWPORT,
      .brw = BRW_NEW_BATCH |
             BRW_NEW_BLORP |
             BRW_NEW_VIEWPORT_COUNT,
   },
   .emit = genX(upload_scissor_state),
};

/* ---------------------------------------------------------------------- */

#define GEN7_MOCS_L3                    1
#define upload_constant_state(brw, stage_state, active, opcode)               \
   do {                                                                       \
      uint32_t mocs = brw->gen < 8 ? GEN7_MOCS_L3 : 0;                        \
      bool active_tmp = active && stage_state->push_const_size != 0;          \
                                                                              \
      brw_batch_emit(brw, GENX(opcode), pkt) {                                \
         if (active_tmp) {                                                    \
            \
            if (brw->gen >= 9) {                                              \
               pkt.ConstantBody.ConstantBuffer2ReadLength =                   \
                  stage_state->push_const_size;                               \
               pkt.ConstantBody.PointerToConstantBuffer2.bo = brw->batch.bo;  \
               pkt.ConstantBody.PointerToConstantBuffer2.read_domains =       \
                  I915_GEM_DOMAIN_RENDER;                                     \
               pkt.ConstantBody.PointerToConstantBuffer2.offset =             \
                  stage_state->push_const_offset;                             \
            } else {                                                          \
               pkt.ConstantBody.ConstantBuffer0ReadLength =                   \
                  stage_state->push_const_size;                               \
               pkt.ConstantBody.PointerToConstantBuffer0.offset =             \
                  stage_state->push_const_offset | mocs;                      \
            }                                                                 \
         }                                                                    \
      }                                                                       \
                                                                              \
      if (brw->gen >= 9)                                                      \
         brw->ctx.NewDriverState |= BRW_NEW_SURFACES;                         \
   } while (0);


#if GEN_GEN >= 7
static void
genX(upload_tes_push_constants)(struct brw_context *brw)
{
   struct brw_stage_state *stage_state = &brw->tes.base;
   /* BRW_NEW_TESS_PROGRAMS */
   const struct brw_program *tep = brw_program_const(brw->tess_eval_program);

   if (tep) {
      /* BRW_NEW_TES_PROG_DATA */
      const struct brw_stage_prog_data *prog_data = brw->tes.base.prog_data;
      _mesa_shader_write_subroutine_indices(&brw->ctx, MESA_SHADER_TESS_EVAL);
      gen6_upload_push_constants(brw, &tep->program, prog_data, stage_state);
   }

   upload_constant_state(brw, stage_state, tep, 3DSTATE_CONSTANT_DS);
}

static const struct brw_tracked_state genX(tes_push_constants) = {
   .dirty = {
      .mesa  = _NEW_PROGRAM_CONSTANTS,
      .brw   = BRW_NEW_BATCH |
               BRW_NEW_BLORP |
               BRW_NEW_PUSH_CONSTANT_ALLOCATION |
               BRW_NEW_TESS_PROGRAMS |
               BRW_NEW_TES_PROG_DATA,
   },
   .emit = genX(upload_tes_push_constants),
};
#endif

static void
genX(upload_vs_push_constants)(struct brw_context *brw)
{
   struct brw_stage_state *stage_state = &brw->vs.base;

   /* _BRW_NEW_VERTEX_PROGRAM */
   const struct brw_program *vp = brw_program_const(brw->vertex_program);
   /* BRW_NEW_VS_PROG_DATA */
   const struct brw_stage_prog_data *prog_data = brw->vs.base.prog_data;

   _mesa_shader_write_subroutine_indices(&brw->ctx, MESA_SHADER_VERTEX);
   gen6_upload_push_constants(brw, &vp->program, prog_data, stage_state);

#if GEN_GEN >= 7
   if (brw->gen == 7 && !brw->is_haswell && !brw->is_baytrail)
      gen7_emit_vs_workaround_flush(brw);

   upload_constant_state(brw, stage_state, true /* active */,
                         3DSTATE_CONSTANT_VS);
#endif
}

const struct brw_tracked_state genX(vs_push_constants) = {
   .dirty = {
      .mesa  = _NEW_PROGRAM_CONSTANTS |
               _NEW_TRANSFORM,
      .brw   = BRW_NEW_BATCH |
               BRW_NEW_BLORP |
               BRW_NEW_PUSH_CONSTANT_ALLOCATION |
               BRW_NEW_VERTEX_PROGRAM |
               BRW_NEW_VS_PROG_DATA,
   },
   .emit = genX(upload_vs_push_constants),
};

/* ---------------------------------------------------------------------- */

void
genX(init_atoms)(struct brw_context *brw)
{
#if GEN_GEN == 6
   static const struct brw_tracked_state *render_atoms[] =
   {
      &genX(sf_clip_viewport),

      /* Command packets: */

      &brw_cc_vp,
      &gen6_viewport_state,	/* must do after *_vp stages */

      &gen6_urb,
      &gen6_blend_state,		/* must do before cc unit */
      &gen6_color_calc_state,	/* must do before cc unit */
      &gen6_depth_stencil_state,	/* must do before cc unit */

      &genX(vs_push_constants), /* Before vs_state */
      &gen6_gs_push_constants, /* Before gs_state */
      &gen6_wm_push_constants, /* Before wm_state */

      /* Surface state setup.  Must come before the VS/WM unit.  The binding
       * table upload must be last.
       */
      &brw_vs_pull_constants,
      &brw_vs_ubo_surfaces,
      &brw_gs_pull_constants,
      &brw_gs_ubo_surfaces,
      &brw_wm_pull_constants,
      &brw_wm_ubo_surfaces,
      &gen6_renderbuffer_surfaces,
      &brw_renderbuffer_read_surfaces,
      &brw_texture_surfaces,
      &gen6_sol_surface,
      &brw_vs_binding_table,
      &gen6_gs_binding_table,
      &brw_wm_binding_table,

      &brw_fs_samplers,
      &brw_vs_samplers,
      &brw_gs_samplers,
      &gen6_sampler_state,
      &gen6_multisample_state,

      &genX(vs_state),
      &genX(gs_state),
      &genX(clip_state),
      &genX(sf_state),
      &genX(wm_state),

      &genX(scissor_state),

      &gen6_binding_table_pointers,

      &brw_depthbuffer,

      &brw_polygon_stipple,
      &brw_polygon_stipple_offset,

      &brw_line_stipple,

      &brw_drawing_rect,

      &brw_indices, /* must come before brw_vertices */
      &brw_index_buffer,
      &brw_vertices,
   };
#elif GEN_GEN == 7
   static const struct brw_tracked_state *render_atoms[] =
   {
      /* Command packets: */

      &brw_cc_vp,
      &genX(sf_clip_viewport),

      &gen7_l3_state,
      &gen7_push_constant_space,
      &gen7_urb,
      &genX(blend_state),		/* must do before cc unit */
      &gen6_color_calc_state,	/* must do before cc unit */
      &genX(depth_stencil_state),	/* must do before cc unit */

      &brw_vs_image_surfaces, /* Before vs push/pull constants and binding table */
      &brw_tcs_image_surfaces, /* Before tcs push/pull constants and binding table */
      &brw_tes_image_surfaces, /* Before tes push/pull constants and binding table */
      &brw_gs_image_surfaces, /* Before gs push/pull constants and binding table */
      &brw_wm_image_surfaces, /* Before wm push/pull constants and binding table */

      &genX(vs_push_constants), /* Before vs_state */
      &gen7_tcs_push_constants,
      &genX(tes_push_constants),
      &gen6_gs_push_constants, /* Before gs_state */
      &gen6_wm_push_constants, /* Before wm_surfaces and constant_buffer */

      /* Surface state setup.  Must come before the VS/WM unit.  The binding
       * table upload must be last.
       */
      &brw_vs_pull_constants,
      &brw_vs_ubo_surfaces,
      &brw_vs_abo_surfaces,
      &brw_tcs_pull_constants,
      &brw_tcs_ubo_surfaces,
      &brw_tcs_abo_surfaces,
      &brw_tes_pull_constants,
      &brw_tes_ubo_surfaces,
      &brw_tes_abo_surfaces,
      &brw_gs_pull_constants,
      &brw_gs_ubo_surfaces,
      &brw_gs_abo_surfaces,
      &brw_wm_pull_constants,
      &brw_wm_ubo_surfaces,
      &brw_wm_abo_surfaces,
      &gen6_renderbuffer_surfaces,
      &brw_renderbuffer_read_surfaces,
      &brw_texture_surfaces,
      &brw_vs_binding_table,
      &brw_tcs_binding_table,
      &brw_tes_binding_table,
      &brw_gs_binding_table,
      &brw_wm_binding_table,

      &brw_fs_samplers,
      &brw_vs_samplers,
      &brw_tcs_samplers,
      &brw_tes_samplers,
      &brw_gs_samplers,
      &gen6_multisample_state,

      &genX(vs_state),
      &genX(hs_state),
      &genX(te_state),
      &genX(ds_state),
      &genX(gs_state),
      &genX(sol_state),
      &genX(clip_state),
      &genX(sbe_state),
      &genX(sf_state),
      &genX(wm_state),
      &genX(ps_state),

      &genX(scissor_state),

      &gen7_depthbuffer,

      &brw_polygon_stipple,
      &brw_polygon_stipple_offset,

      &brw_line_stipple,

      &brw_drawing_rect,

      &brw_indices, /* must come before brw_vertices */
      &brw_index_buffer,
      &brw_vertices,

      &haswell_cut_index,
   };
#elif GEN_GEN >= 8
   static const struct brw_tracked_state *render_atoms[] =
   {
      &brw_cc_vp,
      &genX(sf_clip_viewport),

      &gen7_l3_state,
      &gen7_push_constant_space,
      &gen7_urb,
      &genX(blend_state),
      &gen6_color_calc_state,

      &brw_vs_image_surfaces, /* Before vs push/pull constants and binding table */
      &brw_tcs_image_surfaces, /* Before tcs push/pull constants and binding table */
      &brw_tes_image_surfaces, /* Before tes push/pull constants and binding table */
      &brw_gs_image_surfaces, /* Before gs push/pull constants and binding table */
      &brw_wm_image_surfaces, /* Before wm push/pull constants and binding table */

      &genX(vs_push_constants), /* Before vs_state */
      &gen7_tcs_push_constants,
      &genX(tes_push_constants),
      &gen6_gs_push_constants, /* Before gs_state */
      &gen6_wm_push_constants, /* Before wm_surfaces and constant_buffer */

      /* Surface state setup.  Must come before the VS/WM unit.  The binding
       * table upload must be last.
       */
      &brw_vs_pull_constants,
      &brw_vs_ubo_surfaces,
      &brw_vs_abo_surfaces,
      &brw_tcs_pull_constants,
      &brw_tcs_ubo_surfaces,
      &brw_tcs_abo_surfaces,
      &brw_tes_pull_constants,
      &brw_tes_ubo_surfaces,
      &brw_tes_abo_surfaces,
      &brw_gs_pull_constants,
      &brw_gs_ubo_surfaces,
      &brw_gs_abo_surfaces,
      &brw_wm_pull_constants,
      &brw_wm_ubo_surfaces,
      &brw_wm_abo_surfaces,
      &gen6_renderbuffer_surfaces,
      &brw_renderbuffer_read_surfaces,
      &brw_texture_surfaces,
      &brw_vs_binding_table,
      &brw_tcs_binding_table,
      &brw_tes_binding_table,
      &brw_gs_binding_table,
      &brw_wm_binding_table,

      &brw_fs_samplers,
      &brw_vs_samplers,
      &brw_tcs_samplers,
      &brw_tes_samplers,
      &brw_gs_samplers,
      &gen8_multisample_state,

      &genX(vs_state),
      &genX(hs_state),
      &genX(te_state),
      &genX(ds_state),
      &genX(gs_state),
      &genX(sol_state),
      &genX(clip_state),
      &genX(raster_state),
      &genX(sbe_state),
      &genX(sf_state),
      &genX(ps_blend),
      &genX(ps_extra),
      &genX(ps_state),
      &genX(depth_stencil_state),
      &genX(wm_state),

      &genX(scissor_state),

      &gen7_depthbuffer,

      &brw_polygon_stipple,
      &brw_polygon_stipple_offset,

      &brw_line_stipple,

      &brw_drawing_rect,

      &gen8_vf_topology,

      &brw_indices,
      &gen8_index_buffer,
      &gen8_vertices,

      &haswell_cut_index,
      &gen8_pma_fix,
   };
#endif

   STATIC_ASSERT(ARRAY_SIZE(render_atoms) <= ARRAY_SIZE(brw->render_atoms));
   brw_copy_pipeline_atoms(brw, BRW_RENDER_PIPELINE,
                           render_atoms, ARRAY_SIZE(render_atoms));

#if GEN_GEN >= 7
   static const struct brw_tracked_state *compute_atoms[] =
   {
      &gen7_l3_state,
      &brw_cs_image_surfaces,
      &gen7_cs_push_constants,
      &brw_cs_pull_constants,
      &brw_cs_ubo_surfaces,
      &brw_cs_abo_surfaces,
      &brw_cs_texture_surfaces,
      &brw_cs_work_groups_surface,
      &brw_cs_samplers,
      &brw_cs_state,
   };

   STATIC_ASSERT(ARRAY_SIZE(compute_atoms) <= ARRAY_SIZE(brw->compute_atoms));
   brw_copy_pipeline_atoms(brw, BRW_COMPUTE_PIPELINE,
                           compute_atoms, ARRAY_SIZE(compute_atoms));
#endif
}
