/*
 * Copyright (C) 2015, YouView TV Ltd
 *   Author: Krzysztof Konopko <kris@youview.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation
 * version 2.1 of the License.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "gstmmalvideosink.h"
#include "gstmmalmemory.h"

#include <gst/video/video.h>
#include <gst/video/gstvideosink.h>

#include <bcm_host.h>

#include <interface/mmal/mmal.h>
#include <interface/mmal/util/mmal_util.h>
#include <interface/mmal/util/mmal_default_components.h>
#include <interface/mmal/util/mmal_util_params.h>

#define MMAL_I420_STRIDE_ALIGN 32
#define MMAL_I420_WIDTH_ALIGN GST_ROUND_UP_32
#define MMAL_I420_HEIGHT_ALIGN GST_ROUND_UP_16

#define MMAL_BUFFER_NUM 3

/*
 * Minimum reasonable video height and width is 1/8 of the screen size.
 */
#define MIN_VIDEO_WIDTH_FRACT (1.0 / 8.0)
#define MIN_VIDEO_HEIGHT_FRACT (1.0 / 8.0)

#define MAX_VIDEO_WIDTH 1920
#define MAX_VIDEO_HEIGHT 1080

#define MAX_I420_RES \
    (MMAL_I420_WIDTH_ALIGN (MAX_VIDEO_WIDTH) * \
     MMAL_I420_HEIGHT_ALIGN (MAX_VIDEO_HEIGHT))

#define MAX_I420_BUFFER_SIZE ((3 * MAX_I420_RES) / 2)

#define GST_MMAL_VIDEO_SINK(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_MMALVIDEOSINK, GstMMALVideoSink))

#define GST_MMAL_VIDEO_SINK_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass), GST_TYPE_MMALVIDEOSINK, GstMMALVideoSinkClass))

typedef struct _GstMMALVideoSink GstMMALVideoSink;
typedef struct _GstMMAVideoSinkClass GstMMALVideoSinkClass;

typedef struct
{
  double x, y;
  double width, height;
} VideoWindow;

struct _GstMMALVideoSink
{
  GstVideoSink sink;

  GstVideoInfo vinfo;
  GstVideoInfo *vinfo_padded;

  MMAL_COMPONENT_T *renderer;
  MMAL_POOL_T *pool;

  /* This is set on caps change.  It tells show_frame() that the port needs
     reconfiguring.  We wait until we see the first frame because this is how
     we discover that opaque mode was selected by upstream.
   */
  gboolean need_reconfigure;

  /* This is used in propose_allocation() */
  GstMMALOpaqueAllocator *allocator;

  /* Whether we are receiving opaque MMAL buffers, rather than plain-old raw
     buffers.  Decided via allocation query negotiation.
   */
  gboolean opaque;

  /* This is required to avoid submitting the same buffer twice which the
     renderer doesn't seem to like.  Only relevant in opaque mode.
   */
  MMAL_BUFFER_HEADER_T *last_mmal_buf;

  gboolean started;

  /* Dimensions of the connected screen */
  uint32_t screen_w, screen_h;

  /* Requested video window */
  VideoWindow window;
};

struct _GstMMAVideoSinkClass
{
  GstVideoSinkClass parent_class;
};

typedef struct
{
  GstBuffer *buffer;
  GstMapInfo map_info;
} MappedBuffer;

enum
{
  PROP_0,

  PROP_DEST_VIDEO_WINDOW_REGION,

  PROP_LAST
};

GST_DEBUG_CATEGORY (mmalvideosink_debug);
#define GST_CAT_DEFAULT mmalvideosink_debug

#define gst_mmalvideosink_parent_class parent_class

G_DEFINE_TYPE (GstMMALVideoSink, gst_mmal_video_sink, GST_TYPE_VIDEO_SINK);

static void video_window_clamp (VideoWindow * vw);

static gboolean mmal_port_supports_format_change (MMAL_PORT_T * port);

static void mmal_control_port_cb (MMAL_PORT_T * port,
    MMAL_BUFFER_HEADER_T * buffer);
static void mmal_input_port_cb (MMAL_PORT_T * port,
    MMAL_BUFFER_HEADER_T * buffer);

static void mmal_scale_rect (const VideoWindow * vw, MMAL_RECT_T * rect);

static gboolean gst_mmal_video_sink_set_render_rectangle (GstMMALVideoSink *
    self);

static void gst_mmal_video_sink_set_format (MMAL_ES_FORMAT_T * format,
    GstVideoInfo * vinfo, gboolean opaque);


static void gst_mmal_video_sink_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec);
static void gst_mmal_video_sink_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec);

static gboolean gst_mmal_video_sink_set_caps (GstBaseSink * sink,
    GstCaps * caps);

static gboolean gst_mmal_video_sink_propose_allocation (GstBaseSink * sink,
    GstQuery * query);

static gboolean gst_mmal_video_sink_start (GstBaseSink * sink);
static gboolean gst_mmal_video_sink_stop (GstBaseSink * sink);

static GstFlowReturn gst_mmal_video_sink_show_frame (GstVideoSink * videosink,
    GstBuffer * buffer);

static gboolean gst_mmal_video_sink_configure_pool (GstMMALVideoSink * self);
static gboolean gst_mmal_video_sink_configure_renderer (GstMMALVideoSink *
    self, gboolean opaque);
static void gst_mmal_video_sink_disable_renderer (GstMMALVideoSink * self);
static gboolean gst_mmal_video_sink_enable_renderer (GstMMALVideoSink * self);

static GstStaticPadTemplate sink_template = GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (GST_VIDEO_CAPS_MAKE_WITH_FEATURES
        (GST_CAPS_FEATURE_MEMORY_MMAL_OPAQUE,
            "{ I420 }") ";" GST_VIDEO_CAPS_MAKE ("{ I420 }")));

static void
gst_mmal_video_sink_class_init (GstMMALVideoSinkClass * klass)
{
  GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
  GstElementClass *element_class = GST_ELEMENT_CLASS (klass);
  GstBaseSinkClass *basesink_class = GST_BASE_SINK_CLASS (klass);
  GstVideoSinkClass *videosink_class = GST_VIDEO_SINK_CLASS (klass);

  gobject_class->set_property =
      GST_DEBUG_FUNCPTR (gst_mmal_video_sink_set_property);
  gobject_class->get_property =
      GST_DEBUG_FUNCPTR (gst_mmal_video_sink_get_property);

  g_object_class_install_property (gobject_class, PROP_DEST_VIDEO_WINDOW_REGION,
      g_param_spec_boxed ("destinationwindow",
          "Destination window",
          "Destination video window region specified as a structure of doubles "
          "(0.0 to 1.0): x, y, width, height",
          GST_TYPE_STRUCTURE, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  gst_element_class_add_pad_template (element_class,
      gst_static_pad_template_get (&sink_template));

  gst_element_class_set_static_metadata (element_class,
      "MMAL Video Sink", "Sink/Video",
      "Displays frames using MMAL", "Krzysztof Konopko <kris@youview.com>");

  GST_DEBUG_CATEGORY_INIT (mmalvideosink_debug, "mmalvideosink", 0,
      "MMAL video sink element");

  basesink_class->set_caps = GST_DEBUG_FUNCPTR (gst_mmal_video_sink_set_caps);
  basesink_class->start = GST_DEBUG_FUNCPTR (gst_mmal_video_sink_start);
  basesink_class->stop = GST_DEBUG_FUNCPTR (gst_mmal_video_sink_stop);
  basesink_class->propose_allocation =
      GST_DEBUG_FUNCPTR (gst_mmal_video_sink_propose_allocation);

  videosink_class->show_frame =
      GST_DEBUG_FUNCPTR (gst_mmal_video_sink_show_frame);
}

/**
 * Video window set as a property may not meet our requirements:
 *
 *   * minimum reasonable video size is 1/8 of the screen size
 *
 *   * video window position should take into account minimum video size
 *
 * Here we clamp video window and the result will be applied and returned as
 * a property value.
 */
static void
video_window_clamp (VideoWindow * vw)
{
  vw->x = CLAMP (vw->x, 0.0, 1.0 - MIN_VIDEO_WIDTH_FRACT);
  vw->y = CLAMP (vw->y, 0.0, 1.0 - MIN_VIDEO_HEIGHT_FRACT);
  vw->width = CLAMP (vw->width, MIN_VIDEO_WIDTH_FRACT, 1.0 - vw->x);
  vw->height = CLAMP (vw->height, MIN_VIDEO_HEIGHT_FRACT, 1.0 - vw->y);
}

static gboolean
mmal_port_supports_format_change (MMAL_PORT_T * port)
{
  return port->capabilities & MMAL_PORT_CAPABILITY_SUPPORTS_EVENT_FORMAT_CHANGE;
}

static void
mmal_control_port_cb (MMAL_PORT_T * port, MMAL_BUFFER_HEADER_T * buffer)
{
  GstMMALVideoSink *self = NULL;
  MMAL_STATUS_T status;

  g_return_if_fail (port != NULL);
  g_return_if_fail (port->userdata != NULL);
  g_return_if_fail (buffer != NULL);

  self = GST_MMAL_VIDEO_SINK (port->userdata);
  GST_TRACE_OBJECT (self, "control port callback");

  if (buffer->cmd == MMAL_EVENT_ERROR) {
    status = *(uint32_t *) buffer->data;
    GST_ERROR_OBJECT (self, "MMAL error: %s (%u)",
        mmal_status_to_string (status), status);
  } else {
    GST_DEBUG_OBJECT (self, "MMAL event on control port: %d", buffer->cmd);
  }

  mmal_buffer_header_release (buffer);
}

/**
 * Video size and position are specified as proportions of the screen size.
 * Here we use those proportions to calculate the actual MMAL video rectangle.
 */
static void
mmal_scale_rect (const VideoWindow * vw, MMAL_RECT_T * rect)
{
  rect->x = rect->width * vw->x;
  rect->y = rect->height * vw->y;
  rect->width *= vw->width;
  rect->height *= vw->height;
}

static void
mmal_input_port_cb (MMAL_PORT_T * port, MMAL_BUFFER_HEADER_T * buffer)
{
  GstMMALVideoSink *self = NULL;

  g_return_if_fail (port != NULL);
  g_return_if_fail (port->userdata != NULL);
  g_return_if_fail (buffer != NULL);

  self = GST_MMAL_VIDEO_SINK (port->userdata);
  GST_TRACE_OBJECT (self, "input port callback");

  if (buffer->cmd != 0) {
    GST_DEBUG_OBJECT (self, "input port event: %u", buffer->cmd);
  } else {

    MappedBuffer *mb = (MappedBuffer *) buffer->user_data;

    if (mb) {

      gst_buffer_unmap (mb->buffer, &mb->map_info);
      gst_buffer_unref (mb->buffer);

      /*
       * Restore original MMAL buffer payload we used for storing buffer info
       */
      buffer->data = (uint8_t *) mb;

      buffer->user_data = NULL;
    }
  }

  mmal_buffer_header_release (buffer);
}

static void
gst_mmal_video_sink_init (GstMMALVideoSink * self)
{
  g_return_if_fail (self != NULL);

  gst_video_info_init (&self->vinfo);
  self->vinfo_padded = NULL;

  self->renderer = NULL;
  self->pool = NULL;

  self->need_reconfigure = TRUE;

  self->allocator = NULL;
  self->opaque = FALSE;
  self->last_mmal_buf = NULL;

  self->started = FALSE;

  self->screen_w = 0;
  self->screen_h = 0;

  self->window.x = 0.0;
  self->window.y = 0.0;
  self->window.width = 1.0;
  self->window.height = 1.0;
}

static void
gst_mmal_video_sink_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec)
{
  GstMMALVideoSink *self = GST_MMAL_VIDEO_SINK (object);

  switch (prop_id) {
    case PROP_DEST_VIDEO_WINDOW_REGION:
      g_value_take_boxed (value,
          gst_structure_new ("destinationwindow",
              "x", G_TYPE_DOUBLE, self->window.x,
              "y", G_TYPE_DOUBLE, self->window.y,
              "width", G_TYPE_DOUBLE, self->window.width,
              "height", G_TYPE_DOUBLE, self->window.height, NULL));
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

static void
gst_mmal_video_sink_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec)
{
  GstMMALVideoSink *self = GST_MMAL_VIDEO_SINK (object);

  switch (prop_id) {
    case PROP_DEST_VIDEO_WINDOW_REGION:
    {
      VideoWindow vw;

      if (gst_structure_get (g_value_get_boxed (value),
              "x", G_TYPE_DOUBLE, &vw.x,
              "y", G_TYPE_DOUBLE, &vw.y,
              "width", G_TYPE_DOUBLE, &vw.width,
              "height", G_TYPE_DOUBLE, &vw.height, NULL)) {

        video_window_clamp (&vw);
        self->window = vw;

        /*
         * Set the video window only when started (requires initialising BCM and
         * querying screen size).  If not started, video window size change is
         * postponed to when the sink is actually started.
         */
        if (self->started && !gst_mmal_video_sink_set_render_rectangle (self)) {
          GST_ERROR_OBJECT (self, "Failed to set render rectangle");
        }
      } else {
        GST_WARNING_OBJECT (object, "Invalid destination window");
      }

      break;
    }

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

static void
gst_mmal_video_sink_set_format (MMAL_ES_FORMAT_T * format, GstVideoInfo * vinfo,
    gboolean opaque)
{
  g_return_if_fail (format != NULL);
  g_return_if_fail (vinfo != NULL);

  format->type = MMAL_ES_TYPE_VIDEO;
  format->encoding = opaque ? MMAL_ENCODING_OPAQUE : MMAL_ENCODING_I420;
  format->es->video.crop.width = GST_VIDEO_INFO_WIDTH (vinfo);
  format->es->video.crop.height = GST_VIDEO_INFO_HEIGHT (vinfo);
  format->es->video.width =
      MMAL_I420_WIDTH_ALIGN (GST_VIDEO_INFO_WIDTH (vinfo));
  format->es->video.height =
      MMAL_I420_HEIGHT_ALIGN (GST_VIDEO_INFO_HEIGHT (vinfo));
  format->es->video.frame_rate.num = GST_VIDEO_INFO_FPS_N (vinfo);
  format->es->video.frame_rate.den = GST_VIDEO_INFO_FPS_D (vinfo);
  format->es->video.par.num = GST_VIDEO_INFO_PAR_N (vinfo);
  format->es->video.par.den = GST_VIDEO_INFO_PAR_D (vinfo);
  format->flags = MMAL_ES_FORMAT_FLAG_FRAMED;
}


static gboolean
gst_mmal_video_sink_propose_allocation (GstBaseSink * sink, GstQuery * query)
{
  GstCaps *caps = NULL;
  GstCapsFeatures *features = NULL;
  gboolean need_pool = FALSE;

  GstMMALVideoSink *self = GST_MMAL_VIDEO_SINK (sink);

  gst_query_parse_allocation (query, &caps, &need_pool);

  GST_DEBUG_OBJECT (self, "Allocation query caps: %" GST_PTR_FORMAT, caps);

  features = gst_caps_get_features (caps, 0);

  if (gst_caps_features_contains (features,
          GST_CAPS_FEATURE_MEMORY_MMAL_OPAQUE)) {

    GST_DEBUG_OBJECT (self, "Opaque MMAL buffers requested.");

    if (self->allocator) {
      GST_DEBUG_OBJECT (self, "Adding opaque allocator...");
      gst_query_add_allocation_param (query, GST_ALLOCATOR (self->allocator),
          NULL);
    } else {
      GST_ERROR_OBJECT (self, "Opaque allocator is NULL!");
    }
  }

  return TRUE;
}


static gboolean
gst_mmal_video_sink_set_caps (GstBaseSink * sink, GstCaps * caps)
{
  GstMMALVideoSink *self = NULL;
  GstVideoInfo vinfo;
  GstVideoAlignment align;
  GstCapsFeatures *features;
  gboolean opaque_caps;

  g_return_val_if_fail (sink != NULL, FALSE);

  self = GST_MMAL_VIDEO_SINK (sink);
  GST_DEBUG_OBJECT (self, "set caps: %" GST_PTR_FORMAT, caps);

  if (!gst_video_info_from_caps (&vinfo, caps)) {
    GST_ERROR_OBJECT (self, "Could not turn caps into video info");
    return FALSE;
  }

  /* N.B. Just for extra fun, we can see a change between opaque and plain
     buffers in some cases.  We won't know for sure if opaque buffers are in
     use until we see the first frame, but we have to assume a change if buffer
     type changes.  Comparison of video info alone will not tell us that, so
     check caps features.
   */
  features = gst_caps_get_features (caps, 0);

  opaque_caps = (features != NULL
      && gst_caps_features_contains (features,
          GST_CAPS_FEATURE_MEMORY_MMAL_OPAQUE));

  if (opaque_caps != self->opaque) {
    GST_DEBUG_OBJECT (self, "Caps indicate a change of buffer type. "
        "Was %s, now %s", (self->opaque ? "Opaque" : "Plain"),
        (opaque_caps ? "Opaque" : "Plain"));
  }

  if (opaque_caps == self->opaque && gst_video_info_is_equal (&vinfo,
          &self->vinfo)) {
    GST_DEBUG_OBJECT (self, "Format is unchanged.  No reconfigure needed.");
    return TRUE;
  }

  self->vinfo = vinfo;
  if (self->vinfo_padded) {
    gst_video_info_free (self->vinfo_padded);
    self->vinfo_padded = NULL;
  }

  gst_video_alignment_reset (&align);

  align.padding_bottom =
      MMAL_I420_HEIGHT_ALIGN (GST_VIDEO_INFO_HEIGHT (&vinfo)) -
      GST_VIDEO_INFO_HEIGHT (&vinfo);
  align.padding_right =
      MMAL_I420_WIDTH_ALIGN (GST_VIDEO_INFO_WIDTH (&vinfo)) -
      GST_VIDEO_INFO_WIDTH (&vinfo);

  align.stride_align[0] = MMAL_I420_STRIDE_ALIGN - 1;
  align.stride_align[1] = (MMAL_I420_STRIDE_ALIGN / 2) - 1;
  align.stride_align[2] = (MMAL_I420_STRIDE_ALIGN / 2) - 1;

  gst_video_info_align (&vinfo, &align);
  if (!gst_video_info_is_equal (&vinfo, &self->vinfo)) {
    GST_VIDEO_INFO_WIDTH (&vinfo) += align.padding_right;
    GST_VIDEO_INFO_HEIGHT (&vinfo) += align.padding_bottom;

    self->vinfo_padded = gst_video_info_copy (&vinfo);
  }

  /* Next time we see an input buffer, we need to configure port.
     We need to wait for this in order to see if we're opaque or not.
   */
  GST_DEBUG_OBJECT (self, "Format changed.  Need reconfigure on next frame.");
  self->need_reconfigure = TRUE;

  return TRUE;
}

static gboolean
gst_mmal_video_sink_start (GstBaseSink * sink)
{
  MMAL_PORT_T *input = NULL;
  MMAL_DISPLAYREGION_T display_region;
  MMAL_STATUS_T status;

  GstMMALVideoSink *self = NULL;

  g_return_val_if_fail (sink != NULL, FALSE);

  self = GST_MMAL_VIDEO_SINK (sink);
  GST_DEBUG_OBJECT (self, "start");

  gst_video_info_init (&self->vinfo);

  bcm_host_init ();

  status =
      mmal_component_create (MMAL_COMPONENT_DEFAULT_VIDEO_RENDERER,
      &self->renderer);

  if (status != MMAL_SUCCESS) {
    GST_ERROR_OBJECT (self,
        "Failed to create MMAL renderer component %s: %s (%u)",
        MMAL_COMPONENT_DEFAULT_VIDEO_RENDERER,
        mmal_status_to_string (status), status);
    return FALSE;
  }

  g_return_val_if_fail (self->renderer != NULL, FALSE);
  g_return_val_if_fail (self->renderer->input != NULL, FALSE);

  input = self->renderer->input[0];
  g_return_val_if_fail (input != NULL, FALSE);

  display_region.hdr.id = MMAL_PARAMETER_DISPLAYREGION;
  display_region.hdr.size = sizeof (MMAL_DISPLAYREGION_T);

  status = mmal_port_parameter_get (input, &display_region.hdr);
  if (status != MMAL_SUCCESS) {
    GST_ERROR_OBJECT (self, "Failed to get display region: %s (%u)",
        mmal_status_to_string (status), status);
    return FALSE;
  }

  if ((display_region.set & MMAL_DISPLAY_SET_NUM) == 0) {
    GST_ERROR_OBJECT (self, "Failed to get display identifier");
    return FALSE;
  }

  /* TODO:  This should be checked every time screen size is changed */
  if (graphics_get_display_size (display_region.display_num,
          &self->screen_w, &self->screen_h) < 0) {
    GST_ERROR_OBJECT (self, "Failed to get display dimensions");
    return FALSE;
  }

  self->allocator = g_object_new (gst_mmal_opaque_allocator_get_type (), NULL);

  self->started = TRUE;

  /* TODO:  This should be called every time screen size is updated */
  if (!gst_mmal_video_sink_set_render_rectangle (self)) {
    GST_ERROR_OBJECT (self, "Failed to set render rectangle");
    return FALSE;
  }

  return TRUE;
}

static gboolean
gst_mmal_video_sink_stop (GstBaseSink * sink)
{
  GstMMALVideoSink *self;

  g_return_val_if_fail (sink != NULL, FALSE);

  self = GST_MMAL_VIDEO_SINK (sink);
  GST_DEBUG_OBJECT (self, "stop");

  self->started = FALSE;

  gst_mmal_video_sink_disable_renderer (self);

  if (self->pool) {
    mmal_port_pool_destroy (self->renderer->input[0], self->pool);
    self->pool = NULL;
  }

  if (self->renderer) {
    mmal_component_release (self->renderer);
    self->renderer = NULL;
  }

  bcm_host_deinit ();

  if (self->vinfo_padded) {
    gst_video_info_free (self->vinfo_padded);
    self->vinfo_padded = NULL;
  }

  g_clear_object (&self->allocator);

  return TRUE;
}

static void
gst_mmal_video_sink_copy_frame_raw (guint8 * dest, GstVideoInfo * dinfo,
    guint8 * src, GstVideoInfo * sinfo)
{
  guint i, n_components;

  g_return_if_fail (dest != NULL && dinfo != NULL);
  g_return_if_fail (src != NULL && sinfo != NULL);

  g_return_if_fail (GST_VIDEO_INFO_SIZE (sinfo) <= GST_VIDEO_INFO_SIZE (dinfo));

  n_components = GST_VIDEO_INFO_N_COMPONENTS (sinfo);

  g_return_if_fail (n_components == GST_VIDEO_INFO_N_COMPONENTS (dinfo));

  for (i = 0; i < n_components; ++i) {
    gint sheight = GST_VIDEO_INFO_COMP_HEIGHT (sinfo, i);

    gsize soffset = GST_VIDEO_INFO_COMP_OFFSET (sinfo, i),
        doffset = GST_VIDEO_INFO_COMP_OFFSET (dinfo, i);

    gint sstride = GST_VIDEO_INFO_COMP_STRIDE (sinfo, i),
        dstride = GST_VIDEO_INFO_COMP_STRIDE (dinfo, i);

    gint j;

    g_return_if_fail (sheight <= GST_VIDEO_INFO_COMP_HEIGHT (dinfo, i));

    for (j = 0; j < sheight; ++j) {
      memcpy (dest + doffset + j * dstride,
          src + soffset + j * sstride, sstride);
    }
  }
}

static GstFlowReturn
gst_mmal_video_sink_show_frame (GstVideoSink * sink, GstBuffer * buffer)
{
  MMAL_STATUS_T status;
  MMAL_BUFFER_HEADER_T *mmal_buf = NULL;

  GstMMALVideoSink *self;

  g_return_val_if_fail (sink != NULL, GST_FLOW_ERROR);
  g_return_val_if_fail (buffer != NULL, GST_FLOW_ERROR);

  self = GST_MMAL_VIDEO_SINK (sink);
  GST_TRACE_OBJECT (sink, "show frame");

  GST_DEBUG_OBJECT (sink, "buffer: %p", buffer);

  /* Has there just been a caps change? */
  if (self->need_reconfigure) {

    /* Work out from buffer whether opaque buffers were negotiated. */
    {
      gboolean opaque =
          gst_is_mmal_opaque_memory (gst_buffer_peek_memory (buffer, 0));

      /* N.B. configure_renderer() cares if we're switching buffer type:
         plain <-> opaque.  It will set new self->opaque value.
       */
      if (!gst_mmal_video_sink_configure_renderer (self, opaque)) {

        GST_ERROR_OBJECT (self, "Reconfigure failed!");
        return GST_FLOW_ERROR;
      }
    }
  }

  if (self->opaque) {

    /* Using opaque buffers. Just extract MMAL buffer header from GstMemory */

    mmal_buf =
        gst_mmal_opaque_mem_get_mmal_header (gst_buffer_peek_memory (buffer,
            0));

    g_return_val_if_fail (mmal_buf != NULL, GST_FLOW_ERROR);

    /* NOTE: In opaque case, we take a reference to the MMAL buffer *each time
       we are asked to render it*.

       The reason for this is that we will otherwise double-free the MMAL
       buffer in the case of pre-roll, where we are asked to present the same
       frame twice.

       The GstBuffer is holding it's own reference to the MMAL buffer, which will
       be dropped when the GstBuffer is returned back to it's pool.  This is
       handled by a specialised MMAL Pool implementation.

       The reference we add here will be removed by our port callback:
       mmal_input_port_cb()
     */
    if (mmal_buf == self->last_mmal_buf) {
      return GST_FLOW_OK;
    } else {
      self->last_mmal_buf = mmal_buf;
    }

    mmal_buffer_header_acquire (mmal_buf);

  } else {

    /* Plain buffers. */

    g_return_val_if_fail (self->pool != NULL, GST_FLOW_ERROR);
    g_return_val_if_fail (self->pool->queue != NULL, GST_FLOW_ERROR);

    mmal_buf = mmal_queue_wait (self->pool->queue);

    g_return_val_if_fail (mmal_buf != NULL, GST_FLOW_ERROR);

    if (self->vinfo_padded) {
      GstMapInfo info;

      if (!gst_buffer_map (buffer, &info, GST_MAP_READ)) {
        goto error_map_buffer;
      }

      gst_mmal_video_sink_copy_frame_raw (mmal_buf->data, self->vinfo_padded,
          info.data, &self->vinfo);

      gst_buffer_unmap (buffer, &info);

      mmal_buf->length = GST_VIDEO_INFO_SIZE (self->vinfo_padded);
      mmal_buf->user_data = NULL;
    } else {
      MappedBuffer *mb = NULL;

      /* Use MMAL buffer payload for mapped GStreamer buffer info */
      mb = (MappedBuffer *) mmal_buf->data;
      g_return_val_if_fail (mb != NULL, GST_FLOW_ERROR);

      if (!gst_buffer_map (buffer, &mb->map_info, GST_MAP_READ)) {
        goto error_map_buffer;
      }

      mb->buffer = gst_buffer_ref (buffer);

      /* Provide GStreamer buffer data without copying */
      mmal_buf->data = mb->map_info.data;
      mmal_buf->length = mb->map_info.size;
      mmal_buf->user_data = mb;
    }

    mmal_buf->pts = GST_BUFFER_PTS_IS_VALID (buffer) ?
        GST_TIME_AS_USECONDS (GST_BUFFER_PTS (buffer)) : MMAL_TIME_UNKNOWN;
    mmal_buf->dts = GST_BUFFER_DTS_IS_VALID (buffer) ?
        GST_TIME_AS_USECONDS (GST_BUFFER_DTS (buffer)) : MMAL_TIME_UNKNOWN;
    mmal_buf->flags = MMAL_BUFFER_HEADER_FLAG_FRAME;
  }

  status = mmal_port_send_buffer (self->renderer->input[0], mmal_buf);
  if (status != MMAL_SUCCESS) {
    GST_ERROR_OBJECT (self, "Failed to send MMAL buffer: %s (%u)",
        mmal_status_to_string (status), status);
    return GST_FLOW_ERROR;
  }

  return GST_FLOW_OK;

error_map_buffer:
  mmal_buffer_header_release (mmal_buf);
  GST_ERROR_OBJECT (self, "Failed to map frame buffer for reading");
  return GST_FLOW_ERROR;
}

static gboolean
gst_mmal_video_sink_configure_pool (GstMMALVideoSink * self)
{
  GstVideoInfo *vinfo = NULL;
  MMAL_PORT_T *input = NULL;
  gsize frame_size;

  g_return_val_if_fail (self != NULL, FALSE);

  vinfo = &self->vinfo;

  g_return_val_if_fail (self->renderer != NULL, FALSE);
  g_return_val_if_fail (self->renderer->input != NULL, FALSE);

  input = self->renderer->input[0];
  g_return_val_if_fail (input != NULL, FALSE);

  /* In opaque case we don't need any pool, but it's possible that we're
     switching from opaque to plain buffers, in which case we should destroy
     any existing pool.
   */
  if (self->opaque) {
    if (self->pool != NULL) {
      GST_DEBUG_OBJECT (self, "Switching to opaque buffers, destroying pool");
      mmal_port_pool_destroy (input, self->pool);
      self->pool = NULL;
    }
    return TRUE;
  }

  /* Rest is for plain buffers. */

  input->buffer_num = MMAL_BUFFER_NUM;
  input->buffer_size = MAX_I420_BUFFER_SIZE;

  frame_size = self->vinfo_padded ?
      GST_VIDEO_INFO_SIZE (self->vinfo_padded) : GST_VIDEO_INFO_SIZE (vinfo);

  if (frame_size > input->buffer_size) {
    GST_ERROR_OBJECT (self,
        "Frame buffer size is greater than MMAL buffer size: %u != %u",
        frame_size, input->buffer_size);
    return FALSE;
  }

  if (self->pool == NULL) {
    GST_DEBUG_OBJECT (self, "Creating input pool...");
    self->pool = mmal_port_pool_create (input, MMAL_BUFFER_NUM,
        MAX_I420_BUFFER_SIZE);
  }

  if (!self->pool) {
    GST_ERROR_OBJECT (self, "Failer to create port buffer pool");
    return FALSE;
  }

  GST_DEBUG_OBJECT (self, "MMAL buffer configuration: num=%u, size=%u",
      input->buffer_num, input->buffer_size);
  GST_DEBUG_OBJECT (self, "GStreamer buffer configuration: size=%u, padding=%s",
      GST_VIDEO_INFO_SIZE (vinfo), self->vinfo_padded ? "yes" : "no");

  return TRUE;
}

static gboolean
gst_mmal_video_sink_configure_renderer (GstMMALVideoSink * self,
    gboolean opaque)
{
  MMAL_PORT_T *input = NULL;
  MMAL_STATUS_T status;
  gboolean configured = FALSE;

  g_return_val_if_fail (self != NULL, FALSE);
  g_return_val_if_fail (self->renderer != NULL, FALSE);
  g_return_val_if_fail (self->renderer->input != NULL, FALSE);

  input = self->renderer->input[0];
  g_return_val_if_fail (input != NULL, FALSE);

  /* We would like to avoid disabling port if at all possible.  This is
     particularly important when playing adaptive streams so we can achieve
     smooth representation changes.  However, if pipeline is doing
     something funky to change buffer type after initial negotiation, this
     requires disable & flush.
   */
  if (!mmal_port_supports_format_change (input)) {

    gst_mmal_video_sink_disable_renderer (self);

  } else if (opaque != self->opaque) {

    GST_DEBUG_OBJECT (self, "Changing buffer type, we have to disable port.");

    if (self->renderer->input[0]->is_enabled) {
      mmal_port_disable (self->renderer->input[0]);
      mmal_port_flush (self->renderer->input[0]);
    }
  }

  self->opaque = opaque;

  GST_DEBUG_OBJECT (self, "Setting format. Opaque? %s",
      (self->opaque ? "yes" : "no"));

  gst_mmal_video_sink_set_format (input->format, &self->vinfo, opaque);

  status = mmal_port_format_commit (input);

  if (status != MMAL_SUCCESS) {
    GST_ERROR_OBJECT (self, "Failed to commit input format: %s (%u)",
        mmal_status_to_string (status), status);
    return FALSE;
  }

  /* N.B. we don't need a pool for opaque, but we should destroy existing one
     if we have it.
   */
  configured = gst_mmal_video_sink_configure_pool (self) &&
      gst_mmal_video_sink_enable_renderer (self);

  self->need_reconfigure = !configured;

  return configured;
}

static void
gst_mmal_video_sink_disable_renderer (GstMMALVideoSink * self)
{
  g_return_if_fail (self != NULL);

  if (self->renderer) {
    g_return_if_fail (self->renderer->control != NULL);

    if (self->renderer->control->is_enabled) {
      mmal_port_disable (self->renderer->control);
    }

    g_return_if_fail (self->renderer->input != NULL);
    g_return_if_fail (self->renderer->input[0] != NULL);

    if (self->renderer->input[0]->is_enabled) {
      mmal_port_disable (self->renderer->input[0]);
    }

    if (self->renderer->is_enabled) {
      mmal_component_disable (self->renderer);
    }
  }
}

static gboolean
gst_mmal_video_sink_enable_renderer (GstMMALVideoSink * self)
{
  MMAL_PORT_T *input = NULL;
  MMAL_STATUS_T status;

  g_return_val_if_fail (self != NULL, FALSE);

  if (self->renderer) {
    g_return_val_if_fail (self->renderer->control != NULL, FALSE);

    if (!self->renderer->control->is_enabled) {
      self->renderer->control->userdata = (struct MMAL_PORT_USERDATA_T *) self;

      status = mmal_port_enable (self->renderer->control, mmal_control_port_cb);
      if (status != MMAL_SUCCESS) {
        GST_ERROR_OBJECT (self,
            "Failed to enable control port %s: %s (%u)",
            self->renderer->control->name, mmal_status_to_string (status),
            status);
        return FALSE;
      }
    }

    g_return_val_if_fail (self->renderer->input != NULL, FALSE);

    input = self->renderer->input[0];
    g_return_val_if_fail (input != NULL, FALSE);

    if (!input->is_enabled) {
      input->userdata = (struct MMAL_PORT_USERDATA_T *) self;

      status = mmal_port_enable (input, mmal_input_port_cb);
      if (status != MMAL_SUCCESS) {
        GST_ERROR_OBJECT (self,
            "Failed to enable input port %s: %s (%u)",
            input->name, mmal_status_to_string (status), status);
        return FALSE;
      }
    }

    if (!self->renderer->is_enabled) {
      status = mmal_component_enable (self->renderer);
      if (status != MMAL_SUCCESS) {
        GST_ERROR_OBJECT (self,
            "Failed to enable renderer component: %s (%u)",
            mmal_status_to_string (status), status);
        return FALSE;
      }
    }
  }

  return TRUE;
}

/**
 * Use obtained screen size and video window size to apply MMAL video rectangle
 * configuration every time either screen size or video window is changed.
 */
static gboolean
gst_mmal_video_sink_set_render_rectangle (GstMMALVideoSink * self)
{
  MMAL_DISPLAYREGION_T display_region;
  MMAL_STATUS_T status;

  g_return_val_if_fail (self != NULL, FALSE);
  g_return_val_if_fail (self->started, FALSE);

  GST_DEBUG_OBJECT (self, "video window: x=%f y=%f w=%f h=%f",
      self->window.x, self->window.y, self->window.width, self->window.height);

  display_region.hdr.id = MMAL_PARAMETER_DISPLAYREGION;
  display_region.hdr.size = sizeof (MMAL_DISPLAYREGION_T);

  display_region.fullscreen = MMAL_FALSE;
  display_region.mode = MMAL_DISPLAY_MODE_FILL;

  display_region.dest_rect.x = 0;
  display_region.dest_rect.y = 0;
  display_region.dest_rect.width = self->screen_w;
  display_region.dest_rect.height = self->screen_h;

  mmal_scale_rect (&self->window, &display_region.dest_rect);
  GST_DEBUG_OBJECT (self, "render rectangle: x=%d y=%d w=%d h=%d",
      display_region.dest_rect.x, display_region.dest_rect.y,
      display_region.dest_rect.width, display_region.dest_rect.height);

  display_region.set =
      MMAL_DISPLAY_SET_FULLSCREEN | MMAL_DISPLAY_SET_MODE |
      MMAL_DISPLAY_SET_DEST_RECT;

  status =
      mmal_port_parameter_set (self->renderer->input[0], &display_region.hdr);
  if (status != MMAL_SUCCESS) {
    GST_ERROR_OBJECT (self, "Failed to set display region: %s (%u)",
        mmal_status_to_string (status), status);
    return FALSE;
  }

  return TRUE;
}
