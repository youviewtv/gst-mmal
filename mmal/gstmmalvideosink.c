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

#include <gst/video/video.h>
#include <gst/video/gstvideosink.h>

#include <bcm_host.h>

#include <interface/mmal/mmal.h>
#include <interface/mmal/util/mmal_util.h>
#include <interface/mmal/util/mmal_default_components.h>

#define ARBITRARY_BUFFER_NUM 5

#define GST_MMAL_VIDEO_SINK(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_MMALVIDEOSINK, GstMMALVideoSink))

#define GST_MMAL_VIDEO_SINK_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass), GST_TYPE_MMALVIDEOSINK, GstMMALVideoSinkClass))

typedef struct _GstMMALVideoSink GstMMALVideoSink;
typedef struct _GstMMAVideoSinkClass GstMMALVideoSinkClass;

struct _GstMMALVideoSink
{
  GstVideoSink sink;

  GstVideoInfo info;

  MMAL_COMPONENT_T *renderer;
  MMAL_POOL_T *pool;
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

GST_DEBUG_CATEGORY (mmalvideosink_debug);
#define GST_CAT_DEFAULT mmalvideosink_debug

#define gst_mmalvideosink_parent_class parent_class
G_DEFINE_TYPE (GstMMALVideoSink, gst_mmal_video_sink, GST_TYPE_VIDEO_SINK);

static void mmal_control_port_cb (MMAL_PORT_T * port,
    MMAL_BUFFER_HEADER_T * buffer);
static void mmal_input_port_cb (MMAL_PORT_T * port,
    MMAL_BUFFER_HEADER_T * buffer);

static gboolean gst_mmal_video_sink_set_caps (GstBaseSink * sink,
    GstCaps * caps);
static gboolean gst_mmal_video_sink_start (GstBaseSink * sink);
static gboolean gst_mmal_video_sink_stop (GstBaseSink * sink);

static GstFlowReturn gst_mmal_video_sink_show_frame (GstVideoSink * videosink,
    GstBuffer * buffer);

static gboolean gst_mmal_video_sink_configure_renderer (GstMMALVideoSink *
    self);
static void gst_mmal_video_sink_disable_renderer (GstMMALVideoSink * self);

static GstStaticPadTemplate sink_template = GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (GST_VIDEO_CAPS_MAKE ("I420"))
    );

static void
gst_mmal_video_sink_class_init (GstMMALVideoSinkClass * klass)
{
  GstElementClass *element_class = GST_ELEMENT_CLASS (klass);
  GstBaseSinkClass *basesink_class = GST_BASE_SINK_CLASS (klass);
  GstVideoSinkClass *videosink_class = GST_VIDEO_SINK_CLASS (klass);

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

  videosink_class->show_frame =
      GST_DEBUG_FUNCPTR (gst_mmal_video_sink_show_frame);
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
    GST_DEBUG_OBJECT (self, "Input port event: %u", buffer->cmd);
  } else {
    MappedBuffer *mb = (MappedBuffer *) buffer->user_data;

    g_return_if_fail (mb != NULL);

    gst_buffer_unmap (mb->buffer, &mb->map_info);
    gst_buffer_unref (mb->buffer);

    /*
     * Restore original MMAL buffer payload we used for storing buffer info
     */
    buffer->data = mb;
    buffer->length = buffer->alloc_size = sizeof (*mb);
  }

  mmal_buffer_header_release (buffer);
}

static void
gst_mmal_video_sink_init (GstMMALVideoSink * self)
{
  g_return_if_fail (self != NULL);

  gst_video_info_init (&self->info);

  self->renderer = NULL;
  self->pool = NULL;
}

static gboolean
gst_mmal_video_sink_set_caps (GstBaseSink * sink, GstCaps * caps)
{
  GstMMALVideoSink *self = NULL;
  GstVideoInfo info_new;

  g_return_val_if_fail (sink != NULL, FALSE);

  self = GST_MMAL_VIDEO_SINK (sink);
  GST_DEBUG_OBJECT (self, "set caps: %" GST_PTR_FORMAT, caps);

  if (!gst_video_info_from_caps (&info_new, caps)) {
    GST_ERROR_OBJECT (self, "Could not turn caps into video info");
    return FALSE;
  }

  if (gst_video_info_is_equal (&info_new, &self->info)) {
    return TRUE;
  }

  self->info = info_new;

  return gst_mmal_video_sink_configure_renderer (self);
}

static gboolean
gst_mmal_video_sink_start (GstBaseSink * sink)
{
  MMAL_STATUS_T status;
  MMAL_PORT_T *input = NULL;

  GstMMALVideoSink *self = NULL;

  g_return_val_if_fail (sink != NULL, FALSE);

  self = GST_MMAL_VIDEO_SINK (sink);
  GST_DEBUG_OBJECT (self, "start");

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

  self->pool = mmal_port_pool_create (input,
      /*
       * This really simplifies things as we don't need to reallocate/resize the
       * pool and do all the buffers-in-transit dance (would need to wait for
       * buffers in use to be returned?).  Maybe this is not so complicated and
       * we could still use suggested buffer_num on input port?  Also depends on
       * how we use buffer payload (see below).
       */
      ARBITRARY_BUFFER_NUM,
      /*
       * Here we use MMAL buffer payload for storing mapped GStreamer buffer
       * information as we pass GStreamer buffer data directly to MMAL without
       * copying it.
       */
      sizeof (MappedBuffer));
  if (!self->pool) {
    GST_ERROR_OBJECT (self, "Failer to create port buffer pool");
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

  gst_mmal_video_sink_disable_renderer (self);


  if (self->renderer) {

    if (self->pool) {
      MMAL_PORT_T *input = NULL;

      g_return_val_if_fail (self->renderer->input != NULL, FALSE);

      input = self->renderer->input[0];
      g_return_val_if_fail (input != NULL, FALSE);

      mmal_port_pool_destroy (input, self->pool);
    }

    mmal_component_release (self->renderer);
  }

  bcm_host_deinit ();

  return TRUE;
}

static GstFlowReturn
gst_mmal_video_sink_show_frame (GstVideoSink * sink, GstBuffer * buffer)
{
  MMAL_STATUS_T status;
  MappedBuffer *mb = NULL;
  MMAL_BUFFER_HEADER_T *mmal_buf = NULL;

  GstMMALVideoSink *self;

  g_return_val_if_fail (sink != NULL, GST_FLOW_ERROR);
  g_return_val_if_fail (buffer != NULL, GST_FLOW_ERROR);

  self = GST_MMAL_VIDEO_SINK (sink);
  GST_TRACE_OBJECT (sink, "show frame");

  g_return_val_if_fail (self->pool != NULL, GST_FLOW_ERROR);
  g_return_val_if_fail (self->pool->queue != NULL, GST_FLOW_ERROR);

  mmal_buf = mmal_queue_wait (self->pool->queue);
  g_return_val_if_fail (mmal_buf != NULL, GST_FLOW_ERROR);

  /* Use MMAL buffer payload for mapped GStreamer buffer info */
  mb = (MappedBuffer *) mmal_buf->data;
  g_return_val_if_fail (mb != NULL, GST_FLOW_ERROR);

  mb->buffer = gst_buffer_ref (buffer);

  if (!gst_buffer_map (mb->buffer, &mb->map_info, GST_MAP_READ)) {
    gst_buffer_unref (mb->buffer);
    mmal_buffer_header_release (mmal_buf);
    GST_ERROR_OBJECT (self, "Failed to map frame buffer for reading");
    return GST_FLOW_ERROR;
  }

  /* Provide GStreamer buffer data without copying */
  mmal_buf->data = mb->map_info.data;
  mmal_buf->length = mb->map_info.size;
  mmal_buf->alloc_size = mmal_buf->length;
  mmal_buf->pts = GST_BUFFER_PTS_IS_VALID (buffer) ?
      GST_TIME_AS_USECONDS (GST_BUFFER_PTS (buffer)) : MMAL_TIME_UNKNOWN;
  mmal_buf->dts = GST_BUFFER_DTS_IS_VALID (buffer) ?
      GST_TIME_AS_USECONDS (GST_BUFFER_DTS (buffer)) : MMAL_TIME_UNKNOWN;
  mmal_buf->user_data = mb;

  status = mmal_port_send_buffer (self->renderer->input[0], mmal_buf);
  if (status != MMAL_SUCCESS) {
    GST_ERROR_OBJECT (self, "Failed to send MMAL buffer: %s (%u)",
        mmal_status_to_string (status), status);
    return GST_FLOW_ERROR;
  }

  return GST_FLOW_OK;
}

static gboolean
gst_mmal_video_sink_configure_renderer (GstMMALVideoSink * self)
{
  MMAL_PORT_T *input = NULL;
  MMAL_ES_FORMAT_T *format = NULL;
  MMAL_DISPLAYREGION_T display_region;

  GstVideoInfo *info = NULL;

  MMAL_STATUS_T status;

  g_return_val_if_fail (self != NULL, FALSE);
  g_return_val_if_fail (self->renderer != NULL, FALSE);
  g_return_val_if_fail (self->renderer->input != NULL, FALSE);

  input = self->renderer->input[0];
  g_return_val_if_fail (input != NULL, FALSE);

  format = input->format;
  g_return_val_if_fail (format != NULL, FALSE);

  gst_mmal_video_sink_disable_renderer (self);

  info = &self->info;

  format->type = MMAL_ES_TYPE_VIDEO;
  format->encoding = MMAL_ENCODING_I420;
  format->es->video.crop.width = GST_VIDEO_INFO_WIDTH (info);
  format->es->video.crop.height = GST_VIDEO_INFO_HEIGHT (info);
  format->es->video.width = GST_VIDEO_INFO_WIDTH (info);
  format->es->video.height = GST_VIDEO_INFO_HEIGHT (info);
  format->es->video.frame_rate.num = GST_VIDEO_INFO_FPS_N (info);
  format->es->video.frame_rate.den = GST_VIDEO_INFO_FPS_D (info);
  format->es->video.par.num = GST_VIDEO_INFO_PAR_N (info);
  format->es->video.par.den = GST_VIDEO_INFO_PAR_D (info);
  format->flags = MMAL_ES_FORMAT_FLAG_FRAMED;

  status = mmal_port_format_commit (input);
  if (status != MMAL_SUCCESS) {
    GST_ERROR_OBJECT (self, "Failed to commit input format: %s (%u)",
        mmal_status_to_string (status), status);
    return FALSE;
  }

  input->buffer_num = ARBITRARY_BUFFER_NUM;
  /*
   * We expect that given the format and its parameters are known, GStreamer
   * will provide frame buffers of at least this size.
   */
  input->buffer_size = input->buffer_size_min;

  GST_DEBUG_OBJECT (self, "buffer config: num=%u, size=%u",
      input->buffer_num, input->buffer_size);

  display_region.hdr.id = MMAL_PARAMETER_DISPLAYREGION;
  display_region.hdr.size = sizeof (MMAL_DISPLAYREGION_T);
  display_region.fullscreen = MMAL_TRUE;
  display_region.mode = MMAL_DISPLAY_MODE_FILL;
  display_region.set = MMAL_DISPLAY_SET_FULLSCREEN | MMAL_DISPLAY_SET_MODE;
  status = mmal_port_parameter_set (input, &display_region.hdr);
  if (status != MMAL_SUCCESS) {
    GST_ERROR_OBJECT (self, "Failed to set display region: %s (%u)",
        mmal_status_to_string (status), status);
    return FALSE;
  }

  self->renderer->control->userdata = (struct MMAL_PORT_USERDATA_T *) self;

  status = mmal_port_enable (self->renderer->control, mmal_control_port_cb);
  if (status != MMAL_SUCCESS) {
    GST_ERROR_OBJECT (self,
        "Failed to enable control port %s: %s (%u)",
        self->renderer->control->name, mmal_status_to_string (status), status);
    return FALSE;
  }

  input->userdata = (struct MMAL_PORT_USERDATA_T *) self;

  status = mmal_port_enable (input, mmal_input_port_cb);
  if (status != MMAL_SUCCESS) {
    GST_ERROR_OBJECT (self,
        "Failed to enable input port %s: %s (%u)",
        input->name, mmal_status_to_string (status), status);
    return FALSE;
  }

  status = mmal_component_enable (self->renderer);
  if (status != MMAL_SUCCESS) {
    GST_ERROR_OBJECT (self,
        "Failed to enable renderer component: %s (%u)",
        mmal_status_to_string (status), status);
    return FALSE;
  }

  return TRUE;
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
