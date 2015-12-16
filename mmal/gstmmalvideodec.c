/*
 * Copyright (C) 2015, YouView TV Ltd.
 *   Author: John Sadler <john.sadler@youview.com>
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

#include "gstmmalvideodec.h"

#include <gst/gst.h>

#include <string.h>
#include <stdio.h>

#include <bcm_host.h>

#include <interface/mmal/util/mmal_default_components.h>


GST_DEBUG_CATEGORY_STATIC (gst_mmal_video_dec_debug_category);
#define GST_CAT_DEFAULT gst_mmal_video_dec_debug_category

#define MMAL_TICKS_PER_SECOND 1000000
#define GST_MMAL_VIDEO_DEC_EXTRA_OUTPUT_BUFFERS 3


static const char gst_mmal_video_dec_src_caps_str[] =
GST_VIDEO_CAPS_MAKE ("{ I420 }");

static GstStaticPadTemplate gst_mmal_video_dec_src_factory =
GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (gst_mmal_video_dec_src_caps_str));

/* prototypes */


/*
From GstVideoDecoder:

The bare minimum that a functional subclass needs to implement is:

Provide pad templates

Inform the base class of output caps via gst_video_decoder_set_output_state

Parse input data, if it is not considered packetized from upstream Data will be
provided to parse which should invoke gst_video_decoder_add_to_frame and
gst_video_decoder_have_frame to separate the data belonging to each video frame.

Accept data in handle_frame and provide decoded results to
gst_video_decoder_finish_frame, or call gst_video_decoder_drop_frame.
*/

static void gst_mmal_video_dec_finalize (GObject * object);

static GstStateChangeReturn
gst_mmal_video_dec_change_state (GstElement * element,
    GstStateChange transition);

static gboolean gst_mmal_video_dec_open (GstVideoDecoder * decoder);
static gboolean gst_mmal_video_dec_close (GstVideoDecoder * decoder);
static gboolean gst_mmal_video_dec_start (GstVideoDecoder * decoder);
static gboolean gst_mmal_video_dec_stop (GstVideoDecoder * decoder);
static gboolean gst_mmal_video_dec_set_format (GstVideoDecoder * decoder,
    GstVideoCodecState * state);
static gboolean gst_mmal_video_dec_flush (GstVideoDecoder * decoder);
static GstFlowReturn gst_mmal_video_dec_handle_frame (GstVideoDecoder * decoder,
    GstVideoCodecFrame * frame);

//static GstFlowReturn gst_mmal_video_dec_finish (GstVideoDecoder * decoder);

//static GstFlowReturn gst_mmal_video_dec_drain (GstMMALVideoDec * self);

static GstVideoCodecFrame
    * gst_mmal_video_dec_find_nearest_frame (MMAL_BUFFER_HEADER_T * buf,
    GList * frames);

static gboolean gst_mmal_video_dec_update_src_caps (GstMMALVideoDec * self);

static GstVideoFormat gst_mmal_video_get_format_from_mmal (MMAL_FOURCC_T
    mmal_colorformat);

/* class initialisation */

#define DEBUG_INIT \
  GST_DEBUG_CATEGORY_INIT (gst_mmal_video_dec_debug_category, "mmalvideodec", 0, \
      "debug category for gst-mmal video decoder base class");


G_DEFINE_ABSTRACT_TYPE_WITH_CODE (GstMMALVideoDec, gst_mmal_video_dec,
    GST_TYPE_VIDEO_DECODER, DEBUG_INIT);


static void
gst_mmal_video_dec_class_init (GstMMALVideoDecClass * klass)
{
  GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
  GstElementClass *element_class = GST_ELEMENT_CLASS (klass);
  GstVideoDecoderClass *video_decoder_class = GST_VIDEO_DECODER_CLASS (klass);
  GstPadTemplate *pad_template;

  gobject_class->finalize = gst_mmal_video_dec_finalize;

  element_class->change_state =
      GST_DEBUG_FUNCPTR (gst_mmal_video_dec_change_state);

  video_decoder_class->open = GST_DEBUG_FUNCPTR (gst_mmal_video_dec_open);
  video_decoder_class->close = GST_DEBUG_FUNCPTR (gst_mmal_video_dec_close);
  video_decoder_class->start = GST_DEBUG_FUNCPTR (gst_mmal_video_dec_start);
  video_decoder_class->stop = GST_DEBUG_FUNCPTR (gst_mmal_video_dec_stop);
  video_decoder_class->flush = GST_DEBUG_FUNCPTR (gst_mmal_video_dec_flush);

  /* Optional. Called to request subclass to dispatch any pending remaining data
     at EOS. Sub-classes can refuse to decode new data after.
   */
  // video_decoder_class->finish = GST_DEBUG_FUNCPTR (gst_mmal_video_dec_finish);

  /* GstVideoDecoder vmethods */
  video_decoder_class->set_format =
      GST_DEBUG_FUNCPTR (gst_mmal_video_dec_set_format);

  video_decoder_class->handle_frame =
      GST_DEBUG_FUNCPTR (gst_mmal_video_dec_handle_frame);

  /* src pad  N.B. Sink is done in derived class, since it knows format. */
  pad_template = gst_static_pad_template_get (&gst_mmal_video_dec_src_factory);
  gst_element_class_add_pad_template (element_class, pad_template);

}

static void
gst_mmal_video_dec_init (GstMMALVideoDec * self)
{
  gst_video_decoder_set_packetized (GST_VIDEO_DECODER (self), TRUE);

  self->dec = NULL;
  self->input_state = NULL;
  self->codec_data = NULL;
  self->decoded_frames_queue = NULL;
  self->input_buffer_pool = NULL;
  self->output_buffer_pool = NULL;
  self->started = FALSE;
  self->last_upstream_ts = 0;

}

static gboolean
gst_mmal_video_dec_open (GstVideoDecoder * decoder)
{
  GstMMALVideoDec *self = GST_MMAL_VIDEO_DEC (decoder);

  GST_DEBUG_OBJECT (self, "Opening decoder");

  bcm_host_init ();

  // Create the video decoder component on VideoCore
  if (mmal_component_create (MMAL_COMPONENT_DEFAULT_VIDEO_DECODER, &self->dec)
      != MMAL_SUCCESS) {
    GST_ERROR_OBJECT (self, "Failed to create MMAL decoder component.");
    return FALSE;
  }

  self->decoded_frames_queue = mmal_queue_create ();
  self->dec->output[0]->userdata = (void *) self->decoded_frames_queue;

  self->started = FALSE;

  GST_DEBUG_OBJECT (self, "Opened decoder");

  return TRUE;
}

static gboolean
gst_mmal_video_dec_close (GstVideoDecoder * decoder)
{
  GstMMALVideoDec *self = GST_MMAL_VIDEO_DEC (decoder);

  GST_DEBUG_OBJECT (self, "Closing decoder");

  mmal_component_destroy (self->dec);
  mmal_queue_destroy (self->decoded_frames_queue);

  bcm_host_deinit ();

  self->dec = NULL;
  self->decoded_frames_queue = NULL;
  self->input_buffer_pool = NULL;
  self->output_buffer_pool = NULL;
  self->last_upstream_ts = 0;

  self->started = FALSE;

  GST_DEBUG_OBJECT (self, "Closed decoder");

  return TRUE;
}

static void
gst_mmal_video_dec_finalize (GObject * object)
{
  //GstMMALVideoDec *self = GST_MMAL_VIDEO_DEC (object);

  G_OBJECT_CLASS (gst_mmal_video_dec_parent_class)->finalize (object);
}

static GstStateChangeReturn
gst_mmal_video_dec_change_state (GstElement * element,
    GstStateChange transition)
{
  GstMMALVideoDec *self;
  GstStateChangeReturn ret = GST_STATE_CHANGE_SUCCESS;

  g_return_val_if_fail (GST_IS_MMAL_VIDEO_DEC (element),
      GST_STATE_CHANGE_FAILURE);
  self = GST_MMAL_VIDEO_DEC (element);

  switch (transition) {
    case GST_STATE_CHANGE_NULL_TO_READY:
      break;
    case GST_STATE_CHANGE_READY_TO_PAUSED:
      break;
    case GST_STATE_CHANGE_PAUSED_TO_PLAYING:
      break;
    case GST_STATE_CHANGE_PAUSED_TO_READY:

      // TODO: MMAL Equivalent of gst_omx_port_set_flushing() ?

      break;
    default:
      break;
  }

  if (ret == GST_STATE_CHANGE_FAILURE)
    return ret;

  /* Delegate to baseclass impl */
  ret =
      GST_ELEMENT_CLASS (gst_mmal_video_dec_parent_class)->change_state
      (element, transition);

  if (ret == GST_STATE_CHANGE_FAILURE)
    return ret;

  switch (transition) {
    case GST_STATE_CHANGE_PLAYING_TO_PAUSED:
      break;
    case GST_STATE_CHANGE_PAUSED_TO_READY:
      self->started = FALSE;
      // TODO: MMAL shutdown without close?
//      if (!gst_omx_video_dec_shutdown (self))
//        ret = GST_STATE_CHANGE_FAILURE;
      break;
    case GST_STATE_CHANGE_READY_TO_NULL:
      break;
    default:
      break;
  }

  return ret;
}

static gboolean
gst_mmal_video_dec_start (GstVideoDecoder * decoder)
{
  GstMMALVideoDec *self = GST_MMAL_VIDEO_DEC (decoder);

  self->last_upstream_ts = 0;

  self->started = TRUE;

  return TRUE;
}

static gboolean
gst_mmal_video_dec_stop (GstVideoDecoder * decoder)
{
  GstMMALVideoDec *self;

  self = GST_MMAL_VIDEO_DEC (decoder);

  GST_DEBUG_OBJECT (self, "Stopping decoder");

  // TODO: Equiv of omx set_flushing()

  self->started = FALSE;

  gst_buffer_replace (&self->codec_data, NULL);

  if (self->input_state) {
    gst_video_codec_state_unref (self->input_state);
    self->input_state = NULL;
  }

  GST_DEBUG_OBJECT (self, "Stopped decoder");

  return TRUE;
}



/**
 * This callback is called by MMAL when it's finished with an input buffer.
 * It's associated with the input port at the time the port is enabled.
 */
static void
gst_mmal_video_dec_return_input_buffer_to_pool (MMAL_PORT_T * port,
    MMAL_BUFFER_HEADER_T * buffer)
{
  // This drops the refcount of the buffer header.  It will be returned to it's
  // pool when the refcount reaches zero.
  mmal_buffer_header_release (buffer);
}


/**
 * This callback is called by MMAL when a frame has been decoded.
 * It's associated with the output port at the time the port is enabled.
 */
static void
gst_mmal_video_dec_queue_decoded_frame (MMAL_PORT_T * port,
    MMAL_BUFFER_HEADER_T * buffer)
{
  MMAL_QUEUE_T *queue = (MMAL_QUEUE_T *) port->userdata;
  mmal_queue_put (queue, buffer);
}



/**
 * Bare-bones mapping of supported color formats to GST.  Right now, we will
 * only support I420.
 *
 * TODO: This function needs expanding for other formats, and should be moved to
 * a shared source file.
 */
static GstVideoFormat
gst_mmal_video_get_format_from_mmal (MMAL_FOURCC_T mmal_colorformat)
{
  GstVideoFormat format;

  switch (mmal_colorformat) {
    case MMAL_ENCODING_I420:
      format = GST_VIDEO_FORMAT_I420;
      break;

      /* TODO: Support other formats.  Opaque format, for instance. */
    default:
      format = GST_VIDEO_FORMAT_UNKNOWN;
      break;
  }

  return format;
}


/**
 * Right now, this is called from set_format(), but in the future may be called
 * from another function that is just handling format changes on the decoder
 * output port.
 *
 * TODO: Locking?  omx impl does: GST_VIDEO_DECODER_STREAM_UNLOCK (self);
 * Not sure if that is neccessary in the context this is called?
 */
static gboolean
gst_mmal_video_dec_update_src_caps (GstMMALVideoDec * self)
{
  GstVideoCodecState *state = NULL;
  MMAL_ES_FORMAT_T *output_format = self->dec->output[0]->format;

  GstVideoFormat format =
      gst_mmal_video_get_format_from_mmal (output_format->encoding);

  if (format == GST_VIDEO_FORMAT_UNKNOWN) {

    GST_ERROR_OBJECT (self, "Unsupported color format: %lu",
        (unsigned long) output_format->encoding);

    goto error;
  }

  state = gst_video_decoder_set_output_state (GST_VIDEO_DECODER (self),
      format, output_format->es->video.width,
      output_format->es->video.height, self->input_state);

  if (!gst_video_decoder_negotiate (GST_VIDEO_DECODER (self))) {
    gst_video_codec_state_unref (state);
    GST_ERROR_OBJECT (self, "Failed to negotiate");
    goto error;
  }

  return TRUE;

error:
  /* N.B. would release stream lock here, if acquired. */
  return FALSE;
}

/**
 * This is called by the GstVideoDecoder baseclass to let us know the input
 * format has changed.
 */
static gboolean
gst_mmal_video_dec_set_format (GstVideoDecoder * decoder,
    GstVideoCodecState * state)
{
  GstMMALVideoDec *self;
  GstMMALVideoDecClass *klass;
  GstVideoInfo *info = &state->info;
  gboolean is_format_change = FALSE;
  MMAL_PORT_T *input_port = NULL;
  MMAL_ES_FORMAT_T *input_format = NULL;
  MMAL_PORT_T *output_port = NULL;
  MMAL_ES_FORMAT_T *output_format = NULL;

  self = GST_MMAL_VIDEO_DEC (decoder);
  klass = GST_MMAL_VIDEO_DEC_GET_CLASS (decoder);

  GST_DEBUG_OBJECT (self, "Setting new caps %" GST_PTR_FORMAT, state->caps);

  input_port = self->dec->input[0];

  input_format = input_port->format;

  /* Check if the caps change is a real format change or if only irrelevant
   * parts of the caps have changed or nothing at all.
   */
  is_format_change |= input_format->es->video.width != info->width;
  is_format_change |= input_format->es->video.height != info->height;

  is_format_change |=
      (input_format->es->video.frame_rate.num == 0 && info->fps_n != 0) ||
      (input_format->es->video.frame_rate.num !=
      (info->fps_n << 16) / (info->fps_d));

  is_format_change |= (self->codec_data != state->codec_data);

  /* Ask derived class as well */
  if (klass->is_format_change) {
    is_format_change |= klass->is_format_change (self, input_port, state);
  }

  if (is_format_change) {

    GST_INFO_OBJECT (self, "The input format has changed.  Reconfiguring...");

    gst_buffer_replace (&self->codec_data, state->codec_data);

    if (self->input_state != NULL) {
      gst_video_codec_state_unref (self->input_state);
      self->input_state = NULL;
    }

    self->input_state = gst_video_codec_state_ref (state);

    /* Do we need to disable the component? */

    /* Drain & Flush ? */

    output_port = self->dec->output[0];

    input_format->es->video.width = info->width;
    input_format->es->video.height = info->height;
    input_format->es->video.frame_rate.num =
        ((info->fps_n << 16) / (info->fps_d));
    input_format->es->video.frame_rate.den = info->fps_d;
    input_format->es->video.par.num = info->par_n;
    input_format->es->video.par.den = info->par_d;
    input_format->type = MMAL_ES_TYPE_VIDEO;
    input_format->flags = MMAL_ES_FORMAT_FLAG_FRAMED;

    /* What to stick in here? I'm going to assume it's codec_data */
    input_format->extradata_size = 0;

    if (self->codec_data != NULL) {

      /* FIXME: VLC sources suggest that this needs to be reset after flush. */
      input_format->extradata_size = gst_buffer_get_size (self->codec_data);

      if (mmal_format_extradata_alloc (input_format,
              input_format->extradata_size) != MMAL_SUCCESS) {

        GST_ERROR_OBJECT (self, "Failed to allocate space for extradata!");
        return FALSE;
      }

      gst_buffer_extract (self->codec_data, 0, input_format->extradata,
          gst_buffer_get_size (self->codec_data));
    }

    /* Derived class should set the encoding here. */
    if (klass->set_format) {
      if (!klass->set_format (self, input_port, state)) {
        GST_ERROR_OBJECT (self, "Subclass failed to set the new format");
        return FALSE;
      }
    }


    /* If ports already enabled, then I guess we need to disable them.
       IIRC, VLC code first tries setting format without disable, then
       falls-back to disabling if that fails. */

    if (input_port->is_enabled &&
        mmal_port_disable (input_port) != MMAL_SUCCESS) {

      GST_ERROR_OBJECT (self, "Failed to disable input port!");
      return FALSE;
    }

    if (output_port->is_enabled &&
        mmal_port_disable (output_port) != MMAL_SUCCESS) {
      GST_ERROR_OBJECT (self, "Failed to disable output port!");
      return FALSE;
    }

    if (mmal_port_format_commit (input_port) != MMAL_SUCCESS) {

      GST_ERROR_OBJECT (self, "Failed to commit new input port format!");
      return FALSE;
    }

    /* Assuming we've provided all the required format information, the decoder
       should now have decided on the output format.
       If we haven't then we will be told the output format via an event (ala
       OMX PortSettingsChange). It might turn out to be necessary to do things
       that way such that we can detect interlace and tell downstream, for
       instance, but let's try and keep it simple to being with.
     */
    output_format = output_port->format;

    if (MMAL_ENCODING_UNKNOWN == output_format->encoding) {
      GST_ERROR_OBJECT (self, "Decoder failed to determine output format!");
      return FALSE;
    }

    if (!gst_mmal_video_dec_update_src_caps (self)) {
      GST_ERROR_OBJECT (self, "Failed to update src caps!");
      return FALSE;
    }

    input_port->buffer_num = input_port->buffer_num_recommended;
    input_port->buffer_size = input_port->buffer_size_recommended;

    /* An existing pool can be resized */

    if (self->input_buffer_pool == NULL) {

      self->input_buffer_pool = mmal_pool_create (input_port->buffer_num,
          input_port->buffer_size);

    } else if (mmal_pool_resize (self->input_buffer_pool,
            input_port->buffer_num, input_port->buffer_size) != MMAL_SUCCESS) {

      GST_ERROR_OBJECT (self, "Failed to resize input buffer pool!");
      return FALSE;
    }


    output_port->buffer_num = output_port->buffer_num_recommended +
        GST_MMAL_VIDEO_DEC_EXTRA_OUTPUT_BUFFERS;

    output_port->buffer_size = output_port->buffer_size_recommended;

    if (self->output_buffer_pool == NULL) {

      self->output_buffer_pool = mmal_pool_create (output_port->buffer_num,
          output_port->buffer_size);

    } else if (mmal_pool_resize (self->output_buffer_pool,
            output_port->buffer_num,
            output_port->buffer_size) != MMAL_SUCCESS) {

      GST_ERROR_OBJECT (self, "Failed to resize output buffer pool!");
      return FALSE;
    }

    if (mmal_port_enable (input_port,
            &gst_mmal_video_dec_return_input_buffer_to_pool) != MMAL_SUCCESS) {

      GST_ERROR_OBJECT (self, "Failed to enable input port!");
      return FALSE;
    }

    if (mmal_port_enable (output_port,
            &gst_mmal_video_dec_queue_decoded_frame) != MMAL_SUCCESS) {

      GST_ERROR_OBJECT (self, "Failed to enable output port!");
      return FALSE;
    }

    if (mmal_component_enable (self->dec) != MMAL_SUCCESS) {

      GST_ERROR_OBJECT (self, "Failed to enable decoder component!");
      return FALSE;
    }

  }
  /* end-if (is_format_change) */
  return TRUE;

}


static gboolean
gst_mmal_video_dec_flush (GstVideoDecoder * decoder)
{
  GstMMALVideoDec *self = GST_MMAL_VIDEO_DEC (decoder);
  MMAL_PORT_T *input_port = NULL;
  MMAL_PORT_T *output_port = NULL;

  GST_INFO_OBJECT (self, "flushing...");

  if (!self->dec) {
    GST_ERROR_OBJECT (self, "Decoder component not yet created!");
    return TRUE;
  }

  input_port = self->dec->input[0];
  output_port = self->dec->output[0];

  if (mmal_port_flush (input_port) != MMAL_SUCCESS) {
    GST_ERROR_OBJECT (self, "Failed to set flushing on input port!");
    return FALSE;
  }


  if (mmal_port_flush (output_port) != MMAL_SUCCESS) {
    GST_ERROR_OBJECT (self, "Failed to set flushing on output port!");
    return FALSE;
  }

  /* TODO: Do something to flush buffers ? */

  return TRUE;
}


/**
 * This is lifted from gst-omx
 *
 * The reason it's needed is that the decoder won't always output frames in the
 * order they were presented because decode order does not always equal display
 * order.
 *
 * So, when a frame is decoded, it's not just as simple as calling
 * finish_frame() passing the frame given in the last handle_frame() call.
 *
 * Instead, we need to locate the pending frame with the nearest PTS to the
 * frame that has been decoded.
 *
 * NOTE: We could perhaps avoid this if userdata set on the input buffers were
 * propagated to the output buffers - something to check.
 */
GstVideoCodecFrame *
gst_mmal_video_dec_find_nearest_frame (MMAL_BUFFER_HEADER_T * buf,
    GList * frames)
{
  GstVideoCodecFrame *best = NULL;
  GstClockTimeDiff best_diff = G_MAXINT64;
  GstClockTime timestamp;
  GList *l;

  timestamp = gst_util_uint64_scale ((int64_t) buf->pts, GST_SECOND,
      MMAL_TICKS_PER_SECOND);

  for (l = frames; l; l = l->next) {

    GstVideoCodecFrame *tmp = l->data;
    GstClockTimeDiff diff = ABS (GST_CLOCK_DIFF (timestamp, tmp->pts));

    if (diff < best_diff) {
      best = tmp;
      best_diff = diff;

      if (diff == 0) {
        break;
      }
    }
  }

  if (best) {
    gst_video_codec_frame_ref (best);
  }

  g_list_foreach (frames, (GFunc) gst_video_codec_frame_unref, NULL);
  g_list_free (frames);

  return best;
}


/**
 * This is lifted (and adapted) from gst-omx (gstomxvideodec.c).
 *
 * It is used by gst_mmal_video_dec_handle_decoded_frames(), when not using
 * opaque output buffers and doing dumb copy from decoder output buffer to
 * frame GstBuffer.
 *
 * NOTE: This needs further work.  The way slices and strides are represented
 * is different in MMAL.
 */
static gboolean
gst_mmal_video_dec_fill_buffer (GstMMALVideoDec * self,
    MMAL_BUFFER_HEADER_T * inbuf, GstBuffer * outbuf)
{
  GstVideoCodecState *state =
      gst_video_decoder_get_output_state (GST_VIDEO_DECODER (self));

  GstVideoInfo *vinfo = &state->info;

  MMAL_PORT_T *output_port = self->dec->output[0];
  MMAL_ES_FORMAT_T *output_format = output_port->format;

  gboolean ret = FALSE;
  GstVideoFrame frame;

  if (vinfo->width != output_format->es->video.width ||
      vinfo->height != output_format->es->video.height) {
    GST_ERROR_OBJECT (self, "Resolution do not match: port=%ux%u vinfo=%dx%d",
        (guint) output_format->es->video.width,
        (guint) output_format->es->video.height, vinfo->width, vinfo->height);
    goto done;
  }

  /* Same strides and everything */
  if (gst_buffer_get_size (outbuf) == inbuf->length) {

    GstMapInfo map = GST_MAP_INFO_INIT;

    if (!gst_buffer_map (outbuf, &map, GST_MAP_WRITE)) {
      GST_ERROR_OBJECT (self, "Failed to map output buffer");
      goto done;
    }

    memcpy (map.data, inbuf->data + inbuf->offset, inbuf->length);

    gst_buffer_unmap (outbuf, &map);

    ret = TRUE;
    goto done;
  }

  /* Different strides */
  if (gst_video_frame_map (&frame, vinfo, outbuf, GST_MAP_WRITE)) {

    const guint nstride = output_format->es->video.width;
    const guint nslice = output_format->es->video.height;

    guint src_stride[GST_VIDEO_MAX_PLANES] = { nstride, 0, };
    guint src_size[GST_VIDEO_MAX_PLANES] = { nstride * nslice, 0, };
    gint dst_width[GST_VIDEO_MAX_PLANES] = { 0, };
    gint dst_height[GST_VIDEO_MAX_PLANES] =
        { GST_VIDEO_INFO_HEIGHT (vinfo), 0, };
    const guint8 *src;
    guint p;

    switch (GST_VIDEO_INFO_FORMAT (vinfo)) {
      case GST_VIDEO_FORMAT_ABGR:
      case GST_VIDEO_FORMAT_ARGB:
        dst_width[0] = GST_VIDEO_INFO_WIDTH (vinfo) * 4;
        break;
      case GST_VIDEO_FORMAT_RGB16:
      case GST_VIDEO_FORMAT_BGR16:
      case GST_VIDEO_FORMAT_YUY2:
      case GST_VIDEO_FORMAT_UYVY:
      case GST_VIDEO_FORMAT_YVYU:
        dst_width[0] = GST_VIDEO_INFO_WIDTH (vinfo) * 2;
        break;
      case GST_VIDEO_FORMAT_GRAY8:
        dst_width[0] = GST_VIDEO_INFO_WIDTH (vinfo);
        break;
      case GST_VIDEO_FORMAT_I420:
        dst_width[0] = GST_VIDEO_INFO_WIDTH (vinfo);
        src_stride[1] = nstride / 2;
        src_size[1] = (src_stride[1] * nslice) / 2;
        dst_width[1] = GST_VIDEO_INFO_WIDTH (vinfo) / 2;
        dst_height[1] = GST_VIDEO_INFO_HEIGHT (vinfo) / 2;
        src_stride[2] = nstride / 2;
        src_size[2] = (src_stride[1] * nslice) / 2;
        dst_width[2] = GST_VIDEO_INFO_WIDTH (vinfo) / 2;
        dst_height[2] = GST_VIDEO_INFO_HEIGHT (vinfo) / 2;
        break;
      case GST_VIDEO_FORMAT_NV12:
        dst_width[0] = GST_VIDEO_INFO_WIDTH (vinfo);
        src_stride[1] = nstride;
        src_size[1] = src_stride[1] * nslice / 2;
        dst_width[1] = GST_VIDEO_INFO_WIDTH (vinfo);
        dst_height[1] = GST_VIDEO_INFO_HEIGHT (vinfo) / 2;
        break;
      case GST_VIDEO_FORMAT_NV16:
        dst_width[0] = GST_VIDEO_INFO_WIDTH (vinfo);
        src_stride[1] = nstride;
        src_size[1] = src_stride[1] * nslice;
        dst_width[1] = GST_VIDEO_INFO_WIDTH (vinfo);
        dst_height[1] = GST_VIDEO_INFO_HEIGHT (vinfo);
        break;
      default:
        g_assert_not_reached ();
        break;
    }

    src = inbuf->data + inbuf->offset;
    for (p = 0; p < GST_VIDEO_INFO_N_PLANES (vinfo); p++) {
      const guint8 *data;
      guint8 *dst;
      guint h;

      dst = GST_VIDEO_FRAME_PLANE_DATA (&frame, p);
      data = src;
      for (h = 0; h < dst_height[p]; h++) {
        memcpy (dst, data, dst_width[p]);
        dst += GST_VIDEO_INFO_PLANE_STRIDE (vinfo, p);
        data += src_stride[p];
      }
      src += src_size[p];
    }

    gst_video_frame_unmap (&frame);
    ret = TRUE;
  } else {
    GST_ERROR_OBJECT (self, "Can't map output buffer to frame");
    goto done;
  }

done:
  if (ret) {
    GST_BUFFER_PTS (outbuf) =
        gst_util_uint64_scale (inbuf->pts, GST_SECOND, MMAL_TICKS_PER_SECOND);
  }

  gst_video_codec_state_unref (state);

  return ret;
}

static GstFlowReturn
gst_mmal_video_dec_handle_decoded_frames (GstMMALVideoDec * self)
{
  MMAL_BUFFER_HEADER_T *buffer = NULL;
  GstVideoCodecFrame *frame = NULL;
  GstClockTimeDiff deadline;
  GstFlowReturn flow_ret = GST_FLOW_OK;

  while (flow_ret == GST_FLOW_OK &&
      (buffer = mmal_queue_get (self->decoded_frames_queue)) != NULL) {

    if (buffer->cmd) {

      GST_DEBUG_OBJECT (self, "TODO TODO TODO:  Found command!  Handle it!");

    } else {

      GST_DEBUG_OBJECT (self, "Handling decoded frame: %" G_GUINT64_FORMAT,
          (guint64) buffer->pts);

      frame = gst_mmal_video_dec_find_nearest_frame (buffer,
          gst_video_decoder_get_frames (GST_VIDEO_DECODER (self)));

      /* TODO: gst-omx cleans-up older frames at this point.  We should probably
         do the same. */

      if (frame && (deadline = gst_video_decoder_get_max_decode_time
              (GST_VIDEO_DECODER (self), frame)) < 0) {

        GST_WARNING_OBJECT (self,
            "Frame is too late, dropping (deadline %" GST_TIME_FORMAT ")",
            GST_TIME_ARGS (-deadline));

        flow_ret =
            gst_video_decoder_drop_frame (GST_VIDEO_DECODER (self), frame);
        frame = NULL;

      } else if (!frame) {

        /* TODO: Handle this properly. gst-omx code reckons it can happen. */
        GST_ERROR_OBJECT (self,
            "Failed to find matching frame for PTS: %" G_GUINT64_FORMAT,
            (guint64) buffer->pts);

      } else {

        /* So we've found a matching frame then, and it's not too late ? */

        /* We do dumb thing to start with.  Allocate a new GstBuffer, and copy
           the output frame. Later we will introduce opaque buffers. */

        if ((flow_ret =
                gst_video_decoder_allocate_output_frame (GST_VIDEO_DECODER
                    (self), frame)) != GST_FLOW_OK) {

          GST_ERROR_OBJECT (self, "Failed to allocate output frame!");

        } else {

          if (!gst_mmal_video_dec_fill_buffer (self, buffer,
                  frame->output_buffer)) {

            /* FAIL */

            GST_ERROR_OBJECT (self,
                "Failed to copy MMAL output buffer to frame!");

            gst_buffer_replace (&frame->output_buffer, NULL);

            flow_ret =
                gst_video_decoder_drop_frame (GST_VIDEO_DECODER (self), frame);

          } else {

            GST_DEBUG_OBJECT (self, "Finishing frame...");

            flow_ret =
                gst_video_decoder_finish_frame (GST_VIDEO_DECODER (self),
                frame);
          }
        }
      }

      frame = NULL;
    }

    /* We will always get here, so buffer should always be freed. */
    if (buffer != NULL) {

      /* Release MMAL buffer back to the output pool.
         NOTE: We do this whether it's a "command" or an actual frame. */
      mmal_buffer_header_release (buffer);

      /* This is needed to prevent double-free. See below. */
      buffer = NULL;
    }
  }

  return flow_ret;
}



/**
 * N.B. This is called by the video decoder baseclass when there is some input
 * data to decode.
 */
static GstFlowReturn
gst_mmal_video_dec_handle_frame (GstVideoDecoder * decoder,
    GstVideoCodecFrame * frame)
{

  MMAL_BUFFER_HEADER_T *buffer = NULL;
  guint input_offset = 0;
  guint input_size = 0;
  int64_t pts_microsec = 0;
  GstFlowReturn flow_ret = GST_FLOW_OK;

  GstMMALVideoDec *self = GST_MMAL_VIDEO_DEC (decoder);


  /* First handle any already-decoded frames. */
  if ((flow_ret = gst_mmal_video_dec_handle_decoded_frames (self)) !=
      GST_FLOW_OK) {
    goto error;
  }


  /* Send empty buffers to the output port of the decoder to allow the decoder
   * to start producing frames as soon as it gets input data.
   */

  while ((buffer = mmal_queue_get (self->output_buffer_pool->queue)) != NULL) {

    GST_DEBUG_OBJECT (self, "Sending empty buffer to output port...");

    if (mmal_port_send_buffer (self->dec->output[0], buffer) != MMAL_SUCCESS) {
      GST_ERROR_OBJECT (self, "Failed to send empty output buffer to outport!");
      goto error;
    }
  }


  /* Handle this input frame. */

  if (!self->started) {

    if (!GST_VIDEO_CODEC_FRAME_IS_SYNC_POINT (frame)) {
      gst_video_decoder_drop_frame (GST_VIDEO_DECODER (self), frame);
      return GST_FLOW_OK;
    }

    /* TODO: Is this the right thing to do? */
    self->started = TRUE;
  }

  /* We have to wait for input buffers. */

  input_size = gst_buffer_get_size (frame->input_buffer);

  /* GST timestamp is in nanosecs */

  if (GST_CLOCK_TIME_IS_VALID (frame->pts)) {
    /* FIXME: Should probably use gst scale() util. */
    pts_microsec = frame->pts / 1000;
  } else {
    pts_microsec = MMAL_TIME_UNKNOWN;
  }


  while (input_offset < input_size) {

    /* We have to wait for an input buffer. */

    GST_DEBUG_OBJECT (self, "Waiting for input buffer...");

    if ((buffer = mmal_queue_wait (self->input_buffer_pool->queue)) == NULL) {
      GST_ERROR_OBJECT (self, "Failed to acquire input buffer!");
      goto error;
    }

    GST_DEBUG_OBJECT (self, "Got input buffer");

    /* Now we have a buffer, write the next bit of the input frame. */
    buffer->length =
        MIN (input_size - input_offset, buffer->alloc_size - buffer->offset);

    gst_buffer_extract (frame->input_buffer, input_offset,
        buffer->data + buffer->offset, buffer->length);

    /* Set PTS */
    buffer->pts = pts_microsec;
    buffer->dts = MMAL_TIME_UNKNOWN;

    /* Flags */
    if (input_offset == 0) {

      buffer->flags |= MMAL_BUFFER_HEADER_FLAG_FRAME_START;

      if (GST_VIDEO_CODEC_FRAME_IS_SYNC_POINT (frame)) {
        buffer->flags |= MMAL_BUFFER_HEADER_FLAG_KEYFRAME;
      }
    }

    input_offset += buffer->length;

    if (input_offset == input_size) {
      buffer->flags |= MMAL_BUFFER_HEADER_FLAG_FRAME_END;
    }

    /* Now send the buffer to decoder. */

    if (mmal_port_send_buffer (self->dec->input[0], buffer) != MMAL_SUCCESS) {
      GST_ERROR_OBJECT (self, "Failed to send input buffer to decoder!");
      goto error;
    }
  }

  gst_video_codec_frame_unref (frame);

  return GST_FLOW_OK;

error:

  gst_video_codec_frame_unref (frame);
  return GST_FLOW_ERROR;

}
