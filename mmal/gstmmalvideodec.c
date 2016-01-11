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
#include "gstmmalmemory.h"

#include <gst/gst.h>

#include <string.h>
#include <stdio.h>

#include <bcm_host.h>

#include <interface/mmal/util/mmal_default_components.h>
#include <interface/mmal/util/mmal_util.h>
#include <interface/mmal/util/mmal_util_params.h>


GST_DEBUG_CATEGORY_STATIC (gst_mmal_video_dec_debug_category);
#define GST_CAT_DEFAULT gst_mmal_video_dec_debug_category

#define MMAL_TICKS_PER_SECOND 1000000
#define GST_MMAL_VIDEO_DEC_EXTRA_OUTPUT_BUFFER_HEADERS 3

#define GST_MMAL_VIDEO_DEC_EXTRA_OUTPUT_BUFFER_HEADERS_OPAQUE_MODE 20
#define GST_MMAL_VIDEO_DEC_EXTRA_OUTPUT_BUFFERS_OPAQUE_MODE 20

static GstStaticPadTemplate gst_mmal_video_dec_src_factory =
    GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (GST_VIDEO_CAPS_MAKE_WITH_FEATURES
        (GST_CAPS_FEATURE_MEMORY_MMAL_OPAQUE,
            "{ I420 }") ";" GST_VIDEO_CAPS_MAKE ("{ I420 }")));

struct MappedBuffer_;

struct MappedBuffer_
{
  GstBuffer *buffer;
  GstMapInfo map_info;

  /* N.B. The linked-list structure serves the same purpose as a refcount.
     I don't want to map & unmap the buffer n times, and due to MMAL restriction
     we may have to pass the same GstBuffer into multiple MMAL buffer headers.
   */
  struct MappedBuffer_ *next;
  struct MappedBuffer_ *previous;

};

typedef struct MappedBuffer_ MappedBuffer;


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

static GstFlowReturn gst_mmal_video_dec_drain (GstMMALVideoDec * self);

static GstVideoCodecFrame
    * gst_mmal_video_dec_find_nearest_frame (MMAL_BUFFER_HEADER_T * buf,
    GList * frames);

static gboolean gst_mmal_video_dec_decide_allocation (GstVideoDecoder * decoder,
    GstQuery * query);

static gboolean gst_mmal_video_dec_populate_output_port (GstMMALVideoDec *
    self);

static gboolean gst_mmal_video_dec_update_src_caps (GstMMALVideoDec * self);

static GstFlowReturn gst_mmal_video_dec_handle_decoded_frames (GstMMALVideoDec *
    self, uint32_t wait_for_buffer_timeout_ms);

static GstVideoFormat gst_mmal_video_get_format_from_mmal (MMAL_FOURCC_T
    mmal_colorformat);

static void gst_mmal_print_port_info (MMAL_PORT_T * port, GstElement * element);

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

  video_decoder_class->decide_allocation =
      GST_DEBUG_FUNCPTR (gst_mmal_video_dec_decide_allocation);

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
  self->caps_changed = FALSE;
  self->output_buffer_flags = 0;
  self->opaque = FALSE;
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
  MappedBuffer *mb = NULL;

  /* We're using the GstBuffer directly, rather than copying into an MMAL
     allocated buffer.  So we need to unmap and unref the GstBuffer once the
     decoder is done with it.

     N.B. We `user_data` will be NULL in case buffer had no GstBuffer attached,
     (i.e. when EOS buffer sent from drain()).
   */
  if (!buffer->cmd && buffer->user_data) {

    mb = (MappedBuffer *) buffer->user_data;

    g_return_if_fail (mb != NULL);

    /* Was this the last MMAL buffer header pointing at our mapped GST buffer?
       If so, we can unmap it.

       Incidentally, we can have several MMAL headers pointing to different
       offsets in the same GST buffer mapping since we must tell MMAL upfront
       how big the input buffers are, and our GST buffer might exceed that size.
     */
    if (mb->next == NULL && mb->previous == NULL) {

      gst_buffer_unmap (mb->buffer, &mb->map_info);
      gst_buffer_unref (mb->buffer);

    } else {

      /* Not the last reference. Unmap later. */
      if (mb->previous) {
        mb->previous->next = mb->next;
      }

      if (mb->next) {
        mb->next->previous = mb->previous;
      }
    }

    /* Restore original MMAL buffer payload we used for storing buffer info */
    buffer->data = (uint8_t *) mb;
    buffer->length = buffer->alloc_size = sizeof (*mb);
  }

  /* This drops the refcount of the buffer header.  It will be returned to it's
     pool when the refcount reaches zero.
   */
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
    case MMAL_ENCODING_OPAQUE:
      format = GST_VIDEO_FORMAT_I420;
      break;

      /* TODO: Support other formats.  Opaque format, for instance. */
    default:
      format = GST_VIDEO_FORMAT_UNKNOWN;
      break;
  }

  return format;
}


static const char *
gst_mmal_port_type_to_string (MMAL_PORT_TYPE_T type)
{

  switch (type) {
    case MMAL_PORT_TYPE_CONTROL:
      return "Control";
    case MMAL_PORT_TYPE_INPUT:
      return "Input";
    case MMAL_PORT_TYPE_OUTPUT:
      return "Output";
    case MMAL_PORT_TYPE_CLOCK:
      return "Clock";
    case MMAL_PORT_TYPE_INVALID:
      return "Invalid";
    case MMAL_PORT_TYPE_UNKNOWN:
    default:
      return "Unknown";
  }
}


static const char *
gst_mmal_es_type_to_string (MMAL_ES_TYPE_T type)
{
  switch (type) {
    case MMAL_ES_TYPE_CONTROL:
      return "Control";
    case MMAL_ES_TYPE_AUDIO:
      return "Audio";
    case MMAL_ES_TYPE_VIDEO:
      return "Video";
    case MMAL_ES_TYPE_SUBPICTURE:
      return "Subpicture";
    case MMAL_ES_TYPE_UNKNOWN:
    default:
      return "Unknown";
  }
}

/**
 * This function prints video format of given port.
 */
static void
gst_mmal_print_port_format (MMAL_PORT_T * port, GstElement * element)
{
  MMAL_ES_FORMAT_T *format = port->format;

  char encoding[5];
  char encoding_variant[5];
  char color_space[5];

  GST_DEBUG_OBJECT (element, "\n"
      "type:                    %s\n"
      "encoding:                %s\n"
      "encoding_variant:        %s\n"
      "width:                   %d\n"
      "height:                  %d\n"
      "crop.x:                  %d\n"
      "crop.y:                  %d\n"
      "crop.width:              %d\n"
      "crop.height:             %d\n"
      "frame_rate:              %d/%d\n"
      "par:                     %d/%d\n"
      "color_space:             %s\n",
      gst_mmal_es_type_to_string (format->type),
      mmal_4cc_to_string (encoding, sizeof (encoding), format->encoding),
      mmal_4cc_to_string (encoding_variant, sizeof (encoding_variant),
          format->encoding_variant),
      format->es->video.width,
      format->es->video.height,
      format->es->video.crop.x,
      format->es->video.crop.y,
      format->es->video.crop.width,
      format->es->video.crop.height,
      format->es->video.frame_rate.num, format->es->video.frame_rate.den,
      format->es->video.par.num, format->es->video.par.den,
      mmal_4cc_to_string (color_space, sizeof (color_space),
          format->es->video.color_space)
      );
}


/**
 * This is just simple debug function to print out port information.
 */
static void
gst_mmal_print_port_info (MMAL_PORT_T * port, GstElement * element)
{

  GST_DEBUG_OBJECT (element, "\n"
      "name:                     %s\n"
      "type:                     %s\n"
      "index:                    %d\n"
      "index_all:                %d\n"
      "is_enabled:               %d\n"
      "buffer_num_min:           %d\n"
      "buffer_size_min:          %d\n"
      "buffer_alignment_min:     %d\n"
      "buffer_num_recommended:   %d\n"
      "buffer_size_recommended:  %d\n"
      "buffer_num:               %d\n"
      "buffer_size:              %d\n",
      port->name,
      gst_mmal_port_type_to_string (port->type),
      port->index,
      port->index_all,
      port->is_enabled,
      port->buffer_num_min,
      port->buffer_size_min,
      port->buffer_alignment_min,
      port->buffer_num_recommended,
      port->buffer_size_recommended, port->buffer_num, port->buffer_size);


  gst_mmal_print_port_format (port, element);
}


/**
 * We implement this in order to negotiate MMAL opaque buffers, where downstream
 * supports it.
 */
static gboolean
gst_mmal_video_dec_decide_allocation (GstVideoDecoder * decoder,
    GstQuery * query)
{
  GstMMALVideoDec *self = GST_MMAL_VIDEO_DEC (decoder);
  GstBufferPool *pool;
  GstStructure *config;

  GstCaps *caps;
  gint i, n;
  GstVideoInfo info;

  self->opaque = FALSE;

  gst_query_parse_allocation (query, &caps, NULL);

  if (caps && gst_video_info_from_caps (&info, caps)
      && info.finfo->format == GST_VIDEO_FORMAT_I420) {

    gboolean found = FALSE;
    GstCapsFeatures *feature = gst_caps_get_features (caps, 0);

    /* Prefer an MMAL Opaque allocator if available and we want to use it */
    n = gst_query_get_n_allocation_params (query);

    for (i = 0; i < n; i++) {

      GstAllocator *allocator;
      GstAllocationParams params;

      gst_query_parse_nth_allocation_param (query, i, &allocator, &params);

      if (allocator) {

        if (g_strcmp0 (allocator->mem_type, GST_MMAL_OPAQUE_MEMORY_TYPE) == 0) {

          found = TRUE;

          gst_query_set_nth_allocation_param (query, 0, allocator, &params);
          while (gst_query_get_n_allocation_params (query) > 1) {
            gst_query_remove_nth_allocation_param (query, 1);
          }
        }

        gst_object_unref (allocator);

        if (found) {
          /* We will use opaque buffers. */
          GST_DEBUG_OBJECT (self, "Negotiated to use opaque buffers.");
          self->opaque = TRUE;
          break;
        }
      }
    }

    /* If try to negotiate with caps feature memory:MMALOpaque
       and if allocator is not of type memory MMALOpaque then fails.
       Should not have negotiated MMAL Opaque in allocation query caps
       otherwise.
     */
    if (feature
        && gst_caps_features_contains (feature,
            GST_CAPS_FEATURE_MEMORY_MMAL_OPAQUE) && !found) {

      GST_WARNING_OBJECT (self, "Caps indicate MMAL opaque buffers supported, "
          "but allocator not found!");
      return FALSE;
    }
  }

  if (!GST_VIDEO_DECODER_CLASS
      (gst_mmal_video_dec_parent_class)->decide_allocation (decoder, query)) {
    return FALSE;
  }

  g_assert (gst_query_get_n_allocation_pools (query) > 0);
  gst_query_parse_nth_allocation_pool (query, 0, &pool, NULL, NULL, NULL);
  g_assert (pool != NULL);

  config = gst_buffer_pool_get_config (pool);

  if (gst_query_find_allocation_meta (query, GST_VIDEO_META_API_TYPE, NULL)) {
    gst_buffer_pool_config_add_option (config,
        GST_BUFFER_POOL_OPTION_VIDEO_META);
  }

  if (self->opaque) {

    /* Downstream proposed MMAL allocator, above.

       In this case, we override the choice of buffer pool with a specialised
       one that takes care of releasing MMAL buffers when the GstBuffer is
       returned to the pool.

       Downstream *could* provide this specialised pool as well, but we're
       providing the MMAL buffer headers, so this seems to make sense.

       I wish we didn't need this specialised pool, and could just use the
       allocator, but I don't know of another way to unref the mmal header
       when the buffer is returned to pool (except perhaps some hack with a
       custom meta on the buffer).
     */

    GstAllocator *allocator;
    guint config_size;
    guint config_min_buffers;
    guint config_max_buffers;
    GstAllocationParams allocator_params;

    GST_DEBUG_OBJECT (self, "Creating new MMAL Opaque buffer pool.");

    gst_buffer_pool_config_get_params (config, &caps, &config_size,
        &config_min_buffers, &config_max_buffers);

    gst_buffer_pool_config_get_allocator (config, &allocator,
        &allocator_params);

    gst_object_unref (pool);
    pool = gst_mmal_opaque_buffer_pool_new ();

    gst_buffer_pool_config_set_params (config, caps, config_size,
        config_min_buffers, config_max_buffers);

    gst_buffer_pool_config_set_allocator (config, allocator, &allocator_params);

    gst_query_set_nth_allocation_pool (query, 0, pool, config_size,
        config_min_buffers, config_max_buffers);
  }
  /* end-if (opaque) */

  gst_buffer_pool_set_config (pool, config);
  gst_object_unref (pool);

  return TRUE;
}


/**
 * This is called from set_format()
 *
 * TODO: Locking?  omx impl does: GST_VIDEO_DECODER_STREAM_UNLOCK (self);
 * Not sure if that is neccessary in the context this is called?
 */
static gboolean
gst_mmal_video_dec_update_src_caps (GstMMALVideoDec * self)
{
  GstVideoCodecState *state = NULL;
  MMAL_ES_FORMAT_T *output_format = self->dec->output[0]->format;
  MMAL_PARAMETER_VIDEO_INTERLACE_TYPE_T interlace_type;

  GstVideoFormat format =
      gst_mmal_video_get_format_from_mmal (output_format->encoding);

  /* This requires explanation. The reason we need to twiddle output port
     enablement and commit the output format here, (instead of in set_format()),
     is because:

     a) upstream elements don't tell us the right framerate. 200/1? really?
     b) the decoder can work-out framerate from headers in stream (e.g.H264 SPS)
     c) we don't seem to get any "port settings changed" event on the output
     port, as we might expect.
     d) the decoder will need to see at least the first frame before it can work
     out framerate.
     e) the output port format doesn't seem to be updated unless we do this
     little dance of disable, commit format, and re-enable the output port.

     Perhaps there is a simpler way to achieve the same result, but I don't know
     what that is.
   */
  if (self->dec->output[0]->is_enabled) {

    if (mmal_port_disable (self->dec->output[0]) != MMAL_SUCCESS) {
      GST_ERROR_OBJECT (self, "Failed to disable output port.");
      return FALSE;
    }

    /* FIXME: Chicken-egg: We only find-out if opaque as a result of
       negotiation with downstream.  However, it's pretty unlikely that this is
       going to change, once initial caps have been negotiated.
     */
    if (self->opaque) {

      output_format->encoding = MMAL_ENCODING_OPAQUE;

      if (mmal_port_parameter_set_uint32 (self->dec->output[0],
              MMAL_PARAMETER_EXTRA_BUFFERS,
              GST_MMAL_VIDEO_DEC_EXTRA_OUTPUT_BUFFERS_OPAQUE_MODE) !=
          MMAL_SUCCESS) {

        GST_ERROR_OBJECT (self,
            "Failed to set extra-buffer count on output port!");
        return FALSE;
      }
    }

    if (mmal_port_format_commit (self->dec->output[0]) != MMAL_SUCCESS) {
      GST_ERROR_OBJECT (self, "Failed to commit output port format.");
      return FALSE;
    }

    if (mmal_port_enable (self->dec->output[0],
            &gst_mmal_video_dec_queue_decoded_frame) != MMAL_SUCCESS) {
      GST_ERROR_OBJECT (self, "Failed to re-enable output port.");
      return FALSE;
    }

    /* Make sure the ouput port has buffers, just in case. */
    gst_mmal_video_dec_populate_output_port (self);
  }

  if (format == GST_VIDEO_FORMAT_UNKNOWN) {

    GST_ERROR_OBJECT (self, "Unsupported color format: %lu",
        (unsigned long) output_format->encoding);

    goto error;
  }

  state = gst_video_decoder_set_output_state (GST_VIDEO_DECODER (self),
      format, output_format->es->video.crop.width,
      output_format->es->video.crop.height, self->input_state);

  /* Take framerate from output port for src caps. */
  GST_VIDEO_INFO_FPS_N (&state->info) = output_format->es->video.frame_rate.num;
  GST_VIDEO_INFO_FPS_D (&state->info) = output_format->es->video.frame_rate.den;


  self->output_buffer_flags = 0;

  /* Query whether interlaced. */
  interlace_type.hdr.id = MMAL_PARAMETER_VIDEO_INTERLACE_TYPE;
  interlace_type.hdr.size = sizeof (MMAL_PARAMETER_VIDEO_INTERLACE_TYPE_T);

  if (mmal_port_parameter_get (self->dec->output[0], &interlace_type.hdr) !=
      MMAL_SUCCESS) {

    GST_ERROR_OBJECT (self, "Failed to query interlace type!");

  } else {

    /* N.B. Although MMAL interlace mode tells us field order, I'm not sure
       there is anywhere for us to put that in the vinfo?

       In any case, progressive and interlaced frames can both appear in Mixed
       mode, so we will need to look at each output buffer and set appropriate
       buffer flags on Gst output buffer.

       NOTE: There seems to be a problem with flags on MMAL output buffers.
       Deinterlace flags are not set on buffers, even when the output is clearly
       interlaced.  So all we can do for now is to take what the decoder tells
       us here to derive flags we will apply to all output GstBuffers that
       follow.
     */
    switch (interlace_type.eMode) {

      case MMAL_InterlaceFieldSingleUpperFirst:
        /* Fields are stored in one buffer, use the frame ID to get access to the
           required field.

           Each field has only half the amount of lines as noted in the
           height property. This mode requires multiple GstVideoMeta metadata
           to describe the fields.
         */
        GST_DEBUG_OBJECT (self, "InterlaceFieldSingleUpperFirst");
        GST_VIDEO_INFO_INTERLACE_MODE (&state->info) =
            GST_VIDEO_INTERLACE_MODE_FIELDS;
        self->output_buffer_flags |= GST_VIDEO_BUFFER_FLAG_INTERLACED;
        self->output_buffer_flags |= GST_VIDEO_BUFFER_FLAG_TFF;
        break;

      case MMAL_InterlaceFieldSingleLowerFirst:
        GST_DEBUG_OBJECT (self, "InterlaceFieldSingleLowerFirst");
        GST_VIDEO_INFO_INTERLACE_MODE (&state->info) =
            GST_VIDEO_INTERLACE_MODE_FIELDS;
        self->output_buffer_flags |= GST_VIDEO_BUFFER_FLAG_INTERLACED;
        break;

      case MMAL_InterlaceFieldsInterleavedUpperFirst:
        /* Fields are interleaved in one video frame. Extra buffer flags
           describe the field order.
         */
        GST_DEBUG_OBJECT (self, "InterlaceFieldsInterleavedUpperFirst");
        GST_VIDEO_INFO_INTERLACE_MODE (&state->info) =
            GST_VIDEO_INTERLACE_MODE_INTERLEAVED;
        self->output_buffer_flags |= GST_VIDEO_BUFFER_FLAG_INTERLACED;
        self->output_buffer_flags |= GST_VIDEO_BUFFER_FLAG_TFF;
        break;

      case MMAL_InterlaceFieldsInterleavedLowerFirst:
        GST_DEBUG_OBJECT (self, "InterlaceFieldsInterleavedLowerFirst");
        GST_VIDEO_INFO_INTERLACE_MODE (&state->info) =
            GST_VIDEO_INTERLACE_MODE_INTERLEAVED;
        self->output_buffer_flags |= GST_VIDEO_BUFFER_FLAG_INTERLACED;
        break;

      case MMAL_InterlaceMixed:
        /* Frames contains both interlaced and progressive video, the buffer flags
           describe the frame and fields.
         */
        GST_DEBUG_OBJECT (self, "InterlaceMixed");
        GST_VIDEO_INFO_INTERLACE_MODE (&state->info) =
            GST_VIDEO_INTERLACE_MODE_MIXED;
        self->output_buffer_flags |= GST_VIDEO_BUFFER_FLAG_INTERLACED;
        break;

      case MMAL_InterlaceProgressive:
      default:
        /* All frames are progressive */
        GST_DEBUG_OBJECT (self, "InterlaceProgressive");
        GST_VIDEO_INFO_INTERLACE_MODE (&state->info) =
            GST_VIDEO_INTERLACE_MODE_PROGRESSIVE;
        break;
    }

    if (interlace_type.bRepeatFirstField) {
      GST_DEBUG_OBJECT (self, "Repeat first field");
      self->output_buffer_flags |= GST_VIDEO_BUFFER_FLAG_RFF;
    }
  }


  /* The negotiation with downstream.

     We always want to use opaque buffers if we can negotiate that.  However,
     GstVideoDecoder won't include the needed feature in the caps, so we have
     to do that ourselves here.  We first try negotiating opaque then, if that
     fails, we remove the opaque feature and try to negotiate "plain" buffers.
   */
  if (state->caps) {
    gst_caps_unref (state->caps);
  }

  state->caps = gst_video_info_to_caps (&state->info);

  gst_caps_set_features (state->caps, 0,
      gst_caps_features_new (GST_CAPS_FEATURE_MEMORY_MMAL_OPAQUE, NULL));

  if (!gst_video_decoder_negotiate (GST_VIDEO_DECODER (self))) {

    GST_INFO_OBJECT (self, "Failed to negotiate opaque with downstream.");

    gst_caps_replace (&state->caps, gst_video_info_to_caps (&state->info));

    if (!gst_video_decoder_negotiate (GST_VIDEO_DECODER (self))) {

      GST_ERROR_OBJECT (self, "Failed to negotiate caps with downstream.");

      gst_video_codec_state_unref (state);
      goto error;
    }
  }

  /* Don't want to come back into this function until next set_format() */
  self->caps_changed = FALSE;

  return TRUE;

error:
  /* N.B. would release stream lock here, if acquired. */
  return FALSE;
}


/**
 * Drain the decoder.
 *
 * CHECKME:  This is not hooked-up as a GstVideoDecoder vmethod yet.  gst-omx
 * doesn't seem to do this either.  Should we?
 *
 * We push EOS into the decoder input port, and wait for it to pop-out of the
 * output port.
 *
 * From GstVideoDecoder docs:
 *
 * "Optional. Called to request subclass to decode any data it can at this point
 * but that more data may arrive after. (e.g. at segment end). Sub-classes
 * should be prepared to handle new data afterward, or seamless segment
 * processing will break."
 */
static GstFlowReturn
gst_mmal_video_dec_drain (GstMMALVideoDec * self)
{
  MMAL_BUFFER_HEADER_T *buffer = NULL;
  GstFlowReturn flow_ret = GST_FLOW_OK;

  if (self->input_buffer_pool == NULL) {
    /* We probably haven't set initial format yet.  Don't worry. */
    return GST_FLOW_OK;

  } else {

    if (!gst_mmal_video_dec_populate_output_port (self)) {
      GST_ERROR_OBJECT (self, "Failed to populate output port!");
      return GST_FLOW_ERROR;
    }

    /* Grab a buffer */

    GST_DEBUG_OBJECT (self, "Waiting for input buffer...");

    if ((buffer = mmal_queue_timedwait (self->input_buffer_pool->queue, 500)) ==
        NULL) {
      GST_ERROR_OBJECT (self, "Failed to acquire input buffer!");
      return GST_FLOW_ERROR;
    }

    GST_DEBUG_OBJECT (self, "Got input buffer");

    /* "Resets all variables to default values" */
    mmal_buffer_header_reset (buffer);
    buffer->cmd = 0;

    buffer->flags |= MMAL_BUFFER_HEADER_FLAG_EOS;

    /* Avoid trying to unref GstBuffer when releasing MMAL input buffer.
       See: gst_mmal_video_dec_return_input_buffer_to_pool().
     */
    buffer->user_data = NULL;

    buffer->pts = gst_util_uint64_scale (self->last_upstream_ts,
        MMAL_TICKS_PER_SECOND, GST_SECOND);

    GST_DEBUG_OBJECT (self, "Sending EOS with pts: %" G_GUINT64_FORMAT,
        (guint64) buffer->pts);

    if (mmal_port_send_buffer (self->dec->input[0], buffer) != MMAL_SUCCESS) {
      GST_ERROR_OBJECT (self, "Failed to send input buffer to decoder!");
      mmal_buffer_header_release (buffer);
      return GST_FLOW_ERROR;
    }

    /* OK, so now we've sent EOS to decoder.  Wait for an EOS buffer to plop
       out the other end.

       NOTE: We do this in a loop because there could well be pending output
       frames before the EOS, and if all output buffers are in use we will need
       at least one of these to be recycled back to the decoder before it can
       produce an EOS output buffer.  That's why I call populate_output_port()
       each time round the loop.
     */

    while ((flow_ret = gst_mmal_video_dec_handle_decoded_frames (self, 500)) ==
        GST_FLOW_OK) {
      gst_mmal_video_dec_populate_output_port (self);
    }

    /* We're here because either we got the EOS we were looking for, or there
       was some error.
     */
    if (flow_ret != GST_FLOW_EOS) {
      GST_ERROR_OBJECT (self, "Decoder failed to signal EOS on output port!");
      return GST_FLOW_ERROR;
    }

    GST_DEBUG_OBJECT (self, "Drained decoder.");

    return GST_FLOW_OK;
  }

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
  output_port = self->dec->output[0];

  input_format = input_port->format;

  /* Check if the caps change is a real format change or if only irrelevant
     parts of the caps have changed or nothing at all.
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

    GST_DEBUG_OBJECT (self, "Draining decoder...");

    if (gst_mmal_video_dec_drain (self) != GST_FLOW_OK) {
      GST_ERROR_OBJECT (self, "Drain failed!");
    }

    /*
       If ports already enabled, then I guess we need to disable them.
       IIRC, VLC code first tries setting format without disable, then
       falls-back to disabling if that fails.
     */

    if (output_port->is_enabled && mmal_port_disable (output_port) !=
        MMAL_SUCCESS) {
      GST_ERROR_OBJECT (self, "Failed to disable output port!");
      return FALSE;
    }


    if (input_port->is_enabled &&
        mmal_port_disable (input_port) != MMAL_SUCCESS) {

      GST_ERROR_OBJECT (self, "Failed to disable input port!");
      return FALSE;
    }

    /* Flush now? */
    if (mmal_port_flush (output_port) != MMAL_SUCCESS) {
      GST_ERROR_OBJECT (self, "Failed to flush output port.");
    }

    if (mmal_port_flush (input_port) != MMAL_SUCCESS) {
      GST_ERROR_OBJECT (self, "Failed to flush input port.");
    }

    gst_buffer_replace (&self->codec_data, state->codec_data);

    if (self->input_state != NULL) {
      gst_video_codec_state_unref (self->input_state);
      self->input_state = NULL;
    }

    self->input_state = gst_video_codec_state_ref (state);

    input_format->es->video.crop.x = 0;
    input_format->es->video.crop.y = 0;
    input_format->es->video.crop.width = info->width;
    input_format->es->video.crop.height = info->height;

    /* N.B. These are the alignments XBMC applies. */
    input_format->es->video.width = GST_ROUND_UP_32 (info->width);
    input_format->es->video.height = GST_ROUND_UP_16 (info->height);

    /* Don't provide any hint on framerate. Upstream sometimes gets it wrong.
       Let the decoder figure it out.
     */
    input_format->es->video.frame_rate.num = 0;
    input_format->es->video.frame_rate.den = 0;

    input_format->es->video.par.num = info->par_n;
    input_format->es->video.par.den = info->par_d;
    input_format->type = MMAL_ES_TYPE_VIDEO;
    input_format->flags = MMAL_ES_FORMAT_FLAG_FRAMED;

    /* CHECKME: What to stick in here? I'm going to assume it's codec_data
       I could be wrong about this though.  XBMC sources send codec config
       as a separate special buffer, and that's what happens in gst-omx also
     */

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

    if (mmal_port_format_commit (input_port) != MMAL_SUCCESS) {

      GST_ERROR_OBJECT (self, "Failed to commit new input port format!");
      return FALSE;
    }

    input_port->buffer_num = input_port->buffer_num_recommended;
    input_port->buffer_size = input_port->buffer_size_recommended;

    /*
       An existing pool can be resized.

       Note that in practice we should not need to ever resize the input pool as
       the decoder always seems to request 20 buffers of 80K each.

       XBMC sources are interesting in that they ignore the recommended values
       and instead allocate only 2 x 1MB buffers to "reduce latency".  Perhaps
       that is worth some experimentation.

       In order to avoid copying the input GstBuffers, we ask MMAL to allocate
       payload buffers just big enough to hold some housekeeping data relating
       to the mapped GstBuffer memories.  We don't bother to reallocate the
       input buffer pool on format change, since the payload size and number of
       buffer headers will never change in practice.
     */

    if (self->input_buffer_pool == NULL) {

      self->input_buffer_pool = mmal_pool_create (input_port->buffer_num,
          sizeof (MappedBuffer));
    }

    if (mmal_port_enable (input_port,
            &gst_mmal_video_dec_return_input_buffer_to_pool) != MMAL_SUCCESS) {

      GST_ERROR_OBJECT (self, "Failed to enable input port!");
      return FALSE;
    }


    /* We need to update src caps here, and go through the allocation query
       with downstream.  Otherwise, we can't know whether we should use opaque
       buffers or not on output port.
     */

    gst_mmal_video_dec_update_src_caps (self);

    /* This will cause update_src_caps() to be called (again) by
       handle_decoded_frames(), next time we have an output frame from the
       decoder.

       We defer the call until next output frame as the decoder can then
       figure-out things like framerate and interlaced vs progressive, which we
       want to set in src caps.
     */
    self->caps_changed = TRUE;



    if (self->opaque) {

      GST_DEBUG_OBJECT (self, "Configuring opaque buffers on output port...");

      output_format = output_port->format;

      output_format->encoding = MMAL_ENCODING_OPAQUE;

      if (mmal_port_format_commit (output_port) != MMAL_SUCCESS) {
        GST_ERROR_OBJECT (self, "Failed to commit output port format.");
        return FALSE;
      }

      output_port->buffer_num = output_port->buffer_num_recommended +
          GST_MMAL_VIDEO_DEC_EXTRA_OUTPUT_BUFFER_HEADERS_OPAQUE_MODE;

    } else {

      output_port->buffer_num = output_port->buffer_num_recommended +
          GST_MMAL_VIDEO_DEC_EXTRA_OUTPUT_BUFFER_HEADERS;
    }

    /*
       NOTE: We can also receive notification of output port format changes via
       in-band events.  It seems we don't get those most of the time though, so
       I'm also dealing with the output port here.
     */

    output_port->buffer_size = output_port->buffer_size_recommended;

    GST_DEBUG_OBJECT (self, "Reconfiguring output buffer pool...");

    if (self->output_buffer_pool != NULL) {
      mmal_port_pool_destroy (output_port, self->output_buffer_pool);
    }

    self->output_buffer_pool = mmal_port_pool_create (output_port,
        output_port->buffer_num, output_port->buffer_size);

    /* Recreate the decoded-frames queue. Why? Well, I get an odd assertion
       from MMAL next time I try to retrieve a buffer from it otherwise
       (contains NULL buffer header).
     */

    if (self->decoded_frames_queue) {
      mmal_queue_destroy (self->decoded_frames_queue);
      self->decoded_frames_queue = mmal_queue_create ();
      output_port->userdata = (void *) self->decoded_frames_queue;
    }


    if (mmal_port_enable (output_port,
            &gst_mmal_video_dec_queue_decoded_frame) != MMAL_SUCCESS) {

      GST_ERROR_OBJECT (self, "Failed to enable output port!");
      return FALSE;
    }

    if (!self->dec->is_enabled && mmal_component_enable (self->dec) !=
        MMAL_SUCCESS) {

      GST_ERROR_OBJECT (self, "Failed to enable decoder component!");
      return FALSE;
    }

    GST_DEBUG_OBJECT (self, "Input port info:");
    gst_mmal_print_port_info (input_port, GST_ELEMENT (self));

    GST_DEBUG_OBJECT (self, "Output port info:");
    gst_mmal_print_port_info (output_port, GST_ELEMENT (self));
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

  /* TODO: VLC sources suggest that we should be reloading the codec_data after
     flushing. See set_format() */

  self->last_upstream_ts = 0;

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
 * Copy MMAL ouput buffer to GstBuffer
 *
 * This is lifted (and adapted) from gst-omx (gstomxvideodec.c), with some
 * simplification, since we are only supporting I420.
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

  if (vinfo->width != output_format->es->video.crop.width ||
      vinfo->height != output_format->es->video.crop.height) {
    GST_ERROR_OBJECT (self, "Resolution do not match: port=%ux%u vinfo=%dx%d",
        (guint) output_format->es->video.crop.width,
        (guint) output_format->es->video.crop.height, vinfo->width,
        vinfo->height);
    goto done;
  }

  /* This is the simple case.  The strides and offset of the MMAL & GST buffers
     match, so a single memcpy will suffice.
   */
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

  GST_DEBUG_OBJECT (self, "\n"
      "MMAL Buffer size: %d\nGstBuffer size:   %d\n", inbuf->alloc_size,
      gst_buffer_get_size (outbuf));

  /* More complicated case. Different buffer sizes.  Strides and offsets don't
     match.
   */
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

    GST_DEBUG_OBJECT (self, "Different strides.\n"
        "MMAL buffer size: %d\n"
        "GST buffer size: %d\n", inbuf->length, gst_buffer_get_size (outbuf));


    /* These are for I420 */
    dst_width[0] = GST_VIDEO_INFO_WIDTH (vinfo);
    src_stride[1] = nstride / 2;
    src_size[1] = (src_stride[1] * nslice) / 2;
    dst_width[1] = GST_VIDEO_INFO_WIDTH (vinfo) / 2;
    dst_height[1] = GST_VIDEO_INFO_HEIGHT (vinfo) / 2;
    src_stride[2] = nstride / 2;
    src_size[2] = (src_stride[1] * nslice) / 2;
    dst_width[2] = GST_VIDEO_INFO_WIDTH (vinfo) / 2;
    dst_height[2] = GST_VIDEO_INFO_HEIGHT (vinfo) / 2;

    src = inbuf->data + inbuf->offset;

    /* Plane-by-plane */
    for (p = 0; p < GST_VIDEO_INFO_N_PLANES (vinfo); p++) {
      const guint8 *data;
      guint8 *dst;
      guint h;

      dst = GST_VIDEO_FRAME_PLANE_DATA (&frame, p);
      data = src;

      /* Line-by-line, in each plane */
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


/**
 * NOTE: This function is used by handle_frame() during normal processing, and
 * also by drain() to wait for EOS event.  In the former case, we don't wait
 * for buffers, but just take whatever is available.  In the latter, we will
 * wait - but only for the first decoded frame.  We will process any others that
 * happen to be ready after that one, but we will never block waiting for more
 * than one.
 */
static GstFlowReturn
gst_mmal_video_dec_handle_decoded_frames (GstMMALVideoDec * self,
    uint32_t wait_for_buffer_timeout_ms)
{
  MMAL_BUFFER_HEADER_T *buffer = NULL;
  GstVideoCodecFrame *frame = NULL;
  GstClockTimeDiff deadline;
  GstFlowReturn flow_ret = GST_FLOW_OK;

  if (wait_for_buffer_timeout_ms == 0) {
    /* Don't block.  Just take whatever is available. */
    if ((buffer = mmal_queue_get (self->decoded_frames_queue)) == NULL) {
      return GST_FLOW_OK;
    }
  } else if ((buffer = mmal_queue_timedwait (self->decoded_frames_queue,
              wait_for_buffer_timeout_ms)) == NULL) {
    /* We consider it an error in this case. */
    GST_WARNING_OBJECT (self, "Timed-out waiting for output decoded frame.");
    return GST_FLOW_ERROR;
  }

  while (buffer && flow_ret == GST_FLOW_OK) {

    /* N.B. We never seem to see any events, so these TODO's aren't quite as bad
       as they look!
     */
    if (buffer->cmd) {

      GST_DEBUG_OBJECT (self, "Received command.");

      switch (buffer->cmd) {

        case MMAL_EVENT_EOS:
          GST_DEBUG_OBJECT (self, "Got EOS");
          flow_ret = GST_FLOW_EOS;
          break;
        case MMAL_EVENT_ERROR:
          /* TODO: Pull-out some useful info. */
          GST_DEBUG_OBJECT (self, "TODO: Got Error");
          flow_ret = GST_FLOW_ERROR;
          break;
        case MMAL_EVENT_FORMAT_CHANGED:
          GST_DEBUG_OBJECT (self, "TODO: Output port format changed.");
          break;
        case MMAL_EVENT_PARAMETER_CHANGED:
          GST_DEBUG_OBJECT (self, "TODO: Parameter changed");
          break;
      }

    } else {

      GST_DEBUG_OBJECT (self, "Handling decoded frame: %" G_GUINT64_FORMAT,
          (guint64) buffer->pts);

      if ((buffer->flags & MMAL_BUFFER_HEADER_FLAG_EOS)) {

        /* NOTE: EOS buffer might still have a final payload we need to process.
           At the moment, I'm not expecting this to be the case, since EOS comes
           from us sending (empty) EOS input buffer in drain()
         */
        GST_DEBUG_OBJECT (self, "Buffer signals EOS.");
        flow_ret = GST_FLOW_EOS;
      }

      /* See EOS comment, above */
      if (buffer->length) {

        /* Did we have a caps change via set_format() just previously?
           Now that the decoder has seen a frame with the "new" format, it
           should have some idea about the framerate and whether interlaced.
           It doesn't know these things before seeing frames, and we can't
           really rely on upstream elements to get this right, as we're seeing
           framerate of "200/1" on input caps, and "progressive" when actually
           interlaced.
         */
        if (self->caps_changed) {

          if (!gst_mmal_video_dec_update_src_caps (self)) {
            GST_ERROR_OBJECT (self, "Failed to update src caps!");
            flow_ret = GST_FLOW_ERROR;
          }
        }

        frame = gst_mmal_video_dec_find_nearest_frame (buffer,
            gst_video_decoder_get_frames (GST_VIDEO_DECODER (self)));

        /* TODO: gst-omx cleans-up older "ghost" frames at this point.
           We should probably do the same at this point.
         */

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
             the output frame. Later we will introduce opaque buffers.
           */

          if ((flow_ret =
                  gst_video_decoder_allocate_output_frame (GST_VIDEO_DECODER
                      (self), frame)) != GST_FLOW_OK) {

            GST_ERROR_OBJECT (self, "Failed to allocate output frame!");

          } else {

            /* Set any buffer flags for interlace  See: update_src_caps() */
            GST_BUFFER_FLAG_SET (frame->output_buffer,
                self->output_buffer_flags);

            if (self->opaque) {

              /* Using opaque output buffers.  So the GstMemory in the GstBuffer
                 should have been allocated by our special allocator.
                 We just need to fill-in the pointer to the MMAL header.
               */


              GstMemory *mem = gst_buffer_peek_memory (frame->output_buffer,
                  0);

              if (mem == NULL || !gst_is_mmal_opaque_memory (mem)) {

                GST_ERROR_OBJECT (self, "Expected MMAL Opaque GstMemory!");
                flow_ret = GST_FLOW_ERROR;

              } else {
                gst_mmal_opaque_mem_set_mmal_header (mem, buffer);
              }
            }

            /* Only copy frame if not using opaque. */
            else if (!gst_mmal_video_dec_fill_buffer (self, buffer,
                    frame->output_buffer)) {

              /* FAIL */

              GST_ERROR_OBJECT (self,
                  "Failed to copy MMAL output buffer to frame!");

              gst_buffer_replace (&frame->output_buffer, NULL);

              flow_ret =
                  gst_video_decoder_drop_frame (GST_VIDEO_DECODER (self),
                  frame);
            }

            if (flow_ret == GST_FLOW_OK) {

              GST_DEBUG_OBJECT (self, "Finishing frame...");

              flow_ret =
                  gst_video_decoder_finish_frame (GST_VIDEO_DECODER (self),
                  frame);
            }
          }
          /* end-else (managed to allocate output frame) */
        }
        /* end-else (matching frame and not too late) */

        frame = NULL;
      }
      /* end-else (buffer has payload) */
    }
    /* end-else (not an event) */

    /* We will always get here, so buffer should always be freed. */
    if (buffer != NULL) {

      /* Release MMAL buffer back to the output pool.
         NOTE: We do this whether it's a "command" or an actual frame.
       */
      mmal_buffer_header_release (buffer);
    }

    /* As long as all is OK, try to get next buffer.  But don't block this
       time.
     */
    if (flow_ret == GST_FLOW_OK) {
      buffer = mmal_queue_get (self->decoded_frames_queue);
    }
  }
  /* end-while (more frames and everything OK) */

  return flow_ret;
}


/**
 * This just feeds any available output buffers to the decoder output port.
 * It's called from handle_frame().
 */
static gboolean
gst_mmal_video_dec_populate_output_port (GstMMALVideoDec * self)
{

  MMAL_BUFFER_HEADER_T *buffer = NULL;

  /* Send empty buffers to the output port of the decoder to allow the decoder
   * to start producing frames as soon as it gets input data.
   */

  while ((buffer = mmal_queue_get (self->output_buffer_pool->queue)) != NULL) {

    GST_DEBUG_OBJECT (self, "Sending empty buffer to output port...");

    mmal_buffer_header_reset (buffer);

    if (mmal_port_send_buffer (self->dec->output[0], buffer) != MMAL_SUCCESS) {
      GST_ERROR_OBJECT (self, "Failed to send empty output buffer to outport!");
      return FALSE;
    }
  }

  return TRUE;
}


/**
 * N.B. This is called by the video decoder baseclass when there is some input
 * data to decode.
 */
static GstFlowReturn
gst_mmal_video_dec_handle_frame (GstVideoDecoder * decoder,
    GstVideoCodecFrame * frame)
{
  MappedBuffer *mapped_gst_buffer = NULL;
  MappedBuffer *previous_mapped_gst_buffer = NULL;
  MMAL_BUFFER_HEADER_T *mmal_buffer = NULL;
  guint input_size = 0;
  guint input_offset = 0;
  int64_t pts_microsec = 0;
  GstFlowReturn flow_ret = GST_FLOW_OK;

  GstMMALVideoDec *self = GST_MMAL_VIDEO_DEC (decoder);

  /* First handle any already-decoded frames. */
  if ((flow_ret = gst_mmal_video_dec_handle_decoded_frames (self, 0)) !=
      GST_FLOW_OK) {
    goto done;
  }

  /* Populate output with buffers. */
  if (!gst_mmal_video_dec_populate_output_port (self)) {
    flow_ret = GST_FLOW_ERROR;
    goto done;
  }

  /* Handle this input frame. */

  if (!self->started) {

    if (!GST_VIDEO_CODEC_FRAME_IS_SYNC_POINT (frame)) {

      gst_video_decoder_drop_frame (GST_VIDEO_DECODER (self), frame);

      /* Jumping straight out here, rather than goto as we don't need to unref
         frame.
       */
      return GST_FLOW_OK;
    }

    /* CHECKME: Is this the right thing to do? */
    self->started = TRUE;
  }

  input_size = gst_buffer_get_size (frame->input_buffer);

  /* GST timestamp is in nanosecs */

  if (GST_CLOCK_TIME_IS_VALID (frame->pts)) {
    /* FIXME: Should probably use gst scale() util. */
    pts_microsec = frame->pts / 1000;

    /* N.B. last_upstream_ts is used by drain() */
    self->last_upstream_ts = frame->pts;

    if (frame->duration != GST_CLOCK_TIME_NONE) {
      self->last_upstream_ts += frame->duration;
    }

  } else {
    pts_microsec = MMAL_TIME_UNKNOWN;
  }

  while (input_offset < input_size) {

    /* We have to wait for an input buffer.
       We make this a timed-wait as we don't want to get stuck here forever! */

    GST_DEBUG_OBJECT (self, "Waiting for input buffer...");

    if ((mmal_buffer = mmal_queue_timedwait (self->input_buffer_pool->queue,
                500)) == NULL) {

      GST_ERROR_OBJECT (self, "Failed to acquire input buffer!");
      flow_ret = GST_FLOW_ERROR;
      goto done;
    }

    GST_DEBUG_OBJECT (self, "Got input buffer");

    /* "Resets all variables to default values" */
    mmal_buffer_header_reset (mmal_buffer);
    mmal_buffer->cmd = 0;

    /* Use MMAL buffer payload for mapped GStreamer buffer info */
    mapped_gst_buffer = (MappedBuffer *) mmal_buffer->data;

    mapped_gst_buffer->buffer = gst_buffer_ref (frame->input_buffer);

    if (previous_mapped_gst_buffer != NULL) {

      mapped_gst_buffer->map_info = previous_mapped_gst_buffer->map_info;
      mapped_gst_buffer->buffer = previous_mapped_gst_buffer->buffer;
      previous_mapped_gst_buffer->next = mapped_gst_buffer;

    } else if (!gst_buffer_map (mapped_gst_buffer->buffer,
            &mapped_gst_buffer->map_info, GST_MAP_READ)) {

      gst_buffer_unref (mapped_gst_buffer->buffer);
      mmal_buffer_header_release (mmal_buffer);

      GST_ERROR_OBJECT (self, "Failed to map frame input buffer for reading");
      flow_ret = GST_FLOW_ERROR;
      goto done;
    }

    mapped_gst_buffer->next = NULL;
    mapped_gst_buffer->previous = previous_mapped_gst_buffer;

    mmal_buffer->data = mapped_gst_buffer->map_info.data + input_offset;
    mmal_buffer->alloc_size = self->dec->input[0]->buffer_size;
    mmal_buffer->length = MIN (input_size - input_offset,
        mmal_buffer->alloc_size);
    mmal_buffer->offset = 0;

    /* Need to save this so we can restore in the MMAL buffer release callback */
    mmal_buffer->user_data = mapped_gst_buffer;

    /* Set PTS */
    mmal_buffer->pts = pts_microsec;
    mmal_buffer->dts = MMAL_TIME_UNKNOWN;

    /* Flags */
    if (input_offset == 0) {
      mmal_buffer->flags |= MMAL_BUFFER_HEADER_FLAG_FRAME_START;

      if (GST_VIDEO_CODEC_FRAME_IS_SYNC_POINT (frame)) {
        mmal_buffer->flags |= MMAL_BUFFER_HEADER_FLAG_KEYFRAME;
      }
    }

    input_offset += mmal_buffer->length;

    if (input_offset == input_size) {
      mmal_buffer->flags |= MMAL_BUFFER_HEADER_FLAG_FRAME_END;
    }

    /* Now send the buffer to decoder. */

    if (mmal_port_send_buffer (self->dec->input[0], mmal_buffer) !=
        MMAL_SUCCESS) {

      GST_ERROR_OBJECT (self, "Failed to send input buffer to decoder!");

      /* N.B. We've taken a ref to the GstBuffer, so we need to handle that.
         Our callback function will do that for us.
       */
      gst_mmal_video_dec_return_input_buffer_to_pool (self->dec->input[0],
          mmal_buffer);

      flow_ret = GST_FLOW_ERROR;
      goto done;
    }

    previous_mapped_gst_buffer = mapped_gst_buffer;
  }

done:

  /* Should reach this in all cases, except when we drop the frame. */
  gst_video_codec_frame_unref (frame);

  return flow_ret;
}
