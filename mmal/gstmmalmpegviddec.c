/*
 * Copyright (C) 2016, YouView TV Ltd.
 *   Author: John Sadler <john.sadler@youview.com>
 *           Tomasz Szkutkowski <tomasz.szkutkowski@youview.com>
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

#include "gstmmalmpegviddec.h"

#include <gst/gst.h>


#define GST_MMAL_MPEG_VID_DEC(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_MMAL_MPEG_VID_DEC,GstMMALMPEGVidDec))

#define GST_MMAL_MPEG_VID_DEC_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_MMAL_MPEG_VID_DEC,GstMMALMPEGVidDecClass))

#define GST_MMAL_MPEG_VID_DEC_GET_CLASS(obj) \
  (G_TYPE_INSTANCE_GET_CLASS((obj),GST_TYPE_MMAL_MPEG_VID_DEC,GstMMALMPEGVidDecClass))

typedef struct _GstMMALMPEGVidDec GstMMALMPEGVidDec;
typedef struct _GstMMALMPEGVidDecClass GstMMALMPEGVidDecClass;

struct _GstMMALMPEGVidDec
{
  GstMMALVideoDec parent;
};

struct _GstMMALMPEGVidDecClass
{
  GstMMALVideoDecClass parent_class;
};

GST_DEBUG_CATEGORY_STATIC (gst_mmal_mpeg_vid_dec_debug_category);
#define GST_CAT_DEFAULT gst_mmal_mpeg_vid_dec_debug_category


static const char gst_mmal_mpeg_vid_dec_sink_caps_str[] =
    "video/mpeg, "
    "mpegversion=(int) {1, 2, 4}, "
    "systemstream=(boolean) false, "
    "parsed=(boolean) true, " "width=(int) [1,MAX], " "height=(int) [1,MAX]";

static GstStaticPadTemplate gst_mmal_mpeg_vid_dec_sink_factory =
GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (gst_mmal_mpeg_vid_dec_sink_caps_str));


/* prototypes */
static gboolean gst_mmal_mpeg_vid_dec_is_format_change (GstMMALVideoDec * dec,
    MMAL_PORT_T * port, GstVideoCodecState * state);

static gboolean gst_mmal_mpeg_vid_dec_set_format (GstMMALVideoDec * dec,
    MMAL_PORT_T * port, GstVideoCodecState * state);

/* class initialization */

#define DEBUG_INIT \
  GST_DEBUG_CATEGORY_INIT (gst_mmal_mpeg_vid_dec_debug_category, "mmalmpegviddec", 0, \
      "debug category for gst-mmal mpeg video decoder");

G_DEFINE_TYPE_WITH_CODE (GstMMALMPEGVidDec, gst_mmal_mpeg_vid_dec,
    GST_TYPE_MMAL_VIDEO_DEC, DEBUG_INIT);

static void
gst_mmal_mpeg_vid_dec_class_init (GstMMALMPEGVidDecClass * klass)
{
  GstMMALVideoDecClass *videodec_class = GST_MMAL_VIDEO_DEC_CLASS (klass);
  GstElementClass *element_class = GST_ELEMENT_CLASS (klass);
  GstPadTemplate *pad_template;

  videodec_class->is_format_change =
      GST_DEBUG_FUNCPTR (gst_mmal_mpeg_vid_dec_is_format_change);

  videodec_class->set_format =
      GST_DEBUG_FUNCPTR (gst_mmal_mpeg_vid_dec_set_format);

  /* Sink pad */
  pad_template =
      gst_static_pad_template_get (&gst_mmal_mpeg_vid_dec_sink_factory);
  gst_element_class_add_pad_template (element_class, pad_template);

  gst_element_class_set_static_metadata (element_class,
      "MMAL MPEG-1/2/4p2 Video Decoder",
      "Codec/Decoder/Video",
      "Decode MPEG-1/2/4p2 video streams",
      "John Sadler <john.sadler@youview.com>, "
      "Tomasz Szkutkowski <tomasz.szkutkowski@youview.com>");
}

static void
gst_mmal_mpeg_vid_dec_init (G_GNUC_UNUSED GstMMALMPEGVidDec * self)
{
  return;
}

static gboolean
gst_mmal_mpeg_vid_dec_is_format_change (G_GNUC_UNUSED GstMMALVideoDec * dec,
    G_GNUC_UNUSED MMAL_PORT_T * port, G_GNUC_UNUSED GstVideoCodecState * state)
{
  return FALSE;
}

static gboolean
gst_mmal_mpeg_vid_dec_set_format (GstMMALVideoDec * dec, MMAL_PORT_T * port,
    GstVideoCodecState * state)
{
  MMAL_ES_FORMAT_T *format = port->format;
  gint mpeg_layer = 2;


  if (state != NULL && state->caps != NULL) {

    if (!gst_structure_get_int (gst_caps_get_structure (state->caps, 0),
            "mpegversion", &mpeg_layer)) {

      GST_ERROR_OBJECT (dec, "Missing mpegversion field !!");
      return FALSE;
    }
  }

  switch (mpeg_layer) {
    case 1:                    /* MPEG-1 */
      format->encoding = MMAL_ENCODING_MP1V;
      break;
    case 2:                    /* MPEG-2 */
      format->encoding = MMAL_ENCODING_MP2V;
      break;
    case 4:                    /* MPEG-4 Part 2 */
      format->encoding = MMAL_ENCODING_MP4V;
      break;
    default:
      GST_ERROR_OBJECT (dec, "Unknown or invalid MPEG Layer (%d) !!",
          mpeg_layer);
      return FALSE;
  }

  GST_DEBUG_OBJECT (dec, "MPEG-%d%s video stream", mpeg_layer,
      (mpeg_layer == 4) ? " Part 2" : "");

  return TRUE;
}
