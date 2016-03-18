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

#include "gstmmalvp6dec.h"

#include <gst/gst.h>


#define GST_MMAL_VP6_DEC(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_MMAL_VP6_DEC,GstMMALVP6Dec))

#define GST_MMAL_VP6_DEC_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_MMAL_VP6_DEC,GstMMALVP6DecClass))

#define GST_MMAL_VP6_DEC_GET_CLASS(obj) \
  (G_TYPE_INSTANCE_GET_CLASS((obj),GST_TYPE_MMAL_VP6_DEC,GstMMALVP6DecClass))

typedef struct _GstMMALVP6Dec GstMMALVP6Dec;
typedef struct _GstMMALVP6DecClass GstMMALVP6DecClass;

struct _GstMMALVP6Dec
{
  GstMMALVideoDec parent;
};

struct _GstMMALVP6DecClass
{
  GstMMALVideoDecClass parent_class;
};


GST_DEBUG_CATEGORY_STATIC (gst_mmal_vp6_dec_debug_category);
#define GST_CAT_DEFAULT gst_mmal_vp6_dec_debug_category


static const char gst_mmal_vp6_dec_sink_caps_str[] =
    "video/x-vp6, " "width=(int) [1,MAX], " "height=(int) [1,MAX];"
    "video/x-vp6-flash, " "width=(int) [1,MAX], " "height=(int) [1,MAX]";

static GstStaticPadTemplate gst_mmal_vp6_dec_sink_factory =
GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (gst_mmal_vp6_dec_sink_caps_str));


/* prototypes */
static gboolean gst_mmal_vp6_dec_is_format_change (GstMMALVideoDec * dec,
    MMAL_PORT_T * port, GstVideoCodecState * state);

static gboolean gst_mmal_vp6_dec_set_format (GstMMALVideoDec * dec,
    MMAL_PORT_T * port, GstVideoCodecState * state);

/* class initialization */

#define DEBUG_INIT \
  GST_DEBUG_CATEGORY_INIT (gst_mmal_vp6_dec_debug_category, "mmalvp6dec", 0, \
      "debug category for gst-mmal vp6 video decoder");

G_DEFINE_TYPE_WITH_CODE (GstMMALVP6Dec, gst_mmal_vp6_dec,
    GST_TYPE_MMAL_VIDEO_DEC, DEBUG_INIT);

static void
gst_mmal_vp6_dec_class_init (GstMMALVP6DecClass * klass)
{
  GstMMALVideoDecClass *videodec_class = GST_MMAL_VIDEO_DEC_CLASS (klass);
  GstElementClass *element_class = GST_ELEMENT_CLASS (klass);
  GstPadTemplate *pad_template;

  videodec_class->is_format_change =
      GST_DEBUG_FUNCPTR (gst_mmal_vp6_dec_is_format_change);

  videodec_class->set_format = GST_DEBUG_FUNCPTR (gst_mmal_vp6_dec_set_format);

  /* Sink pad */
  pad_template = gst_static_pad_template_get (&gst_mmal_vp6_dec_sink_factory);
  gst_element_class_add_pad_template (element_class, pad_template);

  gst_element_class_set_static_metadata (element_class,
      "MMAL VP6 Video Decoder",
      "Codec/Decoder/Video",
      "Decode VP6 video streams",
      "John Sadler <john.sadler@youview.com>, "
      "Tomasz Szkutkowski <tomasz.szkutkowski@youview.com>");
}

static void
gst_mmal_vp6_dec_init (G_GNUC_UNUSED GstMMALVP6Dec * self)
{
  return;
}

static gboolean
gst_mmal_vp6_dec_is_format_change (G_GNUC_UNUSED GstMMALVideoDec * dec,
    G_GNUC_UNUSED MMAL_PORT_T * port, G_GNUC_UNUSED GstVideoCodecState * state)
{
  return FALSE;
}

static gboolean
gst_mmal_vp6_dec_set_format (G_GNUC_UNUSED GstMMALVideoDec * dec,
    MMAL_PORT_T * port, G_GNUC_UNUSED GstVideoCodecState * state)
{
  MMAL_ES_FORMAT_T *format = port->format;

  format->encoding = MMAL_ENCODING_VP6;

  return TRUE;
}
