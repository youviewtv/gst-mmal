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

#include "gstmmalh264dec.h"

#include <gst/gst.h>

GST_DEBUG_CATEGORY_STATIC (gst_mmal_h264_dec_debug_category);
#define GST_CAT_DEFAULT gst_mmal_h264_dec_debug_category


static const char gst_mmal_h264_dec_sink_caps_str[] =
    "video/x-h264, "
    "parsed=(boolean) true, "
    "alignment=(string) au, "
    "stream-format=(string) byte-stream, "
    "width=(int) [1,MAX], " "height=(int) [1,MAX]";

static GstStaticPadTemplate gst_mmal_h264_dec_sink_factory =
GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (gst_mmal_h264_dec_sink_caps_str));


/* prototypes */
static gboolean gst_mmal_h264_dec_is_format_change (GstMMALVideoDec * dec,
    MMAL_PORT_T * port, GstVideoCodecState * state);

static gboolean gst_mmal_h264_dec_set_format (GstMMALVideoDec * dec,
    MMAL_PORT_T * port, GstVideoCodecState * state);

enum
{
  PROP_0
};

/* class initialization */

#define DEBUG_INIT \
  GST_DEBUG_CATEGORY_INIT (gst_mmal_h264_dec_debug_category, "mmalh264dec", 0, \
      "debug category for gst-mmal h264 video decoder");

G_DEFINE_TYPE_WITH_CODE (GstMMALH264Dec, gst_mmal_h264_dec,
    GST_TYPE_MMAL_VIDEO_DEC, DEBUG_INIT);

static void
gst_mmal_h264_dec_class_init (GstMMALH264DecClass * klass)
{
  GstMMALVideoDecClass *videodec_class = GST_MMAL_VIDEO_DEC_CLASS (klass);
  GstElementClass *element_class = GST_ELEMENT_CLASS (klass);
  GstPadTemplate *pad_template;

  videodec_class->is_format_change =
      GST_DEBUG_FUNCPTR (gst_mmal_h264_dec_is_format_change);

  videodec_class->set_format = GST_DEBUG_FUNCPTR (gst_mmal_h264_dec_set_format);

  /* Sink pad */
  pad_template = gst_static_pad_template_get (&gst_mmal_h264_dec_sink_factory);
  gst_element_class_add_pad_template (element_class, pad_template);

  gst_element_class_set_static_metadata (element_class,
      "MMAL H.264 Video Decoder",
      "Codec/Decoder/Video",
      "Decode H.264 video streams", "John Sadler <john.sadler@youview.com>");
}

static void
gst_mmal_h264_dec_init (GstMMALH264Dec * self)
{
}

static gboolean
gst_mmal_h264_dec_is_format_change (GstMMALVideoDec * dec,
    G_GNUC_UNUSED MMAL_PORT_T * port, GstVideoCodecState * state)
{
  GstCaps *old_caps = NULL;
  GstCaps *new_caps = state->caps;
  GstStructure *old_structure, *new_structure;
  const gchar *old_profile, *old_level, *new_profile, *new_level;

  if (dec->input_state) {
    old_caps = dec->input_state->caps;
  }

  if (!old_caps) {
    return FALSE;
  }

  old_structure = gst_caps_get_structure (old_caps, 0);
  new_structure = gst_caps_get_structure (new_caps, 0);
  old_profile = gst_structure_get_string (old_structure, "profile");
  old_level = gst_structure_get_string (old_structure, "level");
  new_profile = gst_structure_get_string (new_structure, "profile");
  new_level = gst_structure_get_string (new_structure, "level");

  if (g_strcmp0 (old_profile, new_profile) != 0
      || g_strcmp0 (old_level, new_level) != 0) {
    return TRUE;
  }

  return FALSE;
}

static gboolean
gst_mmal_h264_dec_set_format (GstMMALVideoDec * dec, MMAL_PORT_T * port,
    GstVideoCodecState * state)
{
  MMAL_ES_FORMAT_T *format = port->format;

  format->encoding = MMAL_ENCODING_H264;

  return TRUE;
}
