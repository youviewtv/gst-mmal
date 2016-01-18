/*
 * Copyright (C) 2015, YouView TV Ltd.
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

#include "gstmmalwmvdec.h"

#include <gst/gst.h>


#define GST_MMAL_WMV_DEC(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_MMAL_WMV_DEC,GstMMALWMVDec))

#define GST_MMAL_WMV_DEC_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_MMAL_WMV_DEC,GstMMALWMVDecClass))

#define GST_MMAL_WMV_DEC_GET_CLASS(obj) \
  (G_TYPE_INSTANCE_GET_CLASS((obj),GST_TYPE_MMAL_WMV_DEC,GstMMALWMVDecClass))

typedef struct _GstMMALWMVDec GstMMALWMVDec;
typedef struct _GstMMALWMVDecClass GstMMALWMVDecClass;

struct _GstMMALWMVDec
{
  GstMMALVideoDec parent;
};

struct _GstMMALWMVDecClass
{
  GstMMALVideoDecClass parent_class;
};


GST_DEBUG_CATEGORY_STATIC (gst_mmal_wmv_dec_debug_category);
#define GST_CAT_DEFAULT gst_mmal_wmv_dec_debug_category


static const char gst_mmal_wmv_dec_sink_caps_str[] =
    "video/x-wmv, "
    "wmvversion=(int) {3, 2, 1}, "
    "format=(string) {WVC1, WMV3, WMV2, WMV1}, "
    "width=(int) [1,MAX], " "height=(int) [1,MAX]";

static GstStaticPadTemplate gst_mmal_wmv_dec_sink_factory =
GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (gst_mmal_wmv_dec_sink_caps_str));


/* prototypes */
static gboolean gst_mmal_wmv_dec_is_format_change (GstMMALVideoDec * dec,
    MMAL_PORT_T * port, GstVideoCodecState * state);

static gboolean gst_mmal_wmv_dec_set_format (GstMMALVideoDec * dec,
    MMAL_PORT_T * port, GstVideoCodecState * state);

/* class initialization */

#define DEBUG_INIT \
  GST_DEBUG_CATEGORY_INIT (gst_mmal_wmv_dec_debug_category, "mmalwmvdec", 0, \
      "debug category for gst-mmal wmv video decoder");

G_DEFINE_TYPE_WITH_CODE (GstMMALWMVDec, gst_mmal_wmv_dec,
    GST_TYPE_MMAL_VIDEO_DEC, DEBUG_INIT);

static void
gst_mmal_wmv_dec_class_init (GstMMALWMVDecClass * klass)
{
  GstMMALVideoDecClass *videodec_class = GST_MMAL_VIDEO_DEC_CLASS (klass);
  GstElementClass *element_class = GST_ELEMENT_CLASS (klass);
  GstPadTemplate *pad_template;

  videodec_class->is_format_change =
      GST_DEBUG_FUNCPTR (gst_mmal_wmv_dec_is_format_change);

  videodec_class->set_format = GST_DEBUG_FUNCPTR (gst_mmal_wmv_dec_set_format);

  /* Sink pad */
  pad_template = gst_static_pad_template_get (&gst_mmal_wmv_dec_sink_factory);
  gst_element_class_add_pad_template (element_class, pad_template);

  gst_element_class_set_static_metadata (element_class,
      "MMAL WMV/VC-1 Video Decoder",
      "Codec/Decoder/Video",
      "Decode Windows Media Video and VC-1 video streams",
      "John Sadler <john.sadler@youview.com>, "
      "Tomasz Szkutkowski <tomasz.szkutkowski@youview.com>");
}

static void
gst_mmal_wmv_dec_init (G_GNUC_UNUSED GstMMALWMVDec * self)
{
  return;
}

static gboolean
gst_mmal_wmv_dec_is_format_change (G_GNUC_UNUSED GstMMALVideoDec * dec,
    G_GNUC_UNUSED MMAL_PORT_T * port, G_GNUC_UNUSED GstVideoCodecState * state)
{
  return FALSE;
}

static gboolean
gst_mmal_wmv_dec_set_format (G_GNUC_UNUSED GstMMALVideoDec * dec,
    MMAL_PORT_T * port, G_GNUC_UNUSED GstVideoCodecState * state)
{
  MMAL_ES_FORMAT_T *format = port->format;


  /* if format detection will fail, we'll use VC-1 as a default codec */
  format->encoding = MMAL_ENCODING_WVC1;

  if (state != NULL && state->caps != NULL) {

    const gchar *strFormat =
        gst_structure_get_string (gst_caps_get_structure (state->caps, 0),
        "format");

    if (strFormat != NULL) {
      GST_DEBUG_OBJECT (dec, "Format: %s", strFormat);

      if (!strncmp (strFormat, "WVC1", 4)) {    /* VC-1 aka WMV9 Advanced */
        format->encoding = MMAL_ENCODING_WVC1;
      } else if (!strncmp (strFormat, "WMV3", 4)) {     /* WMV3 aka WMV9 Base */
        format->encoding = MMAL_ENCODING_WMV3;
      } else if (!strncmp (strFormat, "WMV2", 4)) {     /* WMV2 aka WMV8 */
        format->encoding = MMAL_ENCODING_WMV2;
      } else if (!strncmp (strFormat, "WMV1", 4)) {     /* WMV1 aka WMV7 */
        format->encoding = MMAL_ENCODING_WMV1;
      } else {
        GST_ERROR_OBJECT (dec, "Unknown or invalid WMV format (%s) !!",
            strFormat);
        return FALSE;
      }

    } else {
      /* if no format string in caps, try wmvversion */
      gint wmvversion = 0;

      if (gst_structure_get_int (gst_caps_get_structure (state->caps, 0),
              "wmvversion", &wmvversion)) {

        GST_DEBUG_OBJECT (dec, "WMV Version: %d", wmvversion);

        if (wmvversion == 1) {
          format->encoding = MMAL_ENCODING_WMV1;        /* WMV1 aka WMV7 */
        } else if (wmvversion == 2) {
          format->encoding = MMAL_ENCODING_WMV2;        /* WMV2 aka WMV8 */
        } else if (wmvversion == 3) {
          format->encoding = MMAL_ENCODING_WMV3;        /* WMV3 aka WMV9 Base */
        } else {
          GST_ERROR_OBJECT (dec, "Wrong or invalid WMV version (%d) !!",
              wmvversion);
          return FALSE;
        }
      } else {
        /* this should not happen, but let's support bad case when there are no
         * wmvversion or format fields in the caps */
        GST_ERROR_OBJECT (dec,
            "Failed to detect due missing wmvversion and format fields!");
        return FALSE;
      }
    }
  }

  if (_gst_debug_min >= GST_LEVEL_DEBUG) {
    char *cc = (char *) &(format->encoding);
    GST_DEBUG_OBJECT (dec, "WMV Format: %c%c%c%c", cc[0], cc[1], cc[2], cc[3]);
  }

  return TRUE;
}
