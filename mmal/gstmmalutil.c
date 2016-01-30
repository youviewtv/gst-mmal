/*
 * Copyright (C) 2016, YouView TV Ltd.
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
#include "gstmmalutil.h"

#include <interface/mmal/util/mmal_util.h>


/**
 * Bare-bones mapping of supported color formats to GST.  Right now, we will
 * only support I420.
 */
GstVideoFormat
gst_mmal_video_get_format_from_mmal (MMAL_FOURCC_T mmal_colorformat)
{
  GstVideoFormat format;

  switch (mmal_colorformat) {
    case MMAL_ENCODING_I420:
    case MMAL_ENCODING_OPAQUE:
      format = GST_VIDEO_FORMAT_I420;
      break;

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
void
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
