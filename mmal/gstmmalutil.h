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

#ifndef __GST_MMAL_UTIL_H__
#define __GST_MMAL_UTIL_H__

#include <gst/gst.h>
#include <gst/video/video.h>
#include <interface/mmal/mmal.h>


GstVideoFormat gst_mmal_video_get_format_from_mmal (MMAL_FOURCC_T
    mmal_colorformat);

void gst_mmal_print_port_info (MMAL_PORT_T * port, GstElement * element);

#endif /* __GST_MMAL_UTIL_H__ */
