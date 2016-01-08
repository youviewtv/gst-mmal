/*
 * Copyright (C) 2015, YouView TV Ltd.
 *   Author: Krzysztof Konopko <kris@youview.com>
 *           John Sadler <john.sadler@youview.com>
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

#include "gstmmalh264dec.h"
#include "gstmmalmpegviddec.h"
#include "gstmmalwmvdec.h"
#include "gstmmalvp8dec.h"
#include "gstmmalvideosink.h"

#include <gst/gst.h>

static gboolean
plugin_init (GstPlugin * plugin)
{
  if (!gst_element_register (plugin, "mmalh264dec",
          GST_RANK_PRIMARY + 1, GST_TYPE_MMAL_H264_DEC)) {
    return FALSE;
  }

  if (!gst_element_register (plugin, "mmalmpegviddec",
          GST_RANK_PRIMARY + 1, GST_TYPE_MMAL_MPEG_VID_DEC)) {
    return FALSE;
  }

  if (!gst_element_register (plugin, "mmalwmvdec",
          GST_RANK_PRIMARY + 1, GST_TYPE_MMAL_WMV_DEC)) {
    return FALSE;
  }

  if (!gst_element_register (plugin, "mmalvp8dec",
          GST_RANK_PRIMARY + 1, GST_TYPE_MMAL_VP8_DEC)) {
    return FALSE;
  }

  if (!gst_element_register (plugin, "mmalvideosink", GST_RANK_NONE,
          GST_TYPE_MMAL_VIDEO_SINK)) {
    return FALSE;
  }

  return TRUE;
}

GST_PLUGIN_DEFINE (GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    mmal,
    "GStreamer MMAL Plug-ins",
    plugin_init,
    PACKAGE_VERSION, GST_LICENSE, GST_PACKAGE_NAME, GST_PACKAGE_ORIGIN)
