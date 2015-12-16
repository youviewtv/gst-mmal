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

#define GST_MMALVIDEOSINK(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_MMALVIDEOSINK, GstMMALVideoSink))

#define GST_MMALVIDEOSINK_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass), GST_TYPE_MMALVIDEOSINK, GstMMALVideoSinkClass))

typedef struct _GstMMALVideoSink GstMMALVideoSink;
typedef struct _GstMMAVideoSinkClass GstMMALVideoSinkClass;

struct _GstMMALVideoSink
{
  GstVideoSink videosink;
};

struct _GstMMAVideoSinkClass
{
  GstVideoSinkClass parent_class;
};

GST_DEBUG_CATEGORY (mmalvideosink_debug);
#define GST_CAT_DEFAULT mmalvideosink_debug

#define gst_mmalvideosink_parent_class parent_class
G_DEFINE_TYPE (GstMMALVideoSink, gst_mmal_video_sink, GST_TYPE_VIDEO_SINK);

static GstStaticPadTemplate sink_template = GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (GST_VIDEO_CAPS_MAKE ("I420"))
    );

static void
gst_mmal_video_sink_class_init (GstMMALVideoSinkClass * klass)
{
  GstElementClass *element_class = GST_ELEMENT_CLASS (klass);

  gst_element_class_add_pad_template (element_class,
      gst_static_pad_template_get (&sink_template));

  gst_element_class_set_static_metadata (element_class,
      "MMAL Video Sink", "Sink/Video",
      "Displays frames using MMAL", "Krzysztof Konopko <kris@youview.com>");

  GST_DEBUG_CATEGORY_INIT (mmalvideosink_debug, "mmalvideosink", 0,
      "MMAL video sink element");
}

static void
gst_mmal_video_sink_init (G_GNUC_UNUSED GstMMALVideoSink * sink)
{
}
