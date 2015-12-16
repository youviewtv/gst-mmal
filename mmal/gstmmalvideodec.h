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

#ifndef __GST_MMAL_VIDEO_DEC_H__
#define __GST_MMAL_VIDEO_DEC_H__

#include <gst/gst.h>
#include <gst/video/video.h>
#include <gst/video/gstvideodecoder.h>
#include <interface/mmal/mmal.h>

G_BEGIN_DECLS

#define GST_TYPE_MMAL_VIDEO_DEC \
  (gst_mmal_video_dec_get_type())

#define GST_MMAL_VIDEO_DEC(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_MMAL_VIDEO_DEC,GstMMALVideoDec))

#define GST_MMAL_VIDEO_DEC_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_MMAL_VIDEO_DEC,GstMMALVideoDecClass))

#define GST_MMAL_VIDEO_DEC_GET_CLASS(obj) \
  (G_TYPE_INSTANCE_GET_CLASS((obj),GST_TYPE_MMAL_VIDEO_DEC,GstMMALVideoDecClass))

#define GST_IS_MMAL_VIDEO_DEC(obj) \
  (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_MMAL_VIDEO_DEC))

#define GST_IS_MMAL_VIDEO_DEC_CLASS(obj) \
  (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_MMAL_VIDEO_DEC))

typedef struct _GstMMALVideoDec GstMMALVideoDec;
typedef struct _GstMMALVideoDecClass GstMMALVideoDecClass;

struct _GstMMALVideoDec
{
  GstVideoDecoder parent;

  /* < protected > */
  MMAL_COMPONENT_T *dec;

  /* < private > */
  GstVideoCodecState *input_state;
  GstBuffer *codec_data;

  MMAL_QUEUE_T *decoded_frames_queue;
  MMAL_POOL_T *input_buffer_pool;
  MMAL_POOL_T *output_buffer_pool;

  /* TRUE if the component is configured and saw
   * the first buffer */
  gboolean started;

  GstClockTime last_upstream_ts;
};

struct _GstMMALVideoDecClass
{
  GstVideoDecoderClass parent_class;

  gboolean (*is_format_change) (GstMMALVideoDec * self, MMAL_PORT_T * port, GstVideoCodecState * state);
  gboolean (*set_format)       (GstMMALVideoDec * self, MMAL_PORT_T * port, GstVideoCodecState * state);
  GstFlowReturn (*prepare_frame)   (GstMMALVideoDec * self, GstVideoCodecFrame *frame);
};

GType gst_mmal_video_dec_get_type (void);

G_END_DECLS

#endif /* __GST_MMAL_VIDEO_DEC_H__ */
