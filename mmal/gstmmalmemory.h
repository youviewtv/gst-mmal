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
#ifndef __GST_MMAL_MEMORY_H__
#define __GST_MMAL_MEMORY_H__

#include <gst/gst.h>
#include <gst/gstallocator.h>
#include <gst/gstmemory.h>
#include <gst/video/video.h>
#include <gst/video/gstvideopool.h>

#include <interface/mmal/mmal.h>

G_BEGIN_DECLS

#define GST_MMAL_I420_STRIDE_ALIGN 32

#define GST_MMAL_I420_WIDTH_ALIGN GST_ROUND_UP_32
#define GST_MMAL_I420_HEIGHT_ALIGN GST_ROUND_UP_16

#define GST_MMAL_MAX_VIDEO_WIDTH 1920
#define GST_MMAL_MAX_VIDEO_HEIGHT 1080

#define GST_MMAL_MAX_I420_RES \
    (GST_MMAL_I420_WIDTH_ALIGN (GST_MMAL_MAX_VIDEO_WIDTH) * \
     GST_MMAL_I420_HEIGHT_ALIGN (GST_MMAL_MAX_VIDEO_HEIGHT))

#define GST_MMAL_MAX_I420_BUFFER_SIZE ((3 * GST_MMAL_MAX_I420_RES) / 2)

/* Since we use upstream buffer allocation, components must agree on the number
   of buffers used between them.  This is dictated by the requirement that the
   number of buffers sent to MMAL port must not exceed the number of buffer
   declared (`buffer_num`) when the port is enabled.  GPU needs to know how many
   buffers to prepare for the port on its side.

   If the number of buffers sent to the port exceeds `buffer_num`, we get
   ENOMEM.

   Another reason to have this number defined upfront is to avoid reallocating
   pools which in some scenarios (media format change) is onerous.
 */
#define GST_MMAL_NUM_OUTPUT_BUFFERS 20

/* --- MMAL OPAQUE MEMORY --- */

#define GST_MMAL_OPAQUE_MEMORY_TYPE "MMALOpaque"

#define GST_CAPS_FEATURE_MEMORY_MMAL_OPAQUE "memory:MMALOpaque"


gboolean gst_is_mmal_opaque_memory (GstMemory * mem);

void gst_mmal_opaque_mem_set_mmal_header (GstMemory * mem,
    MMAL_BUFFER_HEADER_T *mmal_buffer_header);

MMAL_BUFFER_HEADER_T * gst_mmal_opaque_mem_get_mmal_header (GstMemory * mem);



/* --- MMAL OPAQUE ALLOCATOR --- */

#define GST_TYPE_MMAL_OPAQUE_ALLOCATOR   (gst_mmal_opaque_allocator_get_type())
#define GST_IS_MMAL_OPAQUE_ALLOCATOR(obj) (G_TYPE_CHECK_INSTANCE_TYPE ((obj), GST_TYPE_MMAL_OPAQUE_ALLOCATOR))

typedef struct _GstMMALOpaqueAllocator GstMMALOpaqueAllocator;

GType gst_mmal_opaque_allocator_get_type (void);



/* --- MMAL OPAQUE BUFFER POOL --- */


/* N.B. The only reason we need a specialised buffer pool is so we can deref
   MMAL buffer header when GstBuffer is returned to pool.  That's it!
 */

GstBufferPool * gst_mmal_opaque_buffer_pool_new (void);


G_END_DECLS

#endif /* __GST_MMAL_MEMORY_H__ */
