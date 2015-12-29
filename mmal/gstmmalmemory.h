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
