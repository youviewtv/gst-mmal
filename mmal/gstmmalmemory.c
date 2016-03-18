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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "gstmmalmemory.h"


/* --- MMAL OPAQUE MEMORY --- */

typedef struct _GstMMALOpaqueMemory GstMMALOpaqueMemory;

#define GST_MMAL_OPAQUE_MEMORY(mem) ((GstMMALOpaqueMemory*)(mem))

struct _GstMMALOpaqueMemory
{
  GstMemory parent;
  MMAL_BUFFER_HEADER_T *mmal_buffer_header;
};

gboolean
gst_is_mmal_opaque_memory (GstMemory * mem)
{
  g_return_val_if_fail (mem != NULL, FALSE);
  g_return_val_if_fail (mem->allocator != NULL, FALSE);

  return g_strcmp0 (mem->allocator->mem_type, GST_MMAL_OPAQUE_MEMORY_TYPE) == 0;
}

void
gst_mmal_opaque_mem_set_mmal_header (GstMemory * mem,
    MMAL_BUFFER_HEADER_T * mmal_buffer_header)
{

  if (mem == NULL || !gst_is_mmal_opaque_memory (mem)) {
    return;
  }

  {
    GstMMALOpaqueMemory *mmem = GST_MMAL_OPAQUE_MEMORY (mem);

    /* This function is also used by gst_mmal_opaque_buffer_pool_reset_buffer(),
       so we should take care to unref any header we're already pointing at.
     */
    if (mmem->mmal_buffer_header != NULL) {
      mmal_buffer_header_release (mmem->mmal_buffer_header);
    }

    if (mmal_buffer_header != NULL) {
      mmal_buffer_header_acquire (mmal_buffer_header);
    }

    /* N.B. Setting to NULL is fine. */
    mmem->mmal_buffer_header = mmal_buffer_header;
  }
}


MMAL_BUFFER_HEADER_T *
gst_mmal_opaque_mem_get_mmal_header (GstMemory * mem)
{
  if (mem == NULL || !gst_is_mmal_opaque_memory (mem)) {
    return NULL;
  }

  {
    GstMMALOpaqueMemory *mmem = GST_MMAL_OPAQUE_MEMORY (mem);
    return mmem->mmal_buffer_header;
  }
}


/* --- MMAL OPAQUE ALLOCATOR --- */

typedef struct _GstMMALOpaqueAllocatorClass GstMMALOpaqueAllocatorClass;

struct _GstMMALOpaqueAllocator
{
  GstAllocator parent;
};

struct _GstMMALOpaqueAllocatorClass
{
  GstAllocatorClass parent_class;
};

G_DEFINE_TYPE (GstMMALOpaqueAllocator, gst_mmal_opaque_allocator,
    GST_TYPE_ALLOCATOR);


static GstMemory *
gst_mmal_opaque_allocator_alloc (GstAllocator * allocator, gsize size,
    GstAllocationParams * params)
{
  GstMMALOpaqueMemory *mem = g_slice_new (GstMMALOpaqueMemory);

  gst_memory_init (GST_MEMORY_CAST (mem), GST_MEMORY_FLAG_NOT_MAPPABLE,
      allocator, NULL, size, 31, 0, size);

  /* We will have to fill this in later, once we have it.
     The client will do this by calling:
     gst_mmal_opaque_mem_set_mmal_header()
   */
  mem->mmal_buffer_header = NULL;

  return GST_MEMORY_CAST (mem);
}

static void
gst_mmal_opaque_allocator_free (GstAllocator * allocator, GstMemory * mem)
{
  GstMMALOpaqueMemory *mmem = NULL;

  g_return_if_fail (gst_is_mmal_opaque_memory (mem));

  mmem = GST_MMAL_OPAQUE_MEMORY (mem);

  /* If the memory is still holding an MMAL buffer header then we should deref
     it first.
   */
  gst_mmal_opaque_mem_set_mmal_header (mem, NULL);

  g_slice_free (GstMMALOpaqueMemory, mmem);
}

static gpointer
gst_mmal_opaque_mem_map (GstMemory * mem, gsize maxsize, GstMapFlags flags)
{
  return NULL;
}

static void
gst_mmal_opaque_mem_unmap (GstMemory * mem)
{
}

static GstMemory *
gst_mmal_opaque_mem_share (GstMemory * mem, gssize offset, gssize size)
{
  return NULL;
}

static GstMemory *
gst_mmal_opaque_mem_copy (GstMemory * mem, gssize offset, gssize size)
{
  return NULL;
}

static gboolean
gst_mmal_opaque_mem_is_span (GstMemory * mem1, GstMemory * mem2, gsize * offset)
{
  return FALSE;
}

static void
gst_mmal_opaque_allocator_class_init (GstMMALOpaqueAllocatorClass * klass)
{
  GstAllocatorClass *allocator_class = (GstAllocatorClass *) klass;

  allocator_class->alloc = gst_mmal_opaque_allocator_alloc;
  allocator_class->free = gst_mmal_opaque_allocator_free;
}

static void
gst_mmal_opaque_allocator_init (GstMMALOpaqueAllocator * allocator)
{
  GstAllocator *alloc = GST_ALLOCATOR_CAST (allocator);

  alloc->mem_type = GST_MMAL_OPAQUE_MEMORY_TYPE;
  alloc->mem_map = gst_mmal_opaque_mem_map;
  alloc->mem_unmap = gst_mmal_opaque_mem_unmap;
  alloc->mem_share = gst_mmal_opaque_mem_share;
  alloc->mem_copy = gst_mmal_opaque_mem_copy;
  alloc->mem_is_span = gst_mmal_opaque_mem_is_span;

  GST_OBJECT_FLAG_SET (allocator, GST_ALLOCATOR_FLAG_CUSTOM_ALLOC);
}


/* --- MMAL OPAQUE BUFFER POOL --- */

typedef struct _GstMMALOpaqueBufferPool GstMMALOpaqueBufferPool;
typedef struct _GstMMALOpaqueBufferPoolClass GstMMALOpaqueBufferPoolClass;

#define GST_TYPE_MMAL_OPAQUE_BUFFER_POOL      (gst_mmal_opaque_buffer_pool_get_type())

#define GST_IS_MMAL_OPAQUE_BUFFER_POOL(obj)   (G_TYPE_CHECK_INSTANCE_TYPE ((obj), GST_TYPE_MMAL_OPAQUE_BUFFER_POOL))

#define GST_MMAL_OPAQUE_BUFFER_POOL(obj)      (G_TYPE_CHECK_INSTANCE_CAST ((obj), GST_TYPE_MMAL_OPAQUE_BUFFER_POOL, GstMMALOpaqueBufferPool))

#define GST_MMAL_OPAQUE_BUFFER_POOL_CAST(obj) ((GstMMALOpaqueBufferPool*)(obj))


GType gst_mmal_opaque_buffer_pool_get_type (void);

struct _GstMMALOpaqueBufferPool
{
  GstVideoBufferPool bufferpool;
};

struct _GstMMALOpaqueBufferPoolClass
{
  GstVideoBufferPoolClass parent_class;
};


#define gst_mmal_opaque_buffer_pool_parent_class parent_class
G_DEFINE_TYPE (GstMMALOpaqueBufferPool, gst_mmal_opaque_buffer_pool,
    GST_TYPE_VIDEO_BUFFER_POOL);

GstBufferPool *
gst_mmal_opaque_buffer_pool_new ()
{
  GstMMALOpaqueBufferPool *pool;

  pool = g_object_new (GST_TYPE_MMAL_OPAQUE_BUFFER_POOL, NULL);

  return GST_BUFFER_POOL_CAST (pool);
}


/**
 * This should get called by the pool when the buffer is returned to it.
 * We need to release the reference that the GstMemory is holding on the MMAL
 * buffer header, or it won't be returned to the (MMAL) pool.
 */
static void
gst_mmal_opaque_buffer_pool_reset_buffer (GstBufferPool * pool,
    GstBuffer * buffer)
{
  gst_mmal_opaque_mem_set_mmal_header (gst_buffer_peek_memory (buffer, 0),
      NULL);

  GST_BUFFER_POOL_CLASS (parent_class)->reset_buffer (pool, buffer);
}

static void
gst_mmal_opaque_buffer_pool_class_init (GstMMALOpaqueBufferPoolClass * klass)
{
  GstBufferPoolClass *gstbufferpool_class = (GstBufferPoolClass *) klass;

  gstbufferpool_class->reset_buffer =
      GST_DEBUG_FUNCPTR (gst_mmal_opaque_buffer_pool_reset_buffer);
}

static void
gst_mmal_opaque_buffer_pool_init (GstMMALOpaqueBufferPool * pool)
{
}
