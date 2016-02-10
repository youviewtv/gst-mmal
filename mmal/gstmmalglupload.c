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
#include "gstmmalglupload.h"
#include "gstmmalmemory.h"

#include <gst/video/video.h>
#include <gst/gst.h>
#define GST_USE_UNSTABLE_API
#include <gst/gl/gl.h>
#include <gst/gl/egl/gstglcontext_egl.h>
#include <gst/video/gstvideoaffinetransformationmeta.h>

#include <glib/gstdio.h>

#include <bcm_host.h>

#include <interface/mmal/mmal.h>
#include <interface/mmal/util/mmal_util.h>
#include <interface/mmal/util/mmal_util_params.h>
#include <interface/mmal/util/mmal_default_components.h>

#include <time.h>

GST_DEBUG_CATEGORY_STATIC (gst_mmal_gl_upload_debug_category);
#define GST_CAT_DEFAULT gst_mmal_gl_upload_debug_category

/* Element structure */
#define GST_MMAL_GL_UPLOAD(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_MMAL_GL_UPLOAD, GstMMALGLUpload))

#define GST_MMAL_GL_UPLOAD_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass), GST_TYPE_MMAL_GL_UPLOAD, GstMMALGLUploadClass))


typedef struct _GstMMALGLUpload GstMMALGLUpload;
typedef struct _GstMMALGLUploadClass GstMMALGLUploadClass;
typedef struct _EGLImageCacheEntry EGLImageCacheEntry;

struct _GstMMALGLUpload
{
  GstElement parent;

  GstPad *sink_pad;
  GstPad *src_pad;

  GstMMALOpaqueAllocator *allocator;

  GstVideoInfo output_video_info;

  GstBufferPool *output_pool;

  GArray *eglimage_cache;
};

struct _GstMMALGLUploadClass
{
  GstElementClass parent_class;
};

struct _EGLImageCacheEntry
{
  void *buff_ptr;
  EGLImageKHR eglimage;
};


/* Pads */

static GstStaticPadTemplate gst_mmal_gl_upload_sink_factory =
GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (GST_VIDEO_CAPS_MAKE_WITH_FEATURES
        (GST_CAPS_FEATURE_MEMORY_MMAL_OPAQUE,
            "{ I420 }")));


#define MMAL_GL_SRC_CAPS \
    "video/x-raw(" GST_CAPS_FEATURE_MEMORY_GL_MEMORY "), "              \
    "format = (string) RGBA, "                                          \
    "width = " GST_VIDEO_SIZE_RANGE ", "                                \
    "height = " GST_VIDEO_SIZE_RANGE ", "                               \
    "framerate = " GST_VIDEO_FPS_RANGE ", "                             \
    "texture-target = (string) { external-oes } "                       \


static GstStaticPadTemplate gst_mmal_gl_upload_src_factory =
GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (MMAL_GL_SRC_CAPS));

#define DEBUG_INIT \
    GST_DEBUG_CATEGORY_INIT (gst_mmal_gl_upload_debug_category, \
      "mmalglupload", 0, "debug category for gst-mmal gl upload");

/* Class initialization */
G_DEFINE_TYPE_WITH_CODE (GstMMALGLUpload, gst_mmal_gl_upload,
    GST_TYPE_ELEMENT, DEBUG_INIT);


static gboolean gst_mmal_gl_upload_open (GstMMALGLUpload * self);

static gboolean gst_mmal_gl_upload_close (GstMMALGLUpload * self);

static GstStateChangeReturn gst_mmal_gl_upload_change_state (GstElement *
    element, GstStateChange transition);

static gboolean gst_mmal_gl_upload_sink_query (GstPad * pad, GstObject * parent,
    GstQuery * query);

static gboolean gst_mmal_gl_upload_set_caps (GstMMALGLUpload * self,
    GstPad * pad, GstCaps * caps);

static gboolean gst_mmal_gl_upload_sink_event (GstPad * pad, GstObject * parent,
    GstEvent * event);

static GstFlowReturn gst_mmal_gl_upload_chain (GstPad * pad, GstObject * object,
    GstBuffer * buf);

static gboolean gst_mmal_gl_upload_clean_eglimage_cache (GstMMALGLUpload *
    self);

static const gfloat yflip_matrix[16] = {
  1.0f, 0.0f, 0.0f, 0.0f,
  0.0f, -1.0f, 0.0f, 0.0f,
  0.0f, 0.0f, 1.0f, 0.0f,
  0.0f, 1.0f, 0.0f, 1.0f
};


static void
gst_mmal_gl_upload_class_init (GstMMALGLUploadClass * klass)
{
  GstElementClass *element_class = NULL;
  GstPadTemplate *sink_pad_template = NULL;
  GstPadTemplate *src_pad_template = NULL;

  g_return_if_fail (klass != NULL);

  element_class = GST_ELEMENT_CLASS (klass);

  /* Sink pad */
  sink_pad_template =
      gst_static_pad_template_get (&gst_mmal_gl_upload_sink_factory);
  gst_element_class_add_pad_template (element_class, sink_pad_template);

  /* Src pad */
  src_pad_template =
      gst_static_pad_template_get (&gst_mmal_gl_upload_src_factory);

  gst_element_class_add_pad_template (element_class, src_pad_template);

  gst_element_class_set_static_metadata (element_class,
      "MMAL GL Uploader [experimental]",
      "Filter/Video",
      "Uploads MMAL opaque buffers to GL textures.",
      "John Sadler <john.sadler@youview.com>");

  element_class->change_state = gst_mmal_gl_upload_change_state;
}


static void
gst_mmal_gl_upload_init (GstMMALGLUpload * self)
{
  g_return_if_fail (self != NULL);

  self->sink_pad =
      gst_pad_new_from_static_template (&gst_mmal_gl_upload_sink_factory,
      "sink");

  gst_pad_set_chain_function (self->sink_pad,
      GST_DEBUG_FUNCPTR (gst_mmal_gl_upload_chain));

  gst_pad_set_event_function (self->sink_pad,
      GST_DEBUG_FUNCPTR (gst_mmal_gl_upload_sink_event));

  gst_pad_set_query_function (self->sink_pad,
      GST_DEBUG_FUNCPTR (gst_mmal_gl_upload_sink_query));

  gst_element_add_pad (GST_ELEMENT (self), self->sink_pad);

  self->src_pad =
      gst_pad_new_from_static_template (&gst_mmal_gl_upload_src_factory, "src");

  gst_element_add_pad (GST_ELEMENT (self), self->src_pad);

  self->allocator = NULL;
  self->output_pool = NULL;
  self->eglimage_cache = NULL;
}


static gboolean
gst_mmal_gl_upload_open (GstMMALGLUpload * self)
{
  g_return_val_if_fail (self != NULL, FALSE);

  bcm_host_init ();

  self->allocator = g_object_new (gst_mmal_opaque_allocator_get_type (), NULL);

  self->eglimage_cache = g_array_new (FALSE, FALSE,
      sizeof (EGLImageCacheEntry));

  return TRUE;
}


static gboolean
gst_mmal_gl_upload_close (GstMMALGLUpload * self)
{
  g_return_val_if_fail (self != NULL, FALSE);

  gst_mmal_gl_upload_clean_eglimage_cache (self);

  g_array_free (self->eglimage_cache, TRUE);
  self->eglimage_cache = NULL;

  g_clear_object (&self->allocator);
  g_clear_object (&self->output_pool);

  bcm_host_deinit ();

  return TRUE;
}


static GstStateChangeReturn
gst_mmal_gl_upload_change_state (GstElement * element,
    GstStateChange transition)
{
  GstMMALGLUpload *self = NULL;
  GstStateChangeReturn ret = GST_STATE_CHANGE_SUCCESS;

  g_return_val_if_fail (GST_IS_MMAL_GL_UPLOAD (element),
      GST_STATE_CHANGE_FAILURE);

  self = GST_MMAL_GL_UPLOAD (element);

  GST_DEBUG_OBJECT (self, "state transition: %s -> %s",
      gst_element_state_get_name (GST_STATE_TRANSITION_CURRENT (transition)),
      gst_element_state_get_name (GST_STATE_TRANSITION_NEXT (transition)));

  switch (transition) {
    case GST_STATE_CHANGE_NULL_TO_READY:
      if (!gst_mmal_gl_upload_open (self)) {
        return GST_STATE_CHANGE_FAILURE;
      }
      break;
    case GST_STATE_CHANGE_READY_TO_PAUSED:
      break;
    case GST_STATE_CHANGE_PAUSED_TO_PLAYING:
      break;
    default:
      break;
  }

  if (ret != GST_STATE_CHANGE_SUCCESS) {
    return ret;
  }

  ret =
      GST_ELEMENT_CLASS (gst_mmal_gl_upload_parent_class)->change_state
      (element, transition);

  if (ret == GST_STATE_CHANGE_FAILURE)
    return ret;

  switch (transition) {
    case GST_STATE_CHANGE_PLAYING_TO_PAUSED:
      break;
    case GST_STATE_CHANGE_PAUSED_TO_READY:
      break;
    case GST_STATE_CHANGE_READY_TO_NULL:
      if (!gst_mmal_gl_upload_close (self)) {
        return GST_STATE_CHANGE_FAILURE;
      }
      break;
    default:
      break;
  }

  return ret;
}


static gboolean
gst_mmal_gl_upload_sink_query (GstPad * pad, GstObject * parent,
    GstQuery * query)
{
  GstMMALGLUpload *self = NULL;
  gboolean res = FALSE;

  g_return_val_if_fail (pad != NULL, GST_FLOW_ERROR);
  g_return_val_if_fail (parent != NULL, GST_FLOW_ERROR);
  g_return_val_if_fail (query != NULL, GST_FLOW_ERROR);

  GST_LOG_OBJECT (pad, "Query: %s", GST_QUERY_TYPE_NAME (query));

  self = GST_MMAL_GL_UPLOAD (parent);

  switch (GST_QUERY_TYPE (query)) {
    case GST_QUERY_ALLOCATION:

      /* Need to add opaque allocator to tell upstream we support MMAL
         opaque buffers.
       */
      if (self->allocator) {
        GST_DEBUG_OBJECT (self, "Adding opaque allocator...");
        gst_query_add_allocation_param (query, GST_ALLOCATOR (self->allocator),
            NULL);
        res = TRUE;
      } else {
        GST_ERROR_OBJECT (self, "Opaque allocator is NULL!");
      }
      break;
    default:
      res = gst_pad_query_default (pad, parent, query);
      break;
  }
  return res;
}


static gboolean
gst_mmal_gl_upload_decide_allocation (GstMMALGLUpload * self, GstQuery * query)
{
  GstBufferPool *pool = NULL;
  GstStructure *config;

  GstCaps *caps;

  GST_DEBUG_OBJECT (self, "Deciding allocation...");

  gst_query_parse_allocation (query, &caps, NULL);

  g_assert (gst_query_get_n_allocation_pools (query) > 0);
  gst_query_parse_nth_allocation_pool (query, 0, &pool, NULL, NULL, NULL);

  if (pool == NULL) {
    GST_ERROR_OBJECT (self, "Failed to get pool from downstream!");
    goto error;
  } else if (!GST_IS_GL_BUFFER_POOL (pool)) {
    GST_ERROR_OBJECT (self, "Expected a GL buffer pool!");
    goto error;
  }

  config = gst_buffer_pool_get_config (pool);

  if (gst_query_find_allocation_meta (query, GST_VIDEO_META_API_TYPE, NULL)) {
    gst_buffer_pool_config_add_option (config,
        GST_BUFFER_POOL_OPTION_VIDEO_META);
  }

  gst_buffer_pool_set_config (pool, config);

  gst_object_replace ((GstObject **) & self->output_pool, (GstObject *) pool);

  gst_object_unref (pool);

  return TRUE;

error:
  gst_object_unref (pool);

  return FALSE;
}


static gboolean
gst_mmal_gl_upload_negotiate_pool (GstMMALGLUpload * self, GstCaps * caps)
{
  gboolean ret = TRUE;
  GstQuery *alloc_query = NULL;
  GstBufferPool *pool = NULL;

  alloc_query = gst_query_new_allocation (caps, TRUE);

  if (!gst_pad_peer_query (self->src_pad, alloc_query)) {
    GST_ERROR_OBJECT (self, "Downstream didn't respond to allocation query!");
    ret = FALSE;
    goto done;
  }

  if (!gst_mmal_gl_upload_decide_allocation (self, alloc_query)) {
    GST_ERROR_OBJECT (self, "Failed to decide allocation!");
    ret = FALSE;
    goto done;
  }

  if (gst_query_get_n_allocation_pools (alloc_query) > 0) {
    gst_query_parse_nth_allocation_pool (alloc_query, 0, &pool, NULL, NULL,
        NULL);
  }

  if (!pool) {
    GST_ERROR_OBJECT (self, "No pool offered!");
    ret = FALSE;
    goto done;
  }

  gst_object_unref (self->output_pool);
  self->output_pool = pool;

  gst_buffer_pool_set_active (pool, TRUE);

done:

  gst_query_unref (alloc_query);

  return ret;
}


static gboolean
gst_mmal_gl_upload_set_caps (GstMMALGLUpload * self, GstPad * pad,
    GstCaps * caps)
{
  gboolean ret = TRUE;
  GstCaps *src_caps = NULL;

  g_return_val_if_fail (self != NULL, FALSE);
  g_return_val_if_fail (pad != NULL, FALSE);
  g_return_val_if_fail (caps != NULL, FALSE);

  GST_DEBUG_OBJECT (self, "Sink caps: %" GST_PTR_FORMAT, caps);

  /* New caps, bin any cached EGLImages as the upstream MMAL pool might have
     changed.
   */
  gst_mmal_gl_upload_clean_eglimage_cache (self);

  src_caps = gst_caps_copy (caps);
  src_caps = gst_caps_make_writable (src_caps);

  /* tell on the output only GLMemory memory type is supported */
  gst_caps_set_features (src_caps, 0,
      gst_caps_features_new (GST_CAPS_FEATURE_MEMORY_GL_MEMORY, NULL));

  /* RGBA */
  gst_caps_set_simple (src_caps, "format", G_TYPE_STRING, "RGBA", NULL);
  gst_caps_set_simple (src_caps, "texture-target", G_TYPE_STRING,
      "external-oes", NULL);

  GST_DEBUG_OBJECT (self, "Src caps: %" GST_PTR_FORMAT, src_caps);

  GST_PAD_STREAM_LOCK (self->src_pad);

  if (!gst_pad_set_caps (self->src_pad, src_caps)) {
    GST_ERROR_OBJECT (self, "Failed to set caps on src pad");
    ret = FALSE;
    goto done;
  }

  if (!gst_mmal_gl_upload_negotiate_pool (self, src_caps)) {
    GST_ERROR_OBJECT (self, "Failed to negotiate output pool!");
    ret = FALSE;
    goto done;
  }

  if (!gst_video_info_from_caps (&self->output_video_info, src_caps)) {
    GST_ERROR_OBJECT (self, "Failed to parse-out video info from caps!");
    ret = FALSE;
    goto done;
  }

done:

  GST_PAD_STREAM_UNLOCK (self->src_pad);

  gst_caps_unref (src_caps);

  return ret;
}


/* Called on the input thread (sink pad) */
static gboolean
gst_mmal_gl_upload_sink_event (GstPad * pad, GstObject * parent,
    GstEvent * event)
{
  gboolean ret = TRUE;
  GstMMALGLUpload *self = NULL;

  g_return_val_if_fail (parent != NULL, FALSE);
  g_return_val_if_fail (pad != NULL, FALSE);
  g_return_val_if_fail (event != NULL, FALSE);

  self = GST_MMAL_GL_UPLOAD (parent);

  GST_LOG_OBJECT (pad, "Received %s event: %" GST_PTR_FORMAT,
      GST_EVENT_TYPE_NAME (event), event);

  switch (GST_EVENT_TYPE (event)) {
    case GST_EVENT_CAPS:
    {
      GstCaps *caps = NULL;

      gst_event_parse_caps (event, &caps);
      ret = gst_mmal_gl_upload_set_caps (self, pad, caps);
      gst_event_unref (event);
      break;
    }
    case GST_EVENT_SEGMENT_DONE:
    case GST_EVENT_EOS:
      ret = ret && gst_pad_push_event (self->src_pad, event);
      break;

    case GST_EVENT_FLUSH_STOP:
      ret = ret && gst_pad_push_event (self->src_pad, event);
      break;

    default:
      ret = gst_pad_event_default (pad, parent, event);
      break;
  }

  return ret;
}


/**
 * This is run on the GL thread.
 * It is called by: gst_mmal_gl_upload_create_lookup_or_create_eglimage()
 */
static void
gst_mmal_gl_upload_create_eglimage (GstGLContext * context, gpointer data)
{
  GstGLMemory *gl_mem = (GstGLMemory *) data;

  GstGLContextEGL *gst_egl_context = GST_GL_CONTEXT_EGL (context);

  gl_mem->mem.data = eglCreateImageKHR (gst_egl_context->egl_display,
      EGL_NO_CONTEXT, EGL_IMAGE_BRCM_MULTIMEDIA, gl_mem->mem.data, NULL);
}


/**
 * Because it can take a relatively long time to destroy EGLImage instances
 * (>= 20ms when timed), we don't want to be doing that all of the time.
 *
 * We therefore keep a cache which maps MMAL opaque buffer ptr to the
 * corresponding EGLImage.  This relies on the knowledge that upstream will be
 * providing MMAL buffers from a fixed-size pool.
 *
 * Called from: gst_mmal_gl_upload_chain()
 */
static gboolean
gst_mmal_gl_upload_create_lookup_or_create_eglimage (GstMMALGLUpload * self,
    GstGLContext * context, GstGLMemory * gl_mem, void *buff_ptr)
{
  EGLImageKHR eglimage = EGL_NO_IMAGE_KHR;

  size_t i;
  EGLImageCacheEntry *entry;

  for (i = 0; i < self->eglimage_cache->len; ++i) {

    entry = &g_array_index (self->eglimage_cache, EGLImageCacheEntry, i);

    if (entry->buff_ptr == buff_ptr) {
      eglimage = entry->eglimage;
      break;
    }
  }

  if (eglimage == EGL_NO_IMAGE_KHR) {

    GST_DEBUG_OBJECT (self, "No EGLImage cached for buffer: %p.  Creating...",
        buff_ptr);

    gl_mem->mem.user_data = self;
    gl_mem->mem.data = buff_ptr;

    /* This will run gst_mmal_gl_upload_create_eglimage() on the GL thread and
       block until it's done.
       FIXME: `gstglcontext.h` has a FIXME to remove this function.  Not sure
       what we would replace it with, but this could be a problem in the future.
     */
    gst_gl_context_thread_add (context, &gst_mmal_gl_upload_create_eglimage,
        gl_mem);

    eglimage = (EGLImageKHR) gl_mem->mem.data;

    {
      EGLImageCacheEntry entry = { buff_ptr, eglimage };

      g_array_append_val (self->eglimage_cache, entry);
    }

  } else {
    gl_mem->mem.data = eglimage;
  }

  if (G_UNLIKELY (eglimage == EGL_NO_IMAGE_KHR)) {
    GST_ERROR_OBJECT (self, "Failed to create EGLImage for MMAL buffer: %p",
        buff_ptr);
    return FALSE;
  }

  GST_DEBUG_OBJECT (self, "EGLImage: %p, MMAL Opaque ptr: %p", eglimage,
      buff_ptr);

  return TRUE;
}


/**
 * This is run on GL thread.
 *
 * See: gst_mmal_gl_upload_clean_eglimage_cache()
 */
static void
gst_mmal_gl_upload_destroy_eglimages (GstGLContext * context, gpointer data)
{
  GstMMALGLUpload *self = GST_MMAL_GL_UPLOAD (data);
  GstGLContextEGL *gst_egl_context = GST_GL_CONTEXT_EGL (context);
  GArray *eglimage_cache = self->eglimage_cache;
  guint num_images;
  guint idx;
  EGLImageCacheEntry *cache_entry;

  if (eglimage_cache) {

    num_images = eglimage_cache->len;

    for (idx = 0; idx < num_images; ++idx) {

      cache_entry = &g_array_index (eglimage_cache, EGLImageCacheEntry, idx);

      GST_DEBUG_OBJECT (self, "Destroying EGLImage: %p", cache_entry->eglimage);

      eglDestroyImageKHR (gst_egl_context->egl_display, cache_entry->eglimage);
    }

    self->eglimage_cache = g_array_set_size (eglimage_cache, 0);
  }
}


/**
 * Clear-out the EGLImage cache.
 *
 * We should do this anytime that the underlying MMAL buffer pool might change.
 * as we don't want to keep around EGLImage mappings for buffers that may not
 * exist in the upstream MMAL pool.  For instance, this will be called on caps
 * change.
 */
static gboolean
gst_mmal_gl_upload_clean_eglimage_cache (GstMMALGLUpload * self)
{
  GstGLBufferPool *gl_pool;
  GstGLContext *gl_context;

  if (!self->output_pool) {
    /* The cache is probably empty if we don't even have output pool yet */
    return TRUE;
  }

  if (G_UNLIKELY (!GST_IS_GL_BUFFER_POOL (self->output_pool))) {
    GST_ERROR_OBJECT (self, "Can't get GL context, not a GL buffer pool!");
    return FALSE;
  }

  gl_pool = GST_GL_BUFFER_POOL (self->output_pool);
  gl_context = gl_pool->context;

  /* This will wait for the function to complete on GL thread. */
  gst_gl_context_thread_add (gl_context, &gst_mmal_gl_upload_destroy_eglimages,
      self);

  return TRUE;
}


/**
 * This is required just to prevent a segfault when the GstSyncMeta objects are
 * destroyed.
 */
static void
gst_mmal_gl_upload_sync_meta_free_gl (GstGLSyncMeta * sync, GstGLContext *
    context)
{
}


/**
 * This gets called when the downstream element (e.g. `glimagesinkelement`)
 * calls wait_gl() on the GstSyncMeta attached to the GstBuffer.
 *
 * We exploit this to do the glEGLImageTargetTexture2DOES() call on the GL
 * thread, rather than using gst_gl_context_thread_add() to schedule the work
 * on the GL thread when we create the output buffer.  This is because a
 * blocking call to gst_gl_context_thread_add() has been seen to take 70ms or
 * more, and we don't want to block our upstream thread for that long.
 *
 * It's a bit hacky, but there you go.
 */
static void
gst_mmal_gl_upload_sync_meta_wait_gl (GstGLSyncMeta * sync, GstGLContext *
    context)
{
  GstGLMemory *gl_mem = (GstGLMemory *) sync->data;
  EGLImageKHR eglimage = (EGLImageKHR) gl_mem->mem.data;

  if (G_LIKELY (eglimage != EGL_NO_IMAGE_KHR)) {
    glBindTexture (GL_TEXTURE_EXTERNAL_OES, gl_mem->tex_id);
    glEGLImageTargetTexture2DOES (GL_TEXTURE_EXTERNAL_OES, eglimage);
  }
}


static GstFlowReturn
gst_mmal_gl_upload_chain (GstPad * pad, GstObject * object, GstBuffer * buf)
{
  GstMMALGLUpload *self = NULL;
  GstFlowReturn flow_ret = GST_FLOW_OK;
  GstMemory *in_mem;
  MMAL_BUFFER_HEADER_T *mmal_buffer = NULL;
  GstBuffer *output_buffer = NULL;
  GstMemory *output_mem = NULL;
  GstGLMemory *gl_mem = NULL;
  GstGLContext *context = NULL;
  GstVideoAffineTransformationMeta *af_meta = NULL;
  GstGLSyncMeta *sync_meta = NULL;

  g_return_val_if_fail (pad != NULL, GST_FLOW_ERROR);
  g_return_val_if_fail (object != NULL, GST_FLOW_ERROR);
  g_return_val_if_fail (buf != NULL, GST_FLOW_ERROR);

  self = GST_MMAL_GL_UPLOAD (object);

  GST_DEBUG_OBJECT (self,
      "Input buffer: %p (PTS: %" GST_TIME_FORMAT ")",
      buf, GST_TIME_ARGS (GST_BUFFER_PTS (buf)));

  /* Process input buffer */

  /* We support only MMAL Opaque buffers on input side */
  in_mem = gst_buffer_peek_memory (buf, 0);

  if (G_UNLIKELY (!gst_is_mmal_opaque_memory (in_mem))) {

    GST_ERROR_OBJECT (self,
        "Not a MMAL Opaque buffer: %p (PTS: %" GST_TIME_FORMAT ")",
        buf, GST_TIME_ARGS (GST_BUFFER_PTS (buf)));

    flow_ret = GST_FLOW_ERROR;
    goto done;
  }

  /* Just extract MMAL buffer header from GstMemory */
  mmal_buffer = gst_mmal_opaque_mem_get_mmal_header (in_mem);

  if (G_UNLIKELY (mmal_buffer == NULL)) {
    GST_ERROR_OBJECT (self, "Failed to get MMAL Buffer Header from buffer (%p)",
        buf);
    flow_ret = GST_FLOW_ERROR;
    goto done;
  }

  if (G_UNLIKELY (mmal_buffer->cmd)) {
    GST_DEBUG_OBJECT (self, "Command buffer.  Ignoring.");
    goto done;
  }

  if (G_UNLIKELY (mmal_buffer->flags & MMAL_BUFFER_HEADER_FLAG_EOS)) {
    GST_DEBUG_OBJECT (self, "Buffer signals EOS.  Ignoring.");
    goto done;
  }

  if (G_UNLIKELY ((flow_ret = gst_buffer_pool_acquire_buffer (self->output_pool,
                  &output_buffer, NULL)) != GST_FLOW_OK)) {

    GST_ERROR_OBJECT (self, "Failed to acquire buffer from output pool.");
    goto done;
  }

  output_mem = gst_buffer_peek_memory (output_buffer, 0);

  if (G_UNLIKELY (!output_mem || !gst_is_gl_memory (output_mem))) {
    GST_ERROR_OBJECT (self, "Not GLMemory!");
    flow_ret = GST_FLOW_ERROR;
    goto done;
  }

  gl_mem = (GstGLMemory *) output_mem;

  GST_BUFFER_PTS (output_buffer) = GST_BUFFER_PTS (buf);
  GST_BUFFER_DTS (output_buffer) = GST_BUFFER_DTS (buf);

  /* Hold onto the original buffer (and by extension the MMAL buffer header)
     until the output buffer is dropped.  We don't want the MMAL opaque mem to
     be recycled whilst in use.

     Making the input buffer a parent of the output buffer should achieve this.
   */
  gst_buffer_add_parent_buffer_meta (output_buffer, buf);

  context = gl_mem->mem.context;

  if (G_UNLIKELY (!GST_IS_GL_CONTEXT_EGL (context))) {
    GST_ERROR_OBJECT (self, "Not an EGL context!!");
    flow_ret = GST_FLOW_ERROR;
    goto done;
  }

  /* We only create the EGLImage at this point (or pull it out of cache, if we
     already have an EGLImage for the MMAL buffer.  We don't bind the texture
     to it until later (see: gst_mmal_gl_upload_sync_meta_wait_gl()) */
  gst_mmal_gl_upload_create_lookup_or_create_eglimage (self, context, gl_mem,
      mmal_buffer->data);

  /* N.B. This is to correct upside-down image.  `glimagesink` will pick-up the
     transformation matrix from the meta and apply it. */
  af_meta = gst_buffer_add_video_affine_transformation_meta (output_buffer);
  gst_video_affine_transformation_meta_apply_matrix (af_meta, yflip_matrix);


  sync_meta = gst_buffer_get_gl_sync_meta (output_buffer);

  if (G_LIKELY (sync_meta)) {

    sync_meta->data = gl_mem;
    sync_meta->wait_gl = &gst_mmal_gl_upload_sync_meta_wait_gl;
    sync_meta->free_gl = &gst_mmal_gl_upload_sync_meta_free_gl;

  } else {

    GST_ERROR_OBJECT (self, "Expected output buffer to have GstGLSyncMeta!");

    /* We could add the sync meta ourselves, but the GLBufferPool should be
       doing that aready. */

    flow_ret = GST_FLOW_ERROR;
    goto done;
  }

  GST_DEBUG_OBJECT (self, "Pushing GL buffer: %p downstream (PTS: %"
      GST_TIME_FORMAT ")", output_buffer,
      GST_TIME_ARGS (GST_BUFFER_PTS (output_buffer)));

  flow_ret = gst_pad_push (self->src_pad, output_buffer);

done:

  if (G_UNLIKELY (flow_ret != GST_FLOW_OK)) {
    gst_buffer_unref (output_buffer);
  }

  gst_buffer_unref (buf);

  return flow_ret;
}
