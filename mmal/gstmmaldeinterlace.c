/*
 * Copyright (C) 2015, YouView TV Ltd.
 *   Author: Tomasz Szkutkowski <tomasz.szkutkowski@youview.com>
 *           Krzysztof Konopko <kris@youview.com>
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

#include "gstmmaldeinterlace.h"
#include "gstmmalmemory.h"

#include <gst/video/video.h>
#include <gst/gst.h>

#include <glib/gstdio.h>

#include <bcm_host.h>

#include <interface/mmal/mmal.h>
#include <interface/mmal/util/mmal_util.h>
#include <interface/mmal/util/mmal_util_params.h>
#include <interface/mmal/util/mmal_default_components.h>

#define MMAL_COMPONENT_DEFAULT_IMAGE_FX "vc.ril.image_fx"

#define GST_MMAL_DEINTERLACE_NUM_INPUT_BUFFERS 1

#define GST_MMAL_DEINTERLACE_INPUT_BUFFER_WAIT_FOR_MS 2000

/* N.B. Drain timeout is quite high because with low-bitrate, low-fps stream, we
   can end-up with many input frames buffered ahead of the EOS, and we want to
   wait for those to be sent downstream.
 */
#define GST_MMAL_DEINTERLACE_DRAIN_TIMEOUT_MS 10000

GST_DEBUG_CATEGORY_STATIC (gst_mmal_deinterlace_debug_category);
#define GST_CAT_DEFAULT gst_mmal_deinterlace_debug_category

/* Element structure */
#define GST_MMAL_DEINTERLACE(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_MMAL_DEINTERLACE, GstMMALDeinterlace))

#define GST_MMAL_DEINTERLACE_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass), GST_TYPE_MMAL_DEINTERLACE, GstMMALDeinterlaceClass))

#define GST_MMAL_DEINTERLACE_STREAM_LOCK(deinterlace) \
  g_rec_mutex_lock (&GST_MMAL_DEINTERLACE (deinterlace)->stream_lock)

#define GST_MMAL_DEINTERLACE_STREAM_UNLOCK(deinterlace) \
  g_rec_mutex_unlock (&GST_MMAL_DEINTERLACE (deinterlace)->stream_lock)

typedef struct _GstMMALDeinterlace GstMMALDeinterlace;
typedef struct _GstMMALDeinterlaceClass GstMMALDeinterlaceClass;

struct _GstMMALDeinterlace
{
  /* ---- SHARED STATE (INPUT & OUTPUT) ---- */

  GstElement parent;

  /* Deinterlace component */
  MMAL_COMPONENT_T *image_fx;

  gboolean started;

  GstFlowReturn output_flow_ret;

  MMAL_PARAM_IMAGEFX_T deinterlace_mode;
  MMAL_INTERLACETYPE_T interlace_type;

  /* properties */
  gboolean want_repeat_first_field;
  gint frame_duration;

  /* main stream lock */
  GRecMutex stream_lock;


  /* ---- DRAINING STATE ---- */

  GMutex drain_lock;
  GCond drain_cond;
  gboolean draining;

  /* shared (atomic) */
  gint flushing;

  /* ---- INPUT STATE ---- */
  GstPad *sink_pad;

  /* This is set on caps change.  It tells chain() that the component needs
     reconfiguring.  We wait until we see the first frame because this is how
     we discover that opaque mode was selected by upstream.
   */
  gboolean need_reconfigure;

  GstVideoInfo video_info;

  /* Input buffer pool required for sending MMAL buffers with EOS and USER flags
   * set needed for draining and flushing.
   */
  MMAL_POOL_T *input_buffer_pool;

  GstClockTime last_upstream_ts;


  /* ---- OUTPUT STATE ---- */
  GstPad *src_pad;

  /* A pool for output GStreamer buffers holding MMAL buffers */
  GstBufferPool *output_gstpool;

  /* Output MMAL buffer pool */
  MMAL_POOL_T *output_buffer_pool;

  /* Deinterlaced MMAL buffer pool */
  MMAL_QUEUE_T *output_queue;
};

struct _GstMMALDeinterlaceClass
{
  GstElementClass parent_class;
};

/* Debug */

static const gchar *
get_mmal_deinterlace_mode_str (MMAL_PARAM_IMAGEFX_T mode)
{
  switch (mode) {
    case MMAL_PARAM_IMAGEFX_DEINTERLACE_DOUBLE:
      return "DOUBLE";
    case MMAL_PARAM_IMAGEFX_DEINTERLACE_ADV:
      return "ADV";
    case MMAL_PARAM_IMAGEFX_DEINTERLACE_FAST:
      return "FAST";
    default:
      break;
  }
  return "UNKNOWN";
}

static const gchar *
get_mmal_interlace_type_str (MMAL_INTERLACETYPE_T type)
{
  switch (type) {
    case MMAL_InterlaceProgressive:
      return "Progressive";
    case MMAL_InterlaceFieldSingleUpperFirst:
      return "SingleUpperFirst";
    case MMAL_InterlaceFieldSingleLowerFirst:
      return "SingleLowerFirst";
    case MMAL_InterlaceFieldsInterleavedUpperFirst:
      return "InterleavedUpperFirst";
    case MMAL_InterlaceFieldsInterleavedLowerFirst:
      return "InterleavedLowerFirst";
    case MMAL_InterlaceMixed:
      return "Mixed";
    default:
      break;
  }
  return "UNKNOWN";
}


/* Pads */
static GstStaticPadTemplate gst_mmal_deinterlace_src_factory =
GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (GST_VIDEO_CAPS_MAKE_WITH_FEATURES
        (GST_CAPS_FEATURE_MEMORY_MMAL_OPAQUE,
            "{ I420 }")));


static GstStaticPadTemplate gst_mmal_deinterlace_sink_factory =
GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (GST_VIDEO_CAPS_MAKE_WITH_FEATURES
        (GST_CAPS_FEATURE_MEMORY_MMAL_OPAQUE,
            "{ I420 }")));


/* Properties */
enum
{
  PROP_0,

  PROP_DEINTERLACE_ADV,
  PROP_REPEAT_FIRST_FIELD
};

#define DEBUG_INIT \
    GST_DEBUG_CATEGORY_INIT (gst_mmal_deinterlace_debug_category, \
      "mmaldeinterlace", 0, "debug category for gst-mmal video deinterlacer");

/* Class initialization */
G_DEFINE_TYPE_WITH_CODE (GstMMALDeinterlace, gst_mmal_deinterlace,
    GST_TYPE_ELEMENT, DEBUG_INIT);


/* Prototypes */
static void gst_mmal_deinterlace_finalize (GObject * object);
static GstStateChangeReturn gst_mmal_deinterlace_change_state (GstElement *
    element, GstStateChange transition);

static void gst_mmal_deinterlace_set_property (GObject * self, guint prop_id,
    const GValue * value, GParamSpec * pspec);
static void gst_mmal_deinterlace_get_property (GObject * self, guint prop_id,
    GValue * value, GParamSpec * pspec);

static gboolean gst_mmal_deinterlace_set_src_caps (GstMMALDeinterlace * self,
    GstPad * pad, GstCaps * caps);

static gboolean gst_mmal_deinterlace_sink_event (GstPad * pad,
    GstObject * parent, GstEvent * event);
static gboolean gst_mmal_deinterlace_sink_query (GstPad * pad,
    GstObject * parent, GstQuery * query);
static GstFlowReturn gst_mmal_deinterlace_chain (GstPad * pad,
    GstObject * object, GstBuffer * buf);

static void gst_mmal_deinterlace_start_output_task (GstMMALDeinterlace * self);
static void gst_mmal_deinterlace_output_task_loop (GstMMALDeinterlace * self);

static void gst_mmal_deinterlace_control_port_callback (MMAL_PORT_T * port,
    MMAL_BUFFER_HEADER_T * buffer);
static void gst_mmal_deinterlace_input_port_callback (MMAL_PORT_T * port,
    MMAL_BUFFER_HEADER_T * buffer);
static void gst_mmal_deinterlace_output_port_callback (MMAL_PORT_T * port,
    MMAL_BUFFER_HEADER_T * buffer);

static gboolean gst_mmal_deinterlace_alloc_output_gst_pool (GstMMALDeinterlace *
    self, GstQuery * query);

static gboolean gst_mmal_deinterlace_configure_image_fx (GstMMALDeinterlace *
    self);

static gboolean gst_mmal_deinterlace_open (GstMMALDeinterlace * self);
static gboolean gst_mmal_deinterlace_close (GstMMALDeinterlace * self);

static gboolean gst_mmal_deinterlace_start (GstMMALDeinterlace * self);
static gboolean gst_mmal_deinterlace_stop (GstMMALDeinterlace * self);

static GstFlowReturn gst_mmal_deinterlace_drain (GstMMALDeinterlace * self);

static gboolean gst_mmal_deinterlace_flush_start (GstMMALDeinterlace * self,
    gboolean stop);
static gboolean gst_mmal_deinterlace_flush_stop (GstMMALDeinterlace * self,
    gboolean stop);

static gboolean gst_mmal_deinterlace_ports_enable (GstMMALDeinterlace * self);
static gboolean gst_mmal_deinterlace_ports_disable (GstMMALDeinterlace * self);

static gboolean gst_mmal_deinterlace_flush_image_fx (GstMMALDeinterlace * self);
static gboolean gst_mmal_deinterlace_setup_image_fx (GstMMALDeinterlace * self);

static gboolean gst_mmal_deinterlace_populate_output_port (GstMMALDeinterlace *
    self);

/*----------------------------------------------------------------------------*/

static void
gst_mmal_deinterlace_class_init (GstMMALDeinterlaceClass * klass)
{
  GstElementClass *element_class = NULL;
  GObjectClass *gobject_class = NULL;
  GstPadTemplate *sink_pad_template = NULL;
  GstPadTemplate *src_pad_template = NULL;

  g_return_if_fail (klass != NULL);

  gobject_class = G_OBJECT_CLASS (klass);
  element_class = GST_ELEMENT_CLASS (klass);

  /* Sink pad */
  sink_pad_template =
      gst_static_pad_template_get (&gst_mmal_deinterlace_sink_factory);
  gst_element_class_add_pad_template (element_class, sink_pad_template);

  /* Src pad */
  src_pad_template =
      gst_static_pad_template_get (&gst_mmal_deinterlace_src_factory);
  gst_element_class_add_pad_template (element_class, src_pad_template);

  gst_element_class_set_static_metadata (element_class,
      "MMAL Video Deinterlacer",
      "Filter/Effect/Video/Deinterlace",
      "Video stream deinterlacer",
      "Tomasz Szkutkowski <tomasz.szkutkowski@youview.com>, "
      "Krzysztof Konopko <kris@youview.com>");

  gobject_class->set_property = gst_mmal_deinterlace_set_property;
  gobject_class->get_property = gst_mmal_deinterlace_get_property;
  gobject_class->finalize = gst_mmal_deinterlace_finalize;

  element_class->change_state = gst_mmal_deinterlace_change_state;

  /* Properties */
  g_object_class_install_property (gobject_class, PROP_DEINTERLACE_ADV,
      g_param_spec_boolean ("use-advanced-deinterlacer",
          "Use Advanced Deinterlacer", "Use Advanced Deinterlacer",
          FALSE, G_PARAM_READWRITE | GST_PARAM_CONTROLLABLE));

  g_object_class_install_property (gobject_class, PROP_REPEAT_FIRST_FIELD,
      g_param_spec_boolean ("repeat-first-field",
          "Repeat First Field", "Repeat First Field",
          FALSE, G_PARAM_READWRITE | GST_PARAM_CONTROLLABLE));
}


static void
gst_mmal_deinterlace_init (GstMMALDeinterlace * self)
{
  g_return_if_fail (self != NULL);

  self->sink_pad =
      gst_pad_new_from_static_template (&gst_mmal_deinterlace_sink_factory,
      "sink");

  gst_pad_set_chain_function (self->sink_pad,
      GST_DEBUG_FUNCPTR (gst_mmal_deinterlace_chain));

  gst_pad_set_event_function (self->sink_pad,
      GST_DEBUG_FUNCPTR (gst_mmal_deinterlace_sink_event));

  gst_pad_set_query_function (self->sink_pad,
      GST_DEBUG_FUNCPTR (gst_mmal_deinterlace_sink_query));

  gst_element_add_pad (GST_ELEMENT (self), self->sink_pad);


  self->src_pad =
      gst_pad_new_from_static_template (&gst_mmal_deinterlace_src_factory,
      "src");

  gst_element_add_pad (GST_ELEMENT (self), self->src_pad);

  g_rec_mutex_init (&self->stream_lock);

  self->started = FALSE;
  self->output_flow_ret = GST_FLOW_OK;

  self->need_reconfigure = TRUE;

  self->draining = FALSE;
  g_mutex_init (&self->drain_lock);
  g_cond_init (&self->drain_cond);

  self->flushing = FALSE;

  self->output_gstpool = NULL;

  /* MMAL */
  self->image_fx = NULL;
  self->input_buffer_pool = NULL;
  self->output_buffer_pool = NULL;
  self->output_queue = NULL;

  /* initial settings */
  self->deinterlace_mode = MMAL_PARAM_IMAGEFX_DEINTERLACE_FAST;
  self->interlace_type = MMAL_InterlaceMixed;
  self->want_repeat_first_field = FALSE;
  self->frame_duration = 0;

  self->last_upstream_ts = 0;
}

static void
gst_mmal_deinterlace_finalize (GObject * object)
{
  GstMMALDeinterlace *self = NULL;

  g_return_if_fail (object != NULL);
  self = GST_MMAL_DEINTERLACE (object);

  g_rec_mutex_clear (&self->stream_lock);

  g_mutex_clear (&self->drain_lock);
  g_cond_clear (&self->drain_cond);

  G_OBJECT_CLASS (gst_mmal_deinterlace_parent_class)->finalize (object);
}


static GstStateChangeReturn
gst_mmal_deinterlace_change_state (GstElement * element,
    GstStateChange transition)
{
  GstMMALDeinterlace *self = NULL;
  GstStateChangeReturn ret = GST_STATE_CHANGE_SUCCESS;

  g_return_val_if_fail (GST_IS_MMAL_DEINTERLACE (element),
      GST_STATE_CHANGE_FAILURE);

  self = GST_MMAL_DEINTERLACE (element);

  GST_DEBUG_OBJECT (self, "state transition: %s -> %s",
      gst_element_state_get_name (GST_STATE_TRANSITION_CURRENT (transition)),
      gst_element_state_get_name (GST_STATE_TRANSITION_NEXT (transition)));

  switch (transition) {
    case GST_STATE_CHANGE_NULL_TO_READY:
      if (!gst_mmal_deinterlace_open (self)) {
        return GST_STATE_CHANGE_FAILURE;
      }
      break;
    case GST_STATE_CHANGE_READY_TO_PAUSED:
      GST_MMAL_DEINTERLACE_STREAM_LOCK (self);

      if (!gst_mmal_deinterlace_start (self)) {
        ret = GST_STATE_CHANGE_FAILURE;
      }

      GST_MMAL_DEINTERLACE_STREAM_UNLOCK (self);
      break;
    case GST_STATE_CHANGE_PAUSED_TO_PLAYING:
      break;

    case GST_STATE_CHANGE_PAUSED_TO_READY:
      GST_MMAL_DEINTERLACE_STREAM_LOCK (self);

      if (!gst_mmal_deinterlace_flush_start (self, TRUE)) {
        ret = GST_STATE_CHANGE_FAILURE;
      }

      GST_MMAL_DEINTERLACE_STREAM_UNLOCK (self);

      break;

    default:
      break;
  }

  if (ret != GST_STATE_CHANGE_SUCCESS) {
    return ret;
  }

  ret =
      GST_ELEMENT_CLASS (gst_mmal_deinterlace_parent_class)->change_state
      (element, transition);

  if (ret == GST_STATE_CHANGE_FAILURE)
    return ret;

  switch (transition) {
    case GST_STATE_CHANGE_PLAYING_TO_PAUSED:
      break;
    case GST_STATE_CHANGE_PAUSED_TO_READY:
      GST_MMAL_DEINTERLACE_STREAM_LOCK (self);

      if (!gst_mmal_deinterlace_stop (self)) {
        ret = GST_STATE_CHANGE_FAILURE;
      }

      GST_MMAL_DEINTERLACE_STREAM_UNLOCK (self);

      break;
    case GST_STATE_CHANGE_READY_TO_NULL:
      if (!gst_mmal_deinterlace_close (self)) {
        return GST_STATE_CHANGE_FAILURE;
      }
      break;
    default:
      break;
  }

  return ret;
}

static void
gst_mmal_deinterlace_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec)
{
  GstMMALDeinterlace *self = NULL;

  g_return_if_fail (object != NULL);
  self = GST_MMAL_DEINTERLACE (object);

  switch (prop_id) {
    case PROP_DEINTERLACE_ADV:
      self->deinterlace_mode =
          g_value_get_boolean (value) ?
          MMAL_PARAM_IMAGEFX_DEINTERLACE_ADV
          : MMAL_PARAM_IMAGEFX_DEINTERLACE_FAST;
      break;
    case PROP_REPEAT_FIRST_FIELD:
      self->want_repeat_first_field = g_value_get_boolean (value);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

static void
gst_mmal_deinterlace_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec)
{
  GstMMALDeinterlace *self = NULL;

  g_return_if_fail (object != NULL);
  self = GST_MMAL_DEINTERLACE (object);

  switch (prop_id) {
    case PROP_DEINTERLACE_ADV:
      g_value_set_boolean (value,
          self->deinterlace_mode == MMAL_PARAM_IMAGEFX_DEINTERLACE_ADV);
      break;
    case PROP_REPEAT_FIRST_FIELD:
      g_value_set_boolean (value, self->want_repeat_first_field);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

/* Called on the input thread.  Needs the stream lock to be held. */
static gboolean
gst_mmal_deinterlace_deconfigure (GstMMALDeinterlace * self)
{
  GstFlowReturn flow_ret;

  GST_DEBUG_OBJECT (self, "Draining deinterlace...");

  flow_ret = gst_mmal_deinterlace_drain (self);
  if (flow_ret == GST_FLOW_FLUSHING) {
    return TRUE;
  } else if (flow_ret != GST_FLOW_OK) {
    GST_ERROR_OBJECT (self, "Drain failed!");
    return FALSE;
  }

  /* disable ports prior to any setup */
  if (!gst_mmal_deinterlace_ports_disable (self)) {
    GST_ERROR_OBJECT (self, "Failed to disable ports!");
    return FALSE;
  }

  if (!gst_mmal_deinterlace_flush_image_fx (self)) {
    return FALSE;
  }

  self->need_reconfigure = self->interlace_type != MMAL_InterlaceProgressive;

  return TRUE;
}

/* Called on the input thread.  Needs the stream lock to be held. */
static gboolean
gst_mmal_deinterlace_configure (GstMMALDeinterlace * self)
{
  /* set deinterlacing mode and type */
  if (!gst_mmal_deinterlace_configure_image_fx (self)) {
    GST_ERROR_OBJECT (self, "Failed to configure image_fx mode!");
    return FALSE;
  }

  /* setup video format and enable ports */
  if (!gst_mmal_deinterlace_setup_image_fx (self)) {
    GST_ERROR_OBJECT (self, "Failed to setup image_fx!");
    return FALSE;
  }

  /* setup_image_fx will enable ports after successfull reconfiguration */

  self->need_reconfigure = FALSE;

  return TRUE;
}

/* Called on the input thread (caps event).  Modifies output state. */
static gboolean
gst_mmal_deinterlace_set_src_caps (GstMMALDeinterlace * self, GstPad * pad,
    GstCaps * caps)
{
  gboolean ret = TRUE;
  GstCaps *src_caps = NULL;

  GstVideoInfo *vinfo = NULL;
  gint fps_n, fps_d;

  GstVideoInterlaceMode gst_interlacing_mode;

  g_return_val_if_fail (self != NULL, FALSE);
  g_return_val_if_fail (pad != NULL, FALSE);
  g_return_val_if_fail (caps != NULL, FALSE);

  GST_DEBUG_OBJECT (self, "Sink caps: %" GST_PTR_FORMAT, caps);

  GST_MMAL_DEINTERLACE_STREAM_LOCK (self);

  vinfo = &self->video_info;

  if (!gst_video_info_from_caps (vinfo, caps)) {
    ret = FALSE;
    goto invalid_caps;
  }

  fps_n = GST_VIDEO_INFO_FPS_N (vinfo);
  fps_d = GST_VIDEO_INFO_FPS_D (vinfo);

  gst_interlacing_mode = GST_VIDEO_INFO_INTERLACE_MODE (vinfo);

  src_caps = gst_caps_copy (caps);
  src_caps = gst_caps_make_writable (src_caps);

  gst_caps_set_simple (src_caps,
      /* obviously on the output it's progressive mode */
      "interlace-mode", G_TYPE_STRING, "progressive", NULL);

  if (fps_d != 0) {
    gst_caps_set_simple (src_caps,
        /* deinterlacer doubles the frame rate */
        "framerate", GST_TYPE_FRACTION, fps_n * 2, fps_d, NULL);
  }

  /* tell on the output only opaque mode is supported */
  gst_caps_set_features (src_caps, 0,
      gst_caps_features_new (GST_CAPS_FEATURE_MEMORY_MMAL_OPAQUE, NULL));

  GST_DEBUG_OBJECT (self, "Src caps: %" GST_PTR_FORMAT, src_caps);

  switch (gst_interlacing_mode) {
    case GST_VIDEO_INTERLACE_MODE_INTERLEAVED:
      self->interlace_type = MMAL_InterlaceFieldsInterleavedUpperFirst;
      /* TODO: distinguish between UpperFirst and LowerFirst ... */
      break;
    case GST_VIDEO_INTERLACE_MODE_MIXED:
      self->interlace_type = MMAL_InterlaceMixed;
      break;
    case GST_VIDEO_INTERLACE_MODE_PROGRESSIVE:
    default:
      self->interlace_type = MMAL_InterlaceProgressive;
      break;
  }

  self->frame_duration = fps_n != 0 ?
      G_GINT64_CONSTANT (1000000) * fps_d / fps_n : 0;

  /* Next time we see an input buffer, we need to configure port.
     We need to wait for this in order to see if we're opaque or not.
   */
  GST_DEBUG_OBJECT (self, "Format changed.  Need reconfigure on next buffer.");
  self->need_reconfigure = TRUE;

  /* N.B. yes, still checking need_reconfigure here, in case we insert more
     complicated decision-making above.

     I'm "deconfiguring" here, rather than waiting until later for 2 reasons:

     1) I want to be sure deinterlacer is drained of any buffers with "old"
     format, and these are sent downstream before notifying downstream of a
     format change.  And deconfigure() will call drain().

     2) As far as possible, I want to avoid locking the src pad mutex when the
     output task is running, as that increases the chances of deadlock.
   */
  if (self->need_reconfigure && !gst_mmal_deinterlace_deconfigure (self)) {

    GST_ERROR_OBJECT (self, "Deconfigure failed!");
    ret = FALSE;
  }

  /* Right, now output task should be stopped, (if it was even running in the
     first place), and we should have sent any pending "old" buffers downstream.
     I will now deal with telling downstream about the caps change.
   */

  GST_MMAL_DEINTERLACE_STREAM_UNLOCK (self);

  GST_PAD_STREAM_LOCK (self->src_pad);

  if (!gst_pad_set_caps (self->src_pad, src_caps)) {
    GST_ERROR_OBJECT (self, "Failed to set caps on src pad");
    ret = FALSE;
  }

  GST_PAD_STREAM_UNLOCK (self->src_pad);

  gst_caps_unref (src_caps);

  return ret;

invalid_caps:
  GST_MMAL_DEINTERLACE_STREAM_UNLOCK (self);

  GST_ERROR_OBJECT (pad, "Invalid caps");
  return FALSE;
}

/* Called on the input thread (EOS event, reconfiguration).  Stream lock must be
   held.
*/
static GstFlowReturn
gst_mmal_deinterlace_drain (GstMMALDeinterlace * self)
{
  MMAL_BUFFER_HEADER_T *buffer = NULL;
  GstFlowReturn flow_ret = GST_FLOW_OK;

  GstClockTime pts = GST_TIME_AS_USECONDS (self->last_upstream_ts);

  gint64 wait_until;

  if (!self->started) {
    GST_DEBUG_OBJECT (self, "Output thread not started yet. Nothing to drain.");
    return GST_FLOW_OK;
  }

  if (self->input_buffer_pool == NULL) {
    /* We probably haven't set initial format yet.  Don't worry. */
    return GST_FLOW_OK;

  } else {

    /* Hold this mutex before sending EOS buffer, otherwise we have a race. */
    g_mutex_lock (&self->drain_lock);
    self->draining = TRUE;

    /* Grab a buffer */
    GST_DEBUG_OBJECT (self, "Waiting for input buffer...");

    /* Yield the "big lock" before blocking on input buffer avail. */
    GST_MMAL_DEINTERLACE_STREAM_UNLOCK (self);

    if ((buffer = mmal_queue_timedwait (self->input_buffer_pool->queue,
                GST_MMAL_DEINTERLACE_INPUT_BUFFER_WAIT_FOR_MS)) == NULL) {

      GST_ERROR_OBJECT (self, "Failed to acquire input buffer!");
      flow_ret = GST_FLOW_ERROR;
      goto no_input_buffer;
    }

    GST_DEBUG_OBJECT (self, "Got input buffer");

    /* "Resets all variables to default values" */
    mmal_buffer_header_reset (buffer);
    buffer->cmd = 0;
    buffer->flags |= MMAL_BUFFER_HEADER_FLAG_EOS;
    buffer->pts = pts;

    GST_DEBUG_OBJECT (self, "Sending EOS with pts: %" G_GUINT64_FORMAT,
        (guint64) buffer->pts);

    if (mmal_port_send_buffer (self->image_fx->input[0], buffer) !=
        MMAL_SUCCESS) {

      GST_ERROR_OBJECT (self, "Failed to send input buffer to decoder!");

      mmal_buffer_header_release (buffer);

      flow_ret = GST_FLOW_ERROR;
      goto drain_failed;
    }

    /* OK, so now we've sent EOS to decoder.  Wait for an EOS buffer to plop
       out the other end.  This will be picked-up by the output thread, which
       we're hoping will then signal the condition variable!

       N.B. there may be pending output frames in front of the EOS, which will
       be sent downstream.
     */

    wait_until = g_get_monotonic_time () +
        GST_MMAL_DEINTERLACE_DRAIN_TIMEOUT_MS * G_TIME_SPAN_MILLISECOND;

    if (!g_cond_wait_until (&self->drain_cond, &self->drain_lock, wait_until)) {

      GST_ERROR_OBJECT (self, "Drain failed (timeout)!");

      flow_ret = GST_FLOW_ERROR;
      goto drain_failed;
    } else if (g_atomic_int_get (&self->flushing)) {

      GST_DEBUG_OBJECT (self, "Flushing: not drained.");
      flow_ret = GST_FLOW_FLUSHING;

    } else {

      GST_DEBUG_OBJECT (self, "Drained deinterlace.");
    }

  drain_failed:
    self->draining = FALSE;
    g_mutex_unlock (&self->drain_lock);

  no_input_buffer:
    GST_MMAL_DEINTERLACE_STREAM_LOCK (self);

    return flow_ret;
  }
}

/* Called on the input thread (sink pad) */
static gboolean
gst_mmal_deinterlace_sink_event (GstPad * pad, GstObject * parent,
    GstEvent * event)
{
  gboolean ret = TRUE;
  GstMMALDeinterlace *self = NULL;

  g_return_val_if_fail (parent != NULL, FALSE);
  g_return_val_if_fail (pad != NULL, FALSE);
  g_return_val_if_fail (event != NULL, FALSE);

  self = GST_MMAL_DEINTERLACE (parent);

  GST_LOG_OBJECT (pad, "Received %s event: %" GST_PTR_FORMAT,
      GST_EVENT_TYPE_NAME (event), event);

  switch (GST_EVENT_TYPE (event)) {
    case GST_EVENT_CAPS:
    {
      GstCaps *caps = NULL;

      gst_event_parse_caps (event, &caps);
      ret = gst_mmal_deinterlace_set_src_caps (self, pad, caps);
      gst_event_unref (event);
      break;
    }
    case GST_EVENT_SEGMENT_DONE:
    case GST_EVENT_EOS:
      GST_MMAL_DEINTERLACE_STREAM_LOCK (self);
      ret = gst_mmal_deinterlace_drain (self) == GST_FLOW_OK;
      GST_MMAL_DEINTERLACE_STREAM_UNLOCK (self);

      ret = ret && gst_pad_push_event (self->src_pad, event);
      break;

    case GST_EVENT_FLUSH_START:
      ret = gst_pad_push_event (self->src_pad, event);

      GST_MMAL_DEINTERLACE_STREAM_LOCK (self);
      ret = ret && gst_mmal_deinterlace_flush_start (self, FALSE);
      GST_MMAL_DEINTERLACE_STREAM_UNLOCK (self);

      break;

    case GST_EVENT_FLUSH_STOP:
      ret = gst_pad_push_event (self->src_pad, event);

      GST_MMAL_DEINTERLACE_STREAM_LOCK (self);
      ret = ret && gst_mmal_deinterlace_flush_stop (self, FALSE);
      GST_MMAL_DEINTERLACE_STREAM_UNLOCK (self);

      break;

    default:
      ret = gst_pad_event_default (pad, parent, event);
      break;
  }

  return ret;
}

/* Called on the input thread (sink pad) */
static gboolean
gst_mmal_deinterlace_sink_query (GstPad * pad, GstObject * parent,
    GstQuery * query)
{
  GstMMALDeinterlace *self = NULL;
  gboolean res = FALSE;

  g_return_val_if_fail (pad != NULL, GST_FLOW_ERROR);
  g_return_val_if_fail (parent != NULL, GST_FLOW_ERROR);
  g_return_val_if_fail (query != NULL, GST_FLOW_ERROR);

  GST_LOG_OBJECT (pad, "Query: %s", GST_QUERY_TYPE_NAME (query));

  self = GST_MMAL_DEINTERLACE (parent);

  switch (GST_QUERY_TYPE (query)) {
    case GST_QUERY_ALLOCATION:
      res = gst_pad_peer_query (self->src_pad, query);
      if (!res) {
        GST_ERROR_OBJECT (self, "gst_pad_peer_query() failed");
      }

      GST_PAD_STREAM_LOCK (self->src_pad);
      res = gst_mmal_deinterlace_alloc_output_gst_pool (self, query);
      GST_PAD_STREAM_UNLOCK (self->src_pad);

      if (!res) {
        GST_ERROR_OBJECT (self,
            "gst_mmal_deinterlace_alloc_output_gst_pool() failed");
      }
      break;
    default:
      res = gst_pad_query_default (pad, parent, query);
      break;
  }
  return res;
}

/* Called on the input thread */
static GstFlowReturn
gst_mmal_deinterlace_chain (GstPad * pad, GstObject * object, GstBuffer * buf)
{
  GstMMALDeinterlace *self = NULL;
  GstFlowReturn flow_ret = GST_FLOW_OK;
  MMAL_BUFFER_HEADER_T *mmal_buffer = NULL;
  MMAL_STATUS_T status = MMAL_SUCCESS;
  gboolean is_opaque = TRUE;

  g_return_val_if_fail (pad != NULL, GST_FLOW_ERROR);
  g_return_val_if_fail (object != NULL, GST_FLOW_ERROR);
  g_return_val_if_fail (buf != NULL, GST_FLOW_ERROR);

  self = GST_MMAL_DEINTERLACE (object);

  g_return_val_if_fail (self->image_fx != NULL, GST_FLOW_ERROR);

  GST_MMAL_DEINTERLACE_STREAM_LOCK (self);

  if (g_atomic_int_get (&self->flushing)) {
    GST_DEBUG_OBJECT (self, "Flushing: not processing input frame");
    gst_buffer_unref (buf);
    flow_ret = GST_FLOW_FLUSHING;
    goto done;
  }

  if (self->output_flow_ret != GST_FLOW_OK) {
    GST_ERROR_OBJECT (self, "Output task reported error, stop processing data");
    flow_ret = self->output_flow_ret;
    goto done;
  }

  if (GST_BUFFER_PTS_IS_VALID (buf)) {
    /* N.B. last_upstream_ts is used by drain() */
    self->last_upstream_ts = GST_BUFFER_PTS (buf);
  }

  /* Progressive (non-interlaced) frames shall be passed directly to output */
  if (self->interlace_type == MMAL_InterlaceProgressive) {

    GST_MMAL_DEINTERLACE_STREAM_UNLOCK (self);

    GST_DEBUG_OBJECT (self,
        "Progressive frame.  Pushing directly to output pad (PTS=%lld)",
        GST_BUFFER_PTS (buf));

    GST_TRACE_OBJECT (self,
        "Progressive frame: time: %" GST_TIME_FORMAT ", dur: %" GST_TIME_FORMAT
        ", end: %" GST_TIME_FORMAT, GST_TIME_ARGS (GST_BUFFER_TIMESTAMP (buf)),
        GST_TIME_ARGS (GST_BUFFER_DURATION (buf)),
        GST_TIME_ARGS (GST_BUFFER_TIMESTAMP (buf) + GST_BUFFER_DURATION (buf)));

    flow_ret = gst_pad_push (self->src_pad, buf);
    goto done_unlocked;
  }

  if (self->need_reconfigure && !gst_mmal_deinterlace_configure (self)) {
    GST_ERROR_OBJECT (self, "Reconfigure failed!");

    flow_ret = GST_FLOW_ERROR;
    goto done;
  }

  if (!self->started) {
    gst_mmal_deinterlace_start_output_task (self);
  }

  GST_MMAL_DEINTERLACE_STREAM_UNLOCK (self);

  /* Process input buffer */

  /* image_fx supports only MMAL Opaque buffers */
  is_opaque = gst_is_mmal_opaque_memory (gst_buffer_peek_memory (buf, 0));
  if (!is_opaque) {
    GST_ERROR_OBJECT (self,
        "Not a MMAL Opaque buffer (PTS=%lld)", GST_BUFFER_PTS (buf));

    flow_ret = GST_FLOW_ERROR;
    goto done_unlocked;
  }

  if (self->output_gstpool == NULL) {
    GST_ERROR_OBJECT (self,
        "Output pool is NULL (PTS=%lld)", GST_BUFFER_PTS (buf));
    flow_ret = GST_FLOW_ERROR;
    goto done_unlocked;
  }


  /* Using opaque buffers. Just extract MMAL buffer header from GstMemory */
  mmal_buffer =
      gst_mmal_opaque_mem_get_mmal_header (gst_buffer_peek_memory (buf, 0));
  if (mmal_buffer == NULL) {
    GST_ERROR_OBJECT (self, "Failed to get MMAL Buffer Header from buffer (%p)",
        buf);
    flow_ret = GST_FLOW_ERROR;
    goto done_unlocked;
  }

  mmal_buffer_header_acquire (mmal_buffer);
  gst_buffer_unref (buf);

  GST_DEBUG_OBJECT (self, "Sending MMAL input buffer: %p, PTS: %lld",
      mmal_buffer, mmal_buffer->pts);

  status = mmal_port_send_buffer (self->image_fx->input[0], mmal_buffer);
  if (status != MMAL_SUCCESS) {
    GST_ERROR_OBJECT (self, "Failed to send MMAL buffer: %s (%u)",
        mmal_status_to_string (status), status);
    flow_ret = GST_FLOW_ERROR;
    goto done_unlocked;
  }

  GST_DEBUG_OBJECT (self, "OK");

done_unlocked:
  return flow_ret;

done:
  GST_MMAL_DEINTERLACE_STREAM_UNLOCK (self);

  return flow_ret;
}

/* Called on the input thread.  Stream lock must be held. */
static void
gst_mmal_deinterlace_start_output_task (GstMMALDeinterlace * self)
{
  g_return_if_fail (self != NULL);

  if (!self->started) {
    GST_DEBUG_OBJECT (self, "Starting output task...");

    gst_pad_start_task (self->src_pad,
        (GstTaskFunction) gst_mmal_deinterlace_output_task_loop, self, NULL);

    self->started = TRUE;
  } else {
    GST_WARNING_OBJECT (self, "Output task is already running");
  }
}

/*
 * Output processing task
 */
static void
gst_mmal_deinterlace_output_task_loop (GstMMALDeinterlace * self)
{
  MMAL_BUFFER_HEADER_T *mmal_buffer = NULL;
  GstFlowReturn flow_ret = GST_FLOW_OK;
  uint32_t wait_for_buffer_timeout_ms = 250;
  gboolean draining = FALSE;
  gboolean is_eos = FALSE;

  g_return_if_fail (self != NULL);

  GST_TRACE_OBJECT (self, "Output task waken up");

  if (!gst_mmal_deinterlace_populate_output_port (self)) {
    GST_ERROR_OBJECT (self, "Failed to populate output port with buffers");

    gst_pad_push_event (self->src_pad, gst_event_new_eos ());
    gst_pad_pause_task (self->src_pad);

    GST_MMAL_DEINTERLACE_STREAM_LOCK (self);

    self->output_flow_ret = GST_FLOW_ERROR;
    self->started = FALSE;

    GST_MMAL_DEINTERLACE_STREAM_UNLOCK (self);

    return;
  }

  if ((mmal_buffer = mmal_queue_timedwait (self->output_queue,
              wait_for_buffer_timeout_ms)) == NULL) {

    GST_DEBUG_OBJECT (self, "Timed-out waiting for output frame.");

  } else {
    /* N.B. We never seem to see any events, so these TODO's aren't quite as bad
       as they look!
     */
    if (mmal_buffer->cmd) {

      GST_DEBUG_OBJECT (self, "Received command.");

      switch (mmal_buffer->cmd) {

        case MMAL_EVENT_EOS:
          GST_DEBUG_OBJECT (self, "Got EOS");
          flow_ret = GST_FLOW_EOS;
          break;
        case MMAL_EVENT_ERROR:
          /* TODO: Pull-out some useful info. */
          GST_WARNING_OBJECT (self, "TODO: Got Error");
          flow_ret = GST_FLOW_ERROR;
          break;
        case MMAL_EVENT_FORMAT_CHANGED:
          GST_DEBUG_OBJECT (self, "TODO: Output port format changed.");
          break;
        case MMAL_EVENT_PARAMETER_CHANGED:
          GST_DEBUG_OBJECT (self, "TODO: Parameter changed");
          break;
      }

    } else if (mmal_buffer->flags & MMAL_BUFFER_HEADER_FLAG_USER0) {

      /* This is just a wakeup sent by flush() */

      GST_DEBUG_OBJECT (self, "Got wakeup buffer.  Must be flushing...");

    } else {

      GST_DEBUG_OBJECT (self, "Handling output frame: %" G_GUINT64_FORMAT,
          (guint64) mmal_buffer->pts);

      if ((mmal_buffer->flags & MMAL_BUFFER_HEADER_FLAG_EOS)) {

        /* NOTE: EOS buffer might still have a final payload we need to process
           even though it comes from us sending (empty) EOS input buffer in
           drain().
         */
        GST_DEBUG_OBJECT (self, "Buffer signals EOS.");
        is_eos = TRUE;
      }

      if (mmal_buffer->length) {

        GstBuffer *gstbuf = NULL;

        flow_ret = gst_buffer_pool_acquire_buffer (self->output_gstpool,
            &gstbuf, NULL /* params */ );

        if (flow_ret != GST_FLOW_OK) {

          GST_ERROR_OBJECT (self, "Cannot get buffer from MMAL Opaque pool");
          goto done;

        } else {

          GstMemory *mem = gst_buffer_peek_memory (gstbuf, 0);

          if (mem == NULL || !gst_is_mmal_opaque_memory (mem)) {

            GST_ERROR_OBJECT (self, "Expected MMAL Opaque GstMemory");
            flow_ret = GST_FLOW_ERROR;
            goto done;
          } else {
            gst_mmal_opaque_mem_set_mmal_header (mem, mmal_buffer);
          }
        }


        if (gst_debug_category_get_threshold (GST_CAT_DEFAULT)
            >= GST_LEVEL_DEBUG) {

          char encoding[5];
          char encoding_variant[5];

          MMAL_ES_FORMAT_T *format = self->image_fx->output[0]->format;

          GST_DEBUG_OBJECT (self,
              "Sending buffer downstream: %p (MMAL: %p) PTS=%lld %dx%d "
              "(%dx%d) encoding=%s variant=%s",
              gstbuf, mmal_buffer, mmal_buffer->pts,
              format->es->video.width, format->es->video.height,
              format->es->video.crop.width, format->es->video.crop.height,
              mmal_4cc_to_string (encoding, sizeof (encoding),
                  format->encoding),
              mmal_4cc_to_string (encoding_variant, sizeof (encoding_variant),
                  format->encoding_variant));
        }

        /* set correct GstBuffer fields to keep gst clocking correct */
        GST_BUFFER_PTS (gstbuf) =
            gst_util_uint64_scale (mmal_buffer->pts, GST_SECOND,
            G_USEC_PER_SEC);

        GST_BUFFER_DTS (gstbuf) =
            gst_util_uint64_scale (mmal_buffer->dts, GST_SECOND,
            G_USEC_PER_SEC);

        /* push data to output pad */
        flow_ret = gst_pad_push (self->src_pad, gstbuf);
      }
      /* end-else (buffer has payload) */
    }
    /* end-else (not an event) */

  done:
    if (is_eos || g_atomic_int_get (&self->flushing)) {

      g_mutex_lock (&self->drain_lock);

      draining = self->draining;

      if (draining) {

        /* Input side will be waiting on condvar in drain() */

        GST_DEBUG_OBJECT (self, "Drained");
        self->draining = FALSE;
        g_cond_broadcast (&self->drain_cond);
        flow_ret = GST_FLOW_OK;

        /*
           Pause task so we don't re-enter this loop and try output port
           reconfiguration before input side is finished with format change, or
           whatever it is up to.
         */
        gst_pad_pause_task (self->src_pad);
      } else {

        GST_DEBUG_OBJECT (self, "EOS seen on output thread");
      }

      g_mutex_unlock (&self->drain_lock);
    }

    GST_MMAL_DEINTERLACE_STREAM_LOCK (self);

    self->output_flow_ret = flow_ret;
    self->started = !((is_eos && draining) || flow_ret != GST_FLOW_OK);

    if (flow_ret != GST_FLOW_OK) {
      gst_pad_push_event (self->src_pad, gst_event_new_eos ());

      gst_pad_pause_task (self->src_pad);
    }

    GST_MMAL_DEINTERLACE_STREAM_UNLOCK (self);
  }

  /* Buffer should always be freed. */
  if (mmal_buffer != NULL) {

    /* Release MMAL buffer back to the output pool.
       NOTE: We do this whether it's a "command" or an actual frame.
     */
    mmal_buffer_header_release (mmal_buffer);
  }
}


/*--- MMAL callbacks ---------------------------------------------------------*/

static void
gst_mmal_deinterlace_control_port_callback (MMAL_PORT_T * port,
    MMAL_BUFFER_HEADER_T * buffer)
{
  GstMMALDeinterlace *self = NULL;
  MMAL_STATUS_T status;

  g_return_if_fail (port != NULL);
  g_return_if_fail (port->userdata != NULL);
  g_return_if_fail (buffer != NULL);

  self = GST_MMAL_DEINTERLACE (port->userdata);
  GST_TRACE_OBJECT (self, "Control port callback");

  if (buffer->cmd == MMAL_EVENT_ERROR) {
    status = *(uint32_t *) buffer->data;
    GST_ERROR_OBJECT (self, "MMAL error: %s (%u)",
        mmal_status_to_string (status), status);
  } else {
    GST_DEBUG_OBJECT (self, "MMAL event on control port: %d", buffer->cmd);
  }

  mmal_buffer_header_release (buffer);
}

static void
gst_mmal_deinterlace_input_port_callback (MMAL_PORT_T * port,
    MMAL_BUFFER_HEADER_T * buffer)
{
  GstMMALDeinterlace *self = NULL;

  g_return_if_fail (port != NULL);
  g_return_if_fail (port->userdata != NULL);
  g_return_if_fail (buffer != NULL);

  self = GST_MMAL_DEINTERLACE (port->userdata);

  GST_TRACE_OBJECT (self, "Input port callback: %p", buffer);

  if (buffer->cmd != 0) {
    GST_DEBUG_OBJECT (self, "Input port cmd: %u", buffer->cmd);
  }

  /* opaque buffers doesn't contain any extra/user data,
   * so no additional processing here, just unref MMAL buffer */

  mmal_buffer_header_release (buffer);
}

static void
gst_mmal_deinterlace_output_port_callback (MMAL_PORT_T * port,
    MMAL_BUFFER_HEADER_T * buffer)
{
  MMAL_QUEUE_T *queue = NULL;

  g_return_if_fail (port != NULL);
  g_return_if_fail (port->userdata != NULL);
  g_return_if_fail (buffer != NULL);

  queue = (MMAL_QUEUE_T *) port->userdata;

  mmal_queue_put (queue, buffer);
}


static gboolean
gst_mmal_deinterlace_alloc_output_gst_pool (GstMMALDeinterlace * self,
    GstQuery * query)
{
  GstBufferPool *pool = NULL;
  GstCaps *caps = NULL;
  GstStructure *config = NULL;
  guint size = 0;
  guint min_buffers = 0;
  guint max_buffers = 0;
  GstAllocator *allocator = NULL;
  GstAllocationParams params;
  GstVideoInfo vinfo;
  gboolean update_allocator = TRUE;


  g_return_val_if_fail (self != NULL, FALSE);
  g_return_val_if_fail (query != NULL, FALSE);

  GST_DEBUG_OBJECT (self, "Creating new MMAL Opaque buffer pool...");

  gst_video_info_init (&vinfo);
  gst_query_parse_allocation (query, &caps, NULL);

  if (caps) {
    gst_video_info_from_caps (&vinfo, caps);

    /* check if we have MMAL Opaque buffer type in caps
     *
     * Tomasz: is there any other idea we could use GstVideoFormat to signal
     *         real MMAL opaque buffers somehow? I420 is also used for
     *         non-opaque YUV420 format... */
    if (vinfo.finfo->format == GST_VIDEO_FORMAT_I420) {
      gint i = 0;
      gint n = 0;
      gboolean found = FALSE;
      GstCapsFeatures *feature = gst_caps_get_features (caps, 0);

      /* Prefer an MMAL Opaque allocator if available and we want to use it */
      n = gst_query_get_n_allocation_params (query);

      for (i = 0; i < n; i++) {
        gst_query_parse_nth_allocation_param (query, i, &allocator, &params);

        if (allocator) {
          if (g_strcmp0 (allocator->mem_type, GST_MMAL_OPAQUE_MEMORY_TYPE) == 0) {
            found = TRUE;

            gst_query_set_nth_allocation_param (query, 0, allocator, &params);
            while (gst_query_get_n_allocation_params (query) > 1) {
              gst_query_remove_nth_allocation_param (query, 1);
            }
          }

          gst_object_unref (allocator);
          allocator = NULL;

          if (found) {
            /* We will use opaque buffers. */
            GST_DEBUG_OBJECT (self, "Found MMAL Opaque buffer mode in caps");
            break;
          }
        }
      }

      /* If try to negotiate with caps feature memory:MMALOpaque
         and if allocator is not of type memory MMALOpaque then fails.
         Should not have negotiated MMAL Opaque in allocation query caps
         otherwise.
       */
      if (feature
          && gst_caps_features_contains (feature,
              GST_CAPS_FEATURE_MEMORY_MMAL_OPAQUE) && !found) {

        GST_WARNING_OBJECT (self,
            "Caps indicate MMAL opaque buffers supported, "
            "but allocator not found!");
        return FALSE;
      }

      /* end (vinfo format == GST_VIDEO_FORMAT_I420) */
    }
    /* end (caps != NULL) */
  }

  if (gst_query_get_n_allocation_params (query) > 0) {
    gst_query_parse_nth_allocation_param (query, 0, &allocator, &params);
    update_allocator = TRUE;
  } else {
    allocator = NULL;
    gst_allocation_params_init (&params);
    update_allocator = FALSE;
  }

  size = vinfo.size;
  min_buffers = 0;
  max_buffers = 0;

  if (pool == NULL) {
    if (self->output_gstpool == NULL) {
      /* create new pool */
      pool = gst_mmal_opaque_buffer_pool_new ();
      g_assert (pool != NULL);
      self->output_gstpool = pool;      /* keep ref */
    } else {
      /* we have already pool allocated */
      pool = self->output_gstpool;
    }
  }

  config = gst_buffer_pool_get_config (pool);
  g_assert (config != NULL);

  /* configure our pool */
  gst_buffer_pool_config_set_params (config, caps, size, min_buffers,
      max_buffers);

  gst_buffer_pool_config_set_allocator (config, allocator, &params);

  gst_buffer_pool_set_config (pool, config);

  if (update_allocator) {
    gst_query_set_nth_allocation_param (query, 0, allocator, &params);
  } else {
    gst_query_add_allocation_param (query, allocator, &params);
  }

  gst_query_add_allocation_pool (query, pool, size, min_buffers, max_buffers);

  if (!gst_buffer_pool_set_active (pool, TRUE)) {
    GST_ERROR_OBJECT (self, "Failed to activate MMAL Opaque buffer pool");
    return FALSE;
  }

  GST_DEBUG_OBJECT (self, "MMAL Opaque buffer pool done.");

  return TRUE;
}

/*--- MMAL configuration -----------------------------------------------------*/

static gboolean
gst_mmal_deinterlace_configure_image_fx (GstMMALDeinterlace * self)
{
  MMAL_PORT_T *output_port = NULL;
  MMAL_PORT_T *input_port = NULL;

  g_return_val_if_fail (self != NULL, FALSE);
  g_return_val_if_fail (self->image_fx != NULL, FALSE);
  g_return_val_if_fail (self->image_fx->input[0] != NULL, FALSE);
  g_return_val_if_fail (self->image_fx->output[0] != NULL, FALSE);

  input_port = self->image_fx->input[0];
  g_return_val_if_fail (input_port != NULL, FALSE);

  output_port = self->image_fx->output[0];
  g_return_val_if_fail (output_port != NULL, FALSE);

  if ((self->deinterlace_mode < MMAL_PARAM_IMAGEFX_DEINTERLACE_DOUBLE) &&
      (self->deinterlace_mode > MMAL_PARAM_IMAGEFX_DEINTERLACE_FAST)) {
    GST_ERROR_OBJECT (self, "Wrong deinterlace mode: %d.",
        self->deinterlace_mode);
    return FALSE;
  }

  if (self->interlace_type >= MMAL_InterlaceMax) {
    GST_ERROR_OBJECT (self, "Wrong deinterlace type: %d.",
        self->interlace_type);
    return FALSE;
  }

  {
    MMAL_PARAMETER_IMAGEFX_PARAMETERS_T img_fx_param;

    memset (&img_fx_param, 0, sizeof (MMAL_PARAMETER_IMAGEFX_PARAMETERS_T));

    img_fx_param.hdr.id = MMAL_PARAMETER_IMAGE_EFFECT_PARAMETERS;
    img_fx_param.hdr.size = sizeof (MMAL_PARAMETER_IMAGEFX_PARAMETERS_T);
    img_fx_param.effect = self->deinterlace_mode;

    /* Most of these parameters are a bit of mystery as they are not documented.
     * The values and their meaning have been borrowed from VLC/XBMC.
     */
    img_fx_param.num_effect_params = 4;
    img_fx_param.effect_parameter[0] = 3;       /* ??? */
    img_fx_param.effect_parameter[1] = self->frame_duration;
    img_fx_param.effect_parameter[2] = 0;       /* half framerate ? */
    img_fx_param.effect_parameter[3] = MMAL_TRUE;       /* use QPU ? */

    if (mmal_port_parameter_set (output_port,
            &img_fx_param.hdr) != MMAL_SUCCESS) {
      GST_ERROR_OBJECT (self,
          "Failed to configure deinterlacer output port (mode)");
      return FALSE;
    }
  }

  GST_DEBUG_OBJECT (self,
      "MMAL deinterlace configuration: "
      "mode: %d (%s), type: %d (%s), RFF: %d, frame duration: %d",
      self->deinterlace_mode,
      get_mmal_deinterlace_mode_str (self->deinterlace_mode),
      self->interlace_type, get_mmal_interlace_type_str (self->interlace_type),
      self->want_repeat_first_field, self->frame_duration);

  return TRUE;
}

static gboolean
gst_mmal_deinterlace_open (GstMMALDeinterlace * self)
{
  MMAL_STATUS_T status = MMAL_SUCCESS;

  g_return_val_if_fail (self != NULL, FALSE);
  g_return_val_if_fail (self->image_fx == NULL, TRUE);  /* already opened */

  GST_DEBUG_OBJECT (self, "Opening deinterlacer");

  bcm_host_init ();

  self->started = FALSE;
  self->flushing = FALSE;

  /* Create the image_fx component on VideoCore */
  status =
      mmal_component_create (MMAL_COMPONENT_DEFAULT_IMAGE_FX, &self->image_fx);
  if (status != MMAL_SUCCESS) {
    GST_ERROR_OBJECT (self, "Failed to create MMAL image_fx component: %s (%u)",
        mmal_status_to_string (status), status);
    return FALSE;
  }

  if (self->image_fx->control != NULL) {
    self->image_fx->control->userdata = (struct MMAL_PORT_USERDATA_T *) self;

    status = mmal_port_enable (self->image_fx->control,
        gst_mmal_deinterlace_control_port_callback);
    if (status != MMAL_SUCCESS) {
      GST_ERROR_OBJECT (self, "Failed to enable control port %s: %s (%u)",
          self->image_fx->control->name, mmal_status_to_string (status),
          status);
      return FALSE;
    }
  }

  self->output_queue = mmal_queue_create ();

  if (self->output_queue == NULL) {
    GST_ERROR_OBJECT (self, "Failed to create output buffer queue!");
    return FALSE;
  }

  GST_DEBUG_OBJECT (self, "Opened deinterlacer");

  return TRUE;
}


static gboolean
gst_mmal_deinterlace_close (GstMMALDeinterlace * self)
{
  g_return_val_if_fail (self != NULL, FALSE);
  g_return_val_if_fail (self->image_fx != NULL, TRUE);  /* already closed */

  GST_DEBUG_OBJECT (self, "Closing deinterlacer");

  if (self->image_fx->control != NULL && self->image_fx->control->is_enabled) {
    mmal_port_disable (self->image_fx->control);
  }

  GST_DEBUG_OBJECT (self, "Freeing deinterlaced frames queue");

  if (self->output_queue != NULL) {
    mmal_queue_destroy (self->output_queue);
    self->output_queue = NULL;
  }

  GST_DEBUG_OBJECT (self, "Freeing output buffer pool");

  if (self->output_buffer_pool != NULL) {
    uint32_t output_buffers;

    output_buffers = mmal_queue_length (self->output_buffer_pool->queue);

    if (output_buffers != self->image_fx->output[0]->buffer_num) {
      GST_ERROR_OBJECT (self,
          "Failed to reclaim all output buffers (%u out of %u)",
          output_buffers, self->image_fx->output[0]->buffer_num);
    }

    mmal_port_pool_destroy (self->image_fx->output[0],
        self->output_buffer_pool);
    self->output_buffer_pool = NULL;
  }

  mmal_component_destroy (self->image_fx);

  bcm_host_deinit ();

  self->image_fx = NULL;

  GST_DEBUG_OBJECT (self, "Closed deinterlacer");

  return TRUE;
}

static gboolean
gst_mmal_deinterlace_start (GstMMALDeinterlace * self)
{
  GST_DEBUG_OBJECT (self, "Start");

  self->last_upstream_ts = 0;

  self->output_flow_ret = GST_FLOW_OK;
  /* N.B. This is actually set in handle_frame() */
  self->started = FALSE;

  self->flushing = FALSE;

  GST_MMAL_DEINTERLACE_STREAM_UNLOCK (self);

  g_mutex_lock (&self->drain_lock);
  self->draining = FALSE;
  g_mutex_unlock (&self->drain_lock);

  GST_PAD_STREAM_LOCK (self->src_pad);

  self->image_fx->output[0]->userdata = (void *) self->output_queue;

  GST_PAD_STREAM_UNLOCK (self->src_pad);

  GST_MMAL_DEINTERLACE_STREAM_LOCK (self);

  return TRUE;
}

static gboolean
gst_mmal_deinterlace_stop (GstMMALDeinterlace * self)
{
  GST_DEBUG_OBJECT (self, "Stopping deinterlacer");

  gst_mmal_deinterlace_flush_stop (self, TRUE);

  self->output_flow_ret = GST_FLOW_FLUSHING;

  gst_mmal_deinterlace_ports_disable (self);
  mmal_component_disable (self->image_fx);

  if (self->input_buffer_pool != NULL) {
    mmal_port_pool_destroy (self->image_fx->input[0], self->input_buffer_pool);
    self->input_buffer_pool = NULL;
  }

  GST_PAD_STREAM_LOCK (self->src_pad);

  if (self->output_gstpool != NULL) {
    gst_buffer_pool_set_active (self->output_gstpool, FALSE);
    gst_object_unref (self->output_gstpool);
    self->output_gstpool = NULL;
  }

  GST_PAD_STREAM_UNLOCK (self->src_pad);

  GST_DEBUG_OBJECT (self, "Stopped deinterlacer");

  return TRUE;
}

/**
 * We ask the deinterlace to release all buffers it is holding back to us.
 * This also stops the output task thread.  It will be re-started from the
 * next call to handle_frame().
 */
static gboolean
gst_mmal_deinterlace_flush_start (GstMMALDeinterlace * self, gboolean stop)
{
  MMAL_STATUS_T status;

  gboolean started = self->started;
  GstClockTime pts = GST_TIME_AS_USECONDS (self->last_upstream_ts);

  gboolean ret = FALSE;

  g_atomic_int_set (&self->flushing, TRUE);

  GST_DEBUG_OBJECT (self, "Flush start...");

  if (!self->image_fx) {
    GST_WARNING_OBJECT (self, "Deinterlace component not yet created!");
    return TRUE;
  }

  /* This should wake up input stream thread waiting for an input buffer */
  status = mmal_port_flush (self->image_fx->input[0]);
  if (status != MMAL_SUCCESS) {
    GST_ERROR_OBJECT (self, "Failed to flush input port: %d (%s)",
        status, mmal_status_to_string (status));
    return FALSE;
  }

  g_mutex_lock (&self->drain_lock);
  GST_DEBUG_OBJECT (self, "Flushing: signalling drain done");
  self->draining = FALSE;
  g_cond_broadcast (&self->drain_cond);
  g_mutex_unlock (&self->drain_lock);

  /* Yield the "big lock" before blocking on input buffer avail (see below).
     It's easier if we release it here, since we re-aquire at the end.
   */
  GST_MMAL_DEINTERLACE_STREAM_UNLOCK (self);

  if (started) {

    MMAL_BUFFER_HEADER_T *buffer = NULL;

    /* We stop the output task, but it can be blocked waiting on decoded frames
       queue.  We don't want to wait for that to time-out, so send a special
       buffer with a user flag set to tell it to wake-up.
     */

    GST_DEBUG_OBJECT (self, "Waiting for input buffer...");

    if ((buffer = mmal_queue_timedwait (self->input_buffer_pool->queue,
                GST_MMAL_DEINTERLACE_INPUT_BUFFER_WAIT_FOR_MS)) == NULL) {

      GST_ERROR_OBJECT (self, "Failed to acquire input buffer!");
      goto done;
    }

    GST_DEBUG_OBJECT (self, "Got input buffer");

    /* "Resets all variables to default values" */
    mmal_buffer_header_reset (buffer);
    buffer->cmd = 0;
    buffer->flags |= MMAL_BUFFER_HEADER_FLAG_USER0;
    buffer->pts = pts;

    GST_DEBUG_OBJECT (self,
        "Sending wakeup buffer with pts: %" G_GUINT64_FORMAT,
        (guint64) buffer->pts);

    if (mmal_port_send_buffer (self->image_fx->input[0], buffer) !=
        MMAL_SUCCESS) {

      GST_ERROR_OBJECT (self, "Failed to send input buffer to decoder!");
      mmal_buffer_header_release (buffer);
      goto done;
    }

    /* Stop the task (and wait for it to stop) */

    GST_DEBUG_OBJECT (self, "Stopping output task...");
    if (gst_pad_stop_task (self->src_pad)) {
      GST_DEBUG_OBJECT (self, "Output task stopped.");
      /* No stream lock needed as output task is not running any more */
      self->started = FALSE;
    } else {
      GST_ERROR_OBJECT (self, "Failed to stop output task!");
      goto done;
    }
  }
  /* end-if (already running) */

  ret = TRUE;

done:
  GST_MMAL_DEINTERLACE_STREAM_LOCK (self);

  return ret;
}

static gboolean
gst_mmal_deinterlace_flush_stop (GstMMALDeinterlace * self, gboolean stop)
{
  MMAL_PORT_T *input_port = NULL;
  MMAL_PORT_T *output_port = NULL;

  GST_DEBUG_OBJECT (self, "Flush stop...");

  if (!self->image_fx) {
    GST_ERROR_OBJECT (self, "Deinterlace component not yet created!");
    return TRUE;
  }

  input_port = self->image_fx->input[0];
  output_port = self->image_fx->output[0];

  /* OK, so now we don't need to worry about output side. We can fiddle with
     it's state as the output task was stopped in FLUSH_START.
   */
  if (stop && input_port->is_enabled &&
      mmal_port_disable (input_port) != MMAL_SUCCESS) {
    GST_ERROR_OBJECT (self, "Failed to disable input port!");
    goto flush_failed;
  }

  if (mmal_port_flush (input_port) != MMAL_SUCCESS) {
    GST_ERROR_OBJECT (self, "Failed to set flushing on input port!");
    goto flush_failed;
  }

  /* We take this lock to synchronise access to state belonging to output side */
  GST_PAD_STREAM_LOCK (self->src_pad);

  if (stop && output_port->is_enabled &&
      mmal_port_disable (output_port) != MMAL_SUCCESS) {
    GST_ERROR_OBJECT (self, "Failed to disable output port!");
    goto output_flush_failed;
  }

  if (mmal_port_flush (output_port) != MMAL_SUCCESS) {
    GST_ERROR_OBJECT (self, "Failed to set flushing on output port!");
    goto output_flush_failed;
  }

  if (self->output_queue != NULL) {

    MMAL_BUFFER_HEADER_T *buffer = NULL;

    /* Free any decoded frames */
    while ((buffer = mmal_queue_get (self->output_queue))) {

      GST_DEBUG_OBJECT (self, "Freeing decoded frame %p", buffer);
      mmal_buffer_header_release (buffer);
    }
  }

  if (stop) {
    /* At this point, we expect to have all our buffers back. */
    uint32_t input_buffers;

    if (self->input_buffer_pool != NULL) {

      input_buffers = mmal_queue_length (self->input_buffer_pool->queue);

      if (input_buffers != GST_MMAL_DEINTERLACE_NUM_INPUT_BUFFERS) {
        GST_ERROR_OBJECT (self,
            "Failed to reclaim all input buffers (%d out of %d)",
            input_buffers, input_port->buffer_num);
        goto output_flush_failed;
      }
    }
  }

  GST_PAD_STREAM_UNLOCK (self->src_pad);

  self->last_upstream_ts = 0;
  self->output_flow_ret = GST_FLOW_OK;

  g_atomic_int_set (&self->flushing, FALSE);

  return TRUE;

output_flush_failed:

  GST_PAD_STREAM_UNLOCK (self->src_pad);

flush_failed:

  return FALSE;
}

static gboolean
gst_mmal_deinterlace_flush_image_fx (GstMMALDeinterlace * self)
{
  g_return_val_if_fail (self != NULL, FALSE);
  g_return_val_if_fail (self->image_fx != NULL, FALSE);
  g_return_val_if_fail (self->image_fx->input[0] != NULL, FALSE);
  g_return_val_if_fail (self->image_fx->output[0] != NULL, FALSE);

  GST_DEBUG_OBJECT (self, "Deinterlacer flush start...");

  if (mmal_port_flush (self->image_fx->output[0]) != MMAL_SUCCESS) {
    GST_ERROR_OBJECT (self, "Failed to flush output port.");
  }

  if (mmal_port_flush (self->image_fx->input[0]) != MMAL_SUCCESS) {
    GST_ERROR_OBJECT (self, "Failed to flush input port.");
  }

  GST_DEBUG_OBJECT (self, "Deinterlacer flush end...");

  return TRUE;
}

static gboolean
gst_mmal_deinterlace_ports_enable (GstMMALDeinterlace * self)
{
  MMAL_PORT_T *input_port = NULL;
  MMAL_PORT_T *output_port = NULL;

  g_return_val_if_fail (self != NULL, FALSE);
  g_return_val_if_fail (self->image_fx != NULL, FALSE);
  g_return_val_if_fail (self->image_fx->input[0] != NULL, FALSE);
  g_return_val_if_fail (self->image_fx->output[0] != NULL, FALSE);

  GST_DEBUG_OBJECT (self, "Enable deinterlacer ports");

  input_port = self->image_fx->input[0];
  output_port = self->image_fx->output[0];

  if (!input_port->is_enabled && mmal_port_enable (input_port,
          &gst_mmal_deinterlace_input_port_callback) != MMAL_SUCCESS) {

    GST_ERROR_OBJECT (self, "Failed to enable input port!");
    return FALSE;
  }

  if (!output_port->is_enabled && mmal_port_enable (output_port,
          &gst_mmal_deinterlace_output_port_callback) != MMAL_SUCCESS) {

    GST_ERROR_OBJECT (self, "Failed to enable output port!");
    return FALSE;
  }

  return TRUE;
}

static gboolean
gst_mmal_deinterlace_ports_disable (GstMMALDeinterlace * self)
{
  MMAL_PORT_T *input_port = NULL;
  MMAL_PORT_T *output_port = NULL;

  g_return_val_if_fail (self != NULL, FALSE);
  g_return_val_if_fail (self->image_fx != NULL, FALSE);
  g_return_val_if_fail (self->image_fx->input[0] != NULL, FALSE);
  g_return_val_if_fail (self->image_fx->output[0] != NULL, FALSE);

  GST_DEBUG_OBJECT (self, "Disable deinterlacer ports");

  input_port = self->image_fx->input[0];
  output_port = self->image_fx->output[0];

  if (output_port->is_enabled && mmal_port_disable (output_port) !=
      MMAL_SUCCESS) {

    GST_ERROR_OBJECT (self, "Failed to disable output port!");
    return FALSE;
  }

  if (input_port->is_enabled && mmal_port_disable (input_port) != MMAL_SUCCESS) {

    GST_ERROR_OBJECT (self, "Failed to disable input port!");
    return FALSE;
  }

  return TRUE;
}

/**
 * This function should be called with stream mutex locked, and will acquire the
 * src pad mutex.
 */
static gboolean
gst_mmal_deinterlace_setup_image_fx (GstMMALDeinterlace * self)
{
  MMAL_PORT_T *input_port = NULL;
  MMAL_PORT_T *output_port = NULL;
  MMAL_ES_FORMAT_T *input_format = NULL;
  MMAL_ES_FORMAT_T *output_format = NULL;


  g_return_val_if_fail (self != NULL, FALSE);
  g_return_val_if_fail (self->image_fx != NULL, FALSE);
  g_return_val_if_fail (self->image_fx->input[0] != NULL, FALSE);
  g_return_val_if_fail (self->image_fx->output[0] != NULL, FALSE);
  g_return_val_if_fail (self->image_fx->input[0]->format != NULL, FALSE);
  g_return_val_if_fail (self->image_fx->output[0]->format != NULL, FALSE);

  GST_DEBUG_OBJECT (self, "Setup deinterlacer ports");

  input_port = self->image_fx->input[0];
  output_port = self->image_fx->output[0];

  input_format = input_port->format;
  output_format = output_port->format;

  if (!gst_mmal_deinterlace_ports_disable (self)) {
    GST_ERROR_OBJECT (self, "Failed to disable ports!");
    return FALSE;
  }

  gst_mmal_deinterlace_flush_image_fx (self);


  GST_DEBUG_OBJECT (self, "Setting up input port format");

  input_format->es->video.crop.x = 0;
  input_format->es->video.crop.y = 0;
  input_format->es->video.crop.width = self->video_info.width;
  input_format->es->video.crop.height = self->video_info.height;

  /* N.B. These are the alignments XBMC applies. */
  input_format->es->video.width = GST_ROUND_UP_32 (self->video_info.width);
  input_format->es->video.height = GST_ROUND_UP_16 (self->video_info.height);

  input_format->es->video.frame_rate.num = self->video_info.fps_n;
  input_format->es->video.frame_rate.den = self->video_info.fps_d;

  input_format->es->video.par.num = self->video_info.par_n;
  input_format->es->video.par.den = self->video_info.par_d;

  input_format->type = MMAL_ES_TYPE_VIDEO;
  input_format->flags = MMAL_ES_FORMAT_FLAG_FRAMED;
  input_format->extradata_size = 0;

  /* image_fx supports opaque buffers only */
  input_format->encoding = MMAL_ENCODING_OPAQUE;

  if (mmal_port_parameter_set_boolean (input_port, MMAL_PARAMETER_ZERO_COPY,
          MMAL_TRUE) != MMAL_SUCCESS) {

    GST_ERROR_OBJECT (self, "Failed to set/unset zero-copy on input port!");
    return FALSE;
  }

  if (mmal_port_format_commit (input_port) != MMAL_SUCCESS) {
    GST_ERROR_OBJECT (self, "Failed to commit new input port format!");
    return FALSE;
  }

  GST_DEBUG_OBJECT (self, "buffers recommended (in): %u",
      input_port->buffer_num_recommended);

  input_port->buffer_num = GST_MMAL_NUM_OUTPUT_BUFFERS;
  input_port->buffer_size = input_port->buffer_size_recommended;

  input_port->userdata = (void *) self;

  if (self->input_buffer_pool == NULL) {

    self->input_buffer_pool =
        mmal_port_pool_create (input_port,
        GST_MMAL_DEINTERLACE_NUM_INPUT_BUFFERS, input_port->buffer_size);
  }


  /* Src pad mutex protects output state. */
  GST_PAD_STREAM_LOCK (self->src_pad);

  if (self->output_queue != NULL) {
    MMAL_BUFFER_HEADER_T *buffer;

    while ((buffer = mmal_queue_get (self->output_queue))) {

      GST_DEBUG_OBJECT (self, "Freeing output buffer %p", buffer);
      mmal_buffer_header_release (buffer);
    }

    mmal_queue_destroy (self->output_queue);
  }

  /*
     NOTE: We can also receive notification of output port format changes via
     in-band events.  It seems we don't get those most of the time though, so
     I'm also dealing with the output port here.
   */

  mmal_format_full_copy (output_format, input_format);
  /* Deinterlacer doubles the frame rate on the output.  Not sure if frame rate
   * configuration is respected in the output port format or whether the
   * component works it out.  For consistency, we set it here to what we expect
   * it to be.
   */
  output_format->es->video.frame_rate.num = self->video_info.fps_n * 2;

  GST_DEBUG_OBJECT (self, "buffers recommended (out): %u",
      output_port->buffer_num_recommended);

  output_port->buffer_num = GST_MMAL_NUM_OUTPUT_BUFFERS;
  output_port->buffer_size = input_port->buffer_size;

  if (mmal_port_parameter_set_boolean (output_port, MMAL_PARAMETER_ZERO_COPY,
          MMAL_TRUE) != MMAL_SUCCESS) {

    GST_ERROR_OBJECT (self, "Failed to set/unset zero-copy on output port!");
    goto error_output_locked;
  }

  if (mmal_port_format_commit (output_port) != MMAL_SUCCESS) {
    GST_ERROR_OBJECT (self, "Failed to commit new output port format!");
    goto error_output_locked;
  }

  if (!self->output_buffer_pool) {
    GST_DEBUG_OBJECT (self, "Reconfiguring output buffer pool...");

    self->output_buffer_pool = mmal_port_pool_create (output_port,
        output_port->buffer_num, output_port->buffer_size);

    if (self->output_buffer_pool == NULL) {
      GST_ERROR_OBJECT (self, "Failed to create output buffer pool!");
      goto error_output_locked;
    }
  }

  self->output_queue = mmal_queue_create ();
  if (self->output_queue == NULL) {
    GST_ERROR_OBJECT (self, "Failed to create output buffer queue!");
    goto error_output_locked;
  }

  output_port->userdata = (void *) self->output_queue;

  if (!gst_mmal_deinterlace_ports_enable (self)) {
    GST_ERROR_OBJECT (self, "Failed to enable ports!");
    goto error_output_locked;
  }

  if (!self->image_fx->is_enabled && mmal_component_enable (self->image_fx) !=
      MMAL_SUCCESS) {

    GST_ERROR_OBJECT (self, "Failed to enable deinterlacer component!");
    goto error_output_locked;
  }

  GST_DEBUG_OBJECT (self, "Deinterlacer input & output configuration OK.");

  if (!gst_mmal_deinterlace_populate_output_port (self)) {
    goto error_output_locked;
  }

  GST_PAD_STREAM_UNLOCK (self->src_pad);

  return TRUE;

error_output_locked:

  GST_PAD_STREAM_UNLOCK (self->src_pad);
  return FALSE;

}


/**
 * This just feeds any available output buffers to the deinterlacer output port.
 * It's called after image_fx configuration and in the task loop function.
 *
 * Will be called with src pad mutex locked.
 */
static gboolean
gst_mmal_deinterlace_populate_output_port (GstMMALDeinterlace * self)
{
  MMAL_BUFFER_HEADER_T *buffer = NULL;

  g_return_val_if_fail (self != NULL, FALSE);

  GST_TRACE_OBJECT (self, "Populate output buffers");

  /* Send empty buffers to the output port to allow image_fx to start producing
   * frames as soon as it gets input data.
   */

  if (self->output_buffer_pool != NULL) {
    while ((buffer = mmal_queue_get (self->output_buffer_pool->queue)) != NULL) {

      GST_DEBUG_OBJECT (self, "Sending empty buffer to output port...");

      mmal_buffer_header_reset (buffer);

      if (mmal_port_send_buffer (self->image_fx->output[0],
              buffer) != MMAL_SUCCESS) {

        GST_ERROR_OBJECT (self,
            "Failed to send empty output buffer to output port!");
        return FALSE;
      }
    }
  }

  return TRUE;
}
