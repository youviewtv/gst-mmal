/*
 * Copyright (C) 2016, YouView TV Ltd
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

#include "gstmmalclock.h"

#include <interface/mmal/util/mmal_util.h>
#include <interface/mmal/util/mmal_util_params.h>

GST_DEBUG_CATEGORY_STATIC (gst_mmal_clock_debug_category);
#define GST_CAT_DEFAULT gst_mmal_clock_debug_category

#define GST_MMAL_CLOCK(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_MMAL_CLOCK, GstMMALClock))

#define GST_MMAL_CLOCK_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass), GST_TYPE_MMAL_CLOCK, GstMMALClockClass))

typedef struct _GstMMALClock GstMMALClock;
typedef struct _GstMMALClockClass GstMMALClockClass;

static gboolean gst_mmal_clock_master_cb (GstClock * master,
    GstClockTime time, GstClockID id, gpointer user_data);

struct _GstMMALClock
{
  GstSystemClock parent;

  MMAL_COMPONENT_T *mmal_clk;

  GstClockID master_clk_id;
  GstClockTime start_time;
};

struct _GstMMALClockClass
{
  /* we want to have async wait and other goodies implemented for us */
  GstSystemClockClass parent_class;
};

enum
{
  PROP_0,

  PROP_MMAL_CLOCK,

  PROP_LAST
};

#define gst_mmal_clock_parent_class parent_class

#define DEBUG_INIT \
    GST_DEBUG_CATEGORY_INIT (gst_mmal_clock_debug_category, \
      "mmalclock", 0, "debug category for gst-mmal clock");

G_DEFINE_TYPE_WITH_CODE (GstMMALClock, gst_mmal_clock,
    GST_TYPE_SYSTEM_CLOCK, DEBUG_INIT);

static void gst_mmal_clock_reset_clk (GstMMALClock * self, gpointer mmal_clk);

static void gst_mmal_clock_finalize (GObject * object);

static void gst_mmal_clock_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec);

static GstClockTime gst_mmal_clock_get_internal_time (GstClock * clock);

static void
gst_mmal_clock_class_init (GstMMALClockClass * klass)
{
  GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
  GstClockClass *clock_class = GST_CLOCK_CLASS (klass);

  gobject_class->finalize = GST_DEBUG_FUNCPTR (gst_mmal_clock_finalize);
  gobject_class->set_property = GST_DEBUG_FUNCPTR (gst_mmal_clock_set_property);

  g_object_class_install_property (gobject_class, PROP_MMAL_CLOCK,
      g_param_spec_pointer ("mmal-clock",
          "MMAL clock",
          "MMAL clock component used for getting time",
          G_PARAM_WRITABLE | G_PARAM_CONSTRUCT_ONLY | G_PARAM_STATIC_STRINGS));

  clock_class->get_internal_time =
      GST_DEBUG_FUNCPTR (gst_mmal_clock_get_internal_time);
}

static void
gst_mmal_clock_init (GstMMALClock * clock)
{
  GST_OBJECT_FLAG_SET (clock, GST_CLOCK_FLAG_CAN_SET_MASTER);

  clock->mmal_clk = NULL;
  clock->master_clk_id = NULL;
  clock->start_time = GST_CLOCK_TIME_NONE;
}

static void
gst_mmal_clock_finalize (GObject * object)
{
  GstMMALClock *self = GST_MMAL_CLOCK (object);

  if (self->master_clk_id) {
    gst_clock_id_unschedule (self->master_clk_id);
    gst_clock_id_unref (self->master_clk_id);
    self->master_clk_id = NULL;
  }

  gst_mmal_clock_reset_clk (self, NULL);

  G_OBJECT_CLASS (parent_class)->finalize (object);
}

static void
gst_mmal_clock_reset_clk (GstMMALClock * self, gpointer mmal_clk)
{
  if (self->mmal_clk) {
    mmal_component_release (self->mmal_clk);
    self->mmal_clk = NULL;
  }

  if (mmal_clk) {
    self->mmal_clk = (MMAL_COMPONENT_T *) mmal_clk;
    mmal_component_acquire (self->mmal_clk);
  }
}

static void
gst_mmal_clock_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec)
{
  GstMMALClock *self = GST_MMAL_CLOCK (object);

  switch (prop_id) {
    case PROP_MMAL_CLOCK:
      gst_mmal_clock_reset_clk (self, g_value_get_pointer (value));
      break;

    default:
      G_OBJECT_CLASS (parent_class)->set_property (object, prop_id,
          value, pspec);
      break;
  }
}

static GstClockTime
gst_mmal_clock_get_internal_time (GstClock * clock)
{
  GstMMALClock *self = GST_MMAL_CLOCK (clock);
  MMAL_PORT_T *clk_port = NULL;

  uint64_t mmal_time;
  MMAL_STATUS_T status;

  GstClockTime result;

  if (G_UNLIKELY (!self->mmal_clk)) {
    GST_WARNING_OBJECT (self, "MMAL clock not provided!");
    return GST_CLOCK_TIME_NONE;
  }

  clk_port = self->mmal_clk->clock[0];

  if (G_UNLIKELY (!clk_port->is_enabled)) {
    GST_WARNING_OBJECT (self, "MMAL clock not enabled!");
    return GST_CLOCK_TIME_NONE;
  }

  status =
      mmal_port_parameter_get_uint64 (clk_port, MMAL_PARAMETER_CLOCK_TIME,
      &mmal_time);

  if (G_UNLIKELY (status != MMAL_SUCCESS)) {
    GST_ERROR_OBJECT (self, "Failed to get MMAL time: %s (%u)",
        mmal_status_to_string (status), status);
    return GST_CLOCK_TIME_NONE;
  }

  result = gst_util_uint64_scale (mmal_time, GST_SECOND, G_USEC_PER_SEC);

  GST_TRACE_OBJECT (self,
      "start time: %" GST_TIME_FORMAT ", MMAL time: %" GST_TIME_FORMAT
      ", result: %" GST_TIME_FORMAT, GST_TIME_ARGS (self->start_time),
      GST_TIME_ARGS (result), GST_TIME_ARGS (result - self->start_time));

  if (G_UNLIKELY (!GST_CLOCK_TIME_IS_VALID (self->start_time))) {
    self->start_time = result;
  }

  return result - self->start_time;
}

GstClock *
gst_mmal_clock_new (const gchar * name, MMAL_COMPONENT_T * clk)
{
  return
      g_object_new (GST_TYPE_MMAL_CLOCK, "name", name,
      "clock-type", GST_CLOCK_TYPE_OTHER, "mmal-clock", clk, NULL);
}

static gboolean
gst_mmal_clock_master_cb (GstClock * master, GstClockTime time, GstClockID id,
    gpointer user_data)
{
  GstMMALClock *self = GST_MMAL_CLOCK (user_data);

  uint64_t mmal_time;
  MMAL_STATUS_T status;

  /* get the actual (current) time, not the callback schedule time */
  time = gst_clock_get_time (master);
  mmal_time = GST_CLOCK_TIME_IS_VALID (time) ?
      GST_TIME_AS_USECONDS (time) : MMAL_TIME_UNKNOWN;

  status = mmal_port_parameter_set_uint64 (self->mmal_clk->clock[0],
      MMAL_PARAMETER_CLOCK_TIME, mmal_time);

  if (G_UNLIKELY (status != MMAL_SUCCESS)) {
    GST_WARNING_OBJECT (self, "Failed to set MMAL time: %s (%u)",
        mmal_status_to_string (status), status);
  }

  GST_TRACE_OBJECT (self,
      "Master clock: %" GST_TIME_FORMAT ", MMAL clock: %" GST_TIME_FORMAT,
      GST_TIME_ARGS (time),
      GST_TIME_ARGS (gst_clock_get_time (GST_CLOCK (self))));

  return TRUE;
}

gboolean
gst_mmal_clock_set_master (GstClock * clock, GstClock * master)
{
  GstMMALClock *self;

  if (!GST_IS_MMAL_CLOCK (clock)) {
    GST_ERROR ("Not MMAL clock");
    return FALSE;
  }

  self = GST_MMAL_CLOCK (clock);

  if (self->master_clk_id) {
    gst_clock_id_unschedule (self->master_clk_id);
    gst_clock_id_unref (self->master_clk_id);
    self->master_clk_id = NULL;
  }

  /*
   * We are not worried if `clock == master` (same object).  In fact, this is
   * still required to have a periodic callback scheduled on 'master' clock even
   * if it's the same clock as this one.
   *
   * By scheduling MMAL media time update periodically, we tell MMAL what the
   * "delta" (or timeout) is in GStreamer land and MMAL compensates our idea of
   * time elapsed (delta) according to what it uses as a reference which is VCOS
   * wall clock.
   *
   * Separately, scheduler queues MMAL buffers according to their PTS using VCOS
   * wall clock (expiry) so it's independent of how often we tell it what our
   * idea of media time is. The master clock sampling period used (timeout)
   * indicates how frequently GStreamer compares its clock with MMAL and let
   * MMAL compensate the jitter.  Also MMAL advances media time based on media
   * buffers submitted: scheduler puts a time callback request in the queue and
   * when it's processed, various clock variables are evaluated in MMAL (wall
   * clock, last media time, current media time etc.), using delta calculations
   * (not absolute values).
   */
  if (master != NULL) {
    if (!gst_clock_is_synced (master)) {
      GST_ERROR_OBJECT (self, "Master clock is not synced");
      return FALSE;
    }

    self->master_clk_id = gst_clock_new_periodic_id (master,
        gst_clock_get_time (master), gst_clock_get_timeout (clock));

    if (gst_clock_id_wait_async (self->master_clk_id, gst_mmal_clock_master_cb,
            self, NULL) != GST_CLOCK_OK) {
      GST_ERROR_OBJECT (self, "Failed to add async callback on master clock");
      return FALSE;
    }
  }

  return TRUE;
}
