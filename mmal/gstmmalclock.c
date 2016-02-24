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

struct _GstMMALClock
{
  GstSystemClock parent;

  MMAL_COMPONENT_T *mmal_clk;

  GstClockTime start_time;
};

struct _GstMMALClockClass
{
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
  clock->start_time = GST_CLOCK_TIME_NONE;
}

static void
gst_mmal_clock_finalize (GObject * object)
{
  GstMMALClock *self = GST_MMAL_CLOCK (object);

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
