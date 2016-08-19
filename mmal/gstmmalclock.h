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

#ifndef __GST_MMAL_CLOCK_H__
#define __GST_MMAL_CLOCK_H__

#include <gst/gst.h>

#include <interface/mmal/mmal_component.h>

G_BEGIN_DECLS

#define GST_TYPE_MMAL_CLOCK \
  (gst_mmal_clock_get_type())

#define GST_IS_MMAL_CLOCK(obj) \
  (G_TYPE_CHECK_INSTANCE_TYPE((obj), GST_TYPE_MMAL_CLOCK))

#define GST_IS_MMAL_CLOCK_CLASS(obj) \
  (G_TYPE_CHECK_CLASS_TYPE((klass), GST_TYPE_MMAL_CLOCK))

GType gst_mmal_clock_get_type (void);

GstClock * gst_mmal_clock_new (const gchar * name, MMAL_COMPONENT_T * clk);

gboolean gst_mmal_clock_set_master (GstClock * clock, GstClock * master);

G_END_DECLS

#endif /* __GST_MMAL_CLOCK_H__ */
