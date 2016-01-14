/*
 * Copyright (C) 2015, YouView TV Ltd.
 *   Author: John Sadler <john.sadler@youview.com>
 *           Tomasz Szkutkowski <tomasz.szkutkowski@youview.com>
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

#include "gstmmalvideodec.h"

#include <gst/gst.h>

#ifndef __GST_MMAL_VP6_DEC_H__
#define __GST_MMAL_VP6_DEC_H__

G_BEGIN_DECLS
#define GST_TYPE_MMAL_VP6_DEC \
  (gst_mmal_vp6_dec_get_type())

#define GST_IS_MMAL_VP6_DEC(obj) \
  (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_MMAL_VP6_DEC))

#define GST_IS_MMAL_VP6_DEC_CLASS(obj) \
  (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_MMAL_VP6_DEC))

GType gst_mmal_vp6_dec_get_type (void);

G_END_DECLS
#endif /* __GST_MMAL_VP6_DEC_H__ */
