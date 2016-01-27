/*
Copyright (c) 2012, Broadcom Europe Ltd
All rights reserved.
Copyright (C) 2015, YouView TV Ltd.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <interface/mmal/mmal.h>
#include <interface/mmal/mmal_buffer.h>
#include <interface/mmal/core/mmal_buffer_private.h>


/**
 * These are overrides of functions provided by mmal library.
 *
 * They are overridden here because the original implementation does not take
 * care to handle the refcount in a thread-safe way, so we can (and do!) end-up
 * with the wrong refcount in threaded environment, and particularly in SMP case
 * as with RPi2.
 * 
 * This can easily lead to buffer headers being leaked and the pipeline
 * eventually grinding to a halt due to buffer starvation.
 *
 * This implementation uses atomic operations to manipulate the refcount.
 * We are able to override these functions because the dynamic-linker will pick
 * symbols defined locally in this library in preference to any that are defined
 * in a library it depends on.
 *
 * This is admittedly a somewhat dirty solution to the problem, but alternatives
 * such as wrapping these functions with others that take a big mutex lock are
 * also less than ideal. Plus these functions are anyway called by other mmal
 * functions.
 *
 * To be strictly correct, we should also take care of the functions:
 * mmal_pool_initialise_buffer_headers() and mmal_pool_buffer_header_release()
 * which also set the buffer header refcount.  However, in practice we have a
 * memory barrier between these operations and aquire()/release() manipulations
 * of the refcount, due to other mutex locking, and, due to the usage pattern,
 * the scope for racing with these functions is very small. So as we don't want
 * this dirty fix to become big-and-dirty, we do not re-implement those
 * functions here.
 *
 * Ideally, rpi-userland implementation would be patched upstream, so we can
 * delete this workaround.
 */

void
mmal_buffer_header_acquire (MMAL_BUFFER_HEADER_T * header)
{
  __sync_add_and_fetch (&header->priv->refcount, 1);
}


void
mmal_buffer_header_release (MMAL_BUFFER_HEADER_T * header)
{
  if (__sync_sub_and_fetch (&header->priv->refcount, 1) != 0) {
    return;
  }

  if (header->priv->pf_pre_release) {

    if (header->priv->pf_pre_release (header,
            header->priv->pre_release_userdata)) {
      /* delay releasing the buffer */
      return;
    }
  }

  mmal_buffer_header_release_continue (header);
}
