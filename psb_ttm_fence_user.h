/**************************************************************************
 *
 * Copyright 2006-2008 Tungsten Graphics, Inc., Cedar Park, TX., USA
 * All Rights Reserved.
 * Copyright (c) 2009 VMware, Inc., Palo Alto, CA., USA
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sub license, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS, AUTHORS AND/OR ITS SUPPLIERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
 * USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 **************************************************************************/
/*
 * Authors
 * Thomas Hellström <thomas-at-tungstengraphics-dot-com>
 */

#ifndef TTM_FENCE_USER_H
#define TTM_FENCE_USER_H

#if !defined(__KERNEL__) && !defined(_KERNEL)
#include <stdint.h>
#endif

#define TTM_FENCE_MAJOR 0
#define TTM_FENCE_MINOR 1
#define TTM_FENCE_PL    0
#define TTM_FENCE_DATE  "080819"

/**
 * struct ttm_fence_signaled_req
 *
 * @handle: Handle to the fence object. Input.
 *
 * @fence_type: Fence types we want to flush. Input.
 *
 * @flush: Boolean. Flush the indicated fence_types. Input.
 *
 * Argument to the TTM_FENCE_SIGNALED ioctl.
 */

struct ttm_fence_signaled_req {
	uint32_t handle;
	uint32_t fence_type;
	int32_t flush;
	uint32_t pad64;
};

/**
 * struct ttm_fence_rep
 *
 * @signaled_types: Fence type that has signaled.
 *
 * @fence_error: Command execution error.
 * Hardware errors that are consequences of the execution
 * of the command stream preceding the fence are reported
 * here.
 *
 * Output argument to the TTM_FENCE_SIGNALED and
 * TTM_FENCE_FINISH ioctls.
 */

struct ttm_fence_rep {
	uint32_t signaled_types;
	uint32_t fence_error;
};

union ttm_fence_signaled_arg {
	struct ttm_fence_signaled_req req;
	struct ttm_fence_rep rep;
};

/*
 * Waiting mode flags for the TTM_FENCE_FINISH ioctl.
 *
 * TTM_FENCE_FINISH_MODE_LAZY: Allow for sleeps during polling
 * wait.
 *
 * TTM_FENCE_FINISH_MODE_NO_BLOCK: Don't block waiting for GPU,
 * but return -EBUSY if the buffer is busy.
 */

#define TTM_FENCE_FINISH_MODE_LAZY     (1 << 0)
#define TTM_FENCE_FINISH_MODE_NO_BLOCK (1 << 1)

/**
 * struct ttm_fence_finish_req
 *
 * @handle: Handle to the fence object. Input.
 *
 * @fence_type: Fence types we want to finish.
 *
 * @mode: Wait mode.
 *
 * Input to the TTM_FENCE_FINISH ioctl.
 */

struct ttm_fence_finish_req {
	uint32_t handle;
	uint32_t fence_type;
	uint32_t mode;
	uint32_t pad64;
};

union ttm_fence_finish_arg {
	struct ttm_fence_finish_req req;
	struct ttm_fence_rep rep;
};

/**
 * struct ttm_fence_unref_arg
 *
 * @handle: Handle to the fence object.
 *
 * Argument to the TTM_FENCE_UNREF ioctl.
 */

struct ttm_fence_unref_arg {
	uint32_t handle;
	uint32_t pad64;
};

/*
 * Ioctl offsets frome extenstion start.
 */

#define TTM_FENCE_SIGNALED 0x01
#define TTM_FENCE_FINISH   0x02
#define TTM_FENCE_UNREF    0x03

#endif
