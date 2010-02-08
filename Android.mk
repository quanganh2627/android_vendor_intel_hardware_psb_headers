LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_COPY_HEADERS_TO := libttm/ttm

LOCAL_COPY_HEADERS :=			\
		ttm_fence_user.h	\
		ttm_placement_common.h	\
		ttm_placement_user.h

include $(BUILD_COPY_HEADERS)

include $(CLEAR_VARS)

LOCAL_COPY_HEADERS_TO := libpsb_drm

LOCAL_COPY_HEADERS :=			\
		psb_drm.h		\
		psb_reg.h

include $(BUILD_COPY_HEADERS)
