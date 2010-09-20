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
#ifndef _PSB_DRM_H_
#define _PSB_DRM_H_

#if defined(__linux__) && !defined(__KERNEL__)
#include<stdint.h>
#include "drm_mode.h"
#endif

#include "ttm/ttm_fence_user.h"
#include "ttm/ttm_placement_user.h"

/*
 * Menlow/MRST graphics driver package version
 * a.b.c.xxxx
 * a - Product Family: 5 - Linux
 * b - Major Release Version: 0 - non-Gallium (Unbuntu);
 *                            1 - Gallium (Moblin2)
 * c - Hotfix Release
 * xxxx - Graphics internal build #
 */
#define PSB_PACKAGE_VERSION "5.4.0.32L.0004"

#define DRM_PSB_SAREA_MAJOR 0
#define DRM_PSB_SAREA_MINOR 2
#define PSB_FIXED_SHIFT 16

#define DRM_PSB_FIRST_TA_USE_REG 3
#define DRM_PSB_NUM_TA_USE_REG 5
#define DRM_PSB_FIRST_RASTER_USE_REG 8
#define DRM_PSB_NUM_RASTER_USE_REG 7

#define PSB_NUM_PIPE 2

/*
 * Public memory types.
 */

#define DRM_PSB_MEM_MMU TTM_PL_PRIV1
#define DRM_PSB_FLAG_MEM_MMU TTM_PL_FLAG_PRIV1
#define DRM_PSB_MEM_PDS TTM_PL_PRIV2
#define DRM_PSB_FLAG_MEM_PDS TTM_PL_FLAG_PRIV2
#define DRM_PSB_MEM_APER TTM_PL_PRIV3
#define DRM_PSB_FLAG_MEM_APER TTM_PL_FLAG_PRIV3
#define DRM_PSB_MEM_RASTGEOM TTM_PL_PRIV4
#define DRM_PSB_FLAG_MEM_RASTGEOM TTM_PL_FLAG_PRIV4
#define PSB_MEM_RASTGEOM_START   0x30000000

typedef int32_t psb_fixed;
typedef uint32_t psb_ufixed;

static inline int32_t psb_int_to_fixed(int a)
{
	return a * (1 << PSB_FIXED_SHIFT);
}

static inline uint32_t psb_unsigned_to_ufixed(unsigned int a)
{
	return a << PSB_FIXED_SHIFT;
}

/*Status of the command sent to the gfx device.*/
typedef enum {
	DRM_CMD_SUCCESS,
	DRM_CMD_FAILED,
	DRM_CMD_HANG
} drm_cmd_status_t;

struct drm_psb_scanout {
	uint32_t buffer_id;	/* DRM buffer object ID */
	uint32_t rotation;	/* Rotation as in RR_rotation definitions */
	uint32_t stride;	/* Buffer stride in bytes */
	uint32_t depth;		/* Buffer depth in bits (NOT) bpp */
	uint32_t width;		/* Buffer width in pixels */
	uint32_t height;	/* Buffer height in lines */
	int32_t transform[3][3];	/* Buffer composite transform */
	/* (scaling, rot, reflect) */
};

#define DRM_PSB_SAREA_OWNERS 16
#define DRM_PSB_SAREA_OWNER_2D 0
#define DRM_PSB_SAREA_OWNER_3D 1

#define DRM_PSB_SAREA_SCANOUTS 3

struct drm_psb_sarea {
	/* Track changes of this data structure */

	uint32_t major;
	uint32_t minor;

	/* Last context to touch part of hw */
	uint32_t ctx_owners[DRM_PSB_SAREA_OWNERS];

	/* Definition of front- and rotated buffers */
	uint32_t num_scanouts;
	struct drm_psb_scanout scanouts[DRM_PSB_SAREA_SCANOUTS];

	int planeA_x;
	int planeA_y;
	int planeA_w;
	int planeA_h;
	int planeB_x;
	int planeB_y;
	int planeB_w;
	int planeB_h;
	/* Number of active scanouts */
	uint32_t num_active_scanouts;
};

#define PSB_RELOC_MAGIC         0x67676767
#define PSB_RELOC_SHIFT_MASK    0x0000FFFF
#define PSB_RELOC_SHIFT_SHIFT   0
#define PSB_RELOC_ALSHIFT_MASK  0xFFFF0000
#define PSB_RELOC_ALSHIFT_SHIFT 16

#define PSB_RELOC_OP_OFFSET     0	/* Offset of the indicated
					 * buffer
					 */
#define PSB_RELOC_OP_2D_OFFSET  1	/* Offset of the indicated
					 *  buffer, relative to 2D
					 *  base address
					 */
#define PSB_RELOC_OP_PDS_OFFSET 2	/* Offset of the indicated buffer,
					 *  relative to PDS base address
					 */
#define PSB_RELOC_OP_STRIDE     3	/* Stride of the indicated
					 * buffer (for tiling)
					 */
#define PSB_RELOC_OP_USE_OFFSET 4	/* Offset of USE buffer
					 * relative to base reg
					 */
#define PSB_RELOC_OP_USE_REG    5	/* Base reg of USE buffer */

struct drm_psb_reloc {
	uint32_t reloc_op;
	uint32_t where;		/* offset in destination buffer */
	uint32_t buffer;	/* Buffer reloc applies to */
	uint32_t mask;		/* Destination format: */
	uint32_t shift;		/* Destination format: */
	uint32_t pre_add;	/* Destination format: */
	uint32_t background;	/* Destination add */
	uint32_t dst_buffer;	/* Destination buffer. Index into buffer_list */
	uint32_t arg0;		/* Reloc-op dependant */
	uint32_t arg1;
};


#define PSB_GPU_ACCESS_READ         (1ULL << 32)
#define PSB_GPU_ACCESS_WRITE        (1ULL << 33)
#define PSB_GPU_ACCESS_MASK         (PSB_GPU_ACCESS_READ | PSB_GPU_ACCESS_WRITE)

#define PSB_BO_FLAG_TA              (1ULL << 48)
#define PSB_BO_FLAG_SCENE           (1ULL << 49)
#define PSB_BO_FLAG_FEEDBACK        (1ULL << 50)
#define PSB_BO_FLAG_USSE            (1ULL << 51)
#define PSB_BO_FLAG_COMMAND         (1ULL << 52)

#define PSB_ENGINE_2D 0
#define PSB_ENGINE_VIDEO 1
#define PSB_ENGINE_RASTERIZER 2
#define PSB_ENGINE_TA 3
#define PSB_ENGINE_HPRAST 4
#define LNC_ENGINE_ENCODE 5

/*
 * For this fence class we have a couple of
 * fence types.
 */

#define _PSB_FENCE_EXE_SHIFT           0
#define _PSB_FENCE_TA_DONE_SHIFT       1
#define _PSB_FENCE_RASTER_DONE_SHIFT   2
#define _PSB_FENCE_SCENE_DONE_SHIFT    3
#define _PSB_FENCE_FEEDBACK_SHIFT      4

#define _PSB_ENGINE_TA_FENCE_TYPES   5
#define _PSB_FENCE_TYPE_EXE         (1 << _PSB_FENCE_EXE_SHIFT)
#define _PSB_FENCE_TYPE_TA_DONE     (1 << _PSB_FENCE_TA_DONE_SHIFT)
#define _PSB_FENCE_TYPE_RASTER_DONE (1 << _PSB_FENCE_RASTER_DONE_SHIFT)
#define _PSB_FENCE_TYPE_SCENE_DONE  (1 << _PSB_FENCE_SCENE_DONE_SHIFT)
#define _PSB_FENCE_TYPE_FEEDBACK    (1 << _PSB_FENCE_FEEDBACK_SHIFT)

#define PSB_ENGINE_HPRAST 4
#define PSB_NUM_ENGINES 6

#define PSB_TA_FLAG_FIRSTPASS    (1 << 0)
#define PSB_TA_FLAG_LASTPASS     (1 << 1)

#define PSB_FEEDBACK_OP_VISTEST (1 << 0)

struct drm_psb_extension_rep {
	int32_t exists;
	uint32_t driver_ioctl_offset;
	uint32_t sarea_offset;
	uint32_t major;
	uint32_t minor;
	uint32_t pl;
};

#define DRM_PSB_EXT_NAME_LEN 128

union drm_psb_extension_arg {
	char extension[DRM_PSB_EXT_NAME_LEN];
	struct drm_psb_extension_rep rep;
};

struct psb_validate_req {
	uint64_t set_flags;
	uint64_t clear_flags;
	uint64_t next;
	uint64_t presumed_gpu_offset;
	uint32_t buffer_handle;
	uint32_t presumed_flags;
	uint32_t group;
	uint32_t pad64;
};

struct psb_validate_rep {
	uint64_t gpu_offset;
	uint32_t placement;
	uint32_t fence_type_mask;
};

#define PSB_USE_PRESUMED     (1 << 0)

struct psb_validate_arg {
	int handled;
	int ret;
	union {
		struct psb_validate_req req;
		struct psb_validate_rep rep;
	} d;
};

struct drm_psb_scene {
	int handle_valid;
	uint32_t handle;
	uint32_t w;		/* also contains msaa info */
	uint32_t h;
	uint32_t num_buffers;
};

#define DRM_PSB_FENCE_NO_USER        (1 << 0)

struct psb_ttm_fence_rep {
	uint32_t handle;
	uint32_t fence_class;
	uint32_t fence_type;
	uint32_t signaled_types;
	uint32_t error;
};

typedef struct drm_psb_cmdbuf_arg {
	uint64_t buffer_list;	/* List of buffers to validate */
	uint64_t clip_rects;	/* See i915 counterpart */
	uint64_t scene_arg;
	uint64_t fence_arg;

	uint32_t ta_flags;

	uint32_t ta_handle;	/* TA reg-value pairs */
	uint32_t ta_offset;
	uint32_t ta_size;

	uint32_t oom_handle;
	uint32_t oom_offset;
	uint32_t oom_size;

	uint32_t cmdbuf_handle;	/* 2D Command buffer object or, */
	uint32_t cmdbuf_offset;	/* rasterizer reg-value pairs */
	uint32_t cmdbuf_size;

	uint32_t reloc_handle;	/* Reloc buffer object */
	uint32_t reloc_offset;
	uint32_t num_relocs;

	int32_t damage;		/* Damage front buffer with cliprects */
	/* Not implemented yet */
	uint32_t fence_flags;
	uint32_t engine;

	/*
	 * Feedback;
	 */

	uint32_t feedback_ops;
	uint32_t feedback_handle;
	uint32_t feedback_offset;
	uint32_t feedback_breakpoints;
	uint32_t feedback_size;
} drm_psb_cmdbuf_arg_t;

typedef struct drm_psb_pageflip_arg {
	uint32_t flip_offset;
	uint32_t stride;
} drm_psb_pageflip_arg_t;

typedef enum {
	LNC_VIDEO_DEVICE_INFO,
	LNC_VIDEO_GETPARAM_RAR_INFO,
	LNC_VIDEO_GETPARAM_CI_INFO,
	LNC_VIDEO_GETPARAM_RAR_HANDLER_OFFSET,
	LNC_VIDEO_FRAME_SKIP
} lnc_getparam_key_t;

struct drm_lnc_video_getparam_arg {
	lnc_getparam_key_t key;
	uint64_t arg;	/* argument pointer */
	uint64_t value;	/* feed back pointer */
};

struct drm_psb_xhw_init_arg {
	uint32_t operation;
	uint32_t buffer_handle;
};

/*
 * Feedback components:
 */

/*
 * Vistest component. The number of these in the feedback buffer
 * equals the number of vistest breakpoints + 1.
 * This is currently the only feedback component.
 */

struct drm_psb_vistest {
	uint32_t vt[8];
};

struct drm_psb_sizes_arg {
	uint32_t ta_mem_size;
	uint32_t mmu_size;
	uint32_t pds_size;
	uint32_t rastgeom_size;
	uint32_t tt_size;
	uint32_t vram_size;
};

struct drm_psb_hist_status_arg {
	uint32_t buf[32];
};

struct drm_psb_dpst_lut_arg {
	uint8_t lut[256];
	int output_id;
};

struct mrst_timing_info {
	uint16_t pixel_clock;
	uint8_t hactive_lo;
	uint8_t hblank_lo;
	uint8_t hblank_hi:4;
	uint8_t hactive_hi:4;
	uint8_t vactive_lo;
	uint8_t vblank_lo;
	uint8_t vblank_hi:4;
	uint8_t vactive_hi:4;
	uint8_t hsync_offset_lo;
	uint8_t hsync_pulse_width_lo;
	uint8_t vsync_pulse_width_lo:4;
	uint8_t vsync_offset_lo:4;
	uint8_t vsync_pulse_width_hi:2;
	uint8_t vsync_offset_hi:2;
	uint8_t hsync_pulse_width_hi:2;
	uint8_t hsync_offset_hi:2;
	uint8_t width_mm_lo;
	uint8_t height_mm_lo;
	uint8_t height_mm_hi:4;
	uint8_t width_mm_hi:4;
	uint8_t hborder;
	uint8_t vborder;
	uint8_t unknown0:1;
	uint8_t hsync_positive:1;
	uint8_t vsync_positive:1;
	uint8_t separate_sync:2;
	uint8_t stereo:1;
	uint8_t unknown6:1;
	uint8_t interlaced:1;
} __attribute__((packed));

struct mrst_panel_descriptor_v1{
	uint32_t Panel_Port_Control; /* 1 dword, Register 0x61180 if LVDS */
				/* 0x61190 if MIPI */
	uint32_t Panel_Power_On_Sequencing;/*1 dword,Register 0x61208,*/
	uint32_t Panel_Power_Off_Sequencing;/*1 dword,Register 0x6120C,*/
	uint32_t Panel_Power_Cycle_Delay_and_Reference_Divisor;/* 1 dword */
						/* Register 0x61210 */
	struct mrst_timing_info DTD;/*18 bytes, Standard definition */
	uint16_t Panel_Backlight_Inverter_Descriptor;/* 16 bits, as follows */
				/* Bit 0, Frequency, 15 bits,0 - 32767Hz */
			/* Bit 15, Polarity, 1 bit, 0: Normal, 1: Inverted */
	uint16_t Panel_MIPI_Display_Descriptor;
			/*16 bits, Defined as follows: */
			/* if MIPI, 0x0000 if LVDS */
			/* Bit 0, Type, 2 bits, */
			/* 0: Type-1, */
			/* 1: Type-2, */
			/* 2: Type-3, */
			/* 3: Type-4 */
			/* Bit 2, Pixel Format, 4 bits */
			/* Bit0: 16bpp (not supported in LNC), */
			/* Bit1: 18bpp loosely packed, */
			/* Bit2: 18bpp packed, */
			/* Bit3: 24bpp */
			/* Bit 6, Reserved, 2 bits, 00b */
		/* Bit 8, Minimum Supported Frame Rate, 6 bits, 0 - 63Hz */
			/* Bit 14, Reserved, 2 bits, 00b */
} __attribute__ ((packed));

struct mrst_panel_descriptor_v2{
	uint32_t Panel_Port_Control; /* 1 dword, Register 0x61180 if LVDS */
				/* 0x61190 if MIPI */
	uint32_t Panel_Power_On_Sequencing;/*1 dword,Register 0x61208,*/
	uint32_t Panel_Power_Off_Sequencing;/*1 dword,Register 0x6120C,*/
	uint8_t Panel_Power_Cycle_Delay_and_Reference_Divisor;/* 1 byte */
						/* Register 0x61210 */
	struct mrst_timing_info DTD;/*18 bytes, Standard definition */
	uint16_t Panel_Backlight_Inverter_Descriptor;/*16 bits, as follows*/
				/*Bit 0, Frequency, 16 bits, 0 - 32767Hz*/
	uint8_t Panel_Initial_Brightness;/* [7:0] 0 - 100% */
			/*Bit 7, Polarity, 1 bit,0: Normal, 1: Inverted*/
	uint16_t Panel_MIPI_Display_Descriptor;
			/*16 bits, Defined as follows: */
			/* if MIPI, 0x0000 if LVDS */
			/* Bit 0, Type, 2 bits, */
			/* 0: Type-1, */
			/* 1: Type-2, */
			/* 2: Type-3, */
			/* 3: Type-4 */
			/* Bit 2, Pixel Format, 4 bits */
			/* Bit0: 16bpp (not supported in LNC), */
			/* Bit1: 18bpp loosely packed, */
			/* Bit2: 18bpp packed, */
			/* Bit3: 24bpp */
			/* Bit 6, Reserved, 2 bits, 00b */
		/* Bit 8, Minimum Supported Frame Rate, 6 bits, 0 - 63Hz */
			/* Bit 14, Reserved, 2 bits, 00b */
} __attribute__ ((packed));

union mrst_panel_rx{
	struct{
		uint16_t NumberOfLanes:2; /*Num of Lanes, 2 bits,0 = 1 lane,*/
			/* 1 = 2 lanes, 2 = 3 lanes, 3 = 4 lanes. */
		uint16_t MaxLaneFreq:3; /* 0: 100MHz, 1: 200MHz, 2: 300MHz, */
		/*3: 400MHz, 4: 500MHz, 5: 600MHz, 6: 700MHz, 7: 800MHz.*/
		uint16_t SupportedVideoTransferMode:2; /*0: Non-burst only */
					/* 1: Burst and non-burst */
					/* 2/3: Reserved */
		uint16_t HSClkBehavior:1; /*0: Continuous, 1: Non-continuous*/
		uint16_t DuoDisplaySupport:1; /*1 bit,0: No, 1: Yes*/
		uint16_t ECC_ChecksumCapabilities:1;/*1 bit,0: No, 1: Yes*/
		uint16_t BidirectionalCommunication:1;/*1 bit,0: No, 1: Yes */
		uint16_t Rsvd:5;/*5 bits,00000b */
	} panelrx;
	uint16_t panel_receiver;
} __attribute__ ((packed));

struct gct_ioctl_arg{
	uint8_t bpi; /* boot panel index, number of panel used during boot */
	uint8_t pt; /* panel type, 4 bit field, 0=lvds, 1=mipi */
	struct mrst_timing_info DTD; /* timing info for the selected panel */
	uint32_t Panel_Port_Control;
	uint32_t PP_On_Sequencing;/*1 dword,Register 0x61208,*/
	uint32_t PP_Off_Sequencing;/*1 dword,Register 0x6120C,*/
	uint32_t PP_Cycle_Delay;
	uint16_t Panel_Backlight_Inverter_Descriptor;
	uint16_t Panel_MIPI_Display_Descriptor;
} __attribute__ ((packed));

struct mrst_vbt{
	char Signature[4]; /*4 bytes,"$GCT" */
	uint8_t Revision; /*1 byte */
	uint8_t Size; /*1 byte */
	uint8_t Checksum; /*1 byte,Calculated*/
	void *mrst_gct;
} __attribute__ ((packed));

struct mrst_gct_v1{ /* expect this table to change per customer request*/
	union{ /*8 bits,Defined as follows: */
		struct{
			uint8_t PanelType:4; /*4 bits, Bit field for panels*/
					/* 0 - 3: 0 = LVDS, 1 = MIPI*/
					/*2 bits,Specifies which of the*/
			uint8_t BootPanelIndex:2;
					/* 4 panels to use by default*/
			uint8_t BootMIPI_DSI_RxIndex:2;/*Specifies which of*/
					/* the 4 MIPI DSI receivers to use*/
			} PD;
		uint8_t PanelDescriptor;
	};
	struct mrst_panel_descriptor_v1 panel[4];/*panel descrs,38 bytes each*/
	union mrst_panel_rx panelrx[4]; /* panel receivers*/
} __attribute__ ((packed));

struct mrst_gct_v2{ /* expect this table to change per customer request*/
	union{ /*8 bits,Defined as follows: */
		struct{
			uint8_t PanelType:4; /*4 bits, Bit field for panels*/
					/* 0 - 3: 0 = LVDS, 1 = MIPI*/
					/*2 bits,Specifies which of the*/
			uint8_t BootPanelIndex:2;
					/* 4 panels to use by default*/
			uint8_t BootMIPI_DSI_RxIndex:2;/*Specifies which of*/
					/* the 4 MIPI DSI receivers to use*/
			} PD;
		uint8_t PanelDescriptor;
	};
	struct mrst_panel_descriptor_v2 panel[4];/*panel descrs,38 bytes each*/
	union mrst_panel_rx panelrx[4]; /* panel receivers*/
} __attribute__ ((packed));

#define PSB_DC_CRTC_SAVE 0x01
#define PSB_DC_CRTC_RESTORE 0x02
#define PSB_DC_OUTPUT_SAVE 0x04
#define PSB_DC_OUTPUT_RESTORE 0x08
#define PSB_DC_CRTC_MASK 0x03
#define PSB_DC_OUTPUT_MASK 0x0C

struct drm_psb_dc_state_arg {
	uint32_t flags;
	uint32_t obj_id;
};

struct drm_psb_mode_operation_arg {
	uint32_t obj_id;
	uint16_t operation;
	struct drm_mode_modeinfo mode;
	void *data;
};

struct drm_psb_stolen_memory_arg {
	uint32_t base;
	uint32_t size;
};

/*Display Register Bits*/
#define REGRWBITS_PFIT_CONTROLS			(1 << 0)
#define REGRWBITS_PFIT_AUTOSCALE_RATIOS		(1 << 1)
#define REGRWBITS_PFIT_PROGRAMMED_SCALE_RATIOS	(1 << 2)
#define REGRWBITS_PIPEASRC			(1 << 3)
#define REGRWBITS_PIPEBSRC			(1 << 4)
#define REGRWBITS_VTOTAL_A			(1 << 5)
#define REGRWBITS_VTOTAL_B			(1 << 6)
#ifdef MDFLD_HDCP
#define REGRWBITS_HDCP   			(1 << 7)
#endif
#define REGRWBITS_DSPACNTR	(1 << 8)
#define REGRWBITS_DSPBCNTR	(1 << 9)
#define REGRWBITS_DSPCCNTR	(1 << 10)

/*Overlay Register Bits*/
#define OV_REGRWBITS_OVADD			(1 << 0)
#define OV_REGRWBITS_OGAM_ALL			(1 << 1)

#define OVC_REGRWBITS_OVADD                  (1 << 2)
#define OVC_REGRWBITS_OGAM_ALL			(1 << 3)

struct drm_psb_register_rw_arg {
	uint32_t b_force_hw_on;

	uint32_t display_read_mask;
	uint32_t display_write_mask;

	struct {
		uint32_t pfit_controls;
		uint32_t pfit_autoscale_ratios;
		uint32_t pfit_programmed_scale_ratios;
		uint32_t pipeasrc;
		uint32_t pipebsrc;
		uint32_t vtotal_a;
		uint32_t vtotal_b;
#ifdef MDFLD_HDCP
		uint32_t hdcp_reg;
		uint32_t hdcp_value;
#endif
	} display;

	uint32_t overlay_read_mask;
	uint32_t overlay_write_mask;

	struct {
		uint32_t OVADD;
		uint32_t OGAMC0;
		uint32_t OGAMC1;
		uint32_t OGAMC2;
		uint32_t OGAMC3;
		uint32_t OGAMC4;
		uint32_t OGAMC5;
		uint32_t IEP_ENABLED;
		uint32_t IEP_BLE_MINMAX;
		uint32_t IEP_BSSCC_CONTROL;
	} overlay;

	uint32_t sprite_enable_mask;
	uint32_t sprite_disable_mask;

	struct {
		uint32_t dspa_control;
		uint32_t dspa_key_value;
		uint32_t dspa_key_mask;
		uint32_t dspc_control;
		uint32_t dspc_stride;
		uint32_t dspc_position;
		uint32_t dspc_linear_offset;
		uint32_t dspc_size;
		uint32_t dspc_surface;
	} sprite;

	uint32_t subpicture_enable_mask;
	uint32_t subpicture_disable_mask;
};

struct psb_gtt_mapping_arg {
	void *hKernelMemInfo;
	uint32_t offset_pages;
};

struct drm_psb_getpageaddrs_arg {
	uint32_t handle;
	unsigned long *page_addrs;
	unsigned long gtt_offset;
};

#define PSB_HW_COOKIE_SIZE 16
#define PSB_HW_FEEDBACK_SIZE 8
#define PSB_HW_OOM_CMD_SIZE (6 + DRM_PSB_NUM_RASTER_USE_REG * 2)

struct drm_psb_xhw_arg {
	uint32_t op;
	int ret;
	uint32_t irq_op;
	uint32_t issue_irq;
	uint32_t cookie[PSB_HW_COOKIE_SIZE];
	union {
		struct {
			uint32_t w;	/* also contains msaa info */
			uint32_t h;
			uint32_t size;
			uint32_t clear_p_start;
			uint32_t clear_num_pages;
		} si;
		struct {
			uint32_t fire_flags;
			uint32_t hw_context;
			uint32_t offset;
			uint32_t engine;
			uint32_t flags;
			uint32_t rca;
			uint32_t num_oom_cmds;
			uint32_t oom_cmds[PSB_HW_OOM_CMD_SIZE];
		} sb;
		struct {
			uint32_t pages;
			uint32_t size;
			uint32_t ta_min_size;
		} bi;
		struct {
			uint32_t bca;
			uint32_t rca;
			uint32_t flags;
		} oom;
		struct {
			uint32_t pt_offset;
			uint32_t param_offset;
			uint32_t flags;
		} bl;
		struct {
			uint32_t value;
		} cl;
		uint32_t feedback[PSB_HW_FEEDBACK_SIZE];
	} arg;
};

/* Controlling the kernel modesetting buffers */

#define DRM_PSB_KMS_OFF		0x00
#define DRM_PSB_KMS_ON		0x01
#define DRM_PSB_VT_LEAVE        0x02
#define DRM_PSB_VT_ENTER        0x03
#define DRM_PSB_XHW_INIT        0x04
#define DRM_PSB_XHW             0x05
#define DRM_PSB_EXTENSION       0x06
#define DRM_PSB_SIZES           0x07
#define DRM_PSB_FUSE_REG	0x08
#define DRM_PSB_VBT		0x09
#define DRM_PSB_DC_STATE	0x0A
#define DRM_PSB_ADB		0x0B
#define DRM_PSB_MODE_OPERATION	0x0C
#define DRM_PSB_STOLEN_MEMORY	0x0D
#define DRM_PSB_REGISTER_RW	0x0E
#define DRM_PSB_GTT_MAP         0x0F
#define DRM_PSB_GTT_UNMAP       0x10
#define DRM_PSB_GETPAGEADDRS	0x11
/**
 * NOTE: Add new commands here, but increment
 * the values below and increment their
 * corresponding defines where they're
 * defined elsewhere.
 */
#define DRM_PVR_RESERVED1	0x12
#define DRM_PVR_RESERVED2	0x13
#define DRM_PVR_RESERVED3	0x14
#define DRM_PVR_RESERVED4	0x15
#define DRM_PVR_RESERVED5	0x16

#define DRM_PSB_HIST_ENABLE	0x17
#define DRM_PSB_HIST_STATUS	0x18
#define DRM_PSB_DIET_PROG	0x19
#define DRM_PSB_INIT_COMM	0x1A
#define DRM_PSB_DPST		0x1B
#define DRM_PSB_GAMMA		0x1C
#define DRM_PSB_DPST_BL		0x1D

#define DRM_PVR_RESERVED6	0x1E

/*
 * Xhw commands.
 */

#define PSB_XHW_INIT            0x00
#define PSB_XHW_TAKEDOWN        0x01

#define PSB_XHW_FIRE_RASTER     0x00
#define PSB_XHW_SCENE_INFO      0x01
#define PSB_XHW_SCENE_BIND_FIRE 0x02
#define PSB_XHW_TA_MEM_INFO     0x03
#define PSB_XHW_RESET_DPM       0x04
#define PSB_XHW_OOM             0x05
#define PSB_XHW_TERMINATE       0x06
#define PSB_XHW_VISTEST         0x07
#define PSB_XHW_RESUME          0x08
#define PSB_XHW_TA_MEM_LOAD	0x09
#define PSB_XHW_CHECK_LOCKUP    0x0a

#define PSB_SCENE_FLAG_DIRTY       (1 << 0)
#define PSB_SCENE_FLAG_COMPLETE    (1 << 1)
#define PSB_SCENE_FLAG_SETUP       (1 << 2)
#define PSB_SCENE_FLAG_SETUP_ONLY  (1 << 3)
#define PSB_SCENE_FLAG_CLEARED     (1 << 4)

#define PSB_TA_MEM_FLAG_TA            (1 << 0)
#define PSB_TA_MEM_FLAG_RASTER        (1 << 1)
#define PSB_TA_MEM_FLAG_HOSTA         (1 << 2)
#define PSB_TA_MEM_FLAG_HOSTD         (1 << 3)
#define PSB_TA_MEM_FLAG_INIT          (1 << 4)
#define PSB_TA_MEM_FLAG_NEW_PT_OFFSET (1 << 5)

/*Raster fire will deallocate memory */
#define PSB_FIRE_FLAG_RASTER_DEALLOC  (1 << 0)
/*Isp reset needed due to change in ZLS format */
#define PSB_FIRE_FLAG_NEEDS_ISP_RESET (1 << 1)
/*These are set by Xpsb. */
#define PSB_FIRE_FLAG_XHW_MASK        0xff000000
/*The task has had at least one OOM and Xpsb will
  send back messages on each fire. */
#define PSB_FIRE_FLAG_XHW_OOM         (1 << 24)

#define PSB_SCENE_ENGINE_TA        0
#define PSB_SCENE_ENGINE_RASTER    1
#define PSB_SCENE_NUM_ENGINES      2

#define PSB_LOCKUP_RASTER          (1 << 0)
#define PSB_LOCKUP_TA              (1 << 1)

struct drm_psb_dev_info_arg {
	uint32_t num_use_attribute_registers;
};
#define DRM_PSB_DEVINFO         0x01

#define PSB_MODE_OPERATION_MODE_VALID	0x01
#define PSB_MODE_OPERATION_SET_DC_BASE  0x02

#endif
