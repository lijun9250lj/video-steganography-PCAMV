/*****************************************************************************
 * common.h: h264 encoder
 *****************************************************************************
 * Copyright (C) 2003-2008 x264 project
 *
 * Authors: Laurent Aimar <fenrir@via.ecp.fr>
 *          Loren Merritt <lorenm@u.washington.edu>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02111, USA.
 *****************************************************************************/

#ifndef X264_COMMON_H
#define X264_COMMON_H

/****************************************************************************
 * Macros
 ****************************************************************************/
#define X264_MIN(a,b) ( (a)<(b) ? (a) : (b) )
#define X264_MAX(a,b) ( (a)>(b) ? (a) : (b) )
#define X264_MIN3(a,b,c) X264_MIN((a),X264_MIN((b),(c)))
#define X264_MAX3(a,b,c) X264_MAX((a),X264_MAX((b),(c)))
#define X264_MIN4(a,b,c,d) X264_MIN((a),X264_MIN3((b),(c),(d)))
#define X264_MAX4(a,b,c,d) X264_MAX((a),X264_MAX3((b),(c),(d)))
#define XCHG(type,a,b) do{ type t = a; a = b; b = t; } while(0)
#define FIX8(f) ((int)(f*(1<<8)+.5))

#define CHECKED_MALLOC( var, size )\
{\
    var = x264_malloc( size );\
    if( !var )\
    {\
        x264_log( h, X264_LOG_ERROR, "malloc failed\n" );\
        goto fail;\
    }\
}

#define X264_BFRAME_MAX 16
#define X264_THREAD_MAX 128
#define X264_SLICE_MAX 4
#define X264_NAL_MAX (4 + X264_SLICE_MAX)
#define X264_PCM_COST (386*8)

// number of pixels (per thread) in progress at any given time.
// 16 for the macroblock in progress + 3 for deblocking + 3 for motion compensation filter + 2 for extra safety
#define X264_THREAD_HEIGHT 24

/****************************************************************************
 * Includes
 ****************************************************************************/
#include "osdep.h"
#include <stdarg.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "x264.h"
#include "bs.h"
#include "set.h"
#include "predict.h"
#include "pixel.h"
#include "mc.h"
#include "frame.h"
#include "dct.h"
#include "cabac.h"
#include "quant.h"

/****************************************************************************
 * Generals functions
 ****************************************************************************/
/* x264_malloc : will do or emulate a memalign
 * you have to use x264_free for buffers allocated with x264_malloc */
void *x264_malloc( int );
void *x264_realloc( void *p, int i_size );
void  x264_free( void * );

/* x264_slurp_file: malloc space for the whole file and read it */
char *x264_slurp_file( const char *filename );

/* mdate: return the current date in microsecond */
int64_t x264_mdate( void );

/* x264_param2string: return a (malloced) string containing most of
 * the encoding options */
char *x264_param2string( x264_param_t *p, int b_res );

/* log */
void x264_log( x264_t *h, int i_level, const char *psz_fmt, ... );

void x264_reduce_fraction( int *n, int *d );
void x264_init_vlc_tables();

static inline uint8_t x264_clip_uint8( int x )
{
    return x&(~255) ? (-x)>>31 : x;
}

static inline int x264_clip3( int v, int i_min, int i_max )
{
    return ( (v < i_min) ? i_min : (v > i_max) ? i_max : v );
}

static inline double x264_clip3f( double v, double f_min, double f_max )
{
    return ( (v < f_min) ? f_min : (v > f_max) ? f_max : v );
}

static inline int x264_median( int a, int b, int c )
{
    int t = (a-b)&((a-b)>>31);
    a -= t;
    b += t;
    b -= (b-c)&((b-c)>>31);
    b += (a-b)&((a-b)>>31);
    return b;
}

static inline void x264_median_mv( int16_t *dst, int16_t *a, int16_t *b, int16_t *c )
{
    dst[0] = x264_median( a[0], b[0], c[0] );
    dst[1] = x264_median( a[1], b[1], c[1] );
}

static inline int x264_predictor_difference( int16_t (*mvc)[2], intptr_t i_mvc )
{
    int sum = 0, i;
    for( i = 0; i < i_mvc-1; i++ )
    {
        sum += abs( mvc[i][0] - mvc[i+1][0] )
             + abs( mvc[i][1] - mvc[i+1][1] );
    }
    return sum;
}

/****************************************************************************
 *
 ****************************************************************************/
enum slice_type_e
{
    SLICE_TYPE_P  = 0,
    SLICE_TYPE_B  = 1,
    SLICE_TYPE_I  = 2,
    SLICE_TYPE_SP = 3,
    SLICE_TYPE_SI = 4
};

static const char slice_type_to_char[] = { 'P', 'B', 'I', 'S', 'S' };

typedef struct
{
    x264_sps_t *sps;
    x264_pps_t *pps;

    int i_type;
    int i_first_mb;
    int i_last_mb;

    int i_pps_id;

    int i_frame_num;

    int b_mbaff;
    int b_field_pic;
    int b_bottom_field;

    int i_idr_pic_id;   /* -1 if nal_type != 5 */

    int i_poc_lsb;
    int i_delta_poc_bottom;

    int i_delta_poc[2];
    int i_redundant_pic_cnt;

    int b_direct_spatial_mv_pred;

    int b_num_ref_idx_override;
    int i_num_ref_idx_l0_active;
    int i_num_ref_idx_l1_active;

    int b_ref_pic_list_reordering_l0;
    int b_ref_pic_list_reordering_l1;
    struct {
        int idc;
        int arg;
    } ref_pic_list_order[2][16];

    int i_cabac_init_idc;

    int i_qp;
    int i_qp_delta;
    int b_sp_for_swidth;
    int i_qs_delta;

    /* deblocking filter */
    int i_disable_deblocking_filter_idc;
    int i_alpha_c0_offset;
    int i_beta_offset;

} x264_slice_header_t;

/* From ffmpeg
 */
#define X264_SCAN8_SIZE (6*8)
#define X264_SCAN8_0 (4+1*8)

static const int x264_scan8[16+2*4+3] =
{
    /* Luma */
    4+1*8, 5+1*8, 4+2*8, 5+2*8,
    6+1*8, 7+1*8, 6+2*8, 7+2*8,
    4+3*8, 5+3*8, 4+4*8, 5+4*8,
    6+3*8, 7+3*8, 6+4*8, 7+4*8,

    /* Cb */
    1+1*8, 2+1*8,
    1+2*8, 2+2*8,

    /* Cr */
    1+4*8, 2+4*8,
    1+5*8, 2+5*8,

    /* Luma DC */
    4+5*8,

    /* Chroma DC */
    5+5*8, 6+5*8
};
/*
   0 1 2 3 4 5 6 7
 0
 1   B B   L L L L
 2   B B   L L L L
 3         L L L L
 4   R R   L L L L
 5   R R   DyDuDv
*/

typedef struct x264_ratecontrol_t   x264_ratecontrol_t;

struct x264_t
{
    /* 编码器其参数，包括量化、编码级别等的一些参数 */
    x264_param_t    param;

    x264_t          *thread[X264_THREAD_MAX];
    x264_pthread_t  thread_handle;
    int             b_thread_active;
    int             i_thread_phase; /* which thread to use for the next frame */

    /* bitstream output */
    struct
    {
        int         i_nal;
        x264_nal_t  nal[X264_NAL_MAX];
        int         i_bitstream;    /* size of p_bitstream */
        uint8_t     *p_bitstream;   /* will hold data for all nal */
        bs_t        bs;
        int         i_frame_size;
    } out;

    /**** thread synchronization starts here ****/

    /* 编码帧号，用于计算POC */
    int             i_frame;

    int             i_frame_offset; /* decoding only */
    int             i_frame_num;    /* decoding only */
    int             i_poc_msb;      /* decoding only */
    int             i_poc_lsb;      /* decoding only */
    int             i_poc;          /* decoding only */

    int             i_thread_num;   /* threads only */
    int             i_nal_type;     /* threads only */
    int             i_nal_ref_idc;  /* threads only */

    /* We use only one SPS and one PPS */
    x264_sps_t      sps_array[1];
    x264_sps_t      *sps;
    x264_pps_t      pps_array[1];
    x264_pps_t      *pps;
    int             i_idr_pic_id;

    /* quantization matrix for decoding, [cqm][qp%6][coef_y][coef_x] */
    int             (*dequant4_mf[4])[4][4]; /* [4][6][4][4] */
    int             (*dequant8_mf[2])[8][8]; /* [2][6][8][8] */
    /* quantization matrix for trellis, [cqm][qp][coef] */
    int             (*unquant4_mf[4])[16];   /* [4][52][16] */
    int             (*unquant8_mf[2])[64];   /* [2][52][64] */
    /* quantization matrix for deadzone */
    uint16_t        (*quant4_mf[4])[16];     /* [4][52][16] */
    uint16_t        (*quant8_mf[2])[64];     /* [2][52][64] */
    uint16_t        (*quant4_bias[4])[16];   /* [4][52][16] */
    uint16_t        (*quant8_bias[2])[64];   /* [2][52][64] */

    const uint8_t   *chroma_qp_table; /* includes both the nonlinear luma->chroma mapping and chroma_qp_offset */

    DECLARE_ALIGNED_16( uint32_t nr_residual_sum[2][64] );
    DECLARE_ALIGNED_16( uint16_t nr_offset[2][64] );
    uint32_t        nr_count[2];

    /* Slice header */
    x264_slice_header_t sh;

    /* cabac context */
    x264_cabac_t    cabac;

    struct
    {
        /* 已确定帧类型，待编码帧。每一个GOP编码前，每一帧的类型都已经确定 */
        x264_frame_t *current[X264_BFRAME_MAX*4+3]; // 
        /* 当进行编码时，从这里取出一帧数据*/
        x264_frame_t *next[X264_BFRAME_MAX*4+3];
        /*用于回收那些在编码中分配的frame空间，有新的需要时，直接拿来用，不用重新分析新的空间*/
        x264_frame_t *unused[X264_BFRAME_MAX*4 + X264_THREAD_MAX*2 + 16+4]; 
        /* 上一个非B帧 For adaptive B decision */
        x264_frame_t *last_nonb;

        /* 参考帧序列，都是已重建帧 frames used for reference + sentinels */
        x264_frame_t *reference[16+2];
		/*上一个关键帧序号*/
        int i_last_idr;
		/*已经接受的输入帧数，frame结构中的i_input指示当前帧的(播放顺序)序号*/
        int i_input;  
        /* Number of frames allocated in the decoded picture buffer */
        int i_max_dpb;  
		/* 最大前向参考帧数量 */
        int i_max_ref0;
		/* 最大后向参考帧数量 */
        int i_max_ref1;
        int i_delay;    /* Number of frames buffered for B reordering */
        /* 是否采用半像素精度 */
		int b_have_lowres;  /* Whether 1/2 resolution luma planes are being used */
        int b_have_sub8x8_esa;

    } frames;

    /* 指向当前编码帧 */
    x264_frame_t    *fenc;

    /* 指向当前重构帧，重建帧的序号比当前编码帧的讯号小1 */
    x264_frame_t    *fdec;

    /* 前向参考帧的数量 */
    int             i_ref0;
	/* 前向参考帧数组*/
    x264_frame_t    *fref0[16+3];     /* ref list 0 */
	/* 后向参考帧数量 */
    int             i_ref1;
	/* 后向参考帧数组 */
    x264_frame_t    *fref1[16+3];     /* ref list 1 */
    int             b_ref_reorder[2];



    /* 当前宏块的DCT系数 Current MB DCT coeffs */
    struct
    {
		// Luma的直流系数
        DECLARE_ALIGNED_16( int16_t luma16x16_dc[16] );
		// Cr,Cb的直流系数
        DECLARE_ALIGNED_16( int16_t chroma_dc[2][4] );
        // FIXME share memory?
        DECLARE_ALIGNED_16( int16_t luma8x8[4][64] );
        DECLARE_ALIGNED_16( int16_t luma4x4[16+8][16] );
    } dct;

    /* MB table and cache for current frame/mb */
    struct
    {
        int     i_mb_count; /* number of mbs in a frame */

        /* Strides */
        int     i_mb_stride;
        int     i_b8_stride;
        int     i_b4_stride;

        /* Current index当前宏块在当前帧中的各种索引序号 */
        int     i_mb_x;
        int     i_mb_y;
        int     i_mb_xy;
        int     i_b8_xy;
        int     i_b4_xy;

        /* Search parameters  该宏块的运动估计相关参数*/
        int     i_me_method;//运动估计方式
        int     i_subpel_refine;
        int     b_chroma_me;
        int     b_trellis;
        int     b_noise_reduction;
        int     i_psy_rd; /* Psy RD strength--fixed point value*/
        int     i_psy_trellis; /* Psy trellis strength--fixed point value*/

        int     b_interlaced;

        /* Allowed qpel MV range to stay within the picture + emulated edge pixels   允许的1/4像素mv范围 */
        int     mv_min[2];
        int     mv_max[2];
        /* Subpel MV range for motion search.
         * same mv_min/max but includes levels' i_mv_range. 允许的1/2像素mv范围*/
        int     mv_min_spel[2];
        int     mv_max_spel[2];
        /* Fullpel MV range for motion search 允许的整像素mv范围*/
        int     mv_min_fpel[2];
        int     mv_max_fpel[2];

        /* neighboring MBs 当前宏块的相邻宏块的信息*/
        unsigned int i_neighbour;
        unsigned int i_neighbour8[4];       /* neighbours of each 8x8 or 4x4 block that are available */
        unsigned int i_neighbour4[16];      /* at the time the block is coded */
        int     i_mb_type_top;
        int     i_mb_type_left;
        int     i_mb_type_topleft;
        int     i_mb_type_topright;
        int     i_mb_prev_xy;
        int     i_mb_top_xy;

        /**** thread synchronization ends here ****/
        /* subsequent variables are either thread-local or constant,
         * and won't be copied from one thread to another */

        /* mb table */
        int8_t  *type;                      /* mb type */
        int8_t  *qp;                        /* mb qp */
        int16_t *cbp;                       /* mb cbp: 0x0?: luma, 0x?0: chroma, 0x100: luma dc, 0x0200 and 0x0400: chroma dc  (all set for PCM)*///cbp用于表示是否存在非零值
        int8_t  (*intra4x4_pred_mode)[8];   //intra4x4_pred_mode为指向一个含有8个元素数组的指针，intra4x4_pred_mode[x][y]为第x个宏块中y位置的值（y小于8）add lijun
											/* intra4x4 pred mode. for non I4x4 set to I_PRED_4x4_DC(2) */
                                            /* actually has only 7 entries; set to 8 for write-combining optimizations */
        uint8_t (*non_zero_count)[16+4+4];  /* nzc. for I_PCM set to 16 */
        int8_t  *chroma_pred_mode;          /* chroma_pred_mode. cabac only. for non intra I_PRED_CHROMA_DC(0) */
        int16_t (*mv[2])[2];                /* mb mv. set to 0 for intra mb */
        int16_t (*mvd[2])[2];               /* mb mv difference with predict. set to 0 if intra. cabac only */
        int8_t   *ref[2];                   /* mb ref. set to -1 if non used (intra or Lx only) */
        int16_t (*mvr[2][32])[2];           /* 16x16 mv for each possible ref */
        int8_t  *skipbp;                    /* block pattern for SKIP or DIRECT (sub)mbs. B-frames + cabac only */
        int8_t  *mb_transform_size;         /* transform_size_8x8_flag of each mb */
        uint8_t *intra_border_backup[2][3]; /* bottom pixels of the previous mb row, used for intra prediction after the framebuffer has been deblocked */
        uint8_t (*nnz_backup)[16];          /* when using cavlc + 8x8dct, the deblocker uses a modified nnz */

        /* current value */
        int     i_type;
        int     i_partition;
        DECLARE_ALIGNED_4( uint8_t i_sub_partition[4] );
        int     b_transform_8x8;

        int     i_cbp_luma;
        int     i_cbp_chroma;

        int     i_intra16x16_pred_mode;
        int     i_chroma_pred_mode;

        /* skip flags for i4x4 and i8x8
         * 0 = encode as normal.
         * 1 (non-RD only) = the DCT is still in h->dct, restore fdec and skip reconstruction.
         * 2 (RD only) = the DCT has since been overwritten by RD; restore that too. */
        int i_skip_intra;
        /* skip flag for motion compensation */
        /* if we've already done MC, we don't need to do it again */
        int b_skip_mc;

        struct
        {
            /* space for p_fenc and p_fdec */
			#define FENC_STRIDE 16
			#define FDEC_STRIDE 32
            DECLARE_ALIGNED_16( uint8_t fenc_buf[24*FENC_STRIDE] );//用于存储宏块编码像素数据，包含亮度与色度，前16行为亮度，后8行分成两个8*8（为色度）
			DECLARE_ALIGNED_16(uint8_t fenc_buf_ih[24 * FENC_STRIDE]);//ih 信息隐藏， 用于保存当前块的重构块，但是结构与fenc_buf相同，方便计算拉格朗日失真，lijun
			DECLARE_ALIGNED_16( uint8_t fdec_buf[27*FDEC_STRIDE] );//用于存储宏块重建像素数据，//fdec_buf[]和fenc_buf[]主要的区别在于fdec_buf[]像素块的左边和上边包含了左上方相邻块用于预测的像素。见雷的源码解读文章

            /* i4x4 and i8x8 backup data, for skipping the encode stage when possible */
            DECLARE_ALIGNED_16( uint8_t i4x4_fdec_buf[16*16] );
            DECLARE_ALIGNED_16( uint8_t i8x8_fdec_buf[16*16] );
            DECLARE_ALIGNED_16( int16_t i8x8_dct_buf[3][64] );
            DECLARE_ALIGNED_16( int16_t i4x4_dct_buf[15][16] );
            uint32_t i4x4_nnz_buf[4];
            uint32_t i8x8_nnz_buf[4];
            int i4x4_cbp;
            int i8x8_cbp;

            /* Psy trellis DCT data */
            DECLARE_ALIGNED_16( int16_t fenc_dct8[4][64] );
            DECLARE_ALIGNED_16( int16_t fenc_dct4[16][16] );

            /* Psy RD SATD scores */
            int fenc_satd[4][4];
            int fenc_satd_sum;
            int fenc_sa8d[2][2];
            int fenc_sa8d_sum;

            /* pointer over mb of the frame to be compressed */
            uint8_t *p_fenc[3];//指向fenc_buf[]，通过p_fenc_plane[]拷贝数据，最终保存在 fenc_buf[]中      将要编码的宏块内存,Y,U,V，它们中的3个元素[0]、[1]、[2]分别指向分别指向对应缓存fenc_buf的Y、U、V分量
            /* pointer to the actual source frame, not a block copy */
			uint8_t *p_fenc_ih[3];//指向fenc_buf_ih[]，
			uint8_t *p_fenc_plane[3];//指向当前编码帧的h->fenc->plane[i][i_pix_offset], i_pix_offset是偏移到当前块    指向原图像中宏块的内存

            /* pointer over mb of the frame to be reconstructed  */
            uint8_t *p_fdec[3]; //指向将要重建的宏块内存，与p_fenc类似

            /* pointer over mb of the references */
            int i_fref[2]; 
			//指向参考帧内存数据，p_fref[0]对应前向参考帧，p_fref[1]对应后向参考帧

			//内插像素存储空间    [2]为前向和后向   [32]应该为参考帧   [4+2]6个分量，包括整点像素、1/2水平内插，1/2垂直内插，1/2斜对角内插，色度，色度 add lijun
            uint8_t *p_fref[2][32][4+2]; /* last: lN, lH, lV, lHV, cU, cV */ 
            uint16_t *p_integral[2][16];

            /* fref stride */
            int     i_stride[3];
        } pic;//当前宏块的像素空间、像素指针、重建块的

        /* 宏块信息缓存,表格中存储了一整个宏块的信息，每一个元素代表了一个4x4块 */
        struct
        { 
            /* real intra4x4_pred_mode if I_4X4 or I_8X8, I_PRED_4x4_DC if mb available, -1 if not 
			 * mb.cache.intra4x4_pred_mode[],ref[],mv[][0~1]格式如下
			 *   |
			 * --+--------------
			 *   | 0 0 0 y y y y y
			 *   | 0 U U y Y Y Y Y
			 *   | 0 U U y Y Y Y Y
			 *   | 0 0 0 y Y Y Y Y
			 *   | 0 V V y Y Y Y Y
			 *   | 0 V V 0 L C C 0
			**/
			int8_t  intra4x4_pred_mode[X264_SCAN8_SIZE]; //X264_SCAN8_SIZE=48//在帧内4*4预测方式时各个子块的预测方式，存储结构如上，

            /* i_non_zero_count if available else 0x80 */
            uint8_t non_zero_count[X264_SCAN8_SIZE];


            /* -1 if unused, -2 if unavailable */
            DECLARE_ALIGNED_4( int8_t ref[2][X264_SCAN8_SIZE] );

            /* 0 if not available */
            DECLARE_ALIGNED_16( int16_t mv[2][X264_SCAN8_SIZE][2] );
            DECLARE_ALIGNED_8( int16_t mvd[2][X264_SCAN8_SIZE][2] );

            /* 1 if SKIP or DIRECT. set only for B-frames + CABAC */
            DECLARE_ALIGNED_4( int8_t skip[X264_SCAN8_SIZE] );

            DECLARE_ALIGNED_16( int16_t direct_mv[2][X264_SCAN8_SIZE][2] );
            DECLARE_ALIGNED_4( int8_t  direct_ref[2][X264_SCAN8_SIZE] );
            DECLARE_ALIGNED_4( int16_t pskip_mv[2] );

            /* number of neighbors (top and left) that used 8x8 dct */
            int     i_neighbour_transform_size;
            int     i_neighbour_interlaced;

            /* neighbor CBPs */
            int     i_cbp_top;
            int     i_cbp_left;
        } cache;

        /* */
        int     i_qp;       /* current qp */
        int     i_chroma_qp;
        int     i_last_qp;  /* last qp */
        int     i_last_dqp; /* last delta qp */
        int     b_variable_qp; /* whether qp is allowed to vary per macroblock */
        int     b_lossless;
        int     b_direct_auto_read; /* take stats for --direct auto from the 2pass log */
        int     b_direct_auto_write; /* analyse direct modes, to use and/or save */

        /* B_direct and weighted prediction */
        int16_t dist_scale_factor[16][2];
        int16_t bipred_weight[32][4];
        /* maps fref1[0]'s ref indices into the current list0 */
        int8_t  map_col_to_list0_buf[2]; // for negative indices
        int8_t  map_col_to_list0[16];
    } mb;

	/*用作视频隐写的的全局变量,每一帧宏块个数为22*18*/
	struct
	{
		struct {
			int i_type;//宏块类型
			int i_qp;
			//int i_qp_c;
			int     i_partition;//分割类型
			uint8_t used; // 用于隐写载体构造标记，表示是否用作载体
			uint8_t i_sub_partition[4];//可能的子块分割模式
			// 预测模式
			int8_t  intra4x4_pred_mode[16];
			int8_t intra4x4_pred_mode_stego[16]; // 保存对应模式的次优
			int intra4x4_stego_cost[16];
			int16_t mv_stego[16][2];
			int inter_stego_cost[16];
			//运动矢量相关的，只考虑P帧，不考虑双向
			int8_t ref[16];
			int16_t mv[16][2];//存储me后的mv
			//int8_t filp;//是否需要翻转（修改）	
			int16_t pskip_mv_[2];//保存每个宏块的可能的pskip模式下的预测mv，用于在计算复杂性失真时填充skip模式中的运动矢量，
		}cache[396];
		float rho_final[6336];//每一个宏块的失真 396*16=6336每个帧可能的最大运动矢量数，
		float rho_com[6336];//与复杂性相关的失真，与rho_final配套，可直接用于stc隐写，与后面的rho_com_all不同

		int mv_h[6336];//构造一个以4*4为基本单位的运动矢量矩阵，水平分量，通过调用lib库用于计算有关mv复杂性的失真
		int mv_v[6336];//构造一个以4*4为基本单位的运动矢量矩阵，垂直分量
		float * rho_com_all;//保存每个4*4为单位的mv的有关复杂性的失真，所有宏块都有，skip等按运动矢量0对待，其内存分配在lib中进行

		uint8_t cover[6336];//载体 LSB（h+v）
		uint8_t stego[6336];//隐写后载体
		int length;//一帧所有mv的长度记录
		//uint8_t b_frame_used;//标记这帧用作嵌入,
		uint32_t num_filp;//总共需要修改的mv数量
		uint8_t message[6336];//嵌入的原始消息
		int8_t filp[6336];//这个mv是否需要修改
		uint16_t i_mv_no;//标记一帧中所有可用mv的编号,用于嵌入时定位mv
		uint16_t num_mv_modify_real;//标记一帧中在修改时真正修改的数量，用于校验  
		uint16_t num_p_skip_before_analysis;//统计数量，临时
		uint16_t num_p_skip_after_encode;//统计数量，临时
		uint8_t firstTime;
		uint8_t embed_flag;
		struct {
			uint32_t num_p_block;//代表全视频的mv数，载体数
			uint32_t num_non_optimal_mv;//暂时不用
			uint32_t num_optimal_1_neighbor;//替代mv位于1邻域的个数
			uint32_t num_optimal_2_neighbor;//替代mv位于2邻域的个数
			uint32_t num_error_pos;//原始mv局部最优找不到局部最优替代，原始mv局部非最优que找不到非最优替代，的个数
			uint32_t num_message;//总的嵌入消息个数
		}stat;//保存一些统计信息lijun
	}info;

    /* 码率控制 */
    x264_ratecontrol_t *rc;

    /* 状态（统计信息） */
    struct
    {
        /* Current frame stats */
        struct
        {
            /* MV bits (MV+Ref+Block Type) */
            int i_mv_bits;
            /* Texture bits (DCT coefs) */
            int i_tex_bits;
            /* ? */
            int i_misc_bits;
            /* MB type counts */
            int i_mb_count[19];
            int i_mb_count_i;
            int i_mb_count_p;
            int i_mb_count_skip;
            int i_mb_count_8x8dct[2];
            int i_mb_count_ref[2][32];
            int i_mb_partition[17];
            /* Estimated (SATD) cost as Intra/Predicted frame */
            /* XXX: both omit the cost of MBs coded as P_SKIP */
            int i_intra_cost;
            int i_inter_cost;
            int i_mbs_analysed;
            /* Adaptive direct mv pred */
            int i_direct_score[2];
            /* Metrics */
            int64_t i_ssd[3];
            double f_ssim;
        } frame;

        /* Cumulated stats */

        /* per slice info */
        int     i_slice_count[5];
        int64_t i_slice_size[5];
        double  f_slice_qp[5];
        int     i_consecutive_bframes[X264_BFRAME_MAX+1];
        /* */
        int64_t i_ssd_global[5];
        double  f_psnr_average[5];
        double  f_psnr_mean_y[5];
        double  f_psnr_mean_u[5];
        double  f_psnr_mean_v[5];
        double  f_ssim_mean_y[5];
        /* */
        int64_t i_mb_count[5][19];
        int64_t i_mb_partition[2][17];
        int64_t i_mb_count_8x8dct[2];
        int64_t i_mb_count_ref[2][2][32];
        /* */
        int     i_direct_score[2];
        int     i_direct_frames[2];

		struct//用作保存信息隐藏过程中的全局信息
		{
			int64_t i_mv_num;//总可用mv数
			int64_t i_message_num;//总嵌入消息数
			int64_t i_modify_mv_num;//总共修改的mv数
		}info;

    } stat;

    void *scratch_buffer; /* for any temporary storage that doesn't want repeated malloc */

    /* CPU functions dependents */
    x264_predict_t      predict_16x16[4+3];
    x264_predict_t      predict_8x8c[4+3];
    x264_predict8x8_t   predict_8x8[9+3];
    x264_predict_t      predict_4x4[9+3];
    x264_predict_8x8_filter_t predict_8x8_filter;

    x264_pixel_function_t pixf;
    x264_mc_functions_t   mc;
    x264_dct_function_t   dctf;
    x264_zigzag_function_t zigzagf;
    x264_quant_function_t quantf;
    x264_deblock_function_t loopf;

	/*自己添加的关于宏块的信息保存变量，用于隐写*/



#if VISUALIZE
    struct visualize_t *visualize;
#endif
};

// included at the end because it needs x264_t
#include "macroblock.h"

#ifdef HAVE_MMX
#include "x86/util.h"
#endif

#endif

