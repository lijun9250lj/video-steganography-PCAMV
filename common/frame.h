/*****************************************************************************
 * frame.h: h264 encoder library
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

#ifndef X264_FRAME_H
#define X264_FRAME_H

/* number of pixels past the edge of the frame, for motion estimation/compensation */
#define PADH 32
#define PADV 32

typedef struct
{
    /* */
    int     i_poc;
    int     i_type;// 帧类型（I、P、B）
    int     i_qpplus1;
    int64_t i_pts;
    int     i_frame;    /* Presentation frame number */
    int     i_frame_num; /* Coded frame number */
    int     b_kept_as_ref;
    float   f_qp_avg_rc; /* QPs as decided by ratecontrol */
    float   f_qp_avg_aq; /* QPs as decided by AQ in addition to ratecontrol */

    /* YUV buffer */
    int     i_plane;
    int     i_stride[3];//存储区域的跨度，因为要考虑运动估计，Y有左右两边各加32各像素，UV左右两边各加16
    int     i_width[3];//实际的图像宽度
    int     i_lines[3];//实际的图像高度
	// lowres的stride
    int     i_stride_lowres;
	//lowres的width
    int     i_width_lowres;
	//lowres的行数
    int     i_lines_lowres;
	// YUV指针，实际的像素存储区[0-2]，从x264_image_t拷贝过来的，按照大矩阵的方式存储lijun
    uint8_t *plane[3];//plane[0]为Y，与filtered[0]指向相同，plane[1]为U，指向buffer[1]的实际像素，plane[2]为V，指向buffer[2]的实际像素，lijun

	// 指向buffer[0]
    uint8_t *filtered[4]; /* filtered[0]整像素,filtered[1][2][3] H半像素, V半像素, HV对角线半像素 ，分别指向下面buffer[0]中的四个部分，是实际数据和插值数据    add lijun*/
    uint8_t *lowres[4]; /* half-size copy of input frame: Orig, H, V, HV 原始图像大小的一般，降低分辨率，干什么用？？*/
    uint16_t *integral;

    /* for unrestricted mv we allocate more data than needed
     * allocated data are stored in buffer */
    uint8_t *buffer[4];//存储原始像素+填充的（me）的像素，buffer[0]:Y,大小为4个（352+64）*（288+64）,上下加32，右边加64用于me或插值计算??，每个（352+64）*（288+64）分别存储整、水平半、垂直半、对角半，
                         // buffer[1]:U    周围有填充
						 // buffer[2]:V    周围有填充
	
	
	uint8_t *buffer_lowres[4];//如果需要半分辨率则存储在这里，干什么？？？lijun

    /* motion data */
    int8_t  *mb_type;
    int16_t (*mv[2])[2];
    int16_t (*lowres_mvs[2][X264_BFRAME_MAX+1])[2];
    int     *lowres_mv_costs[2][X264_BFRAME_MAX+1];
    int8_t  *ref[2];
    int     i_ref[2];
    int     ref_poc[2][16];
    int     inv_ref_poc[16]; // inverse values (list0 only) to avoid divisions in MB encoding

    /* for adaptive B-frame decision.
     * contains the SATD cost of the lowres frame encoded in various modes
     * FIXME: how big an array do we need? */
    int     i_cost_est[X264_BFRAME_MAX+2][X264_BFRAME_MAX+2];
    int     i_cost_est_aq[X264_BFRAME_MAX+2][X264_BFRAME_MAX+2];
    int     i_satd; // the i_cost_est of the selected frametype
    int     i_intra_mbs[X264_BFRAME_MAX+2];
    int     *i_row_satds[X264_BFRAME_MAX+2][X264_BFRAME_MAX+2];
    int     *i_row_satd;
    int     *i_row_bits;
    int     *i_row_qp;
    float   *f_qp_offset;
    int     b_intra_calculated;
    uint16_t *i_intra_cost;
    uint16_t *i_inv_qscale_factor;

    /* threading */
    int     i_lines_completed; /* in pixels */
    int     i_reference_count; /* number of threads using this frame (not necessarily the number of pointers) */
    x264_pthread_mutex_t mutex;
    x264_pthread_cond_t  cv;

} x264_frame_t;

typedef void (*x264_deblock_inter_t)( uint8_t *pix, int stride, int alpha, int beta, int8_t *tc0 );
typedef void (*x264_deblock_intra_t)( uint8_t *pix, int stride, int alpha, int beta );
typedef struct
{
    x264_deblock_inter_t deblock_v_luma;
    x264_deblock_inter_t deblock_h_luma;
    x264_deblock_inter_t deblock_v_chroma;
    x264_deblock_inter_t deblock_h_chroma;
    x264_deblock_intra_t deblock_v_luma_intra;
    x264_deblock_intra_t deblock_h_luma_intra;
    x264_deblock_intra_t deblock_v_chroma_intra;
    x264_deblock_intra_t deblock_h_chroma_intra;
} x264_deblock_function_t;

x264_frame_t *x264_frame_new( x264_t *h );
void          x264_frame_delete( x264_frame_t *frame );

int           x264_frame_copy_picture( x264_t *h, x264_frame_t *dst, x264_picture_t *src );

void          x264_frame_expand_border( x264_t *h, x264_frame_t *frame, int mb_y, int b_end );
void          x264_frame_expand_border_filtered( x264_t *h, x264_frame_t *frame, int mb_y, int b_end );
void          x264_frame_expand_border_lowres( x264_frame_t *frame );
void          x264_frame_expand_border_mod16( x264_t *h, x264_frame_t *frame );

void          x264_frame_deblock( x264_t *h );
void          x264_frame_deblock_row( x264_t *h, int mb_y );

void          x264_frame_filter( x264_t *h, x264_frame_t *frame, int mb_y, int b_end );
void          x264_frame_init_lowres( x264_t *h, x264_frame_t *frame );

void          x264_deblock_init( int cpu, x264_deblock_function_t *pf );

void          x264_frame_cond_broadcast( x264_frame_t *frame, int i_lines_completed );
void          x264_frame_cond_wait( x264_frame_t *frame, int i_lines_completed );

void          x264_frame_push( x264_frame_t **list, x264_frame_t *frame );
x264_frame_t *x264_frame_pop( x264_frame_t **list );
void          x264_frame_unshift( x264_frame_t **list, x264_frame_t *frame );
x264_frame_t *x264_frame_shift( x264_frame_t **list );
void          x264_frame_push_unused( x264_t *h, x264_frame_t *frame );
x264_frame_t *x264_frame_pop_unused( x264_t *h );
void          x264_frame_sort( x264_frame_t **list, int b_dts );
#define x264_frame_sort_dts(list) x264_frame_sort(list, 1)
#define x264_frame_sort_pts(list) x264_frame_sort(list, 0)

#endif
