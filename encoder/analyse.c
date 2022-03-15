/*****************************************************************************
 * analyse.c: h264 encoder library
 *****************************************************************************
 * Copyright (C) 2003-2008 x264 project
 *
 * Authors: Laurent Aimar <fenrir@via.ecp.fr>
 *          Loren Merritt <lorenm@u.washington.edu>
 *          Jason Garrett-Glaser <darkshikari@gmail.com>
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

#define _ISOC99_SOURCE
#include <limits.h>
#include <math.h>
#ifndef _MSC_VER
#include <unistd.h>
#endif

#include "common/common.h"
#include "common/cpu.h"
#include "macroblock.h"
#include "me.h"
#include "ratecontrol.h"
#include "analyse.h"
#include "rdo.c"

#include "embed.h"

//#include "stc_embed_c.h"
//#include "stc_extract_c.h"//lijun


#define log2f(x)     ((float)log((double)(x)))/(log((double)2))

typedef struct
{
    /* 16x16 */
    int i_ref;
    int       i_rd16x16;
    x264_me_t me16x16;

    /* 8x8 */
    int       i_cost8x8;
    /* [ref][0] is 16x16 mv, [ref][1..4] are 8x8 mv from partition [0..3] */
    DECLARE_ALIGNED_4( int16_t mvc[32][5][2] );
    x264_me_t me8x8[4];

    /* Sub 4x4 */
    int       i_cost4x4[4]; /* cost per 8x8 partition */
    x264_me_t me4x4[4][4];

    /* Sub 8x4 */
    int       i_cost8x4[4]; /* cost per 8x8 partition */
    x264_me_t me8x4[4][2];

    /* Sub 4x8 */
    int       i_cost4x8[4]; /* cost per 8x8 partition */
    x264_me_t me4x8[4][2];

    /* 16x8 */
    int       i_cost16x8;
    x264_me_t me16x8[2];

    /* 8x16 */
    int       i_cost8x16;
    x264_me_t me8x16[2];

} x264_mb_analysis_list_t;

typedef struct
{
    /* conduct the analysis using this lamda and QP */
    int i_lambda;//ĳ��ϵ�� * QStep����ר����SATD���任����Ժͣ�Ч�ʸߣ�����ô��ȷ�����ڳ���ɸѡ������ģʽ��ʹ�õģ���Ϊ�˰�bitsת����SATD��ͬ�ġ���λ/��������
    int i_lambda2;//ĳ��ϵ�� * QStep * QStep����ר����RDģʽ��ʹ�õģ���Ϊ�˰�bitsת����ssd����ֵƽ���ͣ�����ȷ���������ȷ������ʱʹ�ã���ͬ�ġ���λ/��������ssdΪƽ������
    int i_qp;
	//4*4*2048 + 1
    int16_t *p_cost_mv;
	//33[-16...0...16]
    uint16_t *p_cost_ref0;
    uint16_t *p_cost_ref1;
    int i_mbrd;// ��ʧ��ģʽ������lijun /* mbrd == 1 -> RD mode decision *//* mbrd == 2 -> RD refinement */ // a->i_mbrd = (i>=6) + (i>=8);


    /* I: Intra part */
    /* Take some shortcuts in intra search if intra is deemed unlikely */
    int b_fast_intra;
    int b_try_pskip;

    /* Luma part */
    int i_satd_i16x16;
    int i_satd_i16x16_dir[7];
    int i_predict16x16;

    int i_satd_i8x8;
    int i_cbp_i8x8_luma;
    int i_satd_i8x8_dir[12][4];
    int i_predict8x8[4];

    int i_satd_i4x4;
    int i_predict4x4[16];

    int i_satd_pcm;

    /* Chroma part */
    int i_satd_i8x8chroma;
    int i_satd_i8x8chroma_dir[4];
    int i_predict8x8chroma;

    /* II: Inter part P/B frame */
    x264_mb_analysis_list_t l0;
    x264_mb_analysis_list_t l1;

    int i_cost16x16bi; /* used the same ref and mv as l0 and l1 (at least for now) */
    int i_cost16x16direct;
    int i_cost8x8bi;
    int i_cost8x8direct[4];
    int i_cost16x8bi;
    int i_cost8x16bi;
    int i_rd16x16bi;
    int i_rd16x16direct;
    int i_rd16x8bi;
    int i_rd8x16bi;
    int i_rd8x8bi;

    int i_mb_partition16x8[2]; /* mb_partition_e */
    int i_mb_partition8x16[2];
    int i_mb_type16x8; /* mb_class_e */
    int i_mb_type8x16;

    int b_direct_available;

} x264_mb_analysis_t;

/* lambda = pow(2,qp/6-2) */
const int x264_lambda_tab[52] = {
   1, 1, 1, 1, 1, 1, 1, 1,  /*  0-7 */
   1, 1, 1, 1,              /*  8-11 */
   1, 1, 1, 1, 2, 2, 2, 2,  /* 12-19 */
   3, 3, 3, 4, 4, 4, 5, 6,  /* 20-27 */
   6, 7, 8, 9,10,11,13,14,  /* 28-35 */
  16,18,20,23,25,29,32,36,  /* 36-43 */
  40,45,51,57,64,72,81,91   /* 44-51 */
};

/* lambda2 = pow(lambda,2) * .9 * 256 */
const int x264_lambda2_tab[52] = {
    14,      18,      22,      28,     36,     45,     57,     72, /*  0 -  7 */
    91,     115,     145,     182,    230,    290,    365,    460, /*  8 - 15 */
   580,     731,     921,    1161,   1462,   1843,   2322,   2925, /* 16 - 23 */
  3686,    4644,    5851,    7372,   9289,  11703,  14745,  18578, /* 24 - 31 */
 23407,   29491,   37156,   46814,  58982,  74313,  93628, 117964, /* 32 - 39 */
148626,  187257,  235929,  297252, 374514, 471859, 594505, 749029, /* 40 - 47 */
943718, 1189010, 1498059, 1887436                                  /* 48 - 51 */
};

/* TODO: calculate CABAC costs */
static const int i_mb_b_cost_table[X264_MBTYPE_MAX] = {
    9, 9, 9, 9, 0, 0, 0, 1, 3, 7, 7, 7, 3, 7, 7, 7, 5, 9, 0
};
static const int i_mb_b16x8_cost_table[17] = {
    0, 0, 0, 0, 0, 0, 0, 0, 5, 7, 7, 7, 5, 7, 9, 9, 9
};
static const int i_sub_mb_b_cost_table[13] = {
    7, 5, 5, 3, 7, 5, 7, 3, 7, 7, 7, 5, 1
};
static const int i_sub_mb_p_cost_table[4] = {
    5, 3, 3, 1
};

static void x264_analyse_update_cache( x264_t *h, x264_mb_analysis_t *a );

static void ALWAYS_INLINE x264_macroblock_store_pic_mb2mb_ih(x264_t *h);  //*****

uint16_t *x264_cost_mv_fpel[52][4];
// [QP][0,1,2][-16...0...16]
uint16_t x264_cost_ref[52][3][33];
int16_t *g_cost_mv[52] = { 0 };
uint16_t *g_x264_cost_mv_fpel[52][4] = { 0 };
/* initialize an array of lambda*nbits for all possible mvs */
static void x264_mb_analyse_load_costs( x264_t *h, x264_mb_analysis_t *a )
{
	static int16_t *p_cost_mv[52] = { 0 };//��Ӧ52���������ӣ��ֱ��ʼ��ʧ�� ָ������p_cost_mv��ÿ��p_cost_mv[i]����һ��short * ָ�룬
	int i, j;

    if( !p_cost_mv[a->i_qp] )//ÿ���������� ��Ӧһ������������������������еĿ���mv����Ҫ��bits����lambda*nbits  lijun
    {
        x264_emms();
        /* could be faster, but isn't called many times */
        /* factor of 4 from qpel, 2 from sign, and 2 because mv can be opposite from mvp */
        g_cost_mv[a->i_qp] = p_cost_mv[a->i_qp] = x264_malloc( (4*4*2048 + 1) * sizeof(int16_t) );//���ȹ�4*4*2048 + 1����2*4*2048Ϊ�ص㣬���߸�2*4*2048��Ӧ�������2048���أ�*4��1/4����mv�߶ȣ�
        p_cost_mv[a->i_qp] += 2*4*2048;//Ӧ����ָ���ƶ����м�λ�ã�
        for( i = 0; i <= 2*4*2048; i++ )//p_cost_mv[20][10]Ӧ��������������Ϊ20������£��˶�ʸ��10����Ӧ��ʧ��...lijun,�Ժ������
        {
            p_cost_mv[a->i_qp][-i] =
            p_cost_mv[a->i_qp][i]  = a->i_lambda * (log2f(i+1)*2 + 0.718f + !!i) + .5f;//(log2f(i+1)*2 + 0.718f + !!i)ָ�����ײ����������bits����log2f��2Ϊ�׵Ķ���
        }
        for( i = 0; i < 3; i++ )
            for( j = 0; j < 33; j++ )
                x264_cost_ref[a->i_qp][i][j] = a->i_lambda * bs_size_te( i, j );//���Ǵ洢�ο�֡��Ҫ��ʧ�棨lambda*��������
    }
    a->p_cost_mv = p_cost_mv[a->i_qp];
    a->p_cost_ref0 = x264_cost_ref[a->i_qp][x264_clip3(h->sh.i_num_ref_idx_l0_active-1,0,2)];
    a->p_cost_ref1 = x264_cost_ref[a->i_qp][x264_clip3(h->sh.i_num_ref_idx_l1_active-1,0,2)];

    /* FIXME is this useful for all me methods? */
    if( h->param.analyse.i_me_method >= X264_ME_ESA && !x264_cost_mv_fpel[a->i_qp][0] )
    {
        for( j=0; j<4; j++ )
        {
            g_x264_cost_mv_fpel[a->i_qp][j] = x264_cost_mv_fpel[a->i_qp][j] = x264_malloc( (4*2048 + 1) * sizeof(int16_t) );
            x264_cost_mv_fpel[a->i_qp][j] += 2*2048;
            for( i = -2*2048; i < 2*2048; i++ )
                x264_cost_mv_fpel[a->i_qp][j][i] = p_cost_mv[a->i_qp][i*4+j];
        }
    }
}

static void x264_mb_analyse_init( x264_t *h, x264_mb_analysis_t *a, int i_qp )
{
    int i = h->param.analyse.i_subpel_refine - (h->sh.i_type == SLICE_TYPE_B);
    /* mbrd == 1 -> RD mode decision */
    /* mbrd == 2 -> RD refinement */
    a->i_mbrd = (i>=6) + (i>=8);
    /* conduct the analysis using this lamda and QP */

    a->i_qp = h->mb.i_qp = i_qp;
    h->mb.i_chroma_qp = h->chroma_qp_table[i_qp];

    a->i_lambda = x264_lambda_tab[i_qp];
    a->i_lambda2 = x264_lambda2_tab[i_qp];
    h->mb.i_me_method = h->param.analyse.i_me_method;
    h->mb.i_subpel_refine = h->param.analyse.i_subpel_refine;
    h->mb.b_chroma_me = h->param.analyse.b_chroma_me && h->sh.i_type == SLICE_TYPE_P
                        && h->mb.i_subpel_refine >= 5;
    h->mb.b_trellis = h->param.analyse.i_trellis > 1 && a->i_mbrd;
    h->mb.b_transform_8x8 = 0;
    h->mb.b_noise_reduction = 0;

    /* I: Intra part */
    a->i_satd_i16x16 =
    a->i_satd_i8x8   =
    a->i_satd_i4x4   =
    a->i_satd_i8x8chroma = COST_MAX;

    /* non-RD PCM decision is inaccurate (as is psy-rd), so don't do it */
    a->i_satd_pcm = !h->mb.i_psy_rd && a->i_mbrd ? ((uint64_t)X264_PCM_COST*a->i_lambda2 + 128) >> 8 : COST_MAX;

    a->b_fast_intra = 0;
    h->mb.i_skip_intra =
        h->mb.b_lossless ? 0 :
        a->i_mbrd ? 2 :
        !h->param.analyse.i_trellis && !h->param.analyse.i_noise_reduction;

    /* II: Inter part P/B frame */
    if( h->sh.i_type != SLICE_TYPE_I )
    {
        int i, j;
        int i_fmv_range = 4 * h->param.analyse.i_mv_range;
        // limit motion search to a slightly smaller range than the theoretical limit,
        // since the search may go a few iterations past its given range
        int i_fpel_border = 5; // umh unconditional radius
        int i_spel_border = 8; // 1.5 for subpel_satd, 1.5 for subpel_rd, 2 for bime, round up

        /* Calculate max allowed MV range */
#define CLIP_FMV(mv) x264_clip3( mv, -i_fmv_range, i_fmv_range-1 )
        h->mb.mv_min[0] = 4*( -16*h->mb.i_mb_x - 24 );
        h->mb.mv_max[0] = 4*( 16*( h->sps->i_mb_width - h->mb.i_mb_x - 1 ) + 24 );
        h->mb.mv_min_spel[0] = CLIP_FMV( h->mb.mv_min[0] );
        h->mb.mv_max_spel[0] = CLIP_FMV( h->mb.mv_max[0] );
        h->mb.mv_min_fpel[0] = (h->mb.mv_min_spel[0]>>2) + i_fpel_border;
        h->mb.mv_max_fpel[0] = (h->mb.mv_max_spel[0]>>2) - i_fpel_border;
        if( h->mb.i_mb_x == 0)
        {
            int mb_y = h->mb.i_mb_y >> h->sh.b_mbaff;
            int mb_height = h->sps->i_mb_height >> h->sh.b_mbaff;
            int thread_mvy_range = i_fmv_range;

            if( h->param.i_threads > 1 )
            {
                int pix_y = (h->mb.i_mb_y | h->mb.b_interlaced) * 16;
                int thresh = pix_y + h->param.analyse.i_mv_range_thread;
                for( i = (h->sh.i_type == SLICE_TYPE_B); i >= 0; i-- )
                {
                    x264_frame_t **fref = i ? h->fref1 : h->fref0;
                    int i_ref = i ? h->i_ref1 : h->i_ref0;
                    for( j=0; j<i_ref; j++ )
                    {
                        x264_frame_cond_wait( fref[j], thresh );
                        thread_mvy_range = X264_MIN( thread_mvy_range, fref[j]->i_lines_completed - pix_y );
                    }
                }
                if( h->param.b_deterministic )
                    thread_mvy_range = h->param.analyse.i_mv_range_thread;
                if( h->mb.b_interlaced )
                    thread_mvy_range >>= 1;
            }

            h->mb.mv_min[1] = 4*( -16*mb_y - 24 );
            h->mb.mv_max[1] = 4*( 16*( mb_height - mb_y - 1 ) + 24 );
            h->mb.mv_min_spel[1] = x264_clip3( h->mb.mv_min[1], X264_MAX(4*(-512+i_spel_border), -i_fmv_range), i_fmv_range );
            h->mb.mv_max_spel[1] = CLIP_FMV( h->mb.mv_max[1] );
            h->mb.mv_max_spel[1] = X264_MIN( h->mb.mv_max_spel[1], thread_mvy_range*4 );
            h->mb.mv_min_fpel[1] = (h->mb.mv_min_spel[1]>>2) + i_fpel_border;
            h->mb.mv_max_fpel[1] = (h->mb.mv_max_spel[1]>>2) - i_fpel_border;
        }
#undef CLIP_FMV

        a->l0.me16x16.cost =
        a->l0.i_rd16x16    =
        a->l0.i_cost8x8    = COST_MAX;

        for( i = 0; i < 4; i++ )
        {
            a->l0.i_cost4x4[i] =
            a->l0.i_cost8x4[i] =
            a->l0.i_cost4x8[i] = COST_MAX;
        }

        a->l0.i_cost16x8   =
        a->l0.i_cost8x16   = COST_MAX;
        if( h->sh.i_type == SLICE_TYPE_B )
        {
            a->l1.me16x16.cost =
            a->l1.i_rd16x16    =
            a->l1.i_cost8x8    = COST_MAX;

            for( i = 0; i < 4; i++ )
            {
                a->l1.i_cost4x4[i] =
                a->l1.i_cost8x4[i] =
                a->l1.i_cost4x8[i] =
                a->i_cost8x8direct[i] = COST_MAX;
            }

            a->l1.i_cost16x8   =
            a->l1.i_cost8x16   =
            a->i_rd16x16bi     =
            a->i_rd16x16direct =
            a->i_rd8x8bi       =
            a->i_rd16x8bi      =
            a->i_rd8x16bi      =
            a->i_cost16x16bi   =
            a->i_cost16x16direct =
            a->i_cost8x8bi     =
            a->i_cost16x8bi    =
            a->i_cost8x16bi    = COST_MAX;
        }

        /* Fast intra decision */
        if( h->mb.i_mb_xy - h->sh.i_first_mb > 4 )
        {
            if(   IS_INTRA( h->mb.i_mb_type_left )
               || IS_INTRA( h->mb.i_mb_type_top )
               || IS_INTRA( h->mb.i_mb_type_topleft )
               || IS_INTRA( h->mb.i_mb_type_topright )
               || (h->sh.i_type == SLICE_TYPE_P && IS_INTRA( h->fref0[0]->mb_type[h->mb.i_mb_xy] ))
               || (h->mb.i_mb_xy - h->sh.i_first_mb < 3*(h->stat.frame.i_mb_count[I_4x4] + h->stat.frame.i_mb_count[I_8x8] + h->stat.frame.i_mb_count[I_16x16])) )
            { /* intra is likely */ }
            else
            {
                a->b_fast_intra = 1;
            }
        }
        h->mb.b_skip_mc = 0;
    }
}



/*
 * Handle intra mb
 */
/* Max = 4 */
static void predict_16x16_mode_available( unsigned int i_neighbour, int *mode, int *pi_count )
{
    if( i_neighbour & MB_TOPLEFT )
    {
        /* top and left available */
        *mode++ = I_PRED_16x16_V;
        *mode++ = I_PRED_16x16_H;
        *mode++ = I_PRED_16x16_DC;
        *mode++ = I_PRED_16x16_P;
        *pi_count = 4;
    }
    else if( i_neighbour & MB_LEFT )
    {
        /* left available*/
        *mode++ = I_PRED_16x16_DC_LEFT;
        *mode++ = I_PRED_16x16_H;
        *pi_count = 2;
    }
    else if( i_neighbour & MB_TOP )
    {
        /* top available*/
        *mode++ = I_PRED_16x16_DC_TOP;
        *mode++ = I_PRED_16x16_V;
        *pi_count = 2;
    }
    else
    {
        /* none available */
        *mode = I_PRED_16x16_DC_128;
        *pi_count = 1;
    }
}

/* Max = 4 */
static void predict_8x8chroma_mode_available( unsigned int i_neighbour, int *mode, int *pi_count )
{
    if( i_neighbour & MB_TOPLEFT )
    {
        /* 4��Ԥ�ⷽʽ������ */
        *mode++ = I_PRED_CHROMA_V;
        *mode++ = I_PRED_CHROMA_H;
        *mode++ = I_PRED_CHROMA_DC;
        *mode++ = I_PRED_CHROMA_P;
        *pi_count = 4;
    }
    else if( i_neighbour & MB_LEFT )
    {
        /* ˮƽDC*/
        *mode++ = I_PRED_CHROMA_DC_LEFT;
        *mode++ = I_PRED_CHROMA_H;
        *pi_count = 2;
    }
    else if( i_neighbour & MB_TOP )
    {
        /* ��ֱDC*/
        *mode++ = I_PRED_CHROMA_DC_TOP;
        *mode++ = I_PRED_CHROMA_V;
        *pi_count = 2;
    }
    else
    {
        /* û���ھӿ飬����128ΪĬ��ֵ����DCԤ�� */
        *mode = I_PRED_CHROMA_DC_128;
        *pi_count = 1;
    }
}

/* MAX = 9 */
static void predict_4x4_mode_available( unsigned int i_neighbour,
                                        int *mode, int *pi_count )
{
    int b_l = i_neighbour & MB_LEFT;
    int b_t = i_neighbour & MB_TOP;

    if( b_l && b_t )
    {
        *pi_count = 6;
        *mode++ = I_PRED_4x4_DC;
        *mode++ = I_PRED_4x4_H;
        *mode++ = I_PRED_4x4_V;
        *mode++ = I_PRED_4x4_DDL;
        if( i_neighbour & MB_TOPLEFT )
        {
            *mode++ = I_PRED_4x4_DDR;
            *mode++ = I_PRED_4x4_VR;
            *mode++ = I_PRED_4x4_HD;
            *pi_count += 3;
        }
        *mode++ = I_PRED_4x4_VL;
        *mode++ = I_PRED_4x4_HU;
    }
    else if( b_l )
    {
        *mode++ = I_PRED_4x4_DC_LEFT;
        *mode++ = I_PRED_4x4_H;
        *mode++ = I_PRED_4x4_HU;
        *pi_count = 3;
    }
    else if( b_t )
    {
        *mode++ = I_PRED_4x4_DC_TOP;
        *mode++ = I_PRED_4x4_V;
        *mode++ = I_PRED_4x4_DDL;
        *mode++ = I_PRED_4x4_VL;
        *pi_count = 4;
    }
    else
    {
        *mode++ = I_PRED_4x4_DC_128;
        *pi_count = 1;
    }
}

/* For trellis=2, we need to do this for both sizes of DCT, for trellis=1 we only need to use it on the chosen mode. */
static void inline x264_psy_trellis_init( x264_t *h, int do_both_dct )
{
    DECLARE_ALIGNED_16( int16_t dct8x8[4][8][8] );
    DECLARE_ALIGNED_16( int16_t dct4x4[16][4][4] );
    DECLARE_ALIGNED_16( uint8_t zero[16*FDEC_STRIDE] ) = {0};
    int i;

    if( do_both_dct || h->mb.b_transform_8x8 )
    {
        h->dctf.sub16x16_dct8( dct8x8, h->mb.pic.p_fenc[0], zero );
        for( i = 0; i < 4; i++ )
            h->zigzagf.scan_8x8( h->mb.pic.fenc_dct8[i], dct8x8[i] );
    }
    if( do_both_dct || !h->mb.b_transform_8x8 )
    {
        h->dctf.sub16x16_dct( dct4x4, h->mb.pic.p_fenc[0], zero );
        for( i = 0; i < 16; i++ )
            h->zigzagf.scan_4x4( h->mb.pic.fenc_dct4[i], dct4x4[i] );
    }
}

/* Pre-calculate fenc satd scores for psy RD, minus DC coefficients */
static inline void x264_mb_cache_fenc_satd( x264_t *h )
{
    DECLARE_ALIGNED_16(uint8_t zero[16]) = {0};
    uint8_t *fenc;
    int x, y, satd_sum = 0, sa8d_sum = 0;
    if( h->param.analyse.i_trellis == 2 && h->mb.i_psy_trellis )
        x264_psy_trellis_init( h, h->param.analyse.b_transform_8x8 );
    if( !h->mb.i_psy_rd )
        return;
    for( y = 0; y < 4; y++ )
        for( x = 0; x < 4; x++ )
        {
            fenc = h->mb.pic.p_fenc[0]+x*4+y*4*FENC_STRIDE;
            h->mb.pic.fenc_satd[y][x] = h->pixf.satd[PIXEL_4x4]( zero, 0, fenc, FENC_STRIDE )
                                      - (h->pixf.sad[PIXEL_4x4]( zero, 0, fenc, FENC_STRIDE )>>1);
            satd_sum += h->mb.pic.fenc_satd[y][x];
        }
    for( y = 0; y < 2; y++ )
        for( x = 0; x < 2; x++ )
        {
            fenc = h->mb.pic.p_fenc[0]+x*8+y*8*FENC_STRIDE;
            h->mb.pic.fenc_sa8d[y][x] = h->pixf.sa8d[PIXEL_8x8]( zero, 0, fenc, FENC_STRIDE )
                                      - (h->pixf.sad[PIXEL_8x8]( zero, 0, fenc, FENC_STRIDE )>>2);
            sa8d_sum += h->mb.pic.fenc_sa8d[y][x];
        }
    h->mb.pic.fenc_satd_sum = satd_sum;
    h->mb.pic.fenc_sa8d_sum = sa8d_sum;
}
/*��chroma����ģʽ���֣������������ŵĻ���ģʽ����Ӧ�Ĵ��ۣ���û�н����ؽ�����*/
static void x264_mb_analyse_intra_chroma( x264_t *h, x264_mb_analysis_t *a )
{
    int i;

    int i_max;
    int predict_mode[4];

    uint8_t *p_dstc[2], *p_srcc[2];

    if( a->i_satd_i8x8chroma < COST_MAX )
        return;

    /* 8x8 prediction selection for chroma */
    p_dstc[0] = h->mb.pic.p_fdec[1];
    p_dstc[1] = h->mb.pic.p_fdec[2];
    p_srcc[0] = h->mb.pic.p_fenc[1];
    p_srcc[1] = h->mb.pic.p_fenc[2];

    predict_8x8chroma_mode_available( h->mb.i_neighbour, predict_mode, &i_max );
    a->i_satd_i8x8chroma = COST_MAX;
	// ����ģʽ������
    if( i_max == 4 && h->pixf.intra_satd_x3_8x8c && h->pixf.mbcmp[0] == h->pixf.satd[0] )
    {
        int satdu[4], satdv[4];
		// V��H��DC
        h->pixf.intra_satd_x3_8x8c( p_srcc[0], p_dstc[0], satdu );
        h->pixf.intra_satd_x3_8x8c( p_srcc[1], p_dstc[1], satdv );
        h->predict_8x8c[I_PRED_CHROMA_P]( p_dstc[0] );
        h->predict_8x8c[I_PRED_CHROMA_P]( p_dstc[1] );
        satdu[I_PRED_CHROMA_P] =
            h->pixf.mbcmp[PIXEL_8x8]( p_dstc[0], FDEC_STRIDE, p_srcc[0], FENC_STRIDE );
        satdv[I_PRED_CHROMA_P] =
            h->pixf.mbcmp[PIXEL_8x8]( p_dstc[1], FDEC_STRIDE, p_srcc[1], FENC_STRIDE );

        for( i=0; i<i_max; i++ )
        {
            int i_mode = predict_mode[i];
            int i_satd = satdu[i_mode] + satdv[i_mode]
                       + a->i_lambda * bs_size_ue(i_mode);

            a->i_satd_i8x8chroma_dir[i] = i_satd;
            COPY2_IF_LT( a->i_satd_i8x8chroma, i_satd, a->i_predict8x8chroma, i_mode );
        }
    }
    else
    {
        for( i=0; i<i_max; i++ )
        {
            int i_satd;
            int i_mode = predict_mode[i];

            /* we do the prediction */
            if( h->mb.b_lossless )
                x264_predict_lossless_8x8_chroma( h, i_mode );
            else
            {
                h->predict_8x8c[i_mode]( p_dstc[0] );
                h->predict_8x8c[i_mode]( p_dstc[1] );
            }

            /* we calculate the cost */
            i_satd = h->pixf.mbcmp[PIXEL_8x8]( p_dstc[0], FDEC_STRIDE,
                                               p_srcc[0], FENC_STRIDE ) +
                     h->pixf.mbcmp[PIXEL_8x8]( p_dstc[1], FDEC_STRIDE,
                                               p_srcc[1], FENC_STRIDE ) +
                     a->i_lambda * bs_size_ue( x264_mb_pred_mode8x8c_fix[i_mode] );

            a->i_satd_i8x8chroma_dir[i] = i_satd;
            COPY2_IF_LT( a->i_satd_i8x8chroma, i_satd, a->i_predict8x8chroma, i_mode );
        }
    }
	// �������ŵ�ģʽ
    h->mb.i_chroma_pred_mode = a->i_predict8x8chroma;
}

//֡��Ԥ�����-��16x16��SAD,4��8x8��SAD�ͣ�16��4x4SAD��ѡ�����ŷ�ʽ
static void x264_mb_analyse_intra( x264_t *h, x264_mb_analysis_t *a, int i_satd_inter )
{
    unsigned int flags = h->sh.i_type == SLICE_TYPE_I ? h->param.analyse.intra : h->param.analyse.inter;

    // �����������ָ��
	uint8_t  *p_src = h->mb.pic.p_fenc[0];
	// �����ָ�루�Ա��������Ԥ��󱣴������Ҳ�������յ��ؽ��飨�����ο��飩�ĵ�ַlijun
    uint8_t  *p_dst = h->mb.pic.p_fdec[0];

    int i, idx;
    int i_max;
    int predict_mode[9];
    int b_merged_satd = !!h->pixf.intra_mbcmp_x3_16x16 && !h->mb.b_lossless;

    /*---------------- Try all mode and calculate their score ---------------*/

    /* 16x16 Ԥ��ģʽѡȡ��i_max���ȡ4��,���ǿ�����������û�����أ����еĻ��Ϳ�����4��ģʽ��ֻ����߻��ϱ�������
	��ֻ��������Ԥ��ģʽ�������û�У������Ϊ֡�е�һ����飩����ֻ����һ��Ԥ��ģʽ����Ĭ��Ϊ128��*/
    predict_16x16_mode_available( h->mb.i_neighbour, predict_mode, &i_max );

    if( b_merged_satd && i_max == 4 )
	{// ���4��ģʽ������ʹ�ö��ҿ���ʹ��x3_16x16
		// V��H��DCԤ��
		//intra_mbcmp_x3_16x16�Ǻ���ָ�룬���ն�Ӧ��pixel-a.asm�л�ຯ����
		//void x264_intra_satd_x3_16x16_mmxext( uint8_t *fenc, uint8_t *fdec, int *res )  add lijun
        //������Ԥ�⣬��Ԥ����д��p_dst�У�
		h->pixf.intra_mbcmp_x3_16x16( p_src, p_dst, a->i_satd_i16x16_dir );
		// PԤ��
        h->predict_16x16[I_PRED_16x16_P]( p_dst );
        a->i_satd_i16x16_dir[I_PRED_16x16_P] = h->pixf.mbcmp[PIXEL_16x16]( p_dst, FDEC_STRIDE, p_src, FENC_STRIDE );
        // �������ŵ�Ԥ��ģʽ�ʹ���
		for( i=0; i<4; i++ )
        {
            int cost = a->i_satd_i16x16_dir[i] += a->i_lambda * bs_size_ue(i);
            COPY2_IF_LT( a->i_satd_i16x16, cost, a->i_predict16x16, i );
        }
    }
    else
    {
        for( i = 0; i < i_max; i++ )
        {
            int i_satd;
            int i_mode = predict_mode[i];

            if( h->mb.b_lossless )
                x264_predict_lossless_16x16( h, i_mode );
            else
                h->predict_16x16[i_mode]( p_dst );//���Ԥ��Ŀ�
			// ����������
            i_satd = h->pixf.mbcmp[PIXEL_16x16]( p_dst, FDEC_STRIDE, p_src, FENC_STRIDE ) +
                    a->i_lambda * bs_size_ue( x264_mb_pred_mode16x16_fix[i_mode] );
            COPY2_IF_LT( a->i_satd_i16x16, i_satd, a->i_predict16x16, i_mode );
            a->i_satd_i16x16_dir[i_mode] = i_satd;
        }
    }

    if( h->sh.i_type == SLICE_TYPE_B )/* ����ģʽ��Ҫ�Ĵ洢���� */
        a->i_satd_i16x16 += a->i_lambda * i_mb_b_cost_table[I_16x16];
    if( a->b_fast_intra && a->i_satd_i16x16 > 2*i_satd_inter ) // ���֡�ڱ����֡�����󣬾Ͳ�ʹ��֡�ڱ���
        return;

    /* 8x8 prediction selection */
    if( flags & X264_ANALYSE_I8x8 )
    {
        DECLARE_ALIGNED_16( uint8_t edge[33] );
        x264_pixel_cmp_t sa8d = (h->pixf.mbcmp[0] == h->pixf.satd[0]) ? h->pixf.sa8d[PIXEL_8x8] : h->pixf.mbcmp[PIXEL_8x8];
        int i_satd_thresh = a->i_mbrd ? COST_MAX : X264_MIN( i_satd_inter, a->i_satd_i16x16 );
        int i_cost = 0;
        h->mb.i_cbp_luma = 0;
        b_merged_satd = h->pixf.intra_sa8d_x3_8x8 && h->pixf.mbcmp[0] == h->pixf.satd[0];

        // FIXME some bias like in i4x4?
        if( h->sh.i_type == SLICE_TYPE_B )
            i_cost += a->i_lambda * i_mb_b_cost_table[I_8x8];

        for( idx = 0;; idx++ )
        {
            int x = idx&1;
            int y = idx>>1;
			// (0,0)(1,0),(0,1),(1,1)
            uint8_t *p_src_by = p_src + 8*x + 8*y*FENC_STRIDE;
            uint8_t *p_dst_by = p_dst + 8*x + 8*y*FDEC_STRIDE;
            int i_best = COST_MAX;
            int i_pred_mode = x264_mb_predict_intra4x4_mode( h, 4*idx );

            predict_4x4_mode_available( h->mb.i_neighbour8[idx], predict_mode, &i_max );
            h->predict_8x8_filter( p_dst_by, edge, h->mb.i_neighbour8[idx], ALL_NEIGHBORS );

            if( b_merged_satd && i_max == 9 )
            {
                int satd[9];
				// DC��V��H
                h->pixf.intra_sa8d_x3_8x8( p_src_by, edge, satd );
                satd[i_pred_mode] -= 3 * a->i_lambda;
                for( i=2; i>=0; i-- )
                {
                    int cost = a->i_satd_i8x8_dir[i][idx] = satd[i] + 4 * a->i_lambda;
                    COPY2_IF_LT( i_best, cost, a->i_predict8x8[idx], i );
                }
                i = 3;
            }
            else
                i = 0;

            for( ; i<i_max; i++ )
            {
                int i_satd;
                int i_mode = predict_mode[i];

                if( h->mb.b_lossless )
                    x264_predict_lossless_8x8( h, p_dst_by, idx, i_mode, edge );
                else
                    h->predict_8x8[i_mode]( p_dst_by, edge );

                i_satd = sa8d( p_dst_by, FDEC_STRIDE, p_src_by, FENC_STRIDE )
                       + a->i_lambda * (i_pred_mode == x264_mb_pred_mode4x4_fix(i_mode) ? 1 : 4);
				// ���ŵ�Ԥ��ģʽ������ i_predict8x8[idx]
                COPY2_IF_LT( i_best, i_satd, a->i_predict8x8[idx], i_mode );
                a->i_satd_i8x8_dir[i_mode][idx] = i_satd;
            }
            i_cost += i_best;

            if( idx == 3 || i_cost > i_satd_thresh )// 4���鶼��ɻ��ߴ����Ѿ���������ֵ
                break;

            /* ��Ҫ���������б��룬�Ա��������ڿ�ο� */
            h->predict_8x8[a->i_predict8x8[idx]]( p_dst_by, edge );
            
			x264_mb_encode_i8x8( h, idx, a->i_qp );

			// intra4x4_pred_mode ����ֵ���(4��λ�ö�Ӧ��4��4x4)����Ϊ��8x8,����2*x,2*y��
            x264_macroblock_cache_intra8x8_pred( h, 2*x, 2*y, a->i_predict8x8[idx] );
        }

        if( idx == 3 )
        {
            a->i_satd_i8x8 = i_cost;
            if( h->mb.i_skip_intra )
            {
                h->mc.copy[PIXEL_16x16]( h->mb.pic.i8x8_fdec_buf, 16, p_dst, FDEC_STRIDE, 16 );
                h->mb.pic.i8x8_nnz_buf[0] = *(uint32_t*)&h->mb.cache.non_zero_count[x264_scan8[ 0]];
                h->mb.pic.i8x8_nnz_buf[1] = *(uint32_t*)&h->mb.cache.non_zero_count[x264_scan8[ 2]];
                h->mb.pic.i8x8_nnz_buf[2] = *(uint32_t*)&h->mb.cache.non_zero_count[x264_scan8[ 8]];
                h->mb.pic.i8x8_nnz_buf[3] = *(uint32_t*)&h->mb.cache.non_zero_count[x264_scan8[10]];
                h->mb.pic.i8x8_cbp = h->mb.i_cbp_luma;
                if( h->mb.i_skip_intra == 2 )
                    h->mc.memcpy_aligned( h->mb.pic.i8x8_dct_buf, h->dct.luma8x8, sizeof(h->mb.pic.i8x8_dct_buf) );
            }
        }
        else
        {
            static const uint16_t cost_div_fix8[3] = {1024,512,341};
            a->i_satd_i8x8 = COST_MAX;
            i_cost = (i_cost * cost_div_fix8[idx]) >> 8;
        }
        if( X264_MIN(i_cost, a->i_satd_i16x16) > i_satd_inter*(5+!!a->i_mbrd)/4 )
            return;
    }

    /* 4x4 prediction selection */
    if( flags & X264_ANALYSE_I4x4 )
    {
        int i_cost;
		// ѡ����С�Ĵ�����Ϊ��ֵ�������֮ǰ����С�Ĵ��۶�С�������I_4X4
        int i_satd_thresh = X264_MIN3( i_satd_inter, a->i_satd_i16x16, a->i_satd_i8x8 );
        h->mb.i_cbp_luma = 0;
        b_merged_satd = h->pixf.intra_satd_x3_4x4 && h->pixf.mbcmp[0] == h->pixf.satd[0];
        if( a->i_mbrd )
            i_satd_thresh = i_satd_thresh * (10-a->b_fast_intra)/8;

        i_cost = a->i_lambda * 24;    /* from JVT (SATD0) */
        if( h->sh.i_type == SLICE_TYPE_B )
            i_cost += a->i_lambda * i_mb_b_cost_table[I_4x4];
		//������ֵ���п���ѭ�������С��16��4x4��ʱ��ʹ����˾�ֱ���˳�
        for( idx = 0;; idx++ )
        {
            uint8_t *p_src_by = p_src + block_idx_xy_fenc[idx];//����֡�е�����,block_idx_xy_fenc[]��¼��4x4С����p_fenc�е�ƫ�Ƶ�ַ
            uint8_t *p_dst_by = p_dst + block_idx_xy_fdec[idx];//�ؽ�֡�е�����,block_idx_xy_fdec[]��¼��4x4С����p_fdec�е�ƫ�Ƶ�ַ
            int i_best = COST_MAX;
			// ��������λ�õ�Ԥ�ⷽʽԤ�⵱ǰ�ӿ��Ԥ�ⷽʽ���ο��������ߵģ�
            int i_pred_mode = x264_mb_predict_intra4x4_mode( h, idx );
			//���ݵ�ǰ�ӿ�����ڿ��Ƿ���ڣ����ÿ���ʹ�õ�Ԥ�ⷽʽ
            predict_4x4_mode_available( h->mb.i_neighbour4[idx], predict_mode, &i_max );

            if( (h->mb.i_neighbour4[idx] & (MB_TOPRIGHT|MB_TOP)) == MB_TOP )
                /* emulate missing topright samples */
                *(uint32_t*) &p_dst_by[4 - FDEC_STRIDE] = p_dst_by[3 - FDEC_STRIDE] * 0x01010101U;

            if( b_merged_satd && i_max >= 6 )
            {
                int satd[9];
				// ˮƽ����ֱ��DCԤ������
                h->pixf.intra_satd_x3_4x4( p_src_by, p_dst_by, satd );
                satd[i_pred_mode] -= 3 * a->i_lambda;
                // ����3�ִ��������ŵ�
				for( i=2; i>=0; i-- )
                    COPY2_IF_LT( i_best, satd[i] + 4 * a->i_lambda,
                                 a->i_predict4x4[idx], i );
                i = 3;
            }
            else
                i = 0;
			// ���ű���ʣ�¿��ܵ�ģʽ����֮ǰ�����Ž��бȽϱ���
            for( ; i<i_max; i++ )
            {// ��������Intra4x4֡��ģʽ�����9��
                int i_satd;
                int i_mode = predict_mode[i];
                if( h->mb.b_lossless )
                    x264_predict_lossless_4x4( h, p_dst_by, idx, i_mode );
                else
                    h->predict_4x4[i_mode]( p_dst_by ); // ֡��Ԥ���ຯ��-�洢���ؽ�֡��

				//4*4ģʽ����ʧ��ķ�����add lijun�� p_src_by����֡��p_dst_by�ؽ�֡
                i_satd = h->pixf.mbcmp[PIXEL_4x4]( p_dst_by, FDEC_STRIDE,
                                                   p_src_by, FENC_STRIDE )
                       + a->i_lambda * (i_pred_mode == x264_mb_pred_mode4x4_fix(i_mode) ? 1 : 4);
				// �����ŵ�Ԥ�ⷽʽ���浽i_predict4x4
                COPY2_IF_LT( i_best, i_satd, a->i_predict4x4[idx], i_mode );
            }
            i_cost += i_best; //�ۼӸ���4x4��Ĵ��ۣ��ۼ�ÿ�������С���ۣ�

            if( i_cost > i_satd_thresh || idx == 15 ) // ��ɻ��߳���������ֵ
                break;

            /* ʹ�����ŵķ�ʽ����Ԥ�⣬�Ա��������вο� */
            h->predict_4x4[a->i_predict4x4[idx]]( p_dst_by );
			// �Ե�ǰ�ӿ�����ع���ͬʱҲ�ᱣ�����ϵ����Ҳ�ᱣ���ع�������ֵlijun��
            x264_mb_encode_i4x4( h, idx, a->i_qp );
			//��mode�����intra4x4_pred_mode_cache
            h->mb.cache.intra4x4_pred_mode[x264_scan8[idx]] = a->i_predict4x4[idx];
        }
        if( idx == 15 ) // ������е��ӿ�Ĵ��۶�û�г�����ֵ
        {//�������һ��4x4С�飨һ��16���飩
            a->i_satd_i4x4 = i_cost;
            if( h->mb.i_skip_intra )
            {
				// �ع���鱣�浽i4x4_fdec_buf
                h->mc.copy[PIXEL_16x16]( h->mb.pic.i4x4_fdec_buf, 16, p_dst, FDEC_STRIDE, 16 );
				// nz ���浽i4x4_nnz_buf
                h->mb.pic.i4x4_nnz_buf[0] = *(uint32_t*)&h->mb.cache.non_zero_count[x264_scan8[ 0]];
                h->mb.pic.i4x4_nnz_buf[1] = *(uint32_t*)&h->mb.cache.non_zero_count[x264_scan8[ 2]];
                h->mb.pic.i4x4_nnz_buf[2] = *(uint32_t*)&h->mb.cache.non_zero_count[x264_scan8[ 8]];
                h->mb.pic.i4x4_nnz_buf[3] = *(uint32_t*)&h->mb.cache.non_zero_count[x264_scan8[10]];
                h->mb.pic.i4x4_cbp = h->mb.i_cbp_luma;
                if( h->mb.i_skip_intra == 2 )
                    h->mc.memcpy_aligned( h->mb.pic.i4x4_dct_buf, h->dct.luma4x4, sizeof(h->mb.pic.i4x4_dct_buf) );
            }
        }
        else
            a->i_satd_i4x4 = COST_MAX;
    }
}

static void x264_intra_rd( x264_t *h, x264_mb_analysis_t *a, int i_satd_thresh )
{
    if( a->i_satd_i16x16 <= i_satd_thresh )
    {
        h->mb.i_type = I_16x16;
		// ����cache��ͬʱ���Chroma�������з������������ŵ�ģʽ����Ӧ�Ĵ��۲�δ�����ؽ�������
        x264_analyse_update_cache( h, a );
		// ���������ۣ���û��д�뵽�����������У�����д������ʱ�����У�
		// ͬʱ��chroma �����ؽ�����
        a->i_satd_i16x16 = x264_rd_cost_mb( h, a->i_lambda2 );
    }
    else
        a->i_satd_i16x16 = COST_MAX;

    if( a->i_satd_i4x4 <= i_satd_thresh && a->i_satd_i4x4 < COST_MAX )
    {
        h->mb.i_type = I_4x4;
        x264_analyse_update_cache( h, a );
        a->i_satd_i4x4 = x264_rd_cost_mb( h, a->i_lambda2 );
    }
    else
        a->i_satd_i4x4 = COST_MAX;

    if( a->i_satd_i8x8 <= i_satd_thresh && a->i_satd_i8x8 < COST_MAX )
    {
        h->mb.i_type = I_8x8;
        x264_analyse_update_cache( h, a );
        a->i_satd_i8x8 = x264_rd_cost_mb( h, a->i_lambda2 );
        a->i_cbp_i8x8_luma = h->mb.i_cbp_luma;
    }
    else
        a->i_satd_i8x8 = COST_MAX;
}

static void x264_intra_rd_refine( x264_t *h, x264_mb_analysis_t *a )
{
    uint8_t  *p_src = h->mb.pic.p_fenc[0];
    uint8_t  *p_dst = h->mb.pic.p_fdec[0];

    int i, j, idx, x, y;
    int i_max, i_mode, i_thresh;
    uint64_t i_satd, i_best;
    int i_pred_mode;
    int predict_mode[9];
    h->mb.i_skip_intra = 0;

    if( h->mb.i_type == I_16x16 )
    {
        int old_pred_mode = a->i_predict16x16;
        i_thresh = a->i_satd_i16x16_dir[old_pred_mode] * 9/8;
        i_best = a->i_satd_i16x16;
        predict_16x16_mode_available( h->mb.i_neighbour, predict_mode, &i_max );
        for( i = 0; i < i_max; i++ )
        {
            int i_mode = predict_mode[i];
            if( i_mode == old_pred_mode || a->i_satd_i16x16_dir[i_mode] > i_thresh )
                continue;
            h->mb.i_intra16x16_pred_mode = i_mode;
            i_satd = x264_rd_cost_mb( h, a->i_lambda2 );
            COPY2_IF_LT( i_best, i_satd, a->i_predict16x16, i_mode );
        }
    }

    /* RD selection for chroma prediction */
    predict_8x8chroma_mode_available( h->mb.i_neighbour, predict_mode, &i_max );
    if( i_max > 1 )
    {
        i_thresh = a->i_satd_i8x8chroma * 5/4;

        for( i = j = 0; i < i_max; i++ )
            if( a->i_satd_i8x8chroma_dir[i] < i_thresh &&
                predict_mode[i] != a->i_predict8x8chroma )
            {
                predict_mode[j++] = predict_mode[i];
            }
        i_max = j;

        if( i_max > 0 )
        {
            int i_cbp_chroma_best = h->mb.i_cbp_chroma;
            int i_chroma_lambda = x264_lambda2_tab[h->mb.i_chroma_qp];
            /* the previous thing encoded was x264_intra_rd(), so the pixels and
             * coefs for the current chroma mode are still around, so we only
             * have to recount the bits. */
            i_best = x264_rd_cost_i8x8_chroma( h, i_chroma_lambda, a->i_predict8x8chroma, 0 );
            for( i = 0; i < i_max; i++ )
            {
                i_mode = predict_mode[i];
                if( h->mb.b_lossless )
                    x264_predict_lossless_8x8_chroma( h, i_mode );
                else
                {
                    h->predict_8x8c[i_mode]( h->mb.pic.p_fdec[1] );
                    h->predict_8x8c[i_mode]( h->mb.pic.p_fdec[2] );
                }
                /* if we've already found a mode that needs no residual, then
                 * probably any mode with a residual will be worse.
                 * so avoid dct on the remaining modes to improve speed. */
                i_satd = x264_rd_cost_i8x8_chroma( h, i_chroma_lambda, i_mode, h->mb.i_cbp_chroma != 0x00 );
                COPY3_IF_LT( i_best, i_satd, a->i_predict8x8chroma, i_mode, i_cbp_chroma_best, h->mb.i_cbp_chroma );
            }
            h->mb.i_chroma_pred_mode = a->i_predict8x8chroma;
            h->mb.i_cbp_chroma = i_cbp_chroma_best;
        }
    }

    if( h->mb.i_type == I_4x4 )
    {
        uint32_t pels[4] = {0}; // doesn't need initting, just shuts up a gcc warning
        int i_nnz = 0;
        for( idx = 0; idx < 16; idx++ )
        {
            uint8_t *p_dst_by = p_dst + block_idx_xy_fdec[idx];
            i_best = COST_MAX64;

            i_pred_mode = x264_mb_predict_intra4x4_mode( h, idx );

            predict_4x4_mode_available( h->mb.i_neighbour4[idx], predict_mode, &i_max );

            if( (h->mb.i_neighbour4[idx] & (MB_TOPRIGHT|MB_TOP)) == MB_TOP )
                /* emulate missing topright samples */
                *(uint32_t*) &p_dst_by[4 - FDEC_STRIDE] = p_dst_by[3 - FDEC_STRIDE] * 0x01010101U;

            for( i = 0; i < i_max; i++ )
            {
                i_mode = predict_mode[i];
                if( h->mb.b_lossless )
                    x264_predict_lossless_4x4( h, p_dst_by, idx, i_mode );
                else
                    h->predict_4x4[i_mode]( p_dst_by );
                i_satd = x264_rd_cost_i4x4( h, a->i_lambda2, idx, i_mode );

                if( i_best > i_satd )
                {
                    a->i_predict4x4[idx] = i_mode;
                    i_best = i_satd;
                    pels[0] = *(uint32_t*)(p_dst_by+0*FDEC_STRIDE);
                    pels[1] = *(uint32_t*)(p_dst_by+1*FDEC_STRIDE);
                    pels[2] = *(uint32_t*)(p_dst_by+2*FDEC_STRIDE);
                    pels[3] = *(uint32_t*)(p_dst_by+3*FDEC_STRIDE);
                    i_nnz = h->mb.cache.non_zero_count[x264_scan8[idx]];
                }
            }

            *(uint32_t*)(p_dst_by+0*FDEC_STRIDE) = pels[0];
            *(uint32_t*)(p_dst_by+1*FDEC_STRIDE) = pels[1];
            *(uint32_t*)(p_dst_by+2*FDEC_STRIDE) = pels[2];
            *(uint32_t*)(p_dst_by+3*FDEC_STRIDE) = pels[3];
            h->mb.cache.non_zero_count[x264_scan8[idx]] = i_nnz;

            h->mb.cache.intra4x4_pred_mode[x264_scan8[idx]] = a->i_predict4x4[idx];
        }
    }
    else if( h->mb.i_type == I_8x8 )
    {
        DECLARE_ALIGNED_16( uint8_t edge[33] );
        for( idx = 0; idx < 4; idx++ )
        {
            uint64_t pels_h = 0;
            uint8_t pels_v[7];
            uint16_t i_nnz[2];
            uint8_t *p_src_by;
            uint8_t *p_dst_by;
            int j;
            int cbp_luma_new = 0;
            i_thresh = a->i_satd_i8x8_dir[a->i_predict8x8[idx]][idx] * 11/8;

            i_best = COST_MAX64;
            i_pred_mode = x264_mb_predict_intra4x4_mode( h, 4*idx );
            x = idx&1;
            y = idx>>1;

            p_src_by = p_src + 8*x + 8*y*FENC_STRIDE;
            p_dst_by = p_dst + 8*x + 8*y*FDEC_STRIDE;
            predict_4x4_mode_available( h->mb.i_neighbour8[idx], predict_mode, &i_max );
            x264_predict_8x8_filter( p_dst_by, edge, h->mb.i_neighbour8[idx], ALL_NEIGHBORS );

            for( i = 0; i < i_max; i++ )
            {
                i_mode = predict_mode[i];
                if( a->i_satd_i8x8_dir[i_mode][idx] > i_thresh )
                    continue;
                if( h->mb.b_lossless )
                    x264_predict_lossless_8x8( h, p_dst_by, idx, i_mode, edge );
                else
                    h->predict_8x8[i_mode]( p_dst_by, edge );
                h->mb.i_cbp_luma = a->i_cbp_i8x8_luma;
                i_satd = x264_rd_cost_i8x8( h, a->i_lambda2, idx, i_mode );

                if( i_best > i_satd )
                {
                    a->i_predict8x8[idx] = i_mode;
                    cbp_luma_new = h->mb.i_cbp_luma;
                    i_best = i_satd;

                    pels_h = *(uint64_t*)(p_dst_by+7*FDEC_STRIDE);
                    if( !(idx&1) )
                        for( j=0; j<7; j++ )
                            pels_v[j] = p_dst_by[7+j*FDEC_STRIDE];
                    i_nnz[0] = *(uint16_t*)&h->mb.cache.non_zero_count[x264_scan8[4*idx+0]];
                    i_nnz[1] = *(uint16_t*)&h->mb.cache.non_zero_count[x264_scan8[4*idx+2]];
                }
            }
            a->i_cbp_i8x8_luma = cbp_luma_new;
            *(uint64_t*)(p_dst_by+7*FDEC_STRIDE) = pels_h;
            if( !(idx&1) )
                for( j=0; j<7; j++ )
                    p_dst_by[7+j*FDEC_STRIDE] = pels_v[j];
            *(uint16_t*)&h->mb.cache.non_zero_count[x264_scan8[4*idx+0]] = i_nnz[0];
            *(uint16_t*)&h->mb.cache.non_zero_count[x264_scan8[4*idx+2]] = i_nnz[1];

            x264_macroblock_cache_intra8x8_pred( h, 2*x, 2*y, a->i_predict8x8[idx] );
        }
    }
}

#define LOAD_FENC( m, src, xoff, yoff) \
    (m)->i_stride[0] = h->mb.pic.i_stride[0]; \
    (m)->i_stride[1] = h->mb.pic.i_stride[1]; \
    (m)->p_fenc[0] = &(src)[0][(xoff)+(yoff)*FENC_STRIDE]; \
    (m)->p_fenc[1] = &(src)[1][((xoff)>>1)+((yoff)>>1)*FENC_STRIDE]; \
    (m)->p_fenc[2] = &(src)[2][((xoff)>>1)+((yoff)>>1)*FENC_STRIDE];

//��Ϣ���� �û��˶�����m ��p_fenc_ihָ�� h->mb.pic.p_fenc_ih lijun
#define LOAD_FENC_IH( m, src, xoff, yoff) \
    (m)->p_fenc_ih[0] = &(src)[0][(xoff)+(yoff)*FENC_STRIDE]; \
    (m)->p_fenc_ih[1] = &(src)[1][((xoff)>>1)+((yoff)>>1)*FENC_STRIDE]; \
    (m)->p_fenc_ih[2] = &(src)[2][((xoff)>>1)+((yoff)>>1)*FENC_STRIDE];

#define LOAD_HPELS(m, src, list, ref, xoff, yoff) \
    (m)->p_fref[0] = &(src)[0][(xoff)+(yoff)*(m)->i_stride[0]]; \
    (m)->p_fref[1] = &(src)[1][(xoff)+(yoff)*(m)->i_stride[0]]; \
    (m)->p_fref[2] = &(src)[2][(xoff)+(yoff)*(m)->i_stride[0]]; \
    (m)->p_fref[3] = &(src)[3][(xoff)+(yoff)*(m)->i_stride[0]]; \
    (m)->p_fref[4] = &(src)[4][((xoff)>>1)+((yoff)>>1)*(m)->i_stride[1]]; \
    (m)->p_fref[5] = &(src)[5][((xoff)>>1)+((yoff)>>1)*(m)->i_stride[1]]; \
    (m)->integral = &h->mb.pic.p_integral[list][ref][(xoff)+(yoff)*(m)->i_stride[0]];

#define REF_COST(list, ref) \
    (a->p_cost_ref##list[ref])

static void x264_mb_analyse_inter_p16x16( x264_t *h, x264_mb_analysis_t *a )
{	
	//�˶�������ص���Ϣ
	//����ĳ�ʼ��������Ҫ�ǶԸýṹ�帳ֵ
    x264_me_t m;
    int i_ref, i_mvc;
    DECLARE_ALIGNED_4( int16_t mvc[8][2] );
    int i_halfpel_thresh = INT_MAX;
    int *p_halfpel_thresh = h->mb.pic.i_fref[0]>1 ? &i_halfpel_thresh : NULL;

    /* 16x16 Search on all ref frame */
    m.i_pixel = PIXEL_16x16;//�趨���طֿ��С
    m.p_cost_mv = a->p_cost_mv;//���a->p_cost_mv�Ѿ���ָ��ǰ�������ӵ�mvʸ����ʧ�������ˣ�����(4*4*2048 + 1)�����߸�2*4*2048��a->p_cost_mvָ���м�㣬lijun��m.p_cost_mv-10�����˶�ʸ��Ϊ10ʱ��ʧ�棿
	// �˶����ƽṹ���f_enc ָ��mb.pic.p_fenc
    LOAD_FENC( &m, h->mb.pic.p_fenc, 0, 0 );
	LOAD_FENC_IH(&m, h->mb.pic.p_fenc_ih, 0, 0);

    a->l0.me16x16.cost = INT_MAX;
	// �����еĲο�֡�н����˶�����
    for( i_ref = 0; i_ref < h->mb.pic.i_fref[0]; i_ref++ )
    { // ѭ���������еĲο�֡
        const int i_ref_cost = REF_COST( 0, i_ref );
        i_halfpel_thresh -= i_ref_cost;
        // ��ȡ�ο�֡�Ĵ��ۣ���¼��i_ref_cost
		m.i_ref_cost = i_ref_cost;
        m.i_ref = i_ref;

        /* search with ref */ 
		//���ذ����ص���б�,L0��i_ref������
		//Ӧ���ǻ�òο�֡������ɣ�lijun����������Ժ�(m)->p_fref[0] (m)->p_fref[1].��(m)->p_fref[5]�ֱ�ָ�������ء�ˮƽ�롢��ֱ�롢�Խǰ롢u��v��Ĳο����أ�  
        LOAD_HPELS( &m, h->mb.pic.p_fref[0][i_ref], 0, i_ref, 0, 0 );
        // ���Ԥ����˶�ʸ��MV��ͨ��ȡ��ֵ
		x264_mb_predict_mv_16x16( h, 0, i_ref, m.mvp );///*������Χ�ĺ����˶�ʸ��Ԥ�⵱ǰ�����˶�ʸ��*/
		
		// Ӧ����Ԥ�������˶�ʸ�����浽mvc�ɣ������ݶ���ο��б�list���б��еĸ����ο�֡����ö��mv���浽mvc��
		//This just improves encoder performance, it's not part of the spec
		//mb.mvr�������ÿ���ο�list��ÿ���ο�֡ÿ��16x16�������Ԥ���˶�����   add lijun
        x264_mb_predict_mv_ref16x16( h, 0, i_ref, mvc, &i_mvc );
        // �ؼ����˶����ƣ������ο�֡�������ŵ��˶�ʸ��������m.mv,m.cost,m.cost_mv
		x264_me_search_ref( h, &m, mvc, i_mvc, p_halfpel_thresh );

        /* early termination
         * SSD threshold would probably be better than SATD */
        if( i_ref == 0
            && a->b_try_pskip
            && m.cost-m.cost_mv < 300*a->i_lambda
            &&  abs(m.mv[0]-h->mb.cache.pskip_mv[0])
              + abs(m.mv[1]-h->mb.cache.pskip_mv[1]) <= 1
            && x264_macroblock_probe_pskip( h ))
        {
            h->mb.i_type = P_SKIP;
			// ����cache���˶�ʸ�����ο�֡��
            x264_analyse_update_cache( h, a );
            assert( h->mb.cache.pskip_mv[1] <= h->mb.mv_max_spel[1] || h->param.i_threads == 1 );
            return;
        }

        m.cost += i_ref_cost;
        i_halfpel_thresh += i_ref_cost;

        if( m.cost < a->l0.me16x16.cost )//����ǰ�˶����ƵĽ�����浽�����ṹ��a��l0�б��me16*16�У�add   //*****
            h->mc.memcpy_aligned( &a->l0.me16x16, &m, sizeof(x264_me_t) );

        /* save mv for predicting neighbors *///Ϊ����������ĺ�ѡmv���ο���
        *(uint32_t*)a->l0.mvc[i_ref][0] =
        *(uint32_t*)h->mb.mvr[0][i_ref][h->mb.i_mb_xy] = *(uint32_t*)m.mv;
    }

    x264_macroblock_cache_ref( h, 0, 0, 4, 4, 0, a->l0.me16x16.i_ref );//���浽cache
    assert( a->l0.me16x16.mv[1] <= h->mb.mv_max_spel[1] || h->param.i_threads == 1 );

    h->mb.i_type = P_L0;
    if( a->i_mbrd )//��һ���Ǹ�ʲô�أ�����
    {
        x264_mb_cache_fenc_satd( h );
        if( a->l0.me16x16.i_ref == 0 && *(uint32_t*)a->l0.me16x16.mv == *(uint32_t*)h->mb.cache.pskip_mv )
        {
            h->mb.i_partition = D_16x16;
            x264_macroblock_cache_mv_ptr( h, 0, 0, 4, 4, 0, a->l0.me16x16.mv );//���浽cache
            a->l0.i_rd16x16 = x264_rd_cost_mb( h, a->i_lambda2 );//�������һ��ʵ�ʵı��룬
        }
    }
}

// ih ��������Ƿ���д�ģ�ȥ������ΪP_SKIP�Ĺ���  *****  ���
static void x264_mb_analyse_inter_p16x16_ih(x264_t *h, x264_mb_analysis_t *a)
{
	//�˶�������ص���Ϣ
	//����ĳ�ʼ��������Ҫ�ǶԸýṹ�帳ֵ
	x264_me_t m;
	int i_ref, i_mvc;
	DECLARE_ALIGNED_4(int16_t mvc[8][2]);
	int i_halfpel_thresh = INT_MAX;
	int *p_halfpel_thresh = h->mb.pic.i_fref[0]>1 ? &i_halfpel_thresh : NULL;

	/* 16x16 Search on all ref frame */
	m.i_pixel = PIXEL_16x16;//�趨���طֿ��С
	m.p_cost_mv = a->p_cost_mv;//���a->p_cost_mv�Ѿ���ָ��ǰ�������ӵ�mvʸ����ʧ�������ˣ�����(4*4*2048 + 1)�����߸�2*4*2048��a->p_cost_mvָ���м�㣬lijun��m.p_cost_mv-10�����˶�ʸ��Ϊ10ʱ��ʧ�棿
							   // �˶����ƽṹ���f_enc ָ��mb.pic.p_fenc
	LOAD_FENC(&m, h->mb.pic.p_fenc, 0, 0);
	LOAD_FENC_IH(&m, h->mb.pic.p_fenc_ih, 0, 0);

	a->l0.me16x16.cost = INT_MAX;
	// �����еĲο�֡�н����˶�����
	for (i_ref = 0; i_ref < h->mb.pic.i_fref[0]; i_ref++)
	{ // ѭ���������еĲο�֡
		const int i_ref_cost = REF_COST(0, i_ref);
		i_halfpel_thresh -= i_ref_cost;
		// ��ȡ�ο�֡�Ĵ��ۣ���¼��i_ref_cost
		m.i_ref_cost = i_ref_cost;
		m.i_ref = i_ref;

		/* search with ref */
		//���ذ����ص���б�,L0��i_ref������
		//Ӧ���ǻ�òο�֡������ɣ�lijun����������Ժ�(m)->p_fref[0] (m)->p_fref[1].��(m)->p_fref[5]�ֱ�ָ�������ء�ˮƽ�롢��ֱ�롢�Խǰ롢u��v��Ĳο����أ�  
		LOAD_HPELS(&m, h->mb.pic.p_fref[0][i_ref], 0, i_ref, 0, 0);
		// ���Ԥ����˶�ʸ��MV��ͨ��ȡ��ֵ
		x264_mb_predict_mv_16x16(h, 0, i_ref, m.mvp);///*������Χ�ĺ����˶�ʸ��Ԥ�⵱ǰ�����˶�ʸ��*/

													 // Ӧ����Ԥ�������˶�ʸ�����浽mvc�ɣ������ݶ���ο��б�list���б��еĸ����ο�֡����ö��mv���浽mvc��
													 //This just improves encoder performance, it's not part of the spec
													 //mb.mvr�������ÿ���ο�list��ÿ���ο�֡ÿ��16x16�������Ԥ���˶�����   add lijun
		x264_mb_predict_mv_ref16x16(h, 0, i_ref, mvc, &i_mvc);
		// �ؼ����˶����ƣ������ο�֡�������ŵ��˶�ʸ��������m.mv,m.cost,m.cost_mv
		x264_me_search_ref(h, &m, mvc, i_mvc, p_halfpel_thresh);

		/* early termination
		* SSD threshold would probably be better than SATD */
		/*if (i_ref == 0
			&& a->b_try_pskip
			&& m.cost - m.cost_mv < 300 * a->i_lambda
			&&  abs(m.mv[0] - h->mb.cache.pskip_mv[0])
			+ abs(m.mv[1] - h->mb.cache.pskip_mv[1]) <= 1
			&& x264_macroblock_probe_pskip(h))
		{
			h->mb.i_type = P_SKIP;
			// ����cache���˶�ʸ�����ο�֡��
			x264_analyse_update_cache(h, a);
			assert(h->mb.cache.pskip_mv[1] <= h->mb.mv_max_spel[1] || h->param.i_threads == 1);
			return;
		}*/   // ȥ�������  ih  lijun  ***** 

		m.cost += i_ref_cost;
		i_halfpel_thresh += i_ref_cost;

		if (m.cost < a->l0.me16x16.cost)//����ǰ�˶����ƵĽ�����浽�����ṹ��a��l0�б��me16*16�У�add   //*****
			h->mc.memcpy_aligned(&a->l0.me16x16, &m, sizeof(x264_me_t));

		/* save mv for predicting neighbors *///Ϊ����������ĺ�ѡmv���ο���
		*(uint32_t*)a->l0.mvc[i_ref][0] =
			*(uint32_t*)h->mb.mvr[0][i_ref][h->mb.i_mb_xy] = *(uint32_t*)m.mv;
	}

	x264_macroblock_cache_ref(h, 0, 0, 4, 4, 0, a->l0.me16x16.i_ref);//���浽cache
	assert(a->l0.me16x16.mv[1] <= h->mb.mv_max_spel[1] || h->param.i_threads == 1);

	h->mb.i_type = P_L0;
	if (a->i_mbrd)//��һ���Ǹ�ʲô�أ�����
	{
		x264_mb_cache_fenc_satd(h);
		if (a->l0.me16x16.i_ref == 0 && *(uint32_t*)a->l0.me16x16.mv == *(uint32_t*)h->mb.cache.pskip_mv)
		{
			h->mb.i_partition = D_16x16;
			x264_macroblock_cache_mv_ptr(h, 0, 0, 4, 4, 0, a->l0.me16x16.mv);//���浽cache
			a->l0.i_rd16x16 = x264_rd_cost_mb(h, a->i_lambda2);//�������һ��ʵ�ʵı��룬
		}
	}
}

static void x264_mb_analyse_inter_p8x8_mixed_ref( x264_t *h, x264_mb_analysis_t *a )
{
    x264_me_t m;
    int i_ref;
    uint8_t  **p_fenc = h->mb.pic.p_fenc;
	uint8_t  **p_fenc_ih = h->mb.pic.p_fenc_ih;

    int i_halfpel_thresh = INT_MAX;
    int *p_halfpel_thresh = /*h->mb.pic.i_fref[0]>1 ? &i_halfpel_thresh : */NULL;
    int i;
    int i_maxref = h->mb.pic.i_fref[0]-1;

    h->mb.i_partition = D_8x8;

    /* early termination: if 16x16 chose ref 0, then evalute no refs older
     * than those used by the neighbors */
    if( i_maxref > 0 && a->l0.me16x16.i_ref == 0 &&
        h->mb.i_mb_type_top && h->mb.i_mb_type_left )
    {
        i_maxref = 0;
        i_maxref = X264_MAX( i_maxref, h->mb.cache.ref[0][ X264_SCAN8_0 - 8 - 1 ] );
        i_maxref = X264_MAX( i_maxref, h->mb.cache.ref[0][ X264_SCAN8_0 - 8 + 0 ] );
        i_maxref = X264_MAX( i_maxref, h->mb.cache.ref[0][ X264_SCAN8_0 - 8 + 2 ] );
        i_maxref = X264_MAX( i_maxref, h->mb.cache.ref[0][ X264_SCAN8_0 - 8 + 4 ] );
        i_maxref = X264_MAX( i_maxref, h->mb.cache.ref[0][ X264_SCAN8_0 + 0 - 1 ] );
        i_maxref = X264_MAX( i_maxref, h->mb.cache.ref[0][ X264_SCAN8_0 + 2*8 - 1 ] );
    }

    for( i_ref = 0; i_ref <= i_maxref; i_ref++ )
         *(uint32_t*)a->l0.mvc[i_ref][0] = *(uint32_t*)h->mb.mvr[0][i_ref][h->mb.i_mb_xy];

    for( i = 0; i < 4; i++ )
    {
        x264_me_t *l0m = &a->l0.me8x8[i];
        const int x8 = i%2;
        const int y8 = i/2;

        m.i_pixel = PIXEL_8x8;
        m.p_cost_mv = a->p_cost_mv;

        LOAD_FENC( &m, p_fenc, 8*x8, 8*y8 );
		LOAD_FENC_IH(&m, p_fenc_ih, 8 * x8, 8 * y8);

        l0m->cost = INT_MAX;
        for( i_ref = 0; i_ref <= i_maxref; i_ref++ )
        {
            const int i_ref_cost = REF_COST( 0, i_ref );
            i_halfpel_thresh -= i_ref_cost;
            m.i_ref_cost = i_ref_cost;
            m.i_ref = i_ref;

            LOAD_HPELS( &m, h->mb.pic.p_fref[0][i_ref], 0, i_ref, 8*x8, 8*y8 );
            x264_macroblock_cache_ref( h, 2*x8, 2*y8, 2, 2, 0, i_ref );
            x264_mb_predict_mv( h, 0, 4*i, 2, m.mvp );
            x264_me_search_ref( h, &m, a->l0.mvc[i_ref], i+1, p_halfpel_thresh );

            m.cost += i_ref_cost;
            i_halfpel_thresh += i_ref_cost;
            *(uint32_t*)a->l0.mvc[i_ref][i+1] = *(uint32_t*)m.mv;

            if( m.cost < l0m->cost )
                h->mc.memcpy_aligned( l0m, &m, sizeof(x264_me_t) );
        }
        x264_macroblock_cache_mv_ptr( h, 2*x8, 2*y8, 2, 2, 0, l0m->mv );
        x264_macroblock_cache_ref( h, 2*x8, 2*y8, 2, 2, 0, l0m->i_ref );

        /* mb type cost */
        l0m->cost += a->i_lambda * i_sub_mb_p_cost_table[D_L0_8x8];
    }

    a->l0.i_cost8x8 = a->l0.me8x8[0].cost + a->l0.me8x8[1].cost +
                      a->l0.me8x8[2].cost + a->l0.me8x8[3].cost;
    /* P_8x8 ref0 has no ref cost */
    if( !h->param.b_cabac && !(a->l0.me8x8[0].i_ref | a->l0.me8x8[1].i_ref |
                               a->l0.me8x8[2].i_ref | a->l0.me8x8[3].i_ref) )
        a->l0.i_cost8x8 -= REF_COST( 0, 0 ) * 4;
    h->mb.i_sub_partition[0] = h->mb.i_sub_partition[1] =
    h->mb.i_sub_partition[2] = h->mb.i_sub_partition[3] = D_L0_8x8;
}

static void x264_mb_analyse_inter_p8x8( x264_t *h, x264_mb_analysis_t *a )
{
    const int i_ref = a->l0.me16x16.i_ref;
    const int i_ref_cost = h->param.b_cabac || i_ref ? REF_COST( 0, i_ref ) : 0;
    uint8_t  **p_fref = h->mb.pic.p_fref[0][i_ref]; //��ǰ16*16�飬uint8_t *p_fref[2][32][4 + 2];ָ��ο�֡���أ������б�32��֡��6������
    uint8_t  **p_fenc = h->mb.pic.p_fenc;//��ǰ16*16��ı�������
	uint8_t  **p_fenc_ih = h->mb.pic.p_fenc_ih;//��ǰ16*16��ı�������

	int i_mvc;
    int16_t (*mvc)[2] = a->l0.mvc[i_ref];
    int i;

    /* XXX Needed for x264_mb_predict_mv */
    h->mb.i_partition = D_8x8;

    i_mvc = 1;
    *(uint32_t*)mvc[0] = *(uint32_t*)a->l0.me16x16.mv;//��ǰ��16*16���˶����Ƶ���ѽ����Ϊ����ĺ�ѡmv

    for( i = 0; i < 4; i++ )
    {
        x264_me_t *m = &a->l0.me8x8[i];
        const int x8 = i%2;
        const int y8 = i/2;

        m->i_pixel = PIXEL_8x8;
        m->p_cost_mv = a->p_cost_mv;
        m->i_ref_cost = i_ref_cost;
        m->i_ref = i_ref;

        LOAD_FENC( m, p_fenc, 8*x8, 8*y8 );//p_fencָ���������
		LOAD_FENC_IH(m, p_fenc_ih, 8 * x8, 8 * y8);//p_fencָ���������

		////p_frefָ��ο������أ�
        LOAD_HPELS( m, p_fref, 0, i_ref, 8*x8, 8*y8 );//�õ�������
        x264_mb_predict_mv( h, 0, 4*i, 2, m->mvp );//�õ���ǰ8*8��Ԥ���mv
        x264_me_search( h, m, mvc, i_mvc );

        x264_macroblock_cache_mv_ptr( h, 2*x8, 2*y8, 2, 2, 0, m->mv );

        *(uint32_t*)mvc[i_mvc] = *(uint32_t*)m->mv;
        i_mvc++;

        /* mb type cost */
        m->cost += i_ref_cost;
        m->cost += a->i_lambda * i_sub_mb_p_cost_table[D_L0_8x8];//�������ʧ�滹��Ҫ�����ӿ���ܵļ������ִ�����bits������lijun
    }

    a->l0.i_cost8x8 = a->l0.me8x8[0].cost + a->l0.me8x8[1].cost +
                      a->l0.me8x8[2].cost + a->l0.me8x8[3].cost;//4��8*8��ʧ��ĺ�Ϊ�ܵ�8*8���ֵ�ʧ��lijun
    /* theoretically this should include 4*ref_cost,
     * but 3 seems a better approximation of cabac. */
    if( h->param.b_cabac )
        a->l0.i_cost8x8 -= i_ref_cost;
    h->mb.i_sub_partition[0] = h->mb.i_sub_partition[1] =
    h->mb.i_sub_partition[2] = h->mb.i_sub_partition[3] = D_L0_8x8;
}

static void x264_mb_analyse_inter_p16x8( x264_t *h, x264_mb_analysis_t *a )
{
    x264_me_t m;
    uint8_t  **p_fenc = h->mb.pic.p_fenc;
	uint8_t  **p_fenc_ih = h->mb.pic.p_fenc_ih;

    DECLARE_ALIGNED_4( int16_t mvc[3][2] );
    int i, j;

    /* XXX Needed for x264_mb_predict_mv */
    h->mb.i_partition = D_16x8;

    for( i = 0; i < 2; i++ )
    {
        x264_me_t *l0m = &a->l0.me16x8[i];
        const int ref8[2] = { a->l0.me8x8[2*i].i_ref, a->l0.me8x8[2*i+1].i_ref };
        const int i_ref8s = ( ref8[0] == ref8[1] ) ? 1 : 2;

        m.i_pixel = PIXEL_16x8;
        m.p_cost_mv = a->p_cost_mv;

        LOAD_FENC( &m, p_fenc, 0, 8*i );
		LOAD_FENC_IH(&m, p_fenc_ih, 0, 8 * i);

        l0m->cost = INT_MAX;
        for( j = 0; j < i_ref8s; j++ )
        {
            const int i_ref = ref8[j];
            const int i_ref_cost = REF_COST( 0, i_ref );
            m.i_ref_cost = i_ref_cost;
            m.i_ref = i_ref;

            /* if we skipped the 16x16 predictor, we wouldn't have to copy anything... */
            *(uint32_t*)mvc[0] = *(uint32_t*)a->l0.mvc[i_ref][0];
            *(uint32_t*)mvc[1] = *(uint32_t*)a->l0.mvc[i_ref][2*i+1];
            *(uint32_t*)mvc[2] = *(uint32_t*)a->l0.mvc[i_ref][2*i+2];

            LOAD_HPELS( &m, h->mb.pic.p_fref[0][i_ref], 0, i_ref, 0, 8*i );
            x264_macroblock_cache_ref( h, 0, 2*i, 4, 2, 0, i_ref );
            x264_mb_predict_mv( h, 0, 8*i, 4, m.mvp );
            x264_me_search( h, &m, mvc, 3 );

            m.cost += i_ref_cost;

            if( m.cost < l0m->cost )
                h->mc.memcpy_aligned( l0m, &m, sizeof(x264_me_t) );
        }
        x264_macroblock_cache_mv_ptr( h, 0, 2*i, 4, 2, 0, l0m->mv );//*****
        x264_macroblock_cache_ref( h, 0, 2*i, 4, 2, 0, l0m->i_ref );
    }

    a->l0.i_cost16x8 = a->l0.me16x8[0].cost + a->l0.me16x8[1].cost;
}

static void x264_mb_analyse_inter_p8x16( x264_t *h, x264_mb_analysis_t *a )
{
    x264_me_t m;
    uint8_t  **p_fenc = h->mb.pic.p_fenc;
	uint8_t  **p_fenc_ih = h->mb.pic.p_fenc_ih;

    DECLARE_ALIGNED_4( int16_t mvc[3][2] );
    int i, j;

    /* XXX Needed for x264_mb_predict_mv */
    h->mb.i_partition = D_8x16;

    for( i = 0; i < 2; i++ )
    {
        x264_me_t *l0m = &a->l0.me8x16[i];
        const int ref8[2] = { a->l0.me8x8[i].i_ref, a->l0.me8x8[i+2].i_ref };
        const int i_ref8s = ( ref8[0] == ref8[1] ) ? 1 : 2;

        m.i_pixel = PIXEL_8x16;
        m.p_cost_mv = a->p_cost_mv;

        LOAD_FENC( &m, p_fenc, 8*i, 0 );
		LOAD_FENC_IH(&m, p_fenc_ih, 8 * i, 0);

        l0m->cost = INT_MAX;
        for( j = 0; j < i_ref8s; j++ )
        {
            const int i_ref = ref8[j];
            const int i_ref_cost = REF_COST( 0, i_ref );
            m.i_ref_cost = i_ref_cost;
            m.i_ref = i_ref;

            *(uint32_t*)mvc[0] = *(uint32_t*)a->l0.mvc[i_ref][0];
            *(uint32_t*)mvc[1] = *(uint32_t*)a->l0.mvc[i_ref][i+1];
            *(uint32_t*)mvc[2] = *(uint32_t*)a->l0.mvc[i_ref][i+3];

            LOAD_HPELS( &m, h->mb.pic.p_fref[0][i_ref], 0, i_ref, 8*i, 0 );
            x264_macroblock_cache_ref( h, 2*i, 0, 2, 4, 0, i_ref );
            x264_mb_predict_mv( h, 0, 4*i, 2, m.mvp );
            x264_me_search( h, &m, mvc, 3 );

            m.cost += i_ref_cost;

            if( m.cost < l0m->cost )
                h->mc.memcpy_aligned( l0m, &m, sizeof(x264_me_t) );
        }
        x264_macroblock_cache_mv_ptr( h, 2*i, 0, 2, 4, 0, l0m->mv );
        x264_macroblock_cache_ref( h, 2*i, 0, 2, 4, 0, l0m->i_ref );
    }

    a->l0.i_cost8x16 = a->l0.me8x16[0].cost + a->l0.me8x16[1].cost;
}

static int x264_mb_analyse_inter_p4x4_chroma( x264_t *h, x264_mb_analysis_t *a, uint8_t **p_fref, int i8x8, int pixel )
{
    DECLARE_ALIGNED_8( uint8_t pix1[16*8] );
    uint8_t *pix2 = pix1+8;
    const int i_stride = h->mb.pic.i_stride[1];
    const int or = 4*(i8x8&1) + 2*(i8x8&2)*i_stride;
    const int oe = 4*(i8x8&1) + 2*(i8x8&2)*FENC_STRIDE;

#define CHROMA4x4MC( width, height, me, x, y ) \
    h->mc.mc_chroma( &pix1[x+y*16], 16, &p_fref[4][or+x+y*i_stride], i_stride, (me).mv[0], (me).mv[1], width, height ); \
    h->mc.mc_chroma( &pix2[x+y*16], 16, &p_fref[5][or+x+y*i_stride], i_stride, (me).mv[0], (me).mv[1], width, height );

    if( pixel == PIXEL_4x4 )
    {
        CHROMA4x4MC( 2,2, a->l0.me4x4[i8x8][0], 0,0 );
        CHROMA4x4MC( 2,2, a->l0.me4x4[i8x8][1], 2,0 );
        CHROMA4x4MC( 2,2, a->l0.me4x4[i8x8][2], 0,2 );
        CHROMA4x4MC( 2,2, a->l0.me4x4[i8x8][3], 2,2 );
    }
    else if( pixel == PIXEL_8x4 )
    {
        CHROMA4x4MC( 4,2, a->l0.me8x4[i8x8][0], 0,0 );
        CHROMA4x4MC( 4,2, a->l0.me8x4[i8x8][1], 0,2 );
    }
    else
    {
        CHROMA4x4MC( 2,4, a->l0.me4x8[i8x8][0], 0,0 );
        CHROMA4x4MC( 2,4, a->l0.me4x8[i8x8][1], 2,0 );
    }

    return h->pixf.mbcmp[PIXEL_4x4]( &h->mb.pic.p_fenc[1][oe], FENC_STRIDE, pix1, 16 )
         + h->pixf.mbcmp[PIXEL_4x4]( &h->mb.pic.p_fenc[2][oe], FENC_STRIDE, pix2, 16 );
}

static void x264_mb_analyse_inter_p4x4( x264_t *h, x264_mb_analysis_t *a, int i8x8 )
{
    uint8_t  **p_fref = h->mb.pic.p_fref[0][a->l0.me8x8[i8x8].i_ref];
    uint8_t  **p_fenc = h->mb.pic.p_fenc;
	uint8_t  **p_fenc_ih = h->mb.pic.p_fenc_ih;

    const int i_ref = a->l0.me8x8[i8x8].i_ref;
    int i4x4;

    /* XXX Needed for x264_mb_predict_mv */
    h->mb.i_partition = D_8x8;

    for( i4x4 = 0; i4x4 < 4; i4x4++ )
    {
        const int idx = 4*i8x8 + i4x4;
        const int x4 = block_idx_x[idx];
        const int y4 = block_idx_y[idx];
        const int i_mvc = (i4x4 == 0);

        x264_me_t *m = &a->l0.me4x4[i8x8][i4x4];

        m->i_pixel = PIXEL_4x4;
        m->p_cost_mv = a->p_cost_mv;

        LOAD_FENC( m, p_fenc, 4*x4, 4*y4 );
		LOAD_FENC_IH(m, p_fenc_ih, 4 * x4, 4 * y4);

        LOAD_HPELS( m, p_fref, 0, i_ref, 4*x4, 4*y4 );

        x264_mb_predict_mv( h, 0, idx, 1, m->mvp );
        x264_me_search( h, m, &a->l0.me8x8[i8x8].mv, i_mvc );

        x264_macroblock_cache_mv_ptr( h, x4, y4, 1, 1, 0, m->mv );
    }
    a->l0.i_cost4x4[i8x8] = a->l0.me4x4[i8x8][0].cost +
                            a->l0.me4x4[i8x8][1].cost +
                            a->l0.me4x4[i8x8][2].cost +
                            a->l0.me4x4[i8x8][3].cost +
                            REF_COST( 0, i_ref ) +
                            a->i_lambda * i_sub_mb_p_cost_table[D_L0_4x4];
    if( h->mb.b_chroma_me )
        a->l0.i_cost4x4[i8x8] += x264_mb_analyse_inter_p4x4_chroma( h, a, p_fref, i8x8, PIXEL_4x4 );
}

static void x264_mb_analyse_inter_p8x4( x264_t *h, x264_mb_analysis_t *a, int i8x8 )
{
    uint8_t  **p_fref = h->mb.pic.p_fref[0][a->l0.me8x8[i8x8].i_ref];
    uint8_t  **p_fenc = h->mb.pic.p_fenc;
	uint8_t  **p_fenc_ih = h->mb.pic.p_fenc_ih;

    const int i_ref = a->l0.me8x8[i8x8].i_ref;
    int i8x4;

    /* XXX Needed for x264_mb_predict_mv */
    h->mb.i_partition = D_8x8;

    for( i8x4 = 0; i8x4 < 2; i8x4++ )
    {
        const int idx = 4*i8x8 + 2*i8x4;
        const int x4 = block_idx_x[idx];
        const int y4 = block_idx_y[idx];
        const int i_mvc = (i8x4 == 0);

        x264_me_t *m = &a->l0.me8x4[i8x8][i8x4];

        m->i_pixel = PIXEL_8x4;
        m->p_cost_mv = a->p_cost_mv;

        LOAD_FENC( m, p_fenc, 4*x4, 4*y4 );
		LOAD_FENC_IH(m, p_fenc_ih, 4 * x4, 4 * y4);

        LOAD_HPELS( m, p_fref, 0, i_ref, 4*x4, 4*y4 );

        x264_mb_predict_mv( h, 0, idx, 2, m->mvp );
        x264_me_search( h, m, &a->l0.me4x4[i8x8][0].mv, i_mvc );

        x264_macroblock_cache_mv_ptr( h, x4, y4, 2, 1, 0, m->mv );
    }
    a->l0.i_cost8x4[i8x8] = a->l0.me8x4[i8x8][0].cost + a->l0.me8x4[i8x8][1].cost +
                            REF_COST( 0, i_ref ) +
                            a->i_lambda * i_sub_mb_p_cost_table[D_L0_8x4];
    if( h->mb.b_chroma_me )
        a->l0.i_cost8x4[i8x8] += x264_mb_analyse_inter_p4x4_chroma( h, a, p_fref, i8x8, PIXEL_8x4 );
}

static void x264_mb_analyse_inter_p4x8( x264_t *h, x264_mb_analysis_t *a, int i8x8 )
{
    uint8_t  **p_fref = h->mb.pic.p_fref[0][a->l0.me8x8[i8x8].i_ref];
    uint8_t  **p_fenc = h->mb.pic.p_fenc;
	uint8_t  **p_fenc_ih = h->mb.pic.p_fenc_ih;

    const int i_ref = a->l0.me8x8[i8x8].i_ref;
    int i4x8;

    /* XXX Needed for x264_mb_predict_mv */
    h->mb.i_partition = D_8x8;

    for( i4x8 = 0; i4x8 < 2; i4x8++ )
    {
        const int idx = 4*i8x8 + i4x8;
        const int x4 = block_idx_x[idx];
        const int y4 = block_idx_y[idx];
        const int i_mvc = (i4x8 == 0);

        x264_me_t *m = &a->l0.me4x8[i8x8][i4x8];

        m->i_pixel = PIXEL_4x8;
        m->p_cost_mv = a->p_cost_mv;

        LOAD_FENC( m, p_fenc, 4*x4, 4*y4 );
		LOAD_FENC_IH(m, p_fenc_ih, 4 * x4, 4 * y4);

        LOAD_HPELS( m, p_fref, 0, i_ref, 4*x4, 4*y4 );

        x264_mb_predict_mv( h, 0, idx, 1, m->mvp );
        x264_me_search( h, m, &a->l0.me4x4[i8x8][0].mv, i_mvc );

        x264_macroblock_cache_mv_ptr( h, x4, y4, 1, 2, 0, m->mv );
    }
    a->l0.i_cost4x8[i8x8] = a->l0.me4x8[i8x8][0].cost + a->l0.me4x8[i8x8][1].cost +
                            REF_COST( 0, i_ref ) +
                            a->i_lambda * i_sub_mb_p_cost_table[D_L0_4x8];
    if( h->mb.b_chroma_me )
        a->l0.i_cost4x8[i8x8] += x264_mb_analyse_inter_p4x4_chroma( h, a, p_fref, i8x8, PIXEL_4x8 );
}

static void x264_mb_analyse_inter_direct( x264_t *h, x264_mb_analysis_t *a )
{
    /* Assumes that fdec still contains the results of
     * x264_mb_predict_mv_direct16x16 and x264_mb_mc */

    uint8_t **p_fenc = h->mb.pic.p_fenc;
    uint8_t **p_fdec = h->mb.pic.p_fdec;
    int i;

    a->i_cost16x16direct = a->i_lambda * i_mb_b_cost_table[B_DIRECT];
    for( i = 0; i < 4; i++ )
    {
        const int x = (i&1)*8;
        const int y = (i>>1)*8;
        a->i_cost16x16direct +=
        a->i_cost8x8direct[i] =
            h->pixf.mbcmp[PIXEL_8x8]( &p_fenc[0][x+y*FENC_STRIDE], FENC_STRIDE, &p_fdec[0][x+y*FDEC_STRIDE], FDEC_STRIDE );

        /* mb type cost */
        a->i_cost8x8direct[i] += a->i_lambda * i_sub_mb_b_cost_table[D_DIRECT_8x8];
    }
}

#define WEIGHTED_AVG( size, pix, stride, src1, stride1, src2, stride2 ) \
{ \
    h->mc.avg[size]( pix, stride, src1, stride1, src2, stride2, h->mb.bipred_weight[a->l0.i_ref][a->l1.i_ref] ); \
}

static void x264_mb_analyse_inter_b16x16( x264_t *h, x264_mb_analysis_t *a )
{
    DECLARE_ALIGNED_16( uint8_t pix0[16*16] );
    DECLARE_ALIGNED_16( uint8_t pix1[16*16] );
    uint8_t *src0, *src1;
    int stride0 = 16, stride1 = 16;

    x264_me_t m;
    int i_ref, i_mvc;
    DECLARE_ALIGNED_4( int16_t mvc[9][2] );
    int i_halfpel_thresh = INT_MAX;
    int *p_halfpel_thresh = h->mb.pic.i_fref[0]>1 ? &i_halfpel_thresh : NULL;

    /* 16x16 Search on all ref frame */
    m.i_pixel = PIXEL_16x16;
    m.p_cost_mv = a->p_cost_mv;
    LOAD_FENC( &m, h->mb.pic.p_fenc, 0, 0 );
	LOAD_FENC_IH(&m, h->mb.pic.p_fenc_ih, 0, 0);


    /* ME for List 0 */
    a->l0.me16x16.cost = INT_MAX;
    for( i_ref = 0; i_ref < h->mb.pic.i_fref[0]; i_ref++ )
    {
        /* search with ref */
        LOAD_HPELS( &m, h->mb.pic.p_fref[0][i_ref], 0, i_ref, 0, 0 );
        x264_mb_predict_mv_16x16( h, 0, i_ref, m.mvp );
        x264_mb_predict_mv_ref16x16( h, 0, i_ref, mvc, &i_mvc );
        x264_me_search_ref( h, &m, mvc, i_mvc, p_halfpel_thresh );

        /* add ref cost */
        m.cost += REF_COST( 0, i_ref );

        if( m.cost < a->l0.me16x16.cost )
        {
            a->l0.i_ref = i_ref;
            h->mc.memcpy_aligned( &a->l0.me16x16, &m, sizeof(x264_me_t) );
        }

        /* save mv for predicting neighbors */
        *(uint32_t*)h->mb.mvr[0][i_ref][h->mb.i_mb_xy] = *(uint32_t*)m.mv;
    }
    /* subtract ref cost, so we don't have to add it for the other MB types */
    a->l0.me16x16.cost -= REF_COST( 0, a->l0.i_ref );

    /* ME for list 1 */
    i_halfpel_thresh = INT_MAX;
    p_halfpel_thresh = h->mb.pic.i_fref[1]>1 ? &i_halfpel_thresh : NULL;
    a->l1.me16x16.cost = INT_MAX;
    for( i_ref = 0; i_ref < h->mb.pic.i_fref[1]; i_ref++ )
    {
        /* search with ref */
        LOAD_HPELS( &m, h->mb.pic.p_fref[1][i_ref], 1, i_ref, 0, 0 );
        x264_mb_predict_mv_16x16( h, 1, i_ref, m.mvp );
        x264_mb_predict_mv_ref16x16( h, 1, i_ref, mvc, &i_mvc );
        x264_me_search_ref( h, &m, mvc, i_mvc, p_halfpel_thresh );

        /* add ref cost */
        m.cost += REF_COST( 1, i_ref );

        if( m.cost < a->l1.me16x16.cost )
        {
            a->l1.i_ref = i_ref;
            h->mc.memcpy_aligned( &a->l1.me16x16, &m, sizeof(x264_me_t) );
        }

        /* save mv for predicting neighbors */
        *(uint32_t*)h->mb.mvr[1][i_ref][h->mb.i_mb_xy] = *(uint32_t*)m.mv;
    }
    /* subtract ref cost, so we don't have to add it for the other MB types */
    a->l1.me16x16.cost -= REF_COST( 1, a->l1.i_ref );

    /* Set global ref, needed for other modes? */
    x264_macroblock_cache_ref( h, 0, 0, 4, 4, 0, a->l0.i_ref );
    x264_macroblock_cache_ref( h, 0, 0, 4, 4, 1, a->l1.i_ref );

    /* get cost of BI mode */
    src0 = h->mc.get_ref( pix0, &stride0,
                           h->mb.pic.p_fref[0][a->l0.i_ref], h->mb.pic.i_stride[0],
                           a->l0.me16x16.mv[0], a->l0.me16x16.mv[1], 16, 16 );
    src1 = h->mc.get_ref( pix1, &stride1,
                           h->mb.pic.p_fref[1][a->l1.i_ref], h->mb.pic.i_stride[0],
                           a->l1.me16x16.mv[0], a->l1.me16x16.mv[1], 16, 16 );

    h->mc.avg[PIXEL_16x16]( pix0, 16, src0, stride0, src1, stride1, h->mb.bipred_weight[a->l0.i_ref][a->l1.i_ref] );

    a->i_cost16x16bi = h->pixf.mbcmp[PIXEL_16x16]( h->mb.pic.p_fenc[0], FENC_STRIDE, pix0, 16 )
                     + REF_COST( 0, a->l0.i_ref )
                     + REF_COST( 1, a->l1.i_ref )
                     + a->l0.me16x16.cost_mv
                     + a->l1.me16x16.cost_mv;

    /* mb type cost */
    a->i_cost16x16bi   += a->i_lambda * i_mb_b_cost_table[B_BI_BI];
    a->l0.me16x16.cost += a->i_lambda * i_mb_b_cost_table[B_L0_L0];
    a->l1.me16x16.cost += a->i_lambda * i_mb_b_cost_table[B_L1_L1];
}

static inline void x264_mb_cache_mv_p8x8( x264_t *h, x264_mb_analysis_t *a, int i )
{
    const int x = 2*(i%2);
    const int y = 2*(i/2);

    switch( h->mb.i_sub_partition[i] )
    {
        case D_L0_8x8:
            x264_macroblock_cache_mv_ptr( h, x, y, 2, 2, 0, a->l0.me8x8[i].mv );
            break;
        case D_L0_8x4:
            x264_macroblock_cache_mv_ptr( h, x, y+0, 2, 1, 0, a->l0.me8x4[i][0].mv );
            x264_macroblock_cache_mv_ptr( h, x, y+1, 2, 1, 0, a->l0.me8x4[i][1].mv );
            break;
        case D_L0_4x8:
            x264_macroblock_cache_mv_ptr( h, x+0, y, 1, 2, 0, a->l0.me4x8[i][0].mv );
            x264_macroblock_cache_mv_ptr( h, x+1, y, 1, 2, 0, a->l0.me4x8[i][1].mv );
            break;
        case D_L0_4x4:
            x264_macroblock_cache_mv_ptr( h, x+0, y+0, 1, 1, 0, a->l0.me4x4[i][0].mv );
            x264_macroblock_cache_mv_ptr( h, x+1, y+0, 1, 1, 0, a->l0.me4x4[i][1].mv );
            x264_macroblock_cache_mv_ptr( h, x+0, y+1, 1, 1, 0, a->l0.me4x4[i][2].mv );
            x264_macroblock_cache_mv_ptr( h, x+1, y+1, 1, 1, 0, a->l0.me4x4[i][3].mv );
            break;
        default:
            x264_log( h, X264_LOG_ERROR, "internal error\n" );
            break;
    }
}

#define CACHE_MV_BI(x,y,dx,dy,me0,me1,part) \
    if( x264_mb_partition_listX_table[0][part] ) \
    { \
        x264_macroblock_cache_ref( h, x,y,dx,dy, 0, a->l0.i_ref ); \
        x264_macroblock_cache_mv_ptr( h, x,y,dx,dy, 0, me0.mv ); \
    } \
    else \
    { \
        x264_macroblock_cache_ref( h, x,y,dx,dy, 0, -1 ); \
        x264_macroblock_cache_mv(  h, x,y,dx,dy, 0, 0 ); \
        if( b_mvd ) \
            x264_macroblock_cache_mvd( h, x,y,dx,dy, 0, 0 ); \
    } \
    if( x264_mb_partition_listX_table[1][part] ) \
    { \
        x264_macroblock_cache_ref( h, x,y,dx,dy, 1, a->l1.i_ref ); \
        x264_macroblock_cache_mv_ptr( h, x,y,dx,dy, 1, me1.mv ); \
    } \
    else \
    { \
        x264_macroblock_cache_ref( h, x,y,dx,dy, 1, -1 ); \
        x264_macroblock_cache_mv(  h, x,y,dx,dy, 1, 0 ); \
        if( b_mvd ) \
            x264_macroblock_cache_mvd( h, x,y,dx,dy, 1, 0 ); \
    }

static inline void x264_mb_cache_mv_b8x8( x264_t *h, x264_mb_analysis_t *a, int i, int b_mvd )
{
    int x = (i%2)*2;
    int y = (i/2)*2;
    if( h->mb.i_sub_partition[i] == D_DIRECT_8x8 )
    {
        x264_mb_load_mv_direct8x8( h, i );
        if( b_mvd )
        {
            x264_macroblock_cache_mvd(  h, x, y, 2, 2, 0, 0 );
            x264_macroblock_cache_mvd(  h, x, y, 2, 2, 1, 0 );
            x264_macroblock_cache_skip( h, x, y, 2, 2, 1 );
        }
    }
    else
    {
        CACHE_MV_BI( x, y, 2, 2, a->l0.me8x8[i], a->l1.me8x8[i], h->mb.i_sub_partition[i] );
    }
}
static inline void x264_mb_cache_mv_b16x8( x264_t *h, x264_mb_analysis_t *a, int i, int b_mvd )
{
    CACHE_MV_BI( 0, 2*i, 4, 2, a->l0.me16x8[i], a->l1.me16x8[i], a->i_mb_partition16x8[i] );
}
static inline void x264_mb_cache_mv_b8x16( x264_t *h, x264_mb_analysis_t *a, int i, int b_mvd )
{
    CACHE_MV_BI( 2*i, 0, 2, 4, a->l0.me8x16[i], a->l1.me8x16[i], a->i_mb_partition8x16[i] );
}
#undef CACHE_MV_BI

static void x264_mb_analyse_inter_b8x8( x264_t *h, x264_mb_analysis_t *a )
{
    uint8_t **p_fref[2] =
        { h->mb.pic.p_fref[0][a->l0.i_ref],
          h->mb.pic.p_fref[1][a->l1.i_ref] };
    DECLARE_ALIGNED_8( uint8_t pix[2][8*8] );
    int i, l;

    /* XXX Needed for x264_mb_predict_mv */
    h->mb.i_partition = D_8x8;

    a->i_cost8x8bi = 0;

    for( i = 0; i < 4; i++ )
    {
        const int x8 = i%2;
        const int y8 = i/2;
        int i_part_cost;
        int i_part_cost_bi = 0;
        int stride[2] = {8,8};
        uint8_t *src[2];

        for( l = 0; l < 2; l++ )
        {
            x264_mb_analysis_list_t *lX = l ? &a->l1 : &a->l0;
            x264_me_t *m = &lX->me8x8[i];

            m->i_pixel = PIXEL_8x8;
            m->p_cost_mv = a->p_cost_mv;

            LOAD_FENC( m, h->mb.pic.p_fenc, 8*x8, 8*y8 );
			LOAD_FENC_IH(m, h->mb.pic.p_fenc_ih, 8 * x8, 8 * y8);

            LOAD_HPELS( m, p_fref[l], l, lX->i_ref, 8*x8, 8*y8 );

            x264_mb_predict_mv( h, l, 4*i, 2, m->mvp );
            x264_me_search( h, m, &lX->me16x16.mv, 1 );

            x264_macroblock_cache_mv_ptr( h, 2*x8, 2*y8, 2, 2, l, m->mv );

            /* BI mode */
            src[l] = h->mc.get_ref( pix[l], &stride[l], m->p_fref, m->i_stride[0],
                                    m->mv[0], m->mv[1], 8, 8 );
            i_part_cost_bi += m->cost_mv;
            /* FIXME: ref cost */
        }
        h->mc.avg[PIXEL_8x8]( pix[0], 8, src[0], stride[0], src[1], stride[1], h->mb.bipred_weight[a->l0.i_ref][a->l1.i_ref] );
        i_part_cost_bi += h->pixf.mbcmp[PIXEL_8x8]( a->l0.me8x8[i].p_fenc[0], FENC_STRIDE, pix[0], 8 )
                        + a->i_lambda * i_sub_mb_b_cost_table[D_BI_8x8];
        a->l0.me8x8[i].cost += a->i_lambda * i_sub_mb_b_cost_table[D_L0_8x8];
        a->l1.me8x8[i].cost += a->i_lambda * i_sub_mb_b_cost_table[D_L1_8x8];

        i_part_cost = a->l0.me8x8[i].cost;
        h->mb.i_sub_partition[i] = D_L0_8x8;
        COPY2_IF_LT( i_part_cost, a->l1.me8x8[i].cost, h->mb.i_sub_partition[i], D_L1_8x8 );
        COPY2_IF_LT( i_part_cost, i_part_cost_bi, h->mb.i_sub_partition[i], D_BI_8x8 );
        COPY2_IF_LT( i_part_cost, a->i_cost8x8direct[i], h->mb.i_sub_partition[i], D_DIRECT_8x8 );
        a->i_cost8x8bi += i_part_cost;

        /* XXX Needed for x264_mb_predict_mv */
        x264_mb_cache_mv_b8x8( h, a, i, 0 );
    }

    /* mb type cost */
    a->i_cost8x8bi += a->i_lambda * i_mb_b_cost_table[B_8x8];
}

static void x264_mb_analyse_inter_b16x8( x264_t *h, x264_mb_analysis_t *a )
{
    uint8_t **p_fref[2] =
        { h->mb.pic.p_fref[0][a->l0.i_ref],
          h->mb.pic.p_fref[1][a->l1.i_ref] };
    DECLARE_ALIGNED_16( uint8_t pix[2][16*8] );
    DECLARE_ALIGNED_4( int16_t mvc[2][2] );
    int i, l;

    h->mb.i_partition = D_16x8;
    a->i_cost16x8bi = 0;

    for( i = 0; i < 2; i++ )
    {
        int i_part_cost;
        int i_part_cost_bi = 0;
        int stride[2] = {16,16};
        uint8_t *src[2];

        /* TODO: check only the list(s) that were used in b8x8? */
        for( l = 0; l < 2; l++ )
        {
            x264_mb_analysis_list_t *lX = l ? &a->l1 : &a->l0;
            x264_me_t *m = &lX->me16x8[i];

            m->i_pixel = PIXEL_16x8;
            m->p_cost_mv = a->p_cost_mv;

            LOAD_FENC( m, h->mb.pic.p_fenc, 0, 8*i );
			LOAD_FENC_IH(m, h->mb.pic.p_fenc_ih, 0, 8 * i);

            LOAD_HPELS( m, p_fref[l], l, lX->i_ref, 0, 8*i );

            *(uint32_t*)mvc[0] = *(uint32_t*)lX->me8x8[2*i].mv;
            *(uint32_t*)mvc[1] = *(uint32_t*)lX->me8x8[2*i+1].mv;

            x264_mb_predict_mv( h, l, 8*i, 2, m->mvp );
            x264_me_search( h, m, mvc, 2 );

            /* BI mode */
            src[l] = h->mc.get_ref( pix[l], &stride[l], m->p_fref, m->i_stride[0],
                                    m->mv[0], m->mv[1], 16, 8 );
            /* FIXME: ref cost */
            i_part_cost_bi += m->cost_mv;
        }
        h->mc.avg[PIXEL_16x8]( pix[0], 16, src[0], stride[0], src[1], stride[1], h->mb.bipred_weight[a->l0.i_ref][a->l1.i_ref] );
        i_part_cost_bi += h->pixf.mbcmp[PIXEL_16x8]( a->l0.me16x8[i].p_fenc[0], FENC_STRIDE, pix[0], 16 );

        i_part_cost = a->l0.me16x8[i].cost;
        a->i_mb_partition16x8[i] = D_L0_8x8; /* not actually 8x8, only the L0 matters */
        if( a->l1.me16x8[i].cost < i_part_cost )
        {
            i_part_cost = a->l1.me16x8[i].cost;
            a->i_mb_partition16x8[i] = D_L1_8x8;
        }
        if( i_part_cost_bi + a->i_lambda * 1 < i_part_cost )
        {
            i_part_cost = i_part_cost_bi;
            a->i_mb_partition16x8[i] = D_BI_8x8;
        }
        a->i_cost16x8bi += i_part_cost;

        x264_mb_cache_mv_b16x8( h, a, i, 0 );
    }

    /* mb type cost */
    a->i_mb_type16x8 = B_L0_L0
        + (a->i_mb_partition16x8[0]>>2) * 3
        + (a->i_mb_partition16x8[1]>>2);
    a->i_cost16x8bi += a->i_lambda * i_mb_b16x8_cost_table[a->i_mb_type16x8];
}

static void x264_mb_analyse_inter_b8x16( x264_t *h, x264_mb_analysis_t *a )
{
    uint8_t **p_fref[2] =
        { h->mb.pic.p_fref[0][a->l0.i_ref],
          h->mb.pic.p_fref[1][a->l1.i_ref] };
    DECLARE_ALIGNED_8( uint8_t pix[2][8*16] );
    DECLARE_ALIGNED_4( int16_t mvc[2][2] );
    int i, l;

    h->mb.i_partition = D_8x16;
    a->i_cost8x16bi = 0;

    for( i = 0; i < 2; i++ )
    {
        int i_part_cost;
        int i_part_cost_bi = 0;
        int stride[2] = {8,8};
        uint8_t *src[2];

        for( l = 0; l < 2; l++ )
        {
            x264_mb_analysis_list_t *lX = l ? &a->l1 : &a->l0;
            x264_me_t *m = &lX->me8x16[i];

            m->i_pixel = PIXEL_8x16;
            m->p_cost_mv = a->p_cost_mv;

            LOAD_FENC( m, h->mb.pic.p_fenc, 8*i, 0 );
			LOAD_FENC_IH(m, h->mb.pic.p_fenc_ih, 8 * i, 0);

            LOAD_HPELS( m, p_fref[l], l, lX->i_ref, 8*i, 0 );

            *(uint32_t*)mvc[0] = *(uint32_t*)lX->me8x8[i].mv;
            *(uint32_t*)mvc[1] = *(uint32_t*)lX->me8x8[i+2].mv;

            x264_mb_predict_mv( h, l, 4*i, 2, m->mvp );
            x264_me_search( h, m, mvc, 2 );

            /* BI mode */
            src[l] = h->mc.get_ref( pix[l], &stride[l], m->p_fref,  m->i_stride[0],
                                    m->mv[0], m->mv[1], 8, 16 );
            /* FIXME: ref cost */
            i_part_cost_bi += m->cost_mv;
        }

        h->mc.avg[PIXEL_8x16]( pix[0], 8, src[0], stride[0], src[1], stride[1], h->mb.bipred_weight[a->l0.i_ref][a->l1.i_ref] );
        i_part_cost_bi += h->pixf.mbcmp[PIXEL_8x16]( a->l0.me8x16[i].p_fenc[0], FENC_STRIDE, pix[0], 8 );

        i_part_cost = a->l0.me8x16[i].cost;
        a->i_mb_partition8x16[i] = D_L0_8x8;
        if( a->l1.me8x16[i].cost < i_part_cost )
        {
            i_part_cost = a->l1.me8x16[i].cost;
            a->i_mb_partition8x16[i] = D_L1_8x8;
        }
        if( i_part_cost_bi + a->i_lambda * 1 < i_part_cost )
        {
            i_part_cost = i_part_cost_bi;
            a->i_mb_partition8x16[i] = D_BI_8x8;
        }
        a->i_cost8x16bi += i_part_cost;

        x264_mb_cache_mv_b8x16( h, a, i, 0 );
    }

    /* mb type cost */
    a->i_mb_type8x16 = B_L0_L0
        + (a->i_mb_partition8x16[0]>>2) * 3
        + (a->i_mb_partition8x16[1]>>2);
    a->i_cost8x16bi += a->i_lambda * i_mb_b16x8_cost_table[a->i_mb_type8x16];
}

static void x264_mb_analyse_p_rd( x264_t *h, x264_mb_analysis_t *a, int i_satd )//���ò�ͬ��i_type��i_partion����ʵ�ʵı�������������ʧ��lijun
{
    int thresh = i_satd * 5/4;

    h->mb.i_type = P_L0;
    if( a->l0.i_rd16x16 == COST_MAX && a->l0.me16x16.cost <= i_satd * 3/2 )
    {
        h->mb.i_partition = D_16x16;
        x264_analyse_update_cache( h, a );
        a->l0.i_rd16x16 = x264_rd_cost_mb( h, a->i_lambda2 );  //*****   ����ͨ��һ����ʵ�����������ʵ���������ʧ�棬�����{��ssd+ssd��+lambda*R}�����������16*16�ֿ飬����ʹ�ÿ��������д����������ʵ����ʧ��lijun
    }
    a->l0.me16x16.cost = a->l0.i_rd16x16;

    if( a->l0.i_cost16x8 <= thresh )
    {
        h->mb.i_partition = D_16x8;
        x264_analyse_update_cache( h, a );
        a->l0.i_cost16x8 = x264_rd_cost_mb( h, a->i_lambda2 );
    }
    else
        a->l0.i_cost16x8 = COST_MAX;

    if( a->l0.i_cost8x16 <= thresh )
    {
        h->mb.i_partition = D_8x16;
        x264_analyse_update_cache( h, a );
        a->l0.i_cost8x16 = x264_rd_cost_mb( h, a->i_lambda2 );
    }
    else
        a->l0.i_cost8x16 = COST_MAX;

    if( a->l0.i_cost8x8 <= thresh )
    {
        h->mb.i_type = P_8x8;
        h->mb.i_partition = D_8x8;
        if( h->param.analyse.inter & X264_ANALYSE_PSUB8x8 )
        {
            int i;
            x264_macroblock_cache_ref( h, 0, 0, 2, 2, 0, a->l0.me8x8[0].i_ref );//��a�и��²ο�֡��h��cache�У�
            x264_macroblock_cache_ref( h, 2, 0, 2, 2, 0, a->l0.me8x8[1].i_ref );
            x264_macroblock_cache_ref( h, 0, 2, 2, 2, 0, a->l0.me8x8[2].i_ref );
            x264_macroblock_cache_ref( h, 2, 2, 2, 2, 0, a->l0.me8x8[3].i_ref );
            /* FIXME: In the 8x8 blocks where RDO isn't run, the NNZ values used for context selection
             * for future blocks are those left over from previous RDO calls. */
            for( i = 0; i < 4; i++ )
            {
                int costs[4] = {a->l0.i_cost4x4[i], a->l0.i_cost8x4[i], a->l0.i_cost4x8[i], a->l0.me8x8[i].cost};
                int thresh = X264_MIN4( costs[0], costs[1], costs[2], costs[3] ) * 5 / 4;
                int subtype, btype = D_L0_8x8;
                uint64_t bcost = COST_MAX64;
                for( subtype = D_L0_4x4; subtype <= D_L0_8x8; subtype++ )
                {
                    uint64_t cost;
                    if( costs[subtype] > thresh || (subtype == D_L0_8x8 && bcost == COST_MAX64) )
                        continue;
                    h->mb.i_sub_partition[i] = subtype;
                    x264_mb_cache_mv_p8x8( h, a, i );
                    cost = x264_rd_cost_part( h, a->i_lambda2, i<<2, PIXEL_8x8 );
                    COPY2_IF_LT( bcost, cost, btype, subtype );
                }
                h->mb.i_sub_partition[i] = btype;
                x264_mb_cache_mv_p8x8( h, a, i );
            }
        }
        else
            x264_analyse_update_cache( h, a );
        a->l0.i_cost8x8 = x264_rd_cost_mb( h, a->i_lambda2 );
    }
    else
        a->l0.i_cost8x8 = COST_MAX;
}

static void x264_mb_analyse_b_rd( x264_t *h, x264_mb_analysis_t *a, int i_satd_inter )
{
    int thresh = i_satd_inter * (17 + (!!h->mb.i_psy_rd))/16;

    if( a->b_direct_available && a->i_rd16x16direct == COST_MAX )
    {
        h->mb.i_type = B_DIRECT;
        /* Assumes direct/skip MC is still in fdec */
        /* Requires b-rdo to be done before intra analysis */
        h->mb.b_skip_mc = 1;
        x264_analyse_update_cache( h, a );
        a->i_rd16x16direct = x264_rd_cost_mb( h, a->i_lambda2 );
        h->mb.b_skip_mc = 0;
    }

    //FIXME not all the update_cache calls are needed
    h->mb.i_partition = D_16x16;
    /* L0 */
    if( a->l0.me16x16.cost <= thresh && a->l0.i_rd16x16 == COST_MAX )
    {
        h->mb.i_type = B_L0_L0;
        x264_analyse_update_cache( h, a );
        a->l0.i_rd16x16 = x264_rd_cost_mb( h, a->i_lambda2 );
    }

    /* L1 */
    if( a->l1.me16x16.cost <= thresh && a->l1.i_rd16x16 == COST_MAX )
    {
        h->mb.i_type = B_L1_L1;
        x264_analyse_update_cache( h, a );
        a->l1.i_rd16x16 = x264_rd_cost_mb( h, a->i_lambda2 );
    }

    /* BI */
    if( a->i_cost16x16bi <= thresh && a->i_rd16x16bi == COST_MAX )
    {
        h->mb.i_type = B_BI_BI;
        x264_analyse_update_cache( h, a );
        a->i_rd16x16bi = x264_rd_cost_mb( h, a->i_lambda2 );
    }

    /* 8x8 */
    if( a->i_cost8x8bi <= thresh && a->i_rd8x8bi == COST_MAX )
    {
        h->mb.i_type = B_8x8;
        h->mb.i_partition = D_8x8;
        x264_analyse_update_cache( h, a );
        a->i_rd8x8bi = x264_rd_cost_mb( h, a->i_lambda2 );
        x264_macroblock_cache_skip( h, 0, 0, 4, 4, 0 );
    }

    /* 16x8 */
    if( a->i_cost16x8bi <= thresh && a->i_rd16x8bi == COST_MAX )
    {
        h->mb.i_type = a->i_mb_type16x8;
        h->mb.i_partition = D_16x8;
        x264_analyse_update_cache( h, a );
        a->i_rd16x8bi = x264_rd_cost_mb( h, a->i_lambda2 );
    }

    /* 8x16 */
    if( a->i_cost8x16bi <= thresh && a->i_rd8x16bi == COST_MAX )
    {
        h->mb.i_type = a->i_mb_type8x16;
        h->mb.i_partition = D_8x16;
        x264_analyse_update_cache( h, a );
        a->i_rd8x16bi = x264_rd_cost_mb( h, a->i_lambda2 );
    }
}

static void x264_refine_bidir( x264_t *h, x264_mb_analysis_t *a )
{
    const int i_biweight = h->mb.bipred_weight[a->l0.i_ref][a->l1.i_ref];
    int i;

    if( IS_INTRA(h->mb.i_type) )
        return;

    switch( h->mb.i_partition )
    {
    case D_16x16:
        if( h->mb.i_type == B_BI_BI )
            x264_me_refine_bidir_satd( h, &a->l0.me16x16, &a->l1.me16x16, i_biweight );
        break;
    case D_16x8:
        for( i=0; i<2; i++ )
            if( a->i_mb_partition16x8[i] == D_BI_8x8 )
                x264_me_refine_bidir_satd( h, &a->l0.me16x8[i], &a->l1.me16x8[i], i_biweight );
        break;
    case D_8x16:
        for( i=0; i<2; i++ )
            if( a->i_mb_partition8x16[i] == D_BI_8x8 )
                x264_me_refine_bidir_satd( h, &a->l0.me8x16[i], &a->l1.me8x16[i], i_biweight );
        break;
    case D_8x8:
        for( i=0; i<4; i++ )
            if( h->mb.i_sub_partition[i] == D_BI_8x8 )
                x264_me_refine_bidir_satd( h, &a->l0.me8x8[i], &a->l1.me8x8[i], i_biweight );
        break;
    }
}

static inline void x264_mb_analyse_transform( x264_t *h )
{
    if( x264_mb_transform_8x8_allowed( h ) && h->param.analyse.b_transform_8x8 && !h->mb.b_lossless )
    {
        int i_cost4, i_cost8;
        /* Only luma MC is really needed, but the full MC is re-used in macroblock_encode. */
        x264_mb_mc( h );

        i_cost8 = h->pixf.sa8d[PIXEL_16x16]( h->mb.pic.p_fenc[0], FENC_STRIDE,
                                             h->mb.pic.p_fdec[0], FDEC_STRIDE );
        i_cost4 = h->pixf.satd[PIXEL_16x16]( h->mb.pic.p_fenc[0], FENC_STRIDE,
                                             h->mb.pic.p_fdec[0], FDEC_STRIDE );

        h->mb.b_transform_8x8 = i_cost8 < i_cost4;
        h->mb.b_skip_mc = 1;
    }
}

static inline void x264_mb_analyse_transform_rd( x264_t *h, x264_mb_analysis_t *a, int *i_satd, int *i_rd )
{
    if( x264_mb_transform_8x8_allowed( h ) && h->param.analyse.b_transform_8x8 )
    {
        int i_rd8;
        x264_analyse_update_cache( h, a );
        h->mb.b_transform_8x8 = !h->mb.b_transform_8x8;
        /* FIXME only luma is needed, but the score for comparison already includes chroma */
        i_rd8 = x264_rd_cost_mb( h, a->i_lambda2 );

        if( *i_rd >= i_rd8 )
        {
            if( *i_rd > 0 )
                *i_satd = (int64_t)(*i_satd) * i_rd8 / *i_rd;
            /* prevent a rare division by zero in estimated intra cost */
            if( *i_satd == 0 )
                *i_satd = 1;

            *i_rd = i_rd8;
        }
        else
            h->mb.b_transform_8x8 = !h->mb.b_transform_8x8;
    }
}

// �Լ���ӵ�,��common/macroblock�е�һ���Ĺ���
static inline void x264_mb_mc_0xywh(x264_t *h, int x, int y, int width, int height)
{
	// ��ȡ�˶�ʸ������cache.mv�е�����
	const int i8 = x264_scan8[0] + x + 8 * y;
	//��ȡ�ο�֡��ǰ��
	const int i_ref = h->mb.cache.ref[0][i8];
	// ��ȡ�˶�ʸ��
	const int mvx = x264_clip3(h->mb.cache.mv[0][i8][0], h->mb.mv_min[0], h->mb.mv_max[0]);
	int       mvy = x264_clip3(h->mb.cache.mv[0][i8][1], h->mb.mv_min[1], h->mb.mv_max[1]);
	// luma�˶����ƣ����������pic.p_fdec
	h->mc.mc_luma(&h->mb.pic.p_fdec[0][4 * y*FDEC_STRIDE + 4 * x], FDEC_STRIDE,
		h->mb.pic.p_fref[0][i_ref], h->mb.pic.i_stride[0],
		mvx + 4 * 4 * x, mvy + 4 * 4 * y, 4 * width, 4 * height);

	// chroma is offset if MCing from a field of opposite parity
	if (h->mb.b_interlaced & i_ref)
		mvy += (h->mb.i_mb_y & 1) * 4 - 2;
	// chroma�˶����ƣ����������pic.p_fdec
	h->mc.mc_chroma(&h->mb.pic.p_fdec[1][2 * y*FDEC_STRIDE + 2 * x], FDEC_STRIDE,
		&h->mb.pic.p_fref[0][i_ref][4][2 * y*h->mb.pic.i_stride[1] + 2 * x], h->mb.pic.i_stride[1],
		mvx, mvy, 2 * width, 2 * height);

	h->mc.mc_chroma(&h->mb.pic.p_fdec[2][2 * y*FDEC_STRIDE + 2 * x], FDEC_STRIDE,
		&h->mb.pic.p_fref[0][i_ref][5][2 * y*h->mb.pic.i_stride[2] + 2 * x], h->mb.pic.i_stride[2],
		mvx, mvy, 2 * width, 2 * height);
}


// �ο�MV_SATD���壬���������ع���(m->p_fenc_ih������fenc��Ľṹ������fdec����)��ο������ʧ��lijun
#define MV_SATD_FDEC_IH(mx, my)\
{\
	const int bw = x264_pixel_size[m->i_pixel].w;\
	const int bh = x264_pixel_size[m->i_pixel].h;\
	const int16_t *p_cost_mvx = m->p_cost_mv - m->mvp[0];\
	const int16_t *p_cost_mvy = m->p_cost_mv - m->mvp[1];\
	const int i_pixel = m->i_pixel;\
	const int b_chroma_me = h->mb.b_chroma_me && i_pixel <= PIXEL_8x8;\
	DECLARE_ALIGNED_16(uint8_t pix[2][32 * 18]);\
	int stride = 16; \
    cost = 0;\
	uint8_t *src = h->mc.get_ref( pix[0], &stride, m->p_fref, m->i_stride[0], mx, my, bw, bh ); \
	cost = h->pixf.mbcmp_unaligned[i_pixel]( m->p_fenc_ih[0], FENC_STRIDE, src, stride ) \
				 + p_cost_mvx[ mx ] + p_cost_mvy[ my ]; \
	if( b_chroma_me)\
	{\
	h->mc.mc_chroma( pix[0], 8, m->p_fref[4], m->i_stride[1], mx, my, bw/2, bh/2 ); \
	cost += h->pixf.mbcmp[i_pixel+3]( m->p_fenc_ih[1], FENC_STRIDE, pix[0], 8 ); \
	h->mc.mc_chroma( pix[0], 8, m->p_fref[5], m->i_stride[1], mx, my, bw/2, bh/2 ); \
	cost += h->pixf.mbcmp[i_pixel+3]( m->p_fenc_ih[2], FENC_STRIDE, pix[0], 8 ); \
	}\
}


//�õ�һ���飨�����ǲ�ͬ��С����ʧ�� lijun
//���룬��ǰ���m���ã�ԭʼmv��Ҫ�޸ĵ�m_x,m_y��ַ��
//�����ԭʼmv��Ҫ�޸ĵ�m_x,m_y��ַ������ʧ��cost_opt
static inline int x264_ih_get_mv_cost(x264_t *h, x264_mb_analysis_t *analysis, x264_me_t *m, int16_t *m_x, int16_t *m_y, int8_t d_mv[][2], int8_t d_mv_1_neighborhood[][2],int mb_xy)
{
	float beta1 = 1.4;//��������2������û�оֲ��������
	float beta2 = 4;//�����������mv��λʱʹ�õı���
	float alpha1 = 1;
//----------------------------------------------------------------------------------------------------------------------------------------------------

	int16_t bmx = m->mv[0], bmy = m->mv[1]; 
	int cost = 0, min_cost = COST_MAX;
	//int16_t m_x = 0, m_y = 0;
	uint8_t b_1_neighbor = 0;//����ó����������������ԭʼmv��1����d_mv�е�ǰ�ĸ�������2����d_mv�еĺ�8���� 0��ʾ2���� 1��ʾ1����
	uint8_t b_error_pos = 0;//���mv�Ƿ��λ��������Ҫ�Ҿֲ�����ȴû�У�Ҫ�Ҿֲ�������ȴû�У��������mvʧ��Ҫ����beta2
	int cost_1_neighborhood[9] = { 0 };//����mv 1������ع���ʧ�棬���һ��Ϊԭʼmv��,lijun
							//1���ҵ�m->mv��Ӧ���ع��飨�ع�����fdec�У���Ҫ���临�Ƶ�һ��fenc�У��������Ե�����ʧ��ĺ�������
							//�ع�һ��fenc�ڴ�ռ䣬��������ع���ͼ��
							//2���ҵ�m->mv��Ӧ�Ĳο��飨������ң��ڲο�֡�У� 
							//3������m->mv +-1 ���� ���ع��飬���ҵ����Ӧ�Ĳο���   4���ж��˶�ʸ���ǲ��Ǿֲ�����			
	x264_analyse_update_cache(h, analysis);//����һ��cache
	x264_macroblock_encode(h);//���ݵ�ǰ���ֽ���һ��ʵ�ʵı��룬�Ὣ��ǰ�����룬�������������������ع���h->mb.pic.fdec�У�
	x264_macroblock_store_pic_mb2mb_ih(h);//ih ��Ϣ���� ����ǰ����ؽ����������ݱ��浽h->mb.pic.fenc_buf_ih[i](ָ��m->p_fenc_ih[])��

	for (int k = 0; k < 9; k++) //�жϾֲ�����
	{
		MV_SATD_FDEC_IH((bmx + d_mv_1_neighborhood[k][0]), (bmy + d_mv_1_neighborhood[k][1]));//��bmx,bmy�Ļ����ϼ��Ϻ�ѡ��9��mv�е�һ������ʧ��
		cost_1_neighborhood[k] = cost;
		COPY1_IF_LT(min_cost, cost);
	}
	m->cost_restructure_ih = cost_1_neighborhood[8];//����ԭʼmv��Ӧ���ع���ʧ��
	if (min_cost < m->cost_restructure_ih)//��ԭʼmv���Ǿֲ�����,������mvҲҪ�Ҿֲ�������
	{
		min_cost = COST_MAX; *m_x = 0; *m_y = 0;
		int ii_best = -1;
		for (int ii = 0; ii < 12; ii++) //12����ѡmv�ȼ����ع�ʧ�棬��������1����ʧ�棬���Ƿ�ֲ������ţ�
		{
			int min_cost_1 = COST_MAX;
			int bmx_1 = bmx + d_mv[ii][0];
			int bmy_1 = bmy + d_mv[ii][1];//�������Ϊ���ģ��ж����ǲ��Ǿֲ�����
			m->mv[0] = bmx_1, m->mv[1] = bmy_1;//�ѷ�����analysis�޸�һ�£�Ϊ�ر���׼��
			x264_analyse_update_cache(h, analysis);//����һ��cache��ʵ����û��Ҫ����ȫ��������ֻ�����������mv�Ĳ��֣��������ɣ�
			x264_macroblock_encode(h);//��bmx_1,bmy_1Ϊ�˶�ʸ������һ�α��룬
			x264_macroblock_store_pic_mb2mb_ih(h);
			int cost_1_neighborhood2[9] = { 0 };//������
			for (int k = 0; k < 9; k++) //1���� ���ƾ���
			{
				MV_SATD_FDEC_IH((bmx_1 + d_mv_1_neighborhood[k][0]), (bmy_1 + d_mv_1_neighborhood[k][1]));
				cost_1_neighborhood2[k] = cost;
				COPY1_IF_LT(min_cost_1, cost);
			}//ѭ��������cost��ֵ��ʵ����d_mv[i]��Ӧ��ʧ�棬��Ϊ��ʱd_mv_1_neighborhood[8]Ϊ0
			if (min_cost_1 != cost)//d_mv[ii]�Ǿֲ������ţ�
			{
				COPY4_IF_LT(min_cost, cost, *m_x, d_mv[ii][0], *m_y, d_mv[ii][1], ii_best, ii);//�������оֲ�����������ʧ����С��
			}
			if (ii == 3)//������ǰ�ĸ�����һ��
			{
				if (min_cost != COST_MAX)//ǰ4���У�1���������Ѿ��оֲ������ŵ��ˣ��Ͳ���������8���ˣ�2��������
				{
					break;
				}
			}
		}
		if (min_cost == COST_MAX)////û�оֲ������ŵģ�ֻ�ܴ�ԭʼmv��1�����ǰ4������һ������Ҫͳ����������ı��������ִ�λ�����ʧ��Ӧ�ø���
		{
			h->info.stat.num_error_pos++;
			b_error_pos = 1;
			b_1_neighbor = 1;//���mv��1�����У�����û�б��־ֲ�����
			min_cost = COST_MAX; *m_x = 0; *m_y = 0;
			for (int k = 0; k < 4; k++)//�ҵ�4����ѡmv����ʧ����С�ģ���Ӧ���޸��˶�ʸ�����ȷ���m_x��m_y,ʧ�����min_cost
			{
				COPY3_IF_LT(min_cost, cost_1_neighborhood[k], *m_x, d_mv_1_neighborhood[k][0], *m_y, d_mv_1_neighborhood[k][1]);
			}
		}
		else//�оֲ������ŵģ�����ʧ����С�Ĵ������min_cost��m_x, m_y��,
		{
			if (ii_best <= 3)
			{
				b_1_neighbor = 1;//��1����
			}
			else
			{
				b_1_neighbor = 0;//��2����
			}
		}
	}
	else //ԭʼmv�Ǿֲ����ţ�������mvҲӦ���Ǿֲ����ŵ�
	{
		min_cost = COST_MAX; *m_x = 0; *m_y = 0;
		int ii_best = -1;//����ĸ�mvѡ����
						 //�ֱ����d_mv�е�12����ѡmv��ÿ����Ҫ�ж��Ƿ�ֲ����ţ�ѡ�����оֲ����ţ��Ӿֲ�������ѡ����ʧ�����С��
		for (int ii = 0; ii < 12; ii++) //12����ѡmv�ȼ���ʧ�棬��������1����ʧ�棬���Ƿ����ţ�
		{
			int min_cost_1 = COST_MAX;
			int bmx_1 = bmx + d_mv[ii][0];
			int bmy_1 = bmy + d_mv[ii][1];//�������Ϊ���ģ��ж����ǲ��Ǿֲ�����
			m->mv[0] = bmx_1, m->mv[1] = bmy_1;//�ѷ�����analysis�޸�һ�£�Ϊ�ر���׼��
			x264_analyse_update_cache(h, analysis);//����һ��cache��ʵ����û��Ҫ����ȫ��������ֻ�����������mv�Ĳ��֣��������ɣ�
			x264_macroblock_encode(h);//��bmx_1,bmy_1Ϊ�˶�ʸ������һ�α��룬
			x264_macroblock_store_pic_mb2mb_ih(h);
			int cost_1_neighborhood2[9] = { 0 };//������
			for (int k = 0; k < 9; k++) //1���� ���ƾ���
			{
				MV_SATD_FDEC_IH((bmx_1 + d_mv_1_neighborhood[k][0]), (bmy_1 + d_mv_1_neighborhood[k][1]));
				cost_1_neighborhood2[k] = cost;
				COPY1_IF_LT(min_cost_1, cost);
			}//ѭ��������cost��ֵ��ʵ����d_mv[i]��Ӧ��ʧ�棬��Ϊ��ʱd_mv_1_neighborhood[8]Ϊ0
			if (min_cost_1 == cost)//d_mv[i]�Ǿֲ����ţ�
			{
				COPY4_IF_LT(min_cost, cost, *m_x, d_mv[ii][0], *m_y, d_mv[ii][1], ii_best, ii);//�������оֲ���������ʧ����С��
			}
			if (ii == 3)//������ǰ�ĸ�����һ��
			{
				if (min_cost != COST_MAX)//ǰ4���У�1���������Ѿ��оֲ����ŵ��ˣ��Ͳ���������8���ˣ�2��������
				{
					break;
				}
			}
		}
		if (min_cost == COST_MAX)////û�оֲ����ŵģ�ֻ�ܴ�ԭʼmv��1�����ǰ4������һ������Ҫͳ����������ı��������ִ�λ�����ʧ��Ӧ�ø���
		{
			h->info.stat.num_error_pos++;
			b_error_pos = 1;
			b_1_neighbor = 1;//���mv��1�����У�����û�б��־ֲ�����
			min_cost = COST_MAX; *m_x = 0; *m_y = 0;
			for (int k = 0; k < 4; k++)//�ҵ�4����ѡmv����ʧ����С�ģ���Ӧ���޸��˶�ʸ�����ȷ���m_x��m_y,ʧ�����min_cost
			{
				COPY3_IF_LT(min_cost, cost_1_neighborhood[k], *m_x, d_mv_1_neighborhood[k][0], *m_y, d_mv_1_neighborhood[k][1]);
			}
		}
		else//�оֲ����ŵģ�����ʧ����С�Ĵ������min_cost��m_x, m_y��,
		{
			if (ii_best <= 3)
			{
				b_1_neighbor = 1;//��1����
			}
			else
			{
				b_1_neighbor = 0;//��2����
			}
		}
	}
	int cost_opt = min_cost > m->cost_restructure_ih ? min_cost - m->cost_restructure_ih : 1;////������ֲ��������йص�ʧ��  �����費��Ҫ��abs������  ��С����Ϊ1
	//int cost_opt =  abs(min_cost - m->cost_restructure_ih) ;//���Ծ���ֵ

	if (b_1_neighbor == 0)//��2����
	{
		cost_opt = beta1 * (float)cost_opt;
		h->info.stat.num_optimal_2_neighbor++;
	}
	else if (b_1_neighbor == 1)//��1����
	{
		h->info.stat.num_optimal_1_neighbor++;
		if (b_error_pos)//��λ�ˣ�ʧ��Ҫ����
		{
			cost_opt = beta2 * (float)cost_opt;
		}
	}

	m->mv[0] = bmx, m->mv[1] = bmy;//��analysis����������ԭʼmv�����ȥ����Ϊ����鲻һ���޸ģ���������ʱ����ԭ����mv
	x264_analyse_update_cache(h, analysis);
	return cost_opt;
}

/*****************************************************************************
 * x264_macroblock_analyse:
 *****************************************************************************/
void x264_macroblock_analyse( x264_t *h )
{
    x264_mb_analysis_t analysis;
    int i_cost = COST_MAX;
    int i;

	static const uint8_t index[8] = { 3, 6, 7, 9, 12, 13, 11, 14 };// ʹ���⼸���Ӻ�����Ԥ��ģʽ���޸ģ�����һ����Կ����16�������ѡ8����
	static const int8_t d_mv[][2] = { {0,-1},{1,0}, {0,1},  {-1,0},//�޸ķ���Ϊ1        // ��ѡ���˶�ʸ������
					                  {-2,1},{-1,2},{1,2},  {2, 1},{2,-1},{1,-2},{-1,-2},{-2,-1} };//�޸ķ���Ϊ3
	static const int8_t d_mv_1_neighborhood[][2] = { { 0,-1 },{ 1,0 },{ 0,1 },{ -1,0 },//��ǰmv��1�������һ��Ϊ��ǰmv�������жϵ�ǰmv�Ƿ�ֲ�����lijun
											{ -1,-1 },{ -1,1 },{ 1,-1 },{ 1, 1 }, {0,0}};
	
	const int mb_xy = h->mb.i_mb_xy;
    //ͨ�����ʿ��Ʒ�������ȡ�����QP
    h->mb.i_qp = x264_ratecontrol_qp( h );
    if( h->param.rc.i_aq_mode )
        x264_adaptive_quant( h );
	
	// �ǳ��ؼ�������if�жϣ�ֻ���������ܱ�֤����ѹ������һ���ķָ�, add by hqtang
	if (h->info.embed_flag &&  !h->info.firstTime) { // ����ǵڶ���ѹ������ֱ��ʹ����һ�ε�i_qp
		h->mb.i_qp = h->info.cache[mb_xy].i_qp;
		//h->mb.i_chroma_qp = h->info.cache[mb_xy].i_qp_c;//���ñ���chroma��qp,����ֱ��ͨ��luma��qp���
	}
	// Analysisģ���ʼ��
    x264_mb_analyse_init( h, &analysis, h->mb.i_qp );

	if (h->info.embed_flag && h->info.firstTime){ // add by hqtang   �ڵ�һ�α�������а�qp�������������ڶ���ʹ��
		h->info.cache[mb_xy].i_qp = h->mb.i_qp;
		//h->info.cache[mb_xy].i_qp_c = h->mb.i_chroma_qp;
	}
	
	 
    /*--------------------------- Do the analysis ---------------------------*/
	//I֡��ֻʹ��֡��Ԥ�⣬�ֱ��������16x16��4�֣���4x4��9�֣�����ģʽ�Ĵ���ֵ��ѡ��������С��ģʽ
	//P֡������֡��ģʽ��֡��ģʽ�� P Slice������Intra����P��飻ͬ��B֡Ҳ֧��Intra��飩��
	//��P֡��ÿһ�ַָ����֡��Ԥ�⣬�õ���ѵ��˶�ʸ�������ƥ��顣
	//֡��Ԥ����̣�ѡ�����ʸ������>�ҵ���ѵ������ص㡪��>�ҵ���ѵĶ���֮һ���ص㡪��>�ҵ���ѵ�1/4���ص�
	//Ȼ��ȡ������С��Ϊ���MV�ͷָʽ
	//����֡��ģʽ��֡��ģʽ��ѡ����۱Ƚ�С�ķ�ʽ���п���û���ҵ��ܺõ�ƥ��飬��ʱ���ֱ��ʹ��֡��Ԥ�������֡��Ԥ�⣩��

    if( h->sh.i_type == SLICE_TYPE_I )
    { //ͨ��һϵ��֡��Ԥ��ģʽ��16x16��4��,4x4��9�֣����۵ļ���ó�������С������ģʽ
        if( analysis.i_mbrd ) //Ԥ����SATD��4x4��8x8������Ԥ��ֵΪ0���м��㣩
            x264_mb_cache_fenc_satd( h );
        //��16��16��SAD,4��8��8��SAD��16��4��4SAD��ѡ�����ŷ�ʽ�����Luma��
        x264_mb_analyse_intra( h, &analysis, COST_MAX );
        if( analysis.i_mbrd ) // Chroma�ᱻ�ؽ�����������ر���
            x264_intra_rd( h, &analysis, COST_MAX );
		//����������洢��analysis�ṹ����
        i_cost = analysis.i_satd_i16x16;
        h->mb.i_type = I_16x16;
        COPY2_IF_LT( i_cost, analysis.i_satd_i4x4, h->mb.i_type, I_4x4 );
        COPY2_IF_LT( i_cost, analysis.i_satd_i8x8, h->mb.i_type, I_8x8 );
        if( analysis.i_satd_pcm < i_cost )//���漫�������ʱ�򣬲��п����õ�PCM
            h->mb.i_type = I_PCM;
        else if( analysis.i_mbrd >= 2 )
            x264_intra_rd_refine( h, &analysis );
    }
    else if( h->sh.i_type == SLICE_TYPE_P )
    {
        int b_skip = 0;
        int i_intra_cost, i_intra_type;

        h->mc.prefetch_ref( h->mb.pic.p_fref[0][0][h->mb.i_mb_x&3], h->mb.pic.i_stride[0], 0 );

        /* Fast P_SKIP detection */
        analysis.b_try_pskip = 0;
        if( h->param.analyse.b_fast_pskip )
        {
            if( h->param.i_threads > 1 && h->mb.cache.pskip_mv[1] > h->mb.mv_max_spel[1] )
                // FIXME don't need to check this if the reference frame is done
                {}
            else if( h->param.analyse.i_subpel_refine >= 3 )
                analysis.b_try_pskip = 1;
            else if( h->mb.i_mb_type_left == P_SKIP ||
                     h->mb.i_mb_type_top == P_SKIP ||
                     h->mb.i_mb_type_topleft == P_SKIP ||
                     h->mb.i_mb_type_topright == P_SKIP )// �����Ƿ���skipģʽ������Χ4����һ��Ϊskip,�ż���Ϊskip�Ŀ�����
                b_skip = x264_macroblock_probe_pskip( h );
        }

        h->mc.prefetch_ref( h->mb.pic.p_fref[0][0][h->mb.i_mb_x&3], h->mb.pic.i_stride[0], 1 ); // �պ�������

        if( b_skip )
        {
            h->mb.i_type = P_SKIP;
            h->mb.i_partition = D_16x16;
            assert( h->mb.cache.pskip_mv[1] <= h->mb.mv_max_spel[1] || h->param.i_threads == 1 );
        }
        else
        {
            unsigned int flags = h->param.analyse.inter;

            int i_type;//������ͣ���P_L0����һ����Σ�������16*16��16*8��8*16��,P_8*8���ڶ�����Σ�ÿ��8*8������ϸ�֣�,P_skipģʽ��lijun
            int i_partition;//�ֿ�����
            int i_thresh16x8;
            int i_satd_inter, i_satd_intra;
			// ��Ҫ��Ϊ�����п��ܵ��˶�ʸ��������ۣ��������ֱ��ʹ��p_cost_mv��a->p_cost_ref0��a->p_cost_ref0���Ѿ��Ƕ�Ӧqp�ģ�
            x264_mb_analyse_load_costs( h, &analysis );
			// �Ƚ���P16x16���֡��Ԥ��ģʽ������
            x264_mb_analyse_inter_p16x16( h, &analysis );

			//����������ݹؼ�,ȷ�����α���ǰ�󻮷�һ��,���滹��һ����   //*****
			if (h->info.embed_flag && !h->info.firstTime)
			{
				// ih *****
				//�����һ�β���P_SKIP����϶������壬���ڶ���ȴԤ��Ϊ��P_SKIP����Ҫǿ��ת�䣬��������x264_mb_analyse_inter_p16x16���趨ΪP_SKIP
				//��û�б���me16x16����Ϣ�����º����޷�����me����������Ҫ����һ��x264_mb_analyse_inter_p16x16���Ա���me16x16��Ϣ������һ�����Ƶĺ�����ȥ��P_SKIP����lijun
				if ((h->info.cache[mb_xy].i_type != P_SKIP) && (h->mb.i_type==P_SKIP))
				{
					x264_mb_analyse_inter_p16x16_ih(h, &analysis);
				}

				h->mb.i_type = h->info.cache[mb_xy].i_type;
				i_type = h->info.cache[mb_xy].i_type;
				/*if (h->mb.i_type == P_SKIP && h->info.cache[mb_xy].i_type != P_SKIP)//��ʱ
				{
					printf("pskip��һ�£�\n");
				}*/
			}

			if (h->mb.i_type == P_SKIP)//�����x264_mb_analyse_inter_p16x16�ķ������ܻὫ�˺�鶨ΪP_SKIP,��ֱ���˳���,���÷�����,
			{
				return;
			}

			// �������ָ�16x16�ĺ�飨���ֳ�4��8x8����з�����
            if( flags & X264_ANALYSE_PSUB16x16 )
            {
                if( h->param.analyse.b_mixed_references )
                    x264_mb_analyse_inter_p8x8_mixed_ref( h, &analysis );
                else
					// P8x8���֡��Ԥ��ģʽ������D_8x8��D_L0_8x8
                    x264_mb_analyse_inter_p8x8( h, &analysis );
            }

            /* Select best inter mode */
            i_type = P_L0;
            i_partition = D_16x16;
            i_cost = analysis.l0.me16x16.cost;

			//���4��8*8��ʧ���������һ��16*16��ʧ��ҪС����ÿ��8*8��������lijun
            if( ( flags & X264_ANALYSE_PSUB16x16 ) && analysis.l0.i_cost8x8 < analysis.l0.me16x16.cost )
            {  
                /* Do sub 8x8 �������ָ�8x8,����ע�ͣ�ʹ�ã��ڶ��α����������ʱ�򣬻���ݵ�һ�εĽ����ǿ��*/
                if( flags & X264_ANALYSE_PSUB8x8 )
                {
					/****************************ԭ����3����ֵ������if���************************************/
					/*������������8x8d�Ľ���ϸ�ֵĻ������������ǲ����е㲻��������*/
					i_type = P_8x8; //���õ�ǰ��õĻ�����4��8*8
					i_partition = D_8x8;//�ӿ�ָ����
					i_cost = analysis.l0.i_cost8x8;
					/****************************ԭ����3����ֵ������if���************************************/
                    for( i = 0; i < 4; i++ )
                    {
						// P4x4���֡��Ԥ��ģʽ��������ÿ��8*8��Ϊ4��4*4
                        x264_mb_analyse_inter_p4x4( h, &analysis, i );
						// ���һ��8x8�Ŀ鱻�ֳ�4��4x4�Ŀ�Ĵ��۸�С����᳢�Էָ��4x8��8x4
                        if( analysis.l0.i_cost4x4[i] < analysis.l0.me8x8[i].cost )
                        {
                            int i_cost8x8 = analysis.l0.i_cost4x4[i];
                            h->mb.i_sub_partition[i] = D_L0_4x4;
							// P8x4���֡��Ԥ��ģʽ����
                            x264_mb_analyse_inter_p8x4( h, &analysis, i );
							// ���8x4��4x4�ָ����
                            COPY2_IF_LT( i_cost8x8, analysis.l0.i_cost8x4[i], h->mb.i_sub_partition[i], D_L0_8x4 );
							// P4x8���֡��Ԥ��ģʽ����
                            x264_mb_analyse_inter_p4x8( h, &analysis, i );
                            COPY2_IF_LT( i_cost8x8, analysis.l0.i_cost4x8[i], h->mb.i_sub_partition[i], D_L0_4x8 );

                            i_cost += i_cost8x8 - analysis.l0.me8x8[i].cost;
                        }
                        x264_mb_cache_mv_p8x8( h, &analysis, i );
                    }
                    analysis.l0.i_cost8x8 = i_cost;
                }
            }

            /* ���8x8�Ĵ���ֵС��16x16,����16x8��8x16 */
            i_thresh16x8 = analysis.l0.me8x8[1].cost_mv + analysis.l0.me8x8[2].cost_mv;
            if( ( flags & X264_ANALYSE_PSUB16x16 ) && analysis.l0.i_cost8x8 < analysis.l0.me16x16.cost + i_thresh16x8 )
            {
				// P16x8���֡��Ԥ��ģʽ����
                x264_mb_analyse_inter_p16x8( h, &analysis );
                COPY3_IF_LT( i_cost, analysis.l0.i_cost16x8, i_type, P_L0, i_partition, D_16x8 );
				// P8x16���֡��Ԥ��ģʽ����
                x264_mb_analyse_inter_p8x16( h, &analysis );
                COPY3_IF_LT( i_cost, analysis.l0.i_cost8x16, i_type, P_L0, i_partition, D_8x16 );
            }

            h->mb.i_partition = i_partition;//���յĺ�黮�ַ�ʽlijun

            /* refine qpel �����ؾ�������*/// 
            //FIXME mb_type costs?
            if( analysis.i_mbrd )
            {
                /* refine later *///���ں��ִ����,
            }
            else if( i_partition == D_16x16 )
            {
                x264_me_refine_qpel( h, &analysis.l0.me16x16 );//�����غ�1/4��������
                i_cost = analysis.l0.me16x16.cost;
            }
            else if( i_partition == D_16x8 )
            {
                x264_me_refine_qpel( h, &analysis.l0.me16x8[0] );//�����غ�1/4��������
                x264_me_refine_qpel( h, &analysis.l0.me16x8[1] );//�����غ�1/4��������
                i_cost = analysis.l0.me16x8[0].cost + analysis.l0.me16x8[1].cost;
            }
            else if( i_partition == D_8x16 )
            {
                x264_me_refine_qpel( h, &analysis.l0.me8x16[0] );//�����غ�1/4��������
                x264_me_refine_qpel( h, &analysis.l0.me8x16[1] );//�����غ�1/4��������
                i_cost = analysis.l0.me8x16[0].cost + analysis.l0.me8x16[1].cost;
            }
            else if( i_partition == D_8x8 )
            {
                int i8x8;
                i_cost = 0;
                for( i8x8 = 0; i8x8 < 4; i8x8++ )
                {
                    switch( h->mb.i_sub_partition[i8x8] )
                    {
                        case D_L0_8x8:
                            x264_me_refine_qpel( h, &analysis.l0.me8x8[i8x8] );//�����غ�1/4��������
                            i_cost += analysis.l0.me8x8[i8x8].cost;
                            break;
                        case D_L0_8x4:
                            x264_me_refine_qpel( h, &analysis.l0.me8x4[i8x8][0] );
                            x264_me_refine_qpel( h, &analysis.l0.me8x4[i8x8][1] );
                            i_cost += analysis.l0.me8x4[i8x8][0].cost +
                                      analysis.l0.me8x4[i8x8][1].cost;
                            break;
                        case D_L0_4x8:
                            x264_me_refine_qpel( h, &analysis.l0.me4x8[i8x8][0] );
                            x264_me_refine_qpel( h, &analysis.l0.me4x8[i8x8][1] );
                            i_cost += analysis.l0.me4x8[i8x8][0].cost +
                                      analysis.l0.me4x8[i8x8][1].cost;
                            break;

                        case D_L0_4x4:
                            x264_me_refine_qpel( h, &analysis.l0.me4x4[i8x8][0] );
                            x264_me_refine_qpel( h, &analysis.l0.me4x4[i8x8][1] );
                            x264_me_refine_qpel( h, &analysis.l0.me4x4[i8x8][2] );
                            x264_me_refine_qpel( h, &analysis.l0.me4x4[i8x8][3] );
                            i_cost += analysis.l0.me4x4[i8x8][0].cost +
                                      analysis.l0.me4x4[i8x8][1].cost +
                                      analysis.l0.me4x4[i8x8][2].cost +
                                      analysis.l0.me4x4[i8x8][3].cost;
                            break;
                        default:
                            x264_log( h, X264_LOG_ERROR, "internal error (!8x8 && !4x4)\n" );
                            break;
                    }
                }
            }

            if( h->mb.b_chroma_me )
            {
				// ���ȶ�chroma ����֡�ڷ���
                x264_mb_analyse_intra_chroma( h, &analysis );
				// ��luma����֡�ڷ�����������۴���i_cost - analysis.i_satd_i8x8chroma֡��ǰ���������ʺ�֡��
                x264_mb_analyse_intra( h, &analysis, i_cost - analysis.i_satd_i8x8chroma );
                analysis.i_satd_i16x16 += analysis.i_satd_i8x8chroma;
                analysis.i_satd_i8x8 += analysis.i_satd_i8x8chroma;
                analysis.i_satd_i4x4 += analysis.i_satd_i8x8chroma;
            }
            else //P Slice��Ҳ������Intra��飬����ҲҪ���з���
			{ 
				x264_mb_analyse_intra( h, &analysis, i_cost ); 
			}

            i_satd_inter = i_cost;
            i_satd_intra = X264_MIN3( analysis.i_satd_i16x16,analysis.i_satd_i8x8,analysis.i_satd_i4x4 );

            if( analysis.i_mbrd)//
            {
				// ���½���һ��ʵ�ʵı��룬���ʱ��õ�������������ʧ���׼ȷ�����㲻ͬ�ָʽ�Ĵ���   ��ʧ���Ż� ����д������
                x264_mb_analyse_p_rd( h, &analysis, X264_MIN(i_satd_inter, i_satd_intra) );
                i_type = P_L0;
                i_partition = D_16x16;//����������Ŀ�ʼ�������D_16x16��D_16x8��D_8x16���п���ѡһ�����ŵģ�
                i_cost = analysis.l0.me16x16.cost;
				// ѡ�����ŵ�
                COPY2_IF_LT( i_cost, analysis.l0.i_cost16x8, i_partition, D_16x8 );
                COPY2_IF_LT( i_cost, analysis.l0.i_cost8x16, i_partition, D_8x16 );
				// �ؼ���if�жϣ����������if���ƣ����п��ܽ�ǰһ��û��ʹ��P_8X8�ķֳ�P_8X8�� �ǻ᲻�Ὣ������P_8x8�ķֳ�P_L0???lijun  
				// �����Ǹ�ifҲ��һ���ĵ���������ϣ����ܱ�֤ǰ������һ����  add by hqtang
				if(h->info.embed_flag && h->info.firstTime||(!h->info.firstTime && h->info.cache[mb_xy].used))//ֻ�Ǽ������if��䣬��������ݱ�������lijun
					COPY3_IF_LT(i_cost, analysis.l0.i_cost8x8, i_partition, D_8x8, i_type, P_8x8);		

                h->mb.i_type = i_type;//����
                h->mb.i_partition = i_partition;
                if( i_cost < COST_MAX )
                    x264_mb_analyse_transform_rd( h, &analysis, &i_satd_inter, &i_cost );//8*8�任ģʽ�����ã�����
                x264_intra_rd( h, &analysis, i_satd_inter * 5/4 );//��������֡�ڵ�ʵ�ʱ��룬�õ�֡��ģʽ�Ĳ�ͬ���ۣ�����д������
            }

			//��ȡ��С��֡��Ԥ����ۺ�type
            i_intra_type = I_16x16;
            i_intra_cost = analysis.i_satd_i16x16;
            COPY2_IF_LT( i_intra_cost, analysis.i_satd_i8x8, i_intra_type, I_8x8 );

			if (h->info.embed_flag && h->info.firstTime || (!h->info.firstTime && h->info.cache[mb_xy].used)) // add by hqtang//ֻ�Ǽ������if��䣬��������ݱ�������lijun
				COPY2_IF_LT(i_intra_cost, analysis.i_satd_i4x4, i_intra_type, I_4x4);

            COPY2_IF_LT( i_intra_cost, analysis.i_satd_pcm, i_intra_type, I_PCM );//��I_PCMģʽ�Ƿ����
        
			//�ؼ�  ���������յ�����ģʽlijun      Ϊ�˲�����֡��Ԥ�⣬ע����仰��ͬʱ��encoder.c 1647�е�ifʧ�ܣ������л�P֡��飩
			//COPY2_IF_LT( i_cost, i_intra_cost, i_type, i_intra_type );   //ע�͵���䣬��P֡�в���I�飬��Ϊ���α���������п���I������ͷ����仯��lijun ��Ҫ  *****

            if( i_intra_cost == COST_MAX )
                i_intra_cost = i_cost * i_satd_intra / i_satd_inter + 1;

			//����������ݹؼ�,ȷ�����α���ǰ�󻮷�һ��,   //*****
	        //*****
			if (h->info.embed_flag && !h->info.firstTime && h->info.cache[mb_xy].used)//��ǰ�������������
			{
                i_type = h->info.cache[mb_xy].i_type;//ǿ���޸�i_type,i_partion,i_subpartition,       i_type�ں��汣�浽��h->mb.i_type
				if (i_type==P_L0)
				{
					h->mb.i_partition = h->info.cache[mb_xy].i_partition;
					/*if (h->mb.i_partition != h->info.cache[mb_xy].i_partition)
					{
						h->mb.i_partition = h->info.cache[mb_xy].i_partition;
						//x264_mb_analyse_inter_p16x8(h, &analysis);//��ʱ
						//x264_mb_analyse_inter_p8x16(h, &analysis);
					}*/
					// ���Ᵽ��20220112
				}else if (i_type == P_8x8)
				{
					*(uint32_t*)&h->mb.i_sub_partition[0] = *(uint32_t*)&h->info.cache[mb_xy].i_sub_partition[0];
				}
				else if (i_type == P_SKIP)
				{
					h->mb.i_type == P_SKIP;//���õ�ǰ��ֱ��ΪP_SKIP����Ϊ��һ�α���Ϊskip���������Է�ֹ�ڶ��α���ʱ�����µ�P_8x8��P_L0��
				}

				//��ȷ���������i_type,i_partion,i_subpartition���α���һ�º�,����ȷ�������mv,ref����Ϣһ��,
				   /* | 0  1  4  5 
					* | 2  3  6  7 
					* | 8  9  12 13 
					* | 10 11 14 15 
					*/
				//�����h->info.cache[mb_xy].mv��ref�ǰ����˳��洢�ģ�ÿ����Ԫ����һ��4*4���״�*****
				switch (i_type)
				{
				case P_L0:
					switch (h->mb.i_partition)
					{
					case D_16x16:
						//x264_macroblock_cache_ref(h, 0, 0, 4, 4, 0, a->l0.me16x16.i_ref);//����ο�֡
						//x264_macroblock_cache_mv_ptr(h, 0, 0, 4, 4, 0, a->l0.me16x16.mv);//�����˶�ʸ��mv
						analysis.l0.me16x16.i_ref = h->info.cache[mb_xy].ref[0]; //���µ�analysis��,��������x264_analyse_update_cacheʵ�ʱ���
						*(uint32_t*)&analysis.l0.me16x16.mv[0] = *(uint32_t*)&h->info.cache[mb_xy].mv[0][0];
						break;

					case D_16x8:
						//x264_macroblock_cache_ref(h, 0, 0, 4, 2, 0, a->l0.me16x8[0].i_ref);//cache�е�ref��6*8���֣�ÿ��Ԫ�ر�ʾһ��4*4��λ��4��2��ʾ��ߣ�ʵ�ʾ�����2��4�ж���ֵΪi_ref,   0,0Ϊ16��Y�е�λ�� 
						//x264_macroblock_cache_ref(h, 0, 2, 4, 2, 0, a->l0.me16x8[1].i_ref);//0,2Ϊ16��Y�е�λ��
						//x264_macroblock_cache_mv_ptr(h, 0, 0, 4, 2, 0, a->l0.me16x8[0].mv);//����mv��cache  ����������ͬ
						//x264_macroblock_cache_mv_ptr(h, 0, 2, 4, 2, 0, a->l0.me16x8[1].mv);
						analysis.l0.me16x8[0].i_ref = h->info.cache[mb_xy].ref[0];
						analysis.l0.me16x8[1].i_ref = h->info.cache[mb_xy].ref[8];//���������һ��Ҫע�⣬���׳�����ͬ����zig-ɨ�裬��8*8�飬��4*4���˳��
						*(uint32_t*)&analysis.l0.me16x8[0].mv[0] = *(uint32_t*)&h->info.cache[mb_xy].mv[0][0];
						*(uint32_t*)&analysis.l0.me16x8[1].mv[0] = *(uint32_t*)&h->info.cache[mb_xy].mv[8][0];
						break;

					case D_8x16:
						//x264_macroblock_cache_ref(h, 0, 0, 2, 4, 0, a->l0.me8x16[0].i_ref);
						//x264_macroblock_cache_ref(h, 2, 0, 2, 4, 0, a->l0.me8x16[1].i_ref);
						//x264_macroblock_cache_mv_ptr(h, 0, 0, 2, 4, 0, a->l0.me8x16[0].mv);
						//x264_macroblock_cache_mv_ptr(h, 2, 0, 2, 4, 0, a->l0.me8x16[1].mv);
						analysis.l0.me8x16[0].i_ref = h->info.cache[mb_xy].ref[0];
						analysis.l0.me8x16[1].i_ref = h->info.cache[mb_xy].ref[4];
						*(uint32_t*)&analysis.l0.me8x16[0].mv[0] = *(uint32_t*)&h->info.cache[mb_xy].mv[0][0];
						*(uint32_t*)&analysis.l0.me8x16[1].mv[0] = *(uint32_t*)&h->info.cache[mb_xy].mv[4][0];
						break;

					default:
						x264_log(h, X264_LOG_ERROR, "internal error P_L0 and partition=%d\n", h->mb.i_partition);
						break;
					}
					break;

				case P_8x8:
					/*x264_macroblock_cache_ref(h, 0, 0, 2, 2, 0, a->l0.me8x8[0].i_ref);
					x264_macroblock_cache_ref(h, 2, 0, 2, 2, 0, a->l0.me8x8[1].i_ref);
					x264_macroblock_cache_ref(h, 0, 2, 2, 2, 0, a->l0.me8x8[2].i_ref);
					x264_macroblock_cache_ref(h, 2, 2, 2, 2, 0, a->l0.me8x8[3].i_ref);
					for (i = 0; i < 4; i++)
						x264_mb_cache_mv_p8x8(h, a, i);
					*/
					analysis.l0.me8x8[0].i_ref = h->info.cache[mb_xy].ref[0];//Ӧ���Ƕ���ÿ��8*8�飬��������ôϸ�֣��ο�֡��һ�����������������
					analysis.l0.me8x8[1].i_ref = h->info.cache[mb_xy].ref[4];
					analysis.l0.me8x8[2].i_ref = h->info.cache[mb_xy].ref[8];
					analysis.l0.me8x8[3].i_ref = h->info.cache[mb_xy].ref[12];
					for (i = 0; i < 4; i++){
						switch (h->mb.i_sub_partition[i])
						{
						case D_L0_8x8:
							//x264_macroblock_cache_mv_ptr(h, x, y, 2, 2, 0, a->l0.me8x8[i].mv);
							*(uint32_t*)&analysis.l0.me8x8[i].mv[0] = *(uint32_t*)&h->info.cache[mb_xy].mv[4*i][0];
							break;
						case D_L0_8x4:
							//x264_macroblock_cache_mv_ptr(h, x, y + 0, 2, 1, 0, a->l0.me8x4[i][0].mv);
							//x264_macroblock_cache_mv_ptr(h, x, y + 1, 2, 1, 0, a->l0.me8x4[i][1].mv);
							*(uint32_t*)&analysis.l0.me8x4[i][0].mv[0] = *(uint32_t*)&h->info.cache[mb_xy].mv[4 * i][0];
							*(uint32_t*)&analysis.l0.me8x4[i][1].mv[0] = *(uint32_t*)&h->info.cache[mb_xy].mv[4 * i+2][0];
							break;
						case D_L0_4x8:
							//x264_macroblock_cache_mv_ptr(h, x + 0, y, 1, 2, 0, a->l0.me4x8[i][0].mv);
							//x264_macroblock_cache_mv_ptr(h, x + 1, y, 1, 2, 0, a->l0.me4x8[i][1].mv);
							*(uint32_t*)&analysis.l0.me4x8[i][0].mv[0] = *(uint32_t*)&h->info.cache[mb_xy].mv[4 * i][0];
							*(uint32_t*)&analysis.l0.me4x8[i][1].mv[0] = *(uint32_t*)&h->info.cache[mb_xy].mv[4 * i + 1][0];
							break;
						case D_L0_4x4:
							//x264_macroblock_cache_mv_ptr(h, x + 0, y + 0, 1, 1, 0, a->l0.me4x4[i][0].mv);
							//x264_macroblock_cache_mv_ptr(h, x + 1, y + 0, 1, 1, 0, a->l0.me4x4[i][1].mv);
							//x264_macroblock_cache_mv_ptr(h, x + 0, y + 1, 1, 1, 0, a->l0.me4x4[i][2].mv);
							//x264_macroblock_cache_mv_ptr(h, x + 1, y + 1, 1, 1, 0, a->l0.me4x4[i][3].mv);
							*(uint32_t*)&analysis.l0.me4x4[i][0].mv[0] = *(uint32_t*)&h->info.cache[mb_xy].mv[4 * i][0];
							*(uint32_t*)&analysis.l0.me4x4[i][1].mv[0] = *(uint32_t*)&h->info.cache[mb_xy].mv[4 * i+1][0];
							*(uint32_t*)&analysis.l0.me4x4[i][2].mv[0] = *(uint32_t*)&h->info.cache[mb_xy].mv[4 * i+2][0];
							*(uint32_t*)&analysis.l0.me4x4[i][3].mv[0] = *(uint32_t*)&h->info.cache[mb_xy].mv[4 * i+3][0];
							break;
						default:
							x264_log(h, X264_LOG_ERROR, "internal error\n");
							break;
						}
					}
					break;
				default:
					break;
				}


			}

			h->mb.i_type = i_type;//����,�ؼ�����

            h->stat.frame.i_intra_cost += i_intra_cost;
            h->stat.frame.i_inter_cost += i_cost;
            h->stat.frame.i_mbs_analysed++;

			/******************************************�޸ģ��ڶ������±���ʱ�����޸ģ�P֡��**********************************/
			//��ǰ֡������Ƕ��,�ǵڶ��α���,����֡����Ƕ��,������������Ƕ��,��P_L0��P_8x8
			if ((h->info.embed_flag) && (!h->info.firstTime) && (h->info.cache[mb_xy].used==1) ) {
				if (h->mb.i_type == h->info.cache[mb_xy].i_type) {//ȷ������analysis�ĵ�һ��i_typeһ��  
					switch (h->mb.i_type)
					{
					case P_8x8:
						for (int i = 0; i < 4; i++)
						{
							if (h->mb.i_sub_partition[i]!= h->info.cache[mb_xy].i_sub_partition[i])
							{
								printf("8*8��%d����%d�ӿ����α���ڶ��㻮�ֲ�һ�£�δ��Ƕ�룡\n", mb_xy,i);
							}
							switch (h->mb.i_sub_partition[i]) {
							case D_L0_8x8:
								if (h->info.filp[h->info.i_mv_no++] == 1)
								{
									*(int32_t*)&analysis.l0.me8x8[i].mv[0] = *(int32_t*)&h->info.cache[mb_xy].mv_stego[4 * i][0];//��ˮƽ�ʹ�ֱ����������ֵ
									h->info.num_mv_modify_real++;
								}
								break;
							case D_L0_4x8:
								for (int j = 0; j < 2; j++)
								{
									if (h->info.filp[h->info.i_mv_no++] == 1)
									{
										*(int32_t*)&analysis.l0.me4x8[i][j].mv[0] = *(int32_t*)&h->info.cache[mb_xy].mv_stego[i * 4 + j][0];
										h->info.num_mv_modify_real++;
									}
								}
								break;
							case D_L0_8x4:
								for (int j = 0; j < 2; j++)
								{
									if (h->info.filp[h->info.i_mv_no++] == 1)
									{
										*(int32_t*)&analysis.l0.me8x4[i][j].mv[0] = *(int32_t*)&h->info.cache[mb_xy].mv_stego[i * 4 + 2 * j][0];
										h->info.num_mv_modify_real++;
									}
								}
								break;
							case D_L0_4x4:
								for (int j = 0; j < 4; j++)
								{
									if (h->info.filp[h->info.i_mv_no++] == 1)
									{
										*(int32_t*)&analysis.l0.me4x4[i][j].mv[0] = *(int32_t*)&h->info.cache[mb_xy].mv_stego[i * 4 + j][0];
										h->info.num_mv_modify_real++;
									}
								}
								break;
							default:
								break;
							}
						}
						break;
					case P_L0:
						if (h->mb.i_partition != h->info.cache[mb_xy].i_partition)
						{
							printf("16*16��%d���Ļ��ַ�ʽ���α���ڶ��㻮�ֲ�һ�£�δ��Ƕ�룡\n", mb_xy);
						}
						switch (h->mb.i_partition)
						{
						case D_16x16:
							if (h->info.filp[h->info.i_mv_no++] == 1)
							{
								*(int32_t*)&analysis.l0.me16x16.mv[0] = *(int32_t*)&h->info.cache[mb_xy].mv_stego[0][0];
								h->info.num_mv_modify_real++;
							}
							break;
						case D_8x16:
							for (int j = 0; j < 2; j++)
							{
								if (h->info.filp[h->info.i_mv_no++] == 1)
								{
									*(int32_t*)&analysis.l0.me8x16[j].mv[0] = *(int32_t*)&h->info.cache[mb_xy].mv_stego[j * 4][0];
									h->info.num_mv_modify_real++;
								}
							}
							break;
						case D_16x8:
							for (int j = 0; j < 2; j++)
							{
								if (h->info.filp[h->info.i_mv_no++] == 1)
								{
									*(int32_t*)&analysis.l0.me16x8[j].mv[0] = *(int32_t*)&h->info.cache[mb_xy].mv_stego[j * 8][0];
									h->info.num_mv_modify_real++;
								}
							}
							break;
						default:
							break;
						}
						break;

					default:
						break;

					}
				}
				else
				{
					printf("��%d�������α����һ�㻮�ֲ�һ�£�δ��Ƕ�룡\n", mb_xy);
				}
				if (DEGUG_LIJUN && (mb_xy == h->sh.i_last_mb - 1))//������һ������P_L0��P_8*8,������ˣ�û��ϵ��
				{
					printf("2��ʵ���޸Ĺ�����,��ǰ֡ͳ�Ƶ���mv������:%d����ʵ���޸ĵ�mv�ĸ���%d����\n", h->info.i_mv_no, h->info.num_mv_modify_real);
				}
			}
			
			/*******************************************����********************************************/

			// �ⲿ�ֿ���ע�͵�����ʵûɶ���ö��ڱ���
            if( 0 && analysis.i_mbrd >= 2 && h->mb.i_type != I_PCM )
            {
                if( IS_INTRA( h->mb.i_type ) )
                {
                    x264_intra_rd_refine( h, &analysis );
                }
                else if( i_partition == D_16x16 )
                {
                    x264_macroblock_cache_ref( h, 0, 0, 4, 4, 0, analysis.l0.me16x16.i_ref );
                    x264_me_refine_qpel_rd( h, &analysis.l0.me16x16, analysis.i_lambda2, 0, 0 );
                }
                else if( i_partition == D_16x8 )
                {
                    h->mb.i_sub_partition[0] = h->mb.i_sub_partition[1] =
                    h->mb.i_sub_partition[2] = h->mb.i_sub_partition[3] = D_L0_8x8;
                    x264_macroblock_cache_ref( h, 0, 0, 4, 2, 0, analysis.l0.me16x8[0].i_ref );
                    x264_macroblock_cache_ref( h, 0, 2, 4, 2, 0, analysis.l0.me16x8[1].i_ref );
                    x264_me_refine_qpel_rd( h, &analysis.l0.me16x8[0], analysis.i_lambda2, 0, 0 );
                    x264_me_refine_qpel_rd( h, &analysis.l0.me16x8[1], analysis.i_lambda2, 8, 0 );
                }
                else if( i_partition == D_8x16 )
                {
                    h->mb.i_sub_partition[0] = h->mb.i_sub_partition[1] =
                    h->mb.i_sub_partition[2] = h->mb.i_sub_partition[3] = D_L0_8x8;
                    x264_macroblock_cache_ref( h, 0, 0, 2, 4, 0, analysis.l0.me8x16[0].i_ref );
                    x264_macroblock_cache_ref( h, 2, 0, 2, 4, 0, analysis.l0.me8x16[1].i_ref );
                    x264_me_refine_qpel_rd( h, &analysis.l0.me8x16[0], analysis.i_lambda2, 0, 0 );
                    x264_me_refine_qpel_rd( h, &analysis.l0.me8x16[1], analysis.i_lambda2, 4, 0 );
                }
                else if( i_partition == D_8x8 )
                {
                    int i8x8;
                    x264_analyse_update_cache( h, &analysis );
                    for( i8x8 = 0; i8x8 < 4; i8x8++ )
                    {
                        if( h->mb.i_sub_partition[i8x8] == D_L0_8x8 )
                        {
                            x264_me_refine_qpel_rd( h, &analysis.l0.me8x8[i8x8], analysis.i_lambda2, i8x8*4, 0 );
                        }
                        else if( h->mb.i_sub_partition[i8x8] == D_L0_8x4 )
                        {
                           x264_me_refine_qpel_rd( h, &analysis.l0.me8x4[i8x8][0], analysis.i_lambda2, i8x8*4+0, 0 );
                           x264_me_refine_qpel_rd( h, &analysis.l0.me8x4[i8x8][1], analysis.i_lambda2, i8x8*4+2, 0 );
                        }
                        else if( h->mb.i_sub_partition[i8x8] == D_L0_4x8 )
                        {
                           x264_me_refine_qpel_rd( h, &analysis.l0.me4x8[i8x8][0], analysis.i_lambda2, i8x8*4+0, 0 );
                           x264_me_refine_qpel_rd( h, &analysis.l0.me4x8[i8x8][1], analysis.i_lambda2, i8x8*4+1, 0 );
                        }
                        else if( h->mb.i_sub_partition[i8x8] == D_L0_4x4 )
                        {
                           x264_me_refine_qpel_rd( h, &analysis.l0.me4x4[i8x8][0], analysis.i_lambda2, i8x8*4+0, 0 );
                           x264_me_refine_qpel_rd( h, &analysis.l0.me4x4[i8x8][1], analysis.i_lambda2, i8x8*4+1, 0 );
                           x264_me_refine_qpel_rd( h, &analysis.l0.me4x4[i8x8][2], analysis.i_lambda2, i8x8*4+2, 0 );
                           x264_me_refine_qpel_rd( h, &analysis.l0.me4x4[i8x8][3], analysis.i_lambda2, i8x8*4+3, 0 );
                        }
                    }
                }
            }
        }
    }
    else if( h->sh.i_type == SLICE_TYPE_B ) 
    {
        int i_bskip_cost = COST_MAX;
        int b_skip = 0;

        if( analysis.i_mbrd )
            x264_mb_cache_fenc_satd( h );

        h->mb.i_type = B_SKIP;
        if( h->mb.b_direct_auto_write )
        {
            /* direct=auto heuristic: prefer whichever mode allows more Skip macroblocks */
            for( i = 0; i < 2; i++ )
            {
                int b_changed = 1;
                h->sh.b_direct_spatial_mv_pred ^= 1;
                analysis.b_direct_available = x264_mb_predict_mv_direct16x16( h, i && analysis.b_direct_available ? &b_changed : NULL );
                if( analysis.b_direct_available )
                {
                    if( b_changed )
                    {
                        x264_mb_mc( h );
                        b_skip = x264_macroblock_probe_bskip( h );
                    }
                    h->stat.frame.i_direct_score[ h->sh.b_direct_spatial_mv_pred ] += b_skip;
                }
                else
                    b_skip = 0;
            }
        }
        else
            analysis.b_direct_available = x264_mb_predict_mv_direct16x16( h, NULL );

        if( analysis.b_direct_available )
        {
            if( !h->mb.b_direct_auto_write )
                x264_mb_mc( h );
            if( analysis.i_mbrd )
            {
                i_bskip_cost = ssd_mb( h );
                /* 6 = minimum cavlc cost of a non-skipped MB */
                b_skip = h->mb.b_skip_mc = i_bskip_cost <= ((6 * analysis.i_lambda2 + 128) >> 8);
            }
            else if( !h->mb.b_direct_auto_write )
            {
                /* Conditioning the probe on neighboring block types
                 * doesn't seem to help speed or quality. */
                b_skip = x264_macroblock_probe_bskip( h );
            }
        }

        if( !b_skip )
        {
            const unsigned int flags = h->param.analyse.inter;
            int i_type;
            int i_partition;
            int i_satd_inter = 0; // shut up uninitialized warning
            h->mb.b_skip_mc = 0;

            x264_mb_analyse_load_costs( h, &analysis );

            /* select best inter mode */
            /* direct must be first */
            if( analysis.b_direct_available )
                x264_mb_analyse_inter_direct( h, &analysis );

            x264_mb_analyse_inter_b16x16( h, &analysis );

            i_type = B_L0_L0;
            i_partition = D_16x16;
            i_cost = analysis.l0.me16x16.cost;
            COPY2_IF_LT( i_cost, analysis.l1.me16x16.cost, i_type, B_L1_L1 );
            COPY2_IF_LT( i_cost, analysis.i_cost16x16bi, i_type, B_BI_BI );
            COPY2_IF_LT( i_cost, analysis.i_cost16x16direct, i_type, B_DIRECT );

            if( analysis.i_mbrd && analysis.i_cost16x16direct <= i_cost * 33/32 )
            {
                x264_mb_analyse_b_rd( h, &analysis, i_cost );
                if( i_bskip_cost < analysis.i_rd16x16direct &&
                    i_bskip_cost < analysis.i_rd16x16bi &&
                    i_bskip_cost < analysis.l0.i_rd16x16 &&
                    i_bskip_cost < analysis.l1.i_rd16x16 )
                {
                    h->mb.i_type = B_SKIP;
                    x264_analyse_update_cache( h, &analysis );
                    return;
                }
            }

            if( flags & X264_ANALYSE_BSUB16x16 )
            {
                x264_mb_analyse_inter_b8x8( h, &analysis );
                if( analysis.i_cost8x8bi < i_cost )
                {
                    i_type = B_8x8;
                    i_partition = D_8x8;
                    i_cost = analysis.i_cost8x8bi;

                    if( h->mb.i_sub_partition[0] == h->mb.i_sub_partition[1] ||
                        h->mb.i_sub_partition[2] == h->mb.i_sub_partition[3] )
                    {
                        x264_mb_analyse_inter_b16x8( h, &analysis );
                        COPY3_IF_LT( i_cost, analysis.i_cost16x8bi,
                                     i_type, analysis.i_mb_type16x8,
                                     i_partition, D_16x8 );
                    }
                    if( h->mb.i_sub_partition[0] == h->mb.i_sub_partition[2] ||
                        h->mb.i_sub_partition[1] == h->mb.i_sub_partition[3] )
                    {
                        x264_mb_analyse_inter_b8x16( h, &analysis );
                        COPY3_IF_LT( i_cost, analysis.i_cost8x16bi,
                                     i_type, analysis.i_mb_type8x16,
                                     i_partition, D_8x16 );
                    }
                }
            }

            if( analysis.i_mbrd )
            {
                /* refine later */
            }
            /* refine qpel */
            else if( i_partition == D_16x16 )
            {
                analysis.l0.me16x16.cost -= analysis.i_lambda * i_mb_b_cost_table[B_L0_L0];
                analysis.l1.me16x16.cost -= analysis.i_lambda * i_mb_b_cost_table[B_L1_L1];
                if( i_type == B_L0_L0 )
                {
                    x264_me_refine_qpel( h, &analysis.l0.me16x16 );
                    i_cost = analysis.l0.me16x16.cost
                           + analysis.i_lambda * i_mb_b_cost_table[B_L0_L0];
                }
                else if( i_type == B_L1_L1 )
                {
                    x264_me_refine_qpel( h, &analysis.l1.me16x16 );
                    i_cost = analysis.l1.me16x16.cost
                           + analysis.i_lambda * i_mb_b_cost_table[B_L1_L1];
                }
                else if( i_type == B_BI_BI )
                {
                    x264_me_refine_qpel( h, &analysis.l0.me16x16 );
                    x264_me_refine_qpel( h, &analysis.l1.me16x16 );
                }
            }
            else if( i_partition == D_16x8 )
            {
                for( i=0; i<2; i++ )
                {
                    if( analysis.i_mb_partition16x8[i] != D_L1_8x8 )
                        x264_me_refine_qpel( h, &analysis.l0.me16x8[i] );
                    if( analysis.i_mb_partition16x8[i] != D_L0_8x8 )
                        x264_me_refine_qpel( h, &analysis.l1.me16x8[i] );
                }
            }
            else if( i_partition == D_8x16 )
            {
                for( i=0; i<2; i++ )
                {
                    if( analysis.i_mb_partition8x16[i] != D_L1_8x8 )
                        x264_me_refine_qpel( h, &analysis.l0.me8x16[i] );
                    if( analysis.i_mb_partition8x16[i] != D_L0_8x8 )
                        x264_me_refine_qpel( h, &analysis.l1.me8x16[i] );
                }
            }
            else if( i_partition == D_8x8 )
            {
                for( i=0; i<4; i++ )
                {
                    x264_me_t *m;
                    int i_part_cost_old;
                    int i_type_cost;
                    int i_part_type = h->mb.i_sub_partition[i];
                    int b_bidir = (i_part_type == D_BI_8x8);

                    if( i_part_type == D_DIRECT_8x8 )
                        continue;
                    if( x264_mb_partition_listX_table[0][i_part_type] )
                    {
                        m = &analysis.l0.me8x8[i];
                        i_part_cost_old = m->cost;
                        i_type_cost = analysis.i_lambda * i_sub_mb_b_cost_table[D_L0_8x8];
                        m->cost -= i_type_cost;
                        x264_me_refine_qpel( h, m );
                        if( !b_bidir )
                            analysis.i_cost8x8bi += m->cost + i_type_cost - i_part_cost_old;
                    }
                    if( x264_mb_partition_listX_table[1][i_part_type] )
                    {
                        m = &analysis.l1.me8x8[i];
                        i_part_cost_old = m->cost;
                        i_type_cost = analysis.i_lambda * i_sub_mb_b_cost_table[D_L1_8x8];
                        m->cost -= i_type_cost;
                        x264_me_refine_qpel( h, m );
                        if( !b_bidir )
                            analysis.i_cost8x8bi += m->cost + i_type_cost - i_part_cost_old;
                    }
                    /* TODO: update mvp? */
                }
            }

            if( analysis.i_mbrd )
            {
                i_satd_inter = i_cost;
                x264_mb_analyse_b_rd( h, &analysis, i_satd_inter );
                i_type = B_SKIP;
                i_cost = i_bskip_cost;
                i_partition = D_16x16;
                COPY2_IF_LT( i_cost, analysis.l0.i_rd16x16, i_type, B_L0_L0 );
                COPY2_IF_LT( i_cost, analysis.l1.i_rd16x16, i_type, B_L1_L1 );
                COPY2_IF_LT( i_cost, analysis.i_rd16x16bi, i_type, B_BI_BI );
                COPY2_IF_LT( i_cost, analysis.i_rd16x16direct, i_type, B_DIRECT );
                COPY3_IF_LT( i_cost, analysis.i_rd16x8bi, i_type, analysis.i_mb_type16x8, i_partition, D_16x8 );
                COPY3_IF_LT( i_cost, analysis.i_rd8x16bi, i_type, analysis.i_mb_type8x16, i_partition, D_8x16 );
                COPY3_IF_LT( i_cost, analysis.i_rd8x8bi, i_type, B_8x8, i_partition, D_8x8 );

                h->mb.i_type = i_type;
                h->mb.i_partition = i_partition;
            }

            x264_mb_analyse_intra( h, &analysis, i_satd_inter );

            if( analysis.i_mbrd )
            {
                x264_mb_analyse_transform_rd( h, &analysis, &i_satd_inter, &i_cost );
                x264_intra_rd( h, &analysis, i_satd_inter * 17/16 );
            }

            COPY2_IF_LT( i_cost, analysis.i_satd_i16x16, i_type, I_16x16 );
            COPY2_IF_LT( i_cost, analysis.i_satd_i8x8, i_type, I_8x8 );
            COPY2_IF_LT( i_cost, analysis.i_satd_i4x4, i_type, I_4x4 );
            COPY2_IF_LT( i_cost, analysis.i_satd_pcm, i_type, I_PCM );

            h->mb.i_type = i_type;
            h->mb.i_partition = i_partition;

            if( analysis.i_mbrd >= 2 && IS_INTRA( i_type ) && i_type != I_PCM )
                x264_intra_rd_refine( h, &analysis );
            if( h->mb.i_subpel_refine >= 5 )
                x264_refine_bidir( h, &analysis );

            if( analysis.i_mbrd >= 2 && i_type > B_DIRECT && i_type < B_SKIP )
            {
                const int i_biweight = h->mb.bipred_weight[analysis.l0.i_ref][analysis.l1.i_ref];
                x264_analyse_update_cache( h, &analysis );

                if( i_partition == D_16x16 )
                {
                    if( i_type == B_L0_L0 )
                        x264_me_refine_qpel_rd( h, &analysis.l0.me16x16, analysis.i_lambda2, 0, 0 );
                    else if( i_type == B_L1_L1 )
                        x264_me_refine_qpel_rd( h, &analysis.l1.me16x16, analysis.i_lambda2, 0, 1 );
                    else if( i_type == B_BI_BI )
                        x264_me_refine_bidir_rd( h, &analysis.l0.me16x16, &analysis.l1.me16x16, i_biweight, 0, analysis.i_lambda2 );
                }
                else if( i_partition == D_16x8 )
                {
                    for( i = 0; i < 2; i++ )
                    {
                        h->mb.i_sub_partition[i*2] = h->mb.i_sub_partition[i*2+1] = analysis.i_mb_partition16x8[i];
                        if( analysis.i_mb_partition16x8[i] == D_L0_8x8 )
                            x264_me_refine_qpel_rd( h, &analysis.l0.me16x8[i], analysis.i_lambda2, i*8, 0 );
                        else if( analysis.i_mb_partition16x8[i] == D_L1_8x8 )
                            x264_me_refine_qpel_rd( h, &analysis.l1.me16x8[i], analysis.i_lambda2, i*8, 1 );
                        else if( analysis.i_mb_partition16x8[i] == D_BI_8x8 )
                            x264_me_refine_bidir_rd( h, &analysis.l0.me16x8[i], &analysis.l1.me16x8[i], i_biweight, i*2, analysis.i_lambda2 );
                    }
                }
                else if( i_partition == D_8x16 )
                {
                    for( i = 0; i < 2; i++ )
                    {
                        h->mb.i_sub_partition[i] = h->mb.i_sub_partition[i+2] = analysis.i_mb_partition8x16[i];
                        if( analysis.i_mb_partition8x16[i] == D_L0_8x8 )
                            x264_me_refine_qpel_rd( h, &analysis.l0.me8x16[i], analysis.i_lambda2, i*4, 0 );
                        else if( analysis.i_mb_partition8x16[i] == D_L1_8x8 )
                            x264_me_refine_qpel_rd( h, &analysis.l1.me8x16[i], analysis.i_lambda2, i*4, 1 );
                        else if( analysis.i_mb_partition8x16[i] == D_BI_8x8 )
                            x264_me_refine_bidir_rd( h, &analysis.l0.me8x16[i], &analysis.l1.me8x16[i], i_biweight, i, analysis.i_lambda2 );
                    }
                }
                else if( i_partition == D_8x8 )
                {
                    for( i = 0; i < 4; i++ )
                    {
                        if( h->mb.i_sub_partition[i] == D_L0_8x8 )
                            x264_me_refine_qpel_rd( h, &analysis.l0.me8x8[i], analysis.i_lambda2, i*4, 0 );
                        else if( h->mb.i_sub_partition[i] == D_L1_8x8 )
                            x264_me_refine_qpel_rd( h, &analysis.l1.me8x8[i], analysis.i_lambda2, i*4, 1 );
                        else if( h->mb.i_sub_partition[i] == D_BI_8x8 )
                            x264_me_refine_bidir_rd( h, &analysis.l0.me8x8[i], &analysis.l1.me8x8[i], i_biweight, i, analysis.i_lambda2 );
                    }
                }
            }
        }
    }

	// �������õ��Ľ��analysis���µ�mb.cache�У�ͬʱ����ɫ�ȷ����ķ�������,���ǰ�������Ƕ���޸ģ����ʱ���ձ���Ľ����add lijun
	//���߳��¶�chroma����ģʽ���֣������������ŵĻ���ģʽ����Ӧ�Ĵ��ۣ���û�н����ؽ�����
    x264_analyse_update_cache( h, &analysis );
	
	
    if( !analysis.i_mbrd )
        x264_mb_analyse_transform( h );

    h->mb.b_trellis = h->param.analyse.i_trellis;
    h->mb.b_noise_reduction = !!h->param.analyse.i_noise_reduction;
    if( !IS_SKIP(h->mb.i_type) && h->mb.i_psy_trellis && h->param.analyse.i_trellis == 1 )
        x264_psy_trellis_init( h, 0 );
    if( h->mb.b_trellis == 1 || h->mb.b_noise_reduction )
        h->mb.i_skip_intra = 0;
	


// ����� ���ڼ����˶�ʸ����Ӧ��SATD��������cost��tang������
//h264��Lagrangian�Ż��㷨�������ʣ�ʵ��Ч�����ã�
//X264����һ�ְ뾫��֡��SATD(sum of absolute transformed difference)��Ϊģʽѡ������ add lijun
#define MV_SATD(mx, my)\
{\
	const int bw = x264_pixel_size[m->i_pixel].w;\
	const int bh = x264_pixel_size[m->i_pixel].h;\
	const int16_t *p_cost_mvx = m->p_cost_mv - m->mvp[0];\
	const int16_t *p_cost_mvy = m->p_cost_mv - m->mvp[1];\
	const int i_pixel = m->i_pixel;\
	const int b_chroma_me = h->mb.b_chroma_me && i_pixel <= PIXEL_8x8;\
	DECLARE_ALIGNED_16(uint8_t pix[2][32 * 18]);\
	int stride = 16; \
    cost = 0;\
	uint8_t *src = h->mc.get_ref( pix[0], &stride, m->p_fref, m->i_stride[0], mx, my, bw, bh ); \
	cost = h->pixf.mbcmp_unaligned[i_pixel]( m->p_fenc[0], FENC_STRIDE, src, stride ) \
				 + p_cost_mvx[ mx ] + p_cost_mvy[ my ]; \
	if( b_chroma_me)\
	{\
		h->mc.mc_chroma( pix[0], 8, m->p_fref[4], m->i_stride[1], mx, my, bw/2, bh/2 ); \
		cost += h->pixf.mbcmp[i_pixel+3]( m->p_fenc[1], FENC_STRIDE, pix[0], 8 ); \
		h->mc.mc_chroma( pix[0], 8, m->p_fref[5], m->i_stride[1], mx, my, bw/2, bh/2 ); \
		cost += h->pixf.mbcmp[i_pixel+3]( m->p_fenc[2], FENC_STRIDE, pix[0], 8 ); \
	}\
}



	/*******************������д��Ҫ�ı�����Ϣ����ú�ѡmv������ֲ�����ʧ�� ******************/
	//�������ʱ�򣬱������Ѿ�����������ѵĻ��ַ�ʽh->mb.i_type��h->mb.i_partition��h->mb.i_sub_partition[i]������
	//��h->mb.i_type==P_L0,��Ҫ��h->mb.i_partition����  D_16x16��D_16x8��D_8x16�������
	//��h->mb.i_type==P_8*8,��Ҫ��h->mb.i_sub_partition[i]����4��8*8������  D_L0_4x4��D_L0_8x4��D_L0_4x8��D_L0_8x8�������
	if (h->info.embed_flag && h->info.firstTime && h->sh.i_type == SLICE_TYPE_P) {
		h->info.cache[mb_xy].used = 0; // ��Ҫ�ı�־������0,������д����Ĺ���
		
		switch (h->mb.i_type)
		{
		case P_SKIP:   //�����P_SKIP��ҲҪ���棬�Ա�ڶ��α���ʱ������ͬ���Ļ��ַ�ʽ��
			h->info.cache[mb_xy].i_type = P_SKIP;
			break;
		case P_8x8://�ֳ�4��8*8�飬ÿ��8*8������ٷ֣�����h->mb.i_sub_partition[i]�����ӿ�Ļ��֣�
			h->info.cache[mb_xy].i_type = P_8x8;
			h->info.cache[mb_xy].used = 1;
			// �ӿ黮�ֱ���
			*(uint32_t*)&h->info.cache[mb_xy].i_sub_partition[0] = *(uint32_t*)&h->mb.i_sub_partition[0];
			// �ο�֡����
			*(uint32_t*)&h->info.cache[mb_xy].ref[0] = *(uint32_t*)&h->mb.cache.ref[0][x264_scan8[0]];
			*(uint32_t*)&h->info.cache[mb_xy].ref[4] = *(uint32_t*)&h->mb.cache.ref[0][x264_scan8[2]];
			*(uint32_t*)&h->info.cache[mb_xy].ref[8] = *(uint32_t*)&h->mb.cache.ref[0][x264_scan8[8]];
			*(uint32_t*)&h->info.cache[mb_xy].ref[12] = *(uint32_t*)&h->mb.cache.ref[0][x264_scan8[10]];
			// �˶�ʸ��
			for (int idx = 0; idx < 16;) {
				//h->mb.cache.mv[0][x264_scan8[idx]][0]��h->mb.cache.mv[0][x264_scan8[idx]][1]�ֱ���h��v����lijun  int16���� ��Ϊuint32_t��һ�θ�������
				*(uint32_t*)&h->info.cache[mb_xy].mv[idx][0] = *(uint32_t*)&h->mb.cache.mv[0][x264_scan8[idx]][0];
				*(uint32_t*)&h->info.cache[mb_xy].mv[idx][0] = *(uint32_t*)&h->mb.cache.mv[0][x264_scan8[idx++]][0];  if (idx == 16) break;   //ÿ���⣬
				*(uint32_t*)&h->info.cache[mb_xy].mv[idx][0] = *(uint32_t*)&h->mb.cache.mv[0][x264_scan8[idx++]][0];
				*(uint32_t*)&h->info.cache[mb_xy].mv[idx][0] = *(uint32_t*)&h->mb.cache.mv[0][x264_scan8[idx++]][0];//
			}

			// �޸���������˶�ʸ����Ҫ�����Ĵ���
			for (int i = 0; i < 4; i++) 
			{
				uint8_t partition = h->mb.i_sub_partition[i];
				x264_me_t *m;
				switch (partition)
				{
				case D_L0_8x8: // 3     ֱ����һ����8*8�飬�洢һ��mv��
					h->info.stat.num_p_block++;//ͳ��һ��mv����
					m = &analysis.l0.me8x8[i];
					int16_t m_x = 0, m_y = 0;//���mv���޸���
					int16_t bmx = m->mv[0], bmy = m->mv[1]; //�������˶�����֮��Ԥ����˶�ʸ��
					int cost_opt = x264_ih_get_mv_cost(h, &analysis, m, &m_x, &m_y, d_mv, d_mv_1_neighborhood,mb_xy);//��д��������ʧ��
					
					h->info.cache[mb_xy].mv_stego[i * 4 ][0] = bmx + m_x;
					h->info.cache[mb_xy].mv_stego[i * 4 ][1] = bmy + m_y;
					//int cost_complexity = 0;//�����븴�����йص�ʧ��
					h->info.cache[mb_xy].inter_stego_cost[i * 4] = cost_opt;	
					//h->info.cache[mb_xy].ref[i * 4] = m->i_ref;
					break;
				case D_L0_4x8: // 2
					h->info.stat.num_p_block+=2;//ͳ��һ��mv����
					for (int j = 0; j < 2; j++)
					{
						m = &analysis.l0.me4x8[i][j];
						int16_t bmx = m->mv[0], bmy = m->mv[1]; //�������˶�����֮��Ԥ����˶�ʸ��
						int16_t m_x = 0, m_y = 0;
						int cost_opt = x264_ih_get_mv_cost(h, &analysis, m, &m_x, &m_y, d_mv, d_mv_1_neighborhood, mb_xy);//��д��������ʧ��
						h->info.cache[mb_xy].mv_stego[i * 4 + j][0] = bmx + m_x;
						h->info.cache[mb_xy].mv_stego[i * 4 + j][1] = bmy + m_y;
						h->info.cache[mb_xy].inter_stego_cost[i * 4 + j] = cost_opt;
						//h->info.cache[mb_xy].ref[i * 4 + j] = m->i_ref;
					}	
					break;
				case D_L0_8x4: // 1
					h->info.stat.num_p_block += 2;//ͳ��һ��mv����
					for(int j = 0; j < 2; j++)
					{
						m = &analysis.l0.me8x4[i][j];
						int16_t bmx = m->mv[0], bmy = m->mv[1]; //�������˶�����֮��Ԥ����˶�ʸ��
						int16_t m_x = 0, m_y = 0;
						int cost_opt = x264_ih_get_mv_cost(h, &analysis, m, &m_x, &m_y, d_mv, d_mv_1_neighborhood, mb_xy);//��д��������ʧ��
						h->info.cache[mb_xy].mv_stego[i * 4 + 2 * j][0] = bmx + m_x;
						h->info.cache[mb_xy].mv_stego[i * 4 + 2 * j][1] = bmy + m_y;
						h->info.cache[mb_xy].inter_stego_cost[i * 4 + 2 * j] = cost_opt;
						//h->info.cache[mb_xy].ref[i * 4 + 2 * j] = m->i_ref;	
						// x264_mb_mc_0xywh(h, 2 * (i & 1), 2 * (i >> 1) + j, 1, 2);
					}
					break;
				case D_L0_4x4: // 0
					h->info.stat.num_p_block += 4;//ͳ��һ��mv����
					for (int j = 0; j < 4; j++)
					{
						m = &analysis.l0.me4x4[i][j];
						int16_t bmx = m->mv[0], bmy = m->mv[1]; //�������˶�����֮��Ԥ����˶�ʸ��
						int16_t m_x = 0, m_y = 0;
						int cost_opt = x264_ih_get_mv_cost(h, &analysis, m, &m_x, &m_y, d_mv, d_mv_1_neighborhood, mb_xy);//��д��������ʧ��
						h->info.cache[mb_xy].mv_stego[i * 4 + j][0] = bmx + m_x;
						h->info.cache[mb_xy].mv_stego[i * 4 + j][1] = bmy + m_y;
						h->info.cache[mb_xy].inter_stego_cost[i * 4 + j] = cost_opt;
						//h->info.cache[mb_xy].ref[i * 4 + j] = m->i_ref;

						// x264_mb_mc_0xywh(h, 2 * (i & 1) + (j & 1) , 2 * (i >> 1) + (j >> 1), 1, 1);
					}
					break;
				default:
					break;
				}
			}
			break;
		case P_L0:
			h->info.cache[mb_xy].i_type = P_L0;
			h->info.cache[mb_xy].used = 1;
			// ����
			h->info.cache[mb_xy].i_partition = h->mb.i_partition;
			// �ο�
			*(uint32_t*)&h->info.cache[mb_xy].ref[0] = *(uint32_t*)&h->mb.cache.ref[0][x264_scan8[0]];//����0��2��8��10�ֱ�Ϊÿ�е����
			*(uint32_t*)&h->info.cache[mb_xy].ref[4] = *(uint32_t*)&h->mb.cache.ref[0][x264_scan8[2]];
			*(uint32_t*)&h->info.cache[mb_xy].ref[8] = *(uint32_t*)&h->mb.cache.ref[0][x264_scan8[8]];
			*(uint32_t*)&h->info.cache[mb_xy].ref[12] = *(uint32_t*)&h->mb.cache.ref[0][x264_scan8[10]];
			// �˶�ʸ��
			for (int idx = 0; idx < 16;) {
				//h->mb.cache.mv[0][x264_scan8[idx]][0]��h->mb.cache.mv[0][x264_scan8[idx]][1]�ֱ���h��v����lijun  int16���� ��Ϊuint32_t��һ�θ�������
				*(uint32_t*)&h->info.cache[mb_xy].mv[idx][0] = *(uint32_t*)&h->mb.cache.mv[0][x264_scan8[idx]][0];
				*(uint32_t*)&h->info.cache[mb_xy].mv[idx][0] = *(uint32_t*)&h->mb.cache.mv[0][x264_scan8[idx++]][0];if (idx == 16) break;
				*(uint32_t*)&h->info.cache[mb_xy].mv[idx][0] = *(uint32_t*)&h->mb.cache.mv[0][x264_scan8[idx++]][0]; 
				*(uint32_t*)&h->info.cache[mb_xy].mv[idx][0] = *(uint32_t*)&h->mb.cache.mv[0][x264_scan8[idx++]][0];//����ط�Խ���ˣ��������16�Ժ��������˼��飬�Ѻ�����ڴ���ˣ�
			}

			// �޸���������˶�ʸ����Ҫ�����Ĵ���
			uint8_t partition = h->mb.i_partition;
			x264_me_t *m;
			switch (partition)
			{
			case D_16x16: // 16    
				h->info.stat.num_p_block++;//ͳ��һ��mv����
				m = &analysis.l0.me16x16;
				int16_t m_x = 0, m_y = 0;//���mv���޸���

				int cost_opt = x264_ih_get_mv_cost(h, &analysis, m, &m_x, &m_y, d_mv, d_mv_1_neighborhood, mb_xy);//��д��������ʧ��

				h->info.cache[mb_xy].mv_stego[0][0] = m->mv[0] + m_x;
				h->info.cache[mb_xy].mv_stego[0][1] = m->mv[1] + m_y;
				//int cost_complexity = 0;//�����븴�����йص�ʧ��
				h->info.cache[mb_xy].inter_stego_cost[0] = cost_opt;
				//h->info.cache[mb_xy].ref[0] = m->i_ref;
				break;
			case D_8x16: // 15
				h->info.stat.num_p_block += 2;//ͳ��һ��mv����
				for (int j = 0; j < 2; j++)
				{
					m = &analysis.l0.me8x16[j];
					int16_t bmx = m->mv[0], bmy = m->mv[1]; //�������˶�����֮��Ԥ����˶�ʸ��
					int16_t m_x = 0, m_y = 0;
					int cost_opt = x264_ih_get_mv_cost(h, &analysis, m, &m_x, &m_y, d_mv, d_mv_1_neighborhood, mb_xy);//��д��������ʧ��
					h->info.cache[mb_xy].mv_stego[j * 4 ][0] = bmx + m_x;//j*8  ��
					h->info.cache[mb_xy].mv_stego[j * 4 ][1] = bmy + m_y;
					h->info.cache[mb_xy].inter_stego_cost[j * 4] = cost_opt;
					//h->info.cache[mb_xy].ref[j * 8] = m->i_ref;
				}
				break;
			case D_16x8: // 14
				h->info.stat.num_p_block += 2;//ͳ��һ��mv����
				for (int j = 0; j < 2; j++)
				{
					m = &analysis.l0.me16x8[j];
					int16_t bmx = m->mv[0], bmy = m->mv[1]; //�������˶�����֮��Ԥ����˶�ʸ��
					int16_t m_x = 0, m_y = 0;
					int cost_opt = x264_ih_get_mv_cost(h, &analysis, m, &m_x, &m_y, d_mv, d_mv_1_neighborhood, mb_xy);//��д��������ʧ��
					h->info.cache[mb_xy].mv_stego[j * 8][0] = bmx + m_x;
					h->info.cache[mb_xy].mv_stego[j * 8][1] = bmy + m_y;
					h->info.cache[mb_xy].inter_stego_cost[j * 8] = cost_opt;
					//h->info.cache[mb_xy].ref[j * 8] = m->i_ref;
					// x264_mb_mc_0xywh(h, 2 * (i & 1), 2 * (i >> 1) + j, 1, 2);
				}
				break;
			default:
				break;
			}
			break;
		default:// ����ģʽ

			break;
		}
}

#undef MV_SATD_FDEC_IH

#undef MV_SATD

	/*********************����*****************************/

}

//��a�е���Ϣ���浽h.mb.cache�У������ڱ���ʹ��
//����I�飬��Ҫ��Ԥ��ģʽ�����ҽ���ɫ��֡��Ԥ��
//����P��B�飬��Ҫ�ǲο�֡��mv����
/*-------------------- Update MB from the analysis ----------------------*/
static void x264_analyse_update_cache( x264_t *h, x264_mb_analysis_t *a  )
{
    int i;

    switch( h->mb.i_type )
    {
        case I_4x4:
            for( i = 0; i < 16; i++ )
                h->mb.cache.intra4x4_pred_mode[x264_scan8[i]] = a->i_predict4x4[i];

            x264_mb_analyse_intra_chroma( h, a );
            break;
        case I_8x8:
            for( i = 0; i < 4; i++ )
                x264_macroblock_cache_intra8x8_pred( h, 2*(i&1), 2*(i>>1), a->i_predict8x8[i] );

            x264_mb_analyse_intra_chroma( h, a );
            break;
        case I_16x16:
            h->mb.i_intra16x16_pred_mode = a->i_predict16x16;
            x264_mb_analyse_intra_chroma( h, a );
            break;

        case I_PCM:
            break;

        case P_L0:
            switch( h->mb.i_partition )
            {
                case D_16x16:
                    x264_macroblock_cache_ref( h, 0, 0, 4, 4, 0, a->l0.me16x16.i_ref );//����ο�֡
                    x264_macroblock_cache_mv_ptr( h, 0, 0, 4, 4, 0, a->l0.me16x16.mv );//�����˶�ʸ��mv
                    break;

                case D_16x8:
					/* real intra4x4_pred_mode if I_4X4 or I_8X8, I_PRED_4x4_DC if mb available, -1 if not
					* mb.cache.intra4x4_pred_mode[],ref[],mv[][0~1]��ʽ����
					*   |
					* --+--------------
					*   | 0 0 0 y y y y y
					*   | 0 U U y Y Y Y Y
					*   | 0 U U y Y Y Y Y
					*   | 0 0 0 y Y Y Y Y
					*   | 0 V V y Y Y Y Y
					*   | 0 V V 0 L C C 0
					**/
					//cache�е�ref��6*8���֣�ÿ��Ԫ�ر�ʾһ��4*4��λ��4��2��ʾ��ߣ�ʵ�ʾ�����2��4�ж���ֵΪi_ref,   0,0Ϊ16��Y�е�λ�ã������Ϊ��� 
                    x264_macroblock_cache_ref( h, 0, 0, 4, 2, 0, a->l0.me16x8[0].i_ref );
                    x264_macroblock_cache_ref( h, 0, 2, 4, 2, 0, a->l0.me16x8[1].i_ref );//0,2Ϊ16��Y�е�λ��
                    x264_macroblock_cache_mv_ptr( h, 0, 0, 4, 2, 0, a->l0.me16x8[0].mv );//����mv��cache  ����������ͬ
                    x264_macroblock_cache_mv_ptr( h, 0, 2, 4, 2, 0, a->l0.me16x8[1].mv );
                    break;

                case D_8x16:
                    x264_macroblock_cache_ref( h, 0, 0, 2, 4, 0, a->l0.me8x16[0].i_ref );
                    x264_macroblock_cache_ref( h, 2, 0, 2, 4, 0, a->l0.me8x16[1].i_ref );
                    x264_macroblock_cache_mv_ptr( h, 0, 0, 2, 4, 0, a->l0.me8x16[0].mv );
                    x264_macroblock_cache_mv_ptr( h, 2, 0, 2, 4, 0, a->l0.me8x16[1].mv );
                    break;

                default:
                    x264_log( h, X264_LOG_ERROR, "internal error P_L0 and partition=%d\n", h->mb.i_partition );
                    break;
            }
            break;

        case P_8x8:
            x264_macroblock_cache_ref( h, 0, 0, 2, 2, 0, a->l0.me8x8[0].i_ref );
            x264_macroblock_cache_ref( h, 2, 0, 2, 2, 0, a->l0.me8x8[1].i_ref );
            x264_macroblock_cache_ref( h, 0, 2, 2, 2, 0, a->l0.me8x8[2].i_ref );
            x264_macroblock_cache_ref( h, 2, 2, 2, 2, 0, a->l0.me8x8[3].i_ref );
            for( i = 0; i < 4; i++ )
                x264_mb_cache_mv_p8x8( h, a, i );
            break;

        case P_SKIP:
        {
            h->mb.i_partition = D_16x16;
            x264_macroblock_cache_ref( h, 0, 0, 4, 4, 0, 0 );
            x264_macroblock_cache_mv_ptr( h, 0, 0, 4, 4, 0, h->mb.cache.pskip_mv );
            break;
        }

        case B_SKIP:
        case B_DIRECT:
            x264_mb_load_mv_direct8x8( h, 0 );
            x264_mb_load_mv_direct8x8( h, 1 );
            x264_mb_load_mv_direct8x8( h, 2 );
            x264_mb_load_mv_direct8x8( h, 3 );
            break;

        case B_8x8:
            /* optimize: cache might not need to be rewritten */
            for( i = 0; i < 4; i++ )
                x264_mb_cache_mv_b8x8( h, a, i, 1 );
            break;

        default: /* the rest of the B types */
            switch( h->mb.i_partition )
            {
            case D_16x16:
                switch( h->mb.i_type )
                {
                case B_L0_L0:
                    x264_macroblock_cache_ref( h, 0, 0, 4, 4, 0, a->l0.i_ref );
                    x264_macroblock_cache_mv_ptr( h, 0, 0, 4, 4, 0, a->l0.me16x16.mv );

                    x264_macroblock_cache_ref( h, 0, 0, 4, 4, 1, -1 );
                    x264_macroblock_cache_mv ( h, 0, 0, 4, 4, 1, 0 );
                    x264_macroblock_cache_mvd( h, 0, 0, 4, 4, 1, 0 );
                    break;
                case B_L1_L1:
                    x264_macroblock_cache_ref( h, 0, 0, 4, 4, 0, -1 );
                    x264_macroblock_cache_mv ( h, 0, 0, 4, 4, 0, 0 );
                    x264_macroblock_cache_mvd( h, 0, 0, 4, 4, 0, 0 );

                    x264_macroblock_cache_ref( h, 0, 0, 4, 4, 1, a->l1.i_ref );
                    x264_macroblock_cache_mv_ptr( h, 0, 0, 4, 4, 1, a->l1.me16x16.mv );
                    break;
                case B_BI_BI:
                    x264_macroblock_cache_ref( h, 0, 0, 4, 4, 0, a->l0.i_ref );
                    x264_macroblock_cache_mv_ptr( h, 0, 0, 4, 4, 0, a->l0.me16x16.mv );

                    x264_macroblock_cache_ref( h, 0, 0, 4, 4, 1, a->l1.i_ref );
                    x264_macroblock_cache_mv_ptr( h, 0, 0, 4, 4, 1, a->l1.me16x16.mv );
                    break;
                }
                break;
            case D_16x8:
                x264_mb_cache_mv_b16x8( h, a, 0, 1 );
                x264_mb_cache_mv_b16x8( h, a, 1, 1 );
                break;
            case D_8x16:
                x264_mb_cache_mv_b8x16( h, a, 0, 1 );
                x264_mb_cache_mv_b8x16( h, a, 1, 1 );
                break;
            default:
                x264_log( h, X264_LOG_ERROR, "internal error (invalid MB type)\n" );
                break;
            }
    }

#ifndef NDEBUG
    if( h->param.i_threads > 1 && !IS_INTRA(h->mb.i_type) )
    {
        int l;
        for( l=0; l <= (h->sh.i_type == SLICE_TYPE_B); l++ )
        {
            int completed;
            int ref = h->mb.cache.ref[l][x264_scan8[0]];
            if( ref < 0 )
                continue;
            completed = (l ? h->fref1 : h->fref0)[ ref >> h->mb.b_interlaced ]->i_lines_completed;
            if( (h->mb.cache.mv[l][x264_scan8[15]][1] >> (2 - h->mb.b_interlaced)) + h->mb.i_mb_y*16 > completed )
            {
                x264_log( h, X264_LOG_WARNING, "internal error (MV out of thread range)\n");
                fprintf(stderr, "mb type: %d \n", h->mb.i_type);
                fprintf(stderr, "mv: l%dr%d (%d,%d) \n", l, ref,
                                h->mb.cache.mv[l][x264_scan8[15]][0],
                                h->mb.cache.mv[l][x264_scan8[15]][1] );
                fprintf(stderr, "limit: %d \n", h->mb.mv_max_spel[1]);
                fprintf(stderr, "mb_xy: %d,%d \n", h->mb.i_mb_x, h->mb.i_mb_y);
                fprintf(stderr, "completed: %d \n", completed );
                x264_log( h, X264_LOG_WARNING, "recovering by using intra mode\n");
                x264_mb_analyse_intra( h, a, COST_MAX );
                h->mb.i_type = I_16x16;
                h->mb.i_intra16x16_pred_mode = a->i_predict16x16;
                x264_mb_analyse_intra_chroma( h, a );
            }
        }
    }
#endif
}


//��ÿ�α�����ɺ�ĵ�ǰ�����ؽ����ݷ���h->mb.pic.fenc_buf_ih�У�ָ��h->mb.pic.p_fenc_ih�� lijun
//�ο�common/macroblock.c �µ�x264_macroblock_store_pic��д
static void ALWAYS_INLINE x264_macroblock_store_pic_mb2mb_ih(x264_t *h)  //*****
{
	int w = 0;
	for (int i = 0; i < 3; i++) //YUV
	{
		w = i ? 8 : 16;
		h->mc.copy[i ? PIXEL_8x8 : PIXEL_16x16](h->mb.pic.p_fenc_ih[i], FENC_STRIDE,
			h->mb.pic.p_fdec[i], FDEC_STRIDE, w);//
	}
	
}
#include "slicetype.c"

