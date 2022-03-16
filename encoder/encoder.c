/*****************************************************************************
 * x264: h264 encoder
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

#include <math.h>

#include "common/common.h"
#include "common/cpu.h"

#include "set.h"
#include "analyse.h"
#include "ratecontrol.h"
#include "macroblock.h"

#include "embed.h"

//extern int add_lib_lijun(int a, int b);//���Ծ�̬��lijun
extern * get_cost_lib_for_x264(int w, int h, int * mv_h, int * mv_v);
//#include "mat2D.h"

#if VISUALIZE
#include "common/visualize.h"
#endif


extern uint16_t *g_x264_cost_mv_fpel[52][4];
extern int16_t *g_cost_mv[52];

//#define DEBUG_MB_TYPE

#define NALU_OVERHEAD 5 // startcode + NAL type costs 5 bytes per frame

#define bs_write_ue bs_write_ue_big

static void x264_encoder_frame_end( x264_t *h, x264_t *thread_current,
                                    x264_nal_t **pp_nal, int *pi_nal,
                                    x264_picture_t *pic_out );

/****************************************************************************
 *
 ******************************* x264 libs **********************************
 *
 ****************************************************************************/
static float x264_psnr( int64_t i_sqe, int64_t i_size )
{
    double f_mse = (double)i_sqe / ((double)65025.0 * (double)i_size);
    if( f_mse <= 0.0000000001 ) /* Max 100dB */
        return 100;

    return (float)(-10.0 * log( f_mse ) / log( 10.0 ));
}

static void x264_frame_dump( x264_t *h )
{
    FILE *f = fopen( h->param.psz_dump_yuv, "r+b" );
    int i, y;
    if( !f )
        return;
    /* Write the frame in display order */
    fseek( f, h->fdec->i_frame * h->param.i_height * h->param.i_width * 3/2, SEEK_SET );
    for( i = 0; i < h->fdec->i_plane; i++ )
        for( y = 0; y < h->param.i_height >> !!i; y++ )
            fwrite( &h->fdec->plane[i][y*h->fdec->i_stride[i]], 1, h->param.i_width >> !!i, f );
    fclose( f );
}


/* Fill "default" values */
static void x264_slice_header_init( x264_t *h, x264_slice_header_t *sh,
                                    x264_sps_t *sps, x264_pps_t *pps,
                                    int i_idr_pic_id, int i_frame, int i_qp )
{
    x264_param_t *param = &h->param;
    int i;

    /* First we fill all field */
    sh->sps = sps;
    sh->pps = pps;

    sh->i_first_mb  = 0;
    sh->i_last_mb   = h->sps->i_mb_width * h->sps->i_mb_height;
    sh->i_pps_id    = pps->i_id;

    sh->i_frame_num = i_frame;

    sh->b_mbaff = h->param.b_interlaced;
    sh->b_field_pic = 0;    /* no field support for now */
    sh->b_bottom_field = 0; /* not yet used */

    sh->i_idr_pic_id = i_idr_pic_id;

    /* poc stuff, fixed later */
    sh->i_poc_lsb = 0;
    sh->i_delta_poc_bottom = 0;
    sh->i_delta_poc[0] = 0;
    sh->i_delta_poc[1] = 0;

    sh->i_redundant_pic_cnt = 0;

    if( !h->mb.b_direct_auto_read )
    {
        if( h->mb.b_direct_auto_write )
            sh->b_direct_spatial_mv_pred = ( h->stat.i_direct_score[1] > h->stat.i_direct_score[0] );
        else
            sh->b_direct_spatial_mv_pred = ( param->analyse.i_direct_mv_pred == X264_DIRECT_PRED_SPATIAL );
    }
    /* else b_direct_spatial_mv_pred was read from the 2pass statsfile */

    sh->b_num_ref_idx_override = 0;
    sh->i_num_ref_idx_l0_active = 1;
    sh->i_num_ref_idx_l1_active = 1;

    sh->b_ref_pic_list_reordering_l0 = h->b_ref_reorder[0];
    sh->b_ref_pic_list_reordering_l1 = h->b_ref_reorder[1];

    /* If the ref list isn't in the default order, construct reordering header */
    /* List1 reordering isn't needed yet */
    if( sh->b_ref_pic_list_reordering_l0 )
    {
        int pred_frame_num = i_frame;
        for( i = 0; i < h->i_ref0; i++ )
        {
            int diff = h->fref0[i]->i_frame_num - pred_frame_num;
            if( diff == 0 )
                x264_log( h, X264_LOG_ERROR, "diff frame num == 0\n" );
            sh->ref_pic_list_order[0][i].idc = ( diff > 0 );
            sh->ref_pic_list_order[0][i].arg = abs( diff ) - 1;
            pred_frame_num = h->fref0[i]->i_frame_num;
        }
    }

    sh->i_cabac_init_idc = param->i_cabac_init_idc;

    sh->i_qp = i_qp;
    sh->i_qp_delta = i_qp - pps->i_pic_init_qp;
    sh->b_sp_for_swidth = 0;
    sh->i_qs_delta = 0;

    /* If effective qp <= 15, deblocking would have no effect anyway */
    if( param->b_deblocking_filter
        && ( h->mb.b_variable_qp
        || 15 < i_qp + 2 * X264_MIN(param->i_deblocking_filter_alphac0, param->i_deblocking_filter_beta) ) )
    {
        sh->i_disable_deblocking_filter_idc = 0;
    }
    else
    {
        sh->i_disable_deblocking_filter_idc = 1;
    }
    sh->i_alpha_c0_offset = param->i_deblocking_filter_alphac0 << 1;
    sh->i_beta_offset = param->i_deblocking_filter_beta << 1;
}

static void x264_slice_header_write( bs_t *s, x264_slice_header_t *sh, int i_nal_ref_idc )
{
    int i;

    if( sh->b_mbaff )
    {
        assert( sh->i_first_mb % (2*sh->sps->i_mb_width) == 0 );
        bs_write_ue( s, sh->i_first_mb >> 1 );
    }
    else
        bs_write_ue( s, sh->i_first_mb );

    bs_write_ue( s, sh->i_type + 5 );   /* same type things */
    bs_write_ue( s, sh->i_pps_id );
    bs_write( s, sh->sps->i_log2_max_frame_num, sh->i_frame_num );

    if( !sh->sps->b_frame_mbs_only )
    {
        bs_write1( s, sh->b_field_pic );
        if ( sh->b_field_pic )
            bs_write1( s, sh->b_bottom_field );
    }

    if( sh->i_idr_pic_id >= 0 ) /* NAL IDR */
    {
        bs_write_ue( s, sh->i_idr_pic_id );
    }

    if( sh->sps->i_poc_type == 0 )
    {
        bs_write( s, sh->sps->i_log2_max_poc_lsb, sh->i_poc_lsb );
        if( sh->pps->b_pic_order && !sh->b_field_pic )
        {
            bs_write_se( s, sh->i_delta_poc_bottom );
        }
    }
    else if( sh->sps->i_poc_type == 1 && !sh->sps->b_delta_pic_order_always_zero )
    {
        bs_write_se( s, sh->i_delta_poc[0] );
        if( sh->pps->b_pic_order && !sh->b_field_pic )
        {
            bs_write_se( s, sh->i_delta_poc[1] );
        }
    }

    if( sh->pps->b_redundant_pic_cnt )
    {
        bs_write_ue( s, sh->i_redundant_pic_cnt );
    }

    if( sh->i_type == SLICE_TYPE_B )
    {
        bs_write1( s, sh->b_direct_spatial_mv_pred );
    }
    if( sh->i_type == SLICE_TYPE_P || sh->i_type == SLICE_TYPE_SP || sh->i_type == SLICE_TYPE_B )
    {
        bs_write1( s, sh->b_num_ref_idx_override );
        if( sh->b_num_ref_idx_override )
        {
            bs_write_ue( s, sh->i_num_ref_idx_l0_active - 1 );
            if( sh->i_type == SLICE_TYPE_B )
            {
                bs_write_ue( s, sh->i_num_ref_idx_l1_active - 1 );
            }
        }
    }

    /* ref pic list reordering */
    if( sh->i_type != SLICE_TYPE_I )
    {
        bs_write1( s, sh->b_ref_pic_list_reordering_l0 );
        if( sh->b_ref_pic_list_reordering_l0 )
        {
            for( i = 0; i < sh->i_num_ref_idx_l0_active; i++ )
            {
                bs_write_ue( s, sh->ref_pic_list_order[0][i].idc );
                bs_write_ue( s, sh->ref_pic_list_order[0][i].arg );

            }
            bs_write_ue( s, 3 );
        }
    }
    if( sh->i_type == SLICE_TYPE_B )
    {
        bs_write1( s, sh->b_ref_pic_list_reordering_l1 );
        if( sh->b_ref_pic_list_reordering_l1 )
        {
            for( i = 0; i < sh->i_num_ref_idx_l1_active; i++ )
            {
                bs_write_ue( s, sh->ref_pic_list_order[1][i].idc );
                bs_write_ue( s, sh->ref_pic_list_order[1][i].arg );
            }
            bs_write_ue( s, 3 );
        }
    }

    if( ( sh->pps->b_weighted_pred && ( sh->i_type == SLICE_TYPE_P || sh->i_type == SLICE_TYPE_SP ) ) ||
        ( sh->pps->b_weighted_bipred == 1 && sh->i_type == SLICE_TYPE_B ) )
    {
        /* FIXME */
    }

    if( i_nal_ref_idc != 0 )
    {
        if( sh->i_idr_pic_id >= 0 )
        {
            bs_write1( s, 0 );  /* no output of prior pics flag */
            bs_write1( s, 0 );  /* long term reference flag */
        }
        else
        {
            bs_write1( s, 0 );  /* adaptive_ref_pic_marking_mode_flag */
        }
    }

    if( sh->pps->b_cabac && sh->i_type != SLICE_TYPE_I )
    {
        bs_write_ue( s, sh->i_cabac_init_idc );
    }
    bs_write_se( s, sh->i_qp_delta );      /* slice qp delta */

    if( sh->pps->b_deblocking_filter_control )
    {
        bs_write_ue( s, sh->i_disable_deblocking_filter_idc );
        if( sh->i_disable_deblocking_filter_idc != 1 )
        {
            bs_write_se( s, sh->i_alpha_c0_offset >> 1 );
            bs_write_se( s, sh->i_beta_offset >> 1 );
        }
    }
}

/* If we are within a reasonable distance of the end of the memory allocated for the bitstream, */
/* reallocate, adding an arbitrary amount of space (100 kilobytes). */
static void x264_bitstream_check_buffer( x264_t *h )
{
    if( ( h->param.b_cabac && (h->cabac.p_end - h->cabac.p < 2500) )
     || ( h->out.bs.p_end - h->out.bs.p < 2500 ) )
    {
        uint8_t *bs_bak = h->out.p_bitstream;
        intptr_t delta;
        int i;

        h->out.i_bitstream += 100000;
        h->out.p_bitstream = x264_realloc( h->out.p_bitstream, h->out.i_bitstream );
        delta = h->out.p_bitstream - bs_bak;

        h->out.bs.p_start += delta;
        h->out.bs.p += delta;
        h->out.bs.p_end = h->out.p_bitstream + h->out.i_bitstream;

        h->cabac.p_start += delta;
        h->cabac.p += delta;
        h->cabac.p_end = h->out.p_bitstream + h->out.i_bitstream;

        for( i = 0; i <= h->out.i_nal; i++ )
            h->out.nal[i].p_payload += delta;
    }
}

/****************************************************************************
 *
 ****************************************************************************
 ****************************** External API*********************************
 ****************************************************************************
 *
 ****************************************************************************/

static int x264_validate_parameters( x264_t *h )
{
#ifdef HAVE_MMX
    if( !(x264_cpu_detect() & X264_CPU_MMXEXT) )
    {
        x264_log( h, X264_LOG_ERROR, "your cpu does not support MMXEXT, but x264 was compiled with asm support\n");
        x264_log( h, X264_LOG_ERROR, "to run x264, recompile without asm support (configure --disable-asm)\n");
        return -1;
    }
#endif
    if( h->param.i_width <= 0 || h->param.i_height <= 0 )
    {
        x264_log( h, X264_LOG_ERROR, "invalid width x height (%dx%d)\n",
                  h->param.i_width, h->param.i_height );
        return -1;
    }

    if( h->param.i_width % 2 || h->param.i_height % 2 )
    {
        x264_log( h, X264_LOG_ERROR, "width or height not divisible by 2 (%dx%d)\n",
                  h->param.i_width, h->param.i_height );
        return -1;
    }
    if( h->param.i_csp != X264_CSP_I420 )
    {
        x264_log( h, X264_LOG_ERROR, "invalid CSP (only I420 supported)\n" );
        return -1;
    }

    if( h->param.i_threads == 0 )
        h->param.i_threads = x264_cpu_num_processors() * 3/2;
    h->param.i_threads = x264_clip3( h->param.i_threads, 1, X264_THREAD_MAX );
    if( h->param.i_threads > 1 )
    {
#ifndef HAVE_PTHREAD
        x264_log( h, X264_LOG_WARNING, "not compiled with pthread support!\n");
        h->param.i_threads = 1;
#else
        if( h->param.i_scenecut_threshold >= 0 )
            h->param.b_pre_scenecut = 1;
#endif
    }

    if( h->param.b_interlaced )
    {
        if( h->param.analyse.i_me_method >= X264_ME_ESA )
        {
            x264_log( h, X264_LOG_WARNING, "interlace + me=esa is not implemented\n" );
            h->param.analyse.i_me_method = X264_ME_UMH;
        }
        if( h->param.analyse.i_direct_mv_pred > X264_DIRECT_PRED_SPATIAL )
        {
            x264_log( h, X264_LOG_WARNING, "interlace + direct=temporal is not implemented\n" );
            h->param.analyse.i_direct_mv_pred = X264_DIRECT_PRED_SPATIAL;
        }
    }

    if( h->param.rc.i_rc_method < 0 || h->param.rc.i_rc_method > 2 )
    {
        x264_log( h, X264_LOG_ERROR, "no ratecontrol method specified\n" );
        return -1;
    }
    h->param.rc.f_rf_constant = x264_clip3f( h->param.rc.f_rf_constant, 0, 51 );
    h->param.rc.i_qp_constant = x264_clip3( h->param.rc.i_qp_constant, 0, 51 );
    if( h->param.rc.i_rc_method == X264_RC_CRF )
        h->param.rc.i_qp_constant = h->param.rc.f_rf_constant;
    if( (h->param.rc.i_rc_method == X264_RC_CQP || h->param.rc.i_rc_method == X264_RC_CRF)
        && h->param.rc.i_qp_constant == 0 )
    {
        h->mb.b_lossless = 1;
        h->param.i_cqm_preset = X264_CQM_FLAT;
        h->param.psz_cqm_file = NULL;
        h->param.rc.i_rc_method = X264_RC_CQP;
        h->param.rc.f_ip_factor = 1;
        h->param.rc.f_pb_factor = 1;
        h->param.analyse.b_psnr = 0;
        h->param.analyse.b_ssim = 0;
        h->param.analyse.i_chroma_qp_offset = 0;
        h->param.analyse.i_trellis = 0;
        h->param.analyse.b_fast_pskip = 0;
        h->param.analyse.i_noise_reduction = 0;
        h->param.analyse.f_psy_rd = 0;
        h->param.i_bframe = 0;
        /* 8x8dct is not useful at all in CAVLC lossless */
        if( !h->param.b_cabac )
            h->param.analyse.b_transform_8x8 = 0;
    }
    if( h->param.rc.i_rc_method == X264_RC_CQP )
    {
        float qp_p = h->param.rc.i_qp_constant;
        float qp_i = qp_p - 6*log(h->param.rc.f_ip_factor)/log(2);
        float qp_b = qp_p + 6*log(h->param.rc.f_pb_factor)/log(2);
        h->param.rc.i_qp_min = x264_clip3( (int)(X264_MIN3( qp_p, qp_i, qp_b )), 0, 51 );
        h->param.rc.i_qp_max = x264_clip3( (int)(X264_MAX3( qp_p, qp_i, qp_b ) + .999), 0, 51 );
        h->param.rc.i_aq_mode = 0;
    }
    h->param.rc.i_qp_max = x264_clip3( h->param.rc.i_qp_max, 0, 51 );
    h->param.rc.i_qp_min = x264_clip3( h->param.rc.i_qp_min, 0, h->param.rc.i_qp_max );

    if( ( h->param.i_width % 16 || h->param.i_height % 16 )
        && h->param.i_height != 1080 && !h->mb.b_lossless )
    {
        // There's nothing special about 1080 in that the warning still applies to it,
        // but chances are the user can't help it if his content is already 1080p,
        // so there's no point in warning in that case.
        x264_log( h, X264_LOG_WARNING,
                  "width or height not divisible by 16 (%dx%d), compression will suffer.\n",
                  h->param.i_width, h->param.i_height );
    }

    h->param.i_frame_reference = x264_clip3( h->param.i_frame_reference, 1, 16 );
    if( h->param.i_keyint_max <= 0 )
        h->param.i_keyint_max = 1;
    h->param.i_keyint_min = x264_clip3( h->param.i_keyint_min, 1, h->param.i_keyint_max/2+1 );
    if( !h->param.analyse.i_subpel_refine && h->param.analyse.i_direct_mv_pred > X264_DIRECT_PRED_SPATIAL )
    {
        x264_log( h, X264_LOG_WARNING, "subme=0 + direct=temporal is not supported\n" );
        h->param.analyse.i_direct_mv_pred = X264_DIRECT_PRED_SPATIAL;
    }
    h->param.i_bframe = x264_clip3( h->param.i_bframe, 0, X264_BFRAME_MAX );
    h->param.i_bframe_bias = x264_clip3( h->param.i_bframe_bias, -90, 100 );
    h->param.b_bframe_pyramid = h->param.b_bframe_pyramid && h->param.i_bframe > 1;
    if( !h->param.i_bframe )
        h->param.i_bframe_adaptive = X264_B_ADAPT_NONE;
    h->param.analyse.b_weighted_bipred = h->param.analyse.b_weighted_bipred && h->param.i_bframe > 0;
    h->mb.b_direct_auto_write = h->param.analyse.i_direct_mv_pred == X264_DIRECT_PRED_AUTO
                                && h->param.i_bframe
                                && ( h->param.rc.b_stat_write || !h->param.rc.b_stat_read );
    if( h->param.i_scenecut_threshold < 0 )
        h->param.b_pre_scenecut = 0;

    h->param.i_deblocking_filter_alphac0 = x264_clip3( h->param.i_deblocking_filter_alphac0, -6, 6 );
    h->param.i_deblocking_filter_beta    = x264_clip3( h->param.i_deblocking_filter_beta, -6, 6 );
    h->param.analyse.i_luma_deadzone[0] = x264_clip3( h->param.analyse.i_luma_deadzone[0], 0, 32 );
    h->param.analyse.i_luma_deadzone[1] = x264_clip3( h->param.analyse.i_luma_deadzone[1], 0, 32 );

    h->param.i_cabac_init_idc = x264_clip3( h->param.i_cabac_init_idc, 0, 2 );

    if( h->param.i_cqm_preset < X264_CQM_FLAT || h->param.i_cqm_preset > X264_CQM_CUSTOM )
        h->param.i_cqm_preset = X264_CQM_FLAT;

    if( h->param.analyse.i_me_method < X264_ME_DIA ||
        h->param.analyse.i_me_method > X264_ME_TESA )
        h->param.analyse.i_me_method = X264_ME_HEX;
    if( h->param.analyse.i_me_range < 4 )
        h->param.analyse.i_me_range = 4;
    if( h->param.analyse.i_me_range > 16 && h->param.analyse.i_me_method <= X264_ME_HEX )
        h->param.analyse.i_me_range = 16;
    if( h->param.analyse.i_me_method == X264_ME_TESA &&
        (h->mb.b_lossless || h->param.analyse.i_subpel_refine <= 1) )
        h->param.analyse.i_me_method = X264_ME_ESA;
    h->param.analyse.i_subpel_refine = x264_clip3( h->param.analyse.i_subpel_refine, 0, 9 );
    h->param.analyse.b_mixed_references = h->param.analyse.b_mixed_references && h->param.i_frame_reference > 1;
    h->param.analyse.inter &= X264_ANALYSE_PSUB16x16|X264_ANALYSE_PSUB8x8|X264_ANALYSE_BSUB16x16|
                              X264_ANALYSE_I4x4|X264_ANALYSE_I8x8;
    h->param.analyse.intra &= X264_ANALYSE_I4x4|X264_ANALYSE_I8x8;
    if( !(h->param.analyse.inter & X264_ANALYSE_PSUB16x16) )
        h->param.analyse.inter &= ~X264_ANALYSE_PSUB8x8;
    if( !h->param.analyse.b_transform_8x8 )
    {
        h->param.analyse.inter &= ~X264_ANALYSE_I8x8;
        h->param.analyse.intra &= ~X264_ANALYSE_I8x8;
    }
    h->param.analyse.i_chroma_qp_offset = x264_clip3(h->param.analyse.i_chroma_qp_offset, -12, 12);
    if( !h->param.b_cabac )
        h->param.analyse.i_trellis = 0;
    h->param.analyse.i_trellis = x264_clip3( h->param.analyse.i_trellis, 0, 2 );
    if( !h->param.analyse.i_trellis )
        h->param.analyse.f_psy_trellis = 0;
    h->param.analyse.f_psy_rd = x264_clip3f( h->param.analyse.f_psy_rd, 0, 10 );
    h->param.analyse.f_psy_trellis = x264_clip3f( h->param.analyse.f_psy_trellis, 0, 10 );
    if( h->param.analyse.i_subpel_refine < 6 )
        h->param.analyse.f_psy_rd = 0;
    h->mb.i_psy_rd = FIX8( h->param.analyse.f_psy_rd );
    /* Psy RDO increases overall quantizers to improve the quality of luma--this indirectly hurts chroma quality */
    /* so we lower the chroma QP offset to compensate */
    /* This can be triggered repeatedly on multiple calls to parameter_validate, but since encoding
     * uses the pps chroma qp offset not the param chroma qp offset, this is not a problem. */
    if( h->mb.i_psy_rd )
        h->param.analyse.i_chroma_qp_offset -= h->param.analyse.f_psy_rd < 0.25 ? 1 : 2;
    h->mb.i_psy_trellis = FIX8( h->param.analyse.f_psy_trellis / 4 );
    /* Psy trellis has a similar effect. */
    if( h->mb.i_psy_trellis )
        h->param.analyse.i_chroma_qp_offset -= h->param.analyse.f_psy_trellis < 0.25 ? 1 : 2;
    else
        h->mb.i_psy_trellis = 0;
    h->param.analyse.i_chroma_qp_offset = x264_clip3(h->param.analyse.i_chroma_qp_offset, -12, 12);
    h->param.rc.i_aq_mode = x264_clip3( h->param.rc.i_aq_mode, 0, 1 );
    h->param.rc.f_aq_strength = x264_clip3f( h->param.rc.f_aq_strength, 0, 3 );
    if( h->param.rc.f_aq_strength == 0 )
        h->param.rc.i_aq_mode = 0;
    h->param.analyse.i_noise_reduction = x264_clip3( h->param.analyse.i_noise_reduction, 0, 1<<16 );

    {
        const x264_level_t *l = x264_levels;
        if( h->param.i_level_idc < 0 )
        {
            if( h->param.rc.i_rc_method == X264_RC_ABR && h->param.rc.i_vbv_buffer_size <= 0 )
                h->param.rc.i_vbv_max_bitrate = h->param.rc.i_bitrate * 2;
            h->sps = h->sps_array;
            x264_sps_init( h->sps, h->param.i_sps_id, &h->param );
            do h->param.i_level_idc = l->level_idc;
                while( l[1].level_idc && x264_validate_levels( h, 0 ) && l++ );
            if( h->param.rc.i_vbv_buffer_size <= 0 )
                h->param.rc.i_vbv_max_bitrate = 0;
        }
        else
        {
            while( l->level_idc && l->level_idc != h->param.i_level_idc )
                l++;
            if( l->level_idc == 0 )
            {
                x264_log( h, X264_LOG_ERROR, "invalid level_idc: %d\n", h->param.i_level_idc );
                return -1;
            }
        }
        if( h->param.analyse.i_mv_range <= 0 )
            h->param.analyse.i_mv_range = l->mv_range >> h->param.b_interlaced;
        else
            h->param.analyse.i_mv_range = x264_clip3(h->param.analyse.i_mv_range, 32, 512 >> h->param.b_interlaced);
    }

    if( h->param.i_threads > 1 )
    {
        int r = h->param.analyse.i_mv_range_thread;
        int r2;
        if( r <= 0 )
        {
            // half of the available space is reserved and divided evenly among the threads,
            // the rest is allocated to whichever thread is far enough ahead to use it.
            // reserving more space increases quality for some videos, but costs more time
            // in thread synchronization.
            int max_range = (h->param.i_height + X264_THREAD_HEIGHT) / h->param.i_threads - X264_THREAD_HEIGHT;
            r = max_range / 2;
        }
        r = X264_MAX( r, h->param.analyse.i_me_range );
        r = X264_MIN( r, h->param.analyse.i_mv_range );
        // round up to use the whole mb row
        r2 = (r & ~15) + ((-X264_THREAD_HEIGHT) & 15);
        if( r2 < r )
            r2 += 16;
        x264_log( h, X264_LOG_DEBUG, "using mv_range_thread = %d\n", r2 );
        h->param.analyse.i_mv_range_thread = r2;
    }

    if( h->param.rc.f_qblur < 0 )
        h->param.rc.f_qblur = 0;
    if( h->param.rc.f_complexity_blur < 0 )
        h->param.rc.f_complexity_blur = 0;

    h->param.i_sps_id &= 31;

    if( h->param.i_log_level < X264_LOG_INFO )
    {
        h->param.analyse.b_psnr = 0;
        h->param.analyse.b_ssim = 0;
    }

    /* ensure the booleans are 0 or 1 so they can be used in math */
#define BOOLIFY(x) h->param.x = !!h->param.x
    BOOLIFY( b_cabac );
    BOOLIFY( b_deblocking_filter );
    BOOLIFY( b_interlaced );
    BOOLIFY( analyse.b_transform_8x8 );
    BOOLIFY( analyse.b_chroma_me );
    BOOLIFY( analyse.b_fast_pskip );
    BOOLIFY( rc.b_stat_write );
    BOOLIFY( rc.b_stat_read );
#undef BOOLIFY

    return 0;
}

static void mbcmp_init( x264_t *h )
{
    int satd = !h->mb.b_lossless && h->param.analyse.i_subpel_refine > 1;
    memcpy( h->pixf.mbcmp, satd ? h->pixf.satd : h->pixf.sad_aligned, sizeof(h->pixf.mbcmp) );
    memcpy( h->pixf.mbcmp_unaligned, satd ? h->pixf.satd : h->pixf.sad, sizeof(h->pixf.mbcmp_unaligned) );
    h->pixf.intra_mbcmp_x3_16x16 = satd ? h->pixf.intra_satd_x3_16x16 : h->pixf.intra_sad_x3_16x16;
    satd &= h->param.analyse.i_me_method == X264_ME_TESA;
    memcpy( h->pixf.fpelcmp, satd ? h->pixf.satd : h->pixf.sad, sizeof(h->pixf.fpelcmp) );
    memcpy( h->pixf.fpelcmp_x3, satd ? h->pixf.satd_x3 : h->pixf.sad_x3, sizeof(h->pixf.fpelcmp_x3) );
    memcpy( h->pixf.fpelcmp_x4, satd ? h->pixf.satd_x4 : h->pixf.sad_x4, sizeof(h->pixf.fpelcmp_x4) );
}

/****************************************************************************
 * x264_encoder_open:
 ****************************************************************************/
x264_t *x264_encoder_open   ( x264_param_t *param )
{
    x264_t *h = x264_malloc( sizeof( x264_t ) );
    char buf[1000], *p;
    int i;

    memset( h, 0, sizeof( x264_t ) );

    /* Create a copy of param */
    memcpy( &h->param, param, sizeof( x264_param_t ) );
	// ����������
    if( x264_validate_parameters( h ) < 0 )
    {
        x264_free( h );
        return NULL;
    }

    if( h->param.psz_cqm_file )
        if( x264_cqm_parse_file( h, h->param.psz_cqm_file ) < 0 )
        {
            x264_free( h );
            return NULL;
        }

    if( h->param.rc.psz_stat_out )
        h->param.rc.psz_stat_out = strdup( h->param.rc.psz_stat_out );
    if( h->param.rc.psz_stat_in )
        h->param.rc.psz_stat_in = strdup( h->param.rc.psz_stat_in );

    /* VUI */
    if( h->param.vui.i_sar_width > 0 && h->param.vui.i_sar_height > 0 )
    {
        int i_w = param->vui.i_sar_width;
        int i_h = param->vui.i_sar_height;

        x264_reduce_fraction( &i_w, &i_h );

        while( i_w > 65535 || i_h > 65535 )
        {
            i_w /= 2;
            i_h /= 2;
        }

        h->param.vui.i_sar_width = 0;
        h->param.vui.i_sar_height = 0;
        if( i_w == 0 || i_h == 0 )
        {
            x264_log( h, X264_LOG_WARNING, "cannot create valid sample aspect ratio\n" );
        }
        else
        {
            x264_log( h, X264_LOG_INFO, "using SAR=%d/%d\n", i_w, i_h );
            h->param.vui.i_sar_width = i_w;
            h->param.vui.i_sar_height = i_h;
        }
    }

    x264_reduce_fraction( &h->param.i_fps_num, &h->param.i_fps_den );

    /* Init x264_t */
    h->i_frame = 0;
    h->i_frame_num = 0;
    h->i_idr_pic_id = 0;

    h->sps = &h->sps_array[0];
	// ���в�����
    x264_sps_init( h->sps, h->param.i_sps_id, &h->param );

    h->pps = &h->pps_array[0];
	// ͼ�������
    x264_pps_init( h->pps, h->param.i_sps_id, &h->param, h->sps);

    x264_validate_levels( h, 1 );

    if( x264_cqm_init( h ) < 0 )
    {
        x264_free( h );
        return NULL;
    }

    h->mb.i_mb_count = h->sps->i_mb_width * h->sps->i_mb_height;

    /* Init frames. */
    if( h->param.i_bframe_adaptive == X264_B_ADAPT_TRELLIS )
        h->frames.i_delay = X264_MAX(h->param.i_bframe,3)*4 + h->param.i_threads - 1;
    else
        h->frames.i_delay = h->param.i_bframe + h->param.i_threads - 1;
    h->frames.i_max_ref0 = h->param.i_frame_reference;
    h->frames.i_max_ref1 = h->sps->vui.i_num_reorder_frames;
    h->frames.i_max_dpb  = h->sps->vui.i_max_dec_frame_buffering;
    h->frames.b_have_lowres = !h->param.rc.b_stat_read
        && ( h->param.rc.i_rc_method == X264_RC_ABR
          || h->param.rc.i_rc_method == X264_RC_CRF
          || h->param.i_bframe_adaptive
          || h->param.b_pre_scenecut );
    h->frames.b_have_lowres |= (h->param.rc.b_stat_read && h->param.rc.i_vbv_buffer_size > 0);
    h->frames.b_have_sub8x8_esa = !!(h->param.analyse.inter & X264_ANALYSE_PSUB8x8);

    h->frames.i_last_idr = - h->param.i_keyint_max;
    h->frames.i_input    = 0;
    h->frames.last_nonb  = NULL;

    h->i_ref0 = 0;
    h->i_ref1 = 0;

    h->chroma_qp_table = i_chroma_qp_table + 12 + h->pps->i_chroma_qp_index_offset;

    x264_rdo_init( );

    /* init CPU functions */
	// ���ڳ�ʼ��Intra16x16֡��Ԥ��
    x264_predict_16x16_init( h->param.cpu, h->predict_16x16 );
    x264_predict_8x8c_init( h->param.cpu, h->predict_8x8c );
    x264_predict_8x8_init( h->param.cpu, h->predict_8x8, &h->predict_8x8_filter );
    x264_predict_4x4_init( h->param.cpu, h->predict_4x4 );
    if( !h->param.b_cabac );
        x264_init_vlc_tables();
	// ���ؼ����йصĺ���SAD��SATD��SSD��SSIM�ȵ�
	/* H.264��ʹ��SAD��SATD���к��Ԥ��ģʽ���жϡ����ڵı�����ʹ��SAD���м��㣬
	 * ���ڵı�������ʹ��SATD���м��㡣Ϊʲôʹ��SATD����ʹ��SAD�أ��ؼ�ԭ�����ڱ���֮
	 * �������Ĵ�С�Ǻ�ͼ���DCT�任��Ƶ����Ϣ������صģ����ͱ任ǰ��ʱ����Ϣ������СһЩ��
	 * SADֻ�ܷ�Ӧʱ����Ϣ��SATDȴ���Է�ӳƵ����Ϣ�����Ҽ��㸴�Ӷ�Ҳ����DCT�任��
	 * ����ǱȽϺ��ʵ�ģʽѡ������ݡ�
	 **/
    x264_pixel_init( h->param.cpu, &h->pixf );
    x264_dct_init( h->param.cpu, &h->dctf );
    x264_zigzag_init( h->param.cpu, &h->zigzagf, h->param.b_interlaced );
	// ��ʼ���˶�������صĻ�ຯ��
    x264_mc_init( h->param.cpu, &h->mc );
    x264_quant_init( h, h->param.cpu, &h->quantf );
    x264_deblock_init( h->param.cpu, &h->loopf );
    x264_dct_init_weights();



	//���������رȽϵ�ʱ����SAD����SATD  ����ʧ��������Ҫ����lijun
    mbcmp_init( h );

    p = buf + sprintf( buf, "using cpu capabilities:" );
    for( i=0; x264_cpu_names[i].flags; i++ )
    {
        if( !strcmp(x264_cpu_names[i].name, "SSE2")
            && param->cpu & (X264_CPU_SSE2_IS_FAST|X264_CPU_SSE2_IS_SLOW) )
            continue;
        if( !strcmp(x264_cpu_names[i].name, "SSE3")
            && (param->cpu & X264_CPU_SSSE3 || !(param->cpu & X264_CPU_CACHELINE_64)) )
            continue;
        if( !strcmp(x264_cpu_names[i].name, "SSE4.1")
            && (param->cpu & X264_CPU_SSE42) )
            continue;
        if( (param->cpu & x264_cpu_names[i].flags) == x264_cpu_names[i].flags
            && (!i || x264_cpu_names[i].flags != x264_cpu_names[i-1].flags) )
            p += sprintf( p, " %s", x264_cpu_names[i].name );
    }
    if( !param->cpu )
        p += sprintf( p, " none!" );
    x264_log( h, X264_LOG_INFO, "%s\n", buf );

    h->out.i_nal = 0;
    h->out.i_bitstream = X264_MAX( 1000000, h->param.i_width * h->param.i_height * 4
        * ( h->param.rc.i_rc_method == X264_RC_ABR ? pow( 0.95, h->param.rc.i_qp_min )
          : pow( 0.95, h->param.rc.i_qp_constant ) * X264_MAX( 1, h->param.rc.f_ip_factor )));

    h->thread[0] = h;
    h->i_thread_num = 0;
    for( i = 1; i < h->param.i_threads; i++ )
        h->thread[i] = x264_malloc( sizeof(x264_t) );

    for( i = 0; i < h->param.i_threads; i++ )
    {
        if( i > 0 )
            *h->thread[i] = *h;
        h->thread[i]->fdec = x264_frame_pop_unused( h );
        h->thread[i]->out.p_bitstream = x264_malloc( h->out.i_bitstream );
        if( x264_macroblock_cache_init( h->thread[i] ) < 0 )
            return NULL;
    }
	//�������ʿ���
    if( x264_ratecontrol_new( h ) < 0 )
        return NULL;

    if( h->param.psz_dump_yuv )
    {
        /* create or truncate the reconstructed video file */
        FILE *f = fopen( h->param.psz_dump_yuv, "w" );
        if( f )
            fclose( f );
        else
        {
            x264_log( h, X264_LOG_ERROR, "can't write to fdec.yuv\n" );
            x264_free( h );
            return NULL;
        }
    }

    x264_log( h, X264_LOG_INFO, "profile %s, level %d.%d\n",
        h->sps->i_profile_idc == PROFILE_BASELINE ? "Baseline" :
        h->sps->i_profile_idc == PROFILE_MAIN ? "Main" :
        h->sps->i_profile_idc == PROFILE_HIGH ? "High" :
        "High 4:4:4 Predictive", h->sps->i_level_idc/10, h->sps->i_level_idc%10 );

    return h;
}

/****************************************************************************
 * x264_encoder_reconfig:
 ****************************************************************************/
int x264_encoder_reconfig( x264_t *h, x264_param_t *param )
{
#define COPY(var) h->param.var = param->var
    COPY( i_frame_reference ); // but never uses more refs than initially specified
    COPY( i_bframe_bias );
    if( h->param.i_scenecut_threshold >= 0 && param->i_scenecut_threshold >= 0 )
        COPY( i_scenecut_threshold ); // can't turn it on or off, only vary the threshold
    COPY( b_deblocking_filter );
    COPY( i_deblocking_filter_alphac0 );
    COPY( i_deblocking_filter_beta );
    COPY( analyse.intra );
    COPY( analyse.inter );
    COPY( analyse.i_direct_mv_pred );
    /* Scratch buffer prevents me_range from being increased for esa/tesa */
    if( h->param.analyse.i_me_method < X264_ME_ESA || param->analyse.i_me_range < h->param.analyse.i_me_range )
        COPY( analyse.i_me_range );
    COPY( analyse.i_noise_reduction );
    /* We can't switch out of subme=0 during encoding. */
    if( h->param.analyse.i_subpel_refine )
        COPY( analyse.i_subpel_refine );
    COPY( analyse.i_trellis );
    COPY( analyse.b_chroma_me );
    COPY( analyse.b_dct_decimate );
    COPY( analyse.b_fast_pskip );
    COPY( analyse.b_mixed_references );
    COPY( analyse.f_psy_rd );
    COPY( analyse.f_psy_trellis );
    // can only twiddle these if they were enabled to begin with:
    if( h->param.analyse.i_me_method >= X264_ME_ESA || param->analyse.i_me_method < X264_ME_ESA )
        COPY( analyse.i_me_method );
    if( h->param.analyse.i_me_method >= X264_ME_ESA && !h->frames.b_have_sub8x8_esa )
        h->param.analyse.inter &= ~X264_ANALYSE_PSUB8x8;
    if( h->pps->b_transform_8x8_mode )
        COPY( analyse.b_transform_8x8 );
    if( h->frames.i_max_ref1 > 1 )
        COPY( b_bframe_pyramid );
#undef COPY

    mbcmp_init( h );

    return x264_validate_parameters( h );
}

/* internal usage */
static void x264_nal_start( x264_t *h, int i_type, int i_ref_idc )
{
    x264_nal_t *nal = &h->out.nal[h->out.i_nal];

    nal->i_ref_idc = i_ref_idc;
    nal->i_type    = i_type;

    nal->i_payload= 0;
    nal->p_payload= &h->out.p_bitstream[bs_pos( &h->out.bs ) / 8];
}
static void x264_nal_end( x264_t *h )
{
    x264_nal_t *nal = &h->out.nal[h->out.i_nal];
    nal->i_payload = &h->out.p_bitstream[bs_pos( &h->out.bs ) / 8] - nal->p_payload;
    h->out.i_nal++;
}

/****************************************************************************
 * x264_encoder_headers: ����ļ�ͷ��SPS��PPS��SEI��
 ****************************************************************************/
int x264_encoder_headers( x264_t *h, x264_nal_t **pp_nal, int *pi_nal )
{
    /* init bitstream context */
    h->out.i_nal = 0;
    bs_init( &h->out.bs, h->out.p_bitstream, h->out.i_bitstream );

    /* Put SPS and PPS */
    if( h->i_frame == 0 )
    {
        /* identify ourself */
        x264_nal_start( h, NAL_SEI, NAL_PRIORITY_DISPOSABLE );
        x264_sei_version_write( h, &h->out.bs );
        x264_nal_end( h );

        /* generate sequence parameters */
        x264_nal_start( h, NAL_SPS, NAL_PRIORITY_HIGHEST );
        x264_sps_write( &h->out.bs, h->sps );
        x264_nal_end( h );

        /* generate picture parameters */
        x264_nal_start( h, NAL_PPS, NAL_PRIORITY_HIGHEST );
        x264_pps_write( &h->out.bs, h->pps );
        x264_nal_end( h );
    }
    /* now set output*/
    *pi_nal = h->out.i_nal;
    *pp_nal = &h->out.nal[0];
    h->out.i_nal = 0;

    return 0;
}

static inline void x264_reference_build_list( x264_t *h, int i_poc )//�ο�֡�Ľ���lijun
{
    int i;
    int b_ok;

    /* build ref list 0/1 */
    h->i_ref0 = 0;
    h->i_ref1 = 0;
    for( i = 0; h->frames.reference[i]; i++ )
    {
        if( h->frames.reference[i]->i_poc < i_poc )
        {
            h->fref0[h->i_ref0++] = h->frames.reference[i];
        }
        else if( h->frames.reference[i]->i_poc > i_poc )
        {
            h->fref1[h->i_ref1++] = h->frames.reference[i];
        }
    }

    /* ð������ */
    do
    {
        b_ok = 1;
        for( i = 0; i < h->i_ref0 - 1; i++ )
        {
            if( h->fref0[i]->i_poc < h->fref0[i+1]->i_poc ) // ���i�����ȼ�С��i+1�򽻻�λ��
            {
                XCHG( x264_frame_t*, h->fref0[i], h->fref0[i+1] );
                b_ok = 0;
                break;
            }
        }
    } while( !b_ok );
    /* Order ref1 from lower to higher poc (bubble sort) for B-frame */
    do
    {
        b_ok = 1;
        for( i = 0; i < h->i_ref1 - 1; i++ )
        {
            if( h->fref1[i]->i_poc > h->fref1[i+1]->i_poc )
            {
                XCHG( x264_frame_t*, h->fref1[i], h->fref1[i+1] );
                b_ok = 0;
                break;
            }
        }
    } while( !b_ok );

    /* In the standard, a P-frame's ref list is sorted by frame_num.
     * We use POC, but check whether explicit reordering is needed */
    h->b_ref_reorder[0] =
    h->b_ref_reorder[1] = 0;
    if( h->sh.i_type == SLICE_TYPE_P )
    {
        for( i = 0; i < h->i_ref0 - 1; i++ )
            if( h->fref0[i]->i_frame_num < h->fref0[i+1]->i_frame_num )
            {
                h->b_ref_reorder[0] = 1;
                break;
            }
    }

    h->i_ref1 = X264_MIN( h->i_ref1, h->frames.i_max_ref1 );
    h->i_ref0 = X264_MIN( h->i_ref0, h->frames.i_max_ref0 );
    h->i_ref0 = X264_MIN( h->i_ref0, h->param.i_frame_reference ); // if reconfig() has lowered the limit
    assert( h->i_ref0 + h->i_ref1 <= 16 );
    h->mb.pic.i_fref[0] = h->i_ref0;
    h->mb.pic.i_fref[1] = h->i_ref1;
}

static void x264_fdec_filter_row( x264_t *h, int mb_y )
{
    /* mb_y is the mb to be encoded next, not the mb to be filtered here */
    int b_hpel = h->fdec->b_kept_as_ref;
    int b_deblock = !h->sh.i_disable_deblocking_filter_idc;
    int b_end = mb_y == h->sps->i_mb_height;
    int min_y = mb_y - (1 << h->sh.b_mbaff);
    int max_y = b_end ? h->sps->i_mb_height : mb_y;
    b_deblock &= b_hpel || h->param.psz_dump_yuv;
    if( mb_y & h->sh.b_mbaff )
        return;
    if( min_y < 0 )
        return;

    if( !b_end )//��ǰ֡�������
    {
        int i, j;
        for( j=0; j<=h->sh.b_mbaff; j++ )
            for( i=0; i<3; i++ )
            {//��������һ�����ر��棬��������һ�к���������һ�����صĲο����أ���x264_macroblock_cache_load�ᵼ�뵽mb.pic.p_fdec��
                memcpy( h->mb.intra_border_backup[j][i],
                        h->fdec->plane[i] + ((mb_y*16 >> !!i) + j - 1 - h->sh.b_mbaff) * h->fdec->i_stride[i],
                        h->sps->i_mb_width*16 >> !!i );
            }
    }

    if( b_deblock )
    {
        int y;
        for( y = min_y; y < max_y; y += (1 << h->sh.b_mbaff) )
			//ȥ��ЧӦ�˲�����
            x264_frame_deblock_row( h, y );
    }

    if( b_hpel )
    {
        x264_frame_expand_border( h, h->fdec, min_y, b_end );
        if( h->param.analyse.i_subpel_refine )
        {
			// �����ز�ֵ
            x264_frame_filter( h, h->fdec, min_y, b_end );
            x264_frame_expand_border_filtered( h, h->fdec, min_y, b_end );
        }
    }

    if( h->param.i_threads > 1 && h->fdec->b_kept_as_ref )
    {
        x264_frame_cond_broadcast( h->fdec, mb_y*16 + (b_end ? 10000 : -(X264_THREAD_HEIGHT << h->sh.b_mbaff)) );
    }

    min_y = X264_MAX( min_y*16-8, 0 );
    max_y = b_end ? h->param.i_height : mb_y*16-8;

    if( h->param.analyse.b_psnr )
    {
        int i;
        for( i=0; i<3; i++ )
			// PSNR����
            h->stat.frame.i_ssd[i] +=
                x264_pixel_ssd_wxh( &h->pixf,
                    h->fdec->plane[i] + (min_y>>!!i) * h->fdec->i_stride[i], h->fdec->i_stride[i],
                    h->fenc->plane[i] + (min_y>>!!i) * h->fenc->i_stride[i], h->fenc->i_stride[i],
                    h->param.i_width >> !!i, (max_y-min_y) >> !!i );
    }

    if( h->param.analyse.b_ssim )
    {
        x264_emms();
        /* offset by 2 pixels to avoid alignment of ssim blocks with dct blocks,
         * and overlap by 4 */
        min_y += min_y == 0 ? 2 : -6;
		// SSIM����
        h->stat.frame.f_ssim +=
            x264_pixel_ssim_wxh( &h->pixf,
                h->fdec->plane[0] + 2+min_y*h->fdec->i_stride[0], h->fdec->i_stride[0],
                h->fenc->plane[0] + 2+min_y*h->fenc->i_stride[0], h->fenc->i_stride[0],
                h->param.i_width-2, max_y-min_y, h->scratch_buffer );
    }
}
/* ����ؽ�֡fdec�ǲ����ο���B֡����ֱ�ӷ��أ����fdec�Ǳ��ο���֡��
 * ������x264_frame_push()����֡����frames.reference[]���е�β����
 * ���frames.reference[]�Ѿ����ˣ�������x264_frame_shift()��x264_frame_push_unused()
 * ��frames.reference[]����ͷ����֡�ƶ���frames.unused[]���С�������������
 * x264_frame_pop_unused()��ȡһ���µ��ؽ�֡fdec��
 **/
static inline void x264_reference_update( x264_t *h )
{
    int i;

    if( h->fdec->i_frame >= 0 )
        h->i_frame++;
	//�������Ϊ�ο���֡
    if( !h->fdec->b_kept_as_ref )
    {
        if( h->param.i_threads > 1 )
        {
            x264_frame_push_unused( h, h->fdec );
            h->fdec = x264_frame_pop_unused( h );
        }
        return;
    }

    /* move lowres copy of the image to the ref frame */
    for( i = 0; i < 4; i++)
    {
		// ����
        XCHG( uint8_t*, h->fdec->lowres[i], h->fenc->lowres[i] );
        XCHG( uint8_t*, h->fdec->buffer_lowres[i], h->fenc->buffer_lowres[i] );
    }

    /* adaptive B decision needs a pointer, since it can't use the ref lists */
    if( h->sh.i_type != SLICE_TYPE_B )// ����ؽ�֡����B֡��������ķ�B֡��ֵΪ�ؽ�֡
        h->frames.last_nonb = h->fdec;

    /* �ؽ�֡����ο�֡�б�*/
    x264_frame_push( h->frames.reference, h->fdec );
    if( h->frames.reference[h->frames.i_max_dpb] ) //�б����ˣ���Ҫ�Ƴ�1֡  i_max_dpbĬ��Ϊ1��Ҳ����ֻ��һ֡�����ο�֡��
        x264_frame_push_unused( h, x264_frame_shift( h->frames.reference ) );
    h->fdec = x264_frame_pop_unused( h );//���³�ʼ���ؽ�֡fdec
}

static inline void x264_reference_reset( x264_t *h )
{
    while( h->frames.reference[0] )
        x264_frame_push_unused( h, x264_frame_pop( h->frames.reference ) );
    h->fdec->i_poc =
    h->fenc->i_poc = 0;
}

static inline void x264_slice_init( x264_t *h, int i_nal_type, int i_global_qp )
{
    /* ------------------------ Create slice header  ----------------------- */
    if( i_nal_type == NAL_SLICE_IDR )
    {
        x264_slice_header_init( h, &h->sh, h->sps, h->pps, h->i_idr_pic_id, h->i_frame_num, i_global_qp );

        /* increment id */
        h->i_idr_pic_id = ( h->i_idr_pic_id + 1 ) % 65536;
    }
    else
    {
        x264_slice_header_init( h, &h->sh, h->sps, h->pps, -1, h->i_frame_num, i_global_qp );

        /* always set the real higher num of ref frame used */
        h->sh.b_num_ref_idx_override = 1;
        h->sh.i_num_ref_idx_l0_active = h->i_ref0 <= 0 ? 1 : h->i_ref0;
        h->sh.i_num_ref_idx_l1_active = h->i_ref1 <= 0 ? 1 : h->i_ref1;
    }

    h->fdec->i_frame_num = h->sh.i_frame_num;

    if( h->sps->i_poc_type == 0 )
    {
        h->sh.i_poc_lsb = h->fdec->i_poc & ( (1 << h->sps->i_log2_max_poc_lsb) - 1 );
        h->sh.i_delta_poc_bottom = 0;   /* XXX won't work for field */
    }
    else if( h->sps->i_poc_type == 1 )
    {
        /* FIXME TODO FIXME */
    }
    else
    {
        /* Nothing to do ? */
    }

    x264_macroblock_slice_init( h );
}

//�ж��Ƿ�Ϊ0��1  lijun
#define IS_ZeroOrOne(x) ((x==0) ? 1 : 0)
//#define IS_ZeroOrOne(x) ((x==0)||(x==1)) ? 1 : 0

static void x264_slice_write( x264_t *h )
{
	//ÿ֡��һ�α���ʱ��ʼ��info�������Ϣ
	if ((h->info.embed_flag) && (h->info.firstTime))
	{
		//h->info.b_frame_used = 0;//�Ƿ�ʹ��
		//��ʼ����Ϣ�����йص�ÿ֡��ͳ������ ih lijun
		h->info.stat.num_non_optimal_mv = 0;
		h->info.stat.num_optimal_1_neighbor = 0;
		h->info.stat.num_optimal_2_neighbor = 0;
		h->info.stat.num_p_block = 0;
		h->info.stat.num_error_pos = 0;
		h->info.stat.num_message = 0;

		//��ʼ������Ƿ���Ƕ��ı��λ
		for (int i = 0; i < 396; i++)
		{
			h->info.cache[i].used=0;
			h->info.cache[i].i_type = 6;//ȫ����ʼ��ΪP_Skip���ͣ���Ҫ��
			memset(h->info.cache[i].mv, 0, sizeof(int16_t) * 16*2);
			memset(h->info.cache[i].pskip_mv_, 0, sizeof(int16_t) * 2);
			memset(h->info.cache[i].ref, -2, sizeof(int8_t) * 16);
		}
		
		h->info.i_mv_no = 0;
		h->info.num_mv_modify_real = 0;
		h->info.num_p_skip_before_analysis= 0;
		h->info.num_p_skip_after_encode = 0;

		
	}


    int i_skip;
    int mb_xy, i_mb_x, i_mb_y;
	// ������ţ��Լ���Ŷ�Ӧ��x��y����
    int i, i_list, i_ref;

    /* init stats */
    memset( &h->stat.frame, 0, sizeof(h->stat.frame) );

    //////////////////////////////////////////////Slice ��ʼдһ��NALU
    
	x264_nal_start( h, h->i_nal_type, h->i_nal_ref_idc );

    /* Slice header ��� Slice Header*/
    x264_slice_header_write( &h->out.bs, &h->sh, h->i_nal_ref_idc );
    if( h->param.b_cabac )
    { //���ʹ��CABAC����Ҫ��ʼ��
        /* alignment needed */
        bs_align_1( &h->out.bs );

        /* init cabac */
        x264_cabac_context_init( &h->cabac, h->sh.i_type, h->sh.i_qp, h->sh.i_cabac_init_idc );
        x264_cabac_encode_init ( &h->cabac, h->out.bs.p, h->out.bs.p_end );
    }

    h->mb.i_last_qp = h->sh.i_qp;
    h->mb.i_last_dqp = 0;
    //���λ��-�����꣨��ʼֵ���������꣨��ʼֵ��
    i_mb_y = h->sh.i_first_mb / h->sps->i_mb_width;
    i_mb_x = h->sh.i_first_mb % h->sps->i_mb_width;
    i_skip = 0;
	// ������Ƭ�����е�MB
    while( (mb_xy = i_mb_x + i_mb_y * h->sps->i_mb_width) < h->sh.i_last_mb )
	{
		if (h->sh.i_type== SLICE_TYPE_P)//!h->info.firstTime && 
		{
			if (DEGUG_LIJUN==2)
			{
				printf("����ڣ�%d,%d��mb,��ǰ֡��%d����\n", i_mb_x, i_mb_y, mb_xy);
			}
		}
		/*if (mb_xy==395 && !h->info.firstTime) {   //��ʱ
			for (int i0 = 0; i0 <= 395; i0++)
			{
				printf("%d ", h->info.cache[i0].i_type);
			}
		}*/
		//�����š���i_mb_x��i_mb_y�������
        int mb_spos = bs_pos(&h->out.bs) + x264_cabac_pos(&h->cabac);

        if( i_mb_x == 0 )//һ�еĿ�ʼ��ȥ��ЧӦ�˲��������ز�ֵ��SSIM/PSNR�����
            x264_fdec_filter_row( h, i_mb_y );

        /* load cache ��Ҫ����ĺ������ݺ���Χ�ĺ�����Ϣ����������Ҫ�����桢��߿��ֵ*/
        x264_macroblock_cache_load( h, i_mb_x, i_mb_y );

		//��Ϣ���� ���  ֻ�ܷ���������ܷ���analyse�����skipģʽ�����ˣ�pskip_mv��x264_macroblock_cache_load����ã�
		*(uint32_t*)&h->info.cache[mb_xy].pskip_mv_ = *(uint32_t*)&h->mb.cache.pskip_mv;//������ܵ�pskipģʽ�µ�Ԥ��mv�����㸴����ʧ��ʱ�ã�

        /* analyse parameters
        ** Slice I: choose I_4x4 or I_16x16 mode
        ** Slice P: choose between using P mode or intra (4x4 or 16x16)
        ** ����ģ�顣��ģ�������֡��Ԥ��ģʽ�����Լ�֡���˶����Ƶȡ�ͬʱ���mb.pic.fdec�����ؽ�
		**/

		x264_macroblock_analyse( h );

		// һ֡������Ӧ�ý�������Ĺ����ˣ�ͬʱ����ʧ�棨����������stcǶ��,
		if (h->info.embed_flag && h->info.firstTime && h->sh.i_type == SLICE_TYPE_P) 
		{
			if (mb_xy == h->sh.i_last_mb - 1)//
			{
				//int beta3 = 2;//�����޸ĵ�mvΪ0ʱ��ʧ��������������������20220115   *****
				x264_emms();
				//h->info.b_frame_used = 1;//�����֡��Ƕ��,��Ϊ����mb_xy���ܵ�����395,����bug,
										 // ��������h->info.cover�У�lsb��h+v����,����h->info.length
				
				memset(h->info.mv_h, 0, sizeof(int) * 6336);//��mv������ʧ����صĳ�ʼ��
				memset(h->info.mv_v, 0, sizeof(int) * 6336);
				//memset(h->info.rho_com_all, 0, sizeof(float) * 6336);
				int ii_mb_x = 0, ii_mb_y = 0;//�������x��y����
				int ii_mb_4x4 = 0;//��������4*4Ϊ��λ�������Ͻ�4*4�����꣬
				for (int m_xy = 0; m_xy < h->sh.i_last_mb; m_xy++)
				{
					ii_mb_y = m_xy / h->sps->i_mb_width;
					ii_mb_x = m_xy % h->sps->i_mb_width;
					ii_mb_4x4 = 4 * (ii_mb_y * h->mb.i_b4_stride + ii_mb_x);//��������4*4Ϊ��λ�������Ͻ�4*4�����꣬
					switch (h->info.cache[m_xy].i_type)
					{
					case P_L0:
						switch (h->info.cache[m_xy].i_partition)
						{
						case D_16x16:
							for (int i = 0; i < 4; i++)
							{
								for (int j = 0; j < 4; j++)
								{
									h->info.mv_h[i*h->mb.i_b4_stride + j + ii_mb_4x4] = h->info.cache[m_xy].mv[0][0];//ˮƽ
									h->info.mv_v[i*h->mb.i_b4_stride + j + ii_mb_4x4] = h->info.cache[m_xy].mv[0][1];//��ֱ
								}
							}
							break;
						case D_16x8:
							for (int i = 0; i < 2; i++)
							{
								for (int j = 0; j < 4; j++)
								{
									h->info.mv_h[i*h->mb.i_b4_stride + j + ii_mb_4x4] = h->info.cache[m_xy].mv[0][0];
									h->info.mv_v[i*h->mb.i_b4_stride + j + ii_mb_4x4] = h->info.cache[m_xy].mv[0][1];
								}
							}
							ii_mb_4x4 += 2 * h->mb.i_b4_stride;
							for (int i = 0; i < 2; i++)
							{
								for (int j = 0; j < 4; j++)
								{
									h->info.mv_h[i*h->mb.i_b4_stride + j + ii_mb_4x4] = h->info.cache[m_xy].mv[8][0];
									h->info.mv_v[i*h->mb.i_b4_stride + j + ii_mb_4x4] = h->info.cache[m_xy].mv[8][1];
								}
							}
							break;

						case D_8x16:
							for (int i = 0; i < 4; i++)
							{
								for (int j = 0; j < 2; j++)
								{
									h->info.mv_h[i*h->mb.i_b4_stride + j + ii_mb_4x4] = h->info.cache[m_xy].mv[0][0];
									h->info.mv_v[i*h->mb.i_b4_stride + j + ii_mb_4x4] = h->info.cache[m_xy].mv[0][1];
								}
							}
							ii_mb_4x4 += 2;
							for (int i = 0; i < 4; i++)
							{
								for (int j = 0; j < 2; j++)
								{
									h->info.mv_h[i*h->mb.i_b4_stride + j + ii_mb_4x4] = h->info.cache[m_xy].mv[4][0];
									h->info.mv_v[i*h->mb.i_b4_stride + j + ii_mb_4x4] = h->info.cache[m_xy].mv[4][1];
								}
							}
							break;

						default:
							x264_log(h, X264_LOG_ERROR, "internal error P_L0 and partition=%d\n", h->mb.i_partition);
							break;
						}
						break;

					case P_8x8:
					{
						int temp = ii_mb_4x4;
						for (int ii = 0; ii < 4; ii++) {
							if (ii == 1)//��õ�ǰ8*8�����Ͻǵ�4*4λ��
							{
								ii_mb_4x4 = temp + 2;
							}else if (ii == 2)
							{
								ii_mb_4x4 = temp + 2 * h->mb.i_b4_stride;
							}else if (ii == 3)
							{
								ii_mb_4x4 = temp + 2 * h->mb.i_b4_stride +2;
							}
							switch (h->info.cache[m_xy].i_sub_partition[ii])
							{
							case D_L0_8x8:
								for (int i = 0; i < 2; i++)
								{
									for (int j = 0; j < 2; j++)
									{
										h->info.mv_h[i*h->mb.i_b4_stride + j + ii_mb_4x4] = h->info.cache[m_xy].mv[4 * ii][0];
										h->info.mv_v[i*h->mb.i_b4_stride + j + ii_mb_4x4] = h->info.cache[m_xy].mv[4 * ii][1];
									}
								}
								break;
							case D_L0_8x4:
								for (int j = 0; j < 2; j++)
								{
									h->info.mv_h[j + ii_mb_4x4] = h->info.cache[m_xy].mv[4 * ii][0];
									h->info.mv_v[j + ii_mb_4x4] = h->info.cache[m_xy].mv[4 * ii][1];
								}
								ii_mb_4x4 += h->mb.i_b4_stride;
								for (int j = 0; j < 2; j++)
								{
									h->info.mv_h[j + ii_mb_4x4] = h->info.cache[m_xy].mv[4 * ii + 2][0];
									h->info.mv_v[j + ii_mb_4x4] = h->info.cache[m_xy].mv[4 * ii + 2][1];
								}
								break;
							case D_L0_4x8:
								for (int j = 0; j < 2; j++)
								{
									h->info.mv_h[j * h->mb.i_b4_stride + ii_mb_4x4] = h->info.cache[m_xy].mv[4 * ii][0];
									h->info.mv_v[j * h->mb.i_b4_stride + ii_mb_4x4] = h->info.cache[m_xy].mv[4 * ii][1];
								}
								ii_mb_4x4 += 1;
								for (int j = 0; j < 2; j++)
								{
									h->info.mv_h[j * h->mb.i_b4_stride + ii_mb_4x4] = h->info.cache[m_xy].mv[4 * ii][0];
									h->info.mv_v[j * h->mb.i_b4_stride + ii_mb_4x4] = h->info.cache[m_xy].mv[4 * ii + 1][1];
								}
								break;
							case D_L0_4x4:
								h->info.mv_h[ii_mb_4x4] = h->info.cache[m_xy].mv[4 * ii][0];
								h->info.mv_v[ii_mb_4x4] = h->info.cache[m_xy].mv[4 * ii][1];
								h->info.mv_h[ii_mb_4x4 + 1] = h->info.cache[m_xy].mv[4 * ii + 1][0];
								h->info.mv_v[ii_mb_4x4 + 1] = h->info.cache[m_xy].mv[4 * ii + 1][1];
								h->info.mv_h[h->mb.i_b4_stride + ii_mb_4x4] = h->info.cache[m_xy].mv[4 * ii + 2][0];
								h->info.mv_v[h->mb.i_b4_stride + ii_mb_4x4] = h->info.cache[m_xy].mv[4 * ii + 2][1];
								h->info.mv_h[h->mb.i_b4_stride + ii_mb_4x4 + 1] = h->info.cache[m_xy].mv[4 * ii + 3][0];
								h->info.mv_v[h->mb.i_b4_stride + ii_mb_4x4 + 1] = h->info.cache[m_xy].mv[4 * ii + 3][1];
								break;
							default:
								x264_log(h, X264_LOG_ERROR, "internal error\n");
								break;
							}
						}
						break;
					}
					case P_SKIP://skipģʽ��ʱ��������Ԥ��mv
						for (int i = 0; i < 4; i++)
						{
							for (int j = 0; j < 4; j++)
							{
								h->info.mv_h[i*h->mb.i_b4_stride + j + ii_mb_4x4] = h->info.cache[m_xy].pskip_mv_[0];//ˮƽ
								h->info.mv_v[i*h->mb.i_b4_stride + j + ii_mb_4x4] = h->info.cache[m_xy].pskip_mv_[1];//��ֱ
							}
						}
						break;
					default:
						//
						break;
					}
				}
				//����lib������ʧ��,�õ�mv�йظ�����ʧ���ֵh->info.rho_com_all����float *���飬ÿ4*4���ؿ�һ��ֵ��
				h->info.rho_com_all = get_cost_lib_for_x264(h->sps->i_mb_width * 4, h->sps->i_mb_height * 4,
					h->info.mv_h, h->info.mv_v);
				//��ÿ��mv��Ӧ�����ɸ�4*4����ʧ��ƽ��ֵ������h->info.rho_com_all�����Ͻ�λ��
				for (int m_xy = 0; m_xy < h->sh.i_last_mb; m_xy++)
				{
					ii_mb_y = m_xy / h->sps->i_mb_width;
					ii_mb_x = m_xy % h->sps->i_mb_width;
					ii_mb_4x4 = 4 * (ii_mb_y * h->mb.i_b4_stride + ii_mb_x);//��������4*4Ϊ��λ�������Ͻ�4*4�����꣬
					if (h->info.cache[m_xy].i_type == P_8x8)
					{
						int temp = ii_mb_4x4;
						for (int i = 0; i < 4; i++)
						{
							if (i == 1)//��õ�ǰ8*8�����Ͻǵ�4*4λ��
							{
								ii_mb_4x4 = temp + 2;
							}
							else if (i == 2)
							{
								ii_mb_4x4 = temp + 2 * h->mb.i_b4_stride;
							}
							else if (i == 3)
							{
								ii_mb_4x4 = temp + 2 * h->mb.i_b4_stride + 2;
							}
							uint8_t partition = h->info.cache[m_xy].i_sub_partition[i];
							switch (partition)
							{
							case D_L0_8x8:
								h->info.rho_com_all[ii_mb_4x4]= h->info.rho_com_all[ii_mb_4x4]+ h->info.rho_com_all[ii_mb_4x4+1]
									+ h->info.rho_com_all[ii_mb_4x4 + h->mb.i_b4_stride]+h->info.rho_com_all[ii_mb_4x4 + h->mb.i_b4_stride+1];
								h->info.rho_com_all[ii_mb_4x4] /= 4;
								break;
							case D_L0_4x8:
								for (int j = 0; j < 2; j++)
								{
									h->info.rho_com_all[ii_mb_4x4 + j] = h->info.rho_com_all[ii_mb_4x4 + j] + h->info.rho_com_all[ii_mb_4x4 + j + h->mb.i_b4_stride];
									h->info.rho_com_all[ii_mb_4x4 + j] /= 2;
								}
								break;
							case D_L0_8x4:
								for (int j = 0; j < 2; j++)
								{
									h->info.rho_com_all[ii_mb_4x4 + j * h->mb.i_b4_stride] += h->info.rho_com_all[ii_mb_4x4 + j * h->mb.i_b4_stride + 1];
									h->info.rho_com_all[ii_mb_4x4 + j * h->mb.i_b4_stride] /= 2;
								}
								break;
							case D_L0_4x4:
								//����4��mv�����ÿ��Ǽ�ƽ��
								break;
							default:
								break;
							}
						}
					}
					else if (h->info.cache[m_xy].i_type == P_L0)
					{
						uint8_t partition = h->info.cache[m_xy].i_partition;
						float sum_temp;
						switch (partition)
						{
						case D_16x16:
						{
							sum_temp = 0;
							for (int i = 0; i < 4; i++)
							{
								for (int j=0;j<4;j++)
								{
									sum_temp += h->info.rho_com_all[ii_mb_4x4 + i * h->mb.i_b4_stride + j];
								}
							}
							h->info.rho_com_all[ii_mb_4x4] = sum_temp / 16;
							break;
						}
						case D_8x16:
							for (int j = 0; j < 2; j++)
							{
								sum_temp = 0;
								for (int j2 = 0;j2<4;j2++)
								{
									sum_temp = sum_temp+ h->info.rho_com_all[ii_mb_4x4 + j * 2 + j2 * h->mb.i_b4_stride]
										+ h->info.rho_com_all[ii_mb_4x4 + j * 2 + j2 * h->mb.i_b4_stride+1];
								}
								h->info.rho_com_all[ii_mb_4x4 + j * 2] = sum_temp / 8;
							}
							break;
						case D_16x8:
							for (int j = 0; j < 2; j++)
							{
								sum_temp = 0;
								for (int j3 = 0; j3 < 4; j3++)
								{
									sum_temp = sum_temp + h->info.rho_com_all[ii_mb_4x4 + j * 2 * h->mb.i_b4_stride + j3];
								}
								for (int j3 = 0; j3 < 4; j3++)
								{
									sum_temp = sum_temp + h->info.rho_com_all[ii_mb_4x4 + j * 2 * h->mb.i_b4_stride + h->mb.i_b4_stride+ j3 ];
								}
								h->info.rho_com_all[ii_mb_4x4 + j * 2 * h->mb.i_b4_stride] = sum_temp / 8;
							}
							break;
						default:
							break;
						}
					}
				}

				//����h->info.rho_com_all��ƽ��ֵ�ͷ���Ծ����Ƿ���Ҫ��һ��
				/*float sum = 0;
				for (int i = 0; i < 6336; i++)
				{
					sum += h->info.rho_com_all[i];//���
				}
				float average = sum / 6336;//��ƽ��ֵ
				float var = 0;
				for (int j = 0; j < 6336; j++)
				{
					var += pow(h->info.rho_com_all[j] - average, 2)/6336;//�󷽲�
				}*/

				h->info.length = 0;
				memset(h->info.rho_final, 0, sizeof(int8_t) * 6336);
				memset(h->info.rho_com, 0, sizeof(int8_t) * 6336);
				memset(h->info.cover, 0, sizeof(int8_t) * 6336);
				//�������ÿ������ڵ�ÿ��mv�ľֲ�������ص�ʧ��,��
				for (int m_xy = 0; m_xy < h->sh.i_last_mb; m_xy++)
				{
					if (h->info.cache[m_xy].used)//����P_L0��P_8x8�е�����mv��Ϊ����,
					{
						if (h->info.cache[m_xy].i_type == P_8x8)
						{
							for (int i = 0; i < 4; i++)
							{
								uint8_t partition = h->info.cache[m_xy].i_sub_partition[i];
								switch (partition)
								{
								case D_L0_8x8:
									//1��8*8�飬1���˶�ʸ����2��������
									h->info.cover[h->info.length] = (h->info.cache[m_xy].mv[i * 4][0] + h->info.cache[m_xy].mv[i * 4][1]) & 0x0001;//lsb��h+v��
									h->info.rho_final[h->info.length] = (float)h->info.cache[m_xy].inter_stego_cost[i * 4];// / 4;//��4*4��СΪ��λ����һ����ͬ���ʧ�棬����ʵ����֤������һ����ȫ�Ը��ߣ�����Ŀǰ����Ǿֲ�����ʧ��
									/*if (*(uint32_t *)&(h->info.cache[m_xy].mv[i * 4]) == 0)//���ԭʼmvΪ��0��0����������ʧ��Ŵ�beta3��������
									{
										h->info.rho_final[h->info.length] *= beta3;
									}*/
									h->info.length++;
									break;
								case D_L0_4x8:
									for (int j = 0; j < 2; j++)
									{
										h->info.cover[h->info.length] = (h->info.cache[m_xy].mv[i * 4 + j][0] + h->info.cache[m_xy].mv[i * 4 + j][1]) & 0x0001;
										h->info.rho_final[h->info.length] = (float)h->info.cache[m_xy].inter_stego_cost[i * 4 + j];// / 2;
										h->info.length++;
									}
									break;
								case D_L0_8x4:
									for (int j = 0; j < 2; j++)
									{
										h->info.cover[h->info.length] = (h->info.cache[m_xy].mv[i * 4 + 2 * j][0] + h->info.cache[m_xy].mv[i * 4 + 2 * j][1]) & 0x0001;
										h->info.rho_final[h->info.length] = (float)h->info.cache[m_xy].inter_stego_cost[i * 4 + 2 * j];// / 2;
										h->info.length++;
									}
									break;
								case D_L0_4x4:
									for (int j = 0; j < 4; j++)
									{
										h->info.cover[h->info.length] = (h->info.cache[m_xy].mv[i * 4 + j][0] + h->info.cache[m_xy].mv[i * 4 + j][1]) & 0x0001;
										h->info.rho_final[h->info.length] = (float)h->info.cache[m_xy].inter_stego_cost[i * 4 + j];
										h->info.length++;
									}
									break;
								default:
									break;
								}
							}
						}
						else if (h->info.cache[m_xy].i_type == P_L0)
						{
							uint8_t partition = h->info.cache[m_xy].i_partition;
							switch (partition)
							{
							case D_16x16:
								h->info.cover[h->info.length] = (h->info.cache[m_xy].mv[0][0] + h->info.cache[m_xy].mv[0][1]) & 0x0001;//lsb��h+v��
								h->info.rho_final[h->info.length] = (float)h->info.cache[m_xy].inter_stego_cost[0];// / 16;//��4*4��СΪ��λ����һ����ͬ���ʧ��
								h->info.length++;
								break;
							case D_8x16:
								for (int j = 0; j < 2; j++)
								{
									h->info.cover[h->info.length] = (h->info.cache[m_xy].mv[j * 4][0] + h->info.cache[m_xy].mv[j * 4][1]) & 0x0001;
									h->info.rho_final[h->info.length] = (float)h->info.cache[m_xy].inter_stego_cost[j * 4];// / ;
									h->info.length++;
								}
								break;
							case D_16x8:
								for (int j = 0; j < 2; j++)
								{
									h->info.cover[h->info.length] = (h->info.cache[m_xy].mv[j * 8][0] + h->info.cache[m_xy].mv[j * 8][1]) & 0x0001;
									h->info.rho_final[h->info.length] = (float)h->info.cache[m_xy].inter_stego_cost[j * 8];// / 8;
									h->info.length++;
								}
								break;
							default:
								break;
							}
						}
					}
				}

				//���ϸ�����ʧ�桢�Ծֲ�ʧ�����MVC�������ϳ�����ʧ�棨����+�ֲ�+�ֲ�mvc��
				h->info.length = 0;
				float alpha_loc = 1;//alpha_loc�ֲ�ʧ��Ȩ�أ�alpha_com������ʧ��Ȩ�أ����Ϊ1��*****
				float alpha_com = 0;
				float mvc_c1 = 2.0, mvc_c2 = 0.7;//����mvc�����������ͷ����ӣ���Ӧ��һ�����͵ڶ�����顣
				int d1 = 0, d2 = 0, d3 = 0, d4 = 0, d5 = 0, d6 = 0, d7 = 0, d8 = 0;//��һ�����ʱ��d1=abs(h1-h2),d2=abs(v1-v2);�ڶ������ʱ��d1��d8�ֱ�����������ֵ����ֵ
				int d_num_0_1 = 0;//ͳ��d����0����1������
				for (int m_xy = 0; m_xy < h->sh.i_last_mb; m_xy++)
				{
					if (h->info.cache[m_xy].used)//����P_L0��P_8x8�е�����mv��Ϊ����,
					{
						ii_mb_y = m_xy / h->sps->i_mb_width;
						ii_mb_x = m_xy % h->sps->i_mb_width;
						ii_mb_4x4 = 4 * (ii_mb_y * h->mb.i_b4_stride + ii_mb_x);//��������4*4Ϊ��λ�������Ͻ�4*4�����꣬
						if (h->info.cache[m_xy].i_type == P_8x8)
						{
							//��P_8x8ģʽ����4���ӿ��ǲ��Ƕ���D_L0_8x8������ǣ�����Ҫ����MVC����
							int is4_D_L0_8x8 = ( h->info.cache[m_xy].i_sub_partition[0] == D_L0_8x8 ) && (h->info.cache[m_xy].i_sub_partition[1] == D_L0_8x8) &&
												( h->info.cache[m_xy].i_sub_partition[2] == D_L0_8x8 ) && (h->info.cache[m_xy].i_sub_partition[3] == D_L0_8x8);
							if (is4_D_L0_8x8)//���ȫ��8*8�����������в����������Ļ��֣��������info.length�������û������ģ�����ֱ�ӵ���mvc
							{
								d1 = abs(h->info.cache[m_xy].mv[0][0]  - h->info.cache[m_xy].mv[4][0]);
								d2 = abs(h->info.cache[m_xy].mv[4][0]  - h->info.cache[m_xy].mv[12][0]);
								d3 = abs(h->info.cache[m_xy].mv[12][0] - h->info.cache[m_xy].mv[8][0]);
								d4 = abs(h->info.cache[m_xy].mv[8][0]  - h->info.cache[m_xy].mv[0][0]);
								d5 = abs(h->info.cache[m_xy].mv[0][1]  - h->info.cache[m_xy].mv[4][1]);
								d6 = abs(h->info.cache[m_xy].mv[4][1]  - h->info.cache[m_xy].mv[12][1]);
								d7 = abs(h->info.cache[m_xy].mv[12][1] - h->info.cache[m_xy].mv[8][1]);
								d8 = abs(h->info.cache[m_xy].mv[8][1]  - h->info.cache[m_xy].mv[0][1]);
								d_num_0_1 = (((d1 == 0) || (d1 == 1)) ? 1 : 0) + (((d2 == 0) || (d2 == 1)) ? 1 : 0) + (((d3 == 0) || (d3 == 1)) ? 1 : 0) +
											(((d4 == 0) || (d4 == 1)) ? 1 : 0) + (((d5 == 0) || (d5 == 1)) ? 1 : 0) + (((d6 == 0) || (d6 == 1)) ? 1 : 0) +
											(((d7 == 0) || (d7 == 1)) ? 1 : 0) + (((d8 == 0) || (d8 == 1)) ? 1 : 0);
								h->info.rho_final[h->info.length] = h->info.rho_final[h->info.length] * (mvc_c2 * d_num_0_1 + 1);//����ʧ��
								h->info.rho_final[h->info.length + 1] = h->info.rho_final[h->info.length + 1] * (mvc_c2 * d_num_0_1 + 1);
								h->info.rho_final[h->info.length + 2] = h->info.rho_final[h->info.length + 2] * (mvc_c2 * d_num_0_1 + 1);
								h->info.rho_final[h->info.length + 3] = h->info.rho_final[h->info.length + 3] * (mvc_c2 * d_num_0_1 + 1);
							}
							int temp = ii_mb_4x4;
							for (int i = 0; i < 4; i++)
							{
								if (i == 1)//��õ�ǰ8*8�����Ͻǵ�4*4λ��
								{
									ii_mb_4x4 = temp + 2;
								}
								else if (i == 2)
								{
									ii_mb_4x4 = temp + 2 * h->mb.i_b4_stride;
								}
								else if (i == 3)
								{
									ii_mb_4x4 = temp + 2 * h->mb.i_b4_stride + 2;
								}
								uint8_t partition = h->info.cache[m_xy].i_sub_partition[i];
								switch (partition)
								{
								case D_L0_8x8:
									//1��8*8�飬1���˶�ʸ����2��������
									h->info.rho_com[h->info.length] = h->info.rho_com_all[ii_mb_4x4];//������ʧ��
									h->info.rho_final[h->info.length] = alpha_loc * h->info.rho_final[h->info.length] + alpha_com * h->info.rho_com[h->info.length];
									h->info.length++;
									break;
								case D_L0_4x8:
									d1 = abs(h->info.cache[m_xy].mv[4 * i][0] - h->info.cache[m_xy].mv[4 * i+1][0]);//����mv��ˮƽ������ֵ����ֵ
									d2 = abs(h->info.cache[m_xy].mv[4 * i][1] - h->info.cache[m_xy].mv[4 * i+1][1]);//����mv�Ĵ�ֱ������ֵ����ֵ
									if (d1 + d2 < 2)// 0,1
									{
										h->info.rho_final[h->info.length] = h->info.rho_final[h->info.length] * mvc_c1;//������һ��mv�ľֲ�����ʧ��
										h->info.rho_final[h->info.length + 1] = h->info.rho_final[h->info.length + 1] * mvc_c1;//�����ڶ���mv�ľֲ�����ʧ��
									}
									for (int j = 0; j < 2; j++)
									{
										h->info.rho_com[h->info.length] = h->info.rho_com_all[ii_mb_4x4 + j];//������ʧ��
										h->info.rho_final[h->info.length] = alpha_loc * h->info.rho_final[h->info.length] + alpha_com * h->info.rho_com[h->info.length];
										h->info.length++;
									}
									break;
								case D_L0_8x4:
									d1 = abs(h->info.cache[m_xy].mv[4 * i][0] - h->info.cache[m_xy].mv[4 * i + 2][0]);//����mv��ˮƽ������ֵ����ֵ
									d2 = abs(h->info.cache[m_xy].mv[4 * i][1] - h->info.cache[m_xy].mv[4 * i + 2][1]);//����mv�Ĵ�ֱ������ֵ����ֵ
									if (d1 + d2 < 2)// 0,1
									{
										h->info.rho_final[h->info.length] = h->info.rho_final[h->info.length] * mvc_c1;//������һ��mv�ľֲ�����ʧ��
										h->info.rho_final[h->info.length + 1] = h->info.rho_final[h->info.length + 1] * mvc_c1;//�����ڶ���mv�ľֲ�����ʧ��
									}
									for (int j = 0; j < 2; j++)
									{
										h->info.rho_com[h->info.length] = h->info.rho_com_all[ii_mb_4x4 + j * h->mb.i_b4_stride];//������ʧ��
										h->info.rho_final[h->info.length] = alpha_loc * h->info.rho_final[h->info.length] + alpha_com * h->info.rho_com[h->info.length];
										h->info.length++;
									}
									break;
								case D_L0_4x4:
									d1 = abs(h->info.cache[m_xy].mv[i * 4    ][0] - h->info.cache[m_xy].mv[i * 4 + 1][0]);
									d2 = abs(h->info.cache[m_xy].mv[i * 4 + 1][0] - h->info.cache[m_xy].mv[i * 4 + 3][0]);
									d3 = abs(h->info.cache[m_xy].mv[i * 4 + 2][0] - h->info.cache[m_xy].mv[i * 4 + 3][0]);
									d4 = abs(h->info.cache[m_xy].mv[i * 4    ][0] - h->info.cache[m_xy].mv[i * 4 + 2][0]);
									d5 = abs(h->info.cache[m_xy].mv[i * 4    ][1] - h->info.cache[m_xy].mv[i * 4 + 1][1]);
									d6 = abs(h->info.cache[m_xy].mv[i * 4 + 1][1] - h->info.cache[m_xy].mv[i * 4 + 3][1]);
									d7 = abs(h->info.cache[m_xy].mv[i * 4 + 2][1] - h->info.cache[m_xy].mv[i * 4 + 3][1]);
									d8 = abs(h->info.cache[m_xy].mv[i * 4    ][1] - h->info.cache[m_xy].mv[i * 4 + 2][1]);
									/*d_nonz = IS_ZeroOrOne(d1) + IS_ZeroOrOne(d2) + IS_ZeroOrOne(d3) + IS_ZeroOrOne(d4) +
										IS_ZeroOrOne(d5) + IS_ZeroOrOne(d6) + IS_ZeroOrOne(d7) + IS_ZeroOrOne(d8);*/
									d_num_0_1 = (((d1 == 0) || (d1 == 1)) ? 1 : 0) + (((d2 == 0) || (d2 == 1)) ? 1 : 0) + (((d3 == 0) || (d3 == 1)) ? 1 : 0) +
											    (((d4 == 0) || (d4 == 1)) ? 1 : 0) + (((d5 == 0) || (d5 == 1)) ? 1 : 0) + (((d6 == 0) || (d6 == 1)) ? 1 : 0) +
											    (((d7 == 0) || (d7 == 1)) ? 1 : 0) + (((d8 == 0) || (d8 == 1)) ? 1 : 0);
									h->info.rho_final[h->info.length    ] = h->info.rho_final[h->info.length    ] * (mvc_c2 * d_num_0_1 + 1);//����ʧ��
									h->info.rho_final[h->info.length + 1] = h->info.rho_final[h->info.length + 1] * (mvc_c2 * d_num_0_1 + 1);
									h->info.rho_final[h->info.length + 2] = h->info.rho_final[h->info.length + 2] * (mvc_c2 * d_num_0_1 + 1);
									h->info.rho_final[h->info.length + 3] = h->info.rho_final[h->info.length + 3] * (mvc_c2 * d_num_0_1 + 1);
									for (int j = 0; j < 4; j++)
									{
										int temp2 = ((j % 2) == 1) ? (j == 1 ? 1 : h->mb.i_b4_stride + 1) : (j == 0 ? 0 : h->mb.i_b4_stride); temp2 = ii_mb_4x4 + temp2;//�������
										h->info.rho_com[h->info.length] = h->info.rho_com_all[temp2];//������ʧ��
										h->info.rho_final[h->info.length] = alpha_loc * h->info.rho_final[h->info.length] + alpha_com * h->info.rho_com[h->info.length];
										h->info.length++;
									}
									break;
								default:
									break;
								}
							}
						}
						else if (h->info.cache[m_xy].i_type == P_L0)
						{
							uint8_t partition = h->info.cache[m_xy].i_partition;
							switch (partition)
							{
							case D_16x16:
								//D_16x16���������MVC����
								h->info.rho_com[h->info.length] = h->info.rho_com_all[ii_mb_4x4];//������ʧ��
								h->info.rho_final[h->info.length] = alpha_loc * h->info.rho_final[h->info.length] + alpha_com * h->info.rho_com[h->info.length];
								h->info.length++;
								break;
							case D_8x16:
								//��һ����飬������MV��
								//����mv�ֱ�Ϊ��h->info.cache[m_xy].mv[0]        h->info.cache[m_xy].mv[4]
								d1 = abs(h->info.cache[m_xy].mv[0][0] - h->info.cache[m_xy].mv[4][0]);//����mv��ˮƽ������ֵ����ֵ
								d2 = abs(h->info.cache[m_xy].mv[0][1] - h->info.cache[m_xy].mv[4][1]);//����mv�Ĵ�ֱ������ֵ����ֵ
								if ( d1 + d2 < 2)// 0,1
								{
									h->info.rho_final[h->info.length] = h->info.rho_final[h->info.length] * mvc_c1;//������һ��mv�ľֲ�����ʧ��
									h->info.rho_final[h->info.length+1] = h->info.rho_final[h->info.length+1] * mvc_c1;//�����ڶ���mv�ľֲ�����ʧ��
								}
								for (int j = 0; j < 2; j++)
								{
									h->info.rho_com[h->info.length] = h->info.rho_com_all[ii_mb_4x4 + j * 2];//�õ�������ʧ��
									h->info.rho_final[h->info.length] = alpha_loc * h->info.rho_final[h->info.length] + alpha_com * h->info.rho_com[h->info.length];//��Ȩ����������ʧ��
									h->info.length++;
								}
								break;
							case D_16x8:
								d1 = abs(h->info.cache[m_xy].mv[0][0] - h->info.cache[m_xy].mv[8][0]);//����mv��ˮƽ������ֵ����ֵ
								d2 = abs(h->info.cache[m_xy].mv[0][1] - h->info.cache[m_xy].mv[8][1]);//����mv�Ĵ�ֱ������ֵ����ֵ
								if (d1 + d2 < 2)// 0,1
								{
									h->info.rho_final[h->info.length] = h->info.rho_final[h->info.length] * mvc_c1;//������һ��mv�ľֲ�����ʧ��
									h->info.rho_final[h->info.length + 1] = h->info.rho_final[h->info.length + 1] * mvc_c1;//�����ڶ���mv�ľֲ�����ʧ��
								}
								for (int j = 0; j < 2; j++)
								{
									h->info.rho_com[h->info.length] = h->info.rho_com_all[ii_mb_4x4 + j * 2 * h->mb.i_b4_stride];//������ʧ��
									h->info.rho_final[h->info.length] = alpha_loc * h->info.rho_final[h->info.length] + alpha_com * h->info.rho_com[h->info.length];
									h->info.length++;
								}
								break;
							default:
								break;
							}
						}
					}
				}



				free(h->info.rho_com_all);

				memset(h->info.filp, 0, sizeof(int8_t) * 6336);
				memset(h->info.stego, 0, sizeof(int8_t) * 6336);

				float rate = h->param.eparam.iEmRate; // Ƕ����
				int an = 0;
				if (rate > 1)//��bit per frame��ΪǶ���ʵ�λ��
				{
					an = rate;
				}
				else {//��bpmv��ΪǶ���ʵĵ�λ
					an= (int)(rate*h->info.length); // ����Ϣ����
				}
													 // ����Ƕ����Ϣ�������
				for (int i = 0; i < an; i++) {
					h->info.message[i] = rand() & 0x01;
				}

				// ��д���õ���д������h->info.stego
				stc_embed(h->info.cover, h->info.length, h->info.message, an, h->info.rho_final, h->info.stego, 10);
				//stc_embed(h->info.cover, h->info.length, h->info.message, an, h->info.rho_com, h->info.stego, 10);//���ԣ�ֻ�ø��Ӷ�ʧ��ʵ��



				h->info.num_filp = 0;
				// 
				for (int i = 0; i < h->info.length; i++) {
					if (h->info.cover[i] ^ h->info.stego[i]) { // �����Ҫ���޸�
						h->info.num_filp++; // ͳ����Ҫ�޸ĵĸ���
						h->info.filp[i] = 1;//���mv���Ϊ��Ҫ�޸�
					}
				}
				// ���յ�λ���޸�
				if (DEGUG_LIJUN)
				{
					printf("1���ڵ�һ�α���Ԥ�����У���֡����������%d,��Ƕ���ʣ�%f, ��Ƕ����Ϣ����%d, ��Ҫ�޸ĵ�������:%d\n",
						h->info.length, h->param.eparam.iEmRate, an, h->info.num_filp);
				}
			}
		}
		
		/*
		if ((h->mb.i_type == P_SKIP) && (!h->info.firstTime))//��ʱͳ��
		{
			h->info.num_p_skip_before_analysis++;
			
		}*/

		if (mb_xy==h->sh.i_last_mb-1 && DEGUG_LIJUN && (h->info.firstTime) && h->sh.i_type == SLICE_TYPE_P)
			printf("��һ��Ԥ����������mv��1�������Ŀ��%d,��2���������%d����λ������%d\n", 
				h->info.stat.num_optimal_1_neighbor, h->info.stat.num_optimal_2_neighbor, h->info.stat.num_error_pos);
		

        /* encode this macroblock -> be careful it can change the mb type to P_SKIP if needed */
        // ������ģ�顣��ģ��ͨ���Բв��DCT�任�������ȷ�ʽ�Ժ����б���
		// ����Intra16x16��飬����x264_mb_encode_i16x16()���б��룬����Intra4x4��
		// ����x264_mb_encode_i4x4()���б��롣����Inter���͵ĺ����ֱ���ں�����������롣
		x264_macroblock_encode( h ); //����-�в�DCT�任������   //�ڱ�������л����ܻ��ΪP_SKIP��ע��ע��   *****

		/*
		if ((h->mb.i_type == P_SKIP) && (!h->info.firstTime))//��ʱͳ��
		{
			h->info.num_p_skip_after_encode++;

		}
		if (mb_xy == 395 && DEGUG_LIJUN && (!h->info.firstTime))
			printf("ʵ���޸Ĺ�����,������ܵ�p_skip�ĸ�����%d\n", h->info.num_p_skip_after_encode);
		*/


		x264_bitstream_check_buffer(h);// �ڴ��飬�粻�����



		if( h->param.b_cabac ) // ���CABAC   h->cabac����
		{
			if( mb_xy > h->sh.i_first_mb && !(h->sh.b_mbaff && (i_mb_y&1)) )
				x264_cabac_encode_terminal( &h->cabac );//

			if( IS_SKIP( h->mb.i_type ) )
				x264_cabac_mb_skip( h, 1 );
			else
			{
				if( h->sh.i_type != SLICE_TYPE_I )
					x264_cabac_mb_skip( h, 0 );
				// CABAC�ر���ģ��
				x264_macroblock_write_cabac( h, &h->cabac );
			}
		}
		else
		{
			if( IS_SKIP( h->mb.i_type ) )
				i_skip++;
			else
			{
				if( h->sh.i_type != SLICE_TYPE_I )
				{
					bs_write_ue( &h->out.bs, i_skip );  /* skip run */
					i_skip = 0;
				}
				// CAVLC�ر���ģ��
				x264_macroblock_write_cavlc( h, &h->out.bs );
			}
		}

		
#if VISUALIZE
        if( h->param.b_visualize )
            x264_visualize_mb( h );
#endif

        /* save cache */ 
		//���浱ǰ���ĵ�ֵ����ǰ֡����Ϣ�����У����ں������֡�ı���ʹ��lijun
        //����Intra4x4���֡��Ԥ��ģʽ��DCT����ϵ�����˶�ʸ�����ο�֡��ŵȵȣ�������ǰ����ؽ����������ݱ��浽����ͼƬ�����������У���Ҫlijun
        x264_macroblock_cache_save( h );
	
		/* accumulate mb stats ������״̬����ǰframe��ͳ����*/
		h->stat.frame.i_mb_count[h->mb.i_type]++;//������ͼ���     ��Ҫ������ط����Բ鿴���α���Ļ��ַ�ʽ�Ƿ�ͬ��������һ�£�
		if( !IS_SKIP(h->mb.i_type) && !IS_INTRA(h->mb.i_type) && !IS_DIRECT(h->mb.i_type) )
		{
			if( h->mb.i_partition != D_8x8 )
				h->stat.frame.i_mb_partition[h->mb.i_partition] += 4;
			else
				for( i = 0; i < 4; i++ )
					h->stat.frame.i_mb_partition[h->mb.i_sub_partition[i]] ++;
			if( h->param.i_frame_reference > 1 )
				for( i_list = 0; i_list <= (h->sh.i_type == SLICE_TYPE_B); i_list++ )
					for( i = 0; i < 4; i++ )
					{
						i_ref = h->mb.cache.ref[i_list][ x264_scan8[4*i] ];
						if( i_ref >= 0 )
							h->stat.frame.i_mb_count_ref[i_list][i_ref] ++;
					}
		}
		if( h->mb.i_cbp_luma && !IS_INTRA(h->mb.i_type) )
			{
				h->stat.frame.i_mb_count_8x8dct[0] ++;
				h->stat.frame.i_mb_count_8x8dct[1] += h->mb.b_transform_8x8;
			}


        //ͳ��Ƕ����Ϣ��ȫ�֣�*****   lijun  ih
		if ((h->sh.i_type == SLICE_TYPE_P) && (!h->info.firstTime) && (mb_xy == h->sh.i_last_mb-1))
		{
			h->stat.info.i_mv_num += h->info.i_mv_no;//mv��������
			if (h->param.eparam.iEmRate < 1) 
			{
				h->stat.info.i_message_num += h->info.i_mv_no * h->param.eparam.iEmRate;//Ƕ����Ϣ
			}
			else {
				h->stat.info.i_message_num += h->param.eparam.iEmRate;
			}
			h->stat.info.i_modify_mv_num += h->info.num_mv_modify_real;
		}

		/*
		//��ʱͳ�����α�������и�֡��������  *****
		if (mb_xy == 395 && DEGUG_LIJUN && (h->info.firstTime))
		{
			printf("��һ��Ԥ���㣬������ɺ�I_16x16:��%d��,I_4x4:��%d��,P_L0:��%d��,P_8x8:��%d��,P_skip:��%d��\n", 
				h->stat.frame.i_mb_count[2], h->stat.frame.i_mb_count[0],
				h->stat.frame.i_mb_count[4], h->stat.frame.i_mb_count[5], h->stat.frame.i_mb_count[6]);
		}

		//��ʱͳ�����α�������и�֡��������  *****
		if (mb_xy == 395 && DEGUG_LIJUN && (!h->info.firstTime))
		{
			printf("�ڶ��α�����ɺ�I_16x16:��%d��,I_4x4:��%d��,P_L0:��%d��,P_8x8:��%d��,P_skip:��%d��\n",
				h->stat.frame.i_mb_count[2], h->stat.frame.i_mb_count[0],
				h->stat.frame.i_mb_count[4], h->stat.frame.i_mb_count[5], h->stat.frame.i_mb_count[6]);
		}*/

		// mb��������ʿ���
       x264_ratecontrol_mb( h, bs_pos(&h->out.bs) + x264_cabac_pos(&h->cabac) - mb_spos );

        if( h->sh.b_mbaff )
        {
            i_mb_x += i_mb_y & 1;
            i_mb_y ^= i_mb_x < h->sps->i_mb_width;
        }
        else
            i_mb_x++;
        if(i_mb_x == h->sps->i_mb_width)//������һ�к��
        { 
            i_mb_y++;
            i_mb_x = 0;
        }
    }


    if( h->param.b_cabac ) //�ر������β����
    {
        x264_cabac_encode_flush( h, &h->cabac );
        h->out.bs.p = h->cabac.p;
    }
    else
    {
        if( i_skip > 0 )
            bs_write_ue( &h->out.bs, i_skip );  /* last skip run */
        /* rbsp_slice_trailing_bits */
        bs_rbsp_trailing( &h->out.bs );
    }
	////////////////////////////////////// ����дһ��NALU
    x264_nal_end( h );

	// �˲�ģ�� ��ֵ������psnr��ssim��   ����Ӧ��ֻ�Ƕ����һ�к���ˣ�ǰ������ڱ���ǰ�Ѿ�������
    x264_fdec_filter_row( h, h->sps->i_mb_height );

    /* Compute misc bits */
    h->stat.frame.i_misc_bits = bs_pos( &h->out.bs )
                              + NALU_OVERHEAD * 8
                              - h->stat.frame.i_tex_bits
                              - h->stat.frame.i_mv_bits;
}

static void x264_thread_sync_context( x264_t *dst, x264_t *src )
{
    x264_frame_t **f;
    if( dst == src )
        return;

    // reference counting
    for( f = src->frames.reference; *f; f++ )
        (*f)->i_reference_count++;
    for( f = dst->frames.reference; *f; f++ )
        x264_frame_push_unused( src, *f );
    src->fdec->i_reference_count++;
    x264_frame_push_unused( src, dst->fdec );

    // copy everything except the per-thread pointers and the constants.
    memcpy( &dst->i_frame, &src->i_frame, offsetof(x264_t, mb.type) - offsetof(x264_t, i_frame) );
    dst->stat = src->stat;
}

static void x264_thread_sync_stat( x264_t *dst, x264_t *src )
{
    if( dst == src )
        return;
    memcpy( &dst->stat.i_slice_count, &src->stat.i_slice_count, sizeof(dst->stat) - sizeof(dst->stat.frame) );
}

static int x264_slices_write( x264_t *h )
{
    int i_frame_size;

#ifdef HAVE_MMX
    /* Misalign mask has to be set separately for each thread. */
    if( h->param.cpu&X264_CPU_SSE_MISALIGN )
        x264_cpu_mask_misalign_sse();
#endif

#if VISUALIZE
    if( h->param.b_visualize )
        x264_visualize_init( h );
#endif

	/*x264_t old;
	memcpy(&old, h, sizeof(x264_t));*/

    x264_stack_align( x264_slice_write, h );

	// x264_stack_align( x264_slice_write, &old);
    i_frame_size = h->out.nal[h->out.i_nal-1].i_payload;

#if VISUALIZE
    if( h->param.b_visualize )
    {
        x264_visualize_show( h );
        x264_visualize_close( h );
    }
#endif

    h->out.i_frame_size = i_frame_size;
    return 0;
}

/****************************************************************************
 * x264_encoder_encode:
 *  XXX: i_poc   : is the poc of the current given picture
 *       i_frame : is the number of the frame being coded
 *  ex:  type frame poc
 *       I      0   2*0
 *       P      1   2*3
 *       B      2   2*1
 *       B      3   2*2
 *       P      4   2*6
 *       B      5   2*4
 *       B      6   2*5
 ****************************************************************************/
int     x264_encoder_encode( x264_t *h,
                             x264_nal_t **pp_nal, int *pi_nal,
                             x264_picture_t *pic_in,
                             x264_picture_t *pic_out )
{
    x264_t *thread_current, *thread_prev, *thread_oldest;
    int     i_nal_type;
    int     i_nal_ref_idc;

    int   i_global_qp;

	int re_encode = 1;

    if( h->param.i_threads > 1)
    {
        int i = ++h->i_thread_phase;
        int t = h->param.i_threads;
        thread_current = h->thread[ i%t ];
        thread_prev    = h->thread[ (i-1)%t ];
        thread_oldest  = h->thread[ (i+1)%t ];
        x264_thread_sync_context( thread_current, thread_prev );
        x264_thread_sync_ratecontrol( thread_current, thread_prev, thread_oldest );
        h = thread_current;
				//fprintf(stderr, "current: %p  prev: %p  oldest: %p \n", thread_current, thread_prev, thread_oldest);
    }
    else
    {
        thread_current =
        thread_prev    =
        thread_oldest  = h;
    }

    // ok to call this before encoding any frames, since the initial values of fdec have b_kept_as_ref=0
    // �Ὣ�Ѿ��������֡���뵽reference��ͬʱpopһ������֡��Ϊ�ؽ�֡
	x264_reference_update( h );
    h->fdec->i_lines_completed = -1;

    /* no data out */
    *pi_nal = 0;
    *pp_nal = NULL;

    /* ------------------- Setup new frame from picture -------------------- */
    if( pic_in != NULL ) // �������ͼ��Ϊ��
    {
        /* 1: Copy the picture to a frame and move it to a buffer 
			��ȡ1��x264_frame_t���ͽṹ��fenc�����frames.unused[]���в�Ϊ�գ��͵���
			x264_frame_pop()��unused[]����ȡ1���ֳɵģ�����͵���x264_frame_new()����һ���µ�
		*/
        x264_frame_t *fenc = x264_frame_pop_unused( h );
		// �������ͼ�����ݿ�����fenc��plane�У��������ʽ
        if( x264_frame_copy_picture( h, fenc, pic_in ) < 0 )
            return -1;
		//��չ��16������
        if( h->param.i_width != 16 * h->sps->i_mb_width || h->param.i_height != 16 * h->sps->i_mb_height )
            x264_frame_expand_border_mod16( h, fenc ); 

        fenc->i_frame = h->frames.i_input++;
		// ��fenc����next���У��ȴ�ȷ��֡����
        x264_frame_push( h->frames.next, fenc );
		// ���ͷֱ���
        if( h->frames.b_have_lowres )
            x264_frame_init_lowres( h, fenc );

        if( h->param.rc.i_aq_mode )
            x264_adaptive_quant_frame( h, fenc );

        if( h->frames.i_input <= h->frames.i_delay + 1 - h->param.i_threads )
        {
            /* Nothing yet to encode */
            /* waiting for filling bframe buffer */
            pic_out->i_type = X264_TYPE_AUTO;
            return 0;
        }
    }

	// ���currentû��֡�Ż���������������������֡����Ϊ��
    if( h->frames.current[0] == NULL )
    {
        int bframes = 0;
        /* 2: Select frame types */
        if( h->frames.next[0] == NULL )
        {
            x264_encoder_frame_end( thread_oldest, thread_current, pp_nal, pi_nal, pic_out );
            return 0;
        }
		// ȷ��֡(next�е�����֡�����Ͷ��ᱻȷ��)������Ϣ
        x264_stack_align( x264_slicetype_decide, h );
		// h->param.i_bframe = 0�Ļ�����û��B֡
        /* 3: move some B-frames and 1 non-B to encode queue */
        while( IS_X264_TYPE_B( h->frames.next[bframes]->i_type ) )
            bframes++;
		// ���ҽ�frames.next[bframes]����frames.current[0]��
        x264_frame_push( h->frames.current, x264_frame_shift( &h->frames.next[bframes] ) );
        /* FIXME: when max B-frames > 3, BREF may no longer be centered after GOP closing */
        if( h->param.b_bframe_pyramid && bframes > 1 )
        {
            x264_frame_t *mid = x264_frame_shift( &h->frames.next[bframes/2] );
            mid->i_type = X264_TYPE_BREF;
            x264_frame_push( h->frames.current, mid );
            bframes--;
        }
        while( bframes-- )// ��B֡Ҳ�ŵ�frames.current[1~]
            x264_frame_push( h->frames.current, x264_frame_shift( h->frames.next ) );
    }

    /* ------------------- Get frame to be encoded ------------------------- */
    /* 4: get picture to encode */
	// ��frames.current[]����ͷ��ȡ��һ֡���ڱ���
    h->fenc = x264_frame_shift( h->frames.current );
    if( h->fenc == NULL )
    {
        /* Nothing yet to encode (ex: waiting for I/P with B frames) */
        /* waiting for filling bframe buffer */
        pic_out->i_type = X264_TYPE_AUTO;
        return 0;
    }
	
	h->info.firstTime = 1; // add by hqtang
	if (h->param.eparam.iEmRate == 0) {// �����Ƕ����Ϊ0��������Ƕ��
		h->info.embed_flag = 0;
	}
	else {
		h->info.embed_flag = 1;
	}

do_encode:

    if( h->fenc->i_type == X264_TYPE_IDR )
    {
        h->frames.i_last_idr = h->fenc->i_frame;
    }

    /* ------------------- Setup frame context ----------------------------- */
    /* 5: Init data dependent of frame type */
    if( h->fenc->i_type == X264_TYPE_IDR )
    {
        /* ���ΪIDR֡�����øú�����ղο�֡�б���ԭ����reference�ŵ�unused */
        x264_reference_reset( h );
        i_nal_type    = NAL_SLICE_IDR;
        i_nal_ref_idc = NAL_PRIORITY_HIGHEST;
        h->sh.i_type = SLICE_TYPE_I;
    }
    else if( h->fenc->i_type == X264_TYPE_I )
    {
        i_nal_type    = NAL_SLICE;
        i_nal_ref_idc = NAL_PRIORITY_HIGH; /* Not completely true but for now it is (as all I/P are kept as ref)*/
        h->sh.i_type = SLICE_TYPE_I;
    }
    else if( h->fenc->i_type == X264_TYPE_P )
    {
        i_nal_type    = NAL_SLICE;
        i_nal_ref_idc = NAL_PRIORITY_HIGH; /* Not completely true but for now it is (as all I/P are kept as ref)*/
        h->sh.i_type = SLICE_TYPE_P;
    }
    else if( h->fenc->i_type == X264_TYPE_BREF )
    {
        i_nal_type    = NAL_SLICE;
        i_nal_ref_idc = NAL_PRIORITY_HIGH; /* maybe add MMCO to forget it? -> low */
        h->sh.i_type = SLICE_TYPE_B;
    }
    else    /* B frame */
    {
        i_nal_type    = NAL_SLICE;
        i_nal_ref_idc = NAL_PRIORITY_DISPOSABLE;
        h->sh.i_type = SLICE_TYPE_B;
    }

once_again:
	//�ؽ�֡�����֡�ĸ�ֵ
    h->fdec->i_poc = h->fenc->i_poc = 2 * (h->fenc->i_frame - h->frames.i_last_idr);
    h->fdec->i_type = h->fenc->i_type;
    h->fdec->i_frame = h->fenc->i_frame;
    h->fenc->b_kept_as_ref = h->fdec->b_kept_as_ref = i_nal_ref_idc != NAL_PRIORITY_DISPOSABLE && h->param.i_keyint_max > 1;

    /* ------------------- Init                ----------------------------- */
    /* �����ο�֡�б�list0��list1 */
    x264_reference_build_list( h, h->fdec->i_poc );

    /* �������ʿ��� */
    x264_ratecontrol_start( h, h->fenc->i_qpplus1 );
    i_global_qp = x264_ratecontrol_qp( h );

    pic_out->i_qpplus1 = h->fdec->i_qpplus1 = i_global_qp + 1;

    if( h->sh.i_type == SLICE_TYPE_B )
        x264_macroblock_bipred_init( h );


    /* ------------------------ Create slice header  ----------------------- */
	// ���� Slice Header
    x264_slice_init( h, i_nal_type, i_global_qp );

    if( i_nal_ref_idc != NAL_PRIORITY_DISPOSABLE )
        h->i_frame_num++;

    /* ---------------------- Write the bitstream -------------------------- */
    /* Init bitstream context */
    h->out.i_nal = 0;
    bs_init( &h->out.bs, h->out.p_bitstream, h->out.i_bitstream );

    if(h->param.b_aud){ //���b_aud��Ϊ0�����AUD����NALU
        int pic_type;

        if(h->sh.i_type == SLICE_TYPE_I)
            pic_type = 0;
        else if(h->sh.i_type == SLICE_TYPE_P)
            pic_type = 1;
        else if(h->sh.i_type == SLICE_TYPE_B)
            pic_type = 2;
        else
            pic_type = 7;

        x264_nal_start(h, NAL_AUD, NAL_PRIORITY_DISPOSABLE);
        bs_write(&h->out.bs, 3, pic_type);
        bs_rbsp_trailing(&h->out.bs);
        x264_nal_end(h);
    }

    h->i_nal_type = i_nal_type;
    h->i_nal_ref_idc = i_nal_ref_idc;

    /* Write SPS and PPS */
    if( i_nal_type == NAL_SLICE_IDR && h->param.b_repeat_headers )
    {
        if( h->fenc->i_frame == 0 )
        {
            /* identify ourself */
            x264_nal_start( h, NAL_SEI, NAL_PRIORITY_DISPOSABLE );
            x264_sei_version_write( h, &h->out.bs );
            x264_nal_end( h );
        }

        /* generate sequence parameters */
        x264_nal_start( h, NAL_SPS, NAL_PRIORITY_HIGHEST );
        x264_sps_write( &h->out.bs, h->sps );
        x264_nal_end( h );

        /* generate picture parameters */
        x264_nal_start( h, NAL_PPS, NAL_PRIORITY_HIGHEST );
        x264_pps_write( &h->out.bs, h->pps );
        x264_nal_end( h );
    }

    /* Write frame */
	// ��������(��ؼ��Ĳ���).���е�����x264_slice_write()����˱���Ĺ���
    if( h->param.i_threads > 1 )
    {
        x264_pthread_create( &h->thread_handle, NULL, (void*)x264_slices_write, h );
        h->b_thread_active = 1;
    }
    else
	{
		x264_slices_write(h);

	}

	/* restore CPU state (before using float again) */
	x264_emms();

	// lijun
	if (h->info.embed_flag &&!h->info.firstTime && h->fenc->i_type == X264_TYPE_P && DEGUG_LIJUN) {
		printf("3�����ս��    Ƕ����:%3f ,���峤�ȣ�%3d,��֡Ƕ����Ϣ��:%d���޸ĵ�mv��:%d��Ƕ��Ч��:%3f\n",
			h->param.eparam.iEmRate, h->info.length,(int)(h->param.eparam.iEmRate*h->info.length),h->info.num_filp,
			(float)(h->param.eparam.iEmRate*h->info.length)/ h->info.num_filp);
		printf("-------------------------------------------------------------------------------------------------\n");
	}
	/*************************�Լ���ӵĲ��֣�add by hqtang��****************************/
	if ( h->info.embed_flag && h->info.firstTime &&  h->fenc->i_type == X264_TYPE_P) // ����ͨ����������ٴα�����һ֡
	{
		if (DEGUG_LIJUN)
		{
			printf("���±����%d֡:\n",h->i_frame);
		}
		h->i_frame_num--;
		h->info.firstTime = 0;
		//h->info.i_mv_no = 0;//��ǵ�ǰ֡��mv�ı��,lijun
		goto do_encode;
	}
	/*******************************************************************/

	// ������������������ܻὫP֡����ΪI֡���½��б��������
    if(0 && h->sh.i_type == SLICE_TYPE_P && !h->param.rc.b_stat_read
        && h->param.i_scenecut_threshold >= 0 && !h->param.b_pre_scenecut )
    {
        const int *mbs = h->stat.frame.i_mb_count;
        int i_mb_i = mbs[I_16x16] + mbs[I_8x8] + mbs[I_4x4];
        int i_mb_p = mbs[P_L0] + mbs[P_8x8];
        int i_mb_s = mbs[P_SKIP];
        int i_mb   = h->sps->i_mb_width * h->sps->i_mb_height;
        int64_t i_inter_cost = h->stat.frame.i_inter_cost;
        int64_t i_intra_cost = h->stat.frame.i_intra_cost;

        float f_bias;
        int i_gop_size = h->fenc->i_frame - h->frames.i_last_idr;
        float f_thresh_max = h->param.i_scenecut_threshold / 100.0;
        /* magic numbers pulled out of thin air */
        float f_thresh_min = f_thresh_max * h->param.i_keyint_min / ( h->param.i_keyint_max * 4 );
        if( h->param.i_keyint_min == h->param.i_keyint_max )
             f_thresh_min= f_thresh_max;

        /* macroblock_analyse() doesn't further analyse skipped mbs,
         * so we have to guess their cost */
        if( h->stat.frame.i_mbs_analysed > 0 )
            i_intra_cost = i_intra_cost * i_mb / h->stat.frame.i_mbs_analysed;

        if( i_gop_size < h->param.i_keyint_min / 4 )
            f_bias = f_thresh_min / 4;
        else if( i_gop_size <= h->param.i_keyint_min )
            f_bias = f_thresh_min * i_gop_size / h->param.i_keyint_min;
        else
        {
            f_bias = f_thresh_min
                     + ( f_thresh_max - f_thresh_min )
                       * ( i_gop_size - h->param.i_keyint_min )
                       / ( h->param.i_keyint_max - h->param.i_keyint_min );
        }
        
		f_bias = X264_MIN( f_bias, 1.0 );

        /* �������ΪP֡�Ĵ��۱ȱ���ΪI֡��Ҫ����ô�ͻ��P֡����ΪI֡���½��б��� */
        if( h->stat.frame.i_mbs_analysed > 0 && i_inter_cost >= (1.0 - f_bias) * i_intra_cost )
        {
            int b;

            x264_log( h, X264_LOG_DEBUG, "scene cut at %d Icost:%.0f Pcost:%.0f ratio:%.4f bias:%.4f gop:%d (imb:%d pmb:%d smb:%d)\n",
                      h->fenc->i_frame,
                      (double)i_intra_cost, (double)i_inter_cost,
                      1. - (double)i_inter_cost / i_intra_cost,
                      f_bias, i_gop_size,
                      i_mb_i, i_mb_p, i_mb_s );

            /* Restore frame num */
            h->i_frame_num--;

            for( b = 0; h->frames.current[b] && IS_X264_TYPE_B( h->frames.current[b]->i_type ); b++ );
            if( b > 0 )
            {
                /* ���ʹ��B��ܣ���ǿ�ƹر�GOP�� ��ʹ��֡����I������IDR���ڳ����л�֮ǰǿ��ʹ��P֡Ҳ����������ѹ����
				 * �����в����ȷ�еĳ������ĸ�֡����������޷�����I֡�� ���ǽ�֮ǰ��B֡����ΪP��Ȼ���������б���˳��
				 */

                if( h->param.i_bframe_adaptive || b > 1 )
                    h->fenc->i_type = X264_TYPE_AUTO;
                x264_frame_sort_pts( h->frames.current );
                x264_frame_unshift( h->frames.next, h->fenc );
                h->fenc = h->frames.current[b-1];
                h->frames.current[b-1] = NULL;
                h->fenc->i_type = X264_TYPE_P;
                x264_frame_sort_dts( h->frames.current );
            }
            else if( i_gop_size >= h->param.i_keyint_min )/*Do IDR if needed */
            {
                /* Reset */
                h->i_frame_num = 0;

                /* Reinit field of fenc */
                h->fenc->i_type = X264_TYPE_IDR;
                h->fenc->i_poc = 0;

                /* Put enqueued frames back in the pool */
                while( h->frames.current[0] )
                    x264_frame_push( h->frames.next, x264_frame_shift( h->frames.current ) );
                x264_frame_sort_pts( h->frames.next );
            }
            else
            {
                h->fenc->i_type = X264_TYPE_I; // ����ǰ�����˵�֡��ΪI֡�������±���
            }
            goto do_encode;
        }
    }
	/* �����������һЩ�������������ͷ�һЩ�м�����Լ���ӡ���һЩͳ����Ϣ��
	** ���е�����x264_frame_push_unused()��fenc���·Ż�frames.unused[]���У�
	** ���ҵ���x264_ratecontrol_end()�ر����ʿ��ơ�
	*/
    x264_encoder_frame_end( thread_oldest, thread_current, pp_nal, pi_nal, pic_out );
    return 0;
}

static void x264_encoder_frame_end( x264_t *h, x264_t *thread_current,
                                    x264_nal_t **pp_nal, int *pi_nal,
                                    x264_picture_t *pic_out )
{
    int i, i_list;
    char psz_message[80];

    if( h->b_thread_active )
    {
        x264_pthread_join( h->thread_handle, NULL );
        h->b_thread_active = 0;
    }
    if( !h->out.i_nal )
    {
        pic_out->i_type = X264_TYPE_AUTO;
        return;
    }
	//�����ù��ı���֡fenc,Ϊʲô�أ�
    x264_frame_push_unused( thread_current, h->fenc );

    /* End bitstream, set output  */
    *pi_nal = h->out.i_nal;
    *pp_nal = h->out.nal;
    h->out.i_nal = 0;

    /* Set output picture properties */
	//pic_outΪx264_picture_t���ͽṹ�塣��libx264����Ľṹ��
	//fenc,fdec��x264_frame_t���ͽṹ�塣��libx264���ڲ��ṹ��
    if( h->sh.i_type == SLICE_TYPE_I )
        pic_out->i_type = h->i_nal_type == NAL_SLICE_IDR ? X264_TYPE_IDR : X264_TYPE_I;
    else if( h->sh.i_type == SLICE_TYPE_P )
        pic_out->i_type = X264_TYPE_P;
    else
        pic_out->i_type = X264_TYPE_B;
    pic_out->i_pts = h->fenc->i_pts;

    pic_out->img.i_plane = h->fdec->i_plane;
	// ͼ������
    for(i = 0; i < 3; i++)//�����ع���ĵ�ǰ֡��ʵ��Ҳ�����ս���֡���ݣ���
    {
        pic_out->img.i_stride[i] = h->fdec->i_stride[i];
        pic_out->img.plane[i] = h->fdec->plane[i];
    }

    /* ---------------------- Update encoder state ------------------------- */

    /* update rc */
    x264_emms();
    x264_ratecontrol_end( h, h->out.i_frame_size * 8 );

    /* restore CPU state (before using float again) */
    x264_emms();

    x264_noise_reduction_update( thread_current );

    /* ---------------------- Compute/Print statistics --------------------- */
    x264_thread_sync_stat( h, h->thread[0] );

    /* Slice stat  stat�д洢��ͳ����Ϣ */
    h->stat.i_slice_count[h->sh.i_type]++;// ֡����1
    h->stat.i_slice_size[h->sh.i_type] += h->out.i_frame_size + NALU_OVERHEAD;//֡��С
    h->stat.f_slice_qp[h->sh.i_type] += h->fdec->f_qp_avg_aq;
	// ��ͬ����MBͳ��
    for( i = 0; i < X264_MBTYPE_MAX; i++ )
        h->stat.i_mb_count[h->sh.i_type][i] += h->stat.frame.i_mb_count[i];
    for( i = 0; i < X264_PARTTYPE_MAX; i++ )
        h->stat.i_mb_partition[h->sh.i_type][i] += h->stat.frame.i_mb_partition[i];
    for( i = 0; i < 2; i++ )
        h->stat.i_mb_count_8x8dct[i] += h->stat.frame.i_mb_count_8x8dct[i];
    if( h->sh.i_type != SLICE_TYPE_I )
        for( i_list = 0; i_list < 2; i_list++ )
            for( i = 0; i < 32; i++ )
                h->stat.i_mb_count_ref[h->sh.i_type][i_list][i] += h->stat.frame.i_mb_count_ref[i_list][i];
    if( h->sh.i_type == SLICE_TYPE_P )
        h->stat.i_consecutive_bframes[h->fdec->i_frame - h->fref0[0]->i_frame - 1]++;
    if( h->sh.i_type == SLICE_TYPE_B )
    {
        h->stat.i_direct_frames[ h->sh.b_direct_spatial_mv_pred ] ++;
        if( h->mb.b_direct_auto_write )
        {
            //FIXME somewhat arbitrary time constants
            if( h->stat.i_direct_score[0] + h->stat.i_direct_score[1] > h->mb.i_mb_count )
            {
                for( i = 0; i < 2; i++ )
                    h->stat.i_direct_score[i] = h->stat.i_direct_score[i] * 9/10;
            }
            for( i = 0; i < 2; i++ )
                h->stat.i_direct_score[i] += h->stat.frame.i_direct_score[i];
        }
    }

    psz_message[0] = '\0';
    if( h->param.analyse.b_psnr )
    {
        int64_t ssd[3] = {//SSD��Sum of Squared Difference������ֵ��ƽ����
            h->stat.frame.i_ssd[0],
            h->stat.frame.i_ssd[1],
            h->stat.frame.i_ssd[2],
        };
		//SSD���Ѿ��ڡ��˲������ڼ������
		//SSD�򵥻����PSNR������x264_psnr()
        h->stat.i_ssd_global[h->sh.i_type] += ssd[0] + ssd[1] + ssd[2];
        h->stat.f_psnr_average[h->sh.i_type] += x264_psnr( ssd[0] + ssd[1] + ssd[2], 3 * h->param.i_width * h->param.i_height / 2 );
        h->stat.f_psnr_mean_y[h->sh.i_type] += x264_psnr( ssd[0], h->param.i_width * h->param.i_height );
        h->stat.f_psnr_mean_u[h->sh.i_type] += x264_psnr( ssd[1], h->param.i_width * h->param.i_height / 4 );
        h->stat.f_psnr_mean_v[h->sh.i_type] += x264_psnr( ssd[2], h->param.i_width * h->param.i_height / 4 );

        snprintf( psz_message, 80, " PSNR Y:%5.2f U:%5.2f V:%5.2f",
                  x264_psnr( ssd[0], h->param.i_width * h->param.i_height ),
                  x264_psnr( ssd[1], h->param.i_width * h->param.i_height / 4),
                  x264_psnr( ssd[2], h->param.i_width * h->param.i_height / 4) );
    }

    if( h->param.analyse.b_ssim )
    {
    	//SSIM���Ѿ��ڡ��˲������ڼ������
        double ssim_y = h->stat.frame.f_ssim
                      / (((h->param.i_width-6)>>2) * ((h->param.i_height-6)>>2));
        h->stat.f_ssim_mean_y[h->sh.i_type] += ssim_y;
        snprintf( psz_message + strlen(psz_message), 80 - strlen(psz_message),
                  " SSIM Y:%.5f", ssim_y );
    }
    psz_message[79] = '\0';

    x264_log( h, X264_LOG_DEBUG,
                  "frame=%4d QP=%.2f NAL=%d Slice:%c Poc:%-3d I:%-4d P:%-4d SKIP:%-4d size=%d bytes%s\n",
              h->i_frame,
              h->fdec->f_qp_avg_aq,
              h->i_nal_ref_idc,
              h->sh.i_type == SLICE_TYPE_I ? 'I' : (h->sh.i_type == SLICE_TYPE_P ? 'P' : 'B' ),
              h->fdec->i_poc,
              h->stat.frame.i_mb_count_i,
              h->stat.frame.i_mb_count_p,
              h->stat.frame.i_mb_count_skip,
              h->out.i_frame_size,
              psz_message );

    // keep stats all in one place
    x264_thread_sync_stat( h->thread[0], h );
    // for the use of the next frame
    x264_thread_sync_stat( thread_current, h );

#ifdef DEBUG_MB_TYPE
{
    static const char mb_chars[] = { 'i', 'i', 'I', 'C', 'P', '8', 'S',
        'D', '<', 'X', 'B', 'X', '>', 'B', 'B', 'B', 'B', '8', 'S' };
    int mb_xy;
    for( mb_xy = 0; mb_xy < h->sps->i_mb_width * h->sps->i_mb_height; mb_xy++ )
    {
        if( h->mb.type[mb_xy] < X264_MBTYPE_MAX && h->mb.type[mb_xy] >= 0 )
            fprintf( stderr, "%c ", mb_chars[ h->mb.type[mb_xy] ] );
        else
            fprintf( stderr, "? " );

        if( (mb_xy+1) % h->sps->i_mb_width == 0 )
            fprintf( stderr, "\n" );
    }
}
#endif

    if( h->param.psz_dump_yuv )
        x264_frame_dump( h );
}

static void x264_print_intra( int64_t *i_mb_count, double i_count, int b_print_pcm, char *intra )
{
    intra += sprintf( intra, "I16..4%s: %4.1f%% %4.1f%% %4.1f%%",
        b_print_pcm ? "..PCM" : "",
        i_mb_count[I_16x16]/ i_count,
        i_mb_count[I_8x8]  / i_count,
        i_mb_count[I_4x4]  / i_count );
    if( b_print_pcm )
        sprintf( intra, " %4.1f%%", i_mb_count[I_PCM]  / i_count );
}

/****************************************************************************
 * x264_encoder_close:
 ****************************************************************************/
void    x264_encoder_close  ( x264_t *h )
{
    int64_t i_yuv_size = 3 * h->param.i_width * h->param.i_height / 2;
    int64_t i_mb_count_size[2][7] = {{0}};
    char buf[200];
    int i, j, i_list, i_type;
    int b_print_pcm = h->stat.i_mb_count[SLICE_TYPE_I][I_PCM]
                   || h->stat.i_mb_count[SLICE_TYPE_P][I_PCM]
                   || h->stat.i_mb_count[SLICE_TYPE_B][I_PCM];

    for( i=0; i<h->param.i_threads; i++ )
    {
        // don't strictly have to wait for the other threads, but it's simpler than canceling them
        if( h->thread[i]->b_thread_active )
        {
            x264_pthread_join( h->thread[i]->thread_handle, NULL );
            assert( h->thread[i]->fenc->i_reference_count == 1 );
            x264_frame_delete( h->thread[i]->fenc );
        }
    }

    /* Slices used and PSNR */
    for( i=0; i<5; i++ )
    {
        static const int slice_order[] = { SLICE_TYPE_I, SLICE_TYPE_SI, SLICE_TYPE_P, SLICE_TYPE_SP, SLICE_TYPE_B };
        static const char *slice_name[] = { "P", "B", "I", "SP", "SI" };
        int i_slice = slice_order[i];

        if( h->stat.i_slice_count[i_slice] > 0 )
        {
            const int i_count = h->stat.i_slice_count[i_slice];
            if( h->param.analyse.b_psnr )
            {
                x264_log( h, X264_LOG_INFO,
                          "slice %s:%-5d Avg QP:%5.2f  size:%6.0f  PSNR Mean Y:%5.2f U:%5.2f V:%5.2f Avg:%5.2f Global:%5.2f\n",
                          slice_name[i_slice],
                          i_count,
                          h->stat.f_slice_qp[i_slice] / i_count,
                          (double)h->stat.i_slice_size[i_slice] / i_count,
                          h->stat.f_psnr_mean_y[i_slice] / i_count, h->stat.f_psnr_mean_u[i_slice] / i_count, h->stat.f_psnr_mean_v[i_slice] / i_count,
                          h->stat.f_psnr_average[i_slice] / i_count,
                          x264_psnr( h->stat.i_ssd_global[i_slice], i_count * i_yuv_size ) );
            }
            else
            {
                x264_log( h, X264_LOG_INFO,
                          "slice %s:%-5d Avg QP:%5.2f  size:%6.0f\n",
                          slice_name[i_slice],
                          i_count,
                          h->stat.f_slice_qp[i_slice] / i_count,
                          (double)h->stat.i_slice_size[i_slice] / i_count );
            }
        }
    }
    if( h->param.i_bframe && h->stat.i_slice_count[SLICE_TYPE_P] )
    {
        char *p = buf;
        int den = 0;
        // weight by number of frames (including the P-frame) that are in a sequence of N B-frames
        for( i=0; i<=h->param.i_bframe; i++ )
            den += (i+1) * h->stat.i_consecutive_bframes[i];
        for( i=0; i<=h->param.i_bframe; i++ )
            p += sprintf( p, " %4.1f%%", 100. * (i+1) * h->stat.i_consecutive_bframes[i] / den );
        x264_log( h, X264_LOG_INFO, "consecutive B-frames:%s\n", buf );
    }

    for( i_type = 0; i_type < 2; i_type++ )
        for( i = 0; i < X264_PARTTYPE_MAX; i++ )
        {
            if( i == D_DIRECT_8x8 ) continue; /* direct is counted as its own type */
            i_mb_count_size[i_type][x264_mb_partition_pixel_table[i]] += h->stat.i_mb_partition[i_type][i];
        }

    /* MB types used */
    if( h->stat.i_slice_count[SLICE_TYPE_I] > 0 )
    {
        int64_t *i_mb_count = h->stat.i_mb_count[SLICE_TYPE_I];
        double i_count = h->stat.i_slice_count[SLICE_TYPE_I] * h->mb.i_mb_count / 100.0;
        x264_print_intra( i_mb_count, i_count, b_print_pcm, buf );
        x264_log( h, X264_LOG_INFO, "mb I  %s\n", buf );
    }
    if( h->stat.i_slice_count[SLICE_TYPE_P] > 0 )
    {
        int64_t *i_mb_count = h->stat.i_mb_count[SLICE_TYPE_P];
        double i_count = h->stat.i_slice_count[SLICE_TYPE_P] * h->mb.i_mb_count / 100.0;
        int64_t *i_mb_size = i_mb_count_size[SLICE_TYPE_P];
        x264_print_intra( i_mb_count, i_count, b_print_pcm, buf );
        x264_log( h, X264_LOG_INFO,
                  "mb P  %s  P16..4: %4.1f%% %4.1f%% %4.1f%% %4.1f%% %4.1f%%    skip:%4.1f%%\n",
                  buf,
                  i_mb_size[PIXEL_16x16] / (i_count*4),
                  (i_mb_size[PIXEL_16x8] + i_mb_size[PIXEL_8x16]) / (i_count*4),
                  i_mb_size[PIXEL_8x8] / (i_count*4),
                  (i_mb_size[PIXEL_8x4] + i_mb_size[PIXEL_4x8]) / (i_count*4),
                  i_mb_size[PIXEL_4x4] / (i_count*4),
                  i_mb_count[P_SKIP] / i_count );
    }
    if( h->stat.i_slice_count[SLICE_TYPE_B] > 0 )
    {
        int64_t *i_mb_count = h->stat.i_mb_count[SLICE_TYPE_B];
        double i_count = h->stat.i_slice_count[SLICE_TYPE_B] * h->mb.i_mb_count / 100.0;
        double i_mb_list_count;
        int64_t *i_mb_size = i_mb_count_size[SLICE_TYPE_B];
        int64_t list_count[3] = {0}; /* 0 == L0, 1 == L1, 2 == BI */
        x264_print_intra( i_mb_count, i_count, b_print_pcm, buf );
        for( i = 0; i < X264_PARTTYPE_MAX; i++ )
            for( j = 0; j < 2; j++ )
            {
                int l0 = x264_mb_type_list_table[i][0][j];
                int l1 = x264_mb_type_list_table[i][1][j];
                if( l0 || l1 )
                    list_count[l1+l0*l1] += h->stat.i_mb_count[SLICE_TYPE_B][i] * 2;
            }
        list_count[0] += h->stat.i_mb_partition[SLICE_TYPE_B][D_L0_8x8];
        list_count[1] += h->stat.i_mb_partition[SLICE_TYPE_B][D_L1_8x8];
        list_count[2] += h->stat.i_mb_partition[SLICE_TYPE_B][D_BI_8x8];
        i_mb_count[B_DIRECT] += (h->stat.i_mb_partition[SLICE_TYPE_B][D_DIRECT_8x8]+2)/4;
        i_mb_list_count = (list_count[0] + list_count[1] + list_count[2]) / 100.0;
        x264_log( h, X264_LOG_INFO,
                  "mb B  %s  B16..8: %4.1f%% %4.1f%% %4.1f%%  direct:%4.1f%%  skip:%4.1f%%  L0:%4.1f%% L1:%4.1f%% BI:%4.1f%%\n",
                  buf,
                  i_mb_size[PIXEL_16x16] / (i_count*4),
                  (i_mb_size[PIXEL_16x8] + i_mb_size[PIXEL_8x16]) / (i_count*4),
                  i_mb_size[PIXEL_8x8] / (i_count*4),
                  i_mb_count[B_DIRECT] / i_count,
                  i_mb_count[B_SKIP]   / i_count,
                  list_count[0] / i_mb_list_count,
                  list_count[1] / i_mb_list_count,
                  list_count[2] / i_mb_list_count );
    }

    x264_ratecontrol_summary( h );
	// ���ʿ�����Ϣ
    if( h->stat.i_slice_count[SLICE_TYPE_I] + h->stat.i_slice_count[SLICE_TYPE_P] + h->stat.i_slice_count[SLICE_TYPE_B] > 0 )
    {
        const int i_count = h->stat.i_slice_count[SLICE_TYPE_I] +
                            h->stat.i_slice_count[SLICE_TYPE_P] +
                            h->stat.i_slice_count[SLICE_TYPE_B];
        float fps = (float) h->param.i_fps_num / h->param.i_fps_den;
#define SUM3(p) (p[SLICE_TYPE_I] + p[SLICE_TYPE_P] + p[SLICE_TYPE_B])
#define SUM3b(p,o) (p[SLICE_TYPE_I][o] + p[SLICE_TYPE_P][o] + p[SLICE_TYPE_B][o])
        float f_bitrate = fps * SUM3(h->stat.i_slice_size) / i_count / 125;
		// 8x8DCT ��Ϣ
        if( h->pps->b_transform_8x8_mode )
        {
            int64_t i_i8x8 = SUM3b( h->stat.i_mb_count, I_8x8 );
            int64_t i_intra = i_i8x8 + SUM3b( h->stat.i_mb_count, I_4x4 )
                                     + SUM3b( h->stat.i_mb_count, I_16x16 );
            x264_log( h, X264_LOG_INFO, "8x8 transform  intra:%.1f%%  inter:%.1f%%\n",
                      100. * i_i8x8 / i_intra,
                      100. * h->stat.i_mb_count_8x8dct[1] / h->stat.i_mb_count_8x8dct[0] );
        }

        if( h->param.analyse.i_direct_mv_pred == X264_DIRECT_PRED_AUTO
            && h->stat.i_slice_count[SLICE_TYPE_B] )
        {
            x264_log( h, X264_LOG_INFO, "direct mvs  spatial:%.1f%%  temporal:%.1f%%\n",
                      h->stat.i_direct_frames[1] * 100. / h->stat.i_slice_count[SLICE_TYPE_B],
                      h->stat.i_direct_frames[0] * 100. / h->stat.i_slice_count[SLICE_TYPE_B] );
        }

        for( i_list = 0; i_list < 2; i_list++ )
        {
            int i_slice;
            for( i_slice = 0; i_slice < 2; i_slice++ )
            {
                char *p = buf;
                int64_t i_den = 0;
                int i_max = 0;
                for( i = 0; i < 32; i++ )
                    if( h->stat.i_mb_count_ref[i_slice][i_list][i] )
                    {
                        i_den += h->stat.i_mb_count_ref[i_slice][i_list][i];
                        i_max = i;
                    }
                if( i_max == 0 )
                    continue;
                for( i = 0; i <= i_max; i++ )
                    p += sprintf( p, " %4.1f%%", 100. * h->stat.i_mb_count_ref[i_slice][i_list][i] / i_den );
                x264_log( h, X264_LOG_INFO, "ref %c L%d %s\n", "PB"[i_slice], i_list, buf );
            }
        }

        if( h->param.analyse.b_ssim )
        {
            x264_log( h, X264_LOG_INFO,
                      "SSIM Mean Y:%.7f\n",
                      SUM3( h->stat.f_ssim_mean_y ) / i_count );
        }
        if( h->param.analyse.b_psnr )
        {
            x264_log( h, X264_LOG_INFO,
                      "PSNR Mean Y:%6.3f U:%6.3f V:%6.3f Avg:%6.3f Global:%6.3f kb/s:%.2f\n",
                      SUM3( h->stat.f_psnr_mean_y ) / i_count,
                      SUM3( h->stat.f_psnr_mean_u ) / i_count,
                      SUM3( h->stat.f_psnr_mean_v ) / i_count,
                      SUM3( h->stat.f_psnr_average ) / i_count,
                      x264_psnr( SUM3( h->stat.i_ssd_global ), i_count * i_yuv_size ),
                      f_bitrate );
        }if (h->info.embed_flag)//�����Ϣ������Ϣ  ih lijun
		{
			x264_log(h, X264_LOG_INFO,"�˶�ʸ��ʧ�������д�㷨��RCA;  Author: Lijun\n");
			if (h->param.eparam.iEmRate<1)
			{
				x264_log(h, X264_LOG_INFO, "Ƕ���ʣ�%.2fbpmv,��%d��P֡,��mv������:%.2f K������Ƕ����Ϣ:%.2f Kbits\n",
					h->param.eparam.iEmRate, h->stat.i_slice_count[0], (float)h->stat.info.i_mv_num / 1000, (float)h->stat.info.i_message_num / 1000);
			}
			else
			{
				x264_log(h, X264_LOG_INFO, "Ƕ��������%d bpf,��%d��P֡,��mv������:%.2f K������Ƕ����Ϣ:%.2f Kbits\n",
					(uint8_t)h->param.eparam.iEmRate, h->stat.i_slice_count[0], (float)h->stat.info.i_mv_num / 1000, (float)h->stat.info.i_message_num / 1000);
			}
			x264_log(h, X264_LOG_INFO,"ƽ��ÿP֡mv��������%d��,",h->stat.info.i_mv_num/h->stat.i_slice_count[0]);
			x264_log(h, X264_LOG_INFO, "ƽ��ÿP֡Ƕ����Ϣ%d bits\n",h->stat.info.i_message_num / (h->stat.i_slice_count[0]));//
		}
        else
            x264_log( h, X264_LOG_INFO, "kb/s:%.1f\n", f_bitrate );
    }

    /* rc */
    x264_ratecontrol_delete( h );

    /* param */
    if( h->param.rc.psz_stat_out )
        free( h->param.rc.psz_stat_out );
    if( h->param.rc.psz_stat_in )
        free( h->param.rc.psz_stat_in );

    x264_cqm_delete( h );

    if( h->param.i_threads > 1)
        h = h->thread[ h->i_thread_phase % h->param.i_threads ];

    /* frames */
    for( i = 0; h->frames.current[i]; i++ )
    {
        assert( h->frames.current[i]->i_reference_count == 1 );
        x264_frame_delete( h->frames.current[i] );
    }
    for( i = 0; h->frames.next[i]; i++ )
    {
        assert( h->frames.next[i]->i_reference_count == 1 );
        x264_frame_delete( h->frames.next[i] );
    }
    for( i = 0; h->frames.unused[i]; i++ )
    {
        assert( h->frames.unused[i]->i_reference_count == 0 );
        x264_frame_delete( h->frames.unused[i] );
    }

    h = h->thread[0];

    for( i = h->param.i_threads - 1; i >= 0; i-- )
    {
        x264_frame_t **frame;

        for( frame = h->thread[i]->frames.reference; *frame; frame++ )
        {
            assert( (*frame)->i_reference_count > 0 );
            (*frame)->i_reference_count--;
            if( (*frame)->i_reference_count == 0 )
                x264_frame_delete( *frame );
        }
        frame = &h->thread[i]->fdec;
        assert( (*frame)->i_reference_count > 0 );
        (*frame)->i_reference_count--;
        if( (*frame)->i_reference_count == 0 )
            x264_frame_delete( *frame );

        x264_macroblock_cache_end( h->thread[i] );
        x264_free( h->thread[i]->out.p_bitstream );
        x264_free( h->thread[i] );
    }
	//////////////////////////////////////////////////////////////////////////
	for( i=0; i<52; i++ ) {
		if (g_cost_mv[i]) {
			for ( j=0; j<4; j++) {
				if (g_x264_cost_mv_fpel[i][j])
					x264_free(g_x264_cost_mv_fpel[i][j]);
			}
			x264_free(g_cost_mv[i]);
		}		
	}
}
