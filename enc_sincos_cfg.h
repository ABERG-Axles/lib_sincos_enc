/*
 * enc_sincos_cfg.h
 *
 *  Created on: May 13, 2024
 *      Author: t.shypytiak
 */

#ifndef APPLICATION_USER_ENC_SINCOS_CFG_H_
#define APPLICATION_USER_ENC_SINCOS_CFG_H_

#define V_REG					3.3f
#define ADC_VOLTS( adc_val )	( ( float )adc_val / 4095.0f * V_REG )

#define SINCOS_MIN_AMPLITUDE        0.7         // sqrt(sin^2 + cos^2) has to be larger than this
#define SINCOS_MAX_AMPLITUDE        1.3         // sqrt(sin^2 + cos^2) has to be smaller than this

#define UPPER_BOUND_V			5.0f


/*
c_min 0.927
c_max 2.040
s_min 0.913
s_max 2.030


s_amp 0.558
c_amp 0.556
s_ofs 1.473
c_ofs 1.484
s_gain 1.790
c_gain 1.796


encoder offset 180
encoder ratio 7
sincos filter constant 0.5
 */


#define ENCODER_SIN_AMP			0.558f
#define ENCODER_COS_AMP			0.556f
#define ENCODER_SIN_OFFSET		1.473f
#define ENCODER_COS_OFFSET		1.484f
#define ENCODER_SINCOS_FILTER	0.2f // was 0.5f
#define ENCODER_SINCOS_PHASE	0.0f


#endif /* APPLICATION_USER_ENC_SINCOS_CFG_H_ */
