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

#define SINCOS_MIN_AMPLITUDE        0.7f         // sqrt(sin^2 + cos^2) has to be larger than this
#define SINCOS_MAX_AMPLITUDE        1.4f         // sqrt(sin^2 + cos^2) has to be smaller than this

#define UPPER_BOUND_V			5.0f


#define ENCODER_SIN_AMP			0.478f
#define ENCODER_COS_AMP			0.490f
#define ENCODER_SIN_OFFSET		1.257f
#define ENCODER_COS_OFFSET		1.237f

#define ENCODER_SINCOS_FILTER	0.5f // was 0.5f
#define ENCODER_SINCOS_PHASE	0.0f


#endif /* APPLICATION_USER_ENC_SINCOS_CFG_H_ */
