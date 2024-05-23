/*
 * sincos_encoder.h
 *
 *  Created on: May 13, 2024
 *      Author: t.shypytiak
 */

#ifndef APPLICATION_USER_ENC_SINCOS_H_
#define APPLICATION_USER_ENC_SINCOS_H_

#include <stdbool.h>
#include <stdint.h>

#include "stm32g474xx.h"
#include "speed_pos_fdbk.h"

typedef struct{
	uint32_t 	signal_below_min_error_cnt;
	uint32_t 	signal_above_max_error_cnt;
	float 		mech_angle_deg;				//!< mech rotor angle in [deg]
	
	float 		sin_filtered;
	float 		cos_filtered;
	uint32_t 	inj_adc_reading_sin;
 	uint32_t 	inj_adc_reading_cos;
}EncSinCosStateT;

typedef struct{
	SpeednPosFdbk_Handle_t _Super;                     /*!< SpeednPosFdbk  handle definition. */
	float 			s_gain;
	float 			c_gain;
	float 			s_offset;
	float 			c_offset;
    float           s_amp;
    float           c_amp;
	float 			filter_constant;
	float 			phase_corrrection;
	float 			sph; // sin of the phase_correction angle
	float 			cph; // cos of the phase_correction angle
	ADC_TypeDef* 	adcx1;
	uint32_t 		injected_channel_1;
	ADC_TypeDef* 	adcx2;
	uint32_t 		injected_channel_2;
	EncSinCosStateT	state;
	bool 			TimerOverflowError;                           /*!< true if the number of overflow  occurred is greater than
                                                          'define' ENC_MAX_OVERFLOW_NB*/
}EncSinCosConfigT;

bool enc_sincos_get_defaults( EncSinCosConfigT* pcfg );
void enc_sincos_shutdown( EncSinCosConfigT* pcfg );
void enc_sincos_calibrate( /*EncSinCosConfigT* pcfg,*/ uint32_t adc_value_sin, uint32_t adc_value_cos );

void enc_sincos_read_values( EncSinCosConfigT* pcfg );
void enc_sincos_calc_deg( EncSinCosConfigT* pcfg );

float enc_sincos_get_angle_deg( EncSinCosConfigT* pcfg );

// int16_t enc_sincos_get_angle_s16( EncSinCosConfigT* pcfg );
bool enc_sincos_CalcAvrgMecSpeedUnit( EncSinCosConfigT *pHandle, int16_t *pMecSpeedUnit );
int16_t enc_sincos_SPD_GetElAngle( EncSinCosConfigT* pcfg );
int16_t enc_sincos_SPD_GetS16Speed( EncSinCosConfigT* pcfg );
void enc_sincos_set_mec_angle( EncSinCosConfigT* pHandle, int16_t hMecAngle );


int16_t enc_sincos_get_raw_sin();
int16_t enc_sincos_get_raw_cos();

#endif /* APPLICATION_USER_ENC_SINCOS_H_ */
