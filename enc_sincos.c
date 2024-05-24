/*
 * enc_sincos.c
 *
 *  Created on: May 13, 2024
 *      Author: t.shypytiak
 */
#include "enc_sincos.h"
#include "enc_sincos_cfg.h"
#include "enc_utils.h"
#include "enc_ms_timer.h"
#include "pmsm_motor_parameters.h"

#include <string.h>
#include <math.h>

#include "stm32g4xx_hal.h"
#include "parameters_conversion.h"
#include "tim.h"

extern ADC_HandleTypeDef hadc4;

#define IDLE   0
#define DONE   1
#define F_CLK  170000000U
#define PRESCALAR  340U
#define REF_CLK ( F_CLK / PRESCALAR )
#define MIN_FRQ ( REF_CLK / 65536U )
#define MAX_FRQ 100U

static  uint8_t state = IDLE;
static uint32_t t1 = 0;
static uint32_t t2 = 0;
static  uint32_t ticks = 0;
static uint32_t tim4_ovc = 0;
static  uint32_t freq = 0;
volatile float rpm = 0.0f;
volatile float rpm_prev = 0.0f;
float rpm_filtered = 0.0f;

volatile int16_t spd_s16 = 0;
// ---------------------------------------------- members ----------------------------------------------
//#define ENC_CALIBRATE
#ifdef ENC_CALIBRATE
#define FILTER_CONST 0.8f
static float out_sin = 0.0f;
static float out_cos = 0.0f;
static float out_mod = 0.0f;

static float s_amp = 1.0f;
static float c_amp = 1.0f;
static float s_gain = 1.0f;
static float c_gain = 1.0f;
static float s_ofs = 1.65f;
static float c_ofs = 1.65f;

static float s_max = 0.0f;
static float s_min = UPPER_BOUND_V;
static float c_max = 0.0f;
static float c_min = UPPER_BOUND_V;
static float raw_sin = 0.0f;
static float raw_cos = 0.0f;
#endif

volatile static int last_deg = 0.0f;
// uint32_t 	g_inj_adc_reading_sin;
// uint32_t 	g_inj_adc_reading_cos;


/*
1s16deg = 2PI / 65536
10.4 Measurement units
https://www.st.com/resource/en/user_manual/um1052-stm32f-pmsm-singledual-foc-sdk-v43-stmicroelectronics.pdf
*/
#define RAD2S16T_CONVERSION_FACTOR ( uint16_t )( 65536 / ( 2 * M_PI ) )



// ---------------------------------------------- private ----------------------------------------------
#ifdef ENC_CALIBRATE
static void calculate_amp_off(){
	s_ofs = 0.5f * ( s_max + s_min );
	c_ofs = 0.5f * ( c_max + c_min );
	s_amp = 0.5f * ( s_max - s_min );
	c_amp = 0.5f * ( c_max - c_min );
	s_gain = 1.0f / s_amp;
	c_gain = 1.0f / c_amp;
}
#endif

static inline uint32_t read_inj_channel( ADC_TypeDef* adcx, uint32_t channel ){
	switch ( channel ){
	    case ADC_INJECTED_RANK_4:
	      return adcx->JDR4;
	    case ADC_INJECTED_RANK_3:
	      return adcx->JDR3;
	      break;
	    case ADC_INJECTED_RANK_2:
	      return adcx->JDR2;
	      break;
	    case ADC_INJECTED_RANK_1:
	    default:
	      return adcx->JDR1;
	      break;
	  }
}

// ---------------------------------------------- interface ----------------------------------------------
bool enc_sincos_get_defaults( EncSinCosConfigT* pcfg ){
	memset( &pcfg->state, 0, sizeof( EncSinCosStateT ) );
#ifdef ENC_CALIBRATE
	out_sin = 0.0f;
	out_cos = 0.0f;
	out_mod = 0.0f;
	s_amp = 1.0f;
	c_amp = 1.0f;
	s_ofs = 1.65f;
	c_ofs = 1.65f;
	s_max = 0.0f;
	s_min = UPPER_BOUND_V;
	c_max = 0.0f;
	c_min = UPPER_BOUND_V;
    pcfg->adcx1					= ADC4;
	pcfg->injected_channel_1	= ADC_INJECTED_RANK_1;
	pcfg->adcx2					= ADC4;
	pcfg->injected_channel_2 	= ADC_INJECTED_RANK_2;
#else
    pcfg->s_gain 				= 1.0f / ENCODER_SIN_AMP;
	pcfg->s_offset				= ENCODER_SIN_OFFSET;
	pcfg->c_gain				= 1.0f / ENCODER_COS_AMP;
	pcfg->c_offset				= ENCODER_COS_OFFSET;
	pcfg->filter_constant		= ENCODER_SINCOS_FILTER;
    pcfg->s_amp                 = ENCODER_SIN_AMP;
    pcfg->c_amp                 = ENCODER_COS_AMP;
	pcfg->sph 					= sinf( DEG2RAD( ENCODER_SINCOS_PHASE ) );
	pcfg->cph 					= cosf( DEG2RAD( ENCODER_SINCOS_PHASE ) );
	pcfg->adcx1					= ADC4;
	pcfg->injected_channel_1	= ADC_INJECTED_RANK_1;
	pcfg->adcx2					= ADC4;
	pcfg->injected_channel_2 	= ADC_INJECTED_RANK_2;
	
	pcfg->_Super.bElToMecRatio             = POLE_PAIR_NUM;
    pcfg->_Super.hMaxReliableMecSpeedUnit  = (uint16_t)(1.15 * MAX_APPLICATION_SPEED_UNIT);
    pcfg->_Super.hMinReliableMecSpeedUnit  = (uint16_t)(MIN_APPLICATION_SPEED_UNIT);
    pcfg->_Super.bMaximumSpeedErrorsNumber = M1_SS_MEAS_ERRORS_BEFORE_FAULTS;
    pcfg->_Super.hMaxReliableMecAccelUnitP = 65535;
    pcfg->_Super.hMeasurementFrequency     = TF_REGULATION_RATE_SCALED;
    pcfg->_Super.DPPConvFactor             = DPP_CONV_FACTOR;
#endif
	
	return true;
}

void enc_sincos_shutdown( EncSinCosConfigT* pcfg ){
	memset( &pcfg->state, 0, sizeof( EncSinCosStateT ) );
}

bool enc_sincos_calc_deg( EncSinCosConfigT* pcfg ){
	uint32_t adc_value_sin = pcfg->state.inj_adc_reading_sin;
	uint32_t adc_value_cos = pcfg->state.inj_adc_reading_cos;
	float sin = ( ADC_VOLTS( adc_value_sin ) - pcfg->s_offset ) * pcfg->s_gain;
	float cos = ( ADC_VOLTS( adc_value_cos ) - pcfg->c_offset ) * pcfg->c_gain;
	LP_FAST( pcfg->state.sin_filtered, sin, pcfg->filter_constant );
	LP_FAST( pcfg->state.cos_filtered, cos, pcfg->filter_constant );
	sin = pcfg->state.sin_filtered;
	cos = pcfg->state.cos_filtered;

	//phase correction
	cos = (cos + sin * pcfg->sph) / pcfg->cph;
	float module = SQ( sin ) + SQ( cos );

	// signals vector outside of the valid area. Increase error count and discard measurement
	if( module > SQ( SINCOS_MAX_AMPLITUDE ) ){
		++ pcfg->state.signal_above_max_error_cnt;
		return false;
	}else{
		float ang_rad = utils_fast_atan2( sin, cos );
		pcfg->state.mech_angle_deg = RAD2DEG( ang_rad );
		int32_t mech_angle_s32 = ( int32_t )( ang_rad * RAD2S16T_CONVERSION_FACTOR );
		int32_t el_angle_s32 = mech_angle_s32 * ( int32_t )pcfg->_Super.bElToMecRatio;
        last_deg = ( int )pcfg->state.mech_angle_deg;

		int16_t hMecAnglePrev = pcfg->_Super.hMecAngle;
		pcfg->_Super.hMecAngle = (int16_t)mech_angle_s32;
		pcfg->_Super.hElAngle = (int16_t)el_angle_s32;
		int16_t hMecSpeedDpp = (int16_t)mech_angle_s32 - hMecAnglePrev;
    	pcfg->_Super.wMecAngle += ((int32_t)hMecSpeedDpp);
		return true;
	}
}

void enc_sincos_clear( EncSinCosConfigT *pHandle ){
	enc_sincos_shutdown( pHandle );
}

#ifdef ENC_CALIBRATE
void enc_sincos_calibrate( /*EncSinCosConfigT* pcfg,*/ uint32_t adc_value_sin, uint32_t adc_value_cos ){
    
	LP_FAST( raw_sin, ADC_VOLTS( adc_value_sin ), FILTER_CONST );
	LP_FAST( raw_cos, ADC_VOLTS( adc_value_cos ), FILTER_CONST );

	if( raw_sin > s_max ){
		s_max = raw_sin;
	}

	if( raw_sin < s_min ){
		s_min = raw_sin;
	}

	if( raw_cos > c_max ){
		c_max = raw_cos;
	}

	if( raw_cos > 0 && raw_cos < c_min ){
		c_min = raw_cos;
	}

	out_sin = ( raw_sin - s_ofs ) / s_amp;
	out_cos = ( raw_cos - c_ofs ) / c_amp;
	out_mod = sqrtf( SQ( out_sin ) + SQ( out_cos ) );
	float sdiff = s_max - s_min;
	float cdiff = c_max - c_min;

	if( sdiff > 0.1f && cdiff > 0.1f ){
		calculate_amp_off();
	}
}
#endif

bool enc_sincos_read_values( EncSinCosConfigT* pcfg ){
	HAL_ADC_Start(&hadc4);
	HAL_ADC_PollForConversion( &hadc4, 1);
	pcfg->state.inj_adc_reading_sin = HAL_ADC_GetValue(&hadc4);/*read_inj_channel( pcfg->adcx1, pcfg->injected_channel_1 );*/
    // g_inj_adc_reading_sin = pcfg->state.inj_adc_reading_sin;
	HAL_ADC_Start(&hadc4);
	HAL_ADC_PollForConversion( &hadc4, 1);
	pcfg->state.inj_adc_reading_cos = HAL_ADC_GetValue(&hadc4); /*read_inj_channel( pcfg->adcx2, pcfg->injected_channel_2 );*/
    // g_inj_adc_reading_cos = pcfg->state.inj_adc_reading_cos;
#ifdef ENC_CALIBRATE
    enc_sincos_calibrate( pcfg->state.inj_adc_reading_sin, pcfg->state.inj_adc_reading_cos );
#else
    return enc_sincos_calc_deg( pcfg );
#endif
}


//
// void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc){
//    if( hadc == &hadc4 ){
//        enc_sincos_read_values( &sincos_enc_cfg );
//    }
// }

 

float enc_sincos_get_angle_deg( EncSinCosConfigT* pcfg ){
//	return pcfg->state.mech_angle_deg;
    return pcfg->state.mech_angle_deg;
}

int16_t enc_sincos_SPD_GetElAngle( EncSinCosConfigT* pcfg ){
    return pcfg->_Super.hElAngle;
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim){
	if( htim == &htim3 ){
        // HAL_GPIO_WritePin( LEG_SIGNAL_COMP_GPIO_Port, LEG_SIGNAL_COMP_Pin, GPIO_PIN_SET);
        // HAL_GPIO_WritePin( LEG_SIGNAL_COMP_GPIO_Port, LEG_SIGNAL_COMP_Pin, GPIO_PIN_RESET);

        if(state == IDLE){
            t1 = HAL_TIM_ReadCapturedValue( htim, TIM_CHANNEL_1 );
            tim4_ovc = 0;
            state = DONE;
        }else if(state == DONE){
            t2 = HAL_TIM_ReadCapturedValue( htim, TIM_CHANNEL_1 );
            ticks = ( t2 + ( tim4_ovc * 65536 ) ) - t1;

            if( ticks > 0 ){
                uint32_t f = (uint32_t)( REF_CLK / ticks );

                if( f >=  MIN_FRQ && f <= MAX_FRQ ) {
                    freq = f;
                    rpm = freq * 60;
                    LP_FAST( rpm_filtered, rpm, 0.8f );
                }
            }

            __HAL_TIM_SET_COUNTER(htim, 0);
            state = IDLE;
        }
    }
}
#define ENC_MAX_OVERFLOW_NB     ((uint16_t)2)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
    if( htim == &htim3 ){
        tim4_ovc ++;
    }
}

bool enc_sincos_CalcAvrgMecSpeedUnit( EncSinCosConfigT *pHandle, int16_t *pMecSpeedUnit ){
	bool bReliability = true;
    int16_t mechSpd10Hz =  ( ( int16_t )rpm * SPEED_UNIT ) / U_RPM;
    *pMecSpeedUnit = mechSpd10Hz;
    // TODO: speed is calculated if only COMP calls TIM2 interrupt.
	// must understand that there is no motion.
    {
        /* Stores average mechanical speed */
        // pHandle->SpeedRefUnitExt = ((int32_t)hTargetFinal) * 65536;
        pHandle->_Super.hAvrMecSpeedUnit = (int16_t)mechSpd10Hz;
        rpm_prev = rpm;
    }
    
    spd_s16 = (int16_t)mechSpd10Hz;
    int32_t elSpd = ( int32_t )mechSpd10Hz * ( int32_t )pHandle->_Super.bElToMecRatio * 65536;
    pHandle->_Super.hElSpeedDpp = ( int16_t )elSpd;

	if ((tim4_ovc) > ENC_MAX_OVERFLOW_NB){
		pHandle->TimerOverflowError = true;
	}
    
	if (pHandle->TimerOverflowError)
    {
      bReliability = false;
    }else{
      bReliability = SPD_IsMecSpeedReliable(&pHandle->_Super, pMecSpeedUnit);
    }

	return bReliability;
}


int16_t enc_sincos_SPD_GetS16Speed( EncSinCosConfigT* pHandle ){
    int32_t wAux = (int32_t)pHandle->_Super.hAvrMecSpeedUnit;
    wAux *= INT16_MAX;
    wAux /= (int16_t)pHandle->_Super.hMaxReliableMecSpeedUnit;
    return (int16_t)wAux;
}
    


int16_t enc_sincos_get_raw_sin( EncSinCosConfigT* pHandle ){
    return pHandle->state.inj_adc_reading_sin;
}

int16_t enc_sincos_get_raw_cos( EncSinCosConfigT* pHandle ){
    return pHandle->state.inj_adc_reading_cos;
}

void enc_sincos_set_mec_angle( EncSinCosConfigT* pHandle, int16_t hMecAngle ){
    int16_t localhMecAngle = hMecAngle;
    pHandle->_Super.hMecAngle = localhMecAngle;
    pHandle->_Super.hElAngle = localhMecAngle * (int16_t)pHandle->_Super.bElToMecRatio;
}