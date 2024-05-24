#include "enc_sincos_alighn.h"
#include "mc_type.h"

void EAC_sincos_Init( EncSinCosAlign_Handle_t *pHandle, SpeednTorqCtrl_Handle_t *pSTC, VirtualSpeedSensor_Handle_t *pVSS, EncSinCosConfigT *pENC){
    pHandle->pSTC = pSTC;
    pHandle->pVSS = pVSS;
    pHandle->pENC = pENC;
    pHandle->EncAligned = false;
    pHandle->EncRestart = false;
}

#define PHASE_al_FINAL_SPEED_UNIT         (500*SPEED_UNIT/U_RPM)

void EAC_sincos_StartAlignment( EncSinCosAlign_Handle_t *pHandle ){
    uint32_t wAux;
    VSS_SetMecAcceleration( pHandle->pVSS, 0, 0U );
    VSS_SetMecAngle( pHandle->pVSS, pHandle->hElAngle );
    STC_SetControlMode( pHandle->pSTC, MCM_TORQUE_MODE );
    (void)STC_ExecRamp( pHandle->pSTC, 0, 0U );
    (void)STC_ExecRamp( pHandle->pSTC, pHandle->hFinalTorque, (uint32_t)pHandle->hDurationms );
    VSS_SetMecAcceleration(pHandle->pVSS,PHASE_al_FINAL_SPEED_UNIT,0);
    wAux = ((uint32_t)pHandle->hDurationms) * ((uint32_t)pHandle->hEACFrequencyHz);
    wAux /= 1000U;
    pHandle->hRemainingTicks = (uint16_t)wAux;
    pHandle->hRemainingTicks++;
}

bool EAC_sincos_Exec( EncSinCosAlign_Handle_t *pHandle ){
    bool retVal = true;

    if( pHandle->hRemainingTicks > 0 ){
        pHandle->hRemainingTicks--;

        if (0U == pHandle->hRemainingTicks){
            enc_sincos_set_mec_angle( pHandle->pENC, pHandle->hElAngle / ((int16_t)pHandle->bElToMecRatio) );
            pHandle->EncAligned = true;
            retVal = true;
        }else{
            retVal = false;
        }
    }

    return retVal;
}

bool EAC_sincos_IsAligned( EncSinCosAlign_Handle_t *pHandle ){
    return pHandle->EncAligned;
}

void EAC_sincos_SetRestartState( EncSinCosAlign_Handle_t *pHandle, bool restart ){
    pHandle->EncRestart = restart;
}

bool EAC_sincos_GetRestartState( EncSinCosAlign_Handle_t *pHandle ){
    return pHandle->EncRestart;
}

