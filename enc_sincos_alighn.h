#pragma once
#include "speed_torq_ctrl.h"
#include "virtual_speed_sensor.h"
#include "enc_sincos.h"

typedef struct
{
  SpeednTorqCtrl_Handle_t *pSTC;        /*!< Speed and torque controller object used by  EAC. */
  VirtualSpeedSensor_Handle_t *pVSS;    /*!< Virtual speed sensor object used by EAC. */
  EncSinCosConfigT *pENC;               /*!< Encoder object used by EAC. */
  uint16_t hRemainingTicks;             /*!< Number of tick events remaining to complete the alignment. */
  bool EncAligned;                      /*!< This flag is true if the encoder has been aligned at least once, false if
                                             has not. */
  bool EncRestart;                      /*!< This flag is used to force a restart of the motor after the encoder
                                             alignment. It is true if a restart is programmed otherwise, it is false*/
  uint16_t hEACFrequencyHz;             /*!< EAC_Exec() function calling frequency, in Hz. */
  int16_t hFinalTorque;                 /*!< Motor torque reference imposed by STC at the end of programmed alignment.
                                             This value actually is the Iq current expressed in digit. */
  int16_t hElAngle;                     /*!< Electrical angle of programmed alignment expressed in s16degrees
                                             [(rotor angle unit)](measurement_units.md). */
  uint16_t hDurationms;                 /*!< Duration of the programmed alignment expressed in milliseconds.*/
  uint8_t bElToMecRatio;                /*!< Coefficient used to transform electrical to mechanical quantities and
                                             vice-versa. It usually coincides with motor pole pairs number. */
} EncSinCosAlign_Handle_t;


void EAC_sincos_Init( EncSinCosAlign_Handle_t *pHandle, SpeednTorqCtrl_Handle_t *pSTC, VirtualSpeedSensor_Handle_t *pVSS, EncSinCosConfigT *pENC);

/* Function used to start the encoder alignment procedure */
void EAC_sincos_StartAlignment( EncSinCosAlign_Handle_t *pHandle );

/* Function used to execute the encoder alignment controller */
bool EAC_sincos_Exec( EncSinCosAlign_Handle_t *pHandle );

/* It returns true if the encoder has been aligned at least one time */
bool EAC_sincos_IsAligned( EncSinCosAlign_Handle_t *pHandle );

/* It sets a restart after an encoder alignment */
void EAC_sincos_SetRestartState( EncSinCosAlign_Handle_t *pHandle, bool restart );

/* Returns true if a restart after an encoder alignment has been requested */
bool EAC_sincos_GetRestartState( EncSinCosAlign_Handle_t *pHandle );
