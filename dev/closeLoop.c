#include "ch.h"
#include "hal.h"

#include "closeLoop.h"
#include "canBusProcess.h"
#include <string.h>

ChassisEncoder_canStruct* chassisData;
float speedCommand;
float speedCommandNew;
float speedCommandPrev = 0;
float speedSetpoint;
float currentCommand[4];
pidProfile_t wheelPID[4];

float kP = 30;
float kI = 0.05;
float kD = 0;
float maxCurrent = 16384.0;
float maxI = 10000000;

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static uint8_t abs_limit(float *a, float ABS_MAX)
{
  if (*a > ABS_MAX) {
	  *a = ABS_MAX;
	  return 1;
  }

  if (*a < -ABS_MAX){
	  *a = -ABS_MAX;
	  return 1;
  }

  return 0;

}

static uint8_t maxed(int16_t a, float ABS_MAX)
{
  if (a > ABS_MAX) {
	  return 1;
  }

  if (a < -ABS_MAX){
	 return 1;
  }

  return 0;

}

void driveCloseLoop(void) {  

  for (uint8_t i = 0 ; i < 2 ; i++) {

  	uint8_t maxed = 0;
    wheelPID[i].error = speedSetpoint - chassisData[i].raw_speed;
    wheelPID[i].P = kP * wheelPID[i].error;
  	wheelPID[i].D = kD * (wheelPID[i].error - wheelPID[i].lastError);

  	 currentCommand[i] = wheelPID[i].P + wheelPID[i].D + (wheelPID[i].I * kI);
    //currentCommand[i] = speedSetpoint;
    maxed = abs_limit(&currentCommand[i], maxCurrent);

    wheelPID[i].I = maxed ? 0: wheelPID[i].error + wheelPID[i].I;
  	abs_limit(&wheelPID[i].I, maxI);

  	wheelPID[i].lastError = wheelPID[i].error;

  }



  for (uint8_t i = 2 ; i < 4 ; i++) {

  	uint8_t maxed = 0;
    wheelPID[i].error = -speedSetpoint - chassisData[i].raw_speed;
    wheelPID[i].P = kP * wheelPID[i].error;
  	wheelPID[i].D = kD * (wheelPID[i].error - wheelPID[i].lastError);

  	 currentCommand[i] = wheelPID[i].P + wheelPID[i].D + (wheelPID[i].I * kI);
    //currentCommand[i] = speedSetpoint;
    maxed = abs_limit(&currentCommand[i], maxCurrent);

    wheelPID[i].I = maxed ? 0: wheelPID[i].error + wheelPID[i].I;
  	abs_limit(&wheelPID[i].I, maxI);

  	wheelPID[i].lastError = wheelPID[i].error;

  }

}

static const ADCConversionGroup my_conversion_group = {
  FALSE,                            /*NOT CIRCULAR*/
  1,                              /*NUMB OF CH*/
  NULL,                             /*NO ADC CALLBACK*/
  NULL,                             /*NO ADC ERROR CALLBACK*/
  0,                                /* CR1 */
  ADC_CR2_SWSTART,                  /* CR2 */
  0,                                /* SMPR1 */
  ADC_SMPR2_SMP_AN0(ADC_SAMPLE_28), /* SMPR2 */
  ADC_SQR1_NUM_CH(1),               /* SQR1 */
  0,                                /* SQR2 */
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN10)  /* SQR3 */
};

float average (float newData)
{
  static float samples[10];
  static uint8_t sampleNo = 0;
  if (sampleNo >= 10)
    sampleNo = 0;
  samples[sampleNo] = newData;
  sampleNo++;
  float accum = 0;
  for (uint8_t i = 0; i < 10; i++) {
    accum += samples[i];
  }
  return accum / 10.0;
}

static volatile float maxRateOfChange = 5;
static volatile uint32_t counter = 0;

static THD_WORKING_AREA(chassisControlThd_wa, 1024);
static THD_FUNCTION(chassisControlThd, p) {

  (void)p;

  static systime_t now = 0;
  static systime_t next = 0;

  static volatile adcsample_t sampleBuf;

  static volatile adcsample_t initSampleBuf[50];

  memset(&speedSetpoint, 0, sizeof(speedSetpoint));
  memset(&currentCommand, 0, sizeof(currentCommand));
  memset(&wheelPID, 0, sizeof(pidProfile_t) * 4);
  memset(&sampleBuf, 0, sizeof(adcsample_t));

  palSetPadMode(GPIOC, GPIOC_PIN0, PAL_MODE_INPUT_ANALOG);

  adcStart(&ADCD1, NULL);

  adcConvert(&ADCD1, &my_conversion_group, (adcsample_t*) &initSampleBuf, 50);
  float accum = 0;
  for (uint8_t i = 0; i < 50; i++) {
    accum += initSampleBuf[i];
  }
  float zeroPos = accum / 50.0;

  while(true) {

    now = chVTGetSystemTime();
    next = now + US2ST(1000);

    // adcStart(&ADCD1, NULL);
    adcConvert(&ADCD1, &my_conversion_group, (adcsample_t*) &sampleBuf, 1);

    speedCommand = average(sampleBuf) - zeroPos;

    if (speedCommand < 50 && speedCommand > - 50)
      speedCommand = 0;

    speedCommand *= 1.5;

    if (speedCommand - speedSetpoint > maxRateOfChange)
    {
      speedSetpoint += maxRateOfChange;
      counter++;
    } else {
      if (speedSetpoint - speedCommand > maxRateOfChange)
        {
          speedSetpoint -= maxRateOfChange;
        } else {
          speedSetpoint = speedCommand;
        }
    }
    

    // speedSetpoint = speedCommand;
    // speedCommandPrev = speedCommand;

    driveCloseLoop();

    can_motorSetCurrent(&CAND1, CAN_CHASSIS_CONTROL_ID, currentCommand[0],
    		            currentCommand[1], currentCommand[2], currentCommand[3]);

    chThdSleepUntilWindowed(now, next);

  }

}

void chassisInit(void) {

  kP = 30;
  kI = 0.05;
  kD = 40;
  maxCurrent = 16384.0;
  maxI = 10000000;

  chassisData = can_getChassisMotor();
  //palSetPadMode(GPIOD, 14, PAL_MODE_INPUT_PULLDOWN);

  chThdCreateStatic(chassisControlThd_wa, sizeof(chassisControlThd_wa),
                    NORMALPRIO + 5, chassisControlThd, NULL);

}
