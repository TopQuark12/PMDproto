/**
 * Edward ZHANG, 20171101, fked up by Alex Wong for volvo cart thing
 * @file    canBusProcess.c
 * @brief   CAN driver configuration file
 * @reference   RM2017_Archive
 */

#include "ch.h"
#include "hal.h"

#include <string.h>
#include "canBusProcess.h"

static volatile ChassisEncoder_canStruct chassis_encoder[CHASSIS_MOTOR_NUM];
static volatile Remote_canStruct remoteCommand;

/*
 * 500KBaud, automatic wakeup, automatic recover
 * from abort mode.
 * See section 22.7.7 on the STM32 reference manual.
 */
static const CANConfig cancfg = {
  CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP, //HAL LIB, hcan1.Init.ABOM = DISABLE;hcan1.Init.AWUM = DISABLE;
  CAN_BTR_SJW(0) | CAN_BTR_TS2(3) |
  CAN_BTR_TS1(8) | CAN_BTR_BRP(2)
};

#define CAN_FILTER_NUM 28U
static CANFilter canfilter[CAN_FILTER_NUM];

volatile ChassisEncoder_canStruct* can_getChassisMotor(void)
{
  return chassis_encoder;
}

volatile Remote_canStruct* can_get_remoteData(void){
  return &remoteCommand;
}

static inline void  can_processRemoteCommand
  (volatile Remote_canStruct* db, const CANRxFrame* const rxmsg)
{
    chSysLock();
    memcpy(db, rxmsg->data8, sizeof(uint8_t) * 5);
    db->updated = true;
    chSysUnlock();
}

static inline void can_getMotorOffset
        (volatile ChassisEncoder_canStruct* cm, const CANRxFrame* const rxmsg)
{
    chSysLock();
    cm->updated = true;
    cm->raw_angle = (uint16_t)(rxmsg->data8[0]) << 8 | rxmsg->data8[1];
    cm->raw_speed = (int16_t)(rxmsg->data8[2]) << 8 | rxmsg->data8[3];
    cm->act_current = (int16_t)(rxmsg->data8[4]) << 8 | rxmsg->data8[5];
    cm->temperature = (uint8_t)rxmsg->data8[6];
    chSysUnlock();

    cm->offset_raw_angle = cm->raw_angle;
}

static inline void can_processChassisEncoder
  (volatile ChassisEncoder_canStruct* cm, const CANRxFrame* const rxmsg)
{
    cm->last_raw_angle = cm->raw_angle;

    chSysLock();
    cm->updated = true;
    cm->raw_angle = (uint16_t)(rxmsg->data8[0]) << 8 | rxmsg->data8[1];
    cm->raw_speed = (int16_t)(rxmsg->data8[2]) << 8 | rxmsg->data8[3];
    cm->act_current = (int16_t)(rxmsg->data8[4]) << 8 | rxmsg->data8[5];
    cm->temperature = (uint8_t)rxmsg->data8[6];
    chSysUnlock();

    if      (cm->raw_angle - cm->last_raw_angle >  CAN_ENCODER_RANGE / 2) cm->round_count--;
    else if (cm->raw_angle - cm->last_raw_angle < -CAN_ENCODER_RANGE / 2) cm->round_count++;

    cm->total_ecd = cm->round_count * CAN_ENCODER_RANGE + cm->raw_angle - cm->offset_raw_angle;
    cm->radian_angle = cm->total_ecd * CAN_ENCODER_RADIAN_RATIO;
}

static void can_processEncoderMessage(CANDriver* const canp, const CANRxFrame* const rxmsg)
{
  if(canp == &CAND1)
  {
    switch(rxmsg->SID)
    {
        case CAN_CHASSIS_FL_FEEDBACK_MSG_ID:
            chassis_encoder[FRONT_LEFT].msg_count++;
            chassis_encoder[FRONT_LEFT].msg_count <= 50 ? can_getMotorOffset(&chassis_encoder[FRONT_LEFT],rxmsg) : can_processChassisEncoder(&chassis_encoder[FRONT_LEFT],rxmsg);
            break;
        case CAN_CHASSIS_FR_FEEDBACK_MSG_ID:
            chassis_encoder[FRONT_RIGHT].msg_count++;
            chassis_encoder[FRONT_RIGHT].msg_count <= 50 ? can_getMotorOffset(&chassis_encoder[FRONT_RIGHT],rxmsg) : can_processChassisEncoder(&chassis_encoder[FRONT_RIGHT],rxmsg);
            break;
        case CAN_CHASSIS_BL_FEEDBACK_MSG_ID:
            chassis_encoder[BACK_LEFT].msg_count++;
            chassis_encoder[BACK_LEFT].msg_count <= 50 ? can_getMotorOffset(&chassis_encoder[BACK_LEFT],rxmsg) : can_processChassisEncoder(&chassis_encoder[BACK_LEFT],rxmsg);
            break;
        case CAN_CHASSIS_BR_FEEDBACK_MSG_ID:
            chassis_encoder[BACK_RIGHT].msg_count++;
            chassis_encoder[BACK_RIGHT].msg_count <= 50 ? can_getMotorOffset(&chassis_encoder[BACK_RIGHT],rxmsg) : can_processChassisEncoder(&chassis_encoder[BACK_RIGHT],rxmsg);
          break;
    }
  }
  else
  {
    switch(rxmsg->SID)
    {
        case CAN_REMOTE_RECEIVER_ID:
        	can_processRemoteCommand(&remoteCommand,rxmsg);
					break;
    }
  }
}

/*
 * Receiver thread.
 */
static THD_WORKING_AREA(can_rx1_wa, 256);
static THD_WORKING_AREA(can_rx2_wa, 256);
static THD_FUNCTION(can_rx, p) {

  CANDriver* canp = (CANDriver*)p;
  event_listener_t el;
  CANRxFrame rxmsg;
  (void)p;
  chRegSetThreadName("can receiver");
  chEvtRegister(&canp->rxfull_event, &el, 0);
  while(!chThdShouldTerminateX())
  {
    if (chEvtWaitAnyTimeout(ALL_EVENTS, MS2ST(100)) == 0)
      continue;
    while (canReceive(canp, CAN_ANY_MAILBOX,
                      &rxmsg, TIME_IMMEDIATE) == MSG_OK)
    {
      can_processEncoderMessage(canp, &rxmsg);
    }
  }
  chEvtUnregister(&canp->rxfull_event, &el);
}

/*
 * @brief              Send motor current cmd using CAN driver
 * @param[in] cand     Pointer to CANDriver object we are currently using
 * @param[in] cmx_iq   Current (Torque) cmd of motor
 *
 * @notapi
 */
void can_motorSetCurrent(CANDriver *const CANx,
  const uint16_t EID,
  const int16_t cm1_iq,
  const int16_t cm2_iq,
  const int16_t cm3_iq,
  const int16_t cm4_iq)
{
    CANTxFrame txmsg;

    txmsg.IDE = CAN_IDE_STD;
    txmsg.EID = EID;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 0x08;

    chSysLock();
    txmsg.data8[0] = (uint8_t)(cm1_iq >> 8);
    txmsg.data8[1] = (uint8_t)cm1_iq;

    txmsg.data8[2] = (uint8_t)(cm2_iq >> 8);
    txmsg.data8[3] = (uint8_t)cm2_iq;

    txmsg.data8[4] = (uint8_t)(cm3_iq >> 8);
    txmsg.data8[5] = (uint8_t)cm3_iq;

    txmsg.data8[6] = (uint8_t)(cm4_iq >> 8);
    txmsg.data8[7] = (uint8_t)cm4_iq;
    chSysUnlock();

    canTransmit(CANx, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
}


void can_processInit(void)
{

  memset((void *)&remoteCommand, 0, sizeof(Remote_canStruct));

  uint8_t i;
  for (i = 0; i < CAN_FILTER_NUM; i++)
  {
    canfilter[i].filter = i;
    canfilter[i].mode = 0; //CAN_FilterMode_IdMask
    canfilter[i].scale = 1; //CAN_FilterScale_32bit
    canfilter[i].assignment = 0;
    canfilter[i].register1 = 0;
    canfilter[i].register2 = 0;
  }

  canSTM32SetFilters(14, CAN_FILTER_NUM, canfilter);

  canStart(&CAND1, &cancfg);
  canStart(&CAND2, &cancfg);

  /*
   * Starting the transmitter and receiver threads.
   */
  chThdCreateStatic(can_rx1_wa, sizeof(can_rx1_wa), NORMALPRIO + 7,
                    can_rx, (void *)&CAND1);
  chThdCreateStatic(can_rx2_wa, sizeof(can_rx2_wa), NORMALPRIO + 7,
                    can_rx, (void *)&CAND2);

  chThdSleepMilliseconds(20);
}
