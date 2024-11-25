//
// Provided as part of ELEC 471 Fall 2024
//

#ifndef ELEC471_BRAKEDATA_H
#define ELEC471_BRAKEDATA_H

/**
 * CAN IDs for the brake controller
 */
#define CAN_ID_ACCEL_PEDAL 0x08
#define CAN_ID_BRAKE_PEDAL 0x09
#define CAN_ID_MODE_SWITCH 0x0A
#define CAN_ID_VELOCITY    0x0B
#define CAN_ID_TURN_SIGNAL 0x0C
#define CAN_ID_DRIVER_DISP 0x10
#define CAN_ID_SET_SIGNAL  0xFE
#define CAN_ID_CLR_SIGNAL  0xFF

/**
 * Some constants and thresholds
 */
#define NOT_DEPRESSED    0x0000
#define FULLY_DEPRESSED  0xFFFF
#define ACCEL_THRESHOLD  0x6666
#define TWO_PEDAL_MODE   0x00
#define ONE_PEDAL_MODE   0x01
#define TURNSIG_BOTH_OFF 0x00
#define TURNSIG_RIGHT_ON 0x01
#define TURNSIG_LEFT_ON  0x10
#define TURNSIG_BOTH_ON  0x11

#endif //ELEC471_BRAKEDATA_H
