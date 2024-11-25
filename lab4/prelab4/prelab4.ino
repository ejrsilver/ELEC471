/*
 * demo: CAN-BUS Shield, receive all frames and print all fields id/type/data
 * to receive frame fastly, a poll in loop() is required.
 *
 * Copyright (C) 2020 Seeed Technology Co.,Ltd.
 */
#include <SPI.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include "brakeData.h"
#include "mcp2515_can.h"

#define CAN_2515
#define SPI_CS_PIN 9
#define CAN_INT_PIN 2
#define MAX_DATA_SIZE 8

mcp2515_can CAN(SPI_CS_PIN); // Set CS pin
QueueHandle_t accel_pedal;
QueueHandle_t brake_pedal;
QueueHandle_t mode_switch;
QueueHandle_t velocity;
QueueHandle_t accel_input;
QueueHandle_t driver_display;
QueueHandle_t logic_out;

int read_canbus();
int control_lights();
int write_display();

const byte warning_on[9] = "ErBrkON_", warning_off[9] = "ErBrkOFF";
uint32_t id;
uint8_t  light_status, high_val = 1, low_val = 0, logic_val, mode_val, accel_in_val, len, q_point = 0, type; // bit0: ext, bit1: rtr
uint16_t accel_val, shifting_val, brake_val, vel_reading[2], vel_val[2] = {0}, vel_val_prev[2] = {0};
byte cdata[MAX_DATA_SIZE] = {0};


void setup() {
  SERIAL_PORT_MONITOR.begin(115200);
  while (!SERIAL_PORT_MONITOR) {}

  while (CAN_OK != CAN.begin(CAN_500KBPS)) {             // init can bus : baudrate = 500k
    SERIAL_PORT_MONITOR.println(F("CAN init fail, retry..."));
    delay(100);
  }
  SERIAL_PORT_MONITOR.println(F("CAN init ok!"));

  accel_pedal = xQueueCreate(1, sizeof(uint16_t));
  brake_pedal = xQueueCreate(1, sizeof(uint16_t));
  mode_switch = xQueueCreate(1, sizeof(uint8_t));
  velocity = xQueueCreate(5, sizeof(uint16_t)*2);
  accel_input = xQueueCreate(1, sizeof(uint8_t));
  driver_display = xQueueCreate(1, sizeof(uint8_t)*8);
  logic_out = xQueueCreate(1, sizeof(uint8_t));

  if(accel_pedal != NULL && brake_pedal != NULL && mode_switch != NULL && 
     velocity != NULL && accel_input != NULL && driver_display != NULL && logic_out != NULL) {
    xTaskCreate(read_canbus, "read_messages", 128, NULL, 0, NULL);
    xTaskCreate(control_lights, "control_lights", 128, NULL, 0, NULL);
    xTaskCreate(write_display, "write_messages", 128, NULL, 0, NULL);
  }
}

void read_canbus(void* args) {
  while(1) {
    CAN.readMsgBuf(&len, cdata);
    id = CAN.getCanId();
    switch(id) {
      case CAN_ID_ACCEL_PEDAL:
        // Combine the lower 16 bits into a single value for the queue.
        shifting_val = (((uint16_t) cdata[1] & 0xFF) << 8) | cdata[0] & 0xFF;
        xQueueOverwrite(accel_pedal, &shifting_val);
        break;
      case CAN_ID_BRAKE_PEDAL:
        // Combine the lower 16 bits into a single value for the queue.
        shifting_val = (((uint16_t) cdata[1] & 0xFF) << 8) | cdata[0] & 0xFF;
        xQueueOverwrite(brake_pedal, &shifting_val);
        break;
      case CAN_ID_MODE_SWITCH:
        xQueueOverwrite(mode_switch, &cdata);
        break;
      case CAN_ID_VELOCITY:
        vel_reading[0] = (((uint16_t) cdata[1] & 0xFF) << 8) | cdata[0] & 0xFF;
        vel_reading[1] = millis();
        // Combine the lower 16 bits into a single value for the queue.
        xQueueOverwrite(velocity, &vel_reading);
        break;
      case CAN_ID_SET_SIGNAL:
        xQueueOverwrite(accel_input, &cdata);
        break;
      default:
        break;
    }
  }
}

uint16_t calc_accel(uint16_t* cur, uint16_t* prev) {
  return (abs(cur[0] - prev[0])/(abs(cur[1] - prev[1])*1000));
}

void control_lights(void* args) {
  while(1) {
    vel_val_prev[0] = vel_val[0];
    vel_val_prev[1] = vel_val[1];
    xQueueReceive(accel_pedal, &accel_val, portMAX_DELAY);
    xQueueReceive(brake_pedal, &brake_val, portMAX_DELAY);
    xQueueReceive(mode_switch, &mode_val, portMAX_DELAY);
    xQueueReceive(velocity, &vel_val, portMAX_DELAY);
    xQueueReceive(accel_input, &accel_in_val, portMAX_DELAY);

    // All the logic from the Simulink model.
    if (
      (brake_val > 0) ||
      (mode_val == ONE_PEDAL_MODE && accel_val <= ACCEL_THRESHOLD) ||
      (accel_in_val >= 1.3 && vel_val_prev[1] != 0 && calc_accel(vel_val, vel_val_prev) >= 1.3)
    ) {
      xQueueOverwrite(logic_out, &high_val);
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      xQueueOverwrite(logic_out, &low_val);
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}

void write_display(void* args) {
  while(1) {
    if(xQueueReceive(logic_out, &logic_val, portMAX_DELAY)) {
      light_status = digitalRead(LED_BUILTIN);
      if (logic_val && !light_status) {
        CAN.sendMsgBuf(CAN_ID_DRIVER_DISP, CAN_STDID, 8, warning_off);
      } else if (!logic_val && light_status) {
        CAN.sendMsgBuf(CAN_ID_DRIVER_DISP, CAN_STDID, 8, warning_on);
      }
    }
  }
}

void loop() {}
