/*
 * demo: CAN-BUS Shield, receive all frames and print all fields id/type/data
 * to receive frame fastly, a poll in loop() is required.
 *
 * Copyright (C) 2020 Seeed Technology Co.,Ltd.
 */
#include <SPI.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#define CAN_2515
// #define CAN_2518FD

// Set SPI CS Pin according to your hardware

#if defined(SEEED_WIO_TERMINAL) && defined(CAN_2518FD)
// For Wio Terminal w/ MCP2518FD RPi Hat：
// Channel 0 SPI_CS Pin: BCM 8
// Channel 1 SPI_CS Pin: BCM 7
// Interupt Pin: BCM25
const int SPI_CS_PIN  = BCM8;
const int CAN_INT_PIN = BCM25;
#else

// For Arduino MCP2515 Hat:
// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;
const int CAN_INT_PIN = 2;
#endif


#ifdef CAN_2518FD
#include "mcp2518fd_can.h"
mcp2518fd CAN(SPI_CS_PIN); // Set CS pin

// To TEST MCP2518FD CAN2.0 data transfer
#define MAX_DATA_SIZE 8
// To TEST MCP2518FD CANFD data transfer, uncomment below lines
// #undef  MAX_DATA_SIZE
// #define MAX_DATA_SIZE 64

#endif

#ifdef CAN_2515
#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin
#define MAX_DATA_SIZE 8
#endif

#define LED_PIN 13

int task1();
int task2();
int task3();
QueueHandle_t queue;
QueueHandle_t mailbox;

void setup() {
    SERIAL_PORT_MONITOR.begin(115200);
    while (!SERIAL_PORT_MONITOR) {}

    while (CAN_OK != CAN.begin(CAN_500KBPS)) {             // init can bus : baudrate = 500k
        SERIAL_PORT_MONITOR.println(F("CAN init fail, retry..."));
        delay(100);
    }
    SERIAL_PORT_MONITOR.println(F("CAN init ok!"));

    queue = xQueueCreate(10, sizeof(uint8_t)*MAX_DATA_SIZE);
    mailbox = xQueueCreate(1, sizeof(uint8_t)*MAX_DATA_SIZE);

    if (queue!=NULL) {
      xTaskCreate(task1, "task1", 128, NULL, 0, NULL);
      xTaskCreate(task2, "task2", 128, NULL, 0, NULL);
      xTaskCreate(task3, "task3", 128, NULL, 0, NULL);
    }
}

uint32_t id;
uint8_t  type; // bit0: ext, bit1: rtr
uint8_t  len;
byte cdata[MAX_DATA_SIZE] = {0};
byte cdata2[MAX_DATA_SIZE] = {0};
byte cdata3[MAX_DATA_SIZE] = {0};
uint8_t q_point = 0;

void task1(void* args) {
  int i, n;
  while(1) {
    CAN.readMsgBuf(&len, cdata);
    id = CAN.getCanId();
    if (id == 0x2) {
      xQueueOverwrite(queue, &cdata);
    }
  }
}

void task2(void* args) {
  int i, n;
  char prbuf[32 + MAX_DATA_SIZE * 3];
  while(1) {
    if(xQueueReceive(queue, &cdata2, portMAX_DELAY)) {
      for (i = 0; i < 8; i++) {
        n += sprintf(prbuf + n, "%02X ", cdata2[i]);
      }
      n = 0;
      SERIAL_PORT_MONITOR.println(prbuf);
      xQueueOverwrite(mailbox, &cdata2);
    }
  }
}

void task3(void* args) {
  while(1) {
    if(xQueueReceive(mailbox, &cdata3, portMAX_DELAY)) {
      if (cdata3[0] > 100) {
        digitalWrite(LED_BUILTIN, HIGH);
      } else {
        digitalWrite(LED_BUILTIN, LOW);
      }
    }
  }
}

void loop() {
//    // check if data coming
//    if (CAN_MSGAVAIL != CAN.checkReceive()) {
//        return;
//    }
//
//    char prbuf[32 + MAX_DATA_SIZE * 6];
//    int i, n;
//
//    unsigned long t = millis();
//    // read data, len: data length, buf: data buf
//    CAN.readMsgBuf(&len, cdata);
//
//    id = CAN.getCanId();
//    if (id == 0x1) {
//      for (i = 0; i < len; i++) {
//        n += sprintf(prbuf + n, "%02X ", cdata[i]);
//        cdata[i] = (cdata[i] >> 4) | ((cdata[i] & 0xF) << 4);
//        n += sprintf(prbuf + n, "%02X ", cdata[i]);
//      }
//      CAN.sendMsgBuf(0x5, CAN_STDID, len, cdata);
//      n = 0;
//      SERIAL_PORT_MONITOR.println(prbuf);
//    }
}
