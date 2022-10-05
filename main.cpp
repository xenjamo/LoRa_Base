/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */
#include <cstdint>
#include "mbed.h"
#include "LoRa_interface.h"


//Hardware connections
#define CS_PIN PB_1
#define INT_PIN PA_8
//SPI
#define MOSI_PIN PB_15
#define MISO_PIN PB_14
#define SCLK_PIN PB_13

//BufferedSerial pc(USBTX, USBRX);
SPI spi(MOSI_PIN, MISO_PIN, SCLK_PIN);
bool txFlag = 0;
DigitalOut led(LED1);

int main()
{
    printf("---programm start---\n");
    // initalise spi port
    spi.format(8, 0);
    spi.frequency(1000000);

    // Create LoRa Object
    RFM95 lora(CS_PIN, INT_PIN, &spi);
    // initalise LoRa
    lora.init();

    uint8_t loop = 1;
    while(loop){
        uint8_t buf[] = {"Hello World!"};
        led = 1;
        if(!lora.transmit(buf, sizeof(buf))){
            printf("transmit failed\n");
            break;
        }
        if(!lora.waitForTransmission()){
            printf("timeout\n");
            break;
        }
        led = 0;
        loop++;
        ThisThread::sleep_for(5000ms);
    }
    printf("---program stop---\n");
    
    
    

}

