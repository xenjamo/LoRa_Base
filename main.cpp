/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "LoRa_interface.h"


//Hardware connections
#define CS_PIN PB_6
#define INT_PIN PA_8
//SPI
#define MOSI_PIN PA_7
#define MISO_PIN PA_6
#define SCLK_PIN PA_5

//BufferedSerial pc(USBTX, USBRX);
SPI spi(MOSI_PIN, MISO_PIN, SCLK_PIN);
bool txFlag = 0;

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

    uint8_t data = lora.read(0x06);

    printf("data read from register = %x\n", data);

    uint8_t buf[] = {"Hello World!"};
    lora.transmit(buf, sizeof(buf));
    printf("transmitted something\n");
    

    while(1){
        //do nuthin'
        if((lora.flags & 0x40) == 0x40){
            printf("int triggered = 0x%x\n", lora.flag_handler());
        }
        
    }

}

