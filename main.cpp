/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */
#include <cstdint>
#include "mbed.h"
#include "LoRa_interface.h"
#include "GPS_interface.h"


//Hardware connections
#define CS_PIN PB_1
#define INT_PIN PC_4
//SPI
#define MOSI_PIN PB_15
#define MISO_PIN PB_14
#define SCLK_PIN PB_13


void print_hex(const char *s, int len)
{
    while(len--){
        printf("%02x", (uint8_t) *s++);
    }
}




//BufferedSerial pc(USBTX, USBRX);
UnbufferedSerial uart(PA_0, PA_1, 921600);
BufferedSerial pc(USBTX, USBRX, 115200);
SPI spi(MOSI_PIN, MISO_PIN, SCLK_PIN);
bool txFlag = 0;
DigitalOut led(LED1);

int main()
{
    printf("---programm start Base---\n");

    // initalise serial spi ports
    spi.format(8, 0);
    spi.frequency(1000000);
    uart.format(8,SerialBase::None,1);
    

    // Create GPS object

    RTCM3_UBLOX gps(&uart);
    gps.init();

    // Create LoRa Object

    RFM95 lora(CS_PIN, INT_PIN, &spi);
    lora.init();


    uint8_t data[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(data);
    uint8_t* rtcm_data = NULL;
    rtcm_data = (uint8_t*)malloc(3000);
    uint16_t rtcm_len = 0;
    uint16_t rtcm_len2 = 0;
    int state = 3;
    uint8_t loop = 1;
    
    uint8_t n = 0;
    uint8_t i = 0;
    
    uint8_t buffer[] = {"Hello World! PING!"};
    led = 1;

    if(!lora.transmit(buffer, sizeof(buffer))){
        printf("transmit failed\n");
    }


    while(loop){

        switch(state){
            case 0:
                printf("something went wrong\n");
                ThisThread::sleep_for(2s);
                
            break;
            case 1: // transmit state

                
                if(lora.event_handler() == TX_DONE){
                    state = 2;
                    led = 0;
                    lora.setModeContRX();
                    ThisThread::sleep_for(500ms);
                    
                    
                }

            break;
            case 2: // receive state
                if(lora.event_handler() == RX_DONE){
                    lora.setModeIdle();
                    lora.receive(data, len);
                    printf("%s\n",data);
                    state = 1;
                    led = 1;
                    ThisThread::sleep_for(500ms);
                    if(!lora.transmit(buffer, sizeof(buffer))){ //transmit data
                        printf("transmit failed\n");
                        state = 0;
                        break;
                    }
                }

            break;
            case 3: //read uart
                //printf("congrats! no crash");
                
                if(gps.readCompleteMsg(rtcm_data, rtcm_len)){
                    printf("rx = 0x");
                    print_hex((char*)rtcm_data, rtcm_len);
                    printf(" %d bytes\n", rtcm_len);
                    
                    print_hex((char*)gps.rtcm_msg,gps.rtcm_msg_pointer);
                    printf("\n\n");
                    //led = !led;
                } else{
                    //no message
                }
                /*
                 
                if(!gps.msg_activity() & (gps.msg_pos == MSG_DATA)){
                    //printf("artifical bwebrltwebrtlwjhebrtkjwehbrtkwehjrtbwerjkthbwerjthb delay\n");
                    if(!gps.decode()){
                        gps.clearAll();
                        gps.msg_pos = MSG_IDLE;
                        break;
                    }

                    n = gps.msg_ready();

                    printf("recieved %d message|s\n", n);
                    rtcm_len = gps.getCompleteMsgLength();
                    printf("total bytes: %d/%d\n", rtcm_len, gps.rtcm_msg_pointer);

                    printf("the message's\n");
                    for(int i = 0; i < n; i++){
                        printf("0x%x l=%d 0x%x \n", gps.msg[i].preamble, gps.msg[i].length, gps.msg[i].crc);
                    }

                    print_hex((char*)gps.rtcm_msg,gps.rtcm_msg_pointer);
                    printf("\n\n");

                    /*
                    rtcm_data = (uint8_t*)malloc(rtcm_len);
                    printf("msg complete = 0x");
                    gps.readCompleteMsg(rtcm_data, rtcm_len2);
                    print_hex((char*)rtcm_data, rtcm_len);
                    free(rtcm_data);
                    */
                    /*
                    gps.clearAll();
                    gps.msg_pos = MSG_IDLE;
                }
                */
                    
                

                if(gps.msg_pos == MSG_ERR){
                    printf("something went wrong\n");
                    state = 4;
                }
                //ThisThread::sleep_for(200ms);
            break;
            default:
                printf("you shoudnt get this message\n");
                ThisThread::sleep_for(10s);
            break;
        }
        
    }
    printf("---program stop---\n");
    
    
    

}

