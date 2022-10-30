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

typedef enum{
    RTK_IDLE,
    RTK_ERR,
    RTK_TRANSMIT,
    RTK_RECEIVE,
    RTK_GET_RTCM_MSG

}rtk_state;






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
    uint8_t rx_len = 0;
    uint8_t tx_len = 0;
    uint8_t* rtcm_data = (uint8_t*)malloc(MAXIMUM_BUFFER_SIZE*MAXIMUM_MESSAGES);
    uint16_t rtcm_len = 0;
    uint16_t rtcm_len2 = 0;
    rtk_state state = RTK_IDLE;
    uint8_t loop = 1;
    
    uint8_t n = 0;
    //uint8_t i = 0;
    
    uint8_t buffer[] = {"Hello World! ultra long buffer to extend byte size so we can reach more than 250 bytes"};
    uint16_t buf_len = sizeof(buffer);

    led = 1;


    while(loop){

        switch(state){
            case(RTK_ERR):
                printf("something went wrong\n");
                ThisThread::sleep_for(5s);
                
            break;
            case(RTK_IDLE):
                ThisThread::sleep_for(1s);
                buf_len = sizeof(buffer);
                //maybe pack this in a function
                lora.n_payloads = lora.get_n_payloads(buf_len);
                if(lora.n_payloads){
                    tx_len = RH_RF95_MAX_MESSAGE_LEN;
                    buf_len -= tx_len;
                } else {
                    tx_len = buf_len % RH_RF95_MAX_MESSAGE_LEN;
                    buf_len = 0;
                }
                if(!lora.transmit(buffer, tx_len)){ //transmit data
                    printf("transmit failed\n");
                    state = RTK_ERR;
                    break;
                }
                state = RTK_TRANSMIT;


            break;
            case(RTK_TRANSMIT): // transmit state

                
                if(lora.event_handler() == TX_DONE){
                    state = RTK_RECEIVE;
                    led = 0;
                    lora.setModeContRX();
                    printf("so far so good\n");
                    //ThisThread::sleep_for(500ms);
                }

            break;
            case(RTK_RECEIVE): // receive state

                if(lora.event_handler() == RX_DONE){
                    lora.setModeIdle();
                    lora.receive(data, rx_len);


                    
                    if(data[4] & 0x01){ //first bit of the flag byte indicates ack
                        lora.n_payloads_sent++;
                        lora.n_payloads = lora.get_n_payloads(buf_len);
                        if(lora.n_payloads){
                            tx_len = RH_RF95_MAX_MESSAGE_LEN;
                            buf_len -= tx_len;
                        } else {
                            tx_len = buf_len % RH_RF95_MAX_MESSAGE_LEN;
                            buf_len = 0;
                        }
                        if(!lora.transmit(buffer + lora.n_payloads_sent*RH_RF95_MAX_MESSAGE_LEN, tx_len)){ //transmit data
                            printf("transmit failed\n");
                            state = RTK_ERR;
                            break;
                        }

                        if(lora.n_payloads){
                            state = RTK_TRANSMIT;
                        }else{
                            state = RTK_IDLE;
                        }
                    }else{
                        if(!lora.transmit(buffer + lora.n_payloads_sent*RH_RF95_MAX_MESSAGE_LEN, tx_len)){ //transmit data
                            printf("transmit failed\n");
                            state = RTK_ERR;
                            break;
                        }
                        lora.n_tries++;
                        if(lora.n_tries >= 5){
                            state = RTK_ERR;
                        }
                        
                    }
                    led = 1;
                    //ThisThread::sleep_for(500ms);
                    /*
                    if(!lora.transmit(buffer, sizeof(buffer))){ //transmit data
                        printf("transmit failed\n");
                        state = RTK_ERR;
                        break;
                    }
                    */
                }

            break;
            case(RTK_GET_RTCM_MSG): //read uart
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
                

                if(gps.msg_pos == MSG_ERR){
                    printf("something went wrong\n");
                    state = RTK_ERR;
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

