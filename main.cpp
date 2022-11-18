/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */
#include <cstdint>
#include "mbed.h"
#include "LoRa_interface.h"
#include "GPS_interface.h"
#include "SD_interface.h"


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
    RTK_GET_RTCM_MSG,
    RTK_SEND_RTCM_MSG

}rtk_state;






//BufferedSerial pc(USBTX, USBRX);
UnbufferedSerial uart(PA_0, PA_1, 921600);
BufferedSerial pc(USBTX, USBRX, 115200);
SPI spi(MOSI_PIN, MISO_PIN, SCLK_PIN);
//bool txFlag = 0;
DigitalOut led(LED1);

int main()
{
    printf("---programm start Base---\n");

    // initalise serial spi ports
    spi.format(8, 0);
    spi.frequency(1000000);
    uart.format(8,SerialBase::None,1);
    

    //init SD card
    
    int _fakeint = 2;
    SDCARD sd(_fakeint);
    if(!sd.init()){
        printf("SD init failed\n");
    }

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
    uint8_t* rtcm_data = (uint8_t*)malloc(MAXIMUM_RTCM_MESSAGE_LENGTH*MAXIMUM_RTCM_MESSAGES); //ideally this is dynamic but this should suffice
    uint16_t rtcm_len = 0;
    uint8_t* ubx_data = (uint8_t*)malloc(200);
    uint16_t ubx_len = 0;
    rtk_state state = RTK_GET_RTCM_MSG;
    uint8_t loop = 1;
    uint8_t n = 0;
    //uint8_t i = 0;
    Timer timeout;
    
    led = 0;


    while(loop){

        switch(state){
            case(RTK_ERR):
                printf("something went wrong\n");
                ThisThread::sleep_for(1s);
                led = !led;
                
            break;
            case(RTK_IDLE):
                printf("idleing\n");
                ThisThread::sleep_for(1s);
                led = !led;


            break;
            case(RTK_TRANSMIT): // transmit state

                
                if(lora.event_handler() == TX_DONE){
                    state = RTK_RECEIVE;
                    lora.setModeContRX();
                    //printf("so far so good\n");
                    //ThisThread::sleep_for(500ms);
                    timeout.start();
                    timeout.reset();
                    
                }

            break;
            case(RTK_RECEIVE): // receive state
                led = 0;

                if(timeout.elapsed_time() >= 400ms){
                    lora.setModeIdle();
                    printf("timeout\n");
                    lora.n_payloads_sent = 0;
                    state = RTK_GET_RTCM_MSG;
                    timeout.stop();

                }

                if(lora.event_handler() == RX_DONE){
                    //printf("rx_done\n");
                    led = 1;
                    lora.setModeIdle();
                    lora.receive(data, rx_len);

                    
                    if(data[3] & 0x01){ //first bit of the flag byte indicates ack
                        lora.n_payloads_sent++;
                        if(lora.n_payloads){
                            //please pack this in a function
                            lora.n_payloads = lora.get_n_payloads(rtcm_len);
                            if(lora.n_payloads){
                                tx_len = RH_RF95_MAX_MESSAGE_LEN;
                                rtcm_len -= tx_len;
                            } else {
                                tx_len = rtcm_len % RH_RF95_MAX_MESSAGE_LEN;
                                rtcm_len = 0;
                            }
                            //printf("n=%d, tx=%d, buf=%d\n",lora.n_payloads, tx_len, buf_len);
                            if(!lora.transmit(rtcm_data + lora.n_payloads_sent*RH_RF95_MAX_MESSAGE_LEN, tx_len)){ //transmit data
                                printf("transmit failed\n");
                                state = RTK_ERR;
                                break;
                            }
                            state = RTK_TRANSMIT;
                            ////function end

                        } else{

                            printf("transmission complete total bytes: %d\n\n",(lora.n_payloads_sent-1)*RH_RF95_MAX_MESSAGE_LEN+tx_len);
                            lora.n_payloads_sent = 0;
                            state = RTK_GET_RTCM_MSG;
                            //gps.clearAll(); // for safety
                            timeout.stop();

                        }

                    }else{
                        if(!lora.transmit(rtcm_data + lora.n_payloads_sent*RH_RF95_MAX_MESSAGE_LEN, tx_len)){ //transmit data
                            printf("transmit failed\n");
                            state = RTK_ERR;
                            break;
                        }
                        printf("nack\n");
                        state = RTK_TRANSMIT;
                        lora.n_tries++;
                        if(lora.n_tries >= 5){
                            state = RTK_ERR;
                        }
                    }
                }
                /*
                if(lora.event_handler() == RX_BAD){
                    if(!lora.transmit(rtcm_data + lora.n_payloads_sent*RH_RF95_MAX_MESSAGE_LEN, tx_len)){ //transmit data
                        printf("transmit failed\n");
                        state = RTK_ERR;
                        break;
                    }
                    printf("nack\n");
                    state = RTK_TRANSMIT;
                    lora.n_tries++;
                    if(lora.n_tries >= 5){
                    state = RTK_ERR;
                    }
                }
                */

            break;
            case(RTK_GET_RTCM_MSG): //read uart
                //printf("congrats! no crash");
                led = 0;
                
                if(gps.data_ready()){
                    led = 1;
                    /*
                    printf("cpl = 0x");
                    print_hex((char*)gps.rtcm_msg, gps.rtcm_msg_pointer);
                    printf(" %d bytes\n", gps.rtcm_msg_pointer);
                    */

                    gps.decode();


                    if(gps.printMsgTypes()){
                        gps.encode_RTCM(rtcm_data, rtcm_len);
                        //printf("rtcm = 0x");
                        //print_hex((char*)rtcm_data, rtcm_len);
                        //printf(" %d bytes\n", rtcm_len);
                    } else {
                        printf("no rtcm\n");
                    }
                    
                    //gps.encode_UBX(ubx_data, ubx_len);
                    
                    //print_hex((char*)ubx_data, ubx_len);
                    //printf(" %d bytes\n", ubx_len);
                    

                    printf("total bytes from UART = %d\n", gps.rtcm_msg_pointer);
                    
                    //maybe pack this in a function
                    lora.n_payloads = lora.get_n_payloads(rtcm_len);
                    if(lora.n_payloads){
                        tx_len = RH_RF95_MAX_MESSAGE_LEN;
                        rtcm_len -= tx_len;
                    } else {
                        tx_len = rtcm_len % RH_RF95_MAX_MESSAGE_LEN;
                        rtcm_len = 0;
                    }
                    printf("n=%d, tx=%d, buf=%d\n",lora.n_payloads, tx_len, rtcm_len);
                    /////function end
                    if(!lora.transmit(rtcm_data, tx_len)){ //transmit data
                        printf("transmit failed\n");
                        state = RTK_ERR;
                        break;
                    }

                    //store ubx stuff to sd card
                    char buf[400];
                    uint16_t l = 0;
                    int i = 0;
                    while(gps.ubx[i].isvalid){
                        if(gps.ubx[i].ubx2string(buf, l)){
                            sd.write2sd(buf,l);
                        } else {
                            printf("no ubx[%d]\n", i);
                        }
                        i++;
                    }
                    sd.writeln();
                    gps.clearAll();

                    state = RTK_TRANSMIT;
                    
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

