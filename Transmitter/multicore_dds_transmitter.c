#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"

#include "hardware/sync.h"
#include "hardware/spi.h"

// Include standard libraries
#include <stdlib.h>
#include <string.h>

// Include PICO libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"

// Include hardware libraries
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "hardware/irq.h"

// Include custom library
#include "pt_cornell_rp2040_v1.h"
#include "nrf24_driver.h"
#include "pico/stdlib.h"

#include <tusb.h> // TinyUSB tud_cdc_connected()

// Get data from flex sensor 
#define FLEX_INDEX            26
#define ADC_INDEX_CHAN        0
#define FLEX_MIDDLE           27
#define ADC_MIDDLE_CHAN       1

#define FLEX_RING             28
#define ADC_RING_CHAN         2

uint8_t flex_out_index;  //flex sensor output
uint8_t flex_out_middle;  //flex sensor output
uint8_t flex_out_ring;  //flex sensor output
fn_status_t checks = 0;

//////////////////////////////////Transmiter //////////////////////////////////

// GPIO pin numbers
pin_manager_t my_pins = { 
    .sck = 2, 
    .copi = 3, 
    .cipo = 4, 
    .csn = 5, 
    .ce = 6
};

/**
* nrf_manager_t can be passed to the nrf_client_t
* initialise function, to specify the NRF24L01 
* configuration. If NULL is passed to the initialise 
* function, then the default configuration will be used.
*/
nrf_manager_t my_config = {
    // RF Channel 
    .channel = 120,

    // AW_3_BYTES, AW_4_BYTES, AW_5_BYTES
    .address_width = AW_5_BYTES,

    // dynamic payloads: DYNPD_ENABLE, DYNPD_DISABLE
    .dyn_payloads = DYNPD_ENABLE,

    // data rate: RF_DR_250KBPS, RF_DR_1MBPS, RF_DR_2MBPS
    .data_rate = RF_DR_2MBPS,

    // RF_PWR_NEG_18DBM, RF_PWR_NEG_12DBM, RF_PWR_NEG_6DBM, RF_PWR_0DBM
    .power = RF_PWR_NEG_12DBM,

    // retransmission count: ARC_NONE...ARC_15RT
    .retr_count = ARC_10RT,

    // retransmission delay: ARD_250US, ARD_500US, ARD_750US, ARD_1000US
    .retr_delay = ARD_500US 
};

// SPI baudrate
uint32_t my_baudrate = 10000000;

nrf_client_t my_nrf;

// result of packet transmission
fn_status_t success = 0;

//////////////////////////////////Transmiter ////////////////////////////////// 

// User input thread. User can change draw speed
static PT_THREAD (protothread_transmitter(struct pt *pt))
{
    PT_BEGIN(pt) ;
    while(1) {
        //////////////////////////////////Transmiter //////////////////////////////////
        my_nrf.tx_destination((uint8_t[]){0x37,0x37,0x37,0x37,0x37});
        success = my_nrf.send_packet(&flex_out_index, sizeof(flex_out_index));
        if (success)
        {               
            printf("\nPacket sent: flex_out_index: %d\n", flex_out_index);
        }

        my_nrf.tx_destination((uint8_t[]){0xC7,0xC7,0xC7,0xC7,0xC7});
        success = my_nrf.send_packet(&flex_out_middle, sizeof(flex_out_middle));
        if (success)
        {               
            printf("\nPacket sent: flex_out_middle: %d\n", flex_out_middle);
        }

        my_nrf.tx_destination((uint8_t[]){0xCA,0xC7,0xC7,0xC7,0xC7});
        success = my_nrf.send_packet(&flex_out_ring, sizeof(flex_out_ring));
        if (success)
        {               
            printf("\nPacket sent: flex_out_ring: %d\n", flex_out_ring);
        }

        //////////////////////////////////Transmiter //////////////////////////////////

    }
    PT_END(pt) ;
}

// User input thread. User can change draw speed
static PT_THREAD (protothread_adcs(struct pt *pt))
{
    PT_BEGIN(pt) ;
    while(1) {
        adc_select_input(ADC_INDEX_CHAN) ;
        flex_out_index = (uint8_t) adc_read();
        // printf("Index Flex sensor data is %d \n: ", flex_out_index); 

        adc_select_input(ADC_MIDDLE_CHAN) ;
        flex_out_middle = (uint8_t) adc_read();
        // printf("Middle Flex sensor data is %d \n: ", flex_out_middle); 

        adc_select_input(ADC_RING_CHAN) ;
        flex_out_ring = (uint8_t) adc_read();
        printf("Ring Flex sensor data is %d \n: ", flex_out_ring); 
    }
    PT_END(pt) ;
}

// Entry point for core 1
void core1_entry() {
    pt_add_thread(protothread_adcs) ;
    pt_schedule_start ;
}

int main() {
    // Initialize stdio/uart
    stdio_init_all();        

    //wait until the CDC ACM (serial port emulation) is connected
    while (!tud_cdc_connected()) 
    {
        sleep_ms(10);
    }

    //////////////////////////////////Transmiter //////////////////////////////////

    // initialise my_nrf
    nrf_driver_create_client(&my_nrf);

    // configure GPIO pins and SPI
    my_nrf.configure(&my_pins, my_baudrate);

    // not using default configuration (my_nrf.initialise(NULL)) 
    my_nrf.initialise(&my_config);

    my_nrf.dyn_payloads_enable();

    // set to Standby-I Mode
    my_nrf.standby_mode();

    ///////////////////////////////////////////////////////////////////////////////
    // ============================== ADC CONFIGURATION ==========================
    //////////////////////////////////////////////////////////////////////////////

    adc_gpio_init(FLEX_INDEX);
    adc_gpio_init(FLEX_MIDDLE);
    adc_gpio_init(FLEX_RING);
    adc_init() ;
    
    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // start core 1 
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    // start core 0
    pt_add_thread(protothread_transmitter) ;
    pt_schedule_start ;
    
}