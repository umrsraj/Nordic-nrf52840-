/**
 * Copyright (c) 2014 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup nrf_adc_example main.c
 * @{
 * @ingroup nrf_adc_example
 * @brief ADC Example Application main file.
 *
 * This file contains the source code for a sample application using ADC.
 *
 * @image html example_board_setup_a.jpg "Use board setup A for this example."
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "boards.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"




static nrf_saadc_value_t adv_value;
 

//////////////////////////////////
static void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
    }
}

//***********************************
static void saadc_init()
{
    ret_code_t err_code;
    
    //VCE: The below block is for configuring the whole SAADC peripheral
    nrf_drv_saadc_config_t saadc_config;
    saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOWEST;
    saadc_config.low_power_mode = false;
    saadc_config.oversample = NRF_SAADC_OVERSAMPLE_DISABLED;
    saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;

    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);

    //VCE: The next 2 blocks configure one channel as single ended and the other as differential.
    nrf_saadc_channel_config_t channel_config_0;
    channel_config_0.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
    channel_config_0.resistor_n = NRF_SAADC_RESISTOR_DISABLED;      
    channel_config_0.gain       = NRF_SAADC_GAIN1_6;      
    channel_config_0.reference  = NRF_SAADC_REFERENCE_INTERNAL;
    channel_config_0.acq_time   = NRF_SAADC_ACQTIME_10US;     
    channel_config_0.mode       = NRF_SAADC_MODE_SINGLE_ENDED;
    channel_config_0.burst      = NRF_SAADC_BURST_DISABLED;      
    channel_config_0.pin_p      = NRF_SAADC_INPUT_AIN0;
    channel_config_0.pin_n      = NRF_SAADC_INPUT_DISABLED;
            
    err_code = nrf_drv_saadc_channel_init(0, &channel_config_0);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t channel_config_1;
    channel_config_1.resistor_p = NRF_SAADC_RESISTOR_PULLDOWN;
    channel_config_1.resistor_n = NRF_SAADC_RESISTOR_PULLUP;     
    channel_config_1.gain       = NRF_SAADC_GAIN1_2;      
    channel_config_1.reference  = NRF_SAADC_REFERENCE_VDD4;
    channel_config_1.acq_time   = NRF_SAADC_ACQTIME_40US;     
    channel_config_1.mode       = NRF_SAADC_MODE_DIFFERENTIAL;
    channel_config_1.burst      = NRF_SAADC_BURST_ENABLED;      
    channel_config_1.pin_p      = NRF_SAADC_INPUT_AIN1;
    channel_config_1.pin_n      = NRF_SAADC_INPUT_AIN2;
            
    err_code = nrf_drv_saadc_channel_init(1, &channel_config_1);
    APP_ERROR_CHECK(err_code);
}



////////////////////////////////////



/**
 * @brief Function for main application entry.
 */
 uint32_t adcc = 0;
int main(void)
{
    uint32_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    ret_code_t ret_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(ret_code);

    saadc_init();
    //saadc_sampling_event_init();
    //saadc_sampling_event_enable();


    NRF_LOG_INFO("SAADC HAL simple example started.");
    nrf_drv_saadc_sample(); // NOT required.

    while (1)
    {
        //nrf_pwr_mgmt_run();
         ret_code_t ret_code = nrf_drv_saadc_sample_convert(0, &adv_value);
         if(ret_code!= NRF_SUCCESS)
         {
            NRF_LOG_INFO("READING -- FAILED");
         }
        NRF_LOG_INFO("ADC event number: %d", (int)adv_value);
        adcc = (int)adv_value;
        nrf_delay_ms(1000);
        NRF_LOG_FLUSH();
    }
}


/** @} */
