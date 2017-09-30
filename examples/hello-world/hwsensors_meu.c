/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/

#include "contiki.h"
#include "sys/etimer.h"
#include "sys/ctimer.h"
#include "dev/leds.h"
#include "dev/watchdog.h"
#include "random.h"
#include "button-sensor.h"
#include "board-peripherals.h"

#include "dev/adc-sensor.h"
#include "lib/sensors.h"
#include "ti-lib.h"
#include "lpm.h"

#include <stdio.h>
#include <stdint.h>

static struct etimer et_timer;
static struct sensors_sensor *sensor;

uint8_t pwm_request_max_pm(void){
    return LPM_MODE_DEEP_SLEEP;
}

void sleep_enter(void){
    leds_on(LEDS_RED);
}

void sleep_leave(void) {
    leds_off(LEDS_RED);
}

LPM_MODULE(pwmdrive_module, pwm_request_max_pm,sleep_enter, sleep_leave, LPM_DOMAIN_PERIPH);

int16_t pwminit(int32_t freq){
    uint32_t load = 0;
    ti_lib_ioc_pin_type_gpio_output(IOID_21);
    leds_off(LEDS_RED);

    /* Enable GPT0 clocks under active, sleep, deep sleep mode */
    ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_TIMER0);
    ti_lib_prcm_peripheral_sleep_enable(PRCM_PERIPH_TIMER0);
    ti_lib_prcm_peripheral_deep_sleep_enable(PRCM_PERIPH_TIMER0);
    ti_lib_prcm_load_set();
    while(!ti_lib_prcm_load_get());

    /* Register with LPM. This will keep the PERIPH PD powered on
    * during deep sleep, allowing the pwm to keep working while the chip is
    * being power-cycled */
    lpm_register_module(&pwmdrive_module);

    /* Drive the I/O ID with GPT0 / Timer A */
    ti_lib_ioc_port_configure_set(IOID_21, IOC_PORT_MCU_PORT_EVENT0,IOC_STD_OUTPUT);

    /* GPT0 / Timer A: PWM, Interrupt Enable */
    ti_lib_timer_configure(GPT0_BASE,TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);

    /* Stop the timers */
    ti_lib_timer_disable(GPT0_BASE, TIMER_A);
    ti_lib_timer_disable(GPT0_BASE, TIMER_B);
    if(freq > 0) {
        load = (GET_MCU_CLOCK / freq);
        ti_lib_timer_load_set(GPT0_BASE, TIMER_A, load);
        ti_lib_timer_match_set(GPT0_BASE, TIMER_A, load - 1);
        /* Start */
        ti_lib_timer_enable(GPT0_BASE, TIMER_A);
    }
    return load;
}

/*---------------------------------------------------------------------------*/
PROCESS(gpio_process, "gpio process");
PROCESS(adc_process, "adc process");
PROCESS(pwm_process, "pwm process");
PROCESS(exbtn_process, "exbtn process");
AUTOSTART_PROCESSES(&pwm_process);

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(gpio_process, ev, data)
{
    PROCESS_BEGIN();
    etimer_set(&et_timer, 1*CLOCK_SECOND);
    static unsigned int contador = 0;

    ti_lib_ioc_pin_type_gpio_output(IOID_25);
    ti_lib_ioc_pin_type_gpio_output(IOID_26);

    while(1){
        PROCESS_WAIT_EVENT();
        if(ev == PROCESS_EVENT_TIMER){
            contador++;
            printf("Contador: %d\n", contador);
            printf("BIT 01: %d\n", contador & 0x01);
            printf("BIT 02: %d\n", (contador & 0x02) >> 1);
            ti_lib_gpio_write_dio(25,contador & 0x01);
            ti_lib_gpio_write_dio(26,(contador & 0x02) >> 1);
            etimer_reset(&et_timer);

        }
    }
    PROCESS_END();
}

PROCESS_THREAD(adc_process, ev, data)
{
    PROCESS_BEGIN();
    etimer_set(&et_timer, 1*CLOCK_SECOND);
    sensor = sensors_find(ADC_SENSOR);
    static int valor = 0;

    while(1){
        PROCESS_WAIT_EVENT();
        if(ev == PROCESS_EVENT_TIMER){
            SENSORS_ACTIVATE(*sensor);
            sensor->configure(ADC_SENSOR_SET_CHANNEL,ADC_COMPB_IN_AUXIO2);
            valor = (int) sensor->value(ADC_SENSOR_VALUE);
            printf("Potencia: %d\n", valor);
            SENSORS_DEACTIVATE(*sensor);
            etimer_reset(&et_timer);
        }
    }

    PROCESS_END();
}

PROCESS_THREAD(pwm_process, ev, data)
{
    static int16_t current_duty = 0;
    static int16_t loadvalue, ticks;

    PROCESS_BEGIN();
    loadvalue = pwminit(5000);

    while(1) {
        PROCESS_WAIT_EVENT();
        if(ev == sensors_event){
            if(data == &button_left_sensor){
                if(current_duty < 100){
                    current_duty += 10;
                }
                printf("Current duty: %d\n ", current_duty);
                ticks = (current_duty * loadvalue) / 100;
                if(current_duty == 0){
                    ti_lib_timer_match_set(GPT0_BASE, TIMER_A, loadvalue - 1);
                } else {
                    ti_lib_timer_match_set(GPT0_BASE, TIMER_A, loadvalue - ticks);
                }
            } else if(data == &button_right_sensor){
                if(current_duty > 0){
                    current_duty -= 10;
                }
                printf("Current duty: %d\n ", current_duty);
                ticks = (current_duty * loadvalue) / 100;
                if(current_duty == 0){
                    ti_lib_timer_match_set(GPT0_BASE, TIMER_A, loadvalue - 1);
                } else {
                    ti_lib_timer_match_set(GPT0_BASE, TIMER_A, loadvalue - ticks);
                }
            }
        }
    }
    PROCESS_END();
}

PROCESS_THREAD(exbtn_process, ev, data)
{

    PROCESS_BEGIN();
    etimer_set(&et_timer, 1*CLOCK_SECOND);

    ti_lib_ioc_pin_type_gpio_input(IOID_25);
    ti_lib_gpio_clear_dio(IOID_25);

    while(1){
        PROCESS_WAIT_EVENT();
        printf("%d\n", ev);
        /*if(ti_lib_gpio_get_event_dio(IOID_25)){
            printf("EVENTO DO BOTAO EXTERNO!");
            //printf("Leitura: %d\n", ti_lib_gpio_read_dio(IOID_25));
            etimer_reset(&et_timer);
        }
        ti_lib_gpio_clear_dio(IOID_25);*/
    }
    PROCESS_END();
}
