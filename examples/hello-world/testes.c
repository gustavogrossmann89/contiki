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
#include "dev/button-sensor.h"
#include <random.h>
#include "dev/adc-sensor.h"
#include "lib/sensors.h"
#include "ti-lib.h"
#include "lpm.h"

#include <stdio.h>
#include <stdint.h>

#define IT 5

static struct sensors_sensor *sensor;
static struct etimer et;

static int retornaLampada(){
    static int lampada;
    lampada = random_rand() % 2;
    return lampada;
}


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
PROCESS(trabalho_process, "trabalho process");
PROCESS(teste_process, "teste process");
PROCESS(testebtn_process, "testebtn process");
PROCESS(adc_process, "adc process");
PROCESS(pwm_process, "pwm process");
AUTOSTART_PROCESSES(&pwm_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(trabalho_process, ev, data)
{
    PROCESS_BEGIN();

    SENSORS_ACTIVATE(button_sensor);

    etimer_set(&et, CLOCK_SECOND * 3);
    static int i,j,x;
    static int acertos = 0;
    static int lampada[4];
    static int jogada[4];
    static int venceu = 1;
    static int16_t loadvalue;

    //IOCPinTypeGpioOutput(IOID_21);
    IOCPinTypeGpioOutput(IOID_22);

    //GPIO_clearDio(IOID_21);
    GPIO_clearDio(IOID_22);

    loadvalue = pwminit(10000);

    for (i = 0; i < IT; i++) {
        printf("\nIteracao %d!\n", i);

        etimer_restart(&et);
        for(j=0;j<=i;j++){
            lampada[j] = retornaLampada();

            PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
            etimer_restart(&et);
            if(lampada[j] == 0){
                //GPIO_setDio(IOID_21);
                ti_lib_timer_match_set(GPT0_BASE, TIMER_A, loadvalue - 1);
            } else {
                GPIO_setDio(IOID_22);
            }
            PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
            ti_lib_timer_match_set(GPT0_BASE, TIMER_A, loadvalue - 1);
            //GPIO_clearDio(IOID_21);
            GPIO_clearDio(IOID_22);
            etimer_restart(&et);
        }

        for(j=0;j<=i;j++){
            PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event);
            if (data == &button_right_sensor) {
                jogada[j] = 0;
            } else {
                jogada[j] = 1;
            }
        }


        for(j=0;j<=i;j++){
            if(lampada[j] != jogada[j]){
                venceu = 0;
            }
        }

        if(venceu == 0){
            //GPIO_setDio(IOID_21);
            ti_lib_timer_match_set(GPT0_BASE, TIMER_A, loadvalue);
            GPIO_setDio(IOID_22);
            printf("\nUltima sequencia correta: ");
            for(j=0;j<=i;j++){
                if(lampada[j] == 0){
                    printf("Lampada (BTN DIR) |");
                } else {
                    printf("LED (BTN ESQ) |");
                }
            }
            printf("\nPontuacao final: %d\n", acertos);
            break;
        } else {
            printf("Acertou!!!\n");
            acertos++;
        }

        etimer_restart(&et);
        PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
    }

    etimer_set(&et, CLOCK_SECOND / 2);
    if(venceu == 1){
        //GPIO_clearDio(IOID_21);
        ti_lib_timer_match_set(GPT0_BASE, TIMER_A, loadvalue -1);
        GPIO_clearDio(IOID_22);
        for (x = 0; x < 6; x++) {
            etimer_restart(&et);
            //GPIO_setDio(IOID_21);
            ti_lib_timer_match_set(GPT0_BASE, TIMER_A, loadvalue);
            GPIO_setDio(IOID_22);
            PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
            etimer_restart(&et);
            ti_lib_timer_match_set(GPT0_BASE, TIMER_A, loadvalue -1);
            //GPIO_clearDio(IOID_21);
            GPIO_clearDio(IOID_22);
            PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
        }
        printf("Parabens! Voce venceu!\n");
    }

    SENSORS_DEACTIVATE(button_sensor);

    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(teste_process, ev, data)
{
    PROCESS_BEGIN();
    IOCPinTypeGpioOutput(IOID_21);
    GPIO_clearDio(IOID_21);
    IOCPinTypeGpioOutput(IOID_22);
    GPIO_clearDio(IOID_22);

    etimer_set(&et, CLOCK_SECOND * 10);
    while(1){
        GPIO_setDio(IOID_21);
        GPIO_setDio(IOID_22);
        PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
        etimer_restart(&et);
        GPIO_clearDio(IOID_21);
        GPIO_clearDio(IOID_22);
        PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
        etimer_restart(&et);
    }
    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(testebtn_process, ev, data)
{
    PROCESS_BEGIN();
    IOCPinTypeGpioOutput(IOID_21);
    GPIO_clearDio(IOID_21);

    while(1){
        PROCESS_WAIT_EVENT();
        if(ev == sensors_event){
            if(data == &button_left_sensor){
                GPIO_setDio(IOID_21);
            } else if(data == &button_right_sensor){
                GPIO_clearDio(IOID_21);
            }
        }
    }
    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(adc_process, ev, data)
{
    PROCESS_BEGIN();
    etimer_set(&et, 1*CLOCK_SECOND);
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
            etimer_reset(&et);
        }
    }

    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(pwm_process, ev, data)
{
    static int16_t current_duty = 0;
    static int16_t loadvalue, ticks;

    PROCESS_BEGIN();
    loadvalue = pwminit(12000);

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
