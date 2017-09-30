/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         A very simple Contiki application showing how Contiki programs look
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include <stdio.h>
#include <stdint.h>
#include <random.h>
#include "contiki.h"
#include "sys/timer.h"
#include "sys/ctimer.h"
#include "sys/etimer.h"
#include "dev/leds.h"
#include "dev/button-sensor.h"

#define IT 10

static struct timer nt;
static struct ctimer ct;
static struct etimer et;

static void ctimer_callback_ex(void *ptrValor){
    printf("Deu certo!\n");
    leds_toggle(LEDS_GREEN);
    leds_toggle(LEDS_RED);
}

static int acendeLed(){
    static int valor;
    valor = random_rand() % 2;
    printf("Valor: %d\n",valor);
    if(valor == 0){
        leds_on(LEDS_GREEN);
    } else {
        leds_on(LEDS_RED);
    }
    return valor;
}

/*---------------------------------------------------------------------------*/
PROCESS(lab_3_process, "lab_3_process");
AUTOSTART_PROCESSES(&lab_3_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(lab_3_process, ev, data)
{
    PROCESS_BEGIN();

    SENSORS_ACTIVATE(button_sensor);

    etimer_set(&et, CLOCK_SECOND * 2);
    static int i;
    static int acertos = 0;
    for (i = 0; i < IT; i++) {
        printf("Iteracao %d!\n", i);
        static int led;
        led = acendeLed();
        etimer_restart(&et);
        PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER || ev == sensors_event);
        if (data == &button_right_sensor) {
            if(led == 0){
                acertos++;
                printf("Acertou Verde!\n");
            }
        } else if (data == &button_left_sensor){
            if(led == 1){
                acertos++;
                printf("Acertou Vermelho!\n");
            }
        } else {
            printf("Demorou!\n");
        }

        leds_off(LEDS_GREEN);
        leds_off(LEDS_RED);

        etimer_restart(&et);
        PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
    }

    printf("Qtde acertos: %d\n",acertos);

    SENSORS_DEACTIVATE(button_sensor);

    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
