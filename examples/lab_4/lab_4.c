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
#include "project-conf.h"
#include "dev/button-sensor.h"

#define IT 5

static struct timer nt;
static struct ctimer ct;
static struct etimer et;

static int retornaCor(){
    static int cor;
    cor = random_rand() % 2;
    return cor;
}



/*---------------------------------------------------------------------------*/
PROCESS(lab_4_process, "lab_4_process");
AUTOSTART_PROCESSES(&lab_4_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(lab_4_process, ev, data)
{
    PROCESS_BEGIN();

    SENSORS_ACTIVATE(button_sensor);

    etimer_set(&et, CLOCK_SECOND / 2);
    static int i,j,x;
    static int acertos = 0;
    static int led[4];
    static int jogada[4];
    static int venceu = 1;

    for (i = 0; i < IT; i++) {
        printf("Iteracao %d!\n", i);

        etimer_restart(&et);
        for(j=0;j<=i;j++){
            led[j] = retornaCor();

            PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
            etimer_restart(&et);
            if(led[j] == 0){
                controlaLed(0);
            } else {
                controlaLed(1);
            }
            PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
            controlaLed(3);
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
            if(led[j] != jogada[j]){
                venceu = 0;
            }
        }

        if(venceu == 0){
            controlaLed(2);
            printf("Última sequencia correta: ");
            for(j=0;j<=i;j++){
                if(led[j] == 0){
                    printf("Verde |");
                } else {
                    printf("Vermelho |");
                }
            }
            printf("\nPontuacao final: %d\n", acertos);
            break;
        } else {
            printf("Acertou!!!");
            acertos++;
        }

        etimer_restart(&et);
        PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
    }

    etimer_set(&et, CLOCK_SECOND / 2);
    if(venceu == 1){
        controlaLed(3);
        for (x = 0; x < 6; x++) {
            leds_toggle(LEDS_GREEN);
            leds_toggle(LEDS_RED);
            PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
            etimer_restart(&et);
        }
        printf("Parabens! Voce venceu!\n");
    }

    SENSORS_DEACTIVATE(button_sensor);

    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
