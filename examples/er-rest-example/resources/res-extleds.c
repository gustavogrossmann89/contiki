/*
 * Copyright (c) 2013, Institute for Pervasive Computing, ETH Zurich
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
 */

/**
 * \file
 *      Example resource
 * \author
 *      Matthias Kovatsch <kovatsch@inf.ethz.ch>
 */

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "rest-engine.h"
#include "dev/leds.h"
#include "er-coap.h"

/*
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "lib/random.h"
#include "sys/ctimer.h"
#include "sys/etimer.h"
#include "dev/adc-sensor.h"
#include "lib/sensors.h"
#include "ti-lib.h"
#include "lpm.h"
#include "sys/etimer.h"
#include "dev/watchdog.h"
#include "random.h"
#include "button-sensor.h"
#include "board-peripherals.h"
#include "dev/button-sensor.h"
*/

#define PRINTF(...) printf(__VA_ARGS__)

static uint8_t buf_len;
static char buf[64];

static void res_post_put_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);

RESOURCE(res_extleds,
         "title=\"Control External Leds\";rt=\"Control\"",
         NULL,
         res_post_put_handler,
         res_post_put_handler,
         NULL);


static void
res_post_put_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
    //converte o payload recebido por PUT em um pacote CoAP
    coap_packet_t *const coap_req = (coap_packet_t *)request;
    uint8_t buffer_ptr = 0;
    //verifica se o payload enviado nao eh muito grande para a requisicao
    if(coap_req->payload_len > REST_MAX_CHUNK_SIZE)
    {
        //caso for muito grande, simplesmente configura a resposta como BAD_REQUEST e retorna
        REST.set_response_status(response, REST.status.BAD_REQUEST);
        return;
    }
    else
    {
        //caso contrario, copia a mensagem enviada para o buffer criado
        memcpy((void*)buf, (void*)coap_req->payload, coap_req->payload_len);
        //salva tambem o tamanho da mensagem recebida (para uso futuro)
        buf_len = coap_req->payload_len;

        uint32_t number;
        number = atoi(buf);
        PRINTF("******* VALOR %d *******\n",number);

        if(number == 0){
            printf("DESLIGANDO LEDS\n");
            GPIO_clearDio(IOID_26);
        }else if(number == 1){
            printf("LIGANDO LEDS\n");
            GPIO_setDio(IOID_26);
        }else {
            PRINTF("Comando Invalido!\n");
        }
    }
}
