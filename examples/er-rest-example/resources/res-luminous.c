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
#include "er-coap.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "dev/adc-sensor.h"
#include "lib/sensors.h"
#include "ti-lib.h"

#define PRINTF(...) printf(__VA_ARGS__)

static uint8_t buf_len;
static char buf[64];

static void res_get_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size,int32_t *offset);

RESOURCE(res_luminous,
         "title=\"Sensor de Luminosidade\";rt=\"Control\"",
         res_get_handler,
         NULL,
         NULL,
         NULL);

static void
res_get_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size,int32_t *offset)
{
    uint32_t i;
    uint8_t etag=0;
    //configura o tipo de conteudo da mensagem
    REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
    //etag eh uma propriedade que eh utilizada pelos servidores de cache para saber se uma mensagem mudou
    //duas mensagens com o mesmo valor devem ter o mesmo etag
    for(i=0;i<buf_len;i++)
    {
        //neste caso utilizamos um checksum simples como etag, mas o usuario pode usar o que quiser
        etag += buf[i];
    }

    static int valor = 0;
    static struct sensors_sensor *sensor;
    sensor = sensors_find(ADC_SENSOR);

    SENSORS_ACTIVATE(*sensor);
    sensor->configure(ADC_SENSOR_SET_CHANNEL,ADC_COMPB_IN_AUXIO7);
    valor = (int) sensor->value(ADC_SENSOR_VALUE) / 1000;
    printf("Valor: %d\n", valor);
    SENSORS_DEACTIVATE(*sensor);

    sprintf(buf, "Valor do sensor: %d", valor);
    buf_len = strlen(buf);

    REST.set_header_etag(response, (uint8_t *)&etag, 1);
    //configura o payload a ser retornado
    REST.set_response_payload(response, buf, buf_len);
}
