/*
  Basic MQTT-SN client library
  Copyright (C) 2013 Nicholas Humfrey
  Permission is hereby granted, free of charge, to any person obtaining
  a copy of this software and associated documentation files (the
  "Software"), to deal in the Software without restriction, including
  without limitation the rights to use, copy, modify, merge, publish,
  distribute, sublicense, and/or sell copies of the Software, and to
  permit persons to whom the Software is furnished to do so, subject to
  the following conditions:
  The above copyright notice and this permission notice shall be
  included in all copies or substantial portions of the Software.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
  LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
  WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
  Modifications:
  Copyright (C) 2013 Adam Renner
*/

#include "ti-lib.h"
#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"

#include "sys/ctimer.h"
#include "sys/etimer.h"

#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-nameserver.h"
#include "net/ip/uip-debug.h"
#include "net/ip/resolv.h"
#include "net/rime/rime.h"

#include "dev/button-sensor.h"
#include "dev/adc-sensor.h"
#include "dev/leds.h"
#include "dev/watchdog.h"

#include "lib/sensors.h"
#include "lib/random.h"

#include "simple-udp.h"
#include "mqtt-sn.h"
#include "rpl.h"
#include "lpm.h"

#include "random.h"
#include "button-sensor.h"
#include "board-peripherals.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

//PORTA UDP PADRAO
#define UDP_PORT 1883

//DEFINE PARA DESLIGAR O ALARME | ABRIR A TRAVA
#define TURN_OFF 0

//DEFINE PARA LIGAR O ALARME | FECHAR A TRAVA
#define TURN_ON 1

#define REQUEST_RETRIES 4
#define DEFAULT_SEND_INTERVAL (10 * CLOCK_SECOND)
#define REPLY_TIMEOUT (3 * CLOCK_SECOND)

//Variavel para verificar se a instalacao foi esquecida aberta
static int counter = 0;

//Indica se o alarme esta disparado ou nao
static bool alarm_status = false;

//Indica se ja enviou a mensagem de alerta de disparo de alarme
static bool alert_send = false;

//Indica se ja enviou a mensagem de indicacao de instalação aberta
static bool open_send = false;

//Indica se ja enviou a mensagem de indicacao de instalação fechada
static bool close_send = false;

static struct mqtt_sn_connection mqtt_sn_c;
static char mqtt_client_id[17];

//Topico de controle da trava solenoide
static char lock_topic[24] = "0000000000000000/lock\0";
static uint16_t lock_topic_id;
static uint16_t lock_topic_msg_id;

//Topico de controle do status do alarme
static char alarm_topic[24] = "0000000000000000/alarm\0";
static uint16_t alarm_topic_id;
static uint16_t alarm_topic_msg_id;

//Topico de indicativo de disparo de alarme
static char alert_topic[24] = "0000000000000000/alert\0";
static uint16_t alert_topic_id;
static uint16_t alert_topic_msg_id;

//Topico de indicativo de instalacao esquecida aberta
static char leave_topic[24] = "0000000000000000/leave\0";
static uint16_t leave_topic_id;
static uint16_t leave_topic_msg_id;

//Topico de indicativo de instalacao aberta
static char open_topic[24] = "0000000000000000/open\0";
static uint16_t open_topic_id;
static uint16_t open_topic_msg_id;

static publish_packet_t incoming_packet;
static uint16_t mqtt_keep_alive=10;
static int8_t qos = 1;
static uint8_t retain = FALSE;
static char device_id[17];
static mqtt_sn_subscribe_request subreq;
static mqtt_sn_register_request regreq;

static enum mqttsn_connection_status connection_state = MQTTSN_DISCONNECTED;

/*A few events for managing device state*/
static process_event_t mqttsn_connack_event;

PROCESS(smart_lock_process, "processo main");
PROCESS(topics_process, "processo de subscribe nos topicos relacionados");
PROCESS(init_sensors_process, "processo de inicializacao de dispositivos e sensores");
AUTOSTART_PROCESSES(&smart_lock_process);

void sleep_enter(void){
    leds_on(LEDS_RED);
}

void sleep_leave(void) {
    leds_off(LEDS_RED);
}

/*---------------------------------------------------------------------------*/
static void
puback_receiver(struct mqtt_sn_connection *mqc, const uip_ipaddr_t *source_addr, const uint8_t *data, uint16_t datalen){
    printf("Puback received\n");
}
/*---------------------------------------------------------------------------*/
static void
connack_receiver(struct mqtt_sn_connection *mqc, const uip_ipaddr_t *source_addr, const uint8_t *data, uint16_t datalen){
    uint8_t connack_return_code;
    connack_return_code = *(data + 3);
    printf("Connack received\n");
    if (connack_return_code == ACCEPTED) {
        process_post(&smart_lock_process, mqttsn_connack_event, NULL);
    } else {
        printf("Connack error: %s\n", mqtt_sn_return_code_string(connack_return_code));
    }
}
/*---------------------------------------------------------------------------*/
static void
regack_receiver(struct mqtt_sn_connection *mqc, const uip_ipaddr_t *source_addr, const uint8_t *data, uint16_t datalen){
    regack_packet_t incoming_regack;
    memcpy(&incoming_regack, data, datalen);
    printf("Regack received\n");
    if (incoming_regack.message_id == alert_topic_msg_id) {
        if (incoming_regack.return_code == ACCEPTED) {
            alert_topic_id = uip_htons(incoming_regack.topic_id);
        } else {
            printf("Regack error: %s\n", mqtt_sn_return_code_string(incoming_regack.return_code));
        }
    }
	if (incoming_regack.message_id == leave_topic_msg_id) {
        if (incoming_regack.return_code == ACCEPTED) {
            leave_topic_id = uip_htons(incoming_regack.topic_id);
        } else {
            printf("Regack error: %s\n", mqtt_sn_return_code_string(incoming_regack.return_code));
        }
    }
    if (incoming_regack.message_id == open_topic_msg_id) {
        if (incoming_regack.return_code == ACCEPTED) {
            open_topic_id = uip_htons(incoming_regack.topic_id);
        } else {
            printf("Regack error: %s\n", mqtt_sn_return_code_string(incoming_regack.return_code));
        }
    }
}
/*---------------------------------------------------------------------------*/
static void
suback_receiver(struct mqtt_sn_connection *mqc, const uip_ipaddr_t *source_addr, const uint8_t *data, uint16_t datalen){
    suback_packet_t incoming_suback;
    memcpy(&incoming_suback, data, datalen);
    printf("Suback received\n");
    if (incoming_suback.message_id == lock_topic_msg_id) {
        if (incoming_suback.return_code == ACCEPTED) {
            lock_topic_id = uip_htons(incoming_suback.topic_id);
        } else {
            printf("Suback error: %s\n", mqtt_sn_return_code_string(incoming_suback.return_code));
        }
    }
    if (incoming_suback.message_id == alarm_topic_msg_id) {
        if (incoming_suback.return_code == ACCEPTED) {
            alarm_topic_id = uip_htons(incoming_suback.topic_id);
        } else {
            printf("Suback error: %s\n", mqtt_sn_return_code_string(incoming_suback.return_code));
        }
    }
}
/*---------------------------------------------------------------------------*/
static void
publish_receiver(struct mqtt_sn_connection *mqc, const uip_ipaddr_t *source_addr, const uint8_t *data, uint16_t datalen){
    int funcao;

    memcpy(&incoming_packet, data, datalen);
    incoming_packet.data[datalen-7] = 0x00;
    printf("Published message received: %s\n", incoming_packet.data);
    funcao = atoi(incoming_packet.data);

    if (uip_htons(incoming_packet.topic_id) == lock_topic_id) {
        printf("MSG - TOPICO LOCK\n");
        switch (funcao){
            case TURN_ON:{
                printf("Trava fechada!\n");
                GPIO_setDio(IOID_21);
                GPIO_setDio(IOID_27);
                GPIO_clearDio(IOID_26);
                break;
            }
            case TURN_OFF:{
                printf("Trava aberta!\n");
                GPIO_clearDio(IOID_21);
                GPIO_clearDio(IOID_27);
                GPIO_setDio(IOID_26);
                break;
            }
            default:{
                PRINTF("Comando Invalido!\n");
                break ;
            }
        }
    }else if (uip_htons(incoming_packet.topic_id) == alarm_topic_id) {
        printf("MSG - TOPICO ALARM\n");
        switch (funcao){
            case TURN_ON:{
                printf("Alarme ativado!\n");
                GPIO_setDio(IOID_28);
                alarm_status = true;
                break;
            }
            case TURN_OFF:{
                printf("Alarme desativado!\n");
                GPIO_clearDio(IOID_28);
                GPIO_clearDio(IOID_29);
                alarm_status = false;
                alert_send = false;
                break;
            }
            default:{
                PRINTF("Comando Invalido!\n");
                break ;
            }
        }
    }else {
        printf("unknown publication received\n");
    }
}
/*---------------------------------------------------------------------------*/
static void
pingreq_receiver(struct mqtt_sn_connection *mqc, const uip_ipaddr_t *source_addr, const uint8_t *data, uint16_t datalen){
    printf("PingReq received\n");
}
/*---------------------------------------------------------------------------*/
/*Add callbacks here if we make them*/
static const struct mqtt_sn_callbacks mqtt_sn_call = {
    publish_receiver,
    pingreq_receiver,
    NULL,
    connack_receiver,
    regack_receiver,
    puback_receiver,
    suback_receiver,
    NULL,
    NULL
};

/*---------------------------------------------------------------------------*/
/*the main process will create connection and register topics*/
/*---------------------------------------------------------------------------*/
static struct ctimer connection_timer;
static process_event_t connection_timeout_event;

static void connection_timer_callback(void *mqc){
    process_post(&smart_lock_process, connection_timeout_event, NULL);
}

/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void){
    int i;
    uint8_t state;

    PRINTF("Client IPv6 addresses: ");
    for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
        state = uip_ds6_if.addr_list[i].state;
        if(uip_ds6_if.addr_list[i].isused && (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
            PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
            PRINTF("\n");
        }
    }
}

/*---------------------------------------------------------------------------*/
static resolv_status_t
set_connection_address(uip_ipaddr_t *ipaddr){
    #ifndef UDP_CONNECTION_ADDR
        #if RESOLV_CONF_SUPPORTS_MDNS
            //DNS configurado no http://freedns.afraid.org/
            #define UDP_CONNECTION_ADDR       iotsmartlock.mooo.com
        #elif UIP_CONF_ROUTER
            #define UDP_CONNECTION_ADDR       2804:7f4:3b80:6eaf:1cec:604b:c573:c2f6
        #else
            #define UDP_CONNECTION_ADDR       2804:7f4:3b80:6eaf:1cec:604b:c573:c2f6
        #endif
    #endif /* !UDP_CONNECTION_ADDR */

    #define _QUOTEME(x) #x
    #define QUOTEME(x) _QUOTEME(x)

    resolv_status_t status = RESOLV_STATUS_ERROR;

    if(uiplib_ipaddrconv(QUOTEME(UDP_CONNECTION_ADDR), ipaddr) == 0) {
        uip_ipaddr_t *resolved_addr = NULL;
        status = resolv_lookup(QUOTEME(UDP_CONNECTION_ADDR),&resolved_addr);
        if(status == RESOLV_STATUS_UNCACHED || status == RESOLV_STATUS_EXPIRED) {
            PRINTF("Attempting to look up %s\n",QUOTEME(UDP_CONNECTION_ADDR));
            resolv_query(QUOTEME(UDP_CONNECTION_ADDR));
            status = RESOLV_STATUS_RESOLVING;
        } else if(status == RESOLV_STATUS_CACHED && resolved_addr != NULL) {
            PRINTF("Lookup of \"%s\" succeded!\n",QUOTEME(UDP_CONNECTION_ADDR));
        } else if(status == RESOLV_STATUS_RESOLVING) {
            PRINTF("Still looking up \"%s\"...\n",QUOTEME(UDP_CONNECTION_ADDR));
        } else {
            PRINTF("Lookup of \"%s\" failed. status = %d\n",QUOTEME(UDP_CONNECTION_ADDR),status);
        }
        if(resolved_addr)
            uip_ipaddr_copy(ipaddr, resolved_addr);
    } else {
        status = RESOLV_STATUS_CACHED;
    }

    return status;
}

/*-----------------------------------------------------------------------*/
/* processo para subscrever-se nos devidos topicos e monitorar o trafego */
PROCESS_THREAD(topics_process, ev, data){
    static uint8_t subscription_tries;
    static uint8_t registration_tries;
    static mqtt_sn_subscribe_request *sreq = &subreq;
    static mqtt_sn_register_request *rreq = &regreq;

    PROCESS_BEGIN();

    //TOPICO LOCK
    subscription_tries = 0;
    memcpy(lock_topic,device_id,16);
    printf("requesting subscription\n");
    while(subscription_tries < REQUEST_RETRIES){
        printf("subscribing... topic: %s\n", lock_topic);
        lock_topic_msg_id = mqtt_sn_subscribe_try(sreq,&mqtt_sn_c,lock_topic,0,REPLY_TIMEOUT);

        PROCESS_WAIT_EVENT_UNTIL(mqtt_sn_request_returned(sreq));
        if (mqtt_sn_request_success(sreq)) {
            subscription_tries = 4;
            printf("subscription acked\n");
        } else {
            subscription_tries++;
            if (sreq->state == MQTTSN_REQUEST_FAILED) {
                printf("Suback error: %s\n", mqtt_sn_return_code_string(sreq->return_code));
            }
        }
    }

    //TOPICO ALARM
    subscription_tries = 0;
    memcpy(alarm_topic,device_id,16);
    printf("requesting subscription\n");
    while(subscription_tries < REQUEST_RETRIES){
        printf("subscribing... topic: %s\n", alarm_topic);
        alarm_topic_msg_id = mqtt_sn_subscribe_try(sreq,&mqtt_sn_c,alarm_topic,0,REPLY_TIMEOUT);

        PROCESS_WAIT_EVENT_UNTIL(mqtt_sn_request_returned(sreq));
        if (mqtt_sn_request_success(sreq)) {
            subscription_tries = 4;
            printf("subscription acked\n");
        } else {
            subscription_tries++;
            if (sreq->state == MQTTSN_REQUEST_FAILED) {
                printf("Suback error: %s\n", mqtt_sn_return_code_string(sreq->return_code));
            }
        }
    }

    //TOPICO ALERT
    registration_tries =0;
    memcpy(alert_topic,device_id,16);
    printf("registering topic\n");
    while (registration_tries < REQUEST_RETRIES){
        alert_topic_msg_id = mqtt_sn_register_try(rreq,&mqtt_sn_c,alert_topic,REPLY_TIMEOUT);
        PROCESS_WAIT_EVENT_UNTIL(mqtt_sn_request_returned(rreq));
        if (mqtt_sn_request_success(rreq)) {
            registration_tries = 4;
            printf("registration acked\n");
        } else {
            registration_tries++;
            if (rreq->state == MQTTSN_REQUEST_FAILED) {
                printf("Regack error: %s\n", mqtt_sn_return_code_string(rreq->return_code));
            }
        }
    }

	//TOPICO LEAVE
    registration_tries =0;
    memcpy(leave_topic,device_id,16);
    printf("registering topic\n");
    while (registration_tries < REQUEST_RETRIES){
        leave_topic_msg_id = mqtt_sn_register_try(rreq,&mqtt_sn_c,leave_topic,REPLY_TIMEOUT);
        PROCESS_WAIT_EVENT_UNTIL(mqtt_sn_request_returned(rreq));
        if (mqtt_sn_request_success(rreq)) {
            registration_tries = 4;
            printf("registration acked\n");
        } else {
            registration_tries++;
            if (rreq->state == MQTTSN_REQUEST_FAILED) {
                printf("Regack error: %s\n", mqtt_sn_return_code_string(rreq->return_code));
            }
        }
    }

    //TOPICO OPEN
    registration_tries =0;
    memcpy(open_topic,device_id,16);
    printf("registering topic\n");
    while (registration_tries < REQUEST_RETRIES){
        open_topic_msg_id = mqtt_sn_register_try(rreq,&mqtt_sn_c,open_topic,REPLY_TIMEOUT);
        PROCESS_WAIT_EVENT_UNTIL(mqtt_sn_request_returned(rreq));
        if (mqtt_sn_request_success(rreq)) {
            registration_tries = 4;
            printf("registration acked\n");
        } else {
            registration_tries++;
            if (rreq->state == MQTTSN_REQUEST_FAILED) {
                printf("Regack error: %s\n", mqtt_sn_return_code_string(rreq->return_code));
            }
        }
    }

    PROCESS_END();
}

/*----------------------------------------------------------------------------*/
/* processo que inicializa os dispositivos e sensores conectados na aplicacao */
PROCESS_THREAD(init_sensors_process, ev, data)
{
    PROCESS_BEGIN();

    IOCPinTypeGpioOutput(IOID_21);
    IOCPinTypeGpioOutput(IOID_22);
    IOCPinTypeGpioOutput(IOID_26);
    IOCPinTypeGpioOutput(IOID_27);
    IOCPinTypeGpioOutput(IOID_28);
    IOCPinTypeGpioOutput(IOID_29);

    //SET DO PIN24 COMO INPUT PARA LER O ESTADO DO SENSOR MAGNÉTICO DE PORTA
    ti_lib_rom_ioc_pin_type_gpio_input(IOID_24);

    //TRAVA
    GPIO_clearDio(IOID_21);
    //BUZZER - ALARME
    GPIO_clearDio(IOID_22);
    //LED VERDE (p/ TRAVA ABERTA)
    GPIO_setDio(IOID_26);
    //LED VERMELHO (p/ TRAVA FECHADA)
    GPIO_clearDio(IOID_27);
    //LED BRANCO (p/ ALARME ATIVADO)
    GPIO_clearDio(IOID_28);
    //LED AZUL (p/ ALARME DISPARADO)
    GPIO_clearDio(IOID_29);

    PROCESS_END();
}

/*----------------------------------------------------------------------------*/
/*********************** processo main da aplicacao ***************************/
PROCESS_THREAD(smart_lock_process, ev, data)
{
    static struct etimer periodic_timer, alarmCheck;
    static uip_ipaddr_t broker_addr,google_dns;
    static uint8_t connection_retries = 0;
    static int valor = 0;
    char contiki_hostname[16];
    static uint8_t buf_len;
    static char buf[20];

    PROCESS_BEGIN();

    #if RESOLV_CONF_SUPPORTS_MDNS
        #ifdef CONTIKI_CONF_CUSTOM_HOSTNAME
            sprintf(contiki_hostname,"node%02X%02X",linkaddr_node_addr.u8[6], linkaddr_node_addr.u8[7]);
            resolv_set_hostname(contiki_hostname);
            PRINTF("Setting hostname to %s\n",contiki_hostname);
        #endif
    #endif

    mqttsn_connack_event = process_alloc_event();
    mqtt_sn_set_debug(1);
    uip_ip6addr(&google_dns, 0x2001, 0x4860, 0x4860, 0x0, 0x0, 0x0, 0x0, 0x8888);
    etimer_set(&periodic_timer, 2*CLOCK_SECOND);
    while(uip_ds6_get_global(ADDR_PREFERRED) == NULL){
        PROCESS_WAIT_EVENT();
        if(etimer_expired(&periodic_timer)){
            PRINTF("Aguardando auto-configuracao de IP\n");
            etimer_set(&periodic_timer, 2*CLOCK_SECOND);
        }
    }

    print_local_addresses();
    rpl_dag_t *dag = rpl_get_any_dag();
    if(dag) {
        uip_nameserver_update(&google_dns, UIP_NAMESERVER_INFINITE_LIFETIME);
    }

    static resolv_status_t status = RESOLV_STATUS_UNCACHED;
    while(status != RESOLV_STATUS_CACHED) {
        status = set_connection_address(&broker_addr);

        if(status == RESOLV_STATUS_RESOLVING) {
            PROCESS_WAIT_EVENT_UNTIL(ev == resolv_event_found);
        } else if(status != RESOLV_STATUS_CACHED) {
            PRINTF("Can't get connection address.\n");
            PROCESS_YIELD();
        }
    }

    mqtt_sn_create_socket(&mqtt_sn_c,UDP_PORT, &broker_addr, UDP_PORT);
    (&mqtt_sn_c)->mc = &mqtt_sn_call;

    sprintf(device_id,"%02X%02X%02X%02X%02X%02X%02X%02X",linkaddr_node_addr.u8[0],
            linkaddr_node_addr.u8[1],linkaddr_node_addr.u8[2],linkaddr_node_addr.u8[3],
            linkaddr_node_addr.u8[4],linkaddr_node_addr.u8[5],linkaddr_node_addr.u8[6],
            linkaddr_node_addr.u8[7]);

    sprintf(mqtt_client_id,"sens%02X%02X%02X%02X",linkaddr_node_addr.u8[4],linkaddr_node_addr.u8[5],linkaddr_node_addr.u8[6], linkaddr_node_addr.u8[7]);

    /*Request a connection and wait for connack*/
    printf("requesting connection \n ");
    connection_timeout_event = process_alloc_event();
    ctimer_set( &connection_timer, REPLY_TIMEOUT, connection_timer_callback, NULL);
    mqtt_sn_send_connect(&mqtt_sn_c,mqtt_client_id,mqtt_keep_alive);
    connection_state = MQTTSN_WAITING_CONNACK;
    while (connection_retries < 15){
        PROCESS_WAIT_EVENT();
        if (ev == mqttsn_connack_event) {
            printf("connection acked\n");
            ctimer_stop(&connection_timer);
            connection_state = MQTTSN_CONNECTED;
            connection_retries = 15;
        }

        if (ev == connection_timeout_event) {
            connection_state = MQTTSN_CONNECTION_FAILED;
            connection_retries++;
            printf("connection timeout\n");
            ctimer_restart(&connection_timer);
            if (connection_retries < 15) {
                mqtt_sn_send_connect(&mqtt_sn_c,mqtt_client_id,mqtt_keep_alive);
                connection_state = MQTTSN_WAITING_CONNACK;
            }
        }
    }

    ctimer_stop(&connection_timer);
    if (connection_state == MQTTSN_CONNECTED){
        process_start(&init_sensors_process,0);
        process_start(&topics_process, 0);
        etimer_set(&periodic_timer, 3*CLOCK_SECOND);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&periodic_timer));

        //Define para leitura dos sensores a cada meio segundo
        etimer_set(&alarmCheck, CLOCK_SECOND / 2);

        while(1) {
            PROCESS_WAIT_EVENT();
            if(etimer_expired(&alarmCheck)){

                //Leitura do estado do sensor magnético de curso (1 == aberto | 0 == fechado)
                valor = (int) ti_lib_gpio_read_dio(IOID_24);

                if(valor == 1){
					//Comeca a contar a quantidade de timers de 1/2 segundo que se passaram com a instalacao aberta
                    counter++;
                    //Se a qtde de timers for maior que 3600 (30 minutos = 3600 timers de meio segundo) envia um alerta no topico leave
                    if(counter > 3600){
                        sprintf(buf, "Instalacao esquecida aberta!");
                        printf("publicando: %s -> msg: %s\n", leave_topic, buf);
                        buf_len = strlen(buf);
                        mqtt_sn_send_publish(&mqtt_sn_c, leave_topic_id,MQTT_SN_TOPIC_TYPE_NORMAL,buf, buf_len,qos,retain);

                        //Zera o contador, para que o proximo alerta do tipo so seja enviado em no minimo 30 minutos
                        counter = 0;
                    }

                    //Realiza o envio da msg de instalacao aberta
                    if(!open_send){
                        sprintf(buf, "1");
                        printf("publicando: %s -> msg: %s\n", open_topic, buf);
                        buf_len = strlen(buf);
                        mqtt_sn_send_publish(&mqtt_sn_c, open_topic_id,MQTT_SN_TOPIC_TYPE_NORMAL,buf, buf_len,qos,retain);
                        open_send = true;
                    }
                    //Se alarme ativo, o dispara e envia msg de alerta
                    if(alarm_status){
                        GPIO_toggleDio(IOID_29);
                        GPIO_setDio(IOID_22);
                        if(!alert_send){
                            sprintf(buf, "Alerta! Seu dispositivo foi violado!");
                            printf("publicando: %s -> msg: %s\n", alert_topic, buf);
                            buf_len = strlen(buf);
                            mqtt_sn_send_publish(&mqtt_sn_c, alert_topic_id,MQTT_SN_TOPIC_TYPE_NORMAL,buf, buf_len,qos,retain);
                            alert_send = true;
                        }
                    //Se o alarme foi desativado enquanto a instalacao esta aberta, desliga o alarme
                    } else {
                        GPIO_clearDio(IOID_29);
                        GPIO_clearDio(IOID_22);
                    }
					close_send = false;
                } else {
					//Realiza o envio da msg de instalacao fechada
                    if(!close_send){
                        sprintf(buf, "0");
                        printf("publicando: %s -> msg: %s\n", open_topic, buf);
                        buf_len = strlen(buf);
                        mqtt_sn_send_publish(&mqtt_sn_c, open_topic_id,MQTT_SN_TOPIC_TYPE_NORMAL,buf, buf_len,qos,retain);
                        close_send = true;
                    }
					GPIO_clearDio(IOID_29);
                    GPIO_clearDio(IOID_22);
                    open_send = false;
                    alert_send = false;
					counter = 0;
                }
                etimer_reset(&alarmCheck);
            }
        }
    } else {
        printf("unable to connect\n");
    }

    PROCESS_END();
}
