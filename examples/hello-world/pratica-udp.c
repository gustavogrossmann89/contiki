/*
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

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ip/resolv.h"
#include "dev/leds.h"
#include "button-sensor.h"

#include <string.h>
#include <stdbool.h>

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

#define SEND_ECHO (0xBA)
#define SEND_INTERVAL       5 * CLOCK_SECOND
#define MAX_PAYLOAD_LEN     40
#define CONN_PORT     8802
//SET PARA DEFINIR SE VAI TENTAR CONECTAR VIA MDNS OU NAO
#define MDNS 1

#define LED_TOGGLE_REQUEST (0x79)
#define LED_SET_STATE (0x7A)
#define LED_GET_STATE (0x7B)
#define LED_STATE (0x7C)
#define OP_REQUEST (0x6E)
#define OP_RESULT (0x6F)
#define OP_MULTIPLY (0x22)
#define OP_DIVIDE (0x23)
#define OP_SUM (0x24)
#define OP_SUBTRACT (0x25)

static char buf[MAX_PAYLOAD_LEN];

static struct uip_udp_conn *client_conn;

struct mathopreq {
  uint8_t opRequest;
  int32_t op1;
  uint8_t operation;
  int32_t op2;
  float fc;
}  __attribute__((packed));

struct mathopreply {
  uint8_t opResult;
  int32_t intPart;
  uint32_t fracPart;
  float fpResult;
  uint8_t crc;
}  __attribute__((packed));

#define UIP_UDP_BUF  ((struct uip_udp_hdr *)&uip_buf[UIP_LLH_LEN + UIP_IPH_LEN])
#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client process");
AUTOSTART_PROCESSES(&resolv_process,&udp_client_process);
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
    char *dados;
    char i=0;

    if(uip_newdata()) {
        dados = uip_appdata;
        dados[uip_datalen()] = '\0';
        printf("\nResponse from the server: '%s'\n", dados);
        switch(dados[0]){
            case LED_GET_STATE:{
                printf("RECEBEU PACOTE LED_GET_STATE\n");
                uip_ipaddr_copy (&client_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
                client_conn->rport = UIP_UDP_BUF->destport;
                buf[0] = LED_STATE;
                buf[1] = leds_get();
                uip_udp_packet_send(client_conn,buf,strlen(buf));
                break;
            }
            case LED_SET_STATE:{
                printf("RECEBEU PACOTE LED_SET_STATE\n");
                leds_set(dados[1]);
                uip_ipaddr_copy (&client_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
                client_conn->rport = UIP_UDP_BUF->destport;
                buf[0] = LED_STATE;
                buf[1] = leds_get();
                uip_udp_packet_send(client_conn,buf,strlen(buf));
                break;
            }
            case SEND_ECHO:{
                uip_ipaddr_copy (&client_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
                client_conn->rport = UIP_UDP_BUF->destport;
                uip_udp_packet_send(client_conn,dados,uip_datalen());
                PRINTF("Enviando eco para [");
                PRINT6ADDR (&client_conn->ripaddr);
                PRINTF("]:%u\n",UIP_HTONS(client_conn->rport));
                break;
            }
            case OP_RESULT:{
                struct mathopreply* reply = (struct mathopreply*)uip_appdata;
                PRINTF("OP_RESULT: \n");
                printReply(reply);
                break;
            }
            default:{
                PRINTF("Comando Invalido: ");
                for(i=0;i<uip_datalen();i++){
                    PRINTF("0x%02X",dados[i]);
                }
                PRINTF("\n") ;
                break ;
            }
        }
    }
}
/*---------------------------------------------------------------------------*/
static void
timeout_handler(void)
{
    char payload = 0;
    payload = LED_TOGGLE_REQUEST;

    buf[0] = payload;
    if(uip_ds6_get_global(ADDR_PREFERRED) == NULL) {
      PRINTF("Aguardando auto-configuracao de IP\n");
      return;
    }

    PRINTF("Cliente para [");
    PRINT6ADDR(&client_conn->ripaddr);
    PRINTF("]:%u\n",UIP_HTONS(client_conn->rport));

    uip_udp_packet_send(client_conn, buf, sizeof(payload));
    //uip_udp_packet_send(client_conn, buf, strlen(buf));
}
/*---------------------------------------------------------------------------*/
static void
send_package(void)
{
    struct mathopreq req;
    req.opRequest = OP_REQUEST;
    req.op1 = (int32_t) 2;
    req.operation = OP_SUM;
    req.op2 = (int32_t) 5;
    req.fc = 1;

    uip_udp_packet_send(client_conn, (void*)&req, sizeof(struct mathopreq));
    PRINTF("Enviando OP_REQUEST para [");
    PRINT6ADDR(&client_conn->ripaddr);
    PRINTF("]:%u\n", UIP_HTONS(client_conn->rport));

    return;
}
/*---------------------------------------------------------------------------*/
void printRequest(struct mathopreq *req)
{
    int32_t intPart;
    uint32_t fracPart;
    intPart = (int32_t)req->fc;
    //fracPart = ABS_P((int32_t)((req->fc - intPart)*10000));
    PRINTF("(%d%s%d)*%d.%d",req->op1,operator(req->operation),req->op2,intPart,fracPart);
}
/*---------------------------------------------------------------------------*/
void printReply(struct mathopreply *req)
{
    int32_t intPart;
    uint32_t fracPart;
    intPart = (int32_t)req->fpResult;
    uint8_t* buffer = (uint8_t*)req;
    //fracPart = ABS_P((int32_t)((req->fpResult - intPart)*10000));
    PRINTF("%d.%d (%d.%d): \n",req->intPart,req->fracPart,intPart,fracPart);
    uint8_t crc=0;
    for(int i=0;i<sizeof(struct mathopreply)-1;i++)
    {
        crc+=buffer[i];
    }
    PRINTF("IntPart: 0x%x\n",req->intPart);
    PRINTF("FracPart: 0x%x\n",req->fracPart);
    PRINTF("FpResult:  0x%x\n",req->fpResult);
    PRINTF("CRC: 0x%x\n",req->crc);
    PRINTF("CRC calc: 0x%x, exp: 0x%x -> %s \n",crc,req->crc,crc==req->crc?"OK":"ERR");
}
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTF("Client IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      PRINTF("\n");
    }
  }
}
/*---------------------------------------------------------------------------*/
#if UIP_CONF_ROUTER
static void
set_global_address(void)
{
  uip_ipaddr_t ipaddr;

  uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);
}
#endif /* UIP_CONF_ROUTER */
/*---------------------------------------------------------------------------*/

#if MDNS

static resolv_status_t
set_connection_address(uip_ipaddr_t *ipaddr)
{
#ifndef UDP_CONNECTION_ADDR
#if RESOLV_CONF_SUPPORTS_MDNS
#define UDP_CONNECTION_ADDR       contiki-udp-server.local
#elif UIP_CONF_ROUTER
#define UDP_CONNECTION_ADDR       fd00:0:0:0:0212:7404:0004:0404
#else
#define UDP_CONNECTION_ADDR       fe80:0:0:0:6466:6666:6666:6666
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
#endif

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
  static struct etimer et;
  uip_ipaddr_t ipaddr;

  PROCESS_BEGIN();
  PRINTF("UDP client process started\n");

#if UIP_CONF_ROUTER
  set_global_address();
#endif

  print_local_addresses();

#if MDNS
  static resolv_status_t status = RESOLV_STATUS_UNCACHED;
  while(status != RESOLV_STATUS_CACHED) {
      status = set_connection_address(&ipaddr);

      if(status == RESOLV_STATUS_RESOLVING) {
          PROCESS_WAIT_EVENT_UNTIL(ev == resolv_event_found);
      } else if(status != RESOLV_STATUS_CACHED) {
          PRINTF("Can't get connection address.\n");
          PROCESS_YIELD();
      }
  }
#else
  //c_onfigures the destination IPv6 address
  uip_ip6addr(&ipaddr, 0xfd00, 0, 0, 0, 0x212, 0x4b00, 0x791, 0xb681);
#endif
  /* new connection with remote host */
  client_conn = udp_new(&ipaddr, UIP_HTONS(CONN_PORT), NULL);
  udp_bind(client_conn, UIP_HTONS(CONN_PORT));

  PRINT6ADDR(&client_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n", UIP_HTONS(client_conn->lport), UIP_HTONS(client_conn->rport));

  etimer_set(&et, SEND_INTERVAL);
  while(1) {
    PROCESS_YIELD();
    if(etimer_expired(&et)) {
      timeout_handler();
      etimer_restart(&et);
    } else if(ev == tcpip_event) {
      tcpip_handler();
    } else if(ev == sensors_event){
      send_package();
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
