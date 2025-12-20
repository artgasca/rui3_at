/* 
 * File:   lorawan_otaa_example
 * Author: Arturo Gasca
 *
 * Created on 06 de Diciembre de 2025, 10:32 AM
 */
#define PROTOLINK_DEFAULT true
#include "v2.h"



//#include <bootloader.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdlib.h>
#define FW_VERSION "1.0.0"
// UART hacia módulo RUI3 (RAK3172, 4630, etc.)
#define RUI3_AT_BAUD    115200
#define RUI3_AT_STREAM   RUI3_INT_UART2
#define RUI3_AT_DEBUG    1
#include "../rui3_at.h"
#include "../rui3_at.c"
#include "v2.h"



#define UPLINK_TIME 20
int16 uplink_time_count = 0;


void main(void) {
    delay_ms(100);
    protolink_io_init();
    protolink_timer0_init();
    output_low(LED1);
    output_low(LED2);
    rui3_event_t evt;
    rui3_status_t st;
    

    rui3_at_init();

    // PING
    st = rui3_at_ping(1000);
    if(st != RUI3_ST_OK)
    {
       // manejar error
        protolink_debug_msg("Error\r\n");
        //while(1);
    }
    protolink_debug_msg("RAK3172 ok! \r\n");
    delay_ms(100);
    
    // Config OTAA
    rui3_at_set_join_mode(RUI3_JOIN_MODE_OTAA);
    rui3_at_set_deveui((char*)"ac1f09fffe1cf474");
    rui3_at_set_appeui((char*)"162d2599a938b08f");
    rui3_at_set_appkey((char*)"ac1f09fffe1cf474ac1f09fff9153172");
    
    
    // Disparar JOIN
    st = rui3_at_join_start();
    if(st != RUI3_ST_OK)
    {
       // falló el comando AT+JOIN
        protolink_debug_msg("Fallo comando Join\r\n");
        
    }
    

    // Esperar JOINED o JOIN_FAILED desde el main
    if(rui3_at_wait_event(&evt, 30000)) // 30s
    {
       if(evt.type == RUI3_EVT_JOINED)
       {
          // JOIN OK
           protolink_debug_msg("Join OK!\r\n");
           
       }
       else if(evt.type == RUI3_EVT_JOIN_FAILED)
       {
          // JOIN FAIL
           protolink_debug_msg("Join fail!\r\n");
       }
    }
    else
    {
       // Timeout sin eventos
    }
      while(TRUE)
    {
       // Ejemplo de uso no bloqueante
       if(rui3_at_get_event(&evt))
       {
          switch(evt.type)
          {
             case RUI3_EVT_RX:
                // procesar downlink
                 protolink_debug_msg("Process downlink\r\n");
                break;

             case RUI3_EVT_TX_DONE:
             case RUI3_EVT_CONF_OK:
                // TX OK
                 protolink_debug_msg("TX Done!\r\n");
                break;

             default:
                break;
          }
       }
       if(protolink_one_second()){
           uplink_time_count++;
           if (uplink_time_count >= UPLINK_TIME) {
               uplink_time_count = 0;
               protolink_debug_msg("TIME TO SEND!\r\n");
               st = rui3_at_send_uplink_start(1,(char*)"AABBCC");
               if(st != RUI3_ST_OK){
                   protolink_debug_msg("Algo fallo en el envio\r\n");
               }

            }

       }

       // Aquí puedes hacer tu FSM para mandar uplinks:
       // 1) rui3_at_send_uplink_start(...)
       // 2) luego esperar evento con rui3_at_wait_event o rui3_at_get_event
    }

    
}