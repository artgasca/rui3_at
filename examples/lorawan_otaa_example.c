/* 
 * File:   lorawan_otaa_example
 * Author: Arturo Gasca
 *
 * Created on 06 de Diciembre de 2025, 10:32 AM
 */

#include "v1.h"



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


/* Protolink Default functions
 * Active define for use in main
 * Interrupts use for counters INT0, INT1
 * SMT1 and SMT use for counters
 * Timer0 config for 1 second, consume in protolink_one_second()
 */
// Numero de interrupciones de TIMER0 para ~1 segundo
// 0.013s * 77 ? 1001 ms
int16 NInts = 77;

// VARIABLES GLOBALES
int16 C_Ints = 0;   // Contador de interrupciones ocurridas
int1 Flag = 0;     // Flag que cambia cada ~1 segundo

// Ejemplo de uso: hacer algo cada vez que cambie Flag (~1s)
int1 lastFlag = 0;



#define UPLINK_TIME 20
int16 uplink_time_count = 0;
//
//// ISR de RX UART
//#INT_RDA2
//void RDA2_isr(void)
//{
//   unsigned int8 c = fgetc(RUI3_UART);
//   rui3_at_uart_rx_isr(c);
//}


#INT_TIMER0
void TIMER0_isr(void)
{
    C_Ints++;
    if(C_Ints >= NInts){
        C_Ints = 0;
        Flag = !Flag; //Toggle cada 1s
    }
    clear_interrupt(INT_TIMER0);
}

int1 protolink_one_second(void){
    if(Flag != lastFlag){
        lastFlag = flag;
        return true;
    }
    return false;
}
void main(void) {
    delay_ms(100);
    protolink_io_init();
    output_low(LED1);
    output_low(LED2);
    rui3_event_t evt;
    rui3_status_t st;
    setup_timer_0(T0_INTERNAL | T0_DIV_256 | T0_8_BIT);
    enable_interrupts(INT_TIMER0);
    enable_interrupts(INT_RDA2);
    enable_interrupts(GLOBAL);

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