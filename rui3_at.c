// rui3_at.c - Driver genérico AT para módulos RUI3 (RAKwireless)
// Compatible con CCS C Compiler

#include <string.h>
#include "rui3_at.h"

// UART por hardware

#if (RUI3_AT_STREAM == RUI3_INT_UART1 )
    #use rs232(baud=RUI3_AT_BAUD, UART1, stream=RUI3_UART, errors)
#elif (RUI3_AT_STREAM == RUI3_INT_UART2 )
    #use rs232(baud=RUI3_AT_BAUD, UART2, stream=RUI3_UART, errors)
#elif (RUI3_AT_STREAM == RUI3_INT_UART3 )
    #use rs232(baud=RUI3_AT_BAUD, UART3, stream=RUI3_UART, errors)
#elif (RUI3_AT_STREAM == RUI3_INT_UART4 )
    #use rs232(baud=RUI3_AT_BAUD, UART4, stream=RUI3_UART, errors)
#elif (RUI3_AT_STREAM == RUI3_INT_UART5 )
    #use rs232(baud=RUI3_AT_BAUD, UART5, stream=RUI3_UART, errors)
#endif



// ISR de RX UART
//// ISR de recepción
#if (RUI3_AT_STREAM==RUI3_INT_UART1)
#int_rda
#elif (RUI3_AT_STREAM==RUI3_INT_UART2)
#int_rda2
#elif (RUI3_AT_STREAM==RUI3_INT_UART3)
#int_rda3
#elif (RUI3_AT_STREAM==RUI3_INT_UART4)
#int_rda4
#elif (RUI3_AT_STREAM==RUI3_INT_UART5)
#int_rda5
#endif
void rui3_at_isr(void)
{
   unsigned int8 c = fgetc(RUI3_UART);
   rui3_at_uart_rx_isr(c);
}
// -----------------------------------------------------------------------------
// Estructuras internas
// -----------------------------------------------------------------------------

static volatile unsigned int8  rx_ring[RUI3_AT_RX_RING_SIZE];
static volatile unsigned int16 rx_head = 0;
static volatile unsigned int16 rx_tail = 0;

static char  line_buf[RUI3_AT_LINE_BUF_SIZE];
static unsigned int8 line_len = 0;

// Último status de comando
static volatile rui3_status_t last_cmd_status = RUI3_ST_OK;
static volatile int1 cmd_status_ready = 0;

// Última línea de valor de respuesta (ej. DEVEUI, etc.)
static char  last_value_line[RUI3_AT_LINE_BUF_SIZE];
static unsigned int8 last_value_len = 0;

// Flag de módulo ocupado por comando (simple)
static volatile int1 cmd_in_progress = 0;

// -----------------------------------------------------------------------------
// Utils internos
// -----------------------------------------------------------------------------

static void _rx_ring_push(unsigned int8 c)
{
   unsigned int16 next = (rx_head + 1) % RUI3_AT_RX_RING_SIZE;
   if(next != rx_tail)
   {
      rx_ring[rx_head] = c;
      rx_head = next;
   }
   // Si se llena, se pierde el byte (mejor que colgarse)
}

static int1 _rx_ring_pop(unsigned int8 *c)
{
   if(rx_head == rx_tail)
      return 0;
   *c = rx_ring[rx_tail];
   rx_tail = (rx_tail + 1) % RUI3_AT_RX_RING_SIZE;
   return 1;
}

static int1 _line_is_status(char *line)
{
   if(!strcmp(line, (char*)"OK")) return 1;
   if(!strcmp(line, (char*)"AT_ERROR")) return 1;
   if(!strcmp(line, (char*)"AT_PARAM_ERROR")) return 1;
   if(!strcmp(line, (char*)"AT_BUSY_ERROR")) return 1;
   if(!strcmp(line, (char*)"AT_NO_NETWORK_JOINED")) return 1;
   if(!strcmp(line, (char*)"AT_RX_ERROR")) return 1;
   return 0;
}

static rui3_status_t _map_status(char *line)
{
   if(!strcmp(line, (char*)"OK"))
      return RUI3_ST_OK;
   if(!strcmp(line, (char*)"AT_PARAM_ERROR"))
      return RUI3_ST_PARAM_ERROR;
   if(!strcmp(line, (char*)"AT_BUSY_ERROR"))
      return RUI3_ST_BUSY;
   if(!strcmp(line, (char*)"AT_NO_NETWORK_JOINED"))
      return RUI3_ST_NO_NET;
   if(!strcmp(line, (char*)"AT_ERROR") || !strcmp(line, (char*)"AT_RX_ERROR"))
      return RUI3_ST_ERROR;
   return RUI3_ST_PARSE_ERROR;
}

// Trim CR/LF
static void _trim_crlf(char *s)
{
   unsigned int8 n = strlen(s);
   while(n > 0)
   {
      char c = s[n-1];
      if((c == '\r') || (c == '\n'))
      {
         s[n-1] = '\0';
         n--;
      }
      else
         break;
   }
}

// -----------------------------------------------------------------------------
// API pública base
// -----------------------------------------------------------------------------

void rui3_at_init(void)
{
   rx_head = rx_tail = 0;
   line_len = 0;
   last_cmd_status = RUI3_ST_OK;
   cmd_status_ready = 0;
   last_value_len = 0;
   cmd_in_progress = 0;
}

// Llamar desde ISR del UART
void rui3_at_uart_rx_isr(unsigned int8 c)
{
   _rx_ring_push(c);
}

// Procesa el ring, arma líneas y genera eventos/comandos
int1 rui3_at_task(rui3_event_t *evt)
{
   unsigned int8 c;
   evt->type = RUI3_EVT_NONE;

   while(_rx_ring_pop(&c))
   {
      if(c == '\r')
         continue;

      if(c == '\n')
      {
         // fin de línea
         line_buf[line_len] = '\0';
         _trim_crlf(line_buf);

         if(line_len == 0)
         {
            line_len = 0;
            continue;
         }

#if RUI3_AT_DEBUG
        // rui3_at_debug("[RUI3] RX: %s\r\n", line_buf);
#endif

         // Status de comando
         if(_line_is_status(line_buf))
         {
            last_cmd_status = _map_status(line_buf);
            cmd_status_ready = 1;
            cmd_in_progress = 0;
         }
         // Eventos asíncronos (+EVT:...)
         else if(!strncmp(line_buf, (char*)"+EVT:", 5))
         {
            char *p = line_buf + 5;

            if(!strncmp(p, (char*)"JOINED", 6))
            {
               evt->type = RUI3_EVT_JOINED;
               line_len = 0;
               return 1;
            }
            else if(!strncmp(p, (char*)"JOIN_FAILED", 11))
            {
               evt->type = RUI3_EVT_JOIN_FAILED;
               line_len = 0;
               return 1;
            }
            else if(!strncmp(p, (char*)"TX_DONE", 7))
            {
               evt->type = RUI3_EVT_TX_DONE;
               line_len = 0;
               return 1;
            }
            else if(!strncmp(p, (char*)"SEND_CONFIRMED_OK", 17))
            {
               evt->type = RUI3_EVT_CONF_OK;
               line_len = 0;
               return 1;
            }
            else if(!strncmp(p, (char*)"SEND_CONFIRMED_FAILED", 22))
            {
               evt->type = RUI3_EVT_CONF_FAILED;
               line_len = 0;
               return 1;
            }
            else if(!strncmp(p, (char*)"RX_", 3))
            {
               // Ejemplo:
               // +EVT:RX_1:-70:8:UNICAST:1:1234
               evt->type = RUI3_EVT_RX;

               char tmp[RUI3_AT_LINE_BUF_SIZE];
               char *tok;
               char *ctx;

               strcpy(tmp, p);  // "RX_1:-70:8:UNICAST:1:1234"

               // saltar "RX_X"
               tok = strtok(tmp, (char*)":"); // "RX_1"
               tok = strtok(NULL, (char*)":"); // RSSI
               if(tok) evt->data.rx.rssi = (signed int16)atoi(tok);
               tok = strtok(NULL, (char*)":"); // SNR
               if(tok) evt->data.rx.snr = (signed int16)atoi(tok);
               tok = strtok(NULL, (char*)":"); // UNICAST/MULTICAST
               if(tok) evt->data.rx.is_multicast =
                           (!strcmp(tok, (char*)"MULTICAST")) ? 1 : 0;
               tok = strtok(NULL, (char*)":"); // FPORT
               if(tok) evt->data.rx.fport = (unsigned int8)atoi(tok);
               tok = strtok(NULL, (char*)":"); // PAYLOAD HEX
               if(tok)
               {
                  strncpy(evt->data.rx.payload_hex, tok,
                          sizeof(evt->data.rx.payload_hex)-1);
                  evt->data.rx.payload_hex[
                     sizeof(evt->data.rx.payload_hex)-1] = '\0';
               }
               else
               {
                  evt->data.rx.payload_hex[0] = '\0';
               }

               line_len = 0;
               return 1;
            }
            else if(!strncmp(p, (char*)"TXP2P DONE", 10))
            {
               evt->type = RUI3_EVT_P2P_TX_DONE;
               line_len = 0;
               return 1;
            }
            else if(!strncmp(p,(char*)"RXP2P", 5))
            {
               evt->type = RUI3_EVT_P2P_RX;
               line_len = 0;
               return 1;
            }
            else if(!strncmp(p, (char*)"LINKCHECK", 9))
            {
               evt->type = RUI3_EVT_LINKCHECK;
               line_len = 0;
               return 1;
            }
         }
         else
         {
            // Línea de dato de respuesta (ej. DEVEUI=?)
            last_value_len = strlen(line_buf);
            if(last_value_len >= sizeof(last_value_line))
               last_value_len = sizeof(last_value_line)-1;
            memcpy(last_value_line, line_buf, last_value_len);
            last_value_line[last_value_len] = '\0';
         }

         line_len = 0;
      }
      else
      {
         if(line_len < (RUI3_AT_LINE_BUF_SIZE-1))
         {
            line_buf[line_len++] = c;
         }
      }
   }

   return 0; // No hay evento pendiente
}

// Enviar string como comando AT, agregando \r\n
static void _rui3_at_send_line(char *s)
{
#if RUI3_AT_DEBUG
  // rui3_at_debug("[RUI3] TX: %s\r\n", s);
#endif

   char *p = s;
   while(*p)
   {
      fputc(*p++, RUI3_UART);
   }
   fputc('\r', RUI3_UART);
   fputc('\n', RUI3_UART);
}

// Comando genérico bloqueante (solo a nivel de status AT)
// Usa delay_ms(1) internamente, sin get_ticks.
rui3_status_t rui3_at_cmd(char *cmd_no_crlf,
                          char *value_buf,
                          unsigned int8 value_buf_len,
                          unsigned int16 timeout_ms)
{
   rui3_event_t evt;

   cmd_status_ready = 0;
   last_value_len   = 0;
   cmd_in_progress  = 1;

   _rui3_at_send_line(cmd_no_crlf);

   while(timeout_ms > 0)
   {
      rui3_at_task(&evt); // procesa lo que haya en el buffer

      if(cmd_status_ready)
      {
         cmd_status_ready = 0;
         if(value_buf && value_buf_len)
         {
            unsigned int8 n = last_value_len;
            if(n >= value_buf_len)
               n = value_buf_len - 1;
            memcpy(value_buf, last_value_line, n);
            value_buf[n] = '\0';
         }
         return last_cmd_status;
      }

      delay_ms(1);
      timeout_ms--;
   }

   cmd_in_progress = 0;
   return RUI3_ST_TIMEOUT;
}

// -----------------------------------------------------------------------------
// API de eventos
// -----------------------------------------------------------------------------

int1 rui3_at_get_event(rui3_event_t *evt)
{
   return rui3_at_task(evt);
}

// Espera bloqueante a cualquier evento, basado en delay_ms(1)
int1 rui3_at_wait_event(rui3_event_t *evt, unsigned int16 timeout_ms)
{
   while(timeout_ms > 0)
   {
      if(rui3_at_task(evt))
      {
         return 1; // evento encontrado
      }

      delay_ms(1);
      timeout_ms--;
   }

   return 0; // timeout
}

// -----------------------------------------------------------------------------
// API LoRaWAN (RUI3 AT)
// -----------------------------------------------------------------------------

rui3_status_t rui3_at_set_join_mode(rui3_join_mode_t mode)
{
   char cmd[16];
   // Según RUI3: AT+NJM=1 (OTAA), =0 (ABP)
   sprintf(cmd,"AT+NJM=%u", (mode == RUI3_JOIN_MODE_OTAA) ? 1 : 0);
   return rui3_at_cmd(cmd, NULL, 0, RUI3_AT_CMD_TIMEOUT_MS);
}

rui3_status_t rui3_at_set_deveui(char *deveui16)
{
   char cmd[40];
   sprintf(cmd,"AT+DEVEUI=%s", deveui16);
   return rui3_at_cmd(cmd, NULL, 0, RUI3_AT_CMD_TIMEOUT_MS);
}

rui3_status_t rui3_at_set_appeui(char *appeui16)
{
   char cmd[40];
   sprintf(cmd, "AT+APPEUI=%s", appeui16);
   return rui3_at_cmd(cmd, NULL, 0, RUI3_AT_CMD_TIMEOUT_MS);
}

rui3_status_t rui3_at_set_appkey(char *appkey32)
{
   char cmd[64];
   sprintf(cmd,"AT+APPKEY=%s", appkey32);
   return rui3_at_cmd(cmd, NULL, 0, RUI3_AT_CMD_TIMEOUT_MS);
}

// Dispara JOIN (no espera +EVT:JOINED)
rui3_status_t rui3_at_join_start(void)
{
   return rui3_at_cmd((char*)"AT+JOIN", NULL, 0, RUI3_AT_CMD_TIMEOUT_MS);
}

// Dispara uplink (no espera TX_DONE)
rui3_status_t rui3_at_send_uplink_start(unsigned int8 fport,
                                        char *payload_hex)
{
   char cmd[96];
   sprintf(cmd,"AT+SEND=%u:%s", fport, payload_hex);
   return rui3_at_cmd(cmd, NULL, 0, RUI3_AT_CMD_TIMEOUT_MS);
}

rui3_status_t rui3_at_ping(unsigned int16 timeout_ms)
{
   return rui3_at_cmd((char*)"AT", NULL, 0, timeout_ms);
}
