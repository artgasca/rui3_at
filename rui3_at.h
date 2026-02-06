// rui3_at.h - Driver genérico AT para módulos RUI3 (RAKwireless)
// Pensado para CCS C Compiler

#ifndef __RUI3_AT_H__
#define __RUI3_AT_H__

// -----------------------------------------------------------------------------
// Configuración
// -----------------------------------------------------------------------------
#define RUI3_INT_UART1         7777
#define RUI3_INT_UART2        6666
#define RUI3_INT_UART3        5555
#define RUI3_INT_UART4        4444
#define RUI3_INT_UART5        3333
// Stream de CCS para el UART conectado al módulo RUI3
#ifndef RUI3_AT_STREAM
   #define RUI3_AT_STREAM   RUI3_INT_UART1
#endif
#ifndef RUI3_AT_BAUD
    #define RUI3_AT_BAUD 115200
#endif
// Tamaños de buffers
#ifndef RUI3_AT_RX_RING_SIZE
   #define RUI3_AT_RX_RING_SIZE   256
#endif

#ifndef RUI3_AT_LINE_BUF_SIZE
   #define RUI3_AT_LINE_BUF_SIZE  128
#endif

// Timeout por defecto para comandos (ms)
#ifndef RUI3_AT_CMD_TIMEOUT_MS
   #define RUI3_AT_CMD_TIMEOUT_MS  2000
#endif

// Habilitar logs por el puerto de debug (tú implementas rui3_at_debug)
#ifndef RUI3_AT_DEBUG
   #define RUI3_AT_DEBUG      1
#endif

#if RUI3_AT_DEBUG
   //void rui3_at_debug(char *fmt, ...);
#endif

// -----------------------------------------------------------------------------
// Tipos de datos
// -----------------------------------------------------------------------------

typedef enum {
   RUI3_ST_OK = 0,
   RUI3_ST_TIMEOUT,
   RUI3_ST_BUSY,
   RUI3_ST_PARAM_ERROR,
   RUI3_ST_NO_NET,
   RUI3_ST_ERROR,
   RUI3_ST_PARSE_ERROR
} rui3_status_t;

typedef enum {
   RUI3_JOIN_MODE_OTAA = 0,
   RUI3_JOIN_MODE_ABP  = 1
} rui3_join_mode_t;

// Tipo de evento asíncrono
typedef enum {
   RUI3_EVT_NONE = 0,
   RUI3_EVT_JOINED,
   RUI3_EVT_JOIN_FAILED,
   RUI3_EVT_TX_DONE,
   RUI3_EVT_RX,           // Downlink recibido
   RUI3_EVT_CONF_OK,
   RUI3_EVT_CONF_FAILED,
   RUI3_EVT_P2P_RX,
   RUI3_EVT_P2P_TX_DONE,
   RUI3_EVT_LINKCHECK
} rui3_evt_type_t;

// Estructura de evento de RX LoRaWAN
typedef struct {
   signed int16    rssi;
   signed int16    snr;
   unsigned int8   fport;
   unsigned int8   is_multicast;    // 0 = unicast, 1 = multicast
   char            payload_hex[64]; // payload en HEX (ASCII, null-terminated)
} rui3_rx_t;

// Evento genérico
typedef struct {
   rui3_evt_type_t type;
   union {
      rui3_rx_t rx;
      // Aquí puedes colgar más structs para otros tipos de eventos
   } data;
} rui3_event_t;

// -----------------------------------------------------------------------------
// API base
// -----------------------------------------------------------------------------

// Inicializa estructuras internas del driver (NO configura UART)
void rui3_at_init(void);

// Esta función se llama desde la ISR de recepción del UART
// Ejemplo en main.c:
//   #int_RDA1
//   void RDA1_isr(void) { rui3_at_uart_rx_isr(getc(RUI3_AT_STREAM)); }
void rui3_at_uart_rx_isr(unsigned int8 c);

// Procesa el ring buffer, arma líneas y genera eventos/comandos.
// Devuelve 1 si hay un evento listo.
int1 rui3_at_task(rui3_event_t *evt);

// Enviar comando AT genérico y esperar status (OK / AT_ERROR / etc.).
// cmd_no_crlf: por ejemplo "AT+DEVEUI=?"
// Si value_buf != NULL, se guarda la primera línea de dato (antes del status).
rui3_status_t rui3_at_cmd(char *cmd_no_crlf,
                          char *value_buf,
                          unsigned int8 value_buf_len,
                          unsigned int16 timeout_ms);

// -----------------------------------------------------------------------------
// API de eventos (para uso desde el main)
// -----------------------------------------------------------------------------

// No bloqueante: procesa UART y regresa 1 si encontró un evento
int1 rui3_at_get_event(rui3_event_t *evt);

// Bloqueante genérico: espera cualquier evento hasta timeout_ms (ms)
// Usa delay_ms(1) internamente, sin get_ticks().
int1 rui3_at_wait_event(rui3_event_t *evt, unsigned int16 timeout_ms);

// -----------------------------------------------------------------------------
// API LoRaWAN (RUI3 AT) – no bloqueante a nivel de eventos
// -----------------------------------------------------------------------------

rui3_status_t rui3_at_set_join_mode(rui3_join_mode_t mode);
rui3_status_t rui3_at_set_deveui(char *deveui16);
rui3_status_t rui3_at_set_appeui(char *appeui16);
rui3_status_t rui3_at_set_appkey(char *appkey32);
// -----------------------------------------------------------------------------
// LoRaWAN Regional / Channels / DataRate
// -----------------------------------------------------------------------------

rui3_status_t rui3_at_set_dr(unsigned int8 dr);

// Channel mask (solo aplica en US915/AU915/LA915/CN470)
rui3_status_t rui3_at_set_mask(char *mask4_hex);

// 8-channel mode enable/disable (solo aplica en US915/AU915/LA915/CN470)
rui3_status_t rui3_at_set_che(unsigned int8 enable);

// Single channel frequency (solo aplica en US915/AU915/CN470)
// Frecuencia en Hz, ej: 902300000
rui3_status_t rui3_at_set_chs(unsigned int32 freq_hz);

// Banda/Región (si quieres seleccionar región con AT+BAND)
rui3_status_t rui3_at_set_band(unsigned int8 band_id);
// Dispara JOIN (envía AT+JOIN y solo espera status del comando)
rui3_status_t rui3_at_join_start(void);

// Uplink en HEX: port = FPort, payload_hex = "001122..."
rui3_status_t rui3_at_send_uplink_start(unsigned int8 fport,
                                        char *payload_hex);

// Test básico de comunicación: manda "AT"
rui3_status_t rui3_at_ping(unsigned int16 timeout_ms);

#endif // __RUI3_AT_H__
