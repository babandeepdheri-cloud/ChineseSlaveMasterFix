// Master device to read Chinese Slave flow meter via Modbus RTU
// Controller: MS51PC0AE NUVOTON
// Display: Fixed Width (00000)
// FIX: Remapped UART0 to free P3.0 for fnd13

#include "MS51_32K.h"

/* =========================
   RS485 Direction Control
   ========================= */
sbit RS485_DE = P3^7;   // HIGH=TX, LOW=RX

/* =========================
   7-Segment / IO mapping
   ========================= */
sbit fnd1  = P1^4;
sbit fnd2  = P1^3;
sbit fnd3  = P2^4;
sbit fnd4  = P2^3;
sbit fnd5  = P2^1;
sbit fnd6  = P3^5;
sbit fnd7  = P3^1;
sbit fnd8  = P3^2;
sbit fnd9  = P1^2;
sbit fnd10 = P1^1;
sbit fnd11 = P2^2;
sbit fnd12 = P3^3;
sbit fnd13 = P3^0; // SHARED WITH UART0 RXD (P3.0) - Conflict!

sbit A_Segment = P0^0;
sbit B_Segment = P0^1;
sbit C_Segment = P0^2;
sbit D_Segment = P0^3;
sbit E_Segment = P0^4;
sbit F_Segment = P0^5;
sbit G_Segment = P1^5;
sbit H_Segment = P3^6;

/* Switches */
sbit sw_set   = P2^0;
sbit sw_reset = P3^4;

/* Outputs */
sbit relay  = P1^0;
sbit signal = P1^7;

#define TOTAL_DIGITS        13
#define MAX_SLAVES          5
#define MIN_SLAVE_ID        1
#define MAX_SLAVE_ID        5
#define POLL_CYCLE_MS       1000UL  // Base polling cycle (1 second)
#define TIME_SLOT_MS        200UL   // Time slot per slave (200ms × 5 = 1000ms cycle)
#define DISCOVERY_INTERVAL_MS  5000UL  // Discovery every 5 seconds (less aggressive to avoid conflicts)
#define DISPLAY_TOGGLE_INTERVAL_MS  5000UL  // Toggle display every 5 seconds

/* =========================
   1ms tick (Timer2)
   ========================= */
xdata volatile unsigned long ms_ticks = 0;
xdata unsigned int soft_delay_counter = 0;

/* =========================
   Values to display
   ========================= */
xdata volatile unsigned long disp_total_u = 0; 
xdata volatile unsigned long disp_fr_u    = 0; 
xdata volatile unsigned int  disp_id_u    = 0;  // Initialize to 0 (no slave discovered)

/* Pre-calculated digit arrays for ISR speed optimization */
data unsigned char disp_total_digits[5] = {0,0,0,0,0};  // [10000s, 1000s, 100s, 10s, 1s]
data unsigned char disp_fr_digits[5] = {0,0,0,0,0};     // [10000s, 1000s, 100s, 10s, 1s]
data unsigned char disp_id_digits[3] = {0,0,0};         // [100s, 10s, 1s] - Initialize to 000

/* =========================
   Multi-Slave Management
   ========================= */
// Special character codes for display
#define DASH_CHAR 10
#define CHAR_S 11  // Letter S
#define CHAR_c 12  // Letter c (lowercase)
#define CHAR_A 13  // Letter A
#define CHAR_n 14  // Letter n
#define CHAR_d 15  // Letter d
#define CHAR_E 16  // Letter E
#define CHAR_v 17  // Letter v
#define CHAR_i 18  // Letter i
#define CHAR_SPACE 19  // Space (blank)

typedef struct {
  unsigned char slave_id;               // Actual slave ID (1-255, 0=empty slot)
  unsigned char online;                 // Slave is online (0=offline, 1=online)
  unsigned char discovered;             // Slave has been discovered at least once (0=no, 1=yes)
  unsigned char consecutive_failures;   // Consecutive poll failures for this slave
  float total_flow;                     // Last known total flow value (with decimal precision)
  float flow_rate;                      // Last known flow rate value (with decimal precision)
  unsigned long last_poll_ms;           // Last time this slave was polled
} SlaveInfo;

xdata SlaveInfo slaves[MAX_SLAVES];     // Slave info table (up to 5 slaves with any IDs)
data unsigned char active_slave_count = 0;  // Number of discovered online slaves
data unsigned char current_poll_index = 0;  // Current slave slot being polled (0-4)
data unsigned char current_display_id = 0;  // Currently displayed slave ID (0 = none)
data unsigned char discovery_id = 1;  // Next ID to try discovering (1-255, skip 0)
xdata unsigned long last_discovery_ms = 0;
xdata unsigned long last_display_toggle_ms = 0;
bit discovery_mode = 0;                 // In discovery scan mode
volatile bit scanning_mode = 1;         // In initial scanning phase (show "Scan", "devices", "id")
data unsigned char scanning_cycles = 0;  // Count discovery cycles during scanning mode
bit scanning_mode_exited = 0;           // Flag to track if we've ever exited scanning mode (prevents re-entry)
data unsigned char current_request_slave_id = 0;  // ID of slave for current request

/* =========================
   RS485/Modbus Variables
   ========================= */
#define MODBUS_RX_BUF_MAX  64
#define MODBUS_TX_BUF_MAX  64
xdata volatile unsigned char modbus_rx_buf[MODBUS_RX_BUF_MAX];
xdata volatile unsigned char modbus_rx_len = 0;
xdata unsigned char modbus_tx_buf[MODBUS_TX_BUF_MAX];
volatile bit modbus_frame_ready = 0;
xdata volatile unsigned long last_rx_ms = 0;
volatile bit waiting_for_response = 0;
xdata volatile unsigned long request_sent_ms = 0;
#define MODBUS_TIMEOUT_MS  500
#define MODBUS_MAX_RETRIES 3

/* Retry and error tracking */
data unsigned char modbus_retry_count = 0;
xdata unsigned int modbus_error_count = 0; 
xdata volatile unsigned long last_request_sent_ms = 0;  // Track last request time globally 

/* Connection status and DP flash control */
volatile bit slave_disconnected = 0;  // At least one slave showing disconnect on display
volatile bit data_received_flag = 0;
xdata volatile unsigned long dp_flash_start_ms = 0;
#define DP_FLASH_DURATION_MS 500
#define MAX_CONSECUTIVE_FAILURES 5  // Show "----" after 5 failures (5 × 1s = 5 seconds with staggered polling)
#define MIN_REQUEST_INTERVAL_MS 600  // Chinese slave requires >500ms between requests

static unsigned char scan_d = 0;

/* =========================
   Forward declarations
   ========================= */
void ms_delay(unsigned int x);
void display_digit(unsigned char c);
void update_display_digits(void);
void uart0_init_9600_timer3(void);
void uart0_send_byte(unsigned char c);
void modbus_send_request(unsigned char slave_id, unsigned int start_addr, unsigned int num_regs);
void modbus_rx_task(void);
static unsigned int modbus_crc16(const unsigned char *buf, unsigned char len);
static bit modbus_parse_response(void);
void init_slave_table(void);
void update_display_for_current_slave(void);
unsigned char get_next_online_slave(unsigned char start_id);
void handle_display_toggle(void);
void handle_discovery(void);
void handle_polling(void);
char find_slave_slot_by_id(unsigned char slave_id);  // Returns slot index (0-4) or -1
char find_empty_slave_slot(void);  // Returns empty slot index (0-4) or -1

/* =========================
   TIMER0 ISR: multiplex display
   ========================= */
void Timer0_ISR(void) interrupt 1
{
  _push_(SFRS);
  CLEAR_TIMER0_INTERRUPT_FLAG;

  // 1. Clear all segments
  fnd1=fnd2=fnd3=fnd4=fnd5=fnd6=fnd7=fnd8=fnd9=fnd10=fnd11=fnd12=fnd13=0;
  A_Segment=B_Segment=C_Segment=D_Segment=E_Segment=F_Segment=G_Segment=H_Segment=1;

  // 2. Check if in scanning mode - display "Scan", "devices", "id"
  // Only show scanning display during initial startup (before scanning_mode_exited is set)
  // Additional safeguard: Never show after 15 seconds from boot
  if (scanning_mode && !scanning_mode_exited && (ms_ticks < 15000UL)) {
    switch (scan_d)
    {
      // --- LET (Total Flow) - show "Scan " ---
      case 0: display_digit(CHAR_S); fnd13=1; break;      // S
      case 1: display_digit(CHAR_c); fnd12=1; break;      // c
      case 2: display_digit(CHAR_A); fnd11=1; break;      // A
      case 3: display_digit(CHAR_n); fnd10=1; break;      // n
      case 4: display_digit(CHAR_SPACE); fnd4=1; break;   // space

      // --- KET (Flow Rate) - show "dEvic" ---
      case 5: display_digit(CHAR_d); fnd9=1; break;       // d
      case 6: display_digit(CHAR_E); fnd8=1; break;       // E
      case 7: display_digit(CHAR_v); fnd7=1; break;       // v
      case 8: display_digit(CHAR_i); fnd6=1; break;       // i
      case 9: display_digit(CHAR_c); fnd5=1; break;       // c

      // --- PET (ID) - show " id" ---
      case 10: display_digit(CHAR_SPACE); fnd3=1; break;  // space
      case 11: display_digit(CHAR_i); fnd2=1; break;      // i
      case 12: display_digit(CHAR_d); fnd1=1; break;      // d
    }
  }
  // 3. Check if slave is disconnected - display dashes
  else if (slave_disconnected) {
    switch (scan_d)
    {
      // --- LET (Total Flow) - show dashes ---
      case 0: display_digit(DASH_CHAR); fnd13=1; break;
      case 1: display_digit(DASH_CHAR); fnd12=1; break;
      case 2: display_digit(DASH_CHAR); fnd11=1; break;
      case 3: display_digit(DASH_CHAR); fnd10=1; break;
      case 4: display_digit(DASH_CHAR); fnd4=1; break;

      // --- KET (Flow Rate) - show dashes ---
      case 5: display_digit(DASH_CHAR); fnd9=1; break;
      case 6: display_digit(DASH_CHAR); fnd8=1; break;
      case 7: display_digit(DASH_CHAR); fnd7=1; break;
      case 8: display_digit(DASH_CHAR); fnd6=1; break;
      case 9: display_digit(DASH_CHAR); fnd5=1; break;

      // --- PET (ID) - still show ID ---
      case 10: display_digit(disp_id_digits[0]); fnd3=1; break;
      case 11: display_digit(disp_id_digits[1]); fnd2=1; break;
      case 12: display_digit(disp_id_digits[2]); fnd1=1; break;
    }
  } else {
    // Normal display mode
    switch (scan_d)
    {
      // --- LET (Total Flow) ---
      case 0: // 10,000s place (fnd13 / P3.0)
        display_digit(disp_total_digits[0]); 
        fnd13=1; 
        break;
      case 1: 
        display_digit(disp_total_digits[1]); 
        fnd12=1; 
        break;
      case 2: 
        display_digit(disp_total_digits[2]); 
        fnd11=1; 
        break;
      case 3: 
        display_digit(disp_total_digits[3]); 
        fnd10=1; 
        break;
      case 4: 
        display_digit(disp_total_digits[4]); 
        fnd4=1; 
        break;

      // --- KET (Flow Rate) ---
      case 5: 
        display_digit(disp_fr_digits[0]); 
        fnd9=1; 
        break;
      case 6: 
        display_digit(disp_fr_digits[1]); 
        fnd8=1; 
        break;
      case 7: 
        display_digit(disp_fr_digits[2]); 
        fnd7=1; 
        break;
      case 8: 
        display_digit(disp_fr_digits[3]); 
        fnd6=1; 
        break;
      case 9: 
        display_digit(disp_fr_digits[4]); 
        fnd5=1; 
        break;

      // --- PET (ID) ---
      case 10: 
        display_digit(disp_id_digits[0]); 
        fnd3=1; 
        break;
      case 11: 
        display_digit(disp_id_digits[1]); 
        fnd2=1; 
        break;
      case 12: 
        display_digit(disp_id_digits[2]); 
        fnd1=1; 
        break;
    }
    
    // 3. Handle DP flashing when data is received
    // Flash H_Segment (DP) for DP_FLASH_DURATION_MS after data received
    if (data_received_flag) {
      if ((ms_ticks - dp_flash_start_ms) < DP_FLASH_DURATION_MS) {
        H_Segment=0;  // Turn on DP (decimal point)
      } else {
        data_received_flag = 0;  // Clear flag after flash duration expires
      }
    }
  }

  if (++scan_d == TOTAL_DIGITS) scan_d = 0;

  TH0 = 0xBC;
  TL0 = 0x66;

  _pop_(SFRS);
}

/* =========================
   Timer2 ISR: 1ms ticker
   ========================= */
void Timer2_ISR (void) interrupt 5
{
  _push_(SFRS);
  CLEAR_TIMER2_INTERRUPT_FLAG;
  ms_ticks++;
  if (soft_delay_counter > 0) soft_delay_counter--;
  _pop_(SFRS);
}

/* =========================
   UART0 ISR (interrupt 4)
   ========================= */
void UART0_ISR(void) interrupt 4
{
  _push_(SFRS);
  if (RI)
  {
    unsigned char c = SBUF;
    RI = 0;
    if (!modbus_frame_ready && modbus_rx_len < MODBUS_RX_BUF_MAX)
    {
       modbus_rx_buf[modbus_rx_len++] = c;
       last_rx_ms = ms_ticks;
    }
  }
  if (TI) TI = 0;
  _pop_(SFRS);
}

/* =========================
   UART0 init: 9600 baud @16MHz
   ========================= */
void uart0_init_9600_timer3(void)
{
  // 1. Enable UART0
  SCON = 0x50; 
  
  // 2. CRITICAL FIX: Swap UART0 pins to Alternative (P0.2/P1.6 or P0.6/P0.7 depending on chip)
  // This frees P3.0 (fnd13) from being the RX pin.
  // Bit 7 of AUXR1 is usually UART0FS (Pin selection)
  AUXR1 |= 0x80; 

  TI = 0; 
  RI = 0;

  PCON |= 0x80;

  // Ensure these pins are set for RS485 communication
  P06_PUSHPULL_MODE; 
  P07_INPUT_MODE;

  // 3. FORCE P3.0 to be PUSH-PULL (Display Drive)
  // Standard macros might skip this if defined as Input by default
  P30_PUSHPULL_MODE; 

  T3CON &= 0xF8; 
  T3CON |= 0x20;

  RH3 = 0xFF; 
  RL3 = 0x97;

  T3CON |= 0x08; 
  ES = 1;
}

void uart0_send_byte(unsigned char c)
{
  TI = 0; SBUF = c; while (!TI); TI = 0;
}

static unsigned int modbus_crc16(const unsigned char *buf, unsigned char len)
{
  unsigned int crc = 0xFFFF;
  unsigned char i, j;
  for (i = 0; i < len; i++) {
    crc ^= (unsigned int)buf[i];
    for (j = 0; j < 8; j++) {
      if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
      else crc = crc >> 1;
    }
  }
  return crc;
}

void modbus_send_request(unsigned char slave_id, unsigned int start_addr, unsigned int num_regs)
{
  unsigned char len = 0;
  unsigned int crc;
  unsigned char i;
  
  modbus_tx_buf[len++] = slave_id;
  modbus_tx_buf[len++] = 0x03; 
  modbus_tx_buf[len++] = (unsigned char)(start_addr >> 8);
  modbus_tx_buf[len++] = (unsigned char)(start_addr & 0xFF);
  modbus_tx_buf[len++] = (unsigned char)(num_regs >> 8);
  modbus_tx_buf[len++] = (unsigned char)(num_regs & 0xFF);
  
  crc = modbus_crc16(modbus_tx_buf, len);
  modbus_tx_buf[len++] = (unsigned char)(crc & 0xFF);
  modbus_tx_buf[len++] = (unsigned char)(crc >> 8);
  
  modbus_rx_len = 0;
  modbus_frame_ready = 0;
  
  RS485_DE = 1;
  ms_delay(1);
  for (i = 0; i < len; i++) uart0_send_byte(modbus_tx_buf[i]);
  ms_delay(3);  // Increased from 2ms to 3ms to ensure last byte fully transmitted
  RS485_DE = 0;
  
  waiting_for_response = 1;
  request_sent_ms = ms_ticks;
  last_request_sent_ms = ms_ticks;  // Track globally for Chinese slave timing requirement
}

static bit modbus_parse_response(void)
{
  unsigned int crc_recv, crc_calc;
  unsigned int high_word, low_word;
  unsigned long total_val, fr_val;
  unsigned char slave_id;
  char slot;
  
  // Expected response: 1(ID) + 1(FC) + 1(count) + 28(data) + 2(CRC) = 33 bytes
  if (modbus_rx_len != 33) return 0;
  
  slave_id = modbus_rx_buf[0];
  
  // Check if this is a valid slave ID (1-247, skip 0)
  if (slave_id == 0 || slave_id > 247) return 0;
  
  // Check if response is for the slave we requested
  if (slave_id != current_request_slave_id) return 0;
  
  if (modbus_rx_buf[1] != 0x03) return 0;
  
  // Check byte count field (should be 28 for 14 registers)
  if (modbus_rx_buf[2] != 28) return 0;
  
  crc_recv = ((unsigned int)modbus_rx_buf[32] << 8) | modbus_rx_buf[31];
  crc_calc = modbus_crc16(modbus_rx_buf, 31);
  if (crc_recv != crc_calc) return 0;
  
  high_word = ((unsigned int)modbus_rx_buf[3] << 8) | modbus_rx_buf[4];
  low_word = ((unsigned int)modbus_rx_buf[5] << 8) | modbus_rx_buf[6];
  fr_val = ((unsigned long)high_word << 16) | low_word;
  
  high_word = ((unsigned int)modbus_rx_buf[13] << 8) | modbus_rx_buf[14];
  low_word = ((unsigned int)modbus_rx_buf[15] << 8) | modbus_rx_buf[16];
  total_val = ((unsigned long)high_word << 16) | low_word;
  
  // Find or create slot for this slave
  slot = find_slave_slot_by_id(slave_id);
  
  if (slot < 0) {
    // Slave not in table yet - try to add it
    if (active_slave_count >= MAX_SLAVES) {
      return 0;  // Table full, can't add
    }
    
    slot = find_empty_slave_slot();
    if (slot < 0) {
      return 0;  // No empty slots (shouldn't happen if count is correct)
    }
    
    // Initialize new slave in this slot
    slaves[slot].slave_id = slave_id;
    slaves[slot].discovered = 1;
    slaves[slot].online = 1;
    slaves[slot].consecutive_failures = 0;
    active_slave_count++;
    
    // If this is the first slave discovered, set it as the displayed slave
    if (current_display_id == 0) {
      current_display_id = slave_id;
    }
  } else {
    // Slave already in table - update status
    slaves[slot].online = 1;
    slaves[slot].consecutive_failures = 0;
  }
  
  // Convert to float (values are sent with 3 decimal places, divide by 1000)
  slaves[slot].total_flow = (float)total_val / 1000.0;
  slaves[slot].flow_rate = (float)fr_val / 1000.0;
  slaves[slot].last_poll_ms = ms_ticks;
  
  // If this is the currently displayed slave, update display immediately
  if (current_display_id == slave_id) {
    disp_total_u = (unsigned long)slaves[slot].total_flow;  // Cast float to unsigned long for display (NO MODULO!)
    disp_fr_u = (unsigned long)slaves[slot].flow_rate;      // Cast float to unsigned long for display (NO MODULO!)
    disp_id_u = slave_id;
    update_display_digits();
    slave_disconnected = 0;
  }
  
  // Set data received flag and start DP flash timer
  data_received_flag = 1;
  dp_flash_start_ms = ms_ticks;
  
  return 1;
}

/* =========================
   Update pre-calculated digit arrays
   Optimization: Avoid division in ISR
   ========================= */
void update_display_digits(void)
{
  unsigned long temp;
  unsigned int temp_id;
  
  // Calculate Total Flow digits (5 digits: 00000-99999)
  temp = disp_total_u;
  disp_total_digits[4] = temp % 10; temp /= 10;  // ones
  disp_total_digits[3] = temp % 10; temp /= 10;  // tens
  disp_total_digits[2] = temp % 10; temp /= 10;  // hundreds
  disp_total_digits[1] = temp % 10; temp /= 10;  // thousands
  disp_total_digits[0] = temp % 10;              // ten-thousands
  
  // Calculate Flow Rate digits (5 digits: 00000-99999)
  temp = disp_fr_u;
  disp_fr_digits[4] = temp % 10; temp /= 10;  // ones
  disp_fr_digits[3] = temp % 10; temp /= 10;  // tens
  disp_fr_digits[2] = temp % 10; temp /= 10;  // hundreds
  disp_fr_digits[1] = temp % 10; temp /= 10;  // thousands
  disp_fr_digits[0] = temp % 10;              // ten-thousands
  
  // Calculate ID digits (3 digits: 000-999)
  temp_id = disp_id_u;
  disp_id_digits[2] = temp_id % 10; temp_id /= 10;  // ones
  disp_id_digits[1] = temp_id % 10; temp_id /= 10;  // tens
  disp_id_digits[0] = temp_id % 10;                 // hundreds
}

/* =========================
   Slave Management Functions
   ========================= */
void init_slave_table(void)
{
  unsigned char i;
  for (i = 0; i < MAX_SLAVES; i++) {
    slaves[i].slave_id = 0;  // 0 = empty slot
    slaves[i].online = 0;
    slaves[i].discovered = 0;
    slaves[i].consecutive_failures = 0;
    slaves[i].total_flow = 0.0;  // Initialize float to 0.0
    slaves[i].flow_rate = 0.0;   // Initialize float to 0.0
    slaves[i].last_poll_ms = 0;
  }
  active_slave_count = 0;
  current_poll_index = 0;
  current_display_id = 0;
  discovery_id = 1;  // Start from ID 1 (skip 0)
  scanning_mode = 1;  // Start in scanning mode
  scanning_cycles = 0;  // Reset cycle counter
  scanning_mode_exited = 0;  // Reset exit flag for fresh start
}

// Find slave slot by ID, returns slot index (0-4) or -1 if not found
char find_slave_slot_by_id(unsigned char slave_id)
{
  unsigned char i;
  for (i = 0; i < MAX_SLAVES; i++) {
    if (slaves[i].slave_id == slave_id) {
      return (char)i;
    }
  }
  return -1;  // Not found
}

// Find empty slave slot, returns slot index (0-4) or -1 if all full
char find_empty_slave_slot(void)
{
  unsigned char i;
  for (i = 0; i < MAX_SLAVES; i++) {
    if (slaves[i].slave_id == 0) {  // Empty slot
      return (char)i;
    }
  }
  return -1;  // All slots full
}

unsigned char get_next_online_slave(unsigned char start_id)
{
  unsigned char i, start_slot, checks;
  char slot;
  
  if (active_slave_count == 0) return 0;
  
  // Find starting slot
  if (start_id == 0) {
    start_slot = 0;
  } else {
    slot = find_slave_slot_by_id(start_id);
    if (slot >= 0) {
      start_slot = (unsigned char)(slot + 1) % MAX_SLAVES;  // Start from next slot
    } else {
      start_slot = 0;
    }
  }
  
  // Search for next discovered slave
  for (checks = 0; checks < MAX_SLAVES; checks++) {
    i = (start_slot + checks) % MAX_SLAVES;
    if (slaves[i].discovered && slaves[i].slave_id != 0) {
      return slaves[i].slave_id;  // Return the actual slave ID
    }
  }
  
  return 0;  // No discovered slaves found
}

void update_display_for_current_slave(void)
{
  char slot;
  
  if (current_display_id == 0) {
    // No valid slave to display
    return;
  }
  
  slot = find_slave_slot_by_id(current_display_id);
  if (slot < 0) {
    // Slave not found
    return;
  }
  
  if (!slaves[slot].online) {
    // Slave is offline - show dashes
    slave_disconnected = 1;
    disp_id_u = current_display_id;
    update_display_digits();
  } else {
    // Slave is online - show its data
    slave_disconnected = 0;
    disp_total_u = (unsigned long)slaves[slot].total_flow;  // Cast float to unsigned long for display (no modulo!)
    disp_fr_u = (unsigned long)slaves[slot].flow_rate;      // Cast float to unsigned long for display (no modulo!)
    disp_id_u = current_display_id;
    update_display_digits();
  }
}

void handle_display_toggle(void)
{
  unsigned char next_id;
  
  // If no slaves discovered yet, don't do anything
  if (active_slave_count == 0) {
    current_display_id = 0;
    disp_id_u = 0;
    return;
  }
  
  // If only one slave, keep showing it (don't toggle)
  if (active_slave_count == 1) {
    if (current_display_id == 0) {
      // First time - find the discovered slave
      next_id = get_next_online_slave(0);
      if (next_id != 0) {
        current_display_id = next_id;
        update_display_for_current_slave();
      }
    }
    // Don't toggle when only one slave - just keep showing it
    return;
  }
  
  // Multiple slaves - toggle display every DISPLAY_TOGGLE_INTERVAL_MS
  if ((ms_ticks - last_display_toggle_ms) < DISPLAY_TOGGLE_INTERVAL_MS) {
    return;
  }
  
  last_display_toggle_ms = ms_ticks;
  
  // Find next online slave to display
  if (current_display_id == 0) {
    // First time - find first online slave
    next_id = get_next_online_slave(0);
  } else {
    // Find next online slave after current one
    next_id = get_next_online_slave(current_display_id);
    if (next_id == 0 || next_id == current_display_id) {
      // Wrap around to first
      next_id = get_next_online_slave(0);
    }
  }
  
  if (next_id != 0) {
    current_display_id = next_id;
    update_display_for_current_slave();
  }
}

void handle_discovery(void)
{
  unsigned long discovery_interval;
  unsigned long current_cycle_position;
  unsigned long discovery_time_slot;
  
  // Always continue discovery to find new slaves or rediscover offline ones
  // Discovery now scans IDs 1-247 (skip 0, 248-255 reserved)
  
  // During scanning mode (startup), use faster discovery interval
  if (scanning_mode) {
    discovery_interval = 1000UL;  // 1 second during scanning mode for faster initial discovery
  } else {
    discovery_interval = DISCOVERY_INTERVAL_MS;  // 5 seconds - less aggressive to prioritize polling
  }
  
  if ((ms_ticks - last_discovery_ms) < discovery_interval) {
    return;  // Not time yet
  }
  
  // Check if we can send a request (respect timing)
  if (waiting_for_response) {
    return;  // Already waiting for response
  }
  
  if ((ms_ticks - last_request_sent_ms) < MIN_REQUEST_INTERVAL_MS) {
    return;  // Too soon
  }
  
  // Skip ID 0 (unconfigured device)
  if (discovery_id == 0) {
    discovery_id = 1;
  }
  
  // Use time slots to avoid collision with polling
  // Discovery happens in second half of each slave's time slot (100-199ms within slot)
  current_cycle_position = ms_ticks % POLL_CYCLE_MS;
  discovery_time_slot = ((discovery_id - 1) % 5) * TIME_SLOT_MS + 100;  // Use modulo for IDs > 5
  
  // Only send discovery if we're in the right time slot (with 50ms window)
  if (current_cycle_position >= discovery_time_slot && 
      current_cycle_position < (discovery_time_slot + 50)) {
    
    // Try to discover next slave ID
    last_discovery_ms = ms_ticks;
    discovery_mode = 1;
    current_request_slave_id = discovery_id;
    modbus_retry_count = 0;
    modbus_send_request(discovery_id, 0x0000, 14);
    
    // Move to next ID for next discovery cycle
    discovery_id++;
    if (discovery_id > 247) {  // Scan IDs 1-247 (avoid 248-255 which may be reserved)
      discovery_id = 1;  // Wrap to 1 (skip 0)
      
      // Increment cycle counter when wrapping around during scanning mode
      if (scanning_mode && !scanning_mode_exited) {
        scanning_cycles++;
        // Exit scanning mode after 2 complete cycles
        // With 247 IDs and delays, this takes significant time
        // So we'll exit after finding ANY slave or 10 seconds
        if (scanning_cycles >= 1 || active_slave_count > 0) {  // Exit after 1 cycle or when slave found
          scanning_mode = 0;
          scanning_mode_exited = 1;  // Mark that we've exited - prevents re-entry
        }
      }
    }
  }
}

void handle_polling(void)
{
  unsigned char i;
  unsigned char slave_id;
  unsigned long current_cycle_position;
  unsigned long slave_time_slot;
  
  if (waiting_for_response) {
    return;  // Already waiting
  }
  
  // During scanning mode, prioritize discovery over polling
  if (scanning_mode) {
    return;  // Let discovery happen first
  }
  
  if (active_slave_count == 0) {
    return;  // No slaves to poll
  }
  
  if ((ms_ticks - last_request_sent_ms) < MIN_REQUEST_INTERVAL_MS) {
    return;  // Too soon
  }
  
  // Calculate position within current 1-second cycle (0-999ms)
  current_cycle_position = ms_ticks % POLL_CYCLE_MS;
  
  // Check each slot to see if there's a discovered slave that needs polling
  for (i = 0; i < MAX_SLAVES; i++) {
    if (!slaves[i].discovered || slaves[i].slave_id == 0) {
      continue;  // Skip empty or undiscovered slots
    }
    
    slave_id = slaves[i].slave_id;
    
    // Calculate this slave's time slot within the cycle
    // Use modulo to map any ID to one of 5 time slots
    slave_time_slot = ((slave_id - 1) % 5) * TIME_SLOT_MS;
    
    // Check if we're in this slave's time slot (with 50ms window)
    if (current_cycle_position >= slave_time_slot && 
        current_cycle_position < (slave_time_slot + 50)) {
      
      // Check if this slave hasn't been polled recently (at least 900ms ago)
      // This prevents double-polling in the same cycle
      if ((ms_ticks - slaves[i].last_poll_ms) >= (POLL_CYCLE_MS - 100)) {
        current_poll_index = i;  // Store slot index
        current_request_slave_id = slave_id;
        discovery_mode = 0;
        modbus_retry_count = 0;
        modbus_send_request(slave_id, 0x0000, 14);
        return;  // Poll sent, exit
      }
    }
  }
}

void modbus_rx_task(void)
{
  if (modbus_rx_len > 0 && !modbus_frame_ready) {
    if ((ms_ticks - last_rx_ms) >= 5) modbus_frame_ready = 1;
  }
  
  if (!modbus_frame_ready) return;
  
  if (modbus_parse_response()) {
    // Success! Reset retry counter
    waiting_for_response = 0;
    modbus_retry_count = 0;
  } else {
    // Parse failed - will retry via timeout mechanism
    modbus_error_count++;
  }
  
  modbus_rx_len = 0;
  modbus_frame_ready = 0;
}

/* =========================
   MAIN
   ========================= */
void main(void)
{
  unsigned long last_poll_ms = 0;

  sw_set = sw_reset = 1;
  relay = 0;

  P37_PUSHPULL_MODE;
  RS485_DE = 0;

  ENABLE_GLOBAL_INTERRUPT;
  ALL_GPIO_PUSHPULL_MODE;
  
  // Explicitly ensure P3.0 is PushPull (overriding potential UART defaults)
  P30_PUSHPULL_MODE;

  fnd1=fnd2=fnd3=fnd4=fnd5=fnd6=fnd7=fnd8=fnd9=fnd10=fnd11=fnd12=fnd13=0;

  ENABLE_TIMER0_INTERRUPT;
  ENABLE_TIMER0_MODE1;
  TIMER0_FSYS;
  TH0 = 0xBC; TL0 = 0x66;
  TR0 = 1;

  ENABLE_TIMER2_INTERRUPT;
  TIMER2_Auto_Reload_Delay_Mode;
  TIMER2_DIV_4;
  RCMP2H = 0xF0; RCMP2L = 0x60;
  TR2 = 1;

  uart0_init_9600_timer3();
  ms_delay(500);
  
  // Initialize slave table and digit arrays
  init_slave_table();
  update_display_digits();

  while (1)
  {
    TA = 0xAA; TA = 0x55; WDCON |= 0x40;

    modbus_rx_task();

    if (waiting_for_response) {
      if ((ms_ticks - request_sent_ms) >= MODBUS_TIMEOUT_MS) {
        // Timeout occurred
        modbus_error_count++;
        
        if (modbus_retry_count < MODBUS_MAX_RETRIES) {
          // Check if enough time has passed since last request (Chinese slave requirement)
          if ((ms_ticks - last_request_sent_ms) >= MIN_REQUEST_INTERVAL_MS) {
            // Retry the request to same slave
            modbus_retry_count++;
            modbus_send_request(current_request_slave_id, 0x0000, 14);
          }
          // else: wait for next loop iteration to retry
        } else {
          // Max retries reached for this poll cycle
          waiting_for_response = 0;
          modbus_retry_count = 0;
          
          // Increment consecutive poll failures for this specific slave
          if (current_request_slave_id != 0) {
            char slot = find_slave_slot_by_id(current_request_slave_id);
            if (slot >= 0) {
              slaves[slot].consecutive_failures++;
              
              // Check if slave is disconnected (5 consecutive poll failures)
              if (slaves[slot].consecutive_failures >= MAX_CONSECUTIVE_FAILURES) {
                slaves[slot].online = 0;
                // DON'T remove discovered flag - keep slave in rotation showing "----"
                // This allows the slave to stay in display toggle and be rediscovered
                
                // If this was the displayed slave, show disconnected but keep displaying it
                if (current_display_id == current_request_slave_id) {
                  slave_disconnected = 1;
                  update_display_digits();
                  // DON'T switch to another slave - keep showing this one with "----"
                }
              }
            }
          }
        }
      }
    }

    // Handle display toggling between online slaves
    handle_display_toggle();
    
    // Handle discovery of new slaves
    handle_discovery();
    
    // Handle polling of known slaves
    handle_polling();

    if (sw_reset == 0) {
      ms_delay(200);
      if (sw_reset == 0) {
        // Reset all slaves data
        init_slave_table();
        disp_total_u = 0;
        disp_fr_u = 0;
        disp_id_u = 0;
        current_display_id = 0;
        slave_disconnected = 0;
        update_display_digits();
        ms_delay(200);
      }
      while (sw_reset == 0);
    }
  }
}

/* =========================
   Helpers
   ========================= */
void ms_delay(unsigned int x)
{
  soft_delay_counter = x;
  while (soft_delay_counter);
}

void display_digit(unsigned char c)
{
  switch (c)
  {
    case 0: A_Segment=0; B_Segment=0; C_Segment=0; D_Segment=0; E_Segment=0; F_Segment=0; G_Segment=1; break;
    case 1: A_Segment=1; B_Segment=0; C_Segment=0; D_Segment=1; E_Segment=1; F_Segment=1; G_Segment=1; break;
    case 2: A_Segment=0; B_Segment=0; C_Segment=1; D_Segment=0; E_Segment=0; F_Segment=1; G_Segment=0; break;
    case 3: A_Segment=0; B_Segment=0; C_Segment=0; D_Segment=0; E_Segment=1; F_Segment=1; G_Segment=0; break;
    case 4: A_Segment=1; B_Segment=0; C_Segment=0; D_Segment=1; E_Segment=1; F_Segment=0; G_Segment=0; break;
    case 5: A_Segment=0; B_Segment=1; C_Segment=0; D_Segment=0; E_Segment=1; F_Segment=0; G_Segment=0; break;
    case 6: A_Segment=0; B_Segment=1; C_Segment=0; D_Segment=0; E_Segment=0; F_Segment=0; G_Segment=0; break;
    case 7: A_Segment=0; B_Segment=0; C_Segment=0; D_Segment=1; E_Segment=1; F_Segment=1; G_Segment=1; break;
    case 8: A_Segment=0; B_Segment=0; C_Segment=0; D_Segment=0; E_Segment=0; F_Segment=0; G_Segment=0; break;
    case 9: A_Segment=0; B_Segment=0; C_Segment=0; D_Segment=0; E_Segment=1; F_Segment=0; G_Segment=0; break;
    case DASH_CHAR: // Dash character '-' (only G segment on)
      A_Segment=1; B_Segment=1; C_Segment=1; D_Segment=1; E_Segment=1; F_Segment=1; G_Segment=0; break;
    // Letter characters for scanning display
    case CHAR_S: // Letter S (segments A,C,D,F,G)
      A_Segment=0; B_Segment=1; C_Segment=0; D_Segment=0; E_Segment=1; F_Segment=0; G_Segment=0; break;
    case CHAR_c: // Letter c (segments D,E,G)
      A_Segment=1; B_Segment=1; C_Segment=1; D_Segment=0; E_Segment=0; F_Segment=1; G_Segment=0; break;
    case CHAR_A: // Letter A (segments A,B,C,E,F,G)
      A_Segment=0; B_Segment=0; C_Segment=0; D_Segment=1; E_Segment=0; F_Segment=0; G_Segment=0; break;
    case CHAR_n: // Letter n (segments C,E,G)
      A_Segment=1; B_Segment=1; C_Segment=0; D_Segment=1; E_Segment=0; F_Segment=1; G_Segment=0; break;
    case CHAR_d: // Letter d (segments B,C,D,E,G)
      A_Segment=1; B_Segment=0; C_Segment=0; D_Segment=0; E_Segment=0; F_Segment=1; G_Segment=0; break;
    case CHAR_E: // Letter E (segments A,D,E,F,G)
      A_Segment=0; B_Segment=1; C_Segment=1; D_Segment=0; E_Segment=0; F_Segment=0; G_Segment=0; break;
    case CHAR_v: // Letter v (segments C,D,E)
      A_Segment=1; B_Segment=1; C_Segment=0; D_Segment=0; E_Segment=0; F_Segment=1; G_Segment=1; break;
    case CHAR_i: // Letter i (segments E)
      A_Segment=1; B_Segment=1; C_Segment=1; D_Segment=1; E_Segment=0; F_Segment=1; G_Segment=1; break;
    case CHAR_SPACE: // Space (all segments off)
      A_Segment=1; B_Segment=1; C_Segment=1; D_Segment=1; E_Segment=1; F_Segment=1; G_Segment=1; break;
    default: A_Segment=B_Segment=C_Segment=D_Segment=E_Segment=F_Segment=G_Segment=H_Segment=1; break;
  }
}