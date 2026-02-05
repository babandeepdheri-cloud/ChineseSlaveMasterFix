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
#define CHINESE_SLAVE_ID    1
#define POLL_INTERVAL_MS    1000UL

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
xdata volatile unsigned int  disp_id_u    = CHINESE_SLAVE_ID;

/* Pre-calculated digit arrays for ISR speed optimization */
data unsigned char disp_total_digits[5] = {0,0,0,0,0};  // [10000s, 1000s, 100s, 10s, 1s]
data unsigned char disp_fr_digits[5] = {0,0,0,0,0};     // [10000s, 1000s, 100s, 10s, 1s]
data unsigned char disp_id_digits[3] = {0,0,1};         // [100s, 10s, 1s]

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

/* Connection status and DP flash control */
data unsigned char consecutive_poll_failures = 0;
bit slave_disconnected = 0;
bit data_received_flag = 0;
xdata volatile unsigned long dp_flash_start_ms = 0;
#define DP_FLASH_DURATION_MS 500
#define MAX_CONSECUTIVE_FAILURES 10  // Increased from 5 to 10 for more tolerance
#define DASH_CHAR 10

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

  // 2. Check if slave is disconnected - display dashes
  if (slave_disconnected) {
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
    if (data_received_flag && ((ms_ticks - dp_flash_start_ms) < DP_FLASH_DURATION_MS)) {
      H_Segment=0;  // Turn on DP (decimal point)
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
}

static bit modbus_parse_response(void)
{
  unsigned int crc_recv, crc_calc;
  unsigned int high_word, low_word;
  unsigned long total_val, fr_val;
  
  // Expected response: 1(ID) + 1(FC) + 1(count) + 28(data) + 2(CRC) = 33 bytes
  if (modbus_rx_len != 33) return 0;
  
  if (modbus_rx_buf[0] != CHINESE_SLAVE_ID || modbus_rx_buf[1] != 0x03) return 0;
  
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
  
  disp_total_u = total_val % 100000UL;
  if (fr_val > 99999UL) fr_val = 99999UL;
  disp_fr_u = fr_val;
  
  // Update pre-calculated digit arrays for ISR
  update_display_digits();
  
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

void modbus_rx_task(void)
{
  if (modbus_rx_len > 0 && !modbus_frame_ready) {
    if ((ms_ticks - last_rx_ms) >= 5) modbus_frame_ready = 1;
  }
  
  if (!modbus_frame_ready) return;
  
  if (modbus_parse_response()) {
    // Success! Reset retry counter and consecutive failures
    waiting_for_response = 0;
    modbus_retry_count = 0;
    consecutive_poll_failures = 0;
    slave_disconnected = 0;  // Mark as connected
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
  
  // Initialize digit arrays
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
          // Retry the request
          modbus_retry_count++;
          modbus_send_request(CHINESE_SLAVE_ID, 0x0000, 14);
        } else {
          // Max retries reached for this poll cycle
          waiting_for_response = 0;
          modbus_retry_count = 0;
          
          // Increment consecutive poll failures
          consecutive_poll_failures++;
          
          // Check if slave is disconnected (5 consecutive poll failures)
          if (consecutive_poll_failures >= MAX_CONSECUTIVE_FAILURES) {
            slave_disconnected = 1;
          }
        }
      }
    }

    if (!waiting_for_response) {
      if ((ms_ticks - last_poll_ms) >= POLL_INTERVAL_MS) {
        last_poll_ms = ms_ticks;
        modbus_retry_count = 0;  // Reset retry counter for new request
        modbus_send_request(CHINESE_SLAVE_ID, 0x0000, 14);
      }
    }

    if (sw_reset == 0) {
      ms_delay(200);
      if (sw_reset == 0) {
        disp_total_u = 0;
        disp_fr_u = 0;
        update_display_digits();  // Update digit arrays after reset
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
    default: A_Segment=B_Segment=C_Segment=D_Segment=E_Segment=F_Segment=G_Segment=H_Segment=1; break;
  }
}