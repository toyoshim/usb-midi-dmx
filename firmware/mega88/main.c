/* Name: main.c
 * Project: USB-MIDI-DMX
 * Author: Takashi Toyoshima
 * Creation Date: 2014-01-13
 * Copyright: (c) 2014 by Takashi Toyoshima
 * License: GNU GPL v2 (see License.txt)
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/* ------------------------------------------------------------------------- */
/* LED                                                                       */
/* ------------------------------------------------------------------------- */
static unsigned char led_error = 0;

static void led_init() {
  DDRB |= _BV(DDB0);
  DDRD |= _BV(DDD5) | _BV(DDD6) | _BV(DDD7);
}

static void led_on(unsigned char n) {
  n |= led_error;
  if (n & 1) 
    PORTD &= ~_BV(DDD5);
  if (n & 2)
    PORTD &= ~_BV(DDD6);
  if (n & 4)
    PORTD &= ~_BV(DDD7);
  if (n & 8)
    PORTB &= ~_BV(DDB0);
}

static void led_off(unsigned char n) {
  n &= ~led_error;
  if (n & 1)
    PORTD |= _BV(DDD5);
  if (n & 2)
    PORTD |= _BV(DDD6);
  if (n & 4)
    PORTD |= _BV(DDD7);
  if (n & 8)
    PORTB |= _BV(DDB0);
}

static void led_start() {
  unsigned char i;
  for (i = 0; i < 4; ++i) {
    led_on(1 << i);
    _delay_ms(250);
    led_off(1 << i);
  }
  _delay_ms(250);
  led_on(15);
  _delay_ms(250);
  led_off(15);
}

/* ------------------------------------------------------------------------- */
/* USART (EIA-485/250kbs = System Clock / 40)                                */
/* ------------------------------------------------------------------------- */
static void usart_init() {
  // Set baud rate to 250kbs (10MHz / 8 / (4 + 1))
  UBRR0H = 0;
  UBRR0L = 4;
  UCSR0A = _BV(U2X0);

  // Asynchronous, Parity Disabled, Stop Bit 2-bit, Data 8-bit, LSB First.
  UCSR0B = _BV(RXEN0) | _BV(TXEN0) | _BV(USBS0);
  UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);

  // Enable receive complete interrupt.
  UCSR0B |= _BV(RXCIE0);

  // Reset hidden PD1 pin to zero. It affects a value iff USART TX is disabled.
  DDRD |= _BV(DDD1);
  PORTD &= ~_BV(DDD1);
}

static void usart_low(unsigned int usec) {
  // To output zero, disable USART TX.
  UCSR0B &= ~_BV(TXEN0);
  unsigned int i;
  for (i = 0; i != usec; ++i)
    _delay_us(1);
  UCSR0B |= _BV(TXEN0);
}

static void usart_high(unsigned int usec) {
  // Just wait since default USART value is high.
  unsigned int i;
  for (i = 0; i != usec; ++i)
    _delay_us(1);
}

static void usart_data(unsigned char data) {
  UDR0 = data;
  _delay_us(44);
}

/* ------------------------------------------------------------------------- */
/* DMX                                                                       */
/* ------------------------------------------------------------------------- */
static unsigned char dmx_data[512] = {};  // Note: DMX ch.1 is stored to [0].
static unsigned short dmx_channel = 0;
static unsigned char midi_mode[4] = {};
static unsigned char midi_status = 0x80;
static unsigned char midi_in_data[3] = {};
static unsigned char midi_in_size = 0;
static unsigned char midi_sysex = 0;

static void dmx_init() {
  unsigned short i;
  for (i = 0; i < 512; ++i)
    dmx_data[i] = 0;
  dmx_channel = 0;
  for (i = 0; i < 4; ++i)
    midi_mode[i] = 0;
  midi_in_size = 0;
  midi_sysex = 0;
}

static unsigned char dmx_message_size(unsigned char status) {
  static const char system[8] = { 0, 2, 3, 2, 1, 1, 1, 0 };
  static const char channel[8] = { 3, 3, 3, 3, 2, 2, 3, 3 };
  if (status >= 0xf0)
    return system[status - 0xf0];
  return channel[(status >> 4) - 8];
}

static void dmx_midi() {
  unsigned char size = dmx_message_size(midi_in_data[0]);
  unsigned char status = midi_in_data[0] & 0xf0;
  unsigned char ch = midi_in_data[0] & 0x0f;
  if (midi_in_size != size)
    return;
  midi_in_size = 0;
  if (ch >= 4)
    return;
  if (status == 0xc0) { // Program change.
    midi_mode[ch] = midi_in_data[1];
    if (midi_mode[3])
      dmx_channel = 512;
    if (midi_mode[2])
      dmx_channel = 384;
    if (midi_mode[1])
      dmx_channel = 256;
    if (midi_mode[0])
      dmx_channel = 128;
    else
      dmx_channel = 0;
    return;
  }
  // Note Off / Note On / Pressure
  if ((status == 0x80 && midi_mode[ch] == 1) ||
      (status == 0xa0 && (midi_mode[ch] == 2 || midi_mode[ch] == 3)))
    dmx_data[128 * ch + midi_in_data[1]] = midi_in_data[2] * 2;
  if (status == 0x90)
    dmx_data[128 * ch + midi_in_data[1]] = midi_in_data[2] * 2 + 1;
}

static void dmx_recv(unsigned char data) {
  // Receive system reset.
  if (data == 0xff) {
    dmx_init();
    led_on(15);
    return;
  }

  // Ignore other Real-Time Messages.
  if (data >= 0xf8)
    return;

  // SysEx handling.
  if (midi_sysex) {
    // Skip SysEx data.
    if (data != 0xf7)
      return;
    midi_sysex = 0;
    return;
  }
  if (data == 0xf0) {
    midi_sysex = 1;
    return;
  }

  if (midi_in_size == 0) {
    // Running status.
    if (data < 0x80)
      midi_in_data[midi_in_size++] = midi_status;
    else
      midi_status = data;
  }

  midi_in_data[midi_in_size++] = data;
  dmx_midi();
}

/*          _______        _________________________________________ ...
 * \_______/       XXXXXXXX    \XXXXX/    \XXXXX/        \XXXXX/    X...
 * | BREAK |  MAB  |  SC  |MTBF| CD1 |MTBF| CD2 |...|MTBF|CD512|MTBP|
 *  92us~1s  8us~1s  44us  ~1s  44us  ~1s  44us      ~1s   44us ~1s
 *
 * SC: start bit(1-bit low), zero(8-bit), end bit(1-bit high)
 * CDn: start bit(1-bit low), little endian data(8-bit), end bit(2-bit high) */
static void dmx_send() {
  led_off(15);
  if (!dmx_channel)
    return;
  usart_low(176);  // BREAK (DMX512-A-2004 suggested Tx value)
  usart_high(12);  // MAB (Ujjal suggested Tx value)
  usart_data(0);   // SC
  unsigned short ch;
  for (ch = 0; ch < dmx_channel; ++ch) {
    unsigned char led = 1 << ((ch >> 7) & 0xff);
    led_on(led);
    // MTBF, CDn
    usart_high(0);
    usart_data(dmx_data[ch]);
    led_off(led);
  }
  usart_high(0);  // MTBP
}

/* ------------------------------------------------------------------------- */
/* Entries                                                                   */
/* ------------------------------------------------------------------------- */

int __attribute__((noreturn)) main(void)
{
  // Internal OSC should be set to 10MHz here.
  OSCCAL = 0x6a;

  // Initialize registers.
  DDRB = DDRC = DDRD = 0x00;  // Tri-state
  PORTB = PORTC = PORTD = 0xff;  // Pull-up

  led_init();
  dmx_init();
  usart_init();
  led_start();
  sei();

  for (;;)
    dmx_send();
}

ISR(USART_RX_vect) {
  dmx_recv(UDR0);
}
