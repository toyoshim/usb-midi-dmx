/* Name: main.c
 * Project: USB-MIDI-DMX
 * Author: Takashi Toyoshima
 * Creation Date: 2014-01-13
 * Copyright: (c) 2014 by Takashi Toyoshima
 * License: GNU GPL v2 (see License.txt)
 */

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <avr/pgmspace.h>
#include "usbdrv.h"

/* ------------------------------------------------------------------------- */
/* ---------------------------- USB Descriptors ---------------------------- */
/* ------------------------------------------------------------------------- */
enum {
  STRING_ZERO = 0,
  STRING_MANUFACTURER,
  STRING_PRODUCT,
  STRING_SERIAL_NUMBER
};

static PROGMEM char const device[] = {
  0x12,                       // bLength
  USBDESCR_DEVICE,            // bDescriptor
  0x10, 0x01,                 // bcdUSB
  0x00,                       // bDeviceClass
  0x00,                       // bDeviceSubClass
  0x00,                       // bDeviceProtocol
  0x08,                       // bMaxPacketSize0
  USB_CFG_VENDOR_ID,          // idVendor
  USB_CFG_DEVICE_ID,          // idProduct
  USB_CFG_DEVICE_VERSION,     // bcdDevice
  STRING_MANUFACTURER,        // iManufacturer
  STRING_PRODUCT,             // iProduct
  STRING_SERIAL_NUMBER,       // iSerialNumber
  1                           // bNumConfigurations
};

static PROGMEM char const config[] = {
  0x09,                       // bLength
  USBDESCR_CONFIG,            // bDescriptor
  // 0x48 = 0x09 + 0x09 + 0x09 + 0x09 + 0x24
  0x48, 0x00,                 // wTotalLength
  0x02,                       // bNumInterfaces
  0x01,                       // bConfigurationValue
  STRING_PRODUCT,             // iConfiguration
  USBATTR_BUSPOWER,           // bmAttributes
  USB_CFG_MAX_BUS_POWER / 2,  // bmMaxPower

  // Standard Audio Control Interface descriptor
  0x09,                       // bLength
  USBDESCR_INTERFACE,         // bDescriptor
  0x00,                       // bInterfaceNumber
  0x00,                       // bAlternateSetting
  0x00,                       // bNumEndpoints
  0x01,                       // bInterfaceClass (AUDIO)
  0x01,                       // bInterfaceSubClass (AUDIOCONTROL)
  0x00,                       // bInterfaceProtocol
  STRING_ZERO,                // iInterface

  // Class-specific descriptor
  0x09,                       // bLength
  0x24,                       // bDescriptorType (CS_INTERFACE)
  0x01,                       // bDescriptorSubtype (HEADER)
  0x00, 0x01,                 // bcdADC
  0x09, 0x00,                 // wTotalLength (inc. Unit and Terminal desc.)
  0x01,                       // bInCollection
  0x01,                       // baInterfaceNr(1)

  // Standard MIDIStreaming Interface descriptor
  0x09,                       // bLength
  USBDESCR_INTERFACE,         // bDescriptor
  0x01,                       // bInterfaceNumber
  0x00,                       // bAlternateSetting
  0x02,                       // bNumEndpoints
  0x01,                       // bInterfaceClass (AUDIO)
  0x03,                       // bInterfaceSubClass (MIDISTREAMING)
  0x00,                       // bInterfaceProtocol
  STRING_ZERO,                // iInterface

  // Class-specific descriptor
  0x07,                       // bLength
  0x24,                       // bDescriptorType (CS_INTERFACE)
  0x01,                       // bDescriptorSubtype (MS_HEADER)
  0x00, 0x01,                 // bcdMSC
  // 0x24 = 0x07 + 0x06 + 0x09 + 0x09 + 0x05
  0x24, 0x00,                 // wTotalLength (inc. Jack, Element, and EP.)
  // MIDI IN Jack descriptor (EMBEDDED)
  0x06,                       // bLength
  0x24,                       // bDescriptorType (CS_INTERFACE)
  0x02,                       // bDescriptorSubtype (MIDI_IN_JACK)
  0x01,                       // bJackType (1: EMBEDDED / 2: EXTERNAL)
  0x01,                       // bJackID
  STRING_ZERO,                // iJack
  // MIDI OUT Jack descriptor (EXTERNAL)
  0x09,                       // bLength
  0x24,                       // bDescriptorType (CS_INTERFACE)
  0x03,                       // bDescriptorSubtype (MIDI_OUT_JACK)
  0x02,                       // bJackType (1: EMBEDDED / 2: EXTERNAL)
  0x02,                       // bJackID
  0x01,                       // bNrInputPins
  0x01,                       // baSourceID(1)
  0x01,                       // baSourcePin(1)
  STRING_ZERO,                // iJack
  // Endpoint descriptor (Interrupt Out)
  // Note: This is a workaround to run in low speed mode. Original spec
  // requires Bulk endpoints.
  0x09,                       // bLength
  USBDESCR_ENDPOINT,          // bDescriptorType (ENDPOINT)
  0x01,                       // bEndpointAddress (D7   ; 0 OUT / 1 IN,
                              //                   D6..4; Reserved,
                              //                   D3..0; EP number)
  0x03,                       // bmAttributes     (D7..4: Reserved,
                              //                   D3..2: Sync. type
                              //                   D1..0: Transfer type)
                              // (2: bulk, 3: interrupt)
  0x08, 0x00,                 // wMaxPacketSize
  0x0a,                       // bInterval (in msec)
  0x00,                       // bRefresh
  0x00,                       // bSynchAddress
  // Class specific descriptor
  0x05,                       // bLength
  0x25,                       // bDescriptorType (CS_ENDPOINT)
  0x01,                       // bDescriptorSubtype (MS_GENERAL)
  0x01,                       // bNumberEmbMIDIJack
  0x01                        // baAssocJackID(1)
};

uchar usbFunctionDescriptor(usbRequest_t* rq)
{
  switch (rq->wValue.bytes[1]) {
  case USBDESCR_DEVICE:
    usbMsgPtr = (uchar*)device;
    return sizeof(device);
  case USBDESCR_CONFIG:
    usbMsgPtr = (uchar*)config;
    return sizeof(config);
  }
  return 0;
}

/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
  return 0;
}

static volatile uchar midiData[16];
static volatile uchar midiRdPtr = 0;
static volatile uchar midiWrPtr = 0;

static void midiInit()
{
  // Set baud rate to 250kbps (20MHz / 16 / (4 + 1))
  UBRRH = 0;
  UBRRL = 4;
  UCSRA = 0;

  // Asynchronous, Parity Disabled, Stop Bit 2-bit, Data 8-bit, LSB First.
  UCSRB = _BV(TXEN);
  UCSRC = _BV(USBS) | _BV(UCSZ1) | _BV(UCSZ0);

  // Enable USART TX Complete interrupt.
  UCSRB |= _BV(TXCIE);
}

static void midiSendNext()
{
  if (midiRdPtr == midiWrPtr)
    return;
  UDR = midiData[midiRdPtr];
  midiRdPtr = (midiRdPtr + 1) & 0x0f;  // Atomic.
}

static void midiSend(uchar* data, uchar len)
{
  uchar i;
  for (i = 0; i < len; ++i) {
    while (((midiWrPtr + 1) & 0x0f) == midiRdPtr);
    midiData[midiWrPtr] = data[i];
    midiWrPtr = (midiWrPtr + 1) & 0x0f; // Atomic.
    if (UCSRA & _BV(UDRE))
      midiSendNext();
  }
}

void usbFunctionWriteOut(uchar* data, uchar len)
{
  static const uchar const size[16] = {
    0, 0, 2, 3, 3, 1, 2, 3, 3, 3, 3, 3, 2, 2, 3, 1
  };
  uchar i;
  for (i = 0; (i + 4) <= len; i += 4) {
    uchar packet_len = size[data[i] & 0x0f];
    midiSend(&data[i + 1], packet_len);
  }
}

/* ------------------------------------------------------------------------- */

int __attribute__((noreturn)) main(void)
{
  unsigned int i;
  usbInit();
  usbDeviceDisconnect();
  midiInit();
  for (i = 0; i < 30; i++)
    _delay_ms(10.0);
  usbDeviceConnect();
  sei();

  DDRB = _BV(DDB0);
  PORTB |= _BV(DDB0);

  for (;;)
    usbPoll();
}

ISR(USART_TX_vect)
{
  midiSendNext();
}
/* ------------------------------------------------------------------------- */
