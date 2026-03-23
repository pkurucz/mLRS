//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP USB VCP
//********************************************************
// For ESP32:
// usefull resource
// - https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/uart.html
//********************************************************
#ifndef ESPLIB_USB_VCP_H
#define ESPLIB_USB_VCP_H

#ifdef ESP32
#include "driver/uart.h"
#include "hal/uart_ll.h"
#include <HWCDC.h>
#include <USB.h>

#endif


#ifndef UARTVCP_TXBUFSIZE
  #define UARTVCP_TXBUFSIZE       256 // MUST be 2^N
#endif
#ifndef UARTF_RXBUFSIZE
  #define UARTVCP_RXBUFSIZE       256 // MUST be 2^N
#endif

#ifdef ESP32
  #if (UARTVCP_TXBUFSIZE > 0) && (UARTVCP_TXBUFSIZE < 256) 
    #error UARTVCP_TXBUFSIZE must be 0 or >= 256
  #endif
  #if (UARTVCP_RXBUFSIZE < 256) 
    #error UARTVCP_RXBUFSIZE must be >= 256
  #endif
#endif


//-------------------------------------------------------
// TX routines
//-------------------------------------------------------

//void flush(void) override { IFNSER(); usb_flush(); }
IRAM_ATTR void usb_flush(void) {
    USBSerial.flush();
}

//void putbuf(uint8_t* buf, uint16_t len) override { usb_putbuf(buf, len); }
IRAM_ATTR void usb_putbuf(const uint8_t* buf, uint16_t len) {
    if(USBSerial.availableForWrite()) {
        USBSerial.write(buf, len);
//        usb_flush();
    }
}

IRAM_ATTR uint8_t usb_tx_full(void) {
	return !(USBSerial.availableForWrite());
}

IRAM_ATTR void usb_puts(const char * str) {
    usb_putbuf((const uint8_t*)str, strlen(str));
}

//-------------------------------------------------------
// RX routines
//-------------------------------------------------------

//bool available(void) override { return usb_rx_available(); }
bool usb_rx_available(void) {
    return (USBSerial.available() > 0) ? true : false;
}

//char getc(void) override { return usb_getc(); }
char usb_getc(void) {
    return USBSerial.read();
}


// uint16_t bytes_available(void) override { IFNSER(0); return usb_rx_bytesavailable(); }
uint16_t usb_rx_bytesavailable(void) {
    return USBSerial.available();
}


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------
// Note: ESP32 has a hardware fifo for tx, which is 128 bytes in size. However, MAVLink messages
// can be larger than this, and data would thus be lost when put only into the fifo. It is therefore
// crucial to set a Tx buffer size of sufficient size. setTxBufferSize() is not available for ESP82xx.

#if ARDUINO_USB_MODE
#if ARDUINO_USB_CDC_ON_BOOT//Serial used for USB CDC
#define USB_SERIAL_CLASS Serial;
#else
#define USB_SERIAL_CLASS USBSerial;
#endif
#endif

static void usbEventCallback(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
  if(event_base == ARDUINO_USB_EVENTS){
    arduino_usb_event_data_t * data = (arduino_usb_event_data_t*)event_data;
    switch (event_id){
      case ARDUINO_USB_STARTED_EVENT:
        //Serial.println("USB PLUGGED");
        break;
      case ARDUINO_USB_STOPPED_EVENT:
        //Serial.println("USB UNPLUGGED");
        break;
      case ARDUINO_USB_SUSPEND_EVENT:
        //Serial.printf("USB SUSPENDED: remote_wakeup_en: %u\n", data->suspend.remote_wakeup_en);
        break;
      case ARDUINO_USB_RESUME_EVENT:
        //Serial.println("USB RESUMED");
        break;
      
      default:
        break;
    }
  } else if(event_base == ARDUINO_USB_CDC_EVENTS){
    arduino_usb_cdc_event_data_t * data = (arduino_usb_cdc_event_data_t*)event_data;
    switch (event_id){
      case ARDUINO_USB_CDC_CONNECTED_EVENT:
        //Serial.println("CDC CONNECTED");
        break;
      case ARDUINO_USB_CDC_DISCONNECTED_EVENT:
        //Serial.println("CDC DISCONNECTED");
        break;
      case ARDUINO_USB_CDC_LINE_STATE_EVENT:
        //Serial.printf("CDC LINE STATE: dtr: %u, rts: %u\n", data->line_state.dtr, data->line_state.rts);
        break;
      case ARDUINO_USB_CDC_LINE_CODING_EVENT:
        //Serial.printf("CDC LINE CODING: bit_rate: %lu, data_bits: %u, stop_bits: %u, parity: %u\n", data->line_coding.bit_rate, data->line_coding.data_bits, data->line_coding.stop_bits, data->line_coding.parity);
        break;
      case ARDUINO_USB_CDC_RX_EVENT:
      #if 0
        Serial.printf("CDC RX [%u]:", data->rx.len);
        {
            uint8_t buf[data->rx.len];
            size_t len = USBSerial.read(buf, data->rx.len);
            Serial.write(buf, len);
        }
        Serial.println();
        #endif
        break;
       case ARDUINO_USB_CDC_RX_OVERFLOW_EVENT:
        //Serial.printf("CDC RX Overflow of %d bytes", data->rx_overflow.dropped_bytes);
        break;
     
      default:
        break;
    }
  }
}

void usb_init(void) {
    USBSerial.setTxBufferSize(UARTVCP_TXBUFSIZE);
    USBSerial.setRxBufferSize(UARTVCP_RXBUFSIZE);
    USBSerial.setTxTimeoutMs(1);
    USBSerial.setDebugOutput(true);
    //USBSerial.enableReboot(true);

    USB.onEvent(usbEventCallback);
    //USBSerial.onEvent(usbEventCallback);
    
    USBSerial.begin();
//    USB.begin();
}


#endif // ESPLIB_USB_VCP_H
