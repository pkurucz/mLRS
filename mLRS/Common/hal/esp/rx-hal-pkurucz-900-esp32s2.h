//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//********************************************************

//-------------------------------------------------------
// ESP32, ELRS GENERIC 900 RX
//-------------------------------------------------------

#define DEVICE_HAS_SINGLE_LED
//#define DEVICE_HAS_DIVERSITY_SINGLE_SPI // must be set, doesn't work without it
//#define DEVICE_HAS_NO_DEBUG
#define DEVICE_HAS_SERIAL_OR_DEBUG
#define DEBUG_ENABLED


//-- UARTS
// UART = output port, SBus or whatever
// UARTB = serial port
// UARTC = debug port  (UARTF?)

#define UART_USE_SERIAL1
#define UART_USE_TX_IO            IO_P43
#define UART_USE_RX_IO            IO_P44

#define UARTB_USE_SERIAL2
#define UARTB_BAUD                RX_SERIAL_BAUDRATE
#define UARTB_TXBUFSIZE           RX_SERIAL_TXBUFSIZE
#define UARTB_RXBUFSIZE           RX_SERIAL_RXBUFSIZE
#define UARTB_USE_TX_IO           IO_P12
#define UARTB_USE_RX_IO           IO_P11

#define UARTF_USE_SERIAL1
#define UARTF_BAUD                115200
#define UARTF_USE_TX_IO           IO_P43
#define UARTF_USE_RX_IO           IO_P44

//#define DEVICE_HAS_DEBUG_ON_USB
#if 0
#define UARTC_USE_SERIAL
#define UARTC_BAUD                115200
#endif

//-- SX1: SX12xx & SPI

#define SPI_CS_IO                 IO_P36
#define SPI_MISO                  IO_P39
#define SPI_MOSI                  IO_P38
#define SPI_SCK                   IO_P37
#define SPI_FREQUENCY             1800000L
#define SX_BUSY                   IO_P46
#define SX_DIO1                   IO_P2
#define SX_RESET                  IO_P45

//#define SX_RX_EN                  IO_P46
//#define SX_TX_EN                  IO_P45

#define SX_TR_SWITCH              IO_P40

#define BUZZER_ON_N               IO_P1
#define INTERNAL_ACC_EN_N         IO_P21
#define EXTERNAL_ACC_EN_N         IO_P35


IRQHANDLER(void SX_DIO_EXTI_IRQHandler(void);)

void sx_init_gpio(void)
{
    gpio_init(SX_DIO1, IO_MODE_INPUT_ANALOG);
    gpio_init(SX_BUSY, IO_MODE_INPUT_PU);
//    gpio_init(SX_TX_EN, IO_MODE_OUTPUT_PP_LOW);
//    gpio_init(SX_RX_EN, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX_TR_SWITCH, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_LOW);

    gpio_init(INTERNAL_ACC_EN_N, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(EXTERNAL_ACC_EN_N, IO_MODE_OUTPUT_PP_LOW);
}

IRAM_ATTR bool sx_busy_read(void)
{
    return (gpio_read_activehigh(SX_BUSY)) ? true : false;
}

IRAM_ATTR void sx_amp_transmit(void)
{
    gpio_low(SX_TR_SWITCH);
//    gpio_low(SX_RX_EN);
//    gpio_high(SX_TX_EN);
}

IRAM_ATTR void sx_amp_receive(void)
{
    gpio_high(SX_TR_SWITCH);
//    gpio_low(SX_TX_EN);
//    gpio_high(SX_RX_EN);
}

void sx_dio_init_exti_isroff(void)
{
    detachInterrupt(SX_DIO1);
}

void sx_dio_enable_exti_isr(void)
{
    attachInterrupt(SX_DIO1, SX_DIO_EXTI_IRQHandler, RISING);
}

IRAM_ATTR void sx_dio_exti_isr_clearflag(void) {}

//-- Button

#define BUTTON                    IO_P0

void button_init(void)
{
    gpio_init(BUTTON, IO_MODE_INPUT_PU);
}

IRAM_ATTR bool button_pressed(void)
{
    return gpio_read_activelow(BUTTON) ? true : false;
}

//-- LEDs

#define LED_RED                   IO_P16

void leds_init(void)
{
    gpio_init(LED_RED, IO_MODE_OUTPUT_PP_HIGH);
}

IRAM_ATTR void led_red_off(void) { gpio_high(LED_RED); }
IRAM_ATTR void led_red_on(void) { gpio_low(LED_RED); }
IRAM_ATTR void led_red_toggle(void) { gpio_toggle(LED_RED); }

IRAM_ATTR void led_green_off(void) { gpio_high(LED_RED); }
IRAM_ATTR void led_green_on(void) { gpio_low(LED_RED); }
IRAM_ATTR void led_green_toggle(void) { gpio_toggle(LED_RED); }

IRAM_ATTR void led_white_off(void) { gpio_high(LED_RED); }
IRAM_ATTR void led_white_on(void) { gpio_low(LED_RED); }
IRAM_ATTR void led_white_toggle(void) { gpio_toggle(LED_RED); }

//-- Serial or Com Switch
// use com if FIVEWAY is DOWN during power up, else use serial

#ifdef DEVICE_HAS_SERIAL_OR_COM
bool tx_ser_or_com_serial = true; // we use serial as default

void ser_or_com_init(void)
{
    gpio_read_activelow(UP_BUTTON) ? true : false;
}

IRAM_ATTR bool ser_or_com_serial(void)
{
    return tx_ser_or_com_serial;
}

IRAM_ATTR void ser_or_com_set_to_com(void)
{
    tx_ser_or_com_serial = false;
}
#endif


//-- POWER

#define POWER_PA_NONE_SX126X
#include "../hal-power-pa.h"


