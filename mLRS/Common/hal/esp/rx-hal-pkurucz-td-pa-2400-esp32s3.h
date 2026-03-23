//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//********************************************************

//-------------------------------------------------------
// FlySky PR02 2.4 GHz True Diversity Receiver ESP32S3
//-------------------------------------------------------

#define DEVICE_HAS_OUT
#define DEVICE_HAS_DIVERSITY_SINGLE_SPI // must be set, doesn't work without it
//#define DEVICE_HAS_SINGLE_LED_RGB

#if 1
//-- UARTS
// UARTB = serial port
// UART = output port, SBus or whatever
// UARTF = debug port

#define UARTB_USE_SERIAL
#define UARTB_BAUD                RX_SERIAL_BAUDRATE
#define UARTB_USE_TX_IO           IO_P14
#define UARTB_USE_RX_IO           IO_P13
#define UARTB_TXBUFSIZE           RX_SERIAL_TXBUFSIZE
#define UARTB_RXBUFSIZE           RX_SERIAL_RXBUFSIZE

#define UART_USE_SERIAL1 
#define UART_BAUD                 416666   // CRSF baud rate
#define UART_USE_TX_IO            IO_P12   // TX2 pad
#define UART_USE_RX_IO            -1       
#define UART_TXBUFSIZE            256

#define UARTF_USE_SERIAL2
#define UARTF_BAUD                115200
#define UARTF_USE_TX_IO           IO_P43   // TX3 pad
#define UARTF_USE_RX_IO           IO_P44   // RX3 pad
#define UARTF_TXBUFSIZE           0        // TX FIFO = 128

#else


//-- UARTS
// UARTB = serial port
// UART = output port, SBus or whatever
// UARTC = debug port

#define UART_USE_SERIAL1

#define UARTB_USE_SERIAL2
#define UARTB_BAUD                RX_SERIAL_BAUDRATE
#define UARTB_TXBUFSIZE           RX_SERIAL_TXBUFSIZE
#define UARTB_RXBUFSIZE           RX_SERIAL_RXBUFSIZE

#define DEVICE_HAS_DEBUG_ON_USB
#if 0
#define UARTC_USE_SERIAL
#define UARTC_BAUD                115200
#endif





#endif

//-- SX1: SX12xx & SPI

#define SPI_CS_IO                 IO_P36
#define SPI_MISO                  IO_P39
#define SPI_MOSI                  IO_P38
#define SPI_SCK                   IO_P37
#define SPI_FREQUENCY             1800000L
#define SX_BUSY                   IO_P42
#define SX_DIO1                   IO_P41
#define SX_RESET                  IO_P35
#define SX_RX_EN                  IO_P46
#define SX_TX_EN                  IO_P45

//#define SX_USE_REGULATOR_MODE_DCDC

IRQHANDLER(void SX_DIO_EXTI_IRQHandler(void);)

void sx_init_gpio(void)
{
    gpio_init(SX_DIO1, IO_MODE_INPUT_ANALOG);
    gpio_init(SX_BUSY, IO_MODE_INPUT_ANALOG);
//    gpio_init(SX_BUSY, IO_MODE_INPUT_PU);
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX_TX_EN, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX_RX_EN, IO_MODE_OUTPUT_PP_LOW);
}

IRAM_ATTR bool sx_busy_read(void) { return (gpio_read_activehigh(SX_BUSY)) ? true : false; }
//IRAM_ATTR bool sx_busy_read(void) { return false; } // we do not use busy pin, so always return not busy to avoid unnecessary waiting

IRAM_ATTR void sx_amp_transmit(void)
{
    gpio_low(SX_RX_EN);
    gpio_high(SX_TX_EN);
}

IRAM_ATTR void sx_amp_receive(void)
{
    gpio_low(SX_TX_EN);
    gpio_high(SX_RX_EN);
}

void sx_dio_init_exti_isroff(void) { detachInterrupt(SX_DIO1); }
void sx_dio_enable_exti_isr(void) { attachInterrupt(SX_DIO1, SX_DIO_EXTI_IRQHandler, RISING); }
void sx_dio_exti_isr_clearflag(void) {}


//-- SX2: SX128x & SPI

#define SX2_CS_IO                 IO_P47
#define SX2_BUSY                  IO_P34
#define SX2_DIO1                  IO_P33
#define SX2_RESET                 IO_P1  // Actually SX_RESET, but stubbing in for testing
#define SX2_RX_EN                 IO_P8
#define SX2_TX_EN                 IO_P7

//#define SX2_USE_REGULATOR_MODE_DCDC

IRQHANDLER(void SX2_DIO_EXTI_IRQHandler(void);)

void sx2_init_gpio(void)
{
    gpio_init(SX2_CS_IO, IO_MODE_OUTPUT_PP_HIGH);
    gpio_init(SX2_DIO1, IO_MODE_INPUT_ANALOG);
    gpio_init(SX2_BUSY, IO_MODE_INPUT_ANALOG);
//    gpio_init(SX2_BUSY, IO_MODE_INPUT_PU);
#if (SX2_RESET != SX_RESET)
    gpio_init(SX2_RESET, IO_MODE_OUTPUT_PP_LOW);
#endif
    gpio_init(SX2_TX_EN, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX2_RX_EN, IO_MODE_OUTPUT_PP_LOW);
}

IRAM_ATTR void spib_select(void) { gpio_low(SX2_CS_IO); }
IRAM_ATTR void spib_deselect(void) { gpio_high(SX2_CS_IO); }
IRAM_ATTR bool sx2_busy_read(void) { return (gpio_read_activehigh(SX2_BUSY)) ? true : false; }
//IRAM_ATTR bool sx2_busy_read(void) { return false; } // we do not use busy pin, so always return not busy to avoid unnecessary waiting  

IRAM_ATTR void sx2_amp_transmit(void)
{
    gpio_low(SX2_RX_EN);
    gpio_high(SX2_TX_EN);
}

IRAM_ATTR void sx2_amp_receive(void)
{
    gpio_low(SX2_TX_EN);
    gpio_high(SX2_RX_EN);
}

void sx2_dio_init_exti_isroff(void) { detachInterrupt(SX2_DIO1); }
void sx2_dio_enable_exti_isr(void) { attachInterrupt(SX2_DIO1, SX2_DIO_EXTI_IRQHandler, RISING); }
void sx2_dio_exti_isr_clearflag(void) {}


//-- Out port

void out_init_gpio(void) {}
void out_set_normal(void) { /* gpio_matrix_out((gpio_num_t)UART_USE_TX_IO, U1TXD_OUT_IDX, false, false); */ }
void out_set_inverted(void) { /* gpio_matrix_out((gpio_num_t)UART_USE_TX_IO, U1TXD_OUT_IDX, true, false); */ }


//-- Button

#define ENTER_BUTTON                    IO_P0
#define UP_BUTTON                       IO_P15
#define DOWN_BUTTON                     IO_P16

void button_init(void)
{
    gpio_init(ENTER_BUTTON, IO_MODE_INPUT_PU);
    gpio_init(UP_BUTTON, IO_MODE_INPUT_PU);
    gpio_init(DOWN_BUTTON, IO_MODE_INPUT_PU);
}

IRAM_ATTR bool button_pressed(void)
{
    return gpio_read_activelow(ENTER_BUTTON) ? true : false;
}


//-- LEDs
#define LED_WHITE                   IO_P21
#define LED_RED                     IO_P18
#define LED_GREEN                   IO_P17

void leds_init(void)
{
    gpio_init(LED_WHITE, IO_MODE_OUTPUT_PP_HIGH);
    gpio_init(LED_RED, IO_MODE_OUTPUT_PP_HIGH);
    gpio_init(LED_GREEN, IO_MODE_OUTPUT_PP_HIGH);
}

IRAM_ATTR void led_white_off(void) { gpio_high(LED_WHITE); }
IRAM_ATTR void led_white_on(void) { gpio_low(LED_WHITE); }
IRAM_ATTR void led_white_toggle(void) { gpio_toggle(LED_WHITE); }
IRAM_ATTR void led_red_off(void) { gpio_high(LED_RED); }
IRAM_ATTR void led_red_on(void) { gpio_low(LED_RED); }
IRAM_ATTR void led_red_toggle(void) { gpio_toggle(LED_RED); }
IRAM_ATTR void led_green_off(void) { gpio_high(LED_GREEN); }
IRAM_ATTR void led_green_on(void) { gpio_low(LED_GREEN); }
IRAM_ATTR void led_green_toggle(void) { gpio_toggle(LED_GREEN); }

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
#ifndef POWER_OVERLAY

#define POWER_GAIN_DBM            25 // gain of a PA stage if present
#define POWER_SX1280_MAX          SX1280_POWER_0_DBM  // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           1 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_14_DBM, .mW = 25 },
    { .dbm = POWER_17_DBM, .mW = 50 },
    { .dbm = POWER_20_DBM, .mW = 100 },
//    { .dbm = POWER_24_DBM, .mW = 250 },
};

#endif // !POWER_OVERLAY
