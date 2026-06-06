//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//********************************************************

//-------------------------------------------------------
// ESP32, ELRS GENERIC 900 True Diversity PA RX
//-------------------------------------------------------

//#define DEVICE_HAS_JRPIN5
//#define DEVICE_HAS_SERIAL_OR_COM // hold 5-way in down direction at boot to enable CLI
//#define DEVICE_HAS_IN
//#define DEVICE_HAS_NO_DEBUG
#define DEVICE_HAS_NO_COM
#define DEVICE_HAS_NO_SERIAL


#define DEVICE_HAS_SINGLE_LED
#define DEVICE_HAS_I2C_DISPLAY

//-- UARTS
// UARTB = serial port
// UART = output port, SBus or whatever
// UARTF = debug port

#if 0
#define UARTB_USE_SERIAL // serial, is connected to USB-C via USB<>UART also RT connector
#define UARTB_BAUD                TX_SERIAL_BAUDRATE
#define UARTB_USE_TX_IO           IO_P1
#define UARTB_USE_RX_IO           IO_P3
#define UARTB_TXBUFSIZE           TX_SERIAL_TXBUFSIZE
#define UARTB_RXBUFSIZE           TX_SERIAL_RXBUFSIZE
#endif

#if 1
#define UARTF_USE_SERIAL // serial, is connected to USB-C via USB<>UART also RT connector
//#define UARTF_BAUD                TX_SERIAL_BAUDRATE
#define UARTF_USE_TX_IO           IO_P1
#define UARTF_USE_RX_IO           IO_P3
#define UARTF_TXBUFSIZE           TX_SERIAL_TXBUFSIZE
#define UARTF_RXBUFSIZE           TX_SERIAL_RXBUFSIZE
#endif

#if 0
#define UART_USE_SERIAL1 // JR bay pin5
#define UART_BAUD                 400000
#define UART_USE_TX_IO            IO_P13
#define UART_USE_RX_IO            IO_P13
#define UART_TXBUFSIZE            0 // TX FIFO = 128
#define UART_RXBUFSIZE            0 // RX FIFO = 128 + 1

#define UARTE_USE_SERIAL1 // in port, uses JRPin5
#define UARTE_BAUD                 100000
#define UARTE_USE_TX_IO            -1
#define UARTE_USE_RX_IO            IO_P13
#define UARTE_RXBUFSIZE            0 // RX FIFO = 128 + 1
#endif

//-- SX1: SX12xx & SPI
// antenna1 = left ufl

#define SPI_CS_IO                 IO_P18
#define SPI_MISO                  IO_P19
#define SPI_MOSI                  IO_P27
#define SPI_SCK                   IO_P5
#define SPI_FREQUENCY             10000000L
#define SX_DIO0                   IO_P26
#define SX_DIO1                   IO_P33
#define SX_DIO2                   IO_P32
#define SX_RESET                  IO_P14
//#define SX_RX_EN                  IO_P9
//#define SX_TX_EN                  IO_P15

IRQHANDLER(void SX_DIO_EXTI_IRQHandler(void);)

void sx_init_gpio(void)
{
    gpio_init(SX_DIO0, IO_MODE_INPUT_PU);
    gpio_init(SX_DIO1, IO_MODE_INPUT_PU);
    gpio_init(SX_DIO2, IO_MODE_INPUT_PU);
    //gpio_init(SX_TX_EN, IO_MODE_OUTPUT_PP_LOW);
    //gpio_init(SX_RX_EN, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_LOW);
}

IRAM_ATTR void sx_amp_transmit(void)
{
    //gpio_low(SX_RX_EN);
    //gpio_high(SX_TX_EN);
}

IRAM_ATTR void sx_amp_receive(void)
{
    //gpio_low(SX_TX_EN);
    //gpio_high(SX_RX_EN);
}

void sx_dio_init_exti_isroff(void)
{
    detachInterrupt(SX_DIO0);
}

void sx_dio_enable_exti_isr(void)
{
    attachInterrupt(SX_DIO0, SX_DIO_EXTI_IRQHandler, RISING);
}

IRAM_ATTR void sx_dio_exti_isr_clearflag(void) {}

//-- Button

#if 0
#define BUTTON                    IO_P4

void button_init(void)
{
    gpio_init(BUTTON, IO_MODE_INPUT_PU);
}

IRAM_ATTR bool button_pressed(void)
{
    return gpio_read_activelow(BUTTON) ? true : false;
}
#endif

//-- LEDs


#define LED_RED                   IO_P25

void leds_init(void)
{
    gpio_init(LED_RED, IO_MODE_OUTPUT_PP_HIGH);
}

IRAM_ATTR void led_red_off(void) { gpio_high(LED_RED); }
IRAM_ATTR void led_red_on(void) { gpio_low(LED_RED); }
IRAM_ATTR void led_red_toggle(void) { gpio_toggle(LED_RED); }

IRAM_ATTR void led_green_off(void) {  }
IRAM_ATTR void led_green_on(void) {  }
IRAM_ATTR void led_green_toggle(void) {  }

IRAM_ATTR void led_white_off(void) {  }
IRAM_ATTR void led_white_on(void) {  }
IRAM_ATTR void led_white_toggle(void) {  }


//-- Display I2C

#define I2C_SDA_IO                IO_P21
#define I2C_SCL_IO                IO_P22
#define I2C_CLOCKSPEED            1000000L  // fix - rather too much, but helps with LQ, ESP32 max speed
#define I2C_BUFFER_SIZE           1024
#define DISPLAY_ENABLE_N          IO_P21

//-- 5 Way Switch

#define FIVEWAY_ADC_IO            IO_P4
#define KEY_UP_THRESH             3230
#define KEY_DOWN_THRESH           1205
#define KEY_LEFT_THRESH           1890
#define KEY_RIGHT_THRESH          2623
#define KEY_CENTER_THRESH         0

#if defined DEVICE_HAS_I2C_DISPLAY || defined DEVICE_HAS_I2C_DISPLAY_ROT180 || defined DEVICE_HAS_FIVEWAY

void fiveway_init(void) {
    gpio_init(FIVEWAY_ADC_IO, IO_MODE_INPUT_PU);
    gpio_init(DISPLAY_ENABLE_N, IO_MODE_OUTPUT_PP_LOW); // enable display
    gpio_low(DISPLAY_ENABLE_N); // enable display
} 

IRAM_ATTR uint16_t fiveway_adc_read(void)
{
    return analogRead(FIVEWAY_ADC_IO);
}

IRAM_ATTR uint8_t fiveway_read(void)
{
    int16_t adc = analogRead(FIVEWAY_ADC_IO);
    if (adc > (KEY_CENTER_THRESH-250) && adc < (KEY_CENTER_THRESH+250)) return (1 << KEY_CENTER);
    if (adc > (KEY_LEFT_THRESH-250) && adc < (KEY_LEFT_THRESH+250)) return (1 << KEY_LEFT); 
    if (adc > (KEY_DOWN_THRESH-250) && adc < (KEY_DOWN_THRESH+250)) return (1 << KEY_DOWN);
    if (adc > (KEY_UP_THRESH-250) && adc < (KEY_UP_THRESH+250)) return (1 << KEY_UP);
    if (adc > (KEY_RIGHT_THRESH-250) && adc < (KEY_RIGHT_THRESH+250)) return (1 << KEY_RIGHT);
    return 0;
}
#endif


//-- POWER

#define POWER_GAIN_DBM            16 // gain of a PA stage if present
#define POWER_SX1276_MAX          SX1276_OUTPUT_POWER_MAX // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           0 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_20_DBM, .mW = 100 }, // PA=14 gives ~22.5 dBm, 176 mW, PA=16 ~20.5 dBm, 111 mW,
    { .dbm = POWER_24_DBM, .mW = 250 }, // PA=14 gives ~25 dBm, 330 mW, PA=16 ~24.2 dBm, 256 mW
    { .dbm = POWER_27_DBM, .mW = 500 }, // PA=14 gives ~26.3 dBm, 420 mW, PA=16 ~25.7 dBm, 370 mW
};
