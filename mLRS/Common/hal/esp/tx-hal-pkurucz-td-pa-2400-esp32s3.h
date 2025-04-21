//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//********************************************************

//-------------------------------------------------------
// ESP32, ELRS GENERIC 2400 True Diversity PA Receiver as TX, can only do SiK
//-------------------------------------------------------

//#define DEVICE_HAS_SINGLE_LED
#define DEVICE_HAS_I2C_DISPLAY_ROT180
#define DEVICE_HAS_FIVEWAY
//#define DEVICE_HAS_DIVERSITY_SINGLE_SPI // must be set, doesn't work without it
//#define DEVICE_HAS_JRPIN5
//#define DEVICE_HAS_IN
//#define DEVICE_HAS_NO_COM
//#define DEVICE_HAS_NO_DEBUG
//#define DEVICE_HAS_NO_SERIAL
//#define DEVICE_HAS_SERIAL_OR_COM

// #define USE_FEATURE_MAVLINK_PARAMS // has no CLI, no Lua, hence needs this


//-- UARTS
// UARTB = serial port
// UARTC or USB = COM (CLI)
// UARTD = -
// UART  = JR bay pin5
// UARTE = in port, SBus or whatever
// UARTF = debug port

#define DEVICE_HAS_SERIAL_OR_COM
#define DEVICE_HAS_SERIAL_ON_USB
#if 0
#define UARTB_USE_SERIAL2
#define UARTB_BAUD                TX_SERIAL_BAUDRATE
#define UARTB_TXBUFSIZE           1024 // TX_SERIAL_TXBUFSIZE
#define UARTB_RXBUFSIZE           TX_SERIAL_RXBUFSIZE
#endif

//#define DEVICE_HAS_COM_ON_USB
#if 0
#define UARTC_USE_SERIAL
#define UARTC_BAUD                115200
//#define UARTC_USE_TX_IO           18 
//#define UARTC_USE_RX_IO           5
#define UARTC_TXBUFSIZE           0 // ?? // TX_COM_TXBUFSIZE
#define UARTC_RXBUFSIZE           TX_COM_RXBUFSIZE
#endif

#define UART_USE_SERIAL1
#define UART_BAUD                 400000
#define UART_USE_TX_IO            -1 // no Tx pin needed
#define UART_USE_RX_IO            14
#define UART_TXBUFSIZE            512 //0 // 128 fifo should be sufficient // 512
#define UART_RXBUFSIZE            512

#define UARTE_USE_SERIAL1
#define UARTE_BAUD                100000 // SBus normal baud rate, is being set later anyhow
#define UARTE_USE_TX_IO           -1 // no Tx pin needed
#define UARTE_USE_RX_IO           14
#define UARTE_TXBUFSIZE           0 // not used
#define UARTE_RXBUFSIZE           512

#define UARTF_USE_SERIAL
#define UARTF_BAUD                115200
#define UARTF_USE_TX_IO           13 
#define UARTF_USE_RX_IO           -1
#define UARTF_TXBUFSIZE           0 // ?? // 512


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

IRQHANDLER(void SX_DIO_EXTI_IRQHandler(void);)

void sx_init_gpio(void)
{
    gpio_init(SX_DIO1, IO_MODE_INPUT_ANALOG);
    gpio_init(SX_BUSY, IO_MODE_INPUT_PU);
    gpio_init(SX_TX_EN, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX_RX_EN, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_LOW);
}

IRAM_ATTR bool sx_busy_read(void)
{
    return (gpio_read_activehigh(SX_BUSY)) ? true : false;
}

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

void sx_dio_enable_exti_isr(void)
{
    attachInterrupt(SX_DIO1, SX_DIO_EXTI_IRQHandler, RISING);
}

void sx_dio_init_exti_isroff(void)
{
    detachInterrupt(SX_DIO1);
}

void sx_dio_exti_isr_clearflag(void) {}


//-- SX2: SX12xx & SPI

#define SX2_CS_IO                 IO_P47
#define SX2_BUSY                  IO_P34
#define SX2_DIO1                  IO_P33
#define SX2_RESET                 SX_RESET
#define SX2_RX_EN                 IO_P8
#define SX2_TX_EN                 IO_P7

IRQHANDLER(void SX2_DIO_EXTI_IRQHandler(void);)

void sx2_init_gpio(void)
{
    gpio_init(SX2_CS_IO, IO_MODE_OUTPUT_PP_HIGH);
    gpio_init(SX2_DIO1, IO_MODE_INPUT_ANALOG);
    gpio_init(SX2_BUSY, IO_MODE_INPUT_PU);
    gpio_init(SX2_TX_EN, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX2_RX_EN, IO_MODE_OUTPUT_PP_LOW);
#if (SX2_RESET != SX_RESET)
    gpio_init(SX2_RESET, IO_MODE_OUTPUT_PP_LOW);
#endif
}

IRAM_ATTR void spib_select(void)
{
    gpio_low(SX2_CS_IO);
}

IRAM_ATTR void spib_deselect(void)
{
    gpio_high(SX2_CS_IO);
}

IRAM_ATTR bool sx2_busy_read(void)
{
    return (gpio_read_activehigh(SX2_BUSY)) ? true : false;
}

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

void sx2_dio_init_exti_isroff(void)
{
    detachInterrupt(SX2_DIO1);
}

void sx2_dio_enable_exti_isr(void)
{
    attachInterrupt(SX2_DIO1, SX2_DIO_EXTI_IRQHandler, RISING);
}

void sx2_dio_exti_isr_clearflag(void) {}


//-- Button
//void button_init(void) {}
//IRAM_ATTR bool button_pressed(void) { return false; }

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

//-- Display I2C

#define I2C_SDA_IO                IO_P3
#define I2C_SCL_IO                IO_P2
#define I2C_CLOCKSPEED            1000000L  // fix - rather too much, but helps with LQ, ESP32 max speed
#define I2C_BUFFER_SIZE           1024


//-- 5 Way Switch

#define FIVEWAY_ADC_IO            IO_P14
#define KEY_UP_THRESH             3230
#define KEY_DOWN_THRESH           0
#define KEY_LEFT_THRESH           1890
#define KEY_RIGHT_THRESH          2623
#define KEY_CENTER_THRESH         1205

#if defined DEVICE_HAS_I2C_DISPLAY || defined DEVICE_HAS_I2C_DISPLAY_ROT180 || defined DEVICE_HAS_FIVEWAY

void fiveway_set_pu(void) {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = 1 << FIVEWAY_ADC_IO;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);
}

void fiveway_init(void) {
    //gpio_init(FIVEWAY_ADC_IO, IO_MODE_INPUT_ANALOG);
    gpio_init(FIVEWAY_ADC_IO, IO_MODE_INPUT_PU);
    analogRead(FIVEWAY_ADC_IO);
    gpio_init(FIVEWAY_ADC_IO, IO_MODE_INPUT_PU);

    // Manually configure the pull-up since the gpio_init() call above doesn't seem to do the right thing when doing adc calls...
    fiveway_set_pu();

} // no init needed to read an analog pin in Arduino

IRAM_ATTR uint16_t fiveway_adc_read(void)
{
    // Manually configure the pull-up since the gpio_init() call above doesn't seem to do the right thing when doing adc calls...
    fiveway_set_pu();

    return analogRead(FIVEWAY_ADC_IO);
}

extern void usb_puts(const char* str);

IRAM_ATTR uint8_t fiveway_read(void)
{
    led_white_on();

    int16_t adc = fiveway_adc_read();

#if 0
    do {
        char str[32] = "";

        snprintf(str, sizeof(str)-1, "ADC=%d\r\n", adc);
        usb_puts(str);

    } while(0);
#endif

    led_white_off();

    if(button_pressed()) return (1 << KEY_CENTER);

    if (adc > (KEY_CENTER_THRESH-250) && adc < (KEY_CENTER_THRESH+250)) return (1 << KEY_CENTER);
    if (adc > (KEY_LEFT_THRESH-250) && adc < (KEY_LEFT_THRESH+250)) return (1 << KEY_LEFT); 
    if (adc > (KEY_DOWN_THRESH-250) && adc < (KEY_DOWN_THRESH+250)) return (1 << KEY_DOWN);
    if (adc > (KEY_UP_THRESH-250) && adc < (KEY_UP_THRESH+250)) return (1 << KEY_UP);
    if (adc > (KEY_RIGHT_THRESH-250) && adc < (KEY_RIGHT_THRESH+250)) return (1 << KEY_RIGHT);
    return 0;
}
#endif


//-- Serial or Com Switch
// use com if FIVEWAY is DOWN during power up, else use serial

#ifdef DEVICE_HAS_SERIAL_OR_COM
bool tx_ser_or_com_serial = true; // we use serial as default

void ser_or_com_init(void)
{
#if defined(DEVICE_HAS_FIVEWAY)

    uint8_t cnt = 0;
    for (uint8_t i = 0; i < 16; i++) {
        int16_t adc = analogRead(FIVEWAY_ADC_IO);
        if (adc > (KEY_DOWN_THRESH-200) && adc < (KEY_DOWN_THRESH+200)) cnt++;
    }
    tx_ser_or_com_serial = !(cnt > 8);

    // Force to COM mode
    tx_ser_or_com_serial = false;
#else
    gpio_read_activelow(UP_BUTTON) ? true : false;
#endif
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

#define POWER_GAIN_DBM            25 // gain of a PA stage if present
#define POWER_SX1280_MAX_DBM      SX1280_POWER_0_DBM  // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           1 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_0_DBM, .mW =  1 },
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_20_DBM, .mW = 100 },
};

