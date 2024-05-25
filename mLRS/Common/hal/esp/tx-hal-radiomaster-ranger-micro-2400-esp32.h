//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//*******************************************************


//-------------------------------------------------------
// ESP32, ELRS RADIOMASTER RANGER MICRO 2400 TX
//-------------------------------------------------------
/*
    "serial_rx": 13,
    "serial_tx": 13,
    "radio_busy": 22,
    "radio_dio1": 21,
    "radio_miso": 19,
    "radio_mosi": 23,
    "radio_nss": 4,
    "radio_rst": 5,
    "radio_sck": 18,
    "radio_dcdc": true,
    "power_rxen": 32,
    "power_txen": 33,
    "power_min": 1,
    "power_high": 6,
    "power_max": 6,
    "power_default": 2,
    "power_control": 0,
    "power_values": [-17,-15,-12,-7,-4,2],
    "led_rgb": 15,
    "led_rgb_isgrb": true,
    "use_backpack": true,
    "debug_backpack_baud": 460800,
    "debug_backpack_rx": 16,
    "debug_backpack_tx": 17,
    "backpack_boot": 26,
    "backpack_en": 25,
    "passthrough_baud": 230400,
    "misc_fan_en": 27
*/


//#define DEVICE_HAS_JRPIN5
//#define DEVICE_HAS_IN
#define DEVICE_HAS_SERIAL_OR_COM // board has UART which is shared between Serial or Com, selected by e.g. a switch
//#define DEVICE_HAS_NO_SERIAL
//#define DEVICE_HAS_NO_COM
#define DEVICE_HAS_NO_DEBUG

#define DEVICE_HAS_SINGLE_LED
//#define DEVICE_HAS_FIVEWAY
#define DEVICE_HAS_FAN_ONOFF // board has a Fan, which can be set on or off
#define DEVICE_HAS_ESP_WIFI_BRIDGE_ON_SERIAL2


//-- UARTS
// UARTB = serial port
// UARTC or USB = COM (CLI)
// UARTD = serial2 BT/ESP port
// UART  = JR bay pin5
// UARTE = in port, SBus or whatever
// UARTF = debug port

#define UARTB_USE_SERIAL // serial
#define UARTB_BAUD                TX_SERIAL_BAUDRATE
//#define UARTB_USE_TX_IO           IO_P17
//#define UARTB_USE_RX_IO           IO_P16
#define UARTB_TXBUFSIZE           1024 // TX_SERIAL_TXBUFSIZE
#define UARTB_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

#define UARTC_USE_SERIAL // com USB/CLI
#define UARTC_BAUD                115200
#define UARTC_TXBUFSIZE           0 // ?? // TX_COM_TXBUFSIZE
#define UARTC_RXBUFSIZE           TX_COM_RXBUFSIZE

#define UARTD_USE_SERIAL1 // serial2 BT/ESP
#define UARTD_BAUD                115200
#define UARTD_USE_TX_IO           IO_P17
#define UARTD_USE_RX_IO           IO_P16
#define UARTD_TXBUFSIZE           1024 // TX_SERIAL_TXBUFSIZE
#define UARTD_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

#define UARTF_USE_SERIAL // debug
#define UARTF_BAUD                115200
#define UARTF_TXBUFSIZE           0 // ?? // 512


//-- SX1: SX12xx & SPI

#define SPI_CS_IO                 IO_P4
#define SPI_MISO                  IO_P19
#define SPI_MOSI                  IO_P23
#define SPI_SCK                   IO_P18
#define SPI_FREQUENCY             10000000L
#define SX_RESET                  IO_P5
#define SX_BUSY                   IO_P22
#define SX_DIO1                   IO_P21
#define SX_TX_EN                  IO_P33
#define SX_RX_EN                  IO_P32

#define SX_USE_RFO

IRQHANDLER(void SX_DIO_EXTI_IRQHandler(void);)

void sx_init_gpio(void)
{
    gpio_init(SX_DIO1, IO_MODE_INPUT_ANALOG);
    gpio_init(SX_BUSY, IO_MODE_INPUT_PU);
    gpio_init(SX_TX_EN, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX_RX_EN, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_HIGH);
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

IRAM_ATTR void sx_dio_enable_exti_isr(void)
{
    attachInterrupt(SX_DIO1, SX_DIO_EXTI_IRQHandler, RISING);
}

void sx_dio_init_exti_isroff(void) {}
void sx_dio_exti_isr_clearflag(void) {}


//-- Button

void button_init(void)
{
}

IRAM_ATTR bool button_pressed(void)
{
    return false;
}


//-- LEDs
#include <NeoPixelBus.h>
#define LED_RED                   IO_P16
bool ledRedState;
bool ledGreenState;
bool ledBlueState;

NeoPixelBus<NeoGrbFeature, NeoEsp32I2s0Ws2812xMethod> ledRGB(1, LED_RED);

void leds_init(void)
{
    ledRGB.Begin();
    ledRGB.Show();
}

IRAM_ATTR void led_red_off(void)
{
    if (!ledRedState) return;
    ledRGB.SetPixelColor(0, RgbColor(0, 0, 0));
    ledRGB.Show();
    ledRedState = 0;
}

IRAM_ATTR void led_red_on(void)
{
    if (ledRedState) return;
    ledRGB.SetPixelColor(0, RgbColor(255, 0, 0));
    ledRGB.Show();
    ledRedState = 1;
}

IRAM_ATTR void led_red_toggle(void)
{
    if (ledRedState) { led_red_off(); } else { led_red_on(); }
}

IRAM_ATTR void led_green_off(void)
{
    if (!ledGreenState) return;
    ledRGB.SetPixelColor(0, RgbColor(0, 0, 0));
    ledRGB.Show();
    ledGreenState = 0;
}

IRAM_ATTR void led_green_on(void)
{
    if (ledGreenState) return;
    ledRGB.SetPixelColor(0, RgbColor(0, 255, 0));
    ledRGB.Show();
    ledGreenState = 1;
}

IRAM_ATTR void led_green_toggle(void)
{
    if (ledGreenState) { led_green_off(); } else { led_green_on(); }
}

IRAM_ATTR void led_blue_off(void)
{
    if (!ledBlueState) return;
    ledRGB.SetPixelColor(0, RgbColor(0, 0, 0));
    ledRGB.Show();
    ledBlueState = 0;
}

IRAM_ATTR void led_blue_on(void)
{
    if (ledBlueState) return;
    ledRGB.SetPixelColor(0, RgbColor(0, 0, 255));
    ledRGB.Show();
    ledBlueState = 1;
}

IRAM_ATTR void led_blue_toggle(void)
{
    if (ledBlueState) { led_blue_off(); } else { led_blue_on(); }
}

//-- Serial or Com Switch
// use com if button is DOWN during power up, else use serial

#ifdef DEVICE_HAS_SERIAL_OR_COM
bool tx_ser_or_com_serial = true; // we use serial as default

void ser_or_com_init(void)
{
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < 16; i++) {
        if (button_pressed()) cnt++;
    }
    tx_ser_or_com_serial = !(cnt > 8);
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


//-- Cooling Fan

#define FAN_IO                    IO_P27

void fan_init(void)
{
    gpio_init(FAN_IO, IO_MODE_OUTPUT_PP_LOW);
    gpio_low(FAN_IO);
}

IRAM_ATTR void fan_set_power(int8_t power_dbm)
{
    if (power_dbm >= POWER_23_DBM) {
        gpio_high(FAN_IO);
    } else {
        gpio_low(FAN_IO);
    }
}


//-- ESP32 Wifi Bridge

#define ESP_RESET                 IO_P25 // backpack_en
#define ESP_GPIO0                 IO_P26 // backpack_boot, seems to be inverted
//#define ESP_DTR                   IO_PC14 // DTR from USB-TTL adapter -> GPIO
//#define ESP_RTS                   IO_PC3  // RTS from USB-TTL adapter -> RESET

#ifdef DEVICE_HAS_ESP_WIFI_BRIDGE_ON_SERIAL2
void esp_init(void)
{
    gpio_init(ESP_GPIO0, IO_MODE_OUTPUT_PP_LOW); // high -> esp will start in bootloader mode
    gpio_init(ESP_RESET, IO_MODE_OUTPUT_PP_LOW); // low -> esp is in reset
}

IRAM_ATTR void esp_reset_high(void) { gpio_high(ESP_RESET); }
IRAM_ATTR void esp_reset_low(void) { gpio_low(ESP_RESET); }

IRAM_ATTR void esp_gpio0_high(void) { gpio_low(ESP_GPIO0); }
IRAM_ATTR void esp_gpio0_low(void) { gpio_high(ESP_GPIO0); }

//IRAM_ATTR uint8_t esp_dtr_rts(void) { return 0; }
#endif

//-- POWER

#define POWER_GAIN_DBM            18 // gain of a PA stage if present
#define POWER_SX1280_MAX_DBM      SX1280_POWER_3_DBM  // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           1 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_MIN, .mW = INT8_MIN },
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_17_DBM, .mW = 50 },
    { .dbm = POWER_20_DBM, .mW = 100 },
};
