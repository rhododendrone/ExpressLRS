#pragma once

/**
 * Hardware configuration for the CRSF Bridge on an ESP32-C3 Supermini.
 *
 * This target combines the TX (handset / EdgeTX) and RX (flight-controller)
 * serial ports in a single device with NO RF radio circuitry.
 *
 *   GPIO 1:  S.PORT – half-duplex CRSF to/from EdgeTX radio (400 kbaud)
 *   GPIO 20: FC RX  – CRSF from flight controller (420 kbaud)
 *   GPIO 21: FC TX  – CRSF to flight controller   (420 kbaud)
 */

// -----------------------------------------------------------------
// S.PORT serial (EdgeTX, half-duplex).
// Both TX and RX must point to the same GPIO to enable half-duplex.
// -----------------------------------------------------------------
#define GPIO_PIN_RCSIGNAL_RX   1
#define GPIO_PIN_RCSIGNAL_TX   1

// -----------------------------------------------------------------
// FC serial (Betaflight / INAV, full-duplex)
// -----------------------------------------------------------------
#define GPIO_PIN_SERIAL1_RX   20
#define GPIO_PIN_SERIAL1_TX   21

// -----------------------------------------------------------------
// No radio hardware
// -----------------------------------------------------------------
#define GPIO_PIN_NSS          UNDEF_PIN
#define GPIO_PIN_NSS_2        UNDEF_PIN
#define GPIO_PIN_MOSI         UNDEF_PIN
#define GPIO_PIN_MISO         UNDEF_PIN
#define GPIO_PIN_SCK          UNDEF_PIN
#define GPIO_PIN_RST          UNDEF_PIN
#define GPIO_PIN_RST_2        UNDEF_PIN
#define GPIO_PIN_DIO0         UNDEF_PIN
#define GPIO_PIN_DIO0_2       UNDEF_PIN
#define GPIO_PIN_DIO1         UNDEF_PIN
#define GPIO_PIN_DIO1_2       UNDEF_PIN
#define GPIO_PIN_BUSY         UNDEF_PIN
#define GPIO_PIN_BUSY_2       UNDEF_PIN
#define GPIO_PIN_PA_ENABLE    UNDEF_PIN
#define GPIO_PIN_RX_ENABLE    UNDEF_PIN
#define GPIO_PIN_RX_ENABLE_2  UNDEF_PIN
#define GPIO_PIN_TX_ENABLE    UNDEF_PIN
#define GPIO_PIN_TX_ENABLE_2  UNDEF_PIN
#define GPIO_PIN_ANT_CTRL     UNDEF_PIN

// -----------------------------------------------------------------
// No LEDs, buttons, I2C or peripheral sensors
// -----------------------------------------------------------------
#define GPIO_PIN_LED_WS2812   UNDEF_PIN
#define GPIO_PIN_LED_RED      UNDEF_PIN
#define GPIO_PIN_LED_GREEN    UNDEF_PIN
#define GPIO_PIN_LED_BLUE     UNDEF_PIN
#define GPIO_PIN_BUTTON       UNDEF_PIN
#define GPIO_PIN_SCL          UNDEF_PIN
#define GPIO_PIN_SDA          UNDEF_PIN

// -----------------------------------------------------------------
// Feature flags
// -----------------------------------------------------------------
#define OPT_HAS_SCREEN        false
#define OPT_HAS_GSENSOR       false
#define OPT_HAS_THERMAL       false

// -----------------------------------------------------------------
// No VBAT ADC
// -----------------------------------------------------------------
#define GPIO_ANALOG_VBAT      UNDEF_PIN
#define ANALOG_VBAT_OFFSET    0
#define ANALOG_VBAT_SCALE     0

// -----------------------------------------------------------------
// No PWM servo outputs
// -----------------------------------------------------------------
#define GPIO_PIN_PWM_OUTPUTS       {}
#define GPIO_PIN_PWM_OUTPUTS_COUNT  0
#define OPT_HAS_SERVO_OUTPUT        false
