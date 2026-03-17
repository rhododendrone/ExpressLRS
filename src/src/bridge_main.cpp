/**
 * CRSF Bridge Firmware for ESP32-C3 Supermini
 *
 * Combines TX-module and RX-module functionality in a single device with
 * NO RF radio circuitry.  Acts as a wired serial proxy between an EdgeTX
 * radio transmitter and a Betaflight / INAV flight controller.
 *
 * Hardware (ESP32-C3 Supermini):
 *   GPIO  1 : S.PORT  – half-duplex CRSF to/from EdgeTX  (400 kbaud)
 *   GPIO 20 : FC RX   – CRSF from flight controller       (420 kbaud)
 *   GPIO 21 : FC TX   – CRSF to flight controller         (420 kbaud)
 *
 * CRSF message routing:
 *   EdgeTX → FC  : RC channels, MSP requests, parameter writes
 *   FC → EdgeTX  : Telemetry (battery, GPS, attitude, flight-mode …)
 *   Bridge       : responds to DEVICE_PING on behalf of the bridge
 *                  device so EdgeTX can enumerate it via the Lua script.
 */

#include <Arduino.h>

#if defined(PLATFORM_ESP32)
#include <driver/gpio.h>
#include <hal/uart_ll.h>
#include <soc/uart_reg.h>
#endif

#include "crsf_protocol.h"
#include "CRSFParser.h"
#include "CRSFRouter.h"
#include "options.h"

// =====================================================================
// Pin and baudrate defaults (overridable via build flags)
// =====================================================================
#ifndef SPORT_PIN
#define SPORT_PIN     GPIO_PIN_RCSIGNAL_RX  // 1  – EdgeTX S.PORT (half-duplex)
#endif
#ifndef FC_RX_PIN
#define FC_RX_PIN     GPIO_PIN_SERIAL1_RX   // 20 – from flight controller
#endif
#ifndef FC_TX_PIN
#define FC_TX_PIN     GPIO_PIN_SERIAL1_TX   // 21 – to flight controller
#endif

// ELRS standard CRSF baud for the EdgeTX module-bay serial port
#ifndef HANDSET_BAUD
#define HANDSET_BAUD  400000
#endif

// Standard CRSF baud for Betaflight / INAV
#ifndef FC_BAUD
#define FC_BAUD       420000
#endif

// =====================================================================
// GPIO-matrix constant for "always-HIGH" signal input (chip-specific)
//
// Used to detach the UART0 RX from the physical pin by routing the
// GPIO-matrix RX slot to an internal constant-HIGH signal instead.
// The constant value differs between ESP32 generations:
//   ESP32-C3 (IDF 5.x / esp32c3/soc/gpio_sig_map.h):
//     GPIO_MATRIX_CONST_ONE_INPUT = 0x1F
//   Original ESP32 / S2 / S3 (esp32/soc/gpio_sig_map.h):
//     GPIO_MATRIX_CONST_ONE_INPUT = 0x38
// =====================================================================
#if defined(PLATFORM_ESP32_C3)
static constexpr uint32_t BRIDGE_MATRIX_CONST_HIGH = 0x1FU;
#else
static constexpr uint32_t BRIDGE_MATRIX_CONST_HIGH = 0x38U;
#endif

// =====================================================================
// Required stubs referenced by libraries but unused in bridge firmware
// =====================================================================

// Logging output stream (referenced by logging.h / devs)
Stream *BackpackOrLogStrm = nullptr;

// PWM-serial flag (declared extern in targets.h when TARGET_RX is set)
bool pwmSerialDefined = false;

// =====================================================================
// HandsetConnector – EdgeTX S.PORT, half-duplex CRSF, UART0
// =====================================================================
class HandsetConnector final : public CRSFConnector
{
public:
    void begin()
    {
        // Register devices reachable via the handset port so the router
        // can direct replies back through this connector.
        addDevice(CRSF_ADDRESS_RADIO_TRANSMITTER);
        addDevice(CRSF_ADDRESS_ELRS_LUA);

        // Open UART0 with the same GPIO for TX and RX.  Arduino will
        // accept this; we take over the GPIO direction ourselves below.
        Serial.end();
        Serial.begin(HANDSET_BAUD, SERIAL_8N1,
                     SPORT_PIN, SPORT_PIN,
                     false, 0);
        Serial.setTimeout(0);

        // Start in receive mode.
        setRxMode();

        crsfRouter.addConnector(this);
    }

    void loop()
    {
        // Once the TX FIFO has drained, switch back to receive mode.
        if (_transmitting)
        {
#if defined(PLATFORM_ESP32)
            if (uart_ll_is_tx_idle(UART_LL_GET_HW(0)))
            {
                setRxMode();
            }
#else
            setRxMode();
#endif
            // Do not read incoming bytes until the TX phase is complete.
            return;
        }

        while (Serial.available())
        {
            _parser.processByte(this, static_cast<uint8_t>(Serial.read()));
        }
    }

    /** Send a CRSF frame to EdgeTX over the half-duplex line. */
    void forwardMessage(const crsf_header_t *message) override
    {
        // Drop the frame if we are still in the middle of a previous
        // transmission to avoid corrupting the half-duplex bus.
        if (_transmitting) return;
#if defined(PLATFORM_ESP32)
        if (!uart_ll_is_tx_idle(UART_LL_GET_HW(0))) return;
#endif
        const uint8_t frameLen =
            message->frame_size + CRSF_FRAME_NOT_COUNTED_BYTES;

        setTxMode();
        Serial.write(reinterpret_cast<const uint8_t *>(message), frameLen);
        // setRxMode() is called in loop() once the FIFO drains.
    }

private:
    CRSFParser _parser;
    bool       _transmitting = false;

    /** Switch the shared GPIO to receive (input) mode. */
    void setRxMode()
    {
#if defined(PLATFORM_ESP32)
        gpio_set_direction(static_cast<gpio_num_t>(SPORT_PIN), GPIO_MODE_INPUT);
        gpio_matrix_in(static_cast<gpio_num_t>(SPORT_PIN), U0RXD_IN_IDX, false);
        gpio_pullup_en(static_cast<gpio_num_t>(SPORT_PIN));
        gpio_pulldown_dis(static_cast<gpio_num_t>(SPORT_PIN));
#endif
        _transmitting = false;
    }

    /** Switch the shared GPIO to transmit (output) mode. */
    void setTxMode()
    {
#if defined(PLATFORM_ESP32)
        // Drive the pin HIGH (non-inverted UART idle state).
        gpio_set_level(static_cast<gpio_num_t>(SPORT_PIN), 1);
        gpio_set_direction(static_cast<gpio_num_t>(SPORT_PIN),
                           GPIO_MODE_OUTPUT);
        // Detach the physical pin from the UART0 RX input by wiring the
        // internal "constant HIGH" signal to the RX matrix slot instead.
        gpio_matrix_in(BRIDGE_MATRIX_CONST_HIGH, U0RXD_IN_IDX, false);
        // Route UART0 TX output to the physical pin (non-inverted).
        gpio_matrix_out(static_cast<gpio_num_t>(SPORT_PIN),
                        U0TXD_OUT_IDX, false, false);
#endif
        _transmitting = true;
    }
};

// =====================================================================
// FCConnector – Betaflight / INAV, full-duplex CRSF, UART1
// =====================================================================
class FCConnector final : public CRSFConnector
{
public:
    void begin()
    {
        addDevice(CRSF_ADDRESS_FLIGHT_CONTROLLER);
        Serial1.begin(FC_BAUD, SERIAL_8N1, FC_RX_PIN, FC_TX_PIN, false);
        crsfRouter.addConnector(this);
    }

    void loop()
    {
        while (Serial1.available())
        {
            _parser.processByte(this,
                                static_cast<uint8_t>(Serial1.read()));
        }
    }

    /** Send a CRSF frame to the flight controller. */
    void forwardMessage(const crsf_header_t *message) override
    {
        const uint8_t frameLen =
            message->frame_size + CRSF_FRAME_NOT_COUNTED_BYTES;
        Serial1.write(reinterpret_cast<const uint8_t *>(message), frameLen);
    }

private:
    CRSFParser _parser;
};

// =====================================================================
// BridgeEndpoint – CRSF device-discovery endpoint
//
// Presents itself with address CRSF_ADDRESS_CRSF_RECEIVER (0xEC) so
// EdgeTX can find it via a DEVICE_PING broadcast and enumerate it in
// the ExpressLRS Lua configuration script.
// =====================================================================
class BridgeEndpoint final : public CRSFEndpoint
{
public:
    BridgeEndpoint()
        : CRSFEndpoint(CRSF_ADDRESS_CRSF_RECEIVER) {}

    void handleMessage(const crsf_header_t *message) override
    {
        const auto ext =
            reinterpret_cast<const crsf_ext_header_t *>(message);

        if (message->type == CRSF_FRAMETYPE_DEVICE_PING    ||
            message->type == CRSF_FRAMETYPE_PARAMETER_READ ||
            message->type == CRSF_FRAMETYPE_PARAMETER_WRITE)
        {
            // The ELRS Lua script (running inside EdgeTX) sends DEVICE_PING
            // frames with orig_addr = CRSF_ADDRESS_ELRS_LUA (0xEF) instead
            // of CRSF_ADDRESS_RADIO_TRANSMITTER (0xEA).  The router uses
            // orig_addr to decide which connector to route the reply through,
            // so we remap the Lua address to the radio-transmitter address to
            // ensure replies are delivered back to EdgeTX via the handset port.
            const crsf_addr_e origin =
                (ext->orig_addr == CRSF_ADDRESS_ELRS_LUA)
                    ? CRSF_ADDRESS_RADIO_TRANSMITTER
                    : ext->orig_addr;

            parameterUpdateReq(origin, false,
                               ext->type,
                               ext->payload[0],
                               ext->payload[1]);
        }
    }
};

// =====================================================================
// Global instances
// =====================================================================
static HandsetConnector handsetConnector;
static FCConnector      fcConnector;
static BridgeEndpoint   bridgeEndpoint;

// =====================================================================
// Arduino entry points
// =====================================================================

void setup()
{
    // Initialise firmware options (populates device_name, version, etc.).
    // A missing hardware.json is normal for this target; ignore the return
    // value because all pins are hardcoded in CRSF_Bridge.h.
    options_init();

    // Override device name to identify this as the CRSF Bridge.
    // ELRSOPTS_DEVICENAME_SIZE is 16 chars; "CRSF Bridge" (11) fits safely.
    strncpy(device_name, "CRSF Bridge", ELRSOPTS_DEVICENAME_SIZE);
    device_name[ELRSOPTS_DEVICENAME_SIZE] = '\0';

    // Set up EdgeTX S.PORT connection (half-duplex, UART0, pin 1).
    handsetConnector.begin();

    // Set up flight-controller connection (full-duplex, UART1, pins 20/21).
    fcConnector.begin();

    // Register bridge endpoint so the router can handle DEVICE_PING.
    crsfRouter.addEndpoint(&bridgeEndpoint);
}

void loop()
{
    // Process incoming bytes from both serial ports and route CRSF frames.
    handsetConnector.loop();
    fcConnector.loop();
}
