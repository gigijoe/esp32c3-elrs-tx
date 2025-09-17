#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <stdint.h>

#if 0

#define EXTERNAL_NUM_INTERRUPTS 22
#define NUM_DIGITAL_PINS        22
#define NUM_ANALOG_INPUTS       6

#define analogInputToDigitalPin(p)  (((p)<NUM_ANALOG_INPUTS)?(analogChannelToDigitalPin(p)):-1)
#define digitalPinToInterrupt(p)    (((p)<NUM_DIGITAL_PINS)?(p):-1)
#define digitalPinHasPWM(p)         (p < EXTERNAL_NUM_INTERRUPTS)

#endif

static const uint8_t LED_BUILTIN = 8;
#define BUILTIN_LED LED_BUILTIN // backward compatibility
#define LED_BUILTIN LED_BUILTIN

static const uint8_t BOOT_BUILTIN = 9; // boot button
#define BOOT_BUILTIN BOOT_BUILTIN

static const uint8_t D19 = 19; // USBD_P
static const uint8_t D18 = 18; // USBD_N
// GPIO12~17 NOT recommanded for other uses

static const uint8_t LED = 10; // Blue LED
static const uint8_t SDA = 8;
static const uint8_t SCL = 9;

static const uint8_t SS    = 7; // SD
static const uint8_t MOSI  = 6;
static const uint8_t MISO  = 2;
static const uint8_t SCK   = 5;

static const uint8_t ID0 = 0;
static const uint8_t ID1 = 1;
static const uint8_t ID2 = 2;
static const uint8_t ID3 = 3; 
static const uint8_t A4 = 4;
//static const uint8_t A5 = 5;

static const uint8_t PPS = 10; // GNSS PPS 

#define U2TXD_OUT_IDX 0

#define FIRMWARE_VERSION "v0.0.1"

#define RCGPS_F3X_V2
#undef RCGPS_F3X_MS5611
#undef RCGPS_F3X_ASPDSNR

#undef DEBUG_SPORT

#endif /* Pins_Arduino_h */
