/*******************************************************************************
 * This is a working example of using the Mini-LoRa PCB with TTN
 * (see https://github.com/hallard/Mini-LoRa)
 *
 * It is "low-power-ready" - if you follow the instructions in the
 * README, power usage is less than 40 uA when in sleep mode.
 *
 * Required hardware: Mini-LoRa PCB (see above), BME280 break-out
 * board (I used one from Adafruit, see http://adafru.it/2652, connected
 * via I2C).
 * 
 * The code below is based on the "ttn-otaa" example that comes with the
 * Arduino LMIC library, see the fork I maintain here:
 * https://github.com/crox-net/arduino-lmic
 * 
 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Fill the obtained information in ttn_secrets_template.h and rename it
 * to ttn_secrets.h
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * some code based on the Rocket Scream Low-Power library examples
 * some code from the example here: https://playground.arduino.cc/Main/PinChangeInterrupt/
 *
 * In addition to arduino-lmic (see above), you need the following libraries:
 *   - https://github.com/rocketscream/Low-Power
 *   - https://github.com/adafruit/Adafruit_BME280_Library
 *   - https://github.com/ElectronicCats/CayenneLPP
 *
 * You can install them directly from the Arduino environment
 * (Tools -> Manage Libraries)
 *
 * As per the original "license":
 *
 * Permission is hereby granted, free of charge, to anyone obtaining a copy
 * of this document and accompanying files, to do whatever they want with
 * them without any restriction, including, but not limited to, copying,
 * modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * If you create something that might be useful to others, please consider
 * sharing!
 *
 *******************************************************************************/

#include "LowPower.h"

// this reduces sketch size
#define DISABLE_BEACONS 1

// set these to "1" to get serial output + led feedback,
// set to 0 to save power when running on battery
#define DEBUG 0
#define LEDON 0

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <CayenneLPP.h>

// allocate space for max. 12 bytes, see table here to calculate how much you need:
// https://developers.mydevices.com/cayenne/docs/lora/#lora-cayenne-low-power-payload
CayenneLPP lpp(12);

Adafruit_BME280 bme;

// see ttn_secrets_template.h
#include "ttn_secrets.h"
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}

static osjob_t sendjob;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 15,
    .dio = {2, 7, 8},
};


// RGB LED
#define LEDR 9
#define LEDG 6
#define LEDB 5

// TPL5110
#define TPLDONE 15
#define TPLDRVN 16

// Install Pin change interrupt for a pin, can be called multiple times
void pciSetup(byte pin) {
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

// handle pin change interrupt for A0 to A5 here
ISR (PCINT1_vect) {
   // nothing
}

void onEvent (ev_t ev) {
    #if DEBUG
    Serial.print(os_getTime());
    Serial.print(": ");
    #endif
    switch(ev) {
        #if DEBUG
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        #endif
        case EV_JOINED:
            #if DEBUG
            Serial.println(F("EV_JOINED"));
            #endif
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            break;
        #if DEBUG
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        #endif
        case EV_TXCOMPLETE:
            #if DEBUG
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            #endif
            if (LMIC.txrxFlags & TXRX_ACK) {
              #if DEBUG
              Serial.println(F("Received ack"));
              #endif
            }
            if (LMIC.dataLen) {
              #if DEBUG
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
              #endif
            }
            #if LEDON
            // success -> we blink the green led
            digitalWrite(LEDB, LOW);
            delay(300);
            digitalWrite(LEDB, HIGH);
            #endif
            // tell the TPL5110 to start "counting"
            digitalWrite(TPLDONE, LOW);
            digitalWrite(TPLDONE, HIGH);
            delay(50);
            // this is so we can "wake up" later
            pciSetup(TPLDRVN);
            // now we sleep
            LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
            // we land here once we "wake up"
            detachInterrupt(0);
            #if LEDON
            // wake up -> we blink the red led
            digitalWrite(LEDR, LOW);
            delay(300);
            digitalWrite(LEDR, HIGH);
            #endif
            // Schedule next transmission asap
            os_setTimedCallback(&sendjob, os_getTime(), do_send);
            break;
        #if DEBUG
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        #endif
        default:
            #if DEBUG
            Serial.println(F("Unknown event"));
            #endif
            break;
    }
}

void read_sensor() {

    // this puts the sensor to sleep when not measuring
    // see the datasheet for details
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );
    bme.takeForcedMeasurement();
    
    float temp = bme.readTemperature();
    float hdty = bme.readHumidity();
    float phpa = (bme.readPressure() / 100.0F);

    #if DEBUG
    Serial.print("Temperature = ");
    Serial.print(temp);
    Serial.println(" *C");

    Serial.print("Humidity = ");
    Serial.print(hdty);
    Serial.println(" %");

    Serial.print("Pressure = ");
    Serial.print(phpa);
    Serial.println(" hPa");
    #endif
    
    lpp.reset();
    lpp.addTemperature(1, temp);
    lpp.addRelativeHumidity(2, hdty);
    lpp.addBarometricPressure(3, phpa);
    
    #if DEBUG
    Serial.println("lpp buffer is now:");
    uint8_t* buf = lpp.getBuffer();
    for (int i = 0; i < lpp.getSize(); i++) {
      Serial.print(buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    #endif
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        #if DEBUG
        Serial.println(F("OP_TXRXPEND, not sending"));
        #endif
    } else {
        // Prepare upstream data transmission at the next possible time.
        read_sensor();
        LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
        #if DEBUG
        Serial.println(F("Packet queued"));
        #endif
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {

    Serial.begin(9600);
    Serial.println(F("Starting"));

    unsigned status;    
    status = bme.begin();
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }
    // this puts the sensor to sleep when not measuring
    // see the datasheet for details
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X8, // temperature
                    Adafruit_BME280::SAMPLING_X8, // pressure
                    Adafruit_BME280::SAMPLING_X8, // humidity
                    Adafruit_BME280::FILTER_OFF   );

    // RGB LED
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);
    digitalWrite(LEDB, LOW);
    delay(300);
    digitalWrite(LEDB, HIGH);
    delay(200);
    digitalWrite(LEDB, LOW);
    delay(300);
    digitalWrite(LEDB, HIGH);

    // TPL5110
    pinMode(TPLDONE, OUTPUT);
    digitalWrite(TPLDONE, LOW);
    pinMode(TPLDRVN, INPUT);

    // LMIC init
    os_init();

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // add some tolerance...
    LMIC_setClockError (MAX_CLOCK_ERROR * 10 / 100);

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}

