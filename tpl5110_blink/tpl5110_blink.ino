
/*
 * initial experimentaiton with the TPL5110 on the Mini Lora Node
 * (see https://github.com/hallard/Mini-LoRa)
 *
 * some code based on the Rocket Scream Low-Power library examples
 * some code from the example here: https://playground.arduino.cc/Main/PinChangeInterrupt/
 */

// get it from here -> https://github.com/rocketscream/Low-Power
// or using the Arduino Library Manager
#include "LowPower.h"

// I'm using this LED: Kingbright WP154A4SEJ3VBDZGW/CA, and green/blue are inverted compared to the labels on the PCB
#define LEDR 9
#define LEDB 6
#define LEDG 5

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


void setup() {
    // serial output
    Serial.begin(9600);
    Serial.println(F("Starting"));
    
    // RGB LED
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);

    // TPL5110
    pinMode(TPLDONE, OUTPUT);
    digitalWrite(TPLDONE, LOW);
    pinMode(TPLDRVN, INPUT);
}

void loop() {

    // we "blink" each color in turn
    digitalWrite(LEDR, LOW);
    delay(1000);
    digitalWrite(LEDR, HIGH);
    delay(1000);
    digitalWrite(LEDG, LOW);
    delay(1000);
    digitalWrite(LEDG, HIGH);
    delay(1000);
    digitalWrite(LEDB, LOW);
    delay(1000);
    digitalWrite(LEDB, HIGH);
    delay(1000);

    // tell the TPL5110 to start "counting"
    digitalWrite(TPLDONE, LOW);
    digitalWrite(TPLDONE, HIGH);
    delay(50);
    // this is so we can "wake up" later
    pciSetup(TPLDRVN);
    // now we sleep
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    // we land here once we "wake up"

}

