#include <Arduino.h>
#include <TLA2024.h>
#include <Wire.h>

TLA2024 adc = TLA2024(&Wire1);

uint8_t channelToRead = 2;

void setup() {

    Serial.begin(115200);
    Serial.println("Starting ADC...");

    Wire1.begin(23, 18);

    if (adc.begin()) {
        Serial.println("Device is init-ed");
    }
    else {
        Serial.println("Device is not init-ed. Continue anyway...");
    }

//    adc.setMuxConfig(TLA2024::MUX_AIN2_GND);

    adc.setFullScaleRange(TLA2024::FSR_2_048V);
    adc.setDataRate(TLA2024::DR_3300SPS);
    adc.setOperatingMode(TLA2024::OP_SINGLE);
}

void loop() {

//    float val = adc.analogRead(); //use this in conjunction with adc.setMuxConfig() method;

    float val = adc.analogRead(channelToRead);
    Serial.printf("Channel %d, ADC reading: %.3f\n", channelToRead, val);

//    long converted = map((long)val, -2048, 2047, 0, 2048 );
//    float convF = 2*converted*1.0f/1000;
//    Serial.printf("V = %.3f\n", convF);

    Serial.printf("Voltage = %.3fV\n", adc.voltageRead(channelToRead));
    Serial.println();

    delay(1000);
}