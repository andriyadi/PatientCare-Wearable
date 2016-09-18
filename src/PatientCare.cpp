//
// Created by Andri Yadi on 9/15/16.
//

#include "Arduino.h"

#include <Wire.h>
#include <SPI.h>
#include "MPU6050.h"
#include <ESP8266WiFi.h>
#include "DCX_AppSetting.h"
#include "DCX_LoRaService.h"
#include "HDC1000.h"
#include "ESPectro.h"

ADC_MODE(ADC_VCC);

ESPectro board;

MPU6050 *mpu_;
DCX_LoRaService loraSvc(AppSetting);
HDC1000 *hdc1000;

unsigned long mpuActivityDetectedStartMillis_ = 0, mpuMotionDetectedStartMillis_ = 0;

void setup() {

    Serial.begin(115200);
    while(!Serial);

    delay(1000);

    Wire.begin();

    WiFi.forceSleepBegin();

    AppSetting.load();
    AppSetting.debugPrintTo(Serial);

//    pinMode(10, OUTPUT);
//    digitalWrite(10, 1);

    mpu_ = new MPU6050();

    if (!mpu_->begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_16G))
    {
        Serial.println(F("Could not find a valid MPU6050 sensor, check wiring!"));

        delete mpu_;
        mpu_ = NULL;
    }
    else {
        Serial.println(F("MPU6050 init success"));

        mpu_->setAccelPowerOnDelay(MPU6050_DELAY_3MS);

        mpu_->setIntFreeFallEnabled(false);
        mpu_->setIntZeroMotionEnabled(false);
        mpu_->setIntMotionEnabled(false);

        mpu_->setDHPFMode(MPU6050_DHPF_5HZ);

//		mpu_->setMotionDetectionThreshold(2);
//		mpu_->setMotionDetectionDuration(5);

        mpu_->setMotionDetectionThreshold(5);
        mpu_->setMotionDetectionDuration(100);

        mpu_->setZeroMotionDetectionThreshold(5);
        mpu_->setZeroMotionDetectionDuration(50);
    }

    hdc1000 = new HDC1000();
    if (hdc1000->begin()) {
        Serial.println(F("HDC1000 is OK"));
    }
    else {
        delete hdc1000;
        hdc1000 = NULL;
    }

    loraSvc.begin();
    loraSvc.setMessagePrefix("\\@");

    loraSvc.onMessageAcknowledged([](bool success, String origMsg, uint8_t packetNum) {
        if (success) {
            Serial.print("ACK packet number: ");
            Serial.println(packetNum);
        }
        else {
            Serial.print(F("Not Published!"));
        }
    });
}

void processDetectedActivity(boolean isAct, float x, float y, float z) {

    board.turnOnLED();
//    digitalWrite(10, 1);

    float acc = sqrtf((powf(x, 2) + powf(y, 2) + powf(z, 2)));

    char xStr[9], yStr[9], zStr[9], accStr[9];
    dtostrf(x, 6, 2, xStr);
    dtostrf(y, 6, 2, yStr);
    dtostrf(z, 6, 2, zStr);
    dtostrf(acc, 6, 2, accStr);

    //HDC1000
    char tStr[9] = "0\0", hStr[9] = "0\0";
    if (hdc1000 != NULL) {
        dtostrf(hdc1000->getTemp(), 3, 2, tStr);
        dtostrf(hdc1000->getHumi(), 3, 2, hStr);

        Serial.print("Temperature: ");
        Serial.print(hdc1000->getTemp());
        Serial.print("C, Humidity: ");
        Serial.print(hdc1000->getHumi());
        Serial.println("%");
    }

    uint16_t battMV = ESP.getVcc();

    char payloadStr[100];
    sprintf(payloadStr, "T=%s&H=%s&ACC=%s&AX=%s&AY=%s&AZ=%s&ACT=%d&B=%d&ID=%s",
            tStr, hStr,
            accStr,
            xStr, yStr, zStr,
            isAct,
            battMV,
            SETTING_DEFAULT_ID
    );

    String payload = String(payloadStr);
    payload.replace(" ", "");

    loraSvc.publish(payload, false, true);
}

void loop() {

    if (mpu_ != NULL) {
        Activites act = mpu_->readActivites();
        if (act.isActivity) {
            if (millis() - mpuActivityDetectedStartMillis_ > 2000) {
                Serial.println(F("Activity detected and processed"));

                Vector vect = mpu_->readRawAccel();
                processDetectedActivity(true, vect.XAxis, vect.YAxis, vect.ZAxis);

                mpuActivityDetectedStartMillis_ = millis();
            }
        }
        else {

            if (millis() - mpuMotionDetectedStartMillis_ > 7000) {

//               digitalWrite(10, 0);
                board.turnOffLED();

                Serial.println(F("Sending motion"));

                Vector vect = mpu_->readRawAccel();
                processDetectedActivity(false, vect.XAxis, vect.YAxis, vect.ZAxis);

                mpuMotionDetectedStartMillis_ = millis();
            }
        }
    }
    delay(1);
}
