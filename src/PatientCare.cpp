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
#include "PulseSensor.h"

//ADC_MODE(ADC_VCC);

#define DEVICE_ID   "PatientCareWearable-01"
#define MOTION_INT_PIN      10
#define LORA_MESSAGE_PREFIX     "\\%"  //"\\#"

const uint16_t HEARTRATE_READING_DURATION = 5000;
const uint16_t HEARTRATE_READING_INTERVAL = 10000;
const uint16_t TELEMETRY_PUBLISH_INTERVAL = 20000;

ESPectro board;

PulseSensor pulseSensor;

MPU6050 *mpu_;
DCX_LoRaService loraSvc(AppSetting);
HDC1000 *hdc1000;

int lastBPM = 0;
volatile boolean fallDetected = false;

unsigned long telemetryPublishMillis_ = 0;

unsigned long heartBeatSensingStartMillis_ = 0, heartBeatSensingStopMillis_ = 0;

void publishTelemetry();

void onMotionDetected() {
    Serial.println("Fall detected...");
    fallDetected = true;
}

void setup() {

    Serial.begin(115200);
    while(!Serial);

    delay(1000);

    Wire.begin();

    //Kill the WiFi
    WiFi.forceSleepBegin();

    AppSetting.load();
    AppSetting.debugPrintTo(Serial);

//    pinMode(10, OUTPUT);
//    digitalWrite(10, 1);

    pinMode(MOTION_INT_PIN, INPUT);
    attachInterrupt(MOTION_INT_PIN, onMotionDetected, RISING);

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

//        mpu_->setIntFreeFallEnabled(false);
//        mpu_->setIntZeroMotionEnabled(false);
//        mpu_->setIntMotionEnabled(false);

        mpu_->setIntFreeFallEnabled(true);
        mpu_->setIntZeroMotionEnabled(false);
        mpu_->setIntMotionEnabled(true);

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
    //loraSvc.setMessagePrefix("\\@");
    loraSvc.setMessagePrefix(LORA_MESSAGE_PREFIX);

    loraSvc.onMessageAcknowledged([](bool success, String origMsg, uint8_t packetNum) {
        if (success) {
            Serial.print("ACK packet number: ");
            Serial.println(packetNum);
        }
        else {
            Serial.print(F("Not Published!"));
        }
    });

    //set callback to actually read the raw value from ADC
    pulseSensor.onReadingRawSignal([](int &raw) {

        raw = analogRead(0);
        //Serial.printf("Raw heart beat value: %d\n", raw);

    });

    pulseSensor.start();
    heartBeatSensingStartMillis_ = millis();
}

void publishTelemetry() {

//    board.turnOnLED();
//    digitalWrite(10, 1);

//    float acc = sqrtf((powf(x, 2) + powf(y, 2) + powf(z, 2)));

    char xStr[9] = "0\0", yStr[9] = "0\0", zStr[9] = "0\0";//, accStr[9];
    boolean isAct = false;

    if (mpu_ != NULL) {
        Activites act = mpu_->readActivites();
        isAct = act.isActivity;

        Vector vect = mpu_->readRawAccel();
        float x = vect.XAxis;
        float y = vect.YAxis;
        float z = vect.ZAxis;

        dtostrf(x, 6, 2, xStr);
        dtostrf(y, 6, 2, yStr);
        dtostrf(z, 6, 2, zStr);
//    dtostrf(acc, 6, 2, accStr);
    }

    if (lastBPM == 0) {
        lastBPM = pulseSensor.getLastBPM();
    }
    else {
        int bpm = pulseSensor.getLastBPM();

        //cleansing...
        if (bpm > lastBPM+20 || bpm < lastBPM-20|| bpm > 160 || bpm < 54) {
            Serial.printf("[Not Valid] BPM: %d\n", bpm);
        }
        else {
            Serial.printf("[Valid] BPM: %d\n", bpm);
            lastBPM = bpm;
        }
    }

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

//    const char *tStr = "23.43";
//    const char *hStr = "78.33";

    uint16_t battMV = 3230;//ESP.getVcc(); //hardcoded for now

    char payloadStr[128];
    //sprintf(payloadStr, "T=%s&H=%s&ACC=%s&AX=%s&AY=%s&AZ=%s&ACT=%d&B=%d&ID=%s",
    sprintf(payloadStr, "TP=%s&H=%s&AX=%s&AY=%s&AZ=%s&HR=%d&FD=%d&B=%d&ID=%s",
            tStr, hStr,
//            accStr,
            xStr, yStr, zStr,
            lastBPM,
            isAct,
            battMV,
            DEVICE_ID
    );

    String payload = String(payloadStr);
    payload.replace(" ", "");

    Serial.println(payload);

    loraSvc.publish(payload, false, true);
}

void loop() {

//    if (mpu_ != NULL) {
//        Activites act = mpu_->readActivites();
//        if (act.isActivity) {
//            if (millis() - mpuActivityDetectedStartMillis_ > 2000) {
//                Serial.println(F("Activity detected and processed"));
//
//                Vector vect = mpu_->readRawAccel();
//                processDetectedActivity(true, vect.XAxis, vect.YAxis, vect.ZAxis);
//
//                mpuActivityDetectedStartMillis_ = millis();
//            }
//        }
//        else {
//
//            if (millis() - mpuMotionDetectedStartMillis_ > 7000) {
//
////               digitalWrite(10, 0);
//                board.turnOffLED();
//
//                Serial.println(F("Sending motion"));
//
//                Vector vect = mpu_->readRawAccel();
//                processDetectedActivity(false, vect.XAxis, vect.YAxis, vect.ZAxis);
//
//                mpuMotionDetectedStartMillis_ = millis();
//            }
//        }
//    }

    unsigned long now = millis();

    //Read heartbeat for 5 seconds
    if ((now - heartBeatSensingStartMillis_ > HEARTRATE_READING_DURATION) && pulseSensor.isStarted()) {
        heartBeatSensingStopMillis_ = now;
        Serial.println(F("Stopping heart beat reading"));
        lastBPM = pulseSensor.getLastBPM();
        pulseSensor.stop();
    }

    //Read heartbeat every 15 seconds
    if ((now - heartBeatSensingStopMillis_ > HEARTRATE_READING_INTERVAL) && !pulseSensor.isStarted()) {
        heartBeatSensingStartMillis_ = now;
        Serial.println(F("Starting heart beat reading"));
        pulseSensor.start();
    }

    if (now - telemetryPublishMillis_ > TELEMETRY_PUBLISH_INTERVAL || fallDetected) {
        fallDetected = false;
        publishTelemetry();
        telemetryPublishMillis_ = now;
    }

    delay(1);
}
