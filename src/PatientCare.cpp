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
//#include "ESPectro_Neopixel.h"
//#include "DCX_Neopixel_Animation.h"

ADC_MODE(ADC_VCC);

ESPectro board;

//ESPectro_Neopixel_Default neopixel;

//RgbColor pulseColor(HtmlColor(0x7f0000));
//DCX_Neopixel_PulseAnimation pulseAnimation(neopixel, pulseColor);

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

//    neopixel.Begin();
//    neopixel.Show();
//    pulseAnimation.setPulsingInterval(400);


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

            //uiSvc.setStatusInfo("Published: " + String(packetNum));
        }
        else {
            //uiSvc.setStatusInfo("Not Published!");
        }
    });
}

void processDetectedActivity(boolean isAct, float x, float y, float z) {

//    pulseAnimation.start();
    board.turnOnLED();
//    digitalWrite(10, 1);

    float acc = sqrtf((powf(x, 2) + powf(y, 2) + powf(z, 2)));

    char xStr[9], yStr[9], zStr[9], accStr[9];
    dtostrf(x, 6, 2, xStr);
    dtostrf(y, 6, 2, yStr);
    dtostrf(z, 6, 2, zStr);
    dtostrf(acc, 6, 2, accStr);

    //SHT31
    char tStr[9], hStr[9];
    dtostrf(hdc1000->getTemp(), 3, 2, tStr);
    dtostrf(hdc1000->getHumi(), 3, 2, hStr);

    Serial.print("Temperature: ");
    Serial.print(hdc1000->getTemp());
    Serial.print("C, Humidity: ");
    Serial.print(hdc1000->getHumi());
    Serial.println("%");


    uint16_t battMV = ESP.getVcc();
    char payloadStr[100];
    //sprintf(payloadStr, "T1=%s&P=%s&H1=%s&AX=%s&AY=%s&AZ=%s&GX=%s&GY=%s&GZ=%s&MX=%s&MY=%s&MZ=%s&MH=%s&T2=%s&H2=%s&L=%s&S=%d&G=%d&B=%d",
    sprintf(payloadStr, "T=%s&H=%s&ACC=%s&AX=%s&AY=%s&AZ=%s&ACT=%d&B=%d&ID=%s",
            tStr, hStr,
            accStr,
            xStr, yStr, zStr,
            isAct,
            battMV,
            "beastmaster-01"
    );

    String payload = String(payloadStr);
    payload.replace(" ", "");

    //loraSvc.publish(payloadStr, false, true);
    loraSvc.publish(payload, false, true);
}

void loop() {

    //pulseAnimation.loop();

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
                //pulseAnimation.end();
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

//#include <SparkFunLSM9DS1.h>
//
//LSM9DS1 imu;
//const int INT1_PIN_THS = 10;
////3; // INT1 pin to D3 - will be attached to gyro
//const int INT2_PIN_DRDY = 12;
////4; // INT2 pin to D4 - attached to accel
//const int INTM_PIN_THS = 13;
////5;  // INTM_PIN_THS pin to D5
//const int RDYM_PIN = 14;//6;  // RDY pin to D6
//
//// Variable to keep track of when we print sensor readings:
//unsigned long lastPrint = 0;
//#define PRINT_CALCULATED
////#define PRINT_RAW
//
//// configureIMU sets up our LSM9DS1 interface, sensor scales
//// and sample rates.
//uint16_t configureIMU() {
//    // Set up Device Mode (I2C) and I2C addresses:
//    imu.settings.device.commInterface = IMU_MODE_I2C;
//    imu.settings.device.agAddress = LSM9DS1_AG_ADDR(1);
//    imu.settings.device.mAddress = LSM9DS1_M_ADDR(1);
//
//    // gyro.latchInterrupt controls the latching of the
//    // gyro and accelerometer interrupts (INT1 and INT2).
//    // false = no latching
//    imu.settings.gyro.latchInterrupt = false;
//
//    // Set gyroscope scale to +/-245 dps:
//    imu.settings.gyro.scale = 245;
//    // Set gyroscope (and accel) sample rate to 14.9 Hz
//    imu.settings.gyro.sampleRate = 1;
//    // Set accelerometer scale to +/-2g
//    imu.settings.accel.scale = 2;
//    // Set magnetometer scale to +/- 4g
//    imu.settings.mag.scale = 4;
//    // Set magnetometer sample rate to 0.625 Hz
//    imu.settings.mag.sampleRate = 0;
//
//    // Call imu.begin() to initialize the sensor and instill
//    // it with our new settings.
//    return imu.begin();
//}
//
//void configureLSM9DS1Interrupts() {
//    /////////////////////////////////////////////
//    // Configure INT1 - Gyro & Accel Threshold //
//    /////////////////////////////////////////////
//    // For more information on setting gyro interrupt, threshold,
//    // and configuring the intterup, see the datasheet.
//    // We'll configure INT_GEN_CFG_G, INT_GEN_THS_??_G,
//    // INT_GEN_DUR_G, and INT1_CTRL.
//    // 1. Configure the gyro interrupt generator:
//    //  - ZHIE_G: Z-axis high event (more can be or'd together)
//    //  - false: and/or (false = OR) (not applicable)
//    //  - false: latch interrupt (false = not latched)
//    imu.configGyroInt(ZHIE_G, false, false);
//    // 2. Configure the gyro threshold
//    //   - 500: Threshold (raw value from gyro)
//    //   - Z_AXIS: Z-axis threshold
//    //   - 10: duration (based on ODR)
//    //   - true: wait (wait duration before interrupt goes low)
//    imu.configGyroThs(500, Z_AXIS, 10, true);
//    // 3. Configure accelerometer interrupt generator:
//    //   - XHIE_XL: x-axis high event
//    //     More axis events can be or'd together
//    //   - false: OR interrupts (N/A, since we only have 1)
//    //imu.configAccelInt(ZHIE_XL, false);
//    imu.configAccelInt(XHIE_XL, false);
//    // 4. Configure accelerometer threshold:
//    //   - 20: Threshold (raw value from accel)
//    //     Multiply this value by 128 to get threshold value.
//    //     (20 = 2600 raw accel value)
//    //   - X_AXIS: Write to X-axis threshold
//    //   - 10: duration (based on ODR)
//    //   - false: wait (wait [duration] before interrupt goes low)
//    imu.configAccelThs(20, X_AXIS, 1, false);
//    imu.configAccelThs(10, Z_AXIS, 1, false);
//    // 5. Configure INT1 - assign it to gyro interrupt
//    //   - XG_INT1: Says we're configuring INT1
//    //   - INT1_IG_G | INT1_IG_XL: Sets interrupt source to
//    //     both gyro interrupt and accel
//    //   - INT_ACTIVE_LOW: Sets interrupt to active low.
//    //         (Can otherwise be set to INT_ACTIVE_HIGH.)
//    //   - INT_PUSH_PULL: Sets interrupt to a push-pull.
//    //         (Can otherwise be set to INT_OPEN_DRAIN.)
//    //imu.configInt(XG_INT1, INT1_IG_G | INT_IG_XL, INT_ACTIVE_LOW, INT_OPEN_DRAIN);
//    imu.configInt(XG_INT1, INT_IG_XL, INT_ACTIVE_LOW, INT_PUSH_PULL);
//
//    ////////////////////////////////////////////////
//    // Configure INT2 - Gyro and Accel Data Ready //
//    ////////////////////////////////////////////////
//    // Configure interrupt 2 to fire whenever new accelerometer
//    // or gyroscope data is available.
//    // Note XG_INT2 means configuring interrupt 2.
//    // INT_DRDY_XL is OR'd with INT_DRDY_G
//    imu.configInt(XG_INT2, INT_DRDY_XL | INT_DRDY_G, INT_ACTIVE_LOW, INT_PUSH_PULL);
//
//    //////////////////////////////////////
//    // Configure Magnetometer Interrupt //
//    //////////////////////////////////////
//    // 1. Configure magnetometer interrupt:
//    //   - XIEN: axis to be monitored. Can be an or'd combination
//    //     of XIEN, YIEN, or ZIEN.
//    //   - INT_ACTIVE_LOW: Interrupt goes low when active.
//    //   - true: Latch interrupt
//    imu.configMagInt(XIEN, INT_ACTIVE_LOW, true);
//    // 2. Configure magnetometer threshold.
//    //   There's only one threshold value for all 3 mag axes.
//    //   This is the raw mag value that must be exceeded to
//    //   generate an interrupt.
//    imu.configMagThs(10000);
//
//}
//
//void setup() {
//
//    Serial.begin(115200);
//    while (!Serial);
//
//    // Set up our Arduino pins connected to interrupts.
//    // We configured all of these interrupts in the LSM9DS1
//    // to be active-low.
//    pinMode(INT2_PIN_DRDY, INPUT_PULLUP);
//    pinMode(INT1_PIN_THS, INPUT_PULLUP);
//    pinMode(INTM_PIN_THS, INPUT_PULLUP);
//    // The magnetometer DRDY pin (RDY) is not configurable.
//    // It is active high and always turned on.
//    pinMode(RDYM_PIN, INPUT);
//
//    // Turn on the IMU with configureIMU() (defined above)
//    // check the return status of imu.begin() to make sure
//    // it's connected.
//    uint16_t status = configureIMU();
//    if (!status) {
//        Serial.print("Failed to connect to IMU: 0x");
//        Serial.println(status, HEX);
//        while (!status) {
//            delay(100);
//            status = configureIMU();
//        }
//    }
//
//    // After turning the IMU on, configure the interrupts:
//    configureLSM9DS1Interrupts();
//}
//
//void printAccel() {
//    // To read from the accelerometer, you must first call the
//    // readAccel() function. When this exits, it'll update the
//    // ax, ay, and az variables with the most current data.
//    imu.readAccel();
//
//    // Now we can use the ax, ay, and az variables as we please.
//    // Either print them as raw ADC values, or calculated in g's.
//    Serial.print("A: ");
//#ifdef PRINT_CALCULATED
//    // If you want to print calculated values, you can use the
//    // calcAccel helper function to convert a raw ADC value to
//    // g's. Give the function the value that you want to convert.
//    Serial.print(imu.calcAccel(imu.ax), 2);
//    Serial.print(", ");
//    Serial.print(imu.calcAccel(imu.ay), 2);
//    Serial.print(", ");
//    Serial.print(imu.calcAccel(imu.az), 2);
//    Serial.println(" g");
//
//#elif defined PRINT_RAW
//    Serial.print(imu.ax);
//    Serial.print(", ");
//    Serial.print(imu.ay);
//    Serial.print(", ");
//    Serial.println(imu.az);
//#endif
//
//}
//
//// Print the last read accelerometer, gyro, and mag values:
//void printStats()
//{
//    Serial.println();
//    Serial.print("A: ");
//    Serial.print(imu.ax); Serial.print(", ");
//    Serial.print(imu.ay); Serial.print(", ");
//    Serial.println(imu.az);
//    Serial.print("G: ");
//    Serial.print(imu.gx); Serial.print(", ");
//    Serial.print(imu.gy); Serial.print(", ");
//    Serial.println(imu.gz);
//    Serial.print("M: ");
//    Serial.print(imu.mx); Serial.print(", ");
//    Serial.print(imu.my); Serial.print(", ");
//    Serial.println(imu.mz);
//}
//
//void loop() {
//    // Every 1 second (1000 ms), print the last sensor values
//    // that were read:
//    if (millis() > (lastPrint + 1000))
//    {
//        printStats();
//        lastPrint = millis();
//    }
//
//    // INT2 fires when new accelerometer or gyroscope data
//    // is available.
//    // It's configured to be active LOW:
//    if (digitalRead(INT2_PIN_DRDY) == LOW)
//    {
//        // We don't know if accelerometer or gyroscope data is
//        // available.
//        // Use accelAvailable and gyroAvailable to check, then
//        // read from those sensors if it's new data.
//        if (imu.accelAvailable())
//            imu.readAccel();
//        if (imu.gyroAvailable())
//            imu.readGyro();
//    }
//
//    // INT1 fires when our gyro or accelerometer thresholds
//    // are exceeded.
//    // It's configured to be active LOW:
//    if (digitalRead(INT1_PIN_THS) == LOW)
//    {
//        // Let's keep track of how long the interrupt is active.
//        // We turned off latching, so this pin will stay low
//        // as long as the threshold is exceeded:
//        unsigned long durationStart = millis();
//
//        // Call getGyroIntSrc() and getAccelIntSrc() to determine
//        // if the gyro or accel generated the input (and why).
//        Serial.println("\tINT1 Active!");
//        Serial.print("\t\tGyro int: 0x");
//        Serial.println(imu.getGyroIntSrc(), HEX);
//        Serial.print("\t\tAccel int: 0x");
//        Serial.println(imu.getAccelIntSrc(), HEX);
//
//        // While the interrupt remains active, loop:
//        while (digitalRead(INT1_PIN_THS) == LOW)
//        {
//            //imu.getGyroIntSrc();
//            //imu.getAccelIntSrc();
//            delay(1);
//        }
//        Serial.print("\tINT1 Duration: ");
//        Serial.println(millis() - durationStart);
//
//        printAccel();
//    }
//
//    // INTM fires when the magnetometer exceeds our set
//    // threshold.
//    // It's configured to be active LOW:
////  if (digitalRead(INTM_PIN_THS) == LOW)
////  {
////    // Once again, we'll keep track of how line the interrupt
////    // stays low
////    unsigned long durationStart = millis();
////
////    // Read getMagIntSrc() to see why the interrupt was
////    // generated.
////    Serial.print("\t\tMag int: 0x");
////    Serial.println(imu.getMagIntSrc(), HEX);
////
////    // Loop until the interrupt stops firing
////    while (digitalRead(INTM_PIN_THS) == LOW)
////    {
////    }
////    Serial.print("\t\tINTM_PIN_THS Duration: ");
////    Serial.println(millis() - durationStart);
////  }
//
//    // RDY goes HIGH when new magnetometer is available.
//    // AFAICT the active high/low isn't configurable:
//    if (digitalRead(RDYM_PIN) == HIGH)
//    {
//        if (imu.magAvailable())
//        {
//            imu.readMag();
//        }
//    }
//}