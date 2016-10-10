//
// Created by Andri Yadi on 10/1/16.
//

#ifndef PULSESENSORLION_PULSESENSOR_H
#define PULSESENSORLION_PULSESENSOR_H

#include "Arduino.h"
#include <Ticker.h>
#include <functional>

typedef std::function<void(int&)> PulseSensorRawSignalReadingCallback;
typedef std::function<void(bool)> PulseSensorPulseDetectionCallback;
typedef std::function<void(int)> PulseSensorBPMAvailableCallback;

#define PULSE_SENSOR_SAMPLE_COUNT           10
#define PULSE_SENSOR_READING_INTERVAL_MS    2

class PulseSensor {
public:
    PulseSensor();
    virtual ~PulseSensor();

    void start();
    void stop();
    void processSignal(int signalData);

    int getLastBPM() {
        quantifiedSelf_ = false;
        return BPM_;
    }

    int getLastIBI() {
        return IBI_;
    }

    int getLastRawSignal() {
        return lastRawSignal_;
    }

    bool isBeatDetected() {
        return quantifiedSelf_;
    }

    bool isStarted() {
        return isStarted_;
    }

    void onReadingRawSignal(PulseSensorRawSignalReadingCallback cb);
    int doReadRawSignal();

    void onBPMAvailable(PulseSensorBPMAvailableCallback cb);
    void onPulseDetectionCallback(PulseSensorPulseDetectionCallback cb);

private:

    PulseSensorRawSignalReadingCallback rawSignalReadingCallback_ = NULL;
    PulseSensorBPMAvailableCallback bpmAvailableCallback = NULL;
    PulseSensorPulseDetectionCallback  pulseDetectionCallback = NULL;

    Ticker *readingTicker_ = NULL;
    int lastRawSignal_ = 0;
    bool isStarted_ = false;

    volatile int BPM_;                         // int that holds raw Analog in 0. updated every 2mS
    volatile bool quantifiedSelf_ = false;     // becomes true when Arduoino finds a beat.

    volatile unsigned long IBI_ = 600;                      // int that holds the time interval between beats! Must be seeded!
    volatile bool pulseDetected_ = false;                   // "True" when User's live heartbeat is detected. "False" when not a "live beat".

    volatile int lastRates_[PULSE_SENSOR_SAMPLE_COUNT];                            // array to hold last ten IBI values
    volatile unsigned long sampleCounter_ = 0;              // used to determine pulse timing
    volatile unsigned long lastBeatTime_ = 0;               // used to find IBI
    volatile int peak_ = 512;                               // used to find peak in pulse wave, seeded
    volatile int trough_ = 512;                             // used to find trough in pulse wave, seeded
    volatile int thresh_ = 512;                             // used to find instant moment of heart beat, seeded
    volatile int amplitude_ = 100;                          // used to hold amplitude of pulse waveform, seeded
    volatile bool isFirstBeat_ = true;                      // used to seed rate array so we startup with reasonable BPM
    volatile bool isSecondBeat_ = false;                    // used to seed rate array so we startup with reasonable BPM
};


#endif //PULSESENSORLION_PULSESENSOR_H
