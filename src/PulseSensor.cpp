//
// Created by Andri Yadi on 10/1/16.
//

#include "PulseSensor.h"

void process(PulseSensor *sensor) {
    int raw = sensor->doReadRawSignal();

    sensor->processSignal(raw);
}

PulseSensor::PulseSensor() {

}

PulseSensor::~PulseSensor() {
    if (readingTicker_ != NULL) {
        delete readingTicker_;
        readingTicker_ = NULL;
    }
}

void PulseSensor::start() {

    if (readingTicker_ == NULL) {
        readingTicker_ = new Ticker();
    }

    isStarted_ = true;

//    IBI_ = 600;
//    pulseDetected_ = false;
//
//    sampleCounter_ = 0;
//    lastBeatTime_ = 0;
//    peak_ = 512;
//    trough_ = 512;
//    thresh_ = 525;
//    amplitude_ = 100;
//    isFirstBeat_ = true;
//    isSecondBeat_ = false;

    readingTicker_->attach_ms(PULSE_SENSOR_READING_INTERVAL_MS, process, this);
}

void PulseSensor::stop(){
    isStarted_ = false;

    thresh_ = 512;                          // set thresh default
    peak_ = 512;                            // set P default
    trough_ = 512;                          // set T default
    lastBeatTime_ = sampleCounter_;         // bring the lastBeatTime up to date
    isFirstBeat_ = true;                    // set these to avoid noise
    isSecondBeat_ = false;                  // when we get the heartbeat back

    if (readingTicker_ == NULL) {
        return;
    }

    readingTicker_->detach();
}

void PulseSensor::processSignal(int signalData) {

//    Serial.printf("Raw 2: %d\n", signalData);

    sampleCounter_ += PULSE_SENSOR_READING_INTERVAL_MS;

    unsigned long N = sampleCounter_ - lastBeatTime_;       // monitor the time since the last beat to avoid noise

    if (signalData < thresh_ && N > (IBI_/5)*3){            // avoid dichrotic noise by waiting 3/5 of last IBI
        if (signalData < trough_){
            trough_ = signalData;                           // keep track of lowest point in pulse wave
        }
    }

    if (signalData > thresh_ && signalData > peak_){        // thresh condition helps avoid noise
        peak_ = signalData;
    }

    // NOW IT'S TIME TO LOOK FOR THE HEART BEAT
    // signal surges up in value every time there is a pulse
    if (N > 250){                                   // avoid high frequency noise

        if ( (signalData > thresh_) && (!pulseDetected_) && (N > (IBI_/5)*3) ){
            pulseDetected_ = true;                          // set the Pulse flag when we think there is a pulse

            //digitalWrite(blinkPin,HIGH);                  // turn on pin 13 LED
            if (pulseDetectionCallback) {
                pulseDetectionCallback(true);
            }

            IBI_ = sampleCounter_ - lastBeatTime_;          // measure time between beats in mS
//            Serial.printf("IBI = %d\n", IBI_);

            lastBeatTime_ = sampleCounter_;                 // keep track of time for next pulse

            if(isSecondBeat_){                              // if this is the second beat, if secondBeat == TRUE
//                Serial.printf("Second Beat! %d\n", signalData);
                isSecondBeat_ = false;                      // clear secondBeat flag
                for(int i = 0; i <= (PULSE_SENSOR_SAMPLE_COUNT -1); i++){                // seed the running total to get a realisitic BPM at startup
                    lastRates_[i] = IBI_;
                }
            }

            if(isFirstBeat_){                               // if it's the first time we found a beat, if firstBeat == TRUE
//                Serial.printf("First Beat! %d\n", signalData);
                isFirstBeat_ = false;                       // clear firstBeat flag
                isSecondBeat_ = true;                       // set the second beat flag
                return;                                     // IBI value is unreliable so discard it
            }

            // keep a running total of the last PULSE_SENSOR_SAMPLE_COUNT IBI values
            word runningTotal = 0;                      // clear the runningTotal variable

            for(int i = 0; i <= (PULSE_SENSOR_SAMPLE_COUNT - 2); i++){                    // shift data in the rate array
                lastRates_[i] = lastRates_[i + 1];      // and drop the oldest IBI value
                runningTotal += lastRates_[i];          // add up the (PULSE_SENSOR_SAMPLE_COUNT - 1) oldest IBI values
            }

            lastRates_[PULSE_SENSOR_SAMPLE_COUNT -1 ] = (int)IBI_;      // add the latest IBI to the rate array
            runningTotal += lastRates_[PULSE_SENSOR_SAMPLE_COUNT - 1];  // add the latest IBI to runningTotal
            runningTotal /= PULSE_SENSOR_SAMPLE_COUNT;                  // average the last PULSE_SENSOR_SAMPLE_COUNT IBI values

            quantifiedSelf_ = true;                                     // set Quantified Self flag
            BPM_ = 60000/runningTotal;                                  // how many beats can fit into a minute? that's BPM!


            if (bpmAvailableCallback) {
                bpmAvailableCallback(BPM_);
                quantifiedSelf_ = false;
            }

        }
    }

    if (signalData < thresh_ && pulseDetected_){    // when the values are going down, the beat is over
        //digitalWrite(blinkPin,LOW);               // turn off pin 13 LED
        if (pulseDetectionCallback) {
            pulseDetectionCallback(false);
        }

        pulseDetected_ = false;                     // reset the Pulse flag so we can do it again
        amplitude_ = peak_ - trough_;               // get amplitude of the pulse wave

//        Serial.printf("Last Beat! AMP: %d, P: %d, T: %d\n", amplitude_, peak_, trough_);

        thresh_ = amplitude_ / 2 + trough_;         // set thresh at 50% of the amplitude
        peak_ = thresh_;                            // reset these for next time
        trough_ = thresh_;

    }

    if (N > 2500){                              // if 2.5 seconds go by without a beat
        thresh_ = 512;                          // set thresh default
        peak_ = 512;                            // set P default
        trough_ = 512;                          // set T default
        lastBeatTime_ = sampleCounter_;         // bring the lastBeatTime up to date
        isFirstBeat_ = true;                    // set these to avoid noise
        isSecondBeat_ = false;                  // when we get the heartbeat back
    }
}

void PulseSensor::onReadingRawSignal(PulseSensorRawSignalReadingCallback cb) {
    rawSignalReadingCallback_ = cb;
}

int PulseSensor::doReadRawSignal() {
    if (rawSignalReadingCallback_) {
        rawSignalReadingCallback_(lastRawSignal_);
    }

    return  lastRawSignal_;
}

void PulseSensor::onBPMAvailable(PulseSensorBPMAvailableCallback cb) {
    bpmAvailableCallback = cb;
}

void PulseSensor::onPulseDetectionCallback(PulseSensorPulseDetectionCallback cb) {
    pulseDetectionCallback = cb;
}
