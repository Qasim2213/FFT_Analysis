#ifndef Device_h
#define Device_h

#include <arduinoFFT.h>
#include <driver/adc.h>
#include "filters.h"

//#define ESP_MINI
#define SAMPLES 256
#define SAMPLING_FREQUENCY 10000
#define AMPLITUDE 100
#define REGISTERED_FREQUENCY 4000
#define REGISTERED_AMPLITUDE 100
#define SAMPLED_FREQUENCY 4000
#define SAMPLED_AMPLITUDE 100
#ifdef ESP_MINI
#define ADC_CHANNEL_CHOSEN2 ADC1_CHANNEL_1
#define ADC_CHANNEL_CHOSEN ADC1_CHANNEL_0
#else 
#define ADC_CHANNEL_CHOSEN ADC1_CHANNEL_0
#endif
class Detection {
  private:
    arduinoFFT FFT;
    double sampledSignal[SAMPLES];
    double filteredSignal[SAMPLES];
    double filteredSignal2[SAMPLES];
    double preRegisteredSignal[SAMPLES];
    void processFFT(double *vReal, double *vImag);
    double correlateSignals(const double *signal1, const double *signal2);
    void generatePWM(double* signal, unsigned int frequency, double amplitude);
    void generateSinus(double* signal, double frequency, double amplitude);
    void generateTriangle(double* signal, unsigned int frequency, double amplitude);
    void applyMovingAverageFilter(double *signal, int windowSize);
    void initializeFilter();
  public:
    Detection();
    void sampleSignal();
    void generateSignals();
    int8_t performAnalysis();
    void printSignal(const double *signal, const char *label);
    //getter
    double* getSampledSignal() { return sampledSignal; }
    double computePeak();
    double computeAverage();
};
#endif
