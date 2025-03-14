#include "detection.h"
#include <Arduino.h>

const int16_t squareWave_1kHz_256samples[] = {
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

const float cutoff_freq = 800.0;
const float sampling_time = 0.0001;
Filter fhp(cutoff_freq, sampling_time, IIR::ORDER::OD2, IIR::TYPE::HIGHPASS);
int s1i = 0, s2i = 0;

Detection::Detection() {
    arduinoFFT FFT = arduinoFFT();
    adc1_config_width(ADC_WIDTH_12Bit); // Set resolution to 12-bit
    adc1_config_channel_atten(ADC_CHANNEL_CHOSEN, ADC_ATTEN_0db); // 0 attenuation to get full range
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_0db); // 0 attenuation to get full range
    
}


void Detection::sampleSignal() {
    const float cutoff_freq = 500;   // Cutoff frequency in Hz
    // CHECK FOR ERRORS
    Filter fhp(cutoff_freq, 1/SAMPLING_FREQUENCY, IIR::ORDER::OD2, IIR::TYPE::HIGHPASS);
    static_assert(SAMPLED_FREQUENCY > 0, "SAMPLED_RATE must be greater than 0.");
    static_assert(SAMPLES > 0, "SAMPLES must be defined and greater than 0.");

    unsigned long startTime = micros();
    unsigned long sampleInterval = 1000000 / SAMPLING_FREQUENCY; // Interval between samples in microseconds
#ifdef ADC2
    int rawValue = 0;
#endif
    for (int i = 0; i < SAMPLES; i++) {
        while (micros() - startTime < i * sampleInterval);

#ifndef ADC2
        int16_t rawValue = adc1_get_raw(ADC_CHANNEL_CHOSEN);
#else
        esp_wifi_stop();
        esp_err_t r = adc2_get_raw(ADC_CHANNEL_CHOSEN, ADC_WIDTH_12Bit, &rawValue);
        if (r == ESP_OK) {
            // Successfully read ADC value
        } else if (r == ESP_ERR_TIMEOUT) {
            printf("ADC2 used by Wi-Fi.\n");
        }
        esp_wifi_start();
#endif
        //rawValue = map(rawValue, 0, 4095, 0, 1023);
        double rawValue2 = (double)rawValue;
        this->sampledSignal[i] = rawValue2;
        //float sign_filt = fhp.filterIn(rawValue2);
        // Print both the original (scaled) and filtered signal
    }
}


void Detection::processFFT(double *vReal, double *vImag) {
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  // Hann Windowing
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD); // Compute the FFT
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES); // Convert the complex output to magnitude
}

int8_t Detection::performAnalysis() {
    double average = computeAverage();
    double peak = computePeak();
    Serial.print("Average: ");
    Serial.println(average);
    Serial.print("Peak: ");
    Serial.println(peak);
  
    if(peak > 3900 && peak < 4100) {
        return 1; // Valid touch
    }
    else if(peak > 10) {
        return -1; // Invalid touch
    }
    else {
        return 0; // No touch
    }
}

double Detection::computePeak() {
    double vImag[SAMPLES] = {0};
    //copy the sampled signal to avoid modifying the original
    double sampledSignal_copy[SAMPLES];
    memcpy(sampledSignal_copy, this->filteredSignal, sizeof(filteredSignal));
    processFFT(sampledSignal_copy, vImag); // Process FFT on the pre-registered signal
    double peak = FFT.MajorPeak(sampledSignal_copy, SAMPLES, SAMPLING_FREQUENCY);
    Serial.print("Peak Frequency: ");
    Serial.println(peak);
    return peak;
}

double Detection::computeAverage() {
    float sign_raw, sign_filt;
    for (int i = 0; i < 256; i++) {
        sign_raw = (float)(this->sampledSignal[i]);
        sign_filt = fhp.filterIn(sign_raw);
        this->filteredSignal[i] = sign_filt;
        //printf("Sample %d: Raw Signal = %f, Filtered Signal = %f\n", i, this->sampledSignal[i], filteredSignal[i]);
    }
    double average = 0;
    for(unsigned int i = 0; i < SAMPLES; i++) {
        average += sampledSignal[i];
    }
    average = average / SAMPLES;
    Serial.print("Average Intensity: ");
    Serial.println(average);
    return average;
}


void Detection::generateSignals() {
  generatePWM(preRegisteredSignal, REGISTERED_FREQUENCY, REGISTERED_AMPLITUDE);
  //generatePWM(sampledSignal, SAMPLED_FREQUENCY, SAMPLED_AMPLITUDE);
  sampleSignal();
  //display sampled signal
  //printSignal(sampledSignal, "Sampled Signal");
  //display pre-registered signal
  //printSignal(preRegisteredSignal, "Pre-registered Signal");
}


double Detection::correlateSignals(const double *signal1, const double *signal2) {
  double result = 0.0;
  for (unsigned int i = 0; i < SAMPLES; i++) {
    result += signal1[i] * signal2[i];
  } 
  return result / SAMPLES; // Basic correlation
}

void Detection::printSignal(const double *signal, const char *label) {
  Serial.println(label);
  for (unsigned int i = 0; i < SAMPLES; i++) {
    Serial.print(signal[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void Detection::generatePWM(double* signal, unsigned int frequency, double amplitude) {
  unsigned int periodSamples = SAMPLING_FREQUENCY / frequency; 
  for(unsigned int i = 0; i < SAMPLES; i++) {
    // Check if the current sample is within the first half of the period
    signal[i] = (i % periodSamples) < (periodSamples / 2) ? amplitude : 0;
  }
}
void Detection::generateSinus(double* signal, double frequency, double amplitude) {
  for(unsigned int i = 0; i < SAMPLES; i++) {
    double t = (double)i / SAMPLING_FREQUENCY;
    signal[i] = amplitude * sin(2 * PI * frequency * t);
  }
}
void Detection::generateTriangle(double* signal, unsigned int frequency, double amplitude) {
  unsigned int periodSamples = SAMPLING_FREQUENCY / frequency;
  double slope = (2.0 * amplitude) / (periodSamples / 2); // Slope for half period
  for(unsigned int i = 0; i < SAMPLES; i++) {
    unsigned int positionInPeriod = i % periodSamples;
    if (positionInPeriod < periodSamples / 2) {
      // First half - increasing
      signal[i] = slope * positionInPeriod;
    } else {
      // Second half - decreasing
      signal[i] = 2 * amplitude - slope * positionInPeriod;
    }
  }
}
