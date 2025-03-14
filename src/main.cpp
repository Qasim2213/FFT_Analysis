#include <Arduino.h>
#include "detection.h"
#include <Adafruit_NeoPixel.h>
#include <driver/adc.h>
#include <WiFi.h>
#include <HTTPClient.h>

// ==================== Wi-Fi Configuration ====================
const char* ssid = "FE1/6";         // Replace with your Wi-Fi SSID
const char* password = "pakistan123"; // Replace with your Wi-Fi password
const char* serverName = "http://192.168.100.32/data";  // Your laptop's IP

// ==================== NeoPixel Configuration ====================
#ifdef ESP_MINI
Adafruit_NeoPixel strip(16, 10, NEO_GRB + NEO_KHZ800); // For ESP_MINI
#else 
Adafruit_NeoPixel strip(16, 14, NEO_GRB + NEO_KHZ800); // For other ESP32 boards
#endif

// ==================== Pin Definitions ====================
#define PIN_LED 2
#define PIN_VIB 9
#define PIN_BUTTON 21
#define PIN_MASSE_FLEURET 6
#define PIN_NV 7

// ==================== Global Variables ====================
Detection detect; // Detection class instance
#define MAX_DATASETS 5

struct Dataset {
    double signal[SAMPLES];
    double peakFrequency;
    double averageIntensity;
} datasets[MAX_DATASETS];

int datasetIndex = 0;

// Structure to send touch data to server
struct TouchData {
  char deviceId;  // 'A' or 'B'
  int8_t state;   // -1 (no touch), 0 (invalid), 1 (valid)
  double intensity;
  double peakFrequency;
} touchData;

// Function to send data to server
void sendToServer(TouchData data) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverName);
    http.addHeader("Content-Type", "application/json");

    String jsonData = "{\"deviceId\":\"" + String(data.deviceId) +
                      "\",\"state\":" + String(data.state) +
                      ",\"intensity\":" + String(data.intensity) +
                      ",\"peakFrequency\":" + String(data.peakFrequency) + "}";
    int httpResponseCode = http.POST(jsonData);

    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.print("Server Response: ");
      Serial.println(response);
    } else {
      Serial.print("Error sending to server: ");
      Serial.println(httpResponseCode);
    }
    http.end();
  } else {
    Serial.println("WiFi Disconnected");
  }
}

// ==================== Setup Function ====================
void setup() {
  Serial.begin(115200);
  Serial.println("Starting ESP32 (Device A)...");
  delay(2000);

  // Initialize NeoPixel strip
  strip.begin();
  strip.setBrightness(20);
  strip.show();

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Set Device ID
  touchData.deviceId = 'B';  // This is Device A
}

// ==================== Loop Function ====================
void loop() {
  // Sample the signal
  detect.sampleSignal();  

  // Retrieve the sampled signal and store it in the current dataset
  const double* signal = detect.getSampledSignal();
  memcpy(datasets[datasetIndex].signal, signal, sizeof(datasets[datasetIndex].signal));

  // Perform analysis
  datasets[datasetIndex].averageIntensity = detect.computeAverage(); 
  datasets[datasetIndex].peakFrequency = detect.computePeak();

  // Determine state based on analysis
  int8_t state = -1;
  if (datasets[datasetIndex].averageIntensity > 15) {
    if ((datasets[datasetIndex].peakFrequency < 5100) && (datasets[datasetIndex].peakFrequency > 4900)) {
      state = 1;  // Valid touch
    } else {
      state = 0;  // Invalid touch
    }
  }

  // Update LED strip based on state
  if (state == 1) {
    strip.fill(strip.Color(0, 255, 0), 0, 16); // Green for valid touch
  } else if (state == 0) {
    strip.fill(strip.Color(255, 0, 0), 0, 16); // Red for invalid touch
  } else {
    strip.fill(strip.Color(255, 255, 255), 0, 16); // White for no touch
  }
  strip.show();

  // Update touch data and send if there's a change
  static int8_t lastState = -1;
  //if (state != lastState) {
    touchData.state = state;
    touchData.intensity = datasets[datasetIndex].averageIntensity;
    touchData.peakFrequency = datasets[datasetIndex].peakFrequency;

    // Send to server
    sendToServer(touchData);

    lastState = state;  // Update last state
  //}

  // Update dataset index
  datasetIndex = (datasetIndex + 1) % MAX_DATASETS;

  delay(500);  // Adjust as needed
}