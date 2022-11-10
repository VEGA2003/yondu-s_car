#include <Arduino.h>
#include "arduinoFFT.h"

#define SAMPLES 256
#define SAMPLING_FREQUENCY 3072

arduinoFFT FFT = arduinoFFT();

unsigned int samplingPeriod;
unsigned long microSeconds;
float f1 = 784;   //Note: G5
float f2 = 880;   //Note: A5
float f3 = 1047;  //Note: C6
float f4 = 698;   //Note: F5
float w = 40;     //Window


double vReal[SAMPLES];  //create vector of size SAMPLES to hold real values
double vImag[SAMPLES];  //create vector of size SAMPLES to hold imaginary values


#include "Correlation.h"
using namespace std;

float a0, a1;
int now;
const int period = 255;
float data1[period] = {};
float data2[period] = {};
float corr = 0;
float current_corr;
int kmax = 0;    //Shift (in samples) where the max of the cross correlation occurs
float dt;
float t = 0;
int buffer = 70;    //For better accuracy
int sound_threshold = 150;  //To detect whether there is sound or not
float mean1;
float mean2;


float Average(float data[]) {
  // Declare Variable
  double _Sum = 0;
  // Calculate array sum
  for (int i = 0; i < period; i++) _Sum += data[i];
  // Calculate average
  double _Average = _Sum / period;
  // End Function
  return (_Average);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);

  samplingPeriod = round(1000000 * (1.0 / SAMPLING_FREQUENCY));  //Period in microseconds
}

void loop() {
  for (int i = 0; i < SAMPLES; i++) {
    microSeconds = micros();  //Returns the number of microseconds since the Arduino board began running the current script.

    vReal[i] = analogRead(A0);  //Reads the value from analog pin 0 (A0), quantize it and save it as a real term.
    vImag[i] = 0;               //Makes imaginary term 0 always

    //remaining wait time between samples if necessary
    while (micros() < (microSeconds + samplingPeriod)) {
      //do nothing
    }
  }
  //Perform FFT on samples
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

  //Find peak frequency and print peak
  double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);

  // Take 255 samples. Also calculate the sampling time
  for (int i = 0; i < period; i++) {
    a0 = analogRead(A0);
    a1 = analogRead(A1);
    data1[i] = a0;
    data2[i] = a1;
  }

  //Determine difference between lowest and highest value
  // for a0 to check whether there is sound
  int maxa0 = 0;
  int mina0 = 1000;

  for (int j = 0; j < period; j++) {
    if (data1[j] > maxa0) {
      maxa0 = data1[j];
    }
    if (data1[j] < mina0) {
      mina0 = data1[j];
    }
  }

  int diffa0 = maxa0 - mina0;



  //If there is sound, execute the cross correlation loop
  if (diffa0 > sound_threshold) {

    mean1 = Average(data1);
    mean2 = Average(data2);

    Correlation C(period);
    C.clear();
    corr = 0;
    kmax = 0;

    for (int k = -(period - 1) / 2 + buffer; k < (period + 1) / 2 - buffer; k++) {
      Correlation C(period - abs(k));
      C.clear();
      for (int j = 0; j < period - abs(k); j++) {
          if (k < 0) {
            C.add(data1[j]-mean1, data2[abs(k) + j]-mean2);
          } else {
            C.add(data1[abs(k) + j]-mean1, data2[j]-mean2);
          }
      }
      C.calculate();
      current_corr = C.getR();
      if (abs(current_corr) > abs(corr)) {
        corr = current_corr;
        kmax = k;
      }
    }

    //If correlation smaller than 0.8, results are mostly wrong
    //So make sure these are not taken into account
    if (corr < 0.8) {
      kmax = 0;
    }
  }


  // Couple the results from sound localization and pitch recognition
  // to commands to send it to Arduino Mega
  if (peak > f1 - w && peak < f1 + w && diffa0 > sound_threshold) {
    Serial1.println("1");
  } else if (peak > f2 - w && peak < f2 + w && diffa0 > sound_threshold) {
    Serial1.println("2");
  } else if (peak > f3 - w && peak < f3 + w && diffa0 > sound_threshold) {
    Serial1.println("3");
  } else if (peak > f4 - w && peak < f4 + w && abs(kmax) < 20 && diffa0 > sound_threshold) {
    Serial1.println("4");
  } else if (peak > f4 - w && peak < f4 + w && kmax > 20 && abs(kmax) < 60 && diffa0 > sound_threshold) {
    Serial1.println("5");
  } else if (peak > f4 - w && peak < f4 + w && kmax < -20 && abs(kmax) < 60 && diffa0 > sound_threshold) {
    Serial1.println("6");
  }
}