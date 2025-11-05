#include "HX711.h"
#include <OneWire.h>
#include <DallasTemperature.h>

// -------- Pin assignments (your requested mapping) --------
const int LIFT_DOUT = 11;   // Lift DT
const int LIFT_SCK  = 10;   // Lift SCK

const int DRAG_DOUT = 13;   // Drag DT
const int DRAG_SCK  = 12;   // Drag SCK

// -------- Calibration factors (unchanged) --------
float LIFT_SCALE = -459.542f;   // <-- replace with your lift cell factor
float DRAG_SCALE = -470.000f;   // <-- replace with your drag cell factor

// If you prefer Newtons, set true (unchanged behavior)
const bool PRINT_IN_NEWTONS = false;

HX711 liftCell;
HX711 dragCell;

// ====================== MPX10DP (direct to ADC) ======================
// Wiring: +VOUT -> A5, -VOUT -> A4, VS -> 5V, GND -> GND
const int MPX_POS = A5;
const int MPX_NEG = A4;
const float ADC_VREF  = 5.0f;      // UNO/Nano
const float ADC_COUNT = 1023.0f;   // 10-bit ADC
const float MPX_SENS  = 0.0035f;   // ≈3.5 mV/kPa (0–10 kPa ~35 mV span)
const uint8_t MPX_AVG_SAMPLES = 64;
long mpxZeroDiff = 0;              // captured at startup

// ====================== DS18B20 on D2 ======================
#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// ====================== Helpers ======================
long readAvgADC(int pin, int n=64){
  long s=0; for(int i=0;i<n;i++) s += analogRead(pin);
  return s/n;
}
float gramsToNewtons(float g) { return g * 0.00980665f; }

// ====================== Setup (kept your flow) ======================
void setup() {
  Serial.begin(57600);
  while (!Serial) { /* wait for native USB boards; safe on Uno */ }

  Serial.println("Dual HX711: Lift + Drag");

  // Init both cells
  liftCell.begin(LIFT_DOUT, LIFT_SCK);
  dragCell.begin(DRAG_DOUT, DRAG_SCK);

  // Set scale factors (units baked in by your calibration)
  liftCell.set_scale(LIFT_SCALE);
  dragCell.set_scale(DRAG_SCALE);

  // Tare both with the rig at zero-load condition
  Serial.println("Taring lift...");
  liftCell.tare();
  Serial.println("Taring drag...");
  dragCell.tare();

  // DS18B20
  sensors.begin();

  // MPX10DP zero capture (assumes both ports equal)
  delay(200);
  long rPos = readAvgADC(MPX_POS, MPX_AVG_SAMPLES);
  long rNeg = readAvgADC(MPX_NEG, MPX_AVG_SAMPLES);
  mpxZeroDiff = rPos - rNeg;

  Serial.println("Ready.");
}

// helper to optionally convert grams -> Newtons
float gramsToNewtons(float g);

// ====================== Loop (your HX711 line preserved) ======================
void loop() {
  // Average a few samples from each for stability (unchanged)
  const uint8_t N = 10;

  float lift_g = liftCell.get_units(N); // grams if you calibrated in grams
  float drag_g = dragCell.get_units(N);

  // ---- Your original print line (unchanged formatting) ----
  if (PRINT_IN_NEWTONS) {
    float lift_N = gramsToNewtons(lift_g);
    float drag_N = gramsToNewtons(drag_g);
    Serial.print("Lift: "); Serial.print(lift_N, 4); Serial.print(" N\t");
    Serial.print("Drag: "); Serial.print(drag_N, 4); Serial.println(" N");
  } else {
    Serial.print("Lift: "); Serial.print(lift_g, 2); Serial.print(" g\t");
    Serial.print("Drag: "); Serial.print(drag_g, 2); Serial.println(" g");
  }

  // ---- Extra line: pressure + temperature ----
  // MPX10DP differential (A5 - A4)
  long rPos = readAvgADC(MPX_POS, MPX_AVG_SAMPLES);
  long rNeg = readAvgADC(MPX_NEG, MPX_AVG_SAMPLES);
  long diffCounts = (rPos - rNeg) - mpxZeroDiff;
  float vDiff = (diffCounts * ADC_VREF) / ADC_COUNT;  // Volts
  float mpx_kPa = vDiff / MPX_SENS;                   // VERY approximate w/o amplifier

  // DS18B20
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  float tempF = DallasTemperature::toFahrenheit(tempC);

  // Second line so your load cell line stays exactly as before
  Serial.print("Pressure: ");
  Serial.print(mpx_kPa, 3); Serial.print(" kPa\t");
  if (tempC == DEVICE_DISCONNECTED_C) {
    Serial.println("Temp: (DS18B20 not found)");
  } else {
    Serial.print("Temp: ");
    Serial.print(tempC, 2); Serial.print(" °C / ");
    Serial.print(tempF, 2); Serial.println(" °F");
  }

  delay(200); // ~5 Hz update (unchanged)
}