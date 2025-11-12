#include "HX711.h"
#include <OneWire.h>
#include <DallasTemperature.h>

const int LIFT_DOUT = 11;
const int LIFT_SCK  = 10;
const int DRAG_DOUT = 13;
const int DRAG_SCK  = 12;

const int MPX_POS = A5;
const int MPX_NEG = A4;

#define ONE_WIRE_BUS 2

float LIFT_SCALE = -459.542f;
float DRAG_SCALE = -470.000f;

const bool PRINT_IN_NEWTONS = false;

HX711 liftCell;
HX711 dragCell;

const float ADC_VREF  = 5.0f;
const float ADC_COUNT = 1023.0f;
const float MPX_SENS  = 0.0035f;
const uint8_t MPX_AVG_SAMPLES = 64;
long mpxZeroDiff = 0;

const float AIR_DENSITY = 1.225;
const float KPA_TO_PSI = 0.145038;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

const unsigned long UPDATE_INTERVAL = 200;
unsigned long lastUpdate = 0;

unsigned long tempRequestTime = 0;
bool tempRequested = false;
const unsigned long TEMP_CONVERSION_TIME = 750;
float lastTempC = 0.0;
float lastTempF = 0.0;

long readAvgADC(int pin, int n=64){
  long s=0; for(int i=0;i<n;i++) s += analogRead(pin);
  return s/n;
}

float gramsToNewtons(float g) { return g * 0.00980665f; }

void setup() {
  Serial.begin(57600);
  while (!Serial) { }

  Serial.println("=== Multi-Sensor Data Acquisition System ===");
  Serial.println("Dual HX711: Lift + Drag");
  Serial.println("MPX10DP: Pressure + Wind Velocity");
  Serial.println("DS18B20 Temperature Sensor");
  Serial.println();

  liftCell.begin(LIFT_DOUT, LIFT_SCK);
  dragCell.begin(DRAG_DOUT, DRAG_SCK);

  liftCell.set_scale(LIFT_SCALE);
  dragCell.set_scale(DRAG_SCALE);

  Serial.println("Taring lift...");
  liftCell.tare();
  Serial.println("Taring drag...");
  dragCell.tare();

  analogReference(DEFAULT);

  sensors.begin();

  delay(200);
  long rPos = readAvgADC(MPX_POS, MPX_AVG_SAMPLES);
  long rNeg = readAvgADC(MPX_NEG, MPX_AVG_SAMPLES);
  mpxZeroDiff = rPos - rNeg;

  Serial.println("Ready.");
  Serial.println();
}

void loop() {
  unsigned long currentTime = millis();

  if (!tempRequested) {
    sensors.requestTemperatures();
    tempRequestTime = currentTime;
    tempRequested = true;
  }

  if (tempRequested && (currentTime - tempRequestTime >= TEMP_CONVERSION_TIME)) {
    float tempC = sensors.getTempCByIndex(0);
    if (tempC != DEVICE_DISCONNECTED_C) {
      lastTempC = tempC;
      lastTempF = DallasTemperature::toFahrenheit(tempC);
    }
    tempRequested = false;
  }

  if (currentTime - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = currentTime;

    const uint8_t N = 10;
    float lift_g = 0.0;
    float drag_g = 0.0;

    if (liftCell.is_ready()) {
      lift_g = liftCell.get_units(N);
    }
    if (dragCell.is_ready()) {
      drag_g = dragCell.get_units(N);
    }

    if (PRINT_IN_NEWTONS) {
      float lift_N = gramsToNewtons(lift_g);
      float drag_N = gramsToNewtons(drag_g);
      Serial.print("Lift: "); Serial.print(lift_N, 4); Serial.print(" N\t");
      Serial.print("Drag: "); Serial.print(drag_N, 4); Serial.print(" N\t");
    } else {
      Serial.print("Lift: "); Serial.print(lift_g, 2); Serial.print(" g\t");
      Serial.print("Drag: "); Serial.print(drag_g, 2); Serial.print(" g\t");
    }

    long rPos = readAvgADC(MPX_POS, MPX_AVG_SAMPLES);
    long rNeg = readAvgADC(MPX_NEG, MPX_AVG_SAMPLES);
    long diffCounts = (rPos - rNeg) - mpxZeroDiff;
    float vDiff = (diffCounts * ADC_VREF) / ADC_COUNT;
    float mpx_kPa = vDiff / MPX_SENS;

    if (mpx_kPa < 0) {
      mpx_kPa = 0;
    }

    Serial.print("MPX10: ");
    Serial.print(mpx_kPa, 3); Serial.print(" kPa\t");

    float pressurePa = mpx_kPa * 1000.0;
    float velocity = 0.0;

    if (pressurePa > 0) {
      velocity = sqrt((2.0 * pressurePa) / AIR_DENSITY);
    }

    float pressurePsi = mpx_kPa * KPA_TO_PSI;

    Serial.print("Wind: ");
    Serial.print(velocity, 2); Serial.print(" m/s\t");
    Serial.print("P: ");
    Serial.print(pressurePsi, 4); Serial.print(" PSI\t");

    if (lastTempC == 0.0 && lastTempF == 0.0) {
      Serial.println("Temp: (waiting for first reading...)");
    } else {
      Serial.print("Temp: ");
      Serial.print(lastTempC, 2); Serial.print(" °C / ");
      Serial.print(lastTempF, 2); Serial.println(" °F");
    }
  }
}
