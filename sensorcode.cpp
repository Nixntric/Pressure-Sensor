const int VOUT_PLUS = A0;
const int VOUT_MINUS = A1;

const float VS = 5.0;
const float SENSITIVITY = 3.5;
const float OFFSET_VOLTAGE = 20.0;
const float ADC_RESOLUTION = 1023.0;

const float AIR_DENSITY = 1.225;
const float KPA_TO_PSI = 0.145038;
const float PA_TO_PSI = 0.000145038;

const unsigned long UPDATE_INTERVAL = 500;
unsigned long lastUpdate = 0;

void setup() {
  Serial.begin(9600);
  
  analogReference(DEFAULT);
  
  Serial.println("Wind Velocity (m/s), Pressure (PSI)");
  delay(1000);
}

void loop() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = currentTime;
    
    float vOutPlus = analogRead(VOUT_PLUS) * (VS / ADC_RESOLUTION);
    float vOutMinus = analogRead(VOUT_MINUS) * (VS / ADC_RESOLUTION);
    float vDiff = vOutPlus - vOutMinus;
    
    float vDiffMv = vDiff * 1000.0;
    float pressureKpa = (vDiffMv - OFFSET_VOLTAGE) / SENSITIVITY;
    
    if (pressureKpa < 0) {
      pressureKpa = 0;
    }
    
    float pressurePsi = pressureKpa * KPA_TO_PSI;
    
    float pressurePa = pressureKpa * 1000.0;
    float velocity = 0.0;
    
    if (pressurePa > 0) {
      velocity = sqrt((2.0 * pressurePa) / AIR_DENSITY);
    }
    
    Serial.print(velocity, 2);
    Serial.print(", ");
    Serial.println(pressurePsi, 2);
  }
}
