#include <Arduino.h>
#include <DHT.h>

/* PIN MAP */
#define PIN_LDR     34      // AO modul LDR -> ADC
#define PIN_PIR     23
#define PIN_LED_PWM 2       // lampu jalan (PWM via analogWrite)
#define PIN_LED_G   19      // indikator hijau (kosong)
#define PIN_LED_R   21      // indikator merah (penuh)
// Parkir A
#define TRIG_A 5
#define ECHO_A 18
// Parkir B
#define TRIG_B 17
#define ECHO_B 16
// DHT22
#define DHTPIN  4
#define DHTTYPE DHT22

/* TUNING */
int   TH_PARK_CM   = 12;
int   HYS_PARK_CM  = 3;
int   LDR_MIN = 300, LDR_MAX = 900;
int   PWM_MAX_BASE = 220;   // 0..255
int   PWM_BOOST    = 35;
int   SAMPLE_DELAY = 60;    // ms, cegah crosstalk ultrasonic

/* GLOBAL */
DHT dht(DHTPIN, DHTTYPE);
int  ldrAvg = 0;
bool parkA = false, parkB = false;

static inline int maf(int prev, int now, float a=0.2) { return prev + a*(now-prev); }

long readDistanceCM(int trig, int echo){
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long dur = pulseIn(echo, HIGH, 30000);
  if(!dur) return 400;
  return dur * 0.034 / 2;
}

void setup(){
  Serial.begin(115200);

  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(TRIG_A, OUTPUT); pinMode(ECHO_A, INPUT);
  pinMode(TRIG_B, OUTPUT); pinMode(ECHO_B, INPUT);

  // analogWrite pada ESP32 otomatis set PWM, tidak perlu ledcSetup
  pinMode(PIN_LED_PWM, OUTPUT);

  dht.begin();

  // test indikator
  digitalWrite(PIN_LED_G, HIGH); digitalWrite(PIN_LED_R, HIGH); delay(250);
  digitalWrite(PIN_LED_G, LOW);  digitalWrite(PIN_LED_R, LOW);
}

void loop(){
  // ----- LDR & lampu jalan (PWM) -----
  int ldr = analogRead(PIN_LDR);
  ldrAvg = maf(ldrAvg, ldr);
  int basePWM = map(constrain(ldrAvg, LDR_MIN, LDR_MAX), LDR_MIN, LDR_MAX, 0, PWM_MAX_BASE);
  bool pir = digitalRead(PIN_PIR);
  int targetPWM = constrain(basePWM + (pir ? PWM_BOOST : 0), 0, 255);
  analogWrite(PIN_LED_PWM, targetPWM);   // <--- pakai analogWrite

  // ----- Parkir A -----
  long a = readDistanceCM(TRIG_A, ECHO_A);
  if(!parkA && a < TH_PARK_CM) parkA = true;
  if( parkA && a > TH_PARK_CM + HYS_PARK_CM) parkA = false;

  delay(SAMPLE_DELAY);

  // ----- Parkir B -----
  long b = readDistanceCM(TRIG_B, ECHO_B);
  if(!parkB && b < TH_PARK_CM) parkB = true;
  if( parkB && b > TH_PARK_CM + HYS_PARK_CM) parkB = false;

  bool anyOccupied = parkA || parkB;
  digitalWrite(PIN_LED_R, anyOccupied ? HIGH : LOW);
  digitalWrite(PIN_LED_G, anyOccupied ? LOW  : HIGH);

  // ----- Log DHT tiap 2 detik -----
  static uint32_t t0=0;
  if(millis()-t0>2000){
    float t=dht.readTemperature(), h=dht.readHumidity();
    Serial.print("LDR=");Serial.print(ldrAvg);
    Serial.print(" PWM=");Serial.print(targetPWM);
    Serial.print("  A=");Serial.print(a);Serial.print("cm");
    Serial.print("  B=");Serial.print(b);Serial.print("cm");
    Serial.print("  ParkA=");Serial.print(parkA);
    Serial.print(" ParkB=");Serial.print(parkB);
    if(!isnan(t)&&!isnan(h)){ Serial.print("  T=");Serial.print(t,1);Serial.print("C H=");Serial.print(h,0);Serial.print("%"); }
    else { Serial.print("  DHT:nan"); }
    Serial.println();
    t0=millis();
  }

  delay(50);
}
