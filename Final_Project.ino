// Robert Wright
// Loghan Flanders

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <RTClib.h>
#include <Stepper.h>
#include <avr/io.h> // for ADC & port names

// State values
#define ST_DIS 0
#define ST_IDLE 1
#define ST_RUN 2
#define ST_ERR 3

// Prototypes
void stDis();
void stIdle();
void stRun();
void stErr();

void pinModeReg(uint8_t pin, uint8_t mode);
void writeReg(uint8_t pin, uint8_t val);
uint8_t readReg(uint8_t pin);
void adcInit();
uint16_t adcRead(uint8_t ch);
void setLED(int s);
void logEvent(const char* msg);
void initPins();
void initPeripherals();
void lcdUpdate(float t, float h, int s);
void getTH(float &t, float &h);
void fanOn();
void fanOff();
void ventCtrl();
uint16_t getWaterLevel();
void startButtonISR();

// UART register helpers (USART0)
void uart_init(unsigned long baud);
void uart_send_char(char c);
void uart_send_str(const char* s);
void uart_send_line(const char* s);

// Pins
#define LED_YELLOW_PIN 22
#define LED_GREEN_PIN 23
#define LED_BLUE_PIN 24
#define LED_RED_PIN 25
#define FAN_MOTOR_PIN 4
#define BUTTON_START_PIN 2
#define BUTTON_STOP_PIN 30
#define BUTTON_RESET_PIN 31
#define WATER_LEVEL_ADC_CHANNEL 0
#define VENT_POT_ADC_CHANNEL 1
#define DHT_PIN 7
#define DHT_TYPE DHT11
#define STEPS_PER_REV 2048
#define STEPPER_IN1 8
#define STEPPER_IN2 9
#define STEPPER_IN3 10
#define STEPPER_IN4 11

// Thresholds
#define TEMP_THRESHOLD_HIGH 25.0f
#define TEMP_THRESHOLD_LOW 22.0f
#define WATER_THRESHOLD_LOW 50
#define MAX_VENT_POSITION 100

// Objects
LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C backpack assumed (B)
DHT dht(DHT_PIN, DHT_TYPE);
RTC_DS1307 rtc; // DS1307 as per kit (C)
Stepper ventMotor(STEPS_PER_REV, STEPPER_IN1, STEPPER_IN3, STEPPER_IN2, STEPPER_IN4);

// Timing
unsigned long lastDisplayUpdateTime = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL = 60000UL;

// Vent
int targetVentPosition = 0;
int currentStepPosition = 0;
const int STEPS_PER_INCREMENT = STEPS_PER_REV / MAX_VENT_POSITION;

// Make currentState volatile because ISR changes it
volatile int currentState = ST_DIS;

// ----------------- Setup & Loop -----------------
void setup(){
  // Initialize UART (register-level) at 9600
  uart_init(9600);

  initPins();
  initPeripherals();
  // ISR for start button - allowed per spec
  attachInterrupt(digitalPinToInterrupt(BUTTON_START_PIN), startButtonISR, FALLING);

  currentState = ST_DIS;
  setLED(ST_DIS);
  lcdUpdate(0,0,ST_DIS);
  logEvent("System Ready");
}

void loop(){
  if(currentState==ST_DIS) stDis();
  else if(currentState==ST_IDLE) stIdle();
  else if(currentState==ST_RUN) stRun();
  else if(currentState==ST_ERR) stErr();
}

// ----------------- ISR -----------------
void startButtonISR(){
  if(currentState==ST_DIS) currentState = ST_IDLE;
}

// ----------------- Pin helpers (no Arduino pinMode/digitalWrite) -----------------
void pinModeReg(uint8_t pin, uint8_t mode){
  // mode: 1=OUTPUT, 0=INPUT
  uint8_t port = digitalPinToPort(pin);
  uint8_t mask = digitalPinToBitMask(pin);
  volatile uint8_t *ddr = portModeRegister(port);
  volatile uint8_t *out = portOutputRegister(port);
  if(mode) {
    *ddr |= mask;   // output
  } else {
    *ddr &= ~mask;  // input
    // do not change pull-up here
  }
}

void writeReg(uint8_t pin, uint8_t val){
  uint8_t port = digitalPinToPort(pin);
  uint8_t mask = digitalPinToBitMask(pin);
  volatile uint8_t *out = portOutputRegister(port);
  if(val) *out |= mask;
  else    *out &= ~mask;
}

uint8_t readReg(uint8_t pin){
  uint8_t port = digitalPinToPort(pin);
  uint8_t mask = digitalPinToBitMask(pin);
  volatile uint8_t *in = portInputRegister(port);
  return ((*in) & mask) ? 1 : 0;
}

// ----------------- ADC (register-level) -----------------
void adcInit(){
  // Reference = AVcc (REFS0 = 1), ADC enable, prescaler = 128
  ADMUX = (1 << REFS0); // channel selected later
  ADCSRA = (1 << ADEN)  // ADC Enable
         | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // prescaler 128
}

uint16_t adcRead(uint8_t ch){
  // select channel (0-7)
  ADMUX = (ADMUX & 0xF0) | (ch & 0x0F);
  ADCSRA |= (1 << ADSC); // start conversion
  while (ADCSRA & (1 << ADSC)); // wait
  return ADC; // 16-bit ADC register alias from <avr/io.h>
}

// ----------------- LED -----------------
void setLED(int s){
  writeReg(LED_YELLOW_PIN,0);
  writeReg(LED_GREEN_PIN,0);
  writeReg(LED_BLUE_PIN,0);
  writeReg(LED_RED_PIN,0);
  if(s==ST_DIS) writeReg(LED_YELLOW_PIN,1);
  else if(s==ST_IDLE) writeReg(LED_GREEN_PIN,1);
  else if(s==ST_RUN) writeReg(LED_BLUE_PIN,1);
  else if(s==ST_ERR) writeReg(LED_RED_PIN,1);
}

// ----------------- UART (register-level) -----------------
void uart_init(unsigned long baud) {
  // Use USART0, set baud rate with UBRR0, double speed
  uint16_t ubrr = (F_CPU / 4 / baud - 1) / 2;
  UCSR0A = (1 << U2X0);
  UBRR0H = (ubrr >> 8);
  UBRR0L = ubrr & 0xFF;
  UCSR0B = (1 << TXEN0); // enable transmitter
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8-bit char
}

void uart_send_char(char c) {
  while (!(UCSR0A & (1 << UDRE0)));
  UDR0 = (uint8_t)c;
}

void uart_send_str(const char* s) {
  while (*s) uart_send_char(*s++);
}

void uart_send_line(const char* s) {
  uart_send_str(s);
  uart_send_char('\r');
  uart_send_char('\n');
}

// ----------------- Logging via RTC and UART -----------------
void logEvent(const char* msg){
  DateTime n;
  if(rtc.isrunning()) n = rtc.now();
  else {
    // If RTC not running, produce a partial timestamp (00...); keep it simple
    n = DateTime(2000,1,1,0,0,0);
  }
  char buf[128];
  snprintf(buf, sizeof(buf), "%04u-%02u-%02u %02u:%02u:%02u | %s",
           n.year(), n.month(), n.day(),
           n.hour(), n.minute(), n.second(), msg);
  uart_send_line(buf);
}

// ----------------- Init pins & peripherals -----------------
void initPins(){
  // LEDs outputs
  pinModeReg(LED_YELLOW_PIN,1);
  pinModeReg(LED_GREEN_PIN,1);
  pinModeReg(LED_BLUE_PIN,1);
  pinModeReg(LED_RED_PIN,1);
  // Fan output
  pinModeReg(FAN_MOTOR_PIN,1);
  // Buttons: make inputs with internal pull-ups
  // For each button, set DDR bit = 0 and set PORT bit = 1 (pull-up)
  // Use direct register helpers:
  // start button (pin 2) handled by ISR; enable pull-up
  {
    uint8_t p = digitalPinToPort(BUTTON_START_PIN);
    uint8_t m = digitalPinToBitMask(BUTTON_START_PIN);
    volatile uint8_t *ddr = portModeRegister(p);
    volatile uint8_t *out = portOutputRegister(p);
    *ddr &= ~m; // input
    *out |= m;  // pull-up
  }
  // stop and reset buttons
  {
    uint8_t p = digitalPinToPort(BUTTON_STOP_PIN);
    uint8_t m = digitalPinToBitMask(BUTTON_STOP_PIN);
    volatile uint8_t *ddr = portModeRegister(p);
    volatile uint8_t *out = portOutputRegister(p);
    *ddr &= ~m; *out |= m;
  }
  {
    uint8_t p = digitalPinToPort(BUTTON_RESET_PIN);
    uint8_t m = digitalPinToBitMask(BUTTON_RESET_PIN);
    volatile uint8_t *ddr = portModeRegister(p);
    volatile uint8_t *out = portOutputRegister(p);
    *ddr &= ~m; *out |= m;
  }
  // Note: ADC pins (A0/A1) are inputs by default; no pinMode needed.
  setLED(ST_DIS);
  writeReg(FAN_MOTOR_PIN, 0); // ensure fan off
}

void initPeripherals(){
  // LCD I2C
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Init...");
  // RTC
  Wire.begin();
  rtc.begin();
  // DHT
  dht.begin();
  // ADC
  adcInit();
  // Stepper speed
  ventMotor.setSpeed(15);
}

// ----------------- Sensors -----------------
uint16_t getWaterLevel(){ return adcRead(WATER_LEVEL_ADC_CHANNEL); }

void getTH(float &t, float &h){
  t = dht.readTemperature();
  h = dht.readHumidity();
  if(isnan(t) || isnan(h)) { t = -99; h = -99; }
}

// ----------------- LCD update -----------------
void lcdUpdate(float t,float h,int s){
  lcd.clear();
  lcd.setCursor(0,0);
  if(s==ST_DIS) lcd.print("S:DIS");
  else if(s==ST_IDLE) lcd.print("S:IDLE");
  else if(s==ST_RUN) lcd.print("S:RUN");
  else lcd.print("S:ERR");

  if(s==ST_ERR){ lcd.setCursor(0,1); lcd.print("WATER LOW"); return; }
  if(s==ST_DIS){ lcd.setCursor(0,1); lcd.print("System Off"); return; }

  lcd.setCursor(0,1);
  // print with limited precision
  char buf[17];
  if (t > -50 && h > -50) {
    // Format: T:xx.x H:yy
    snprintf(buf, sizeof(buf), "T:%.1f H:%.0f", t, h);
  } else {
    snprintf(buf, sizeof(buf), "T:-- H:--");
  }
  lcd.print(buf);
}

// ----------------- State implementations -----------------
void stDis() {
    setLED(ST_DIS);
    fanOff(); // ensure fan is off
    // Disabled state: do not poll sensors or water.
    // LCD already updated by caller when state changed.
    // ISR for start button will move to IDLE.
    // Nothing else to do here.
}

void stIdle() {
    setLED(ST_IDLE);

    // STOP button → DISABLED (active LOW because pull-up)
    if (readReg(BUTTON_STOP_PIN) == 0) {
        currentState = ST_DIS;
        logEvent("STOP pressed -> DISABLED");
        lcdUpdate(0,0,ST_DIS);
        return;
    }

    // Check water level (ADC)
    uint16_t water = getWaterLevel();
    if (water < WATER_THRESHOLD_LOW) {
        currentState = ST_ERR;
        logEvent("Low Water -> ERROR");
        lcdUpdate(0,0,ST_ERR);
        return;
    }

    // Vent control allowed in IDLE
    ventCtrl();

    // Update temp/humidity once per minute
    unsigned long now = millis();
    if (now - lastDisplayUpdateTime >= DISPLAY_UPDATE_INTERVAL) {
        lastDisplayUpdateTime = now;
        float t, h;
        getTH(t, h);
        lcdUpdate(t, h, ST_IDLE);
        logEvent("TH updated in IDLE");
    }

    // Temperature too high? → RUN
    {
        float t, h;
        getTH(t, h);
        if (t >= TEMP_THRESHOLD_HIGH) {
            currentState = ST_RUN;
            fanOn();
            logEvent("Temp High -> RUNNING");
            lcdUpdate(t, h, ST_RUN);
        }
    }
}

void stRun() {
    setLED(ST_RUN);
    fanOn();

    // STOP button → DISABLED
    if (readReg(BUTTON_STOP_PIN) == 0) {
        fanOff();
        currentState = ST_DIS;
        logEvent("STOP pressed -> DISABLED");
        lcdUpdate(0,0,ST_DIS);
        return;
    }

    // Check water level
    uint16_t water = getWaterLevel();
    if (water < WATER_THRESHOLD_LOW) {
        fanOff();
        currentState = ST_ERR;
        logEvent("Low Water -> ERROR");
        lcdUpdate(0,0,ST_ERR);
        return;
    }

    // Vent control allowed
    ventCtrl();

    // Update LCD once per minute
    unsigned long now = millis();
    if (now - lastDisplayUpdateTime >= DISPLAY_UPDATE_INTERVAL) {
        lastDisplayUpdateTime = now;
        float t, h;
        getTH(t, h);
        lcdUpdate(t, h, ST_RUN);
        logEvent("TH updated in RUNNING");
    }

    // Check temperature to go back to IDLE
    {
        float t, h;
        getTH(t, h);
        if (t <= TEMP_THRESHOLD_LOW) {
            fanOff();
            currentState = ST_IDLE;
            logEvent("Temp Low -> IDLE");
            lcdUpdate(t, h, ST_IDLE);
        }
    }
}

void stErr() {
    setLED(ST_ERR);
    fanOff();

    lcdUpdate(0,0,ST_ERR); // Displays WATER LOW on LCD

    // Vent control still allowed
    ventCtrl();

    // Reset button → go to IDLE only if water OK (active LOW)
    if (readReg(BUTTON_RESET_PIN) == 0) {
        uint16_t water = getWaterLevel();
        if (water >= WATER_THRESHOLD_LOW) {
            currentState = ST_IDLE;
            logEvent("RESET -> IDLE");
            lcdUpdate(0,0,ST_IDLE);
        } else {
            logEvent("RESET pressed but water still low");
        }
    }
}

// ----------------- Fan control -----------------
void fanOn() {
  writeReg(FAN_MOTOR_PIN, 1);
}

void fanOff() {
  writeReg(FAN_MOTOR_PIN, 0);
}

// ----------------- Vent control -----------------
void ventCtrl() {
  uint16_t pot = adcRead(VENT_POT_ADC_CHANNEL);
  int target = map(pot, 0, 1023, 0, MAX_VENT_POSITION);

  if (target > targetVentPosition) {
    ventMotor.step(STEPS_PER_INCREMENT);
    targetVentPosition++;
    logEvent("Vent +1");
  }
  else if (target < targetVentPosition) {
    ventMotor.step(-STEPS_PER_INCREMENT);
    targetVentPosition--;
    logEvent("Vent -1");
  }
}