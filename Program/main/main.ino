// OLED libraries
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// load cell library
#include "HX711.h"

// OLED config
constexpr int SCREEN_WIDTH  = 128;
constexpr int SCREEN_HEIGHT = 64;
constexpr int OLED_SDA = 21;
constexpr int OLED_SCL = 22;
constexpr uint8_t OLED_ADDRESS = 0x3C;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// HX711 config
// DAT/DOUT is output from HX711 to ESP32, CLK/SCK is clock from ESP32 to HX711
constexpr int HX711_DAT = 19;
constexpr int HX711_CLK = 18;

HX711 scale;

// UI pins
constexpr int ZERO_BUTTON_PIN = 27;
constexpr int MODE_PIN = 26;
constexpr int BUZZER_PIN = 25;

// calibration constant via manual tuning w/known weight
const float calFactor = 734;

// -------------------------
//    function prototypes
// -------------------------

void beep(int); // buzzer beep
float quantize(float g); // quantize values to nearest 0.1 g
float hysteresis(float read_g); // restrict screen updates if change is too small
float varZeroClamp(float g);

// tuning knobs
const float emaBig = 0.4; // ema for large changes
const float emaMed = 0.7; // ema for medium changes
const float emaSma = 0.95; // ema for tiny changes/noise
const float threshold = 0.08; // hysteresis threshold for movement
const int sma = 3; // sma N value for scale.get_value
const float zClampPos = 0.2; // positive z clamp value
const float zClampNeg = 0.3; // negative z clamp value
const float timerStartG = 2.0; // gram weight for timer to start
unsigned long minFlowT = 800; // ms without flow for timer to stop
const float minFlowG = 0.2; // gram minimum flow change for timer to stop

// timer
unsigned long tStart = 0; // ms since boot
unsigned long flowStopTimer = 0; // ms tracker to stop flow

void setup() {
  Serial.begin(115200);
  delay(200);

  // init UI pins
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(ZERO_BUTTON_PIN, INPUT_PULLUP);
  pinMode(MODE_PIN, INPUT_PULLUP);

  // start I2C
  Wire.begin(OLED_SDA, OLED_SCL);

  // init OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("SSD1306 init failed");
    while (true) { delay(1000); }
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Init HX711
  scale.begin(HX711_DAT, HX711_CLK);
  scale.set_gain(128);
  scale.set_scale(calFactor);

  // Startup screen
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("Scale (1)");
  display.display();
  beep(120);
  delay(800);

  // Tare on startup (make sure nothing is on the scale)
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Taring...");
  display.display();

  scale.tare();
  beep(200);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Ready.");
  display.display();
  delay(300);
  
}

void loop() {
  // define weigh variables
  long val = 0;
  float grams = 0.0f;
  static float gFilt = 0;
  static float prevGFilt = 0; // timer auto-stop derivative

  // timer variables
  static bool running = false; // state of timer, running or not
  static float time = 0;
  static bool startOnce = true; // timer can only start once, in beginning

  // Button is INPUT_PULLUP => pressed = LOW
  if (digitalRead(ZERO_BUTTON_PIN) == LOW) {
    // Basic debounce
    delay(25);
    if (digitalRead(ZERO_BUTTON_PIN) == LOW) {
      scale.tare();
      gFilt = 0;
      running = false;
      time = 0;
      flowStopTimer = 0;
      startOnce = true;
      beep(150);

      // Wait for release so it doesn't spam tare
      while (digitalRead(ZERO_BUTTON_PIN) == LOW) {
        delay(10);
      }
    }
  }

  if (scale.is_ready()) {
    val = scale.get_value(1);
    grams = scale.get_units(sma);
  }

  // raw reading processing

  // adaptive EMA
  float err = fabsf(grams - gFilt);

  if(err > 1.0f) {
    gFilt = grams; // snap immediately for large change
  }
  else {
    float ema = 0;
    if (err > 0.3f)         {ema = emaBig;}  // moving fast
    else if (err > 0.1f)   {ema = emaMed;}  // moving medium
    else                    {ema = emaSma;} // still/no change

    gFilt = ema * gFilt + (1.0f - ema) * grams;
  }

  gFilt = hysteresis(gFilt); // Controls when UI can change to ignore noise
  gFilt = varZeroClamp (gFilt);
  gFilt = quantize(gFilt); // quantize to fixed steps
  
  if (running == true) {
    float deltaG = gFilt - prevGFilt; // change in weight

    if (deltaG < minFlowG) { // flow stopped, weight barely changing
      if (flowStopTimer == 0) {
        flowStopTimer = millis(); // start counting flow stopped time
      } else if (millis() - flowStopTimer > minFlowT){ // if flow stop timer is running alr
        running = false;
        startOnce = false;
        flowStopTimer = 0;
      }
    } else {
        flowStopTimer = 0;
    }
  }
  
  prevGFilt = gFilt;

  // timer
  if (gFilt > timerStartG && running == false) {
    running = true;
    tStart = millis();
    flowStopTimer = 0;
  }

  if (running == true && startOnce == true) {
    time = (millis() - tStart)/1000.0f;
  }

  // Update OLED
  display.clearDisplay();

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Raw:");

  display.setTextSize(1);
  display.setCursor(0, 16);
  display.print(val);

  display.setTextSize(2);
  display.setCursor(55, 16);
  display.print(grams,2);

  display.setTextSize(2);
  display.setCursor(0, 40);
  display.print(gFilt,1);

  display.setTextSize(2);
  display.setCursor(55,40);
  display.print(time,1);

  display.display();

  delay(30);
}

// ---------------
//    Functions
// ---------------

// beep
void beep(int ms = 15) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(ms);
  digitalWrite(BUZZER_PIN, LOW);
}

// quantize values to nearest 0.1 g
float quantize(float g) {
  return roundf(g * 10.0f) / 10.0f;
}

// restrict screen updates if change is too small
float hysteresis(float read_g) {
  static float shown_g = 0.00f;
  if (fabsf(read_g - shown_g) >= threshold) {
    shown_g = read_g;
  }
  return shown_g;
}

// locks values close enough to 0.0 to display 0.0 for UX
float varZeroClamp(float g){
  if (g < 0 && fabsf(g) < zClampNeg) return 0.0f;
  if (g > 0 && fabsf(g) < zClampPos) return 0.0f;

  return g;
}