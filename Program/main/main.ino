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
const float calFactor = 748;

// -------------------------
//    function prototypes
// -------------------------

void beep(int); // buzzer beep
float quantize(float g); // quantize values to nearest 0.1 g
float hysteresis(float read_g); // restrict screen updates if change is too small

// tuning knobs
float emaBig = 0.4;
float emaMed = 0.7;
float emaSma = 0.95;
float threshold = 0.08;
float sma = 3;
float zClamp = 0.08;

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

  // Button is INPUT_PULLUP => pressed = LOW
  if (digitalRead(ZERO_BUTTON_PIN) == LOW) {
    // Basic debounce
    delay(25);
    if (digitalRead(ZERO_BUTTON_PIN) == LOW) {
      scale.tare();
      gFilt = 0;
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
  gFilt = zeroClamp (gFilt);
  gFilt = quantize(gFilt); // quantize to fixed steps

  // Update OLED
  display.clearDisplay();

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Raw:");

  // display.setTextSize(1);
  // display.setCursor(0, 16);
  // display.print(raw);

  display.setTextSize(1);
  display.setCursor(0, 16);
  display.print(val);

  display.setTextSize(2);
  display.setCursor(45, 16);
  display.print(grams,2);

  display.setTextSize(2);
  display.setCursor(0, 40);
  display.print(gFilt,1);

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
float zeroClamp(float g){
  if (fabsf(g) < zClamp) return 0.0f;

  return g;
}