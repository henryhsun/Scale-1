// OLED libraries
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// load cell library
#include "HX711.h"

// wifi library
#include <WiFi.h>

// OLED config
constexpr int SCREEN_WIDTH = 128;
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
constexpr int zeroButtonPin = 27;
constexpr int modeButtonPin = 26;
constexpr int buzzerPin = 25;

// calibration constant via manual tuning w/known weight
const float calFactor = 734;

// tuning knobs
const float emaBig = 0.4;   // ema for large changes
const float emaMed = 0.7;   // ema for medium changes
const float emaSma = 0.95;  // ema for tiny changes/noise

const float threshold = 0.08;  // hysteresis threshold for movement
const int sma = 3;             // sma N value for scale.get_value

const float zClampPos = 0.2;  // positive z clamp value
const float zClampNeg = 0.3;  // negative z clamp value

const float timerStartG = 2.0;       // gram weight for timer to start
const unsigned long minFlowT = 800;  // ms without flow for timer to stop
const float minFlowG = 0.2;          // gram minimum flow change for timer to stop
const float sleepHold = 1000;        // ms to hold tare button for sleep mode

// timer
unsigned long tStart = 0;         // ms since boot
unsigned long flowStopTimer = 0;  // ms tracker to stop flow

// millis based timing
const unsigned long refreshAwake = 30;   // ms between executing loop
const unsigned long refreshSleep = 300;  // longer ms between executing loop while asleep to save power
const unsigned long debounce = 25;       // ms for debounce

// flowrate scaling
const float shotFlow = 25;
const float pourFlow = 50;

// function prototypes
void beep(int);                                                                                                             // buzzer beep
float quantize(float g);                                                                                                    // quantize values to nearest 0.1 g
float hysteresis(float read_g);                                                                                             // restrict screen updates if change is too small
float varZeroClamp(float g);                                                                                                // clamp values close to 0
void tare(unsigned long nowTime, float &gFilt, bool &running, unsigned long &flowStopTimer, float &time, bool &startOnce);  // tare scale
void hx711PowerDown();                                                                                                      // power down to save battery
void hx711PowerUp();
void convertTime(unsigned long time);  // timer display for minutes, seconds
float computeFlowrate(float gFilt, float &prevGFilt, unsigned long nowTime);

// FSM modes
enum Mode {
  MODE_POUR,     // 0: weight only
  MODE_SHOT,     // 1: weight + auto start-stop timer
  MODE_KITCHEN,  // 2: weight + cts timer
  MODE_SLEEP,    // 4: low power UX "off"
  MODE_COUNT     // 5: counts total number of modes for cycling
};

static Mode mode = MODE_POUR;  // default starting mode

// FSM mode update/draw function prototypes

void updatePour(float gFilt, bool &running, unsigned long nowTime, float &time, bool &startOnce);
void updateShot(float gFilt, bool &running, unsigned long nowTime, float &time, bool &startOnce, unsigned long &flowStopTimer, float &prevGFilt);
void updateKitchen(bool &running, float &time, bool &startOnce, unsigned long &flowStopTimer);

void drawPour(float gFilt, int minutes, int seconds, int milliseconds);
void drawShot(float gFilt, int minutes, int seconds, int milliseconds);
void drawKitchen(float grams, float gFilt);


void setup() {
  Serial.begin(115200);
  delay(200);

  // init UI pins
  pinMode(buzzerPin, OUTPUT);
  pinMode(zeroButtonPin, INPUT_PULLUP);
  pinMode(modeButtonPin, INPUT_PULLUP);

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
  pinMode(HX711_CLK, OUTPUT);
  digitalWrite(HX711_CLK, LOW);

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
  static float prevGFilt = 0;  // timer auto-stop helper

  // timer variables
  static bool running = false;  // state of timer, running or not
  static float time = 0;
  static bool startOnce = true;  // timer can only start once, in beginning

  static unsigned int minutes = 0;
  static unsigned int seconds = 0;
  static unsigned int milliseconds = 0;

  // millis based refresh
  static unsigned long lastTime = 0;
  unsigned long nowTime = millis();
  static unsigned long refresh = refreshAwake;

  // flowrate
  static float flowrate = 0;

  // sleep mode tare button detection
  static bool zeroWasPressed = false;
  static unsigned long zeroPressTime = 0;
  static bool asleep = false;

  bool zeroPressed = (digitalRead(zeroButtonPin) == LOW);  // pressed = HIGH = true, default = not pressed
  bool modePressed = (digitalRead(modeButtonPin) == LOW);
  static bool sleepArm = false;

  Serial.println(refresh); // debugging

    if (mode == MODE_SLEEP) {
    if (!sleepArm) {
      if (!zeroPressed && !modePressed) {
        sleepArm = true;
        display.ssd1306_command(SSD1306_DISPLAYOFF);
        refresh = refreshSleep;
        btStop();
        hx711PowerDown();
      }
      return;
    }

    if (zeroPressed || modePressed) {
      mode = MODE_POUR;
      display.ssd1306_command(SSD1306_DISPLAYON);
      beep(60);
      refresh = refreshAwake;
      hx711PowerUp();
    }
    return;
  }

  // detect when zero was first pressed
  if (zeroPressed && !zeroWasPressed) {
    zeroWasPressed = true;
    zeroPressTime = nowTime;
    asleep = false;
  }

  // detect how long zero was pressed
  if (zeroPressed && zeroWasPressed && !asleep) {
    if (nowTime - zeroPressTime >= sleepHold) {
      asleep = true;
      mode = MODE_SLEEP;
      display.ssd1306_command(SSD1306_DISPLAYOFF);
      sleepArm = false;
    }
  }

  if (!zeroPressed && zeroWasPressed) {
    zeroWasPressed = false;
  }

  // -------------- FSM mode swticher --------------
  static bool lastModeButton = false;

  static unsigned long lastModePress = 0;
  if (lastModeButton == false && modePressed == true) {
    if (nowTime - lastModePress > debounce) {  // debounce
      lastModePress = nowTime;

      // cycle modes
      mode = (Mode)((mode + 1) % MODE_COUNT);

      if (mode == MODE_SLEEP) mode = MODE_POUR;  // skip sleep

      beep(100);
    }
  }

  lastModeButton = modePressed;

  // zero on mode change
  static Mode prevMode = MODE_COUNT;
  if (mode != prevMode) {
    // mode just changed
    scale.tare();

    gFilt = 0.0f;
    running = false;
    time = 0.0f;
    startOnce = true;
    flowStopTimer = 0;

    prevMode = mode;
  }

  // millis based refresh rate based on refresh variable
  if (nowTime - lastTime >= refresh) {
    lastTime = nowTime;

    // Update OLED
    display.clearDisplay();

    // millis based zero function
    tare(nowTime, gFilt, running, flowStopTimer, time, startOnce);

    if (scale.is_ready()) {
      val = scale.get_value(1);
      grams = scale.get_units(sma);
    }

    // --------- raw reading processing -------------

    // adaptive EMA
    float err = fabsf(grams - gFilt);

    if (err > 1.0f) {
      gFilt = grams;  // snap immediately for large change
    } else {
      float ema = 0;
      if (err > 0.3f) { ema = emaBig; }  // moving fast
      else if (err > 0.1f) {
        ema = emaMed;
      }                       // moving medium
      else { ema = emaSma; }  // still/no change

      gFilt = ema * gFilt + (1.0f - ema) * grams;
    }

    gFilt = hysteresis(gFilt);    // controls when UI can change to ignore noise
    gFilt = varZeroClamp(gFilt);  // clamps to zero when close, UX stability
    gFilt = quantize(gFilt);      // quantize to fixed steps for 0.1 g accuracy

    flowrate = computeFlowrate(gFilt, prevGFilt, nowTime);

    // fsm non-blocking mode switching
    switch (mode) {
      case MODE_POUR:
        updatePour(gFilt, running, nowTime, time, startOnce);
        convertTime(time, minutes, seconds, milliseconds);
        drawPour(gFilt, minutes, seconds, milliseconds);
        drawFlowBar(flowrate, pourFlow);
        break;

      case MODE_SHOT:
        // auto-stop shot timer enabled
        updateShot(gFilt, running, nowTime, time, startOnce, flowStopTimer, prevGFilt);
        convertTime(time, minutes, seconds, milliseconds);
        drawShot(gFilt, minutes, seconds, milliseconds);
        drawFlowBar(flowrate, shotFlow);
        break;

      case MODE_KITCHEN:
        updateKitchen(running, time, startOnce, flowStopTimer);
        drawKitchen(grams, gFilt);
        break;

      case MODE_SLEEP:  // save power
        return;         //ignore display values, keep screen off
    }

    prevGFilt = gFilt;

    display.display();
  }
}

// -------------- FSM mode functions ------------------
void updatePour(float gFilt, bool &running, unsigned long nowTime, float &time, bool &startOnce) {
  if (running == false && startOnce == true && gFilt > timerStartG) {
    running = true;
    tStart = nowTime;
    time = 0.0f;
  }

  if (running == true) {
    time = nowTime - tStart;
  }
}

void updateShot(float gFilt, bool &running, unsigned long nowTime, float &time, bool &startOnce, unsigned long &flowStopTimer, float &prevGFilt) {
  if (running == false && startOnce == true && gFilt > timerStartG) {
    running = true;
    tStart = nowTime;
    time = 0.0f;
    startOnce = false;
    flowStopTimer = 0;
  }

  // auto stop checker
  if (running == true) {
    float deltaG = gFilt - prevGFilt;  // change in weight

    if (fabsf(deltaG) < minFlowG) {  // flow stopped, weight barely changing
      if (flowStopTimer == 0) {
        flowStopTimer = nowTime;                        // start counting flow stopped time
      } else if (nowTime - flowStopTimer > minFlowT) {  // if flow stop timer is running
        running = false;
        startOnce = false;
        flowStopTimer = 0;
      }
    } else {
      flowStopTimer = 0;
    }
  }

  // display time calculator
  if (running == true) {
    time = (nowTime - tStart) / 1000.0f;
  }

  // re-arm timer if cup is removed
  if (!running && gFilt < 1.0f) {
    startOnce = true;
  }
}

void updateKitchen(bool &running, float &time, bool &startOnce, unsigned long &flowStopTimer) {
  // disable timer, weight only
  running = false;
  time = 0.0f;
  startOnce = true;
  flowStopTimer = 0;
}

void drawPour(float gFilt, int minutes, int seconds, int milliseconds) {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("mode: pourover");

  // filtered grams
  display.setTextSize(2);
  display.setCursor(0, 16);
  display.print(gFilt, 1);
  display.println(" g");

  // time
  display.setTextSize(2);
  display.setCursor(0, 40);
  display.print(minutes);
  display.print(":");
  if (seconds < 10) {
    display.print("0");
  }
  display.print(seconds);
  display.print(".");
  display.print(milliseconds);

  display.setTextSize(1);
  display.setCursor(5,57);
  display.print("m");
  display.setCursor(41,57);
  display.print("s");
}

void drawShot(float gFilt, int minutes, int seconds, int milliseconds) {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("mode: shot");

  // filtered grams
  display.setTextSize(2);
  display.setCursor(0, 16);
  display.print(gFilt, 1);
  display.println(" g");

  // time
  display.setTextSize(2);
  display.setCursor(0, 40);
  display.print(minutes);
  display.print(":");
  if (seconds < 10) {
    display.print("0");
  }
  display.print(seconds);
  display.print(".");
  display.print(milliseconds);

  display.setTextSize(1);
  display.setCursor(5,57);
  display.print("m");
  display.setCursor(41,57);
  display.print("s");
}

void drawKitchen(float grams, float gFilt) {
  float oz = gFilt / 28.3495;
  float pounds = gFilt / 453.592;

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("mode: kitchen");

  // pounds
  display.setTextSize(2);
  display.setCursor(0, 16);
  display.print(gFilt, 1);
  display.print(" g");

  // pounds
  display.setTextSize(2);
  display.setCursor(0, 40);
  display.print(pounds, 2);

  // oz
  display.setTextSize(2);
  display.setCursor(65, 40);
  display.print(oz, 1);

  display.setTextSize(1);
  display.setCursor(0,57);
  display.print("lbs");
  display.setCursor(65,57);
  display.print("oz");
}


// ---------------
//    Functions
// ---------------

// beep
void beep(int ms = 15) {
  digitalWrite(buzzerPin, HIGH);
  delay(ms);
  digitalWrite(buzzerPin, LOW);
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
float varZeroClamp(float g) {
  if (g < 0 && fabsf(g) < zClampNeg) return 0.0f;
  if (g > 0 && fabsf(g) < zClampPos) return 0.0f;

  return g;
}

void tare(unsigned long nowTime, float &gFilt, bool &running, unsigned long &flowStopTimer, float &time, bool &startOnce) {
  static unsigned long lastZero = 0;
  static bool lastState = HIGH;

  bool state = digitalRead(zeroButtonPin);

  if (lastState == HIGH && state == LOW) {
    if (nowTime - lastZero > debounce) {
      lastZero = nowTime;

      // reset everything
      scale.tare();
      gFilt = 0;
      running = false;
      time = 0;
      flowStopTimer = 0;
      startOnce = true;
      beep(100);
    }
  }

  lastState = state;
}

void hx711PowerDown() {
  digitalWrite(HX711_CLK, HIGH);
  delayMicroseconds(80);
}

void hx711PowerUp() {
  digitalWrite(HX711_CLK, LOW);
  delayMicroseconds(80);

  delay(50);
}

void convertTime(unsigned long time, unsigned int &minutes, unsigned int &seconds, unsigned int &milliseconds) {
  milliseconds = (time % 1000) / 100;
  seconds = time / 1000 % 60;
  minutes = time / 60000;
}

float computeFlowrate(float gFilt, float &prevGFilt, unsigned long nowTime) {
  static unsigned long prevTime = 0;
  static bool inited = false;

  // init on first call
  if (!inited) {
    prevGFilt = gFilt;
    prevTime = nowTime;
    inited = true;
    return 0.0f;
  }

  unsigned long dtMs = nowTime - prevTime;
  if (dtMs == 0) return 0.0f;

  float dg = gFilt - prevGFilt;

  // grams per second
  float flow = dg / (dtMs / 1000.0f);

  // UX: ignore negative flow
  if (flow < 0.0f) flow = 0.0f;

  // update history
  prevGFilt = gFilt;
  prevTime = nowTime;

  return flow;
}

void drawFlowBar(float flowrate, float flowMax) {
  if (flowrate < 0) flowrate = 0;
  if (flowrate > flowMax) flowrate = flowMax;

  const int barW = 6;
  const int barX = 128 - barW;
  const int barHmax = 60;
  const int barBottomY = 63;

  int h = (int)((flowrate / flowMax) * barHmax);
  h = constrain(h, 0, barHmax);

  display.drawRect(barX, barBottomY - barHmax, barW, barHmax, SSD1306_WHITE);
  display.fillRect(barX, barBottomY - h, barW, h, SSD1306_WHITE);
}
