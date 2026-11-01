#include <MD_MAX72XX.h>
#include <SPI.h>

#define HARDWARE_TYPE MD_MAX72XX::FC16_HW   
#define MAX_DEVICES 4                      
const uint8_t COLS = MAX_DEVICES * 8;      

const bool MIRROR_X = false;      
const bool FLIP_Y = false;        
const bool ENC_REVERSE = false;   

const uint8_t START_BRIGHTNESS = 3;       
const unsigned long TIMEOUT_MS = 1000;   

const uint8_t PIN_CS = D7;
const uint8_t PIN_ENC_A = D0;
const uint8_t PIN_ENC_B = D1;
const uint8_t PIN_ENC_SW = D2;  

MD_MAX72XX mx(HARDWARE_TYPE, PIN_CS, MAX_DEVICES);

uint8_t heights[COLS];
uint8_t brightness = START_BRIGHTNESS;
unsigned long lastFrame = 0;
bool gotData = false;


int8_t readEncoder() {
  static const int8_t table[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
  static uint8_t prev = 0xFF;
  static int8_t acc = 0;

  uint8_t cur = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  if (prev == 0xFF) {       
    prev = cur;
    return 0;
  }
  if (cur == prev) return 0;

  acc += table[(prev << 2) | cur];
  prev = cur;

  if (acc >= 4) {
    acc = 0;
    return ENC_REVERSE ? -1 : 1;
  }
  if (acc <= -4) {
    acc = 0;
    return ENC_REVERSE ? 1 : -1;
  }
  return 0;
}

uint8_t barBits(uint8_t h) {
  if (h > 8) h = 8;
  uint8_t v = (0xFF >> (8 - h)) & 0xFF;
  if (FLIP_Y) {
    uint8_t r = 0;
    for (uint8_t i = 0; i < 8; i++) {
      if (v & (1 << i)) r |= (1 << (7 - i));
    }
    v = r;
  }
  return v;
}

void drawBars() {
  mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
  for (uint8_t i = 0; i < COLS; i++) {
    uint8_t col = MIRROR_X ? i : (COLS - 1 - i);
    mx.setColumn(col, barBits(heights[i]));
  }
  mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
  mx.update();
}

void drawIdle() {
  for (uint8_t i = 0; i < COLS; i++) heights[i] = 1;
  drawBars();
}


bool readFrame() {
  static uint8_t buf[COLS];
  static uint8_t idx = 0;
  static bool inFrame = false;
  bool done = false;

  while (Serial.available()) {
    int b = Serial.read();
    if (b == 0xFF) {      
      idx = 0;
      inFrame = true;
      continue;
    }
    if (!inFrame) continue;

    buf[idx++] = (b > 8) ? 8 : b;
    if (idx >= COLS) {
      memcpy(heights, buf, COLS);
      inFrame = false;
      done = true;
    }
  }
  return done;
}


void setup() {
  Serial.begin(115200);
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_ENC_SW, INPUT_PULLUP);

  mx.begin();
  mx.control(MD_MAX72XX::INTENSITY, brightness);
  mx.clear();
  drawIdle();
}

void loop() {
  int8_t step = readEncoder();
  if (step != 0) {
    int16_t b = (int16_t)brightness + step;
    if (b < 0) b = 0;
    if (b > MAX_INTENSITY) b = MAX_INTENSITY;
    if (b != brightness) {
      brightness = (uint8_t)b;
      mx.control(MD_MAX72XX::INTENSITY, brightness);
    }
  }

  if (readFrame()) {
    gotData = true;
    lastFrame = millis();
    drawBars();
  } else if (gotData && millis() - lastFrame > TIMEOUT_MS) {
    gotData = false;
    drawIdle();
  }
}
