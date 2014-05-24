#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <avr/pgmspace.h>

#define LED_PIN  5

const uint32_t GREEN_COLOR = Adafruit_NeoPixel::Color(0, 255, 0);
const uint32_t RED_COLOR = Adafruit_NeoPixel::Color(255, 0, 0);
const uint32_t BLUE_COLOR = Adafruit_NeoPixel::Color(0, 0, 255);

void setMappedLed(uint16_t num, uint32_t color);
void solid(uint32_t color);
void blink(uint32_t color, uint16_t time);
void pulse(uint32_t color, uint16_t period);
void converge(uint32_t color, uint16_t time, uint16_t num);
void randomPattern(uint16_t time);
void off();

// LED Mapping
#define NUM_MAPPED_LEDS 122

#define LEFT_MAP_START 0
#define LEFT_START 0
#define LEFT_LENGTH 41

#define TOP_FRONT_MAP_START (LEFT_MAP_START + LEFT_LENGTH + 8)
#define TOP_FRONT_START (LEFT_START + LEFT_LENGTH)
#define TOP_FRONT_LENGTH 32

#define TOP_BACK_MAP_START (LEFT_MAP_START + LEFT_LENGTH)
#define TOP_BACK_START (TOP_FRONT_START + TOP_FRONT_LENGTH)
#define TOP_BACK_LENGTH 40

#define RIGHT_MAP_START (TOP_BACK_MAP_START + TOP_BACK_LENGTH + 12)
#define RIGHT_START (TOP_BACK_START + TOP_BACK_LENGTH)
#define RIGHT_LENGTH 29

#define NUM_LEDS (LEFT_LENGTH + TOP_FRONT_LENGTH + TOP_BACK_LENGTH + RIGHT_LENGTH)

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setMappedLed(uint16_t num, uint32_t color) {
  if (num >= LEFT_MAP_START && num < LEFT_MAP_START + LEFT_LENGTH) {
    strip.setPixelColor(LEFT_START + (num - LEFT_MAP_START), color);
  }
  if (num >= TOP_FRONT_MAP_START && num < TOP_FRONT_MAP_START + TOP_FRONT_LENGTH) {
    strip.setPixelColor(TOP_FRONT_START + (TOP_FRONT_LENGTH - (num - TOP_FRONT_MAP_START) - 1), color);
  }
  if (num >= TOP_BACK_MAP_START && num < TOP_BACK_MAP_START + TOP_BACK_LENGTH) {
    strip.setPixelColor(TOP_BACK_START + (num - TOP_BACK_MAP_START), color);
  }
  if (num >= RIGHT_MAP_START && num < RIGHT_MAP_START + RIGHT_LENGTH) {
    strip.setPixelColor(RIGHT_START + (num - RIGHT_MAP_START), color);
  }
}

uint32_t scaleBrightness(uint32_t color, float value) {
  uint8_t r = ((float)(color >> 16)) * value;
  uint8_t g = ((float)((color >> 8) & 0xFF)) * value;
  uint8_t b = ((float)(color & 0xFF)) * value;
  return Adafruit_NeoPixel::Color(r, g, b);
}

void solid(uint32_t color) {
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, color);
  }
}

void blink(uint32_t color, uint16_t time) {
  static uint64_t lastChangeTime = 0;
  static bool on = false;

  uint64_t currTime = millis();
  if (currTime - lastChangeTime > time) {
    on = !on;
    if (on) {
      solid(color);
    }
    else {
      off();
    }
    lastChangeTime = currTime;
  }
}

/**
 * Sets the lights to a solid color in a sine wave proportional to the time since the Arduino started.
 */
void pulse(uint32_t color, uint16_t period) {
  float value = (sin(((float) (millis() % period) / (float) period) * 2.0 * PI) + 1.0) / 2.0;
  //  uint8_t r = ((float)(color >> 16)) * value;
  //  uint8_t g = ((float)((color >> 8) & 0xFF)) * value;
  //  uint8_t b = ((float)(color & 0xFF)) * value;

  //  solid(Adafruit_NeoPixel::Color(r, g, b));
  solid(scaleBrightness(color, value));
}

void converge(uint32_t color, uint64_t time, uint16_t num) {
  static uint16_t i = 0;
  static uint64_t lastChangeTime = 0;
  uint64_t currTime = millis();

  if (currTime - lastChangeTime > time) {
    strip.clear();
    i++;
    i %= NUM_MAPPED_LEDS / 2;

    for (uint16_t l = 0; l < num; l++) {
      uint32_t c = scaleBrightness(color, (1.0 / (float) num) * l);
      uint16_t pos = num - l;
      setMappedLed((i - pos) % NUM_MAPPED_LEDS, c);
      setMappedLed(((NUM_MAPPED_LEDS - i) + pos) % NUM_MAPPED_LEDS, c);
    }
    lastChangeTime = currTime;
  }
}

void diverge(uint32_t color, uint64_t time, uint16_t num) {
  static uint16_t i = 0;
  static uint64_t lastChangeTime = 0;
  uint64_t currTime = millis();
  static uint16_t center = NUM_MAPPED_LEDS / 2;

  if (currTime - lastChangeTime > time) {
    strip.clear();
    i++;
    i %= NUM_MAPPED_LEDS / 2;

    for (uint16_t l = 0; l < num; l++) {
      uint32_t c = scaleBrightness(color, (1.0 / (float) num) * l);
      uint16_t pos = num - l;
      setMappedLed((center - i) + pos % NUM_MAPPED_LEDS, c);
      setMappedLed((center + i) - pos % NUM_MAPPED_LEDS, c);
    }
    lastChangeTime = currTime;
  }
}

void randomPattern(uint64_t time) {
  static uint64_t lastChangeTime = 0;

  uint64_t currTime = millis();
  if (currTime - lastChangeTime > time) {
    for (int i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, random(0xFFFFFF));
    }
    lastChangeTime = currTime;
  }
}

void off() {
  strip.clear();
}

void setup() {

  delay(1000);

  Serial.begin(9600);
  Serial.println("/=========================================\\");
  Serial.println("|             Robots By The C             |");
  Serial.println("| Arduino Micro LED Strip Controller V1.0 |");
  Serial.println("\\=========================================/");

  DDRF &= B00001100; // Set pins A0 - A5 to input
  PORTF |= B11110011; // turn on the pullups

  randomSeed(analogRead(A6)); // Initialize random # generator on empty analog pin

  strip.begin();
  strip.show();
}

const uint64_t BLINK_TIME          = 200;
const uint64_t PULSE_PERIOD        = 500;
const uint64_t CONVERGE_TIME       =  10;
const uint8_t  CONVERGE_NUM        =  15;
const uint64_t DIVERGE_TIME        =  10;
const uint8_t  DIVERGE_NUM         =  15;
const uint64_t RANDOM_PATTERN_TIME =  10;

const uint8_t DISABLE_CODE        = 0x3F;
const uint8_t SOLID_RED_CODE      = 0x00;
const uint8_t SOLID_BLUE_CODE     = 0x01;
const uint8_t SOLID_GREEN_CODE    = 0x02;
const uint8_t BLINK_RED_CODE      = 0x03;
const uint8_t BLINK_BLUE_CODE     = 0x04;
const uint8_t BLINK_GREEN_CODE    = 0x05;
const uint8_t PULSE_RED_CODE      = 0x06;
const uint8_t PULSE_BLUE_CODE     = 0x07;
const uint8_t PULSE_GREEN_CODE    = 0x08;
const uint8_t CONVERGE_RED_CODE   = 0x09;
const uint8_t CONVERGE_BLUE_CODE  = 0x0A;
const uint8_t CONVERGE_GREEN_CODE = 0x0B;
const uint8_t DIVERGE_RED_CODE    = 0x0C;
const uint8_t DIVERGE_BLUE_CODE   = 0x0D;
const uint8_t DIVERGE_GREEN_CODE  = 0x0E;
const uint8_t RANDOM_PATTERN_CODE = 0x0F;

uint8_t code = DISABLE_CODE;
bool animated = false;

void loop() {

  uint8_t input = PINF;
  input = (input >> 2) | (input & 0x3);

  if (input != code || animated) {
    code = input;
    switch (code) {
      case SOLID_RED_CODE:
        animated = false;
        solid(RED_COLOR);
        break;
      case SOLID_BLUE_CODE:
        animated = false;
        solid(BLUE_COLOR);
        break;
      case SOLID_GREEN_CODE:
        animated = false;
        solid(GREEN_COLOR);
        break;
      case BLINK_RED_CODE:
        animated = true;
        blink(RED_COLOR, BLINK_TIME);
        break;
      case BLINK_BLUE_CODE:
        animated = true;
        blink(BLUE_COLOR, BLINK_TIME);
        break;
      case BLINK_GREEN_CODE:
        animated = true;
        blink(GREEN_COLOR, BLINK_TIME);
        break;
      case PULSE_RED_CODE:
        animated = true;
        pulse(RED_COLOR, PULSE_PERIOD);
        break;
      case PULSE_BLUE_CODE:
        animated = true;
        pulse(BLUE_COLOR, PULSE_PERIOD);
        break;
      case PULSE_GREEN_CODE:
        animated = true;
        pulse(GREEN_COLOR, PULSE_PERIOD);
        break;
      case CONVERGE_RED_CODE:
        animated = true;
        converge(RED_COLOR, CONVERGE_TIME, CONVERGE_NUM);
        break;
      case CONVERGE_BLUE_CODE:
        animated = true;
        converge(BLUE_COLOR, CONVERGE_TIME, CONVERGE_NUM);
        break;
      case CONVERGE_GREEN_CODE:
        animated = true;
        converge(GREEN_COLOR, CONVERGE_TIME, CONVERGE_NUM);
        break;
      case DIVERGE_RED_CODE:
        animated = true;
        diverge(RED_COLOR, DIVERGE_TIME, DIVERGE_NUM);
        break;
      case DIVERGE_BLUE_CODE:
        animated = true; 
        diverge(BLUE_COLOR, DIVERGE_TIME, DIVERGE_NUM);
        break;
      case DIVERGE_GREEN_CODE:
        animated = true;
        diverge(GREEN_COLOR, DIVERGE_TIME, DIVERGE_NUM);
        break;
      case RANDOM_PATTERN_CODE:
        animated = true;
        randomPattern(RANDOM_PATTERN_TIME);
        break;
      default:
      case DISABLE_CODE:
        animated = false;
        off();
        break;
    }
    strip.show();
  }
  delay(4);
}
