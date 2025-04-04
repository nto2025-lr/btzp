#include <FastLED.h>
#include <Servo.h>

// Настройки светодиодной матрицы
#define LED_PIN 6
#define NUM_LEDS 256
#define BRIGHTNESS 50
CRGB leds[NUM_LEDS];

// Настройки сервоприводов
#define SERVO1_PIN 9
#define SERVO2_PIN 10
Servo servo1;
Servo servo2;

// Настройки кнопки
#define BUTTON_PIN 2
bool buttonState = false;
bool lastButtonState = HIGH;
unsigned long pressStartTime = 0;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// Конфигурация матрицы 16x16
#define MATRIX_WIDTH 16
#define MATRIX_HEIGHT 16
#define MATRIX_TYPE 0

uint16_t XY(uint8_t x, uint8_t y) {
  if (x >= MATRIX_WIDTH || y >= MATRIX_HEIGHT) return 9999;
  
  if (MATRIX_TYPE == 0) {
    return (y % 2 == 0) ? (y * MATRIX_WIDTH) + x 
                        : (y * MATRIX_WIDTH) + (MATRIX_WIDTH - 1 - x);
  } 
  return (y * MATRIX_WIDTH) + x;
}

void setup() {
  // Инициализация светодиодов
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  
  // Инициализация сервоприводов
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo1.write(90);
  servo2.write(90);
  
  // Настройка кнопки
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void loop() {
  // Обработка кнопки с антидребезгом
  int reading = digitalRead(BUTTON_PIN);
  
  // Обновление состояния кнопки
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      
      // Управление сервоприводами
      if (buttonState == LOW) {
        servo1.write(0);
        servo2.write(180);
        pressStartTime = millis();
      } else {
        servo1.write(90);
        servo2.write(90);
      }
    }
  }
  lastButtonState = reading;

  // Управление светодиодной матрицей
  if (buttonState == LOW) {
    unsigned long pressDuration = millis() - pressStartTime;
    
    if (pressDuration < 5000) {
      animateDownArrow((millis()/200) % 4);
    } else {
      drawStaticDownArrow();
      animateBorder((millis()/50) % 128);
    }
  } else {
    animateUpArrow((millis()/200) % 4);
  }

  FastLED.show();
}

// Функции анимации (полностью сохранены из оригинального кода)
void animateUpArrow(int frame) {
  FastLED.clear();
  CRGB color = CRGB::Red;
  
  int offset = frame - 2;
  for (int i = 0; i < 5; i++) {
    int y = 12 + offset - i;
    if (y >= 0 && y < 16) {
      for (int x = 6; x <= 9; x++) {
        leds[XY(x, y)] = color;
      }
    }
  }
  for (int x = 4; x <= 11; x++) {
    int y = 13 + offset;
    if (y >= 0 && y < 16) leds[XY(x, y)] = color;
  }
}

void animateDownArrow(int frame) {
  FastLED.clear();
  CRGB color = CRGB::Green;
  
  int offset = frame - 2;
  for (int i = 0; i < 5; i++) {
    int y = 3 + offset + i;
    if (y >= 0 && y < 16) {
      for (int x = 6; x <= 9; x++) {
        leds[XY(x, y)] = color;
      }
    }
  }
  for (int x = 4; x <= 11; x++) {
    int y = 2 + offset;
    if (y >= 0 && y < 16) leds[XY(x, y)] = color;
  }
}

void drawStaticDownArrow() {
  CRGB color = CRGB::Green;
  
  for (int y = 7; y <= 9; y++) {
    for (int x = 6; x <= 9; x++) {
      leds[XY(x, y)] = color;
    }
  }
  for (int x = 4; x <= 11; x++) {
    leds[XY(x, 10)] = color;
  }
}

void animateBorder(int pos) {
  CRGB color = CRGB::Blue;
  
  for (int i = 0; i < 128; i++) {
    int currentPos = (pos + i) % 128;
    uint8_t brightness = (i < 16) ? 255 : max(0, 255 - (i-16)*15);
    
    if (currentPos < 32) {
      int x = currentPos % 16;
      leds[XY(x, 0)].nscale8_video(brightness);
    } else if (currentPos < 64) {
      int y = (currentPos - 32) % 16;
      leds[XY(15, y)].nscale8_video(brightness);
    } else if (currentPos < 96) {
      int x = 15 - (currentPos - 64) % 16;
      leds[XY(x, 15)].nscale8_video(brightness);
    } else {
      int y = 15 - (currentPos - 96) % 16;
      leds[XY(0, y)].nscale8_video(brightness);
    }
  }
}