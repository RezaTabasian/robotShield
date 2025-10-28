#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

// پیکربندی پایه‌ها برای ATmega328PB
#define SR04_TRIG 10 // PD6
#define SR04_ECHO 11 // PD7
#define BUZZER 2     // PC2 (A2)
#define SW1 0        // PE0
#define SW2 2        // PD2
#define SW3 3        // PD3
#define SW4 4        // PD4
#define SW5 1        // PE1

// تعریف رجیسترهای پورت E
#define PORTE _SFR_IO8(0x02)
#define DDRE _SFR_IO8(0x01)
#define PINE _SFR_IO8(0x00)
#define PE0 0
#define PE1 1

// تنظیمات OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// متغیرهای گلوبال'
volatile float threshold_cm = 50.0;
bool auto_mode = false;
unsigned long last_measurement = 0;

// بافر برای فیلتر میانگین متحرک
#define FILTER_SIZE 5
float distance_buffer[FILTER_SIZE];
int buffer_index = 0;
bool buffer_initialized = false;

// پروتوتایپ توابع
void display_output(float value, const char *unit);
void display_threshold();
void display_auto_mode();
bool is_object_detected(float detection_threshold);
float read_filtered_distance();

void setup()
{
  // تنظیم پایه‌های سنسور
  pinMode(SR04_TRIG, OUTPUT);
  pinMode(SR04_ECHO, INPUT);

  pinMode(SW2, INPUT_PULLUP);
  pinMode(SW3, INPUT_PULLUP);
  pinMode(SW4, INPUT_PULLUP);

  DDRE &= ~((1 << PE0) | (1 << PE1)); // تنظیم به عنوان ورودی
  PORTE |= (1 << PE0) | (1 << PE1);   // فعال کردن پول-آپ

  // تنظیم بازر
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);

  // مقداردهی اولیه بافر فیلتر
  for (int i = 0; i < FILTER_SIZE; i++)
  {
    distance_buffer[i] = 0.0;
  }

  // راه‌اندازی OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    while (1)
      ; // در صورت خطا متوقف شود
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  // نمایش پیام اولیه
  display.setCursor(0, 0);
  display.print("Distance Sensor");
  display.setCursor(0, 16);
  display.print("Ready...");
  display.display();
  delay(1000);
}

float read_distance_cm()
{
  // تولید پالس تریگر
  digitalWrite(SR04_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(SR04_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(SR04_TRIG, LOW);

  // منتظر پایین رفتن پالس اکو
  while (digitalRead(SR04_ECHO) == LOW)
    ;

  // اندازه‌گیری مدت پالس بالا
  unsigned long start_time = micros();
  while (digitalRead(SR04_ECHO) == HIGH)
  {
    if (micros() - start_time > 30000)
      return 0; // تایم‌اوت 30ms
  }
  unsigned long duration = micros() - start_time;

  return (duration * 0.0343) / 2;
}

// تابع جدید: تشخیص جسم در فاصله مشخص
bool is_object_detected(float detection_threshold)
{
  float current_distance = read_distance_cm();

  // اگر فاصله معتبر باشد و کمتر از آستانه باشد
  if (current_distance > 0 && current_distance <= detection_threshold)
  {
    return true;
  }
  return false;
}

// تابع جدید: خواندن فاصله با فیلتر برای کاهش نویز
float read_filtered_distance()
{
  delay(100); // تاخیر 100 میلی‌ثانیه قبل از خواندن

  // خواندن فاصله جدید
  float new_distance = read_distance_cm();

  // اگر بافر هنوز مقداردهی اولیه نشده
  if (!buffer_initialized)
  {
    for (int i = 0; i < FILTER_SIZE; i++)
    {
      distance_buffer[i] = new_distance;
    }
    buffer_initialized = true;
  }

  // اضافه کردن مقدار جدید به بافر
  distance_buffer[buffer_index] = new_distance;
  buffer_index = (buffer_index + 1) % FILTER_SIZE;

  // محاسبه میانگین
  float sum = 0;
  int valid_count = 0;

  for (int i = 0; i < FILTER_SIZE; i++)
  {
    if (distance_buffer[i] > 0)
    { // فقط مقادیر معتبر
      sum += distance_buffer[i];
      valid_count++;
    }
  }

  if (valid_count > 0)
  {
    return sum / valid_count;
  }
  else
  {
    return 0; // اگر هیچ مقدار معتبری نبود
  }
}

void handle_buttons()
{
  // بررسی کلیدهای پورت E با رجیستر مستقیم
  if (!(PINE & (1 << PE0)))
  { // SW1 فشرده شده
    auto_mode = false;

    bool object_detected = is_object_detected(30.0);
    float dist = read_distance_cm();

    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Distance: ");
    display.print(dist, 1);
    display.println("cm");
    display.print("Object <30cm: ");
    display.println(object_detected ? "YES" : "NO");
    display.print("Filtered: ");
    display.print(read_filtered_distance(), 1);
    display.print("cm");
    display.display();

    delay(300);
  }
  else if (!(PINE & (1 << PE1)))
  { // SW5 فشرده شده
    auto_mode = true;
    display_auto_mode();
    delay(300);
  }

  // بررسی کلیدهای پورت D با توابع استاندارد
  else if (digitalRead(SW2) == LOW)
  {
    auto_mode = false;

    // استفاده از فیلتر برای اندازه‌گیری دقیق‌تر
    float dist = read_filtered_distance() / 2.54;

    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Filtered Distance:");
    display.setCursor(0, 16);
    display.print(dist, 1);
    display.println(" IN");
    display.setCursor(0, 32);
    display.print("Raw: ");
    display.print(read_distance_cm() / 2.54, 1);
    display.print(" IN");
    display.display();

    delay(300);
  }
  else if (digitalRead(SW3) == LOW)
  {
    threshold_cm += 1.0;
    display_threshold();
    delay(200);
  }
  else if (digitalRead(SW4) == LOW)
  {
    threshold_cm = (threshold_cm - 1.0 > 0) ? threshold_cm - 1.0 : 0;
    display_threshold();
    delay(200);
  }
}

void display_output(float value, const char *unit)
{
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Distance: ");
  display.print(value, 1);
  display.print(" ");
  display.println(unit);
  display.print("Threshold: ");
  display.print(threshold_cm, 0);
  display.print("cm");
  display.setCursor(0, 32);
  display.print("Auto: ");
  display.print(auto_mode ? "ON" : "OFF");
  display.display();
}

void display_threshold()
{
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Set Threshold:");
  display.setCursor(0, 16);
  display.print(threshold_cm, 0);
  display.print(" cm");
  display.setCursor(0, 32);
  display.print("Press SW5 for Auto");
  display.display();
}

void display_auto_mode()
{
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("AUTO MODE ACTIVE");
  display.print("Threshold: ");
  display.print(threshold_cm, 0);
  display.println("cm");
  display.setCursor(0, 32);
  display.print("Press any key");
  display.setCursor(0, 40);
  display.print("to exit");
  display.display();
}

void loop()
{
  handle_buttons();

  if (auto_mode)
  {
    unsigned long current_time = millis();
    if (current_time - last_measurement >= 100)
    {
      // استفاده از فاصله فیلتر شده در حالت اتوماتیک
      float dist = read_filtered_distance();

      display.clearDisplay();
      display.setCursor(0, 0);
      display.print("AUTO (Filtered): ");
      display.print(dist, 1);
      display.print("cm");
      display.setCursor(0, 16);
      display.print("Threshold: ");
      display.print(threshold_cm, 0);
      display.print("cm");

      // کنترل بازر با استفاده از تابع تشخیص جسم
      if (is_object_detected(threshold_cm))
      {
        digitalWrite(BUZZER, HIGH);
        display.setCursor(0, 32);
        display.print("ALARM: ON");
        display.setCursor(0, 40);
        display.print("Object Detected!");
      }
      else
      {
        digitalWrite(BUZZER, LOW);
        display.setCursor(0, 32);
        display.print("ALARM: OFF");
      }

      display.display();
      last_measurement = current_time;
    }
  }

  delay(10); // کاهش بار پردازنده
}