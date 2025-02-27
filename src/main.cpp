#include <Arduino.h>
#include <FastLED.h>
#include <NimBLEDevice.h>
#include "NimBLEClient.h"
#include "esp_log.h"
#define TAG "BLE_CLIENT"

#ifndef ATOMS3
#define ATOMS3
#endif

#ifdef ATOMS3
#include "M5GFX.h"
#include "M5Unified.h"

M5Canvas canvas(&M5.Display);

#endif

// Cílový název zařízení
// #define TARGET_DEVICE_NAME "SIGMA SPEED 17197"
#define TARGET_DEVICE_NAME "SIGMA SPEED"

// UUID služby CSC a měřící charakteristiky
#define CSC_SERVICE_UUID "1816"
#define CSC_CHAR_UUID "2A5B"

// Doba skenování (0 = kontinuální)
#define SCAN_TIME_MS 0

const float WHEEL_CIRCUMFERENCE = 2.146; // [m]

// Globální proměnné pro uchování předchozího měření (pro výpočet rychlosti)
uint32_t previousCumulativeRevs = 0;
uint16_t previousLastEventTime = 0;
bool firstMeasurement = true;

// PWM definice
#define PWM_GPIO 38
#define PWM_CHANNEL 0
#define PWM_FREQUENCY 5000 // 5 kHz
#define PWM_RESOLUTION 10  // 8bit (0-255)

// WS2812 LED definice
#define LED_PIN 35
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

// Rychlostní limity
#define MIN_SPEED 3.0f  // km/h, pod tuto hodnotu PWM = 0%
#define MAX_SPEED 35.0f // km/h, při této hodnotě PWM = 100%

// Globální proměnné
static bool doConnect = false;
static const NimBLEAdvertisedDevice *advDevice;
NimBLEClient *pClient = nullptr;
NimBLERemoteCharacteristic *pCSCCharacteristic = nullptr;

float speed_kmph = 0.0f;
float pwm_percent = 0.0f;

void setPWM(float speed);
void drawGUI();

// Callback pro klientské události
class MyClientCallbacks : public NimBLEClientCallbacks
{
  void onConnect(NimBLEClient *pClient) override
  {
    ESP_LOGI(TAG, "Připojeno k zařízení");
  }

  void onDisconnect(NimBLEClient *pClient, int reason) override
  {
    ESP_LOGI(TAG, "Odpojeno, důvod: %d", reason);
    // Resetujeme příznaky a opětovně spustíme skenování
    doConnect = false;
    advDevice = nullptr;
    NimBLEDevice::getScan()->start(SCAN_TIME_MS, false, false);
  }

  void onPassKeyEntry(NimBLEConnInfo &connInfo) override
  {
    ESP_LOGI(TAG, "Vyžadován PassKey");
    // Pokud by zařízení vyžadovalo spárování s PINem, můžeme jej zde zadat.
    NimBLEDevice::injectPassKey(connInfo, 123456);
  }

  void onConfirmPasskey(NimBLEConnInfo &connInfo, uint32_t pass_key) override
  {
    ESP_LOGI(TAG, "Potvrzuji PassKey: %d", pass_key);
    NimBLEDevice::injectConfirmPasskey(connInfo, true);
  }

  void onAuthenticationComplete(NimBLEConnInfo &connInfo) override
  {
    if (!connInfo.isEncrypted())
    {
      ESP_LOGI(TAG, "Šifrované spojení selhalo, odpojuji se");
      NimBLEDevice::getClientByHandle(connInfo.getConnHandle())->disconnect();
    }
  }
};

// Callback pro skenování
class ScanCallbacks : public NimBLEScanCallbacks
{
  void onResult(const NimBLEAdvertisedDevice *advertisedDevice) override
  {
    // Kontrola, zda zařízení má název a zda se shoduje s cílovým
    if (advertisedDevice->haveName())
    {
      String devName = advertisedDevice->getName().c_str();
      ESP_LOGI(TAG, "Nalezeno zařízení: %s", devName.c_str());
      if (devName.startsWith(TARGET_DEVICE_NAME))
      {
        ESP_LOGI(TAG, "Nalezeno cílové zařízení!");
        advDevice = advertisedDevice;
        doConnect = true;
        NimBLEDevice::getScan()->stop();
      }
    }
  }
} scanCallbacks;

// Funkce pro dekódování CSC měření
void decodeCSCMeasurement(uint8_t *data, size_t length)
{
  if (length < 1)
  {
    ESP_LOGI(TAG, "Nebyla obdržena žádná data");
    return;
  }

  uint8_t flags = data[0];
  ESP_LOGI(TAG, "Flags: 0x%02X", flags);

  // Pokud je nastaven bit 0 – jsou přítomna kolečková data
  if (flags & 0x01)
  {
    if (length < 7)
    {
      ESP_LOGI(TAG, "Nedostatek bajtů pro kolečková data");
      return;
    }
    // Načtení kumulativního počtu otáček (4 bajty, little-endian)
    uint32_t currentCumulativeRevs = (uint32_t)data[1] | ((uint32_t)data[2] << 8) | ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 24);
    // Načtení času poslední události (2 bajty, little-endian; jednotky 1/1024 s)
    uint16_t currentLastEventTime = data[5] | (data[6] << 8);

    ESP_LOGI(TAG, "Kumulativní otáčky kola: %u", currentCumulativeRevs);
    ESP_LOGI(TAG, "Čas poslední události: %u (%0.2f sec)", currentLastEventTime, currentLastEventTime / 1024.0);

    // Výpočet rychlosti, pokud nejde o první měření
    if (!firstMeasurement)
    {
      uint32_t deltaRevs = currentCumulativeRevs - previousCumulativeRevs;

      // Výpočet rozdílu času s ošetřením přetečení (max. hodnota 2^16 = 65536)
      uint16_t deltaTime;
      if (currentLastEventTime >= previousLastEventTime)
      {
        deltaTime = currentLastEventTime - previousLastEventTime;
      }
      else
      {
        deltaTime = currentLastEventTime + (65536 - previousLastEventTime);
      }
      float deltaTimeSec = deltaTime / 1024.0;

      if (deltaTimeSec > 0)
      {
        float distance = deltaRevs * WHEEL_CIRCUMFERENCE; // ujetá vzdálenost v metrech
        float speed_mps = distance / deltaTimeSec;        // rychlost v m/s
        speed_kmph = speed_mps * 3.6;                     // rychlost v km/h

        if (speed_kmph > 99)
          speed_kmph = 99;

        if (speed_kmph > 0)
        {
          setPWM(speed_kmph);
        }

        ESP_LOGI(TAG, "Delta revolucí: %u", deltaRevs);
        ESP_LOGI(TAG, "Delta času: %0.2f sec", deltaTimeSec);
        ESP_LOGI(TAG, "Rychlost: %0.2f m/s, %0.2f km/h", speed_mps, speed_kmph);
      }
      else
      {
        ESP_LOGI(TAG, "Delta času je nula, nelze vypočítat rychlost.");
        setPWM(0);
        speed_kmph = 0;
      }
    }
    else
    {
      ESP_LOGI(TAG, "První měření, nelze spočítat rychlost.");
      firstMeasurement = false;
    }

    // Aktualizace předchozích hodnot pro další měření
    previousCumulativeRevs = currentCumulativeRevs;
    previousLastEventTime = currentLastEventTime;
  }

  // Případné zpracování dat pro šlapací (crank) otáčky, pokud je nastaven bit 1 (rozšíření dle specifikace)
  if (flags & 0x02)
  {
    if (length >= 11)
    {
      uint16_t crankRevs = data[7] | (data[8] << 8);
      uint16_t lastCrankEventTime = data[9] | (data[10] << 8);
      ESP_LOGI(TAG, "Crank otáčky: %u", crankRevs);
      ESP_LOGI(TAG, "Čas posledního crank eventu: %u (%0.2f sec)", lastCrankEventTime, lastCrankEventTime / 1024.0);
    }
  }
}

void setPWM(float speed)
{
  uint8_t pwm_value = 0;

  if (speed >= MIN_SPEED)
  {
    // Přepočet rychlosti na PWM výkon s kvadratickou nelinearitou
    float normalizedSpeed = (speed - MIN_SPEED) / (MAX_SPEED - MIN_SPEED);
    pwm_percent = 2.0f + (normalizedSpeed * normalizedSpeed) * 98.0f; // Kvadratická interpolace

    if (pwm_percent > 100.0f)
      pwm_percent = 100.0f;

    pwm_value = (uint8_t)((pwm_percent / 100.0f) * 1023);

    if (pwm_value > 1000)
      pwm_value = 1023;
  }
  else
  {
    pwm_value = 0;
  }

  // Nastavení PWM na GPIO5
  ledcWrite(PWM_CHANNEL, pwm_value);
  ESP_LOGI(TAG, "PWM value: %d (%.1f%%)", pwm_value, (pwm_value / 1023.0f) * 100);

#ifndef ATOMS3
  // Nastavení barvy WS2812
  if (pwm_value == 0)
  {
    leds[0] = CRGB::Black; // LED zhasnutá při PWM = 0
  }
  else
  {
    // Přechod od zelené (nízké PWM) přes oranžovou k červené (vysoké PWM)
    // Pokud pwm_value je mimo rozsah 1-255, omezíme jej
    uint8_t value = pwm_value < 1 ? 1 : (pwm_value > 255 ? 255 : pwm_value);
    uint8_t red, green;
    // Použijeme dvojúrovňovou interpolaci:
    // V rozsahu 1-127: přechod z čisté zelené (0,255,0) na oranžovou (255,165,0)
    // V rozsahu 128-255: přechod z oranžové (255,165,0) na červenou (255,0,0)
    if (value <= 127)
    {
      float factor = (value - 1) / 126.0f;
      red = (uint8_t)(factor * 255);
      green = (uint8_t)(255 - factor * (255 - 165));
    }
    else
    {
      float factor = (value - 128) / 127.0f;
      red = 255;
      green = (uint8_t)(165 - factor * 165);
    }
    leds[0] = CRGB(red, green, 0);
  }

  FastLED.show();
  ESP_LOGI(TAG, "LED color: R=%d, G=%d, B=0", leds[0].r, leds[0].g);
#endif
}

void setup()
{
#ifdef ATOMS3
  M5.begin();
  M5.Display.begin();

  M5.Display.setBrightness(35);
  // M5.Display.setRotation(1);
  // M5.Display.setTextColor(RED, BLACK);
  // M5.Display.setTextDatum(middle_center);
  // M5.Display.setFont(&fonts::FreeSans12pt7b);
  // M5.Display.setTextSize(1);

  // M5.Display.fillRect(0, 0, 240, 135, BLACK);
  // M5.Display.setCursor(10, 20);
  // M5.Display.print("Bat:");
#endif

  Serial.begin(115200);
  vTaskDelay(2000 / portTICK_PERIOD_MS); // Zpoždění pro připojení sériového monitoru
  ESP_LOGI(TAG, "Spouštím BLE klienta pro SIGMA SPEED 17197...");

  // Inicializace BLE
  NimBLEDevice::init("");
  // Vypnutí pairing/šifrování, pokud není potřeba – záleží na zařízení
  NimBLEDevice::setSecurityAuth(false, false, false);

  // Nastavení skenování
  NimBLEScan *pScan = NimBLEDevice::getScan();
  pScan->setScanCallbacks(&scanCallbacks, false);
  pScan->setInterval(45);
  pScan->setWindow(15);
  pScan->setActiveScan(true);
  pScan->start(SCAN_TIME_MS, false, false);

  // M5.Display.setFont(&fonts::FreeSans12pt7b);
  canvas.createSprite(M5.Display.width(), M5.Display.height());
  canvas.setTextColor(WHITE);

  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(PWM_GPIO, PWM_CHANNEL);

#ifndef ATOMS3
  // Inicializace WS2812C-2020 na GPIO35
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.clear();
  FastLED.show();
#endif
}

void loop()
{
  // Pokud bylo nalezeno cílové zařízení, pokusíme se o připojení
  if (doConnect && advDevice)
  {
    ESP_LOGI(TAG, "Pokus o připojení k cílovému zařízení...");
    pClient = NimBLEDevice::createClient();
    pClient->setClientCallbacks(new MyClientCallbacks(), false);

    if (!pClient->connect(advDevice))
    {
      ESP_LOGI(TAG, "Připojení se nezdařilo!");
      NimBLEDevice::getScan()->start(SCAN_TIME_MS, false, false);
      return;
    }

    ESP_LOGI(TAG, "Úspěšně připojeno!");
    // Hledáme službu CSC (UUID: 1816)
    NimBLERemoteService *pService = pClient->getService(CSC_SERVICE_UUID);
    if (!pService)
    {
      ESP_LOGI(TAG, "Služba CSC (1816) nebyla nalezena!");
      pClient->disconnect();
      return;
    }

    // Hledáme charakteristiku měření (UUID: 2A5B)
    pCSCCharacteristic = pService->getCharacteristic(CSC_CHAR_UUID);
    if (!pCSCCharacteristic)
    {
      ESP_LOGI(TAG, "Charakteristika CSC (2A5B) nebyla nalezena!");
      pClient->disconnect();
      return;
    }

    // Pokud podporuje notifikace, přihlásíme se
    if (pCSCCharacteristic->canNotify())
    {
      ESP_LOGI(TAG, "Přihlašuji se na notifikace z CSC charakteristiky...");
      pCSCCharacteristic->subscribe(true,
                                    [](NimBLERemoteCharacteristic *pChar, uint8_t *data, size_t length, bool isNotify)
                                    {
                                      ESP_LOGI(TAG, "Obdržena notifikace, délka: %d", length);
                                      decodeCSCMeasurement(data, length);
                                    });
    }
    else
    {
      ESP_LOGI(TAG, "Charakteristika nepodporuje notifikace!");
    }

    // Po úspěšném připojení resetujeme příznak
    doConnect = false;
  }

  // V hlavní smyčce lze případně zpracovávat další logiku
  drawGUI();

  // speed_kmph += 1;
  // if (speed_kmph > 40)
  //   speed_kmph = 0;

  // #endif
  delay(250);
}

void drawGUI()
{
  canvas.fillSprite(BLACK);
  // Draw speed gauge circle
  int centerX = canvas.width() / 2;
  int centerY = canvas.height() / 2;
  int radius = min(centerX, centerY) - 5;

  // Draw background circle
  // canvas.drawCircle(centerX, centerY, radius, WHITE);

  float comp_speed = speed_kmph > MAX_SPEED ? MAX_SPEED : speed_kmph;

  // Calculate angle based on current speed (0-270 degrees)
  float angle = (comp_speed / MAX_SPEED) * 270.0f;
  // angle = 270;
  // Draw filled arc from -45 to current angle
  if (speed_kmph > 0)
  {
    canvas.fillArc(centerX, centerY, radius - 17, radius + 5,
                   135,         // Start at -45 degrees (225 in fillArc coordinates)
                   135 + angle, // End at calculated angle
                   RED);
  }

  canvas.setFont(&fonts::Font7);
  canvas.setTextSize(1);
  canvas.setTextDatum(middle_center);
  canvas.drawString(String(speed_kmph, 0), centerX, centerY);
  // canvas.drawString("45", centerX, centerY);

  canvas.setFont(&fonts::FreeSans18pt7b);
  canvas.setTextSize(0.5);
  canvas.drawString(String(pwm_percent, 1), centerX, 107);
  // canvas.drawString("100", centerX - 1, 107);

  canvas.pushSprite(0, 0);
}