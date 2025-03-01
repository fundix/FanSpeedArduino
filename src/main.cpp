#ifndef ATOMS3
#define ATOMS3
#endif

#include <Arduino.h>
#ifndef ATOMS3
#include <FastLED.h>
#endif
#include <NimBLEDevice.h>
#include "NimBLEClient.h"
#include "esp_log.h"
#define TAG "BLE_CLIENT"

#ifdef ATOMS3
#include "M5GFX.h"
#include "M5Unified.h"

M5Canvas canvas(&M5.Display);

#define BTN1 41
#endif

#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
WebServer webServer(80);

// Target device name
// #define TARGET_DEVICE_NAME "SIGMA SPEED 17197"
#define TARGET_DEVICE_NAME "SIGMA SPEED"

// UUID of CSC service and measurement characteristic
#define CSC_SERVICE_UUID "1816"
#define CSC_CHAR_UUID "2A5B"

// Scan duration (0 = continuous)
#define SCAN_TIME_MS 0

const float WHEEL_CIRCUMFERENCE = 2.146; // [m]

// Global variables for storing previous measurement (for speed calculation)
uint32_t previousCumulativeRevs = 0;
uint16_t previousLastEventTime = 0;
bool firstMeasurement = true;

void setupWiFiClient();
void buttonLoop();
void shortPressed();
void longPressed();

// PWM definitions
#define PWM_GPIO 38
#define PWM_CHANNEL 0
#define PWM_FREQUENCY 10000 // 10 kHz
#define PWM_RESOLUTION 10   // 10bit (0-1023)

#ifndef ATOMS3
// WS2812 LED definitions
#define LED_PIN 35
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];
#endif

// Speed limits
#define MIN_SPEED 3.0f  // km/h, below this value PWM = 0%
#define MAX_SPEED 35.0f // km/h, at this value PWM = 100%
// Global variables
static bool doConnect = false;
static const NimBLEAdvertisedDevice *advDevice;
NimBLEClient *pClient = nullptr;
NimBLERemoteCharacteristic *pCSCCharacteristic = nullptr;

float speed_kmph = 0.0f;
float pwm_percent = 0.0f;

#ifdef ATOMS3
static unsigned long buttonPressStartTime = 0;
static bool buttonPressed = false;
static bool longButtonPressed = false;
static bool updateStarted = false;
#endif

void setPWM(float speed);
void drawGUI();

// Client event callbacks
class MyClientCallbacks : public NimBLEClientCallbacks
{
  void onConnect(NimBLEClient *pClient) override
  {
    ESP_LOGI(TAG, "Connected to device");
  }

  void onDisconnect(NimBLEClient *pClient, int reason) override
  {
    ESP_LOGI(TAG, "Disconnected, reason: %d", reason);
    // Reset flags and restart scanning
    doConnect = false;
    advDevice = nullptr;
    NimBLEDevice::getScan()->start(SCAN_TIME_MS, false, false);
  }

  void onPassKeyEntry(NimBLEConnInfo &connInfo) override
  {
    ESP_LOGI(TAG, "PassKey required");
    // If device requires pairing with PIN, we can enter it here
    NimBLEDevice::injectPassKey(connInfo, 123456);
  }

  void onConfirmPasskey(NimBLEConnInfo &connInfo, uint32_t pass_key) override
  {
    ESP_LOGI(TAG, "Confirming PassKey: %d", pass_key);
    NimBLEDevice::injectConfirmPasskey(connInfo, true);
  }

  void onAuthenticationComplete(NimBLEConnInfo &connInfo) override
  {
    if (!connInfo.isEncrypted())
    {
      ESP_LOGI(TAG, "Encrypted connection failed, disconnecting");
      NimBLEDevice::getClientByHandle(connInfo.getConnHandle())->disconnect();
    }
  }
};

// Scanning callback
class ScanCallbacks : public NimBLEScanCallbacks
{
  void onResult(const NimBLEAdvertisedDevice *advertisedDevice) override
  {
    // Check if device has name and if it matches target
    if (advertisedDevice->haveName())
    {
      String devName = advertisedDevice->getName().c_str();
      if (devName.length() > 0)
      {
        ESP_LOGI(TAG, "Device found: %s", devName.c_str());
      }
      if (devName.startsWith(TARGET_DEVICE_NAME))
      {
        ESP_LOGI(TAG, "Target device found!");
        advDevice = advertisedDevice;
        doConnect = true;
        NimBLEDevice::getScan()->stop();
      }
    }
  }
} scanCallbacks;

// Function for decoding CSC measurement
void decodeCSCMeasurement(uint8_t *data, size_t length)
{
  if (length < 1)
  {
    ESP_LOGI(TAG, "No data received");
    return;
  }

  uint8_t flags = data[0];
  ESP_LOGI(TAG, "Flags: 0x%02X", flags);

  // If bit 0 is set - wheel data is present
  if (flags & 0x01)
  {
    if (length < 7)
    {
      ESP_LOGI(TAG, "Not enough bytes for wheel data");
      return;
    }
    // Read cumulative wheel revolutions (4 bytes, little-endian)
    uint32_t currentCumulativeRevs = (uint32_t)data[1] | ((uint32_t)data[2] << 8) | ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 24);
    // Read last event time (2 bytes, little-endian; units 1/1024 s)
    uint16_t currentLastEventTime = data[5] | (data[6] << 8);

    ESP_LOGI(TAG, "Cumulative wheel revolutions: %u", currentCumulativeRevs);
    ESP_LOGI(TAG, "Last event time: %u (%0.2f sec)", currentLastEventTime, currentLastEventTime / 1024.0);

    // Calculate speed if not first measurement
    if (!firstMeasurement)
    {
      uint32_t deltaRevs = currentCumulativeRevs - previousCumulativeRevs;

      // Calculate time difference handling overflow (max value 2^16 = 65536)
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
        float distance = deltaRevs * WHEEL_CIRCUMFERENCE; // distance in meters
        float speed_mps = distance / deltaTimeSec;        // speed in m/s
        speed_kmph = speed_mps * 3.6;                     // speed in km/h

        if (speed_kmph > 99)
          speed_kmph = 99;

        if (speed_kmph > 0)
        {
          setPWM(speed_kmph);
        }

        ESP_LOGI(TAG, "Delta revolutions: %u", deltaRevs);
        ESP_LOGI(TAG, "Delta time: %0.2f sec", deltaTimeSec);
        ESP_LOGI(TAG, "Speed: %0.2f m/s, %0.2f km/h", speed_mps, speed_kmph);
      }
      else
      {
        ESP_LOGI(TAG, "Delta time is zero, cannot calculate speed.");
        setPWM(0);
        speed_kmph = 0;
      }
    }
    else
    {
      ESP_LOGI(TAG, "First measurement, cannot calculate speed.");
      firstMeasurement = false;
    }

    // Update previous values for next measurement
    previousCumulativeRevs = currentCumulativeRevs;
    previousLastEventTime = currentLastEventTime;
  }

  // Process crank data if bit 1 is set (extension per specification)
  if (flags & 0x02)
  {
    if (length >= 11)
    {
      uint16_t crankRevs = data[7] | (data[8] << 8);
      uint16_t lastCrankEventTime = data[9] | (data[10] << 8);
      ESP_LOGI(TAG, "Crank revolutions: %u", crankRevs);
      ESP_LOGI(TAG, "Last crank event time: %u (%0.2f sec)", lastCrankEventTime, lastCrankEventTime / 1024.0);
    }
  }
}
void setPWM(float speed)
{
  uint16_t pwm_value = 0;

  if (speed >= MIN_SPEED)
  {
    // Convert speed to PWM power with quadratic non-linearity
    float normalizedSpeed = (speed - MIN_SPEED) / (MAX_SPEED - MIN_SPEED);
    pwm_percent = 4.0f + (normalizedSpeed * normalizedSpeed) * 98.0f; // Quadratic interpolation

    if (pwm_percent > 100.0f)
      pwm_percent = 100.0f;

    pwm_value = (uint16_t)((pwm_percent / 100.0f) * 1023);
  }
  else
  {
    pwm_value = 0;
    pwm_percent = 0.0f;
  }

  // Set PWM on GPIO5
  ledcWrite(PWM_CHANNEL, pwm_value);
  ESP_LOGI(TAG, "PWM value: %d (%.1f%%)", pwm_value, (pwm_value / 1023.0f) * 100);

#ifndef ATOMS3
  // Set WS2812 color
  if (pwm_value == 0)
  {
    leds[0] = CRGB::Black; // LED off when PWM = 0
  }
  else
  {
    // Transition from green (low PWM) through orange to red (high PWM)
    // If pwm_value is out of 1-255 range, limit it
    uint8_t value = pwm_value < 1 ? 1 : (pwm_value > 255 ? 255 : pwm_value);
    uint8_t red, green;
    // Use two-level interpolation:
    // Range 1-127: transition from pure green (0,255,0) to orange (255,165,0)
    // Range 128-255: transition from orange (255,165,0) to red (255,0,0)
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
#endif

  Serial.begin(115200);
  vTaskDelay(2000 / portTICK_PERIOD_MS); // Delay for serial monitor connection
  ESP_LOGI(TAG, "Starting BLE client for SIGMA SPEED 17197...");

  // Initialize BLE
  NimBLEDevice::init("");
  // Disable pairing/encryption if not needed - depends on device
  NimBLEDevice::setSecurityAuth(false, false, false);

  // Setup scanning
  NimBLEScan *pScan = NimBLEDevice::getScan();
  pScan->setScanCallbacks(&scanCallbacks, false);
  pScan->setInterval(45);
  pScan->setWindow(15);
  pScan->setActiveScan(true);
  pScan->start(SCAN_TIME_MS, false, false);

  canvas.createSprite(M5.Display.width(), M5.Display.height());
  canvas.setTextColor(WHITE);

  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(PWM_GPIO, PWM_CHANNEL);

#ifndef ATOMS3
  // Initialize WS2812C-2020 on GPIO35
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.clear();
  FastLED.show();
#endif

#ifdef ATOMS3
  pinMode(BTN1, INPUT_PULLUP);
#endif

  static TimerHandle_t guiTimer = NULL;
  if (guiTimer == NULL)
  {
    guiTimer = xTimerCreate(
        "GUITimer",
        pdMS_TO_TICKS(250),
        pdTRUE, // Auto reload
        nullptr,
        [](TimerHandle_t xTimer)
        {
          drawGUI();
        });
    xTimerStart(guiTimer, 0);
  }
}

void loop()
{
  // If target device was found, attempt to connect
  if (doConnect && advDevice)
  {
    ESP_LOGI(TAG, "Attempting to connect to target device...");
    pClient = NimBLEDevice::createClient();
    pClient->setClientCallbacks(new MyClientCallbacks(), false);

    if (!pClient->connect(advDevice))
    {
      ESP_LOGI(TAG, "Connection failed!");
      NimBLEDevice::getScan()->start(SCAN_TIME_MS, false, false);
      return;
    }

    ESP_LOGI(TAG, "Successfully connected!");
    // Look for CSC service (UUID: 1816)
    NimBLERemoteService *pService = pClient->getService(CSC_SERVICE_UUID);
    if (!pService)
    {
      ESP_LOGI(TAG, "CSC service (1816) not found!");
      pClient->disconnect();
      return;
    }

    // Look for measurement characteristic (UUID: 2A5B)
    pCSCCharacteristic = pService->getCharacteristic(CSC_CHAR_UUID);
    if (!pCSCCharacteristic)
    {
      ESP_LOGI(TAG, "CSC characteristic (2A5B) not found!");
      pClient->disconnect();
      return;
    }

    // If notifications are supported, subscribe
    if (pCSCCharacteristic->canNotify())
    {
      ESP_LOGI(TAG, "Subscribing to CSC characteristic notifications...");
      pCSCCharacteristic->subscribe(true,
                                    [](NimBLERemoteCharacteristic *pChar, uint8_t *data, size_t length, bool isNotify)
                                    {
                                      ESP_LOGI(TAG, "Notification received, length: %d", length);
                                      decodeCSCMeasurement(data, length);
                                    });
    }
    else
    {
      ESP_LOGI(TAG, "Characteristic doesn't support notifications!");
    }

    // After successful connection, reset flag
    doConnect = false;
  }

  // Main loop can process additional logic here

  delay(10);

  buttonLoop();
}

void buttonLoop()
{
  if (digitalRead(BTN1) == LOW)
  { // Tlačítko stisknuto (aktivní LOW)
    if (!buttonPressed)
    {
      buttonPressed = true;
      buttonPressStartTime = millis();
    }
    // Pokud držíte tlačítko déle než 2 sekundy a ještě nebyl detekován dlouhý stisk
    else if (!longButtonPressed && (millis() - buttonPressStartTime > 2000))
    {
      longButtonPressed = true;
      longPressed(); // Zavoláme funkci pro dlouhý stisk
    }
  }
  else
  { // Tlačítko uvolněno
    if (buttonPressed)
    {
      if (!longButtonPressed)
      {
        shortPressed(); // Zavoláme funkci pro krátký stisk
      }
      // Resetujeme stav tlačítka
      buttonPressed = false;
      longButtonPressed = false;
    }
  }
}

void shortPressed()
{
  ESP_LOGI(TAG, "Short button press");
}

void longPressed()
{
  ESP_LOGI(TAG, "Long button press");

  if (updateStarted)
    return;

  NimBLEDevice::getScan()->stop();
  NimBLEDevice::deinit(true);
  setupWiFiClient();

  updateStarted = true;
}

void drawGUI()
{
  canvas.fillSprite(BLACK);
  // Draw speed gauge circle
  int centerX = canvas.width() / 2;
  int centerY = canvas.height() / 2;
  int radius = min(centerX, centerY) - 5;

  if (updateStarted)
  {
    canvas.fillArc(centerX, centerY, 58, 90, 0, 360, GREEN);
    canvas.setTextSize(0.65);
    canvas.setTextDatum(middle_center);
    canvas.drawString("Updating", centerX, centerY - 11);

    if (WiFi.status() == WL_CONNECTED)
    {
      IPAddress IP = WiFi.localIP();

      // Vypsání IP na displej (M5Canvas)
#ifdef ATOMS3
      canvas.setTextSize(0.5);
      canvas.setTextDatum(middle_center);
      canvas.drawString(IP.toString().c_str(), centerX, centerY + 11);
#endif
    }

    canvas.pushSprite(0, 0);
    return;
  }
  else
  {
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
    canvas.drawString(String(pwm_percent, 0), centerX, 107);
    // canvas.drawString("100", centerX - 1, 107);

    canvas.pushSprite(0, 0);
  }
}

void setupWiFiClient()
{
  // Připojení k WiFi síti
  WiFi.begin("Vivien", "Bionicman123"); // Nahraď SSID a heslo správnými údaji

  ESP_LOGI(TAG, "Connecting to WiFi...");

  // Čekání na připojení
  int timeout = 20; // Maximální čas připojení (10 sekund)
  while (WiFi.status() != WL_CONNECTED && timeout > 0)
  {
    delay(500);
    ESP_LOGI(TAG, ".");
    timeout--;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    ESP_LOGI(TAG, "Connected to WiFi!");
    IPAddress IP = WiFi.localIP();
    ESP_LOGI(TAG, "Client IP address: %s", IP.toString().c_str());

    webServer.on("/", HTTP_GET, [&webServer]()
                 {
    webServer.sendHeader("Connection", "close");
    webServer.send(200, "text/html",
    "<form method='POST' action='/update' enctype='multipart/form-data'>"
    "<input type='file' name='update'>"
    "<input type='submit' value='Update'>"
    "</form>"); });

    webServer.on("/update", HTTP_POST, [&webServer]()
                 {
    webServer.sendHeader("Connection", "close");
    webServer.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart(); }, []()
                 {
    HTTPUpload& upload = webServer.upload();
    if (upload.status == UPLOAD_FILE_START) {
    ESP_LOGI(TAG, "Update: %s", upload.filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Update.printError(Serial);
    }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    }
    } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      ESP_LOGI(TAG, "Update Success: %u bytes", upload.totalSize);
    } else {
      Update.printError(Serial);
    }
    } });

    webServer.begin();
  }
  else
  {
    ESP_LOGI(TAG, "Failed to connect to WiFi!");
  }
}