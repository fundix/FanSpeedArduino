#include <Arduino.h>
#include <NimBLEDevice.h>
#include "NimBLEClient.h"

// Cílový název zařízení
#define TARGET_DEVICE_NAME "SIGMA SPEED 17197"

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

// Globální proměnné
static bool doConnect = false;
static const NimBLEAdvertisedDevice *advDevice;
NimBLEClient *pClient = nullptr;
NimBLERemoteCharacteristic *pCSCCharacteristic = nullptr;

// Callback pro klientské události
class MyClientCallbacks : public NimBLEClientCallbacks
{
  void onConnect(NimBLEClient *pClient) override
  {
    Serial.println("Připojeno k zařízení");
  }

  void onDisconnect(NimBLEClient *pClient, int reason) override
  {
    Serial.print("Odpojeno, důvod: ");
    Serial.println(reason);
    // Resetujeme příznaky a opětovně spustíme skenování
    doConnect = false;
    advDevice = nullptr;
    NimBLEDevice::getScan()->start(SCAN_TIME_MS, false, false);
  }

  void onPassKeyEntry(NimBLEConnInfo &connInfo) override
  {
    Serial.println("Vyžadován PassKey");
    // Pokud by zařízení vyžadovalo spárování s PINem, můžeme jej zde zadat.
    NimBLEDevice::injectPassKey(connInfo, 123456);
  }

  void onConfirmPasskey(NimBLEConnInfo &connInfo, uint32_t pass_key) override
  {
    Serial.print("Potvrzuji PassKey: ");
    Serial.println(pass_key);
    NimBLEDevice::injectConfirmPasskey(connInfo, true);
  }

  void onAuthenticationComplete(NimBLEConnInfo &connInfo) override
  {
    if (!connInfo.isEncrypted())
    {
      Serial.println("Šifrované spojení selhalo, odpojuji se");
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
      Serial.print("Nalezeno zařízení: ");
      Serial.println(devName);
      if (devName == TARGET_DEVICE_NAME)
      {
        Serial.println("Nalezeno cílové zařízení!");
        advDevice = advertisedDevice;
        doConnect = true;
        NimBLEDevice::getScan()->stop();
      }
    }
  }
} scanCallbacks;

// Callback pro notifikace z charakteristiky CSC
// class MyCSCCharacteristicCallbacks : public NimBLECharacteristicCallbacks
// {
//   void onNotify(NimBLECharacteristic *pCharacteristic) override
//   {
//     std::string value = pCharacteristic->getValue();
//     Serial.print("Obdržena notifikace, délka: ");
//     Serial.println(value.length());
//     decodeCSCMeasurement((uint8_t *)value.data(), value.length());
//   }
// };

// Funkce pro dekódování CSC měření
// Dekóduje primárně kolečková data (flag bit 0)
// Pokud by byl nastaven také bit 1, lze dle specifikace rozšířit dekódování o data z šlapání.
void decodeCSCMeasurement(uint8_t *data, size_t length)
{
  if (length < 1)
  {
    Serial.println("Nebyla obdržena žádná data");
    return;
  }

  uint8_t flags = data[0];
  Serial.print("Flags: 0x");
  Serial.println(flags, HEX);

  // Pokud je nastaven bit 0 – jsou přítomna kolečková data
  if (flags & 0x01)
  {
    if (length < 7)
    {
      Serial.println("Nedostatek bajtů pro kolečková data");
      return;
    }
    // Načtení kumulativního počtu otáček (4 bajty, little-endian)
    uint32_t currentCumulativeRevs = (uint32_t)data[1] | ((uint32_t)data[2] << 8) | ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 24);
    // Načtení času poslední události (2 bajty, little-endian; jednotky 1/1024 s)
    uint16_t currentLastEventTime = data[5] | (data[6] << 8);

    Serial.print("Kumulativní otáčky kola: ");
    Serial.println(currentCumulativeRevs);
    Serial.print("Čas poslední události: ");
    Serial.print(currentLastEventTime);
    Serial.print(" (");
    Serial.print(currentLastEventTime / 1024.0, 2);
    Serial.println(" sec)");

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
        float speed_kmph = speed_mps * 3.6;               // rychlost v km/h

        Serial.print("Delta revolucí: ");
        Serial.println(deltaRevs);
        Serial.print("Delta času: ");
        Serial.print(deltaTimeSec, 2);
        Serial.println(" sec");
        Serial.print("Rychlost: ");
        Serial.print(speed_mps, 2);
        Serial.print(" m/s, ");
        Serial.print(speed_kmph, 2);
        Serial.println(" km/h");
      }
      else
      {
        Serial.println("Delta času je nula, nelze vypočítat rychlost.");
      }
    }
    else
    {
      Serial.println("První měření, nelze spočítat rychlost.");
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
      Serial.print("Crank otáčky: ");
      Serial.println(crankRevs);
      Serial.print("Čas posledního crank eventu: ");
      Serial.print(lastCrankEventTime);
      Serial.print(" (");
      Serial.print(lastCrankEventTime / 1024.0, 2);
      Serial.println(" sec)");
    }
  }
}

void setup()
{
  Serial.begin(115200);
  vTaskDelay(2000 / portTICK_PERIOD_MS); // Zpoždění pro připojení sériového monitoru
  Serial.println("Spouštím BLE klienta pro SIGMA SPEED 17197...");

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
}

void loop()
{
  // Pokud bylo nalezeno cílové zařízení, pokusíme se o připojení
  if (doConnect && advDevice)
  {
    Serial.println("Pokus o připojení k cílovému zařízení...");
    pClient = NimBLEDevice::createClient();
    pClient->setClientCallbacks(new MyClientCallbacks(), false);

    if (!pClient->connect(advDevice))
    {
      Serial.println("Připojení se nezdařilo!");
      NimBLEDevice::getScan()->start(SCAN_TIME_MS, false, false);
      return;
    }

    Serial.println("Úspěšně připojeno!");
    // Hledáme službu CSC (UUID: 1816)
    NimBLERemoteService *pService = pClient->getService(CSC_SERVICE_UUID);
    if (!pService)
    {
      Serial.println("Služba CSC (1816) nebyla nalezena!");
      pClient->disconnect();
      return;
    }

    // Hledáme charakteristiku měření (UUID: 2A5B)
    pCSCCharacteristic = pService->getCharacteristic(CSC_CHAR_UUID);
    if (!pCSCCharacteristic)
    {
      Serial.println("Charakteristika CSC (2A5B) nebyla nalezena!");
      pClient->disconnect();
      return;
    }

    // Pokud podporuje notifikace, přihlásíme se
    if (pCSCCharacteristic->canNotify())
    {
      Serial.println("Přihlašuji se na notifikace z CSC charakteristiky...");
      pCSCCharacteristic->subscribe(true,
                                    [](NimBLERemoteCharacteristic *pChar, uint8_t *data, size_t length, bool isNotify)
                                    {
                                      Serial.print("Obdržena notifikace, délka: ");
                                      Serial.println(length);
                                      decodeCSCMeasurement(data, length);
                                    });
    }
    else
    {
      Serial.println("Charakteristika nepodporuje notifikace!");
    }

    // Po úspěšném připojení resetujeme příznak
    doConnect = false;
  }

  // V hlavní smyčce lze případně zpracovávat další logiku
  delay(1000);
}