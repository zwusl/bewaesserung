#include <WiFiS3.h>
#include <ArduinoHA.h>
#include <RunningMedian.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_AM2320.h"
#include "arduino_secrets.h"

// --- WICHTIGE HARDWARE-PINS ---
#define TAKT_PIN_ESP32  6  // Hier kommt das präzise 10-Hz-Signal vom ESP32-C6 an!

#define FREQ_PIN1       2
#define FREQ_PIN2       3  // Angepasst an deine alten Pin-Definitionen
#define FREQ_PIN3       8
#define FREQ_PIN4       12

#define POWER_PIN1      4
#define POWER_PIN2      5
#define POWER_PIN3      9
#define POWER_PIN4      7
#define POWER_PIN_TEMP  10

WiFiClient client;
HADevice device("Outdoor_multi");
HAMqtt mqtt(client, device);
Adafruit_AM2320 am2320 = Adafruit_AM2320();

unsigned long lastUpdateAt;
unsigned long waitmillis = 600000; // 10 Minuten Intervall

// --- VARIABLEN FÜR DIE PRÄZISIONSMESSUNG ---
volatile unsigned long impulsZaehler = 0;
volatile unsigned long letzteFrequenz = 0;
volatile bool messungBereit = false;

bool pins_powered = false;
bool connected = false;
int soil_in_charge = 1;

int median_size_soil = 9;
int median_size_temp = 9;
uint32_t min_soil = 0;
uint32_t max_soil = 450000; // Erhöht, da ein Gies-o-mat bis zu 450 kHz senden kann

int messVersucheSoil = 0;
int messVersucheTemp = 0;

RunningMedian samples_soil = RunningMedian(median_size_soil);
RunningMedian samples_temp = RunningMedian(median_size_temp);

// --- OPENHAB / HOME ASSISTANT ENTITIES ---
HASensorNumber analogSensor_temp("Temp", HASensorNumber::PrecisionP2);
HASensorNumber analogSensor_soil1("Soil1", HASensorNumber::PrecisionP2);
HASensorNumber analogSensor_soil2("Soil2", HASensorNumber::PrecisionP2);
HASensorNumber analogSensor_soil3("Soil3", HASensorNumber::PrecisionP2);
HASensorNumber analogSensor_soil4("Soil4", HASensorNumber::PrecisionP2);

HASensorNumber wifi_rssi("wifi_signal", HASensorNumber::PrecisionP0);

// ISR für die Zählung der Gies-o-mat Impulse
void signalImpulsISR() {
    impulsZaehler++;
}

// ISR für das präzise 10-Hz-Zeitfenster vom ESP32-C6
void rtcSekundenISR() {
    letzteFrequenz = impulsZaehler;
    impulsZaehler = 0;
    messungBereit = true;
}

void setup_wifi() {
    Serial.print("Trying to connect to: ");
    Serial.println(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    int i = 0;
    while (WiFi.status() != WL_CONNECTED && i < 20) {
        delay(500);
        Serial.print(".");
        i++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWLAN verbunden!");
        Serial.print("IP: "); Serial.println(WiFi.localIP());
        connected = true;
    } else {
        Serial.println("\nWiFi Verbindung fehlgeschlagen.");
        connected = false;
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("Starting Hardware Setup...");

    pinMode(FREQ_PIN1, INPUT_PULLUP);
    pinMode(FREQ_PIN2, INPUT_PULLUP);
    pinMode(FREQ_PIN3, INPUT_PULLUP);
    pinMode(FREQ_PIN4, INPUT_PULLUP);

    pinMode(POWER_PIN1, OUTPUT);
    pinMode(POWER_PIN2, OUTPUT);
    pinMode(POWER_PIN3, OUTPUT);
    pinMode(POWER_PIN4, OUTPUT);
    pinMode(POWER_PIN_TEMP, OUTPUT);

    digitalWrite(POWER_PIN1, LOW);
    digitalWrite(POWER_PIN2, LOW);
    digitalWrite(POWER_PIN3, LOW);
    digitalWrite(POWER_PIN4, LOW);
    digitalWrite(POWER_PIN_TEMP, LOW);

    setup_wifi();

    device.enableSharedAvailability();
    device.setName("Arduino_multi");
    device.setSoftwareVersion("1.0.0");

    analogSensor_temp.setIcon("mdi:thermometer");
    analogSensor_temp.setName("Temperatur Outdoor");
    analogSensor_temp.setUnitOfMeasurement("°C");

    for (int s = 1; s <= 4; s++) {
        HASensorNumber* sensor;
        if (s == 1) sensor = &analogSensor_soil1;
        if (s == 2) sensor = &analogSensor_soil2;
        if (s == 3) sensor = &analogSensor_soil3;
        if (s == 4) sensor = &analogSensor_soil4;
        sensor->setIcon("mdi:water-percent");
        sensor->setUnitOfMeasurement("kHz");
    }
    analogSensor_soil1.setName("Bodenfeuchte 1");
    analogSensor_soil2.setName("Bodenfeuchte 2");
    analogSensor_soil3.setName("Bodenfeuchte 3");
    analogSensor_soil4.setName("Bodenfeuchte 4");

    wifi_rssi.setIcon("mdi:wifi");
    wifi_rssi.setName("WLAN Signalstärke");
    wifi_rssi.setUnitOfMeasurement("dBm");

    am2320.begin();

    pinMode(TAKT_PIN_ESP32, INPUT_PULLUP);
    // Das ESP32-Signal triggert dauerhaft den Zeitfenster-Interrupt
    attachInterrupt(digitalPinToInterrupt(TAKT_PIN_ESP32), rtcSekundenISR, FALLING);

    mqtt.begin(BROKER_ADDR, BROKER_USERNAME, BROKER_PASSWORD);
    Serial.println("System bereit und MQTT initialisiert.");

    lastUpdateAt = millis() - waitmillis; // Erzwingt sofortige Messung beim Start
}

void loop() {
    mqtt.loop();

    // 1. SCHRITT: Prüfen ob das Intervall um ist und Sensoren bestromen
    if ((millis() - lastUpdateAt > waitmillis) && !pins_powered) {
        if (WiFi.status() != WL_CONNECTED) {
            setup_wifi();
        }

        // Nur den aktuell ausgewählten Feuchtigkeitssensor einschalten (Korrosionsschutz)
        digitalWrite(POWER_PIN1, soil_in_charge == 1 ? HIGH : LOW);
        digitalWrite(POWER_PIN2, soil_in_charge == 2 ? HIGH : LOW);
        digitalWrite(POWER_PIN3, soil_in_charge == 3 ? HIGH : LOW);
        digitalWrite(POWER_PIN4, soil_in_charge == 4 ? HIGH : LOW);

        digitalWrite(POWER_PIN_TEMP, HIGH); // Temperatursensor einschalten
        pins_powered = true;

        delay(30); // Stabilisierungszeit für Sensor-Schwingkreise
        samples_soil.clear();
        samples_temp.clear();

        Serial.print("Starte Messsequenz für Sensor: "); Serial.println(soil_in_charge);
    }

    // 2. SCHRITT: Wenn Saft auf den Pins ist, sammeln wir die Median-Messwerte
    if (pins_powered) {

        // A) TEMPERATUR EINLESEN mit eigenem Zähler
        if (samples_temp.getCount() < median_size_temp && messVersucheTemp < 15) {
            messVersucheTemp++; // Zählt nur hoch, solange die Temperatur noch sucht
            float t = am2320.readTemperature();
            if (!isnan(t)) {
                samples_temp.add(t);
            }
        } else {
            // Schaltet den Sensor ab, sobald voll ODER nach 15 eigenen Versuchen
            digitalWrite(POWER_PIN_TEMP, LOW);
        }

        // B) PRÄZISE FREQUENZMESSUNG mit eigenem Zähler
        // Wir messen nur, wenn der Median nicht voll ist UND wir unter 15 Versuchen sind
        bool soilMessungAktiv = (samples_soil.getCount() < median_size_soil && messVersucheSoil < 15);

        if (soilMessungAktiv) {
            messVersucheSoil++; // Zählt nur hoch, solange die Feuchtigkeit noch sucht

            int aktuellerFreqPin = FREQ_PIN1;
            if (soil_in_charge == 2) aktuellerFreqPin = FREQ_PIN2;
            if (soil_in_charge == 3) aktuellerFreqPin = FREQ_PIN3;
            if (soil_in_charge == 4) aktuellerFreqPin = FREQ_PIN4;

            // Synchronisation mit ESP32-C6
            messungBereit = false;
            while (!messungBereit) { mqtt.loop(); }

            impulsZaehler = 0;
            messungBereit = false;

            attachInterrupt(digitalPinToInterrupt(aktuellerFreqPin), signalImpulsISR, RISING);
            while (!messungBereit) {  }
            detachInterrupt(digitalPinToInterrupt(aktuellerFreqPin));

            unsigned long impulseProSekunde = letzteFrequenz * 10;
            Serial.println(impulseProSekunde);

            if (impulseProSekunde >= min_soil && impulseProSekunde <= max_soil) {
                samples_soil.add(impulseProSekunde);
            }
        }

        // C) AUSWERTUNG: Erst wenn BEIDE Sensoren fertig sind (entweder voll oder Timeout erreicht)
        // "Fertig" bedeutet: Der Median ist voll ODER der jeweilige Zähler hat die 15 erreicht.
        bool tempFertig = (samples_temp.getCount() >= median_size_temp || messVersucheTemp >= 15);
        bool soilFertig = (samples_soil.getCount() >= median_size_soil || messVersucheSoil >= 15);

        if (tempFertig && soilFertig) {
            // Alle Pins abschalten
            digitalWrite(POWER_PIN1, LOW);
            digitalWrite(POWER_PIN2, LOW);
            digitalWrite(POWER_PIN3, LOW);
            digitalWrite(POWER_PIN4, LOW);
            digitalWrite(POWER_PIN_TEMP, LOW);

            // Temperatur-Auswertung
            if (samples_temp.getCount() > 0) {
                float tempMedian = samples_temp.getMedian();
                analogSensor_temp.setValue(tempMedian);
            } else {
                Serial.println("FEHLER: Temperatursensor AM2320 ausgefallen!");
                analogSensor_temp.setValue((float)-99.0);
            }

            // Bodenfeuchte-Auswertung
            if (samples_soil.getCount() > 0) {
                float soilFrequenzKhz = samples_soil.getMedian() / 1000.0f;
                if (soil_in_charge == 1) analogSensor_soil1.setValue(soilFrequenzKhz);
                if (soil_in_charge == 2) analogSensor_soil2.setValue(soilFrequenzKhz);
                if (soil_in_charge == 3) analogSensor_soil3.setValue(soilFrequenzKhz);
                if (soil_in_charge == 4) analogSensor_soil4.setValue(soilFrequenzKhz);
            } else {
                Serial.print("FEHLER: Sensor "); Serial.print(soil_in_charge); Serial.println(" ausgefallen!");
                if (soil_in_charge == 1) analogSensor_soil1.setValue(0.0f);
                if (soil_in_charge == 2) analogSensor_soil2.setValue(0.0f);
                if (soil_in_charge == 3) analogSensor_soil3.setValue(0.0f);
                if (soil_in_charge == 4) analogSensor_soil4.setValue(0.0f);
            }


            // WLAN-Signalstärke (RSSI) auslesen und an openHAB senden
            if (WiFi.status() == WL_CONNECTED) {
                long rssi = WiFi.RSSI();
                Serial.print("WLAN Signalstärke: ");
                Serial.print(rssi);
                Serial.println(" dBm");

                // Wert an die Library übergeben
                wifi_rssi.setValue((float)rssi);
            } else {
                // Falls nicht verbunden, senden wir einen Fehlerwert (z.B. -999)
                wifi_rssi.setValue((float)-99.0);
            }


            // BEIDE Zähler für den nächsten Sensor-Durchlauf zurücksetzen
            messVersucheSoil = 0;
            messVersucheTemp = 0;

            soil_in_charge++;
            if (soil_in_charge > 4) {
                soil_in_charge = 1;
            }

            pins_powered = false;
            lastUpdateAt = millis();
            Serial.println("Messsequenz erfolgreich beendet. Gehe jetzt 10 min in den Wartezustand.");
        }
    }
}
