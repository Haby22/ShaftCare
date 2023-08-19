/*
  ESP32 Soil Moisture Sensor
  http:://www.electronicwings.com
*/

#include "DHT.h"
#include <WiFi.h>

// const char* ssid = "AEIG_EVENT"; // Nom du réseau Wi-Fi
// const char* password = "3yEmTa3iJ"; // Mot de passe du réseau Wi-Fi

// const char* serverAddress = "10.50.1.92"; // Adresse IP à joindre
const char *ssid = "agirEau";     // Nom du réseau Wi-Fi
const char *password = "agirEau"; // Mot de passe du réseau Wi-Fi

const char *serverAddress = "agireau.biristechnologie.com"; // Adresse IP à joindre

const int serverPort = 3000; // Port de communication

WiFiClient client; // Client Wi-Fi pour la communication
#define DHTPIN 19
#define DHTTYPE DHT11
// DHTTYPE = DHT11, mais il existe aussi le DHT22 et 21

#define TRIG_PIN 5  // ESP32 pin GIOP23 connected to Ultrasonic Sensor's TRIG pin
#define ECHO_PIN 18 // ESP32 pin GIOP22 connected to Ultrasonic Sensor's ECHO pin
float duration_us, distance_cm;

DHT dht(DHTPIN, DHTTYPE); // constructeur pour déclarer notre capteur

int _moisture, sensor_analog;
const int sensor_pin = A0; /* Soil moisture sensor O/P pin */

void setup(void)
{
    Serial.begin(115200); /* Set the baudrate to 115200*/
    dht.begin();
    // configure the trigger pin to output mode
    pinMode(TRIG_PIN, OUTPUT);
    // configure the echo pin to input mode
    pinMode(ECHO_PIN, INPUT);
    // Connexion au réseau Wi-Fi
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("Connexion au réseau Wi-Fi...");
    }

    Serial.println("Connecté au réseau Wi-Fi !");
}

void loop(void)
{
    //
    // generate 10-microsecond pulse to TRIG pin
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // measure duration of pulse from ECHO pin
    duration_us = pulseIn(ECHO_PIN, HIGH);

    // calculate the distance
    distance_cm = 0.017 * duration_us;

    // print the value to Serial Monitor
    Serial.print("distance: ");
    Serial.print(distance_cm);
    Serial.println(" cm");

    delay(500);
    //

    sensor_analog = analogRead(sensor_pin);
    _moisture = (100 - ((sensor_analog / 4095.00) * 100));
    Serial.print("Moisture = ");
    Serial.print(_moisture); /* Print Temperature on the serial window */
    Serial.println("%");
    delay(1000); /* Wait for 1000mS */

    // Le DHT11 renvoie au maximum une mesure toute les 1s
    float h = dht.readHumidity();
    // Lis le taux d'humidite en %
    float t = dht.readTemperature();
    // Lis la température en degré celsius
    float f = dht.readTemperature(true);
    // Avec (true), renvoi la température en fahrenheit

    if (isnan(h) || isnan(t) || isnan(f))
    {
        Serial.println("Echec reception");
        return;
        // Renvoie une erreur si l'ESP32 ne reçoit aucune mesure
    }

    Serial.print("Humidite: ");
    Serial.print(h);
    Serial.print("%  Temperature: ");
    Serial.print(t);
    Serial.print("°C, ");
    Serial.print(f);
    Serial.println("°F");
    // Transmet les mesures reçues dans le moniteur série
    if (!client.connected())
    {
        if (client.connect(serverAddress, serverPort))
        {
            Serial.println("Connecté au serveur !");

            // Envoyer des données au serveur
            String url = "/api/captor-data/?ambiantTemp=" + String(t) + "&SizePlant=" + String(distance_cm) + "&soilHumidity=" + String(_moisture) + "&captorId=64ac5f8bd543447932b74631";
            client.print("POST " + url + " HTTP/1.1\r\n");
            //  client.print("PUT /api/feed HTTP/1.1\r\n");
            client.println("Host: " + String(serverAddress));
            client.println("Connection: close");
            client.println();
        }
        else
        {
            Serial.println("Échec de la connexion au serveur !");
        }
    }

    // Lire la réponse du serveur
    while (client.available())
    {
        String response = client.readStringUntil('\r');
        Serial.print(response);
    }

    if (!client.connected())
    {
        client.stop();
        Serial.println("Déconnecté du serveur !");

        delay(5000);
    }
}