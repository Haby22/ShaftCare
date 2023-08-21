/*
  ESP32 Soil Moisture Sensor
  http:://www.electronicwings.com
*/

#include "DHT.h"
#include <ESP8266WiFi.h>
 
// wifi access id and mdp , server host and port
// const char* ssid = "ASUSOW"; // Nom du réseau Wi-Fi
// const char* password = "22052000"; // Mot de passe du réseau Wi-Fi
const char *ssid = "ShaftCare";                             // Nom du réseau Wi-Fi
const char *password = "ShaftCare2023";                     // Mot de passe du réseau Wi-Fi
const char *serverAddress = "agireau.biristechnologie.com"; // Adresse IP à joindre
const int serverPort = 3000;                                // Port de communication

// Soil moisiture default value
int sensorValueMin = 100;  // Remplacez par la valeur minimale lue en sol humide
int sensorValueMax = 1023; // Remplacez par la valeur maximale lue en sol sec

// wifi init client
WiFiClient client; // Client Wi-Fi pour la communication
#define DHTPIN 12
#define DHTTYPE DHT11
// DHTTYPE = DHT11, mais il existe aussi le DHT22 et 21

#define TRIG_PIN 5 // ESP32 pin GIOP23 connected to Ultrasonic Sensor's TRIG pin
#define ECHO_PIN 4 // ESP32 pin GIOP22 connected to Ultrasonic Sensor's ECHO pin
float duration_us, distance_cm;

DHT dht(DHTPIN, DHTTYPE); // constructeur pour déclarer notre capteur
String macAddress;
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
    String macAddress = getMacAddress();
    int SoilMoisture = getSoilMoisture();
    float size = getUltrasonData();
    float ambiantHumidity = getdhtHumidity();
    float ambiantTemperature = getdhtTemp();
    Serial.println(macAddress);
    Serial.println(SoilMoisture);
    Serial.println(size);
    Serial.println(ambiantHumidity);
    Serial.println(ambiantTemperature);
    sendShaftCareData(macAddress, SoilMoisture, size, ambiantHumidity, ambiantTemperature);
}

String getMacAddress()
{
    // Get the MAC address and store it in the global variable
    String macAddress = WiFi.macAddress();

    // Print the MAC address for verification
    return macAddress;
}
int getSoilMoisture()
{
    int soilMoistureValue = analogRead(sensor_pin);
    // Convertir la valeur brute en pourcentage d'humidité
    int moisturePercentage = map(soilMoistureValue, sensorValueMin, sensorValueMax, 100, 0);
    _moisture = constrain(moisturePercentage, 0, 100); // Assurer que la valeur reste entre 0 et 100
    return _moisture;
    delay(1000);
}
float getUltrasonData()
{
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    // measure duration of pulse from ECHO pin
    duration_us = pulseIn(ECHO_PIN, HIGH);
    // calculate the distance
    distance_cm = 0.017 * duration_us;

    // print the value to Serial Monitor
    delay(500);
    return distance_cm;
}
float getdhtHumidity()
{
    float ambianthumidity = dht.readHumidity();
    if (isnan(ambianthumidity))
    {
        Serial.println("Echec reception");
        return 0.0;
        // Renvoie une erreur si l'ESP32 ne reçoit aucune mesure
    }
    return ambianthumidity;
}
float getdhtTemp()
{
    float ambiantTemp = dht.readTemperature();
    if (isnan(ambiantTemp))
    {
        Serial.println("Echec reception");
        return 0.0;
        // Renvoie une erreur si l'ESP32 ne reçoit aucune mesure
    }
    return ambiantTemp;
}
void sendShaftCareData(String macAddress, float SoilMoisture, int size, float ambiantHumidity, float ambiantTemperature)
{
    if (!client.connected())
    {
        if (client.connect(serverAddress, serverPort))
        {
            Serial.println("Connecté au serveur !");

            // Envoyer des données au serveur
            String url = "/api/captor-data/?ambiantTemperature=" + String(ambiantTemperature) + "&SizePlant=" + String(size) + "&SoilMoisture=" + String(SoilMoisture) + "&macAddress=" + macAddress + "&ambiantHumidity=" + String(ambiantHumidity);
            client.print("POST " + url + " HTTP/1.1\r\n");
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
