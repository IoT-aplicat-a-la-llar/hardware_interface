#include <WiFi.h>
#include <PubSubClient.h>
#include <DFRobot_ENS160.h>
#include "DFRobot_BME280.h"

#define SEA_LEVEL_PRESSURE 1015.0f

// Configuración WiFi
const char* ssid = "Lab-Modul";
const char* password = "Nix929Xr";

// Configuración MQTT
const char* mqttServer = "10.11.0.155";
const int mqttPort = 1883;
const char* mqttUser = "Home_Assistan";
const char* mqttPassword = "1234";
const char* mqttTopic = "dev/test";

// Pines y variables
const int micPin = 32;          // Pin analógico para el micrófono
const int pinPresencia = 23;    // Pin para sensor de presencia
const int sampleWindow = 50;    // Ventana de muestreo en ms (50ms = 20Hz)

WiFiClient espClient;
PubSubClient client(espClient);
DFRobot_ENS160_I2C ENS160(&Wire, 0x53);
typedef DFRobot_BME280_IIC BME;
BME bme(&Wire, 0x76);

// Variables globales
unsigned int sample;
int presencia;
float temp, humi, alti;
uint32_t press;
uint8_t AQI;

void printLastOperateStatus(BME::eStatus_t eStatus) {
  switch(eStatus) {
    case BME::eStatusOK: Serial.println("everything ok"); break;
    case BME::eStatusErr: Serial.println("unknown error"); break;
    case BME::eStatusErrDeviceNotDetected: Serial.println("device not detected"); break;
    case BME::eStatusErrParameter: Serial.println("parameter error"); break;
    default: Serial.println("unknown status"); break;
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mensaje recibido en el tema: ");
  Serial.println(topic);
  Serial.print("Mensaje: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Conectando a MQTT...");
    if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
      Serial.println("Conectado al broker MQTT");
      client.subscribe("raspberrypi/comandos");
    } else {
      Serial.print("Falló la conexión con error ");
      Serial.print(client.state());
      Serial.println(", reintentando en 5 segundos...");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  
  // Inicializar WiFi
  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado");
  Serial.print("IP asignada: ");
  Serial.println(WiFi.localIP());

  // Configurar MQTT
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  reconnectMQTT();

  // Inicializar sensores
  bme.reset();
  Serial.println("Inicializando BME280...");
  while(bme.begin() != BME::eStatusOK) {
    Serial.println("bme begin falló");
    printLastOperateStatus(bme.lastOperateStatus);
    delay(2000);
  }
  Serial.println("BME280 inicializado correctamente");

  Serial.println("Inicializando ENS160...");
  while(NO_ERR != ENS160.begin()) {
    Serial.println("Comunicación con ENS160 fallida, reintentando...");
    delay(2000);
  }
  Serial.println("ENS160 inicializado correctamente");
  ENS160.setPWRMode(ENS160_STANDARD_MODE);

  // Configurar pines
  pinMode(micPin, INPUT);
  pinMode(pinPresencia, INPUT);
  Serial.println("Todos los sensores inicializados");
}

void leerSensores() {
  // Leer sonido
  unsigned long startMillis = millis();
  unsigned int peakToPeak = 0;
  unsigned int signalMax = 0;
  unsigned int signalMin = 4095;

  while (millis() - startMillis < sampleWindow) {
    sample = analogRead(micPin);
    if (sample < 4095) {
      if (sample > signalMax) signalMax = sample;
      else if (sample < signalMin) signalMin = sample;
    }
  }
  peakToPeak = signalMax - signalMin;

  // Leer presencia
  presencia = digitalRead(pinPresencia) == HIGH ? 1 : 0;

  // Leer calidad del aire
  temp = bme.getTemperature();
  press = bme.getPressure();
  alti = bme.calAltitude(SEA_LEVEL_PRESSURE, press);
  humi = bme.getHumidity();
  ENS160.setTempAndHum(temp, humi);
  AQI = ENS160.getAQI();


  // Mostrar datos por serial
  Serial.println("--- Datos de los sensores ---");
  Serial.print("Temperatura: "); Serial.print(temp); Serial.println(" °C");
  Serial.print("Humedad: "); Serial.print(humi); Serial.println(" %");
  Serial.print("Presión: "); Serial.print(press); Serial.println(" Pa");
  Serial.print("AQI: "); Serial.println(AQI);
  Serial.print("Ruido: "); Serial.println(peakToPeak);
  Serial.print("Presencia: "); Serial.println(presencia);
}

void loop() {
  // Verificar conexiones
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  static unsigned long lastTime = 0;
  if (millis() - lastTime > 5000) {
    lastTime = millis();
    
    leerSensores();
    
  String mensaje = "{";
  mensaje += "\"temp\":" + String(temp, 2) + ",";
  mensaje += "\"humi\":" + String(humi, 2) + ",";
  mensaje += "\"press\":" + String(press) + ",";
  mensaje += "\"aqi\":" + String(AQI) + ",";
  mensaje += "\"sound\":" + String(peakToPeak) + ",";
  mensaje += "\"presencia\":" + String(presencia);
  mensaje += "}";

    if (client.publish(mqttTopic, mensaje.c_str())) {
      Serial.println("Mensaje MQTT enviado correctamente");
    } else {
      Serial.println("Error al enviar mensaje MQTT");
    }
  }
}