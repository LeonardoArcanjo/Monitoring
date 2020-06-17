/*  Envoriment Monitoring of Fume hood room OPTIMa/UFAM
    Author: Leonardo Arcanjo - github: leonardoarcanjo
    About:  This code uses a ESP32 with MQ-2 sensor, DHT11 sensor, DS18B20 sensor and Water level sensor
            to monitor a room envoriment and a water tank. This project uses MQTT protocol in order to user
            can view the datas in MQTT Dashboard as subscriber.
    Version: 4.0
*/

/*
    Libraries used: Wifi, MQTT, DHT Sensor, One Wire, Dallas Temperature(DS18B20), MQ-2 Gas Sensor
*/
#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <MQ2.h>

/*Define DHT parameters*/
#define DHTPIN 15                         //Pin connected to DATA pin from DHT11 sensor
#define DHTTYPE DHT11                     //Type of DHT sensor

/*MQ-2 Sensor Pin which analog input channel it'll used*/
#define MQ_PIN 33

/*Constants used in DS18B20 sensor calibration */
#define a 1.023
#define b -0.7

/*Define periferics pins*/
#define BUZZER 4
#define BUTTON 22

/*Define Water Level Sensors pins*/
#define WSENSOR1 5
#define WSENSOR2 18
#define WSENSOR3 19

/*Define Tank Temperature Sensor Pin*/
#define DBSensor 21

/*Constructors used in this project*/
DHT dht(DHTPIN, DHT11);
WiFiClient espClient;
PubSubClient client(espClient);
OneWire oneWire(DBSensor);
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress; 
MQ2 GasSensor(MQ_PIN);

/*MQTT and WIFI constants used to connection:
   ssid: Wifi network name
   password: Wi-Fi network password
   mqttServer: MQTT Server link (there are many server, the user has to choose one of them)
   mqttPort: The default PORT in MQTT application is 1883, but the user can choose other port,
   but paying attention on MQTT requests.
   Ps: The user has to create MQTT User and Passwords because they're uniques and to keep the security
   of its application.
*/
const char* ssid = "xxxxxxxxxxxxxxxxx";
const char* password = "xxxxxxxxxxxx";
const char* mqttServer = "mqtt.eclipse.org";
const int mqttPort = 1883;
const char* mqttUser = "xxxxxxxxxxxx";
const char* mqttPassword = "xxxxxxxxxxxxxxx";

/*Time Constants to reference and to send message with parameters measured to MQTT broker */
unsigned long verifyTime = 0;
unsigned long returnTime = 30000; //Time to send a message tp broker

/*Connection functions*/
void wifi(void);
void connectBroker(void);

/*Sensors Readers functions*/
void readSensorDHT(float &, float &);
void readSensorDB(float &);
void readSensorMQ(float &, float&, float&);
int readTankLvl(void);

/* functions that shows paramaters (temperature, humidity, GLP, Monoxide, smoke, Tank Temperature, Tank Level)
   to the user, via serial and send message to MQTT Broker */
void showSerial(float, float, float, float, float, float, int);
void sendMsg(float, float, float, float, float, float, int);

/*Auxiliar periferics functions */
void setBuzzer(float, float, float);
bool checkButton(void);

float Ro = 0; // Reference Resistance used in Concentration Gas in Air

void setup() {
  /* In setup function, the serial communication is started and MQ-2 sensor is heated in order to obtain
     a reference resistance to the calcuation of level gases (LPG, CO and SMOKE) in ppm. It's started the
     wifi and MQTT Broker connection. If MQTT Broker connection doesn't happen, the client.state() returns
     what tpye of failure. The DHT and DS18B20 sensors are initialized and the pin Modes of some periferics
     are defined.
  */
  Serial.begin(115200);

  GasSensor.begin();
  
  Ro = GasSensor.calibration();
  Serial.print("Ro = ");
  Serial.print(Ro);
  Serial.println(" kohm");

  wifi();

  client.setServer(mqttServer, mqttPort);

  while (!client.connected()) {
    Serial.println("Connecting to MQTT broker...");
    if (client.connect("ESP32Client", mqttUser, mqttPassword)) Serial.println("Connected to broker!");
    else {
      Serial.print("Connection to broker has failed - State: ");
      Serial.println(client.state());
      delay(2000);
    }
  }

  dht.begin();

  sensors.begin();
  sensors.setResolution(11);
  
  pinMode(BUZZER, OUTPUT);
  pinMode(BUTTON, INPUT);
  pinMode(WSENSOR1, INPUT);
  pinMode(WSENSOR2, INPUT);
  pinMode(WSENSOR3, INPUT);
}

void loop() {
  /* In loop function, the sensors are readed all time in its respective function and after 30 seconds, ESP-32 connects
     wifi and tries to send the sensors values to MQTT Broker. In case of Gases Level be more then 100 ppm to LPG,
     11 ppm to Carbon Monoxide and 10 ppm to Smoke, the buzzer will be actived. Besides, the button is checked
     to stop the buzzer, but even if the gases levels are more than the limits, the buzzer will continue actived.
  */
  float temp;
  float humid;
  float temp_tank;
  float GLP = 0;
  float Mono = 0;
  float fumaca = 0;
  int LvlTank;

  readSensorMQ(GLP, Mono, fumaca);
  readSensorDHT(temp, humid);
  readSensorDB(temp_tank);
  LvlTank = readTankLvl();
  
  if (GLP > 100.0 || Mono > 11.0 || fumaca > 10.0) digitalWrite(BUZZER, HIGH);
  else if (checkButton()) digitalWrite(BUZZER, LOW);

  if ((millis() - verifyTime) > returnTime) {
    verifyTime = millis();
    if (!client.connected()) connectBroker();
    showSerial(temp, humid, GLP, Mono, fumaca, temp_tank, LvlTank);
    sendMsg(temp, humid, GLP, Mono, fumaca, temp_tank, LvlTank);
  }
}

void wifi() {
  /*Try to connect the wifi network using the ssid and password network
    Parameters: none
    Return: none
  */
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Starting the connection in WiFi network...");
  }
  Serial.println("Connected in wifi network");
}

void connectBroker() {
  /* This funciton tries to connect the ESP-32 to MQTT Broker, in case of failure, client.state() returns
      a integer that indicates the failure and it's show in serial terminal.
      Parameters: none
      return: none
  */
  while (!client.connected()) {
    Serial.println("Trying to MQTT broker...");
    bool publisher = strlen(mqttUser) > 0 ? client.connect("ESP32Client", mqttUser, mqttPassword) : client.connect("ESP32Client");
    if (publisher) Serial.println("Connected to Broker!");
    else {
      Serial.print("Connection to broker has failed - State: ");
      Serial.println(String (client.state()).c_str());
      Serial.println("Trying again in 10 seconds");
      delay(10000);
    }
  }
}

void readSensorDHT(float &t, float &h) {
  /* Read temperature and humidity from envoriment via DHT sensor.
     Parameters:  Float Temperature
                  Float Humidity
                  Ps: The parameters are passed by reference only to refresh the values in loop function
     Return: none
  */
  t = dht.readTemperature();
  h = dht.readHumidity();
}

void readSensorDB(float &tempTank) {
  /* Request and Read temperature from Water Tank via DS18B20 sensor.
     Parameters:  Float Temperature
                  Ps: The parameter is passed by reference only to refresh the temperature value in loop function
     Return: none
  */
  sensors.requestTemperatures();
  float tempRead = sensors.getTempCByIndex(0);
  tempTank = a * tempRead + b;
}

int readTankLvl() {
  /*
    Read Water level sensors and return the tank level
    parameters: none
    return:   4 - Full
              3 - Above half
              2 - Below half
              1 - Empty
              -1 - Read Error
  */
  int sensor1 = digitalRead(WSENSOR1);
  int sensor2 = digitalRead(WSENSOR2);
  int sensor3 = digitalRead(WSENSOR3);

  if (sensor3 == false && sensor2 == true && sensor1 == true) return 4;
  else if (sensor3 == true && sensor2 == true && sensor1 == true) return 3;
  else if (sensor3 == true && sensor2 == false && sensor1 == true) return 2;
  else if (sensor3 == true && sensor2 == false && sensor1 == false) return 1;
  else return -1;
}

void showSerial(float temperatura, float umidade, float glp, float co, float fumo, float tank, int level) {
  /*
    Shows via serial all the envoriment and tank paramaters
    parameters: Float Temperature       (ºC)
                float Humidity          (%)
                float LPG               (ppm)
                float Carbon Monoxide   (ppm)
                float Smoke             (ppm)
                float tank temperature  (ºC)
                int Tank Level
    return: none
  */
  Serial.print("Temperature: ");
  Serial.print(temperatura);
  Serial.print(" Humidity: ");
  Serial.print(umidade);
  Serial.print(" GLP: ");
  Serial.print(glp);
  Serial.print("(ppm) CO: ");
  Serial.print(co);
  Serial.print("(ppm) Smoke: ");
  Serial.print(fumo);
  Serial.print("(ppm) Tank Temperature: ");
  Serial.println(tank);

  switch (level) {
    case 4:
      Serial.println("Full Tank");
      break;
    case 3:
      Serial.println("Above half");
      break;
    case 2:
      Serial.println("Below half");
      break;
    case 1:
      Serial.println("Empty Tank");
      break;
    case -1:
      Serial.println("Read Error");
      break;
  }
}

void sendMsg(float temperatura, float umidade, float glp, float co, float fumo, float tanque, int nivel) {
  /*
    Takes the parameters and converts to various CHAR arrays that it'll send to MQTT Broker and it'll
    be readed by the subscribers. There is a typecasting from some parameters like: Humidity, LPG, Carbon
    Monoxide and Smoke.
    Parameters: Float Temperature       (ºC)
                Float Humidity          (%)
                Float LGP               (ppm)
                Float Carbon Monoxide   (ppm)
                Float Smoke             (ppm)
                Float temperature Tank  (ºC)
                int Level Tank
  */
  //Buffers utilizados onde os valores convertidos em strings vao ser armazenados
  char MsgTemperatura[5];
  char MsgUmidade[4];
  char MsgGLP[4];
  char MsgCO[4];
  char MsgSMOKE[4];
  char MsgTank[5];

  int f = (float)umidade;
  int g = (float)glp;
  int c = (float)co;
  int s = (float)fumo;

  sprintf(MsgTemperatura, "%.2f", temperatura);
  client.publish("Capelas/temperature", MsgTemperatura);
  sprintf(MsgUmidade, "%i", f);
  client.publish("Capelas/humidity", MsgUmidade);
  sprintf(MsgGLP, "%i", g);
  client.publish("Capelas/GLP", MsgGLP);
  sprintf(MsgCO, "%i", c);
  client.publish("Capelas/CO", MsgCO);
  sprintf(MsgSMOKE, "%i", s);
  client.publish("Capelas/SMOKE", MsgSMOKE);
  sprintf(MsgTank, "%.2f", tanque);
  client.publish("Capelas/Tank", MsgTank);

  switch (nivel) {
    case 4:
      client.publish("Capelas/LTank", "Cheio");
      break;
    case 3:
      client.publish("Capelas/LTank", "Acima Metade");
      break;
    case 2:
      client.publish("Capelas/LTank", "Abaixo Metade");
      break;
    case 1:
      client.publish("Capelas/LTank", "Seco");
      break;
    case -1:
      client.publish("Capelas/LTank", "ErroR");
      break;
  }
}

void readSensorMQ(float &gas, float &monox, float &fumarola) {
  /* Read LPG, CO and Smoke via MQ-2 Sensor
     Parameters:  Float LPG
                  Float Carbon Monoxide
                  Float Smoke
                  Ps: The parameters are passed by reference only to refresh the values in loop function
     Return: none
  */
  gas = GasSensor.GetPercentageGas(Ro, LPG);
  monox = GasSensor.GetPercentageGas(Ro, CO);
  fumarola = GasSensor.GetPercentageGas(Ro, SMOKE);
}

bool checkButton() {
  /* Check button status
   * Parameters: none
   * return: Boolean
  */
  int readBut = digitalRead(BUTTON);
  if (readBut == HIGH) return true;
  else return false;
}
