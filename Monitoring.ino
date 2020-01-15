  /*  Envoriment Monitoring of Fume hood room OPTIMa/UFAM
  Author: Leonardo Arcanjo - leonardoarcanjoo@gmail.com
  About: This code uses a ESP32 with MQ-2 sensor and DHT11 sensor
  to monitor the Fume hood room of OPTIMa/UFAM.
  Version: 2.0
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"

#define DHTPIN 23                         //Pin connected to DATA pin from DHT11 sensor
#define DHTTYPE DHT11                     //Type of DHT sensor

#define MQ_PIN 34                         //define which analog input channel you are going to use 
#define RL_VALUE 1                        //define the load resistance on the FC-22 module, in kilo ohms
#define RO_CLEAN_AIR_FACTOR 9.83          //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO

#define CALIBRATION_SAMPLE_TIMES  50      //define how many samples you are going to take in the calibration routine
#define CALIBRATION_SAMPLE_INTERVAL 500   //define the time interal(in milisecond) between each samples in the
//cablibration routine
#define READ_SAMPLE_INTERVAL 50           //define how many samples you are going to take in read routine
#define READ_SAMPLE_TIMES 5               //define the time interal(in milisecond) between each samples in 
//read routine
#define GAS_LPG 0
#define GAS_CO 1
#define GAS_SMOKE 2

#define BUZZER 18
#define BUTTON 19

float LPGCurve[3] = {2.3, 0.21, -0.47};
float COCurve[3] = {2.3, 0.72, -0.34};
float SmokeCurve[3] = {2.3, 0.53, -0.44};

float Ro = 10;

DHT dht(DHTPIN, DHT11);
WiFiClient espClient;
PubSubClient client(espClient);

const char* ssid = "";
const char* password = "";

const char* mqttServer = "";
const int mqttPort = ;
const char* mqttUser = "";
const char* mqttPassword = "";

unsigned long verifyTime = 0;
unsigned long returnTime = 30000; //Time to send a message

float calibrationMQ(int);
float MQResistanceCalculation(int);
float MQRead(int);
int MQGetGasPercentage(float, int);
int MQGetPercentage(float, float*);
void readSensorMQ(float &, float&, float&);
void wifi(void);
void connectBroker(void);
void readSensorDHT(float &, float &);
void showSerial(float, float);
void sendMsg(float, float);
void setBuzzer(float, float, float);
bool checkButton(void);

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");
  Serial.println("Heating the sensor");
  delay(60000);
  
  Ro = calibrationMQ(MQ_PIN);
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

  pinMode(BUZZER, OUTPUT);
  pinMode(BUTTON, INPUT);
}

void loop() {
  bool flag;
  float temp;
  float humid;
  float GLP = 0;
  float Mono = 0;
  float fumaca = 0;

  readSensorDHT(temp, humid);
  readSensorMQ(GLP, Mono, fumaca);  

  if(GLP > 100.0 || Mono > 11.0 || fumaca > 10.0) digitalWrite(BUZZER, HIGH);
  else if (checkButton()) digitalWrite(BUZZER,LOW);

  if ((millis() - verifyTime) > returnTime) {
    verifyTime = millis();
    if (!client.connected()) connectBroker();
    showSerial(temp, humid, GLP, Mono, fumaca);
    sendMsg(temp, humid, GLP, Mono, fumaca);
  }
}

void wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Starting the connection in WiFi network...");
  }
  Serial.println("Connected in wifi network");
}

void connectBroker() {
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
  t = dht.readTemperature();
  h = dht.readHumidity();
}

void showSerial(float temperatura, float umidade, float glp, float co, float fumo) {
  Serial.print("Temperatura: ");
  Serial.print(temperatura);
  Serial.print(" Umidade: ");
  Serial.print(umidade);
  Serial.print(" GLP: ");
  Serial.print(glp);
  Serial.print("(ppm) CO: ");
  Serial.print(co);
  Serial.print("(ppm) Fumaca: ");
  Serial.print(fumo);
  Serial.println("(ppm)");
}

void sendMsg(float temperatura, float umidade, float glp, float co, float fumo) {
  char MsgTemperatura[4];
  char MsgUmidade[4];
  char MsgGLP[4];
  char MsgCO[4];
  char MsgSMOKE[4];

  int f = (float)umidade;
  int g = (float)glp;
  int c = (float)co;
  int s = (float)fumo;

  sprintf(MsgTemperatura, "%f", temperatura);
  client.publish("Capelas/temperature", MsgTemperatura);
  sprintf(MsgUmidade, "%i", f);
  client.publish("Capelas/humidity", MsgUmidade);
  sprintf(MsgGLP, "%i", g);
  client.publish("Capelas/GLP", MsgGLP);
  sprintf(MsgCO, "%i", c);
  client.publish("Capelas/CO", MsgCO);
  sprintf(MsgSMOKE, "%i", s);
  client.publish("Capelas/SMOKE", MsgSMOKE);
}

float calibrationMQ(int mq_pin) {
  int i = 0;
  float val = 0;

  for (i = 0; i < CALIBRATION_SAMPLE_TIMES; i++) {
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val / CALIBRATION_SAMPLE_TIMES;

  val = val / RO_CLEAN_AIR_FACTOR;

  return val;
}

float MQResistanceCalculation(int raw_adc) {
  if (raw_adc == 0) return 20;
  else return ((float)RL_VALUE * ((4095 - raw_adc) / raw_adc));
}

void readSensorMQ(float &gas, float &monox, float &fumarola) {
  gas = MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_LPG);
  monox = MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_CO);
  fumarola = MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_SMOKE);
}

float MQRead(int mq_pin) {
  int i;
  float rs = 0;

  for (i = 0; i < READ_SAMPLE_TIMES; i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }

  rs = rs / READ_SAMPLE_TIMES;

  return rs;
}

int MQGetGasPercentage(float rs_ro_ratio, int gas_id) {
  if ( gas_id == GAS_LPG ) {
    return MQGetPercentage(rs_ro_ratio, LPGCurve);
  } else if ( gas_id == GAS_CO ) {
    return MQGetPercentage(rs_ro_ratio, COCurve);
  } else if ( gas_id == GAS_SMOKE ) {
    return MQGetPercentage(rs_ro_ratio, SmokeCurve);
  }
  return 0;
}

int MQGetPercentage(float rs_ro_ratio, float *pcurve) {
  return (pow(10, ( ((log(rs_ro_ratio) - pcurve[1]) / pcurve[2]) + pcurve[0])));
}

bool checkButton() {
  int readBut = digitalRead(BUTTON);
  if (readBut == HIGH) return true;
  else return false; 
}
