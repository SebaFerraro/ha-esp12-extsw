#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <BH1750FVI.h>
#include "SparkFunBME280.h"
#include "Wire.h"
#include "SPI.h"

//#include "Ticker.h"
//#include "DHTesp.h"

#define I2C_SCL 14
#define I2C_SDA 12
#define RLY_P1 10
#define MQTT_SERVER "10.77.17.123"  //you MQTT IP Address
#define MQTT_PORT 1883  //you MQTT IP Address
#define MQTT_USER "ha"  //you MQTT IP Address
#define MQTT_PASS "4ut0m4t1c0"  //you MQTT IP Address
#define INTERVALO 60000

const char* ssid     = "wifi";
const char* password = "secret1703secret1703";
int analog_value = 0;
float divisor = 5.0244;
float divisori = 1;
int soffset = -17;
//float Sensibilidad=0.066; //sensibilidad en V/A para nuestro sensor
float Sensibilidad=0.185; //sensibilidad en V/A para nuestro sensor
float vmsensor = 2.52;
float offset=0.0; // Equivale a la amplitud del ruido
float Ioffset=-0.0; // Equivale a la amplitud del ruido
String OTAPassword = "Secret1703";
String TopicBase = "funes";
String TopicDev = "dev-e-02";
String Tswitch1 = "luz/ee/sw2";
String TTemp = "temperatura";
String THum = "humedad";
String TLuz = "luz";
String TCor = "corriente";
String TPres = "presion";
uint32_t tiempo=millis();
uint32_t tiempor=0;
bool Ebme280=true;
bool Ebh1750=true;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
BME280 mySensorA;
BH1750FVI myBH1750(BH1750_DEFAULT_I2CADDR, BH1750_CONTINUOUS_HIGH_RES_MODE_2, BH1750_SENSITIVITY_DEFAULT, BH1750_ACCURACY_DEFAULT);


uint8_t inicia_bme280()
{
	//***Set up sensor 'A'******************************//
	//commInterface can be I2C_MODE or SPI_MODE
	mySensorA.settings.commInterface = I2C_MODE;
	mySensorA.settings.I2CAddress = 0x76;
	mySensorA.settings.runMode = 3; //  3, Normal mode
	mySensorA.settings.tStandby = 0; //  0, 0.5ms
	mySensorA.settings.filter = 0; //  0, filter off
	//tempOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
	mySensorA.settings.tempOverSample = 1;
	//pressOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
    mySensorA.settings.pressOverSample = 1;
	//humidOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
	mySensorA.settings.humidOverSample = 1;

	Serial.println("Starting BME280s... result of .begin():");
	delay(10);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
	//Calling .begin() causes the settings to be loaded
	Serial.print("Sensor A: 0x");
	uint8_t sal=mySensorA.begin();
	Serial.println(sal, HEX);
  return sal;
  }

float get_corriente(){
  float voltajeSensor;
  float corriente=0;
  float pcorriente=0;
  float tcorriente=0;
  float fcorriente=0;
  float vsensor=0;
  int n=0;
  long tiempo=millis();
  for ( int k=0; k<20; k++ ){
    while ( millis() - tiempo < 21 ){
      vsensor=analogRead(A0);
      //Serial.println(vsensor);
      voltajeSensor = (((vsensor + soffset) / 1023.0) * divisor * divisori )+ offset;//lectura del sensor
      //Serial.println(voltajeSensor);
      corriente=((voltajeSensor - vmsensor) / Sensibilidad); //Ecuación  para obtener la corriente
      //Serial.println(corriente);
      pcorriente=pcorriente + sq(corriente );
      n++;
    }
    delay(200);
    tcorriente=(sqrt((pcorriente/n))+ Ioffset);
    fcorriente=fcorriente + tcorriente;
  }
  Serial.print("Nro de Muestras :");
  Serial.print(n);
  Serial.print("  P Corriente :");
  Serial.print(sqrt(pcorriente/n));
  Serial.print("  Corriente - offset:");
  Serial.print(tcorriente);
  Serial.print("  FCorriente /10:");
  Serial.print(fcorriente/20);
  return(fcorriente/20);
}


void wifi_conectar(void){
  WiFi.disconnect();
  Serial.printf("Conectando a %s\n", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Conectado, dirección IP: ");
  Serial.println(WiFi.localIP());
}

char* string2char(String command){
    if(command.length()!=0){
        char *p = const_cast<char*>(command.c_str());
        return p;
    }else{
      return 0;
    }
}

int pubicatopic_mqtt(String topic, String msg){
  int rsus;
  rsus=mqttClient.publish(string2char(topic), msg.c_str());
  Serial.print( "Publish : ");
  Serial.println(rsus);
  Serial.print( "Topic : ");
  Serial.println(topic);
  Serial.print( "Valor :");
  Serial.println(msg);
  return rsus;
}

void on_message(char* topic, byte* payload, unsigned int length) {
  Serial.println("Mensaje Recibido");
  String topicStr = String(topic);
  Serial.print("Topic: ");
  Serial.println(topicStr);
  Serial.print("Payload: ");
  Serial.println((char)payload[0]);
  Serial.print("length: ");
  Serial.println(length);
  char cpayload[length + 1];
  strncpy (cpayload, (char*)payload, length);
  cpayload[length] = '\0';
  String msg=cpayload;
  String Topic1=TopicBase + "/" + Tswitch1;
  String Topic1C=TopicBase + "/" + Tswitch1 + "c";
  
  if (topicStr == Topic1){
    if((cpayload[0] == '1') || (cpayload[0] == '0')){
       Serial.print("Topic1 Payload : ");
       Serial.println(msg.toInt());
       digitalWrite(RLY_P1, msg.toInt());
       pubicatopic_mqtt(Topic1C, msg);
    }
  }else{
    Serial.println("Topic1 No coincide : ");
    Serial.println(topicStr);
    Serial.println(Topic1);
  }

}

int suscribetopics_mqtt(){
  int rsus;
  String Topic1=TopicBase + "/" + Tswitch1;
  rsus=mqttClient.subscribe(string2char(Topic1));
  Serial.print( "[SUBSCRIBE] Topic1" );
  Serial.println(Topic1);
  Serial.println(rsus);
  return (rsus);
}

void send_mqtt(float t, float h, float l, float i, float pr){
  int rsus;
  
  if (WiFi.isConnected()){
    if(!mqttClient.connected()) {
      Serial.print("Conectando Cior ...");
      if ( mqttClient.connect((char*) TopicDev.c_str(), MQTT_USER, MQTT_PASS)) {
        Serial.println( "[DONE]" );
        suscribetopics_mqtt();
      } else {
        Serial.print( "[FAILED] [ rc = " );
        Serial.print( mqttClient.state() );
      }
    }
    if(mqttClient.connected()){
      String temperatura = String(t);
      String humedad = String(h);
      String luminosidad = String(l);
      String corriente = String(i);
      String presion = String(pr);
      
      String Topic=TopicBase + "/" + TopicDev + "/" + TTemp ;
      rsus=pubicatopic_mqtt(Topic, temperatura);
      Serial.print( "Publish Temp: ");
      Serial.println(rsus);
      Serial.print( "Temp: ");
      Serial.println(temperatura);
  
      Topic=TopicBase + "/" + TopicDev + "/" + THum ;
      rsus=pubicatopic_mqtt(Topic,humedad);
      Serial.print( "Publish Hum:");
      Serial.println(rsus);
      Serial.print( "Hum: ");
      Serial.println(humedad);
      
      Topic=TopicBase + "/" + TopicDev + "/" + TLuz ;
      rsus=pubicatopic_mqtt(Topic,luminosidad);
      Serial.print( "Publish Luminosidad: ");
      Serial.println(rsus);
      Serial.print( "Luz: ");
      Serial.println(luminosidad);
      
      Topic=TopicBase + "/" + TopicDev + "/" + TCor ;
      rsus=pubicatopic_mqtt(Topic,corriente);
      Serial.print( "Publish Corriente: ");
      Serial.println(rsus);
      Serial.print( "Cor: ");
      Serial.println(corriente);
      
      Topic=TopicBase + "/" + TopicDev + "/" + TPres ;
      rsus=pubicatopic_mqtt(Topic,presion);
      Serial.print( "Publish Presion: ");
      Serial.println(rsus);
      Serial.print( "Pres: ");
      Serial.println(presion);
      }
  }
}


void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("Configurando IO...");  
  // put your setup code here, to run once:
  pinMode(RLY_P1, OUTPUT);
  digitalWrite(RLY_P1, HIGH);
  Serial.println("Configurando DHT IO...");  
  pinMode(I2C_SDA,INPUT_PULLUP);
  pinMode(I2C_SCL,INPUT_PULLUP);
  Serial.println("Configurando ADC...");  
  Serial.println("Configurando WIFI...");  
  WiFi.mode(WIFI_STA);
  Serial.println("Configurando OTA...");  
  ArduinoOTA.setHostname((const char*) TopicDev.c_str()); // A name given to your ESP8266 module when discovering it as a port in ARDUINO IDE
  ArduinoOTA.setPassword((const char*) OTAPassword.c_str());
  delay(400);
  bool sbme=false;
	int p=3;
	while( p > 0 && ! sbme){
		p--;
		sbme=inicia_bme280();
		Serial.print("Bme return:");
		Serial.println(sbme);
		delay(4000); 
	}
	if ( p == 0 ){
    Ebme280=false;
    }

	bool sbh=false;
  p=3;
	while ( ! sbh && p > 0 ){
		p--;
		//sbh=myBH1750.begin(I2C_SDA, I2C_SCL);
		sbh=myBH1750.begin(D2, D1);
    Serial.print("Bbh return:");
		Serial.println(sbh); 
    delay(4000);
  }
	if ( p == 0 ){
    Ebh1750=false;
  }
	Serial.print("Ebme280 :");
	Serial.print(Ebme280); 
  Serial.print("  Ebh1750 :");
	Serial.println(Ebh1750); 
    
	mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(on_message);
  Serial.println("Setup Terminado...");  
     
}

void loop() {
  float l=0,t=0,h=0,pr=0,i=0;
  // put your main code here, to run repeatedly:
  if(WiFi.isConnected()){
    Serial.print("(");
    if(!mqttClient.connected()) {
      Serial.print("Conectando Cior ...");
      if ( mqttClient.connect((char*) TopicDev.c_str(), MQTT_USER, MQTT_PASS)) {
        Serial.println( "[DONE]" );
        suscribetopics_mqtt();
      } else {
         Serial.print( "[FAILED] [ rc = " );
         Serial.print( mqttClient.state() );
      }
    } else {
      if(!mqttClient.loop()){
        Serial.println("Error en Loop.");  
      }
      tiempor=millis();
      if (tiempor > tiempo + INTERVALO) {
        tiempo=tiempor;
        Serial.println(")");
        if (Ebme280){
          	t=mySensorA.readTempC();
	          pr=mySensorA.readFloatPressure()/100;
	          h=mySensorA.readFloatHumidity();
	      }
        if (Ebh1750){
          myBH1750.setSensitivity(1.00);
          l=myBH1750.readLightLevel();
        }
        float Ip=get_corriente();//obtenemos la corriente pico
        float Irms=Ip; //Intensidad RMS = Ipico/(2^1/2)
        float P=Ip*220.0; // P=IV watts
        Serial.print("Ip: ");
        Serial.print(Ip,3);
        Serial.print("A , Irms: ");
        Serial.print(Irms,3);
        Serial.print("A, Potencia: ");
        Serial.print(P,3);  
        Serial.println("W");

        Serial.print("Humedad :");
        Serial.print(h);
        Serial.print(" Temperatura :");
        Serial.print(t);
        Serial.print(" Luminosidad :");
        Serial.print(l);
        Serial.print(" Corriente :");
        Serial.print(Ip);
        Serial.print(" Presion :");
        Serial.println(pr);
        send_mqtt(t,h,l,Ip,pr);
      }
    }
    
  } else {
    Serial.print(" No Conectado.");
    Serial.println(WiFi.status());
    Serial.print(" Iniciando conexion Wifi ..");
    wifi_conectar();
  }
  delay(300);
  ArduinoOTA.handle();
 }
