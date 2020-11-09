/*
MIT License

Copyright (c) 2018 esp-rfid Community
Copyright (c) 2017 Ömer Şiar Baysal

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */
#define VERSION "1.0.2"

#include "Arduino.h"
#include <ESP8266WiFi.h>
#include <SPI.h>
#include <ESP8266mDNS.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <TimeLib.h>
#include <Ticker.h>
#include "Ntp.h"
#include <AsyncMqttClient.h>
#include <Bounce2.h>
//#include <IRremoteESP8266.h>
//#include <IRrecv.h>
//#include <IRutils.h>
//const uint16_t kRecvPin = 0;
//IRrecv irrecv(kRecvPin);

#include <SoftwareSerial.h>
char ByteReceived;
String recebido;
SoftwareSerial mySerial(3, 	1); // RX, TX


//decode_results results;
//#include <ESP32Servo.h>
//ESP32PWM pwm;


//int freq = 1000;
//Servo myServo; 
int spin;
int rgbrpin;
int rgbgpin;
int rgbbpin;

int SER   = 5;
int SER2   = 2;
int RCLK  = 4;
int SRCLK = 0;

int regis[] = {0,0,0,0,0,0,0,0};
//contador para fazer acender os LEDs em sequencia
int counter = 0;
#include <LiquidCrystal595.h>

// initialize the library with the numbers of the interface pins + the row count
// datapin, latchpin, clockpin, num_lines
LiquidCrystal595 lcd(5,4,0);

//#define DEBUG

//#include <RFTransmitter.h>

//#define NODE_ID          1
//#define OUTPUT_PIN       9

// Send on digital pin 11 and identify as node 1
//RFTransmitter transmitter(OUTPUT_PIN, NODE_ID);

#ifdef OFFICIALBOARD

#include <Wiegand.h>

WIEGAND wg;
int relayPin = 13;

#endif

#ifndef OFFICIALBOARD

#include <MFRC522.h>
#include "PN532.h"
#include <Wiegand.h>
#include "rfid125kHz.h"


MFRC522 mfrc522 = MFRC522();
PN532 pn532;
WIEGAND wg;
RFID_Reader RFIDr;

int rfidss;
int readerType;
int relayPin;

#endif

// these are from vendors
#include "webh/glyphicons-halflings-regular.woff.gz.h"
#include "webh/required.css.gz.h"
#include "webh/required.js.gz.h"

// these are from us which can be updated and changed
#include "webh/esprfid.js.gz.h"
#include "webh/esprfid.htm.gz.h"
#include "webh/index.html.gz.h"

#ifdef ESP8266
extern "C" {
	#include "user_interface.h"
}
#endif

NtpClient NTP;
AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;
WiFiEventHandler wifiDisconnectHandler, wifiConnectHandler;
Bounce button;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

unsigned long blink_ = millis();
bool wifiFlag = false;
bool configMode = false;
int wmode;
uint8_t wifipin = 255;
uint8_t buttonPin = 255;
#define LEDoff HIGH
#define LEDon LOW

// Variables for whole scope
const char *http_username = "admin";
char *http_pass = NULL;
unsigned long previousMillis = 0;
unsigned long previousLoopMillis = 0;
unsigned long currentMillis = 0;
unsigned long cooldown = 0;
unsigned long deltaTime = 0;
unsigned long uptime = 0;
bool shouldReboot = false;
bool activateRelay = false;
bool deactivateRelay = false;
bool activateLedR = false;
bool activateLedG = false;
bool activateLedB = false;
bool inAPMode = false;
bool isWifiConnected = false;
unsigned long autoRestartIntervalSeconds = 0;

bool wifiDisabled = true;
bool doDisableWifi = false;
bool doEnableWifi = false;
bool timerequest = false;
bool formatreq = false;
unsigned long wifiTimeout = 0;
unsigned long wiFiUptimeMillis = 0;
char *deviceHostname = NULL;

int mqttenabled = 0;
char *mqttTopic = NULL;
char *mhs = NULL;
char *muser = NULL;
char *mpas = NULL;
int mport;

int lockType;
int relayType;
unsigned long activateTime;
int timeZone;

unsigned long nextbeat = 0;
unsigned long interval = 1800;

#include "log.esp"
#include "mqtt.esp"
#include "helpers.esp"
#include "wsResponses.esp"
#include "rfid.esp"
#include "wifi.esp"
#include "config.esp"
#include "websocket.esp"
#include "webserver.esp"





void ICACHE_FLASH_ATTR setup()
{
#ifdef OFFICIALBOARD
	// Set relay pin to LOW signal as early as possible
	pinMode(13, OUTPUT);
	digitalWrite(13, LOW);
	delay(200);
#endif

#ifdef DEBUG
	Serial.begin(9600);
	Serial.println();

	Serial.print(F("[ INFO ] ESP RFID v"));
	Serial.println(VERSION);

	uint32_t realSize = ESP.getFlashChipRealSize();
	uint32_t ideSize = ESP.getFlashChipSize();
	FlashMode_t ideMode = ESP.getFlashChipMode();
	Serial.printf("Flash real id:   %08X\n", ESP.getFlashChipId());
	Serial.printf("Flash real size: %u\n\n", realSize);
	Serial.printf("Flash ide  size: %u\n", ideSize);
	Serial.printf("Flash ide speed: %u\n", ESP.getFlashChipSpeed());
	Serial.printf("Flash ide mode:  %s\n", (ideMode == FM_QIO ? "QIO" : ideMode == FM_QOUT ? "QOUT" : ideMode == FM_DIO ? "DIO" : ideMode == FM_DOUT ? "DOUT" : "UNKNOWN"));
	if (ideSize != realSize)
	{
		Serial.println("Flash Chip configuration wrong!\n");
	}
	else
	{
		Serial.println("Flash Chip configuration ok.\n");
	}
#endif

	if (!SPIFFS.begin())
	{
#ifdef DEBUG
		Serial.print(F("[ WARN ] Formatting filesystem..."));
#endif
		if (SPIFFS.format())
		{
			writeEvent("WARN", "sys", "Filesystem formatted", "");

#ifdef DEBUG
			Serial.println(F(" completed!"));
#endif
		}
		else
		{
#ifdef DEBUG
			Serial.println(F(" failed!"));
			Serial.println(F("[ WARN ] Could not format filesystem!"));
#endif
		}
	}
	wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);
	wifiConnectHandler = WiFi.onStationModeConnected(onWifiConnect);
	configMode = loadConfiguration();
	if (!configMode)
	{
		fallbacktoAPMode();
		configMode = false;
	}
	else {
		configMode = true;
	}
	setupWebServer();
	writeEvent("INFO", "sys", "System setup completed, running", "");
}


//Baixa todos os pinos do 74HC595
void clear(){
  for(int i=7; i >=  0; i--){
     regis[i] = LOW;
  }
} 

//função para efetivar a modificação após mudança dos pinos
void changeValues(int val){
  clear();
  if (val <8){
    regis[val] = 1;
  }
  digitalWrite(RCLK, LOW);
  for(int i=8; i >=  0; i--){
    digitalWrite(SRCLK, LOW);
    int PIN = regis[i];
    digitalWrite(SER2, PIN);
    digitalWrite(SRCLK, HIGH);
  }
  digitalWrite(RCLK, HIGH);
}


/**
 * Função que lê uma string da Serial
 * e retorna-a
 */
String leStringSerial(){
  String conteudo = "";
  char caractere;
  
  // Enquanto receber algo pela serial
  while(Serial.available() > 0) {
    // Lê byte da serial
    caractere = Serial.read();
    // Ignora caractere de quebra de linha
    if (caractere != '\n'){
      // Concatena valores
      conteudo.concat(caractere);
    }
    // Aguarda buffer serial ler próximo caractere
    delay(10);
  }
    
  Serial.print("Recebi: ");
  Serial.println(conteudo);
    
  return conteudo;
}

void ICACHE_RAM_ATTR loop()
{
	currentMillis = millis();
	deltaTime = currentMillis - previousLoopMillis;
	uptime = NTP.getUptimeSec();
	previousLoopMillis = currentMillis;

	
  // Se receber algo pela serial
  if (Serial.available() > 0){
    // Lê toda string recebida
    String recebido = leStringSerial();
	mqtt_publish_access(now(), "Sistema", "Admin", recebido, "unknown");
  }


	//Serial.println(digitalRead(RCLK));
	
	/*changeValues(counter);
	counter += 1;
	//Apos aplicar a mudança, faz um delay
	delay(600);
	if (counter >8){counter =0;}
	clear();*/
    
  
/*
	if (irrecv.decode(&results))
	{	
		
		//serialPrintUint64(results.value, HEX);
		//Serial.println("");
		//Serial.println("-");
		//Serial.println("");
		//serialPrintUint64(results.value);
		//Serial.println("");
		//Serial.println("============");

		if(results.value==16720605)
		{
			mqtt_publish_access(now(), "Sistema", "Admin", "SystemFunction", "unknown");
			mqtt_publish_access(now(), "Sistema", "Admin", "SystemFunction", "PlayPause");			
		}
		else if(results.value==16761405)
		{
			mqtt_publish_access(now(), "Sistema", "Admin", "SystemFunction", "unknown");
			mqtt_publish_access(now(), "Sistema", "Admin", "SystemFunction", "Next");			
		}
		else if(results.value==16712445)
		{
			mqtt_publish_access(now(), "Sistema", "Admin", "SystemFunction", "unknown");
			mqtt_publish_access(now(), "Sistema", "Admin", "SystemFunction", "Prev");			
		}
		else if(results.value==16748655)
		{
			mqtt_publish_access(now(), "Sistema", "Admin", "SystemFunction", "unknown");
			mqtt_publish_access(now(), "Sistema", "Admin", "SystemFunction", "VolUp");			
		}
		else if(results.value==16754775)
		{
			mqtt_publish_access(now(), "Sistema", "Admin", "SystemFunction", "unknown");
			mqtt_publish_access(now(), "Sistema", "Admin", "SystemFunction", "VolDown");			
		}
		else if(results.value==16724175)
		{
			mqtt_publish_access(now(), "Sistema", "Admin", "SystemFunction", "unknown");
			mqtt_publish_access(now(), "Sistema", "Admin", "SystemFunction", "Num1");
			const char *sinalbackup = "Num1";
		}
		else if(results.value==16718055)
		{
			mqtt_publish_access(now(), "Sistema", "Admin", "SystemFunction", "unknown");
			mqtt_publish_access(now(), "Sistema", "Admin", "SystemFunction", "Num2");	
			const char *sinalbackup = "Num2";
		}
		else if(results.value==16743045)
		{
			mqtt_publish_access(now(), "Sistema", "Admin", "SystemFunction", "unknown");
			mqtt_publish_access(now(), "Sistema", "Admin", "SystemFunction", "Num3");
			const char *sinalbackup = "Num3";
		}
		


		irrecv.resume();  // Receive the next value
  	}*/
	
	

	button.update();
	if (button.fell()) 
	{
#ifdef DEBUG
		Serial.println("Button has been pressed");
#endif
		writeLatest("", "(used open/close button)", 1);
		activateRelay = true;
	}

	if (wifipin != 255 && configMode && !wmode)
	{
		if (!wifiFlag)
		{
			if ((currentMillis - blink_) > 500)
			{
				blink_ = currentMillis;
				digitalWrite(wifipin, !digitalRead(wifipin));
			}
		}
		else
		{
			if (!(digitalRead(wifipin)==LEDon)) digitalWrite(wifipin, LEDon);
		}
	}

	if (currentMillis >= cooldown)
	{
		rfidloop();
	}

	// Continuous relay mode
	if (lockType == 1)
	{
		if (activateRelay)
		{
			// currently OFF, need to switch ON
			if (digitalRead(relayPin) == !relayType)
			{
#ifdef DEBUG
				Serial.print("mili : ");
				Serial.println(millis());
				Serial.println("activating relay now");
#endif
				digitalWrite(relayPin, relayType);
			}
			else	// currently ON, need to switch OFF
			{
#ifdef DEBUG
				Serial.print("mili : ");
				Serial.println(millis());
				Serial.println("deactivating relay now");
#endif				
				digitalWrite(relayPin, !relayType);
			}
			activateRelay = false;	
		}
	}
	else if (lockType == 0)	// momentary relay mode
	{
		if (activateLedR)
		{			
			//digitalWrite(rgbrpin, HIGH);
			//delay(200);
			//digitalWrite(rgbrpin, LOW);
			activateLedR = false;
		}
		if (activateLedG)
		{			
			//digitalWrite(rgbgpin, HIGH);
			//delay(200);
			//digitalWrite(rgbgpin, LOW);
			activateLedG = false;
		}
		
		if (activateRelay)
		{
#ifdef DEBUG
			Serial.print("mili : ");
			Serial.println(millis());
			Serial.println("activating relay now");
#endif
			digitalWrite(relayPin, relayType);
			previousMillis = millis();
			activateRelay = false;
			deactivateRelay = true;
		}
		else if ((currentMillis - previousMillis >= activateTime) && (deactivateRelay))
		{
#ifdef DEBUG
			Serial.println(currentMillis);
			Serial.println(previousMillis);
			Serial.println(activateTime);
			Serial.println(activateRelay);
			Serial.println("deactivate relay after this");
			Serial.print("mili : ");
			Serial.println(millis());
#endif
			digitalWrite(relayPin, !relayType);
			deactivateRelay = false;
		}
	}

	if (formatreq)
	{
#ifdef DEBUG
		Serial.println(F("[ WARN ] Factory reset initiated..."));
#endif
		SPIFFS.end();
		ws.enable(false);
		SPIFFS.format();
		ESP.restart();
	}

	if (timerequest)
	{
		timerequest = false;
		sendTime();
	}

	if (autoRestartIntervalSeconds > 0 && uptime > autoRestartIntervalSeconds * 1000)
	{
		writeEvent("INFO", "sys", "System is going to reboot", "");
#ifdef DEBUG
		Serial.println(F("[ WARN ] Auto restarting..."));
#endif
		shouldReboot = true;
	}

	if (shouldReboot)
	{
		writeEvent("INFO", "sys", "System is going to reboot", "");
#ifdef DEBUG
		Serial.println(F("[ INFO ] Rebooting..."));
#endif
		ESP.restart();
	}

	if (isWifiConnected)
	{
		wiFiUptimeMillis += deltaTime;
	}

	if (wifiTimeout > 0 && wiFiUptimeMillis > (wifiTimeout * 1000) && isWifiConnected == true)
	{
		writeEvent("INFO", "wifi", "WiFi is going to be disabled", "");
		doDisableWifi = true;
	}

	if (doDisableWifi == true)
	{
		doDisableWifi = false;
		wiFiUptimeMillis = 0;
		disableWifi();
	}
	else if (doEnableWifi == true)
	{
		writeEvent("INFO", "wifi", "Enabling WiFi", "");
		doEnableWifi = false;
		if (!isWifiConnected)
		{
			wiFiUptimeMillis = 0;
			enableWifi();
		}
	}

	if (mqttenabled == 1)
	{
		if (mqttClient.connected())
		{
			if ((unsigned)now() > nextbeat)
			{
				mqtt_publish_heartbeat(now());
				nextbeat = (unsigned)now() + interval;
#ifdef DEBUG
				Serial.print("[ INFO ] Nextbeat=");
				Serial.println(nextbeat);
#endif
			}
		}
	}
}
