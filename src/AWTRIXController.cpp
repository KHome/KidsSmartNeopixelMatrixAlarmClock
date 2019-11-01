// AWTRIX Controller
// Copyright (C) 2019
// by Blueforcer & Mazze2000
// Version 0.17, KHome added Adafruit 7 Segment LCD support
// Version 0.18, KHome added Button to turn mp3 off
// Version 0.19, KHome added flicker (strobe) mode

#include <FS.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266httpUpdate.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>
#include <FastLED.h>
#include <FastLED_NeoMatrix.h>
#include <Fonts/TomThumb.h>
#include <Fonts/Tiny3x3a2pt7b.h>
#include <Fonts/Org_01.h>
#include <Fonts/Picopixel.h>
#include <LightDependentResistor.h>
#include <Wire.h>
#include <SparkFun_APDS9960.h>
#include "SoftwareSerial.h"
#include <DFPlayerMini_Fast.h>
#include <WiFiManager.h>
#include <DoubleResetDetect.h>
#include <Wire.h>
#include <BME280_t.h>
#include "Adafruit_HTU21DF.h"
#include "icons.h" // thanks to Nygma2004: https://github.com/nygma2004/nrmetric
#include "OneButton.h" // thanks to Steve: https://steve.fi/Hardware/d1-alarm-button/

// instantiate temp sensor
BME280<> BMESensor;
Adafruit_HTU21DF htu = Adafruit_HTU21DF();

int tempState = 0;	 // 0 = None ; 1 = BME280 ; 2 = htu21d
int audioState = true;	// 0 = false ; 1 = true
int gestureState = false;  // 0 = false ; 1 = true
int ldrState = 1;		   // 0 = None
bool USBConnection = false; // true = usb...
bool MatrixType2  = false; //depends on matrix
int matrixTempCorrection = 0;
int seg7lcdState = true; // 0 = false; 1 = true
int buttonState = true;  // 0 = false; 1 = true

uint16_t myflickercounter = 0; // can be set by passing values
int myFlickerTime = millis();// timer for millis
boolean mytoogle = false; // toogle screen brigthness
uint8_t mybrightnessbefore = 30; // remember old brightness


String version = "0.19";
char awtrix_server[16] = "192.168.178.2"; //bypass initial setting

// could be later moved to http config site
const char mqttfeedbackchannel[14] = "matrix1Client";
const char mqttreceivechannel[15] = "awtrixmatrax/#";
const char mqttuser[3] = "hi";
const char mqttpass[8] = "hohoho1";
int mqttport = 1883;
const char wifipass[9] = "awtrixxx";
String jsonconfigfile = "/awtrix1.json";

IPAddress Server;
WiFiClient espClient;
PubSubClient client(espClient);

WiFiManager wifiManager;

//update
ESP8266WebServer server(80);
const char *serverIndex = "<form method='POST' action='/update' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>";

//resetdetector
#define DRD_TIMEOUT 8.0 // changed for case of matrix issue
#define DRD_ADDRESS 0x00
DoubleResetDetect drd(DRD_TIMEOUT, DRD_ADDRESS);

bool firstStart = true;
int myTime;  //need for loop
int myTime2; //need for loop
int myTime3; //need for loop3
int myCounter;
int myCounter2;
boolean getLength = true;
int prefix = -5;
boolean awtrixFound = false;
int myPointer[14];
uint32_t messageLength = 0;
uint32_t SavemMessageLength = 0;

//USB Connection:
byte myBytes[1000];
int bufferpointer;

//Zum speichern...
int cfgStart = 0;

//flag for saving data
bool shouldSaveConfig = false;

/// LDR Config
#define LDR_RESISTOR 1000 //ohms
#define LDR_PIN A0
#define LDR_PHOTOCELL LightDependentResistor::GL5516
LightDependentResistor photocell(LDR_PIN, ldrState, LDR_PHOTOCELL);

// Gesture Sensor
#define APDS9960_INT D6
#define I2C_SDA D3
#define I2C_SCL D1
SparkFun_APDS9960 apds = SparkFun_APDS9960();
volatile bool isr_flag = 0;

#ifndef ICACHE_RAM_ATTR
#define ICACHE_RAM_ATTR IRAM_ATTR
#endif

bool updating = false;

// Audio
DFPlayerMini_Fast myMP3;
SoftwareSerial mySoftwareSerial(D7, D5); // RX, TX

// Matrix Settings
CRGB leds[256];
FastLED_NeoMatrix *matrix;

// for 7 Segement LCD, Adafruit_LEDBackpack.h
#ifndef _BV
  #define _BV(bit) (1<<(bit))
#endif
Adafruit_7segment lcd7matrix = Adafruit_7segment();

// for Button Detection connected to D8
OneButton button(D8, true);
//
// Are there pending clicks to process?
//
volatile bool short_click = false;
volatile bool long_click = false;


static byte c1; // Last character buffer
byte utf8ascii(byte ascii)
{
	if (ascii < 128) // Standard ASCII-set 0..0x7F handling
	{
		c1 = 0;
		return (ascii);
	}
	// get previous input
	byte last = c1; // get last char
	c1 = ascii;		// remember actual character
	switch (last)   // conversion depending on first UTF8-character
	{
	case 0xC2:
		return (ascii)-34;
		break;
	case 0xC3:
		return (ascii | 0xC0) - 34;
		break;
	case 0x82:
		if (ascii == 0xAC)
			return (0xEA);
	}
	return (0);
}

bool saveConfig()
{
	DynamicJsonBuffer jsonBuffer;
	JsonObject &json = jsonBuffer.createObject();
	json["awtrix_server"] = awtrix_server;
	json["MatrixType"] = MatrixType2;
	json["temp"] = tempState;
	json["usbWifi"] = USBConnection;
	json["ldr"] = ldrState;
	json["gesture"] = gestureState;
	json["audio"] = audioState;
	json["matrixCorrection"] = matrixTempCorrection;
  json["seg7lcd"] = seg7lcdState;
  json["button"] = buttonState;

	File configFile = SPIFFS.open(jsonconfigfile, "w");
	if (!configFile)
	{
		if (!USBConnection)
		{
			Serial.println("failed to open config file for writing");
		}

		return false;
	}

	json.printTo(configFile);
	configFile.close();
	//end save
	return true;
}

void debuggingWithMatrix(String text)
{
	matrix->setCursor(7, 6);
	matrix->clear();
	matrix->print(text);
	matrix->show();
}

void sendToServer(String s)
{
	if (USBConnection)
	{
		uint32_t laenge = s.length();
		Serial.printf("%c%c%c%c%s", (laenge & 0xFF000000) >> 24, (laenge & 0x00FF0000) >> 16, (laenge & 0x0000FF00) >> 8, (laenge & 0x000000FF), s.c_str());
	}
	else
	{
		client.publish(mqttfeedbackchannel, s.c_str()); //change to 2 for another one
	}
}

void logToServer(String s)
{
	StaticJsonBuffer<400> jsonBuffer;
	JsonObject &root = jsonBuffer.createObject();
	root["type"] = "log";
	root["msg"] = s;
	String JS;
	root.printTo(JS);
	sendToServer(JS);
}

String utf8ascii(String s)
{
	String r = "";
	char c;
	for (unsigned int i = 0; i < s.length(); i++)
	{
		c = utf8ascii(s.charAt(i));
		if (c != 0)
			r += c;
	}
	return r;
}

void hardwareAnimatedUncheck(int typ, int x, int y)
{
	int wifiCheckTime = millis();
	int wifiCheckPoints = 0;
  matrix->setBrightness(20); // set initially to very low brightness
	while (millis() - wifiCheckTime < 2000)
	{
		while (wifiCheckPoints < 10)
		{
			matrix->clear();
			switch (typ)
			{
			case 0:
				matrix->setCursor(7, 6);
				matrix->print("WiFi");
				break;
			case 1:
				matrix->setCursor(1, 6);
				matrix->print("Server");
				break;
			case 2:
				matrix->setCursor(7, 6);
				matrix->print("Temp");
				break;
			case 4:
				matrix->setCursor(3, 6);
				matrix->print("Gest.");
				break;
			case 6:
				matrix->setCursor(1, 6);
				matrix->print("LCD 7s");
				break;
      case 7:
  			matrix->setCursor(1, 6);
  			matrix->print("Button");
  			break;
			}

			switch (wifiCheckPoints)
			{
			case 9:
				matrix->drawPixel(x, y + 4, 0xF800);
			case 8:
				matrix->drawPixel(x - 1, y + 3, 0xF800);
			case 7:
				matrix->drawPixel(x - 2, y + 2, 0xF800);
			case 6:
				matrix->drawPixel(x - 3, y + 1, 0xF800);
			case 5:
				matrix->drawPixel(x - 4, y, 0xF800);
			case 4:
				matrix->drawPixel(x - 4, y + 4, 0xF800);
			case 3:
				matrix->drawPixel(x - 3, y + 3, 0xF800);
			case 2:
				matrix->drawPixel(x - 2, y + 2, 0xF800);
			case 1:
				matrix->drawPixel(x - 1, y + 1, 0xF800);
			case 0:
				matrix->drawPixel(x, y, 0xF800);
				break;
			}
			wifiCheckPoints++;
			matrix->show();
			delay(100);
		}
	}
}

void hardwareAnimatedCheck(int typ, int x, int y)
{
	int wifiCheckTime = millis();
	int wifiCheckPoints = 0;
  matrix->setBrightness(20); // set initially to very low brightness
  matrix->show();
	while (millis() - wifiCheckTime < 2000)
	{
		while (wifiCheckPoints < 8)
		{
			matrix->clear();
			switch (typ)
			{
			case 0:
				matrix->setCursor(7, 6);
        matrix->print("WiFi");
				break;
			case 1:
				matrix->setCursor(1, 6);
				matrix->print("Server");
				break;
			case 2:
				matrix->setCursor(7, 6);
				matrix->print("Temp");
				break;
			case 3:
				matrix->setCursor(3, 6);
				matrix->print("Audio");
				break;
			case 4:
				matrix->setCursor(3, 6);
				matrix->print("Gest.");
				break;
			case 5:
				matrix->setCursor(7, 6);
				matrix->print("LDR");
				break;
			case 6:
				matrix->setCursor(1, 6);
				matrix->print("LCD 7s");
				break;
      case 7:
        matrix->setCursor(1, 6);
        matrix->print("Button");
        break;
			}

			switch (wifiCheckPoints)
			{
			case 6:
				matrix->drawPixel(x, y, 0x07E0);
			case 5:
				matrix->drawPixel(x - 1, y + 1, 0x07E0);
			case 4:
				matrix->drawPixel(x - 2, y + 2, 0x07E0);
			case 3:
				matrix->drawPixel(x - 3, y + 3, 0x07E0);
			case 2:
				matrix->drawPixel(x - 4, y + 4, 0x07E0);
			case 1:
				matrix->drawPixel(x - 5, y + 3, 0x07E0);
			case 0:
				matrix->drawPixel(x - 6, y + 2, 0x07E0);
				break;
			}
			wifiCheckPoints++;
			matrix->show();
			delay(100);
		}
	}
}

void serverSearch(int rounds, int typ, int x, int y)
{
	matrix->clear();
	matrix->setTextColor(0x55FF);
	matrix->setCursor(1, 6);
	matrix->print("Server");

	if (typ == 0)
	{
		switch (rounds)
		{
		case 3:
			matrix->drawPixel(x, y, 0x22ff);
			matrix->drawPixel(x + 1, y + 1, 0x22ff);
			matrix->drawPixel(x + 2, y + 2, 0x22ff);
			matrix->drawPixel(x + 3, y + 3, 0x22ff);
			matrix->drawPixel(x + 2, y + 4, 0x22ff);
			matrix->drawPixel(x + 1, y + 5, 0x22ff);
			matrix->drawPixel(x, y + 6, 0x22ff);
		case 2:
			matrix->drawPixel(x - 1, y + 2, 0x22ff);
			matrix->drawPixel(x, y + 3, 0x22ff);
			matrix->drawPixel(x - 1, y + 4, 0x22ff);
		case 1:
			matrix->drawPixel(x - 3, y + 3, 0x22ff);
		case 0:
			break;
		}
	}
	else if (typ == 1)
	{

		switch (rounds)
		{
		case 12:
			//matrix->drawPixel(x+3, y+2, 0x22ff);
			matrix->drawPixel(x + 3, y + 3, 0x22ff);
			//matrix->drawPixel(x+3, y+4, 0x22ff);
			matrix->drawPixel(x + 3, y + 5, 0x22ff);
			//matrix->drawPixel(x+3, y+6, 0x22ff);
		case 11:
			matrix->drawPixel(x + 2, y + 2, 0x22ff);
			matrix->drawPixel(x + 2, y + 3, 0x22ff);
			matrix->drawPixel(x + 2, y + 4, 0x22ff);
			matrix->drawPixel(x + 2, y + 5, 0x22ff);
			matrix->drawPixel(x + 2, y + 6, 0x22ff);
		case 10:
			matrix->drawPixel(x + 1, y + 3, 0x22ff);
			matrix->drawPixel(x + 1, y + 4, 0x22ff);
			matrix->drawPixel(x + 1, y + 5, 0x22ff);
		case 9:
			matrix->drawPixel(x, y + 4, 0x22ff);
		case 8:
			matrix->drawPixel(x - 1, y + 4, 0x22ff);
		case 7:
			matrix->drawPixel(x - 2, y + 4, 0x22ff);
		case 6:
			matrix->drawPixel(x - 3, y + 4, 0x22ff);
		case 5:
			matrix->drawPixel(x - 3, y + 5, 0x22ff);
		case 4:
			matrix->drawPixel(x - 3, y + 6, 0x22ff);
		case 3:
			matrix->drawPixel(x - 3, y + 7, 0x22ff);
		case 2:
			matrix->drawPixel(x - 4, y + 7, 0x22ff);
		case 1:
			matrix->drawPixel(x - 5, y + 7, 0x22ff);
		case 0:
			break;
		}
	}
	matrix->show();
}

void hardwareAnimatedSearch(int typ, int x, int y)
{
  matrix->setBrightness(20); // set initially to very low brightness
  matrix->show();

	for (int i = 0; i < 4; i++)
	{
		matrix->clear();
		matrix->setTextColor(0x22FF);
		if (typ == 0)
		{
			matrix->setCursor(7, 6);
			matrix->print("WiFi");
		}
		else if (typ == 1)
		{
			matrix->setCursor(1, 6);
			matrix->print("Server");
		}
		switch (i)
		{
		case 3:
			matrix->drawPixel(x, y, 0x22ff);
			matrix->drawPixel(x + 1, y + 1, 0x22ff);
			matrix->drawPixel(x + 2, y + 2, 0x22ff);
			matrix->drawPixel(x + 3, y + 3, 0x22ff);
			matrix->drawPixel(x + 2, y + 4, 0x22ff);
			matrix->drawPixel(x + 1, y + 5, 0x22ff);
			matrix->drawPixel(x, y + 6, 0x22ff);
		case 2:
			matrix->drawPixel(x - 1, y + 2, 0x22ff);
			matrix->drawPixel(x, y + 3, 0x22ff);
			matrix->drawPixel(x - 1, y + 4, 0x22ff);
		case 1:
			matrix->drawPixel(x - 3, y + 3, 0x22ff);
		case 0:
			break;
		}
		matrix->show();
		delay(100);
	}
}

void utf8ascii(char *s)
{
	int k = 0;
	char c;
	for (unsigned int i = 0; i < strlen(s); i++)
	{
		c = utf8ascii(s[i]);
		if (c != 0)
			s[k++] = c;
	}
	s[k] = 0;
}

String GetChipID()
{
	return String(ESP.getChipId());
}

int GetRSSIasQuality(int rssi)
{
	int quality = 0;

	if (rssi <= -100)
	{
		quality = 0;
	}
	else if (rssi >= -50)
	{
		quality = 100;
	}
	else
	{
		quality = 2 * (rssi + 100);
	}
	return quality;
}

void updateMatrix(byte payload[], int length)
{
	int y_offset = 5;
	if (firstStart)
	{
		//hardwareAnimatedCheck(1, 30, 2);
		firstStart = false;
	}

	switch (payload[0])
	{
	case 0:
	{
		//Command 0: DrawText

		//Prepare the coordinates
		uint16_t x_coordinate = int(payload[1] << 8) + int(payload[2]);
		uint16_t y_coordinate = int(payload[3] << 8) + int(payload[4]);

		//matrix->setCursor(x_coordinate + 1, y_coordinate + y_offset); # +1 needed?
    matrix->setCursor(x_coordinate, y_coordinate + y_offset);
		matrix->setTextColor(matrix->Color(payload[5], payload[6], payload[7]));

		String myText = "";
		for (int i = 8; i < length; i++)
		{
			char c = payload[i];
			myText += c;
		}
		matrix->print(utf8ascii(myText));
		break;
	}
	case 1:
	{
		//Command 1: DrawBMP

		//Prepare the coordinates
		uint16_t x_coordinate = int(payload[1] << 8) + int(payload[2]);
		uint16_t y_coordinate = int(payload[3] << 8) + int(payload[4]);

		int16_t width = payload[5];
		int16_t height = payload[6];

		unsigned short colorData[width * height];

		for (int i = 0; i < width * height * 2; i++)
		{
			colorData[i / 2] = (payload[i + 7] << 8) + payload[i + 1 + 7];
			i++;
		}

		for (int16_t j = 0; j < height; j++, y_coordinate++)
		{
			for (int16_t i = 0; i < width; i++)
			{
				matrix->drawPixel(x_coordinate + i, y_coordinate, (uint16_t)colorData[j * width + i]);
			}
		}
		break;
	}

	case 2:
	{
		//Command 2: DrawCircle

		//Prepare the coordinates
		uint16_t x0_coordinate = int(payload[1] << 8) + int(payload[2]);
		uint16_t y0_coordinate = int(payload[3] << 8) + int(payload[4]);
		uint16_t radius = payload[5];
		matrix->drawCircle(x0_coordinate, y0_coordinate, radius, matrix->Color(payload[6], payload[7], payload[8]));
		break;
	}
	case 3:
	{
		//Command 3: FillCircle

		//Prepare the coordinates
		uint16_t x0_coordinate = int(payload[1] << 8) + int(payload[2]);
		uint16_t y0_coordinate = int(payload[3] << 8) + int(payload[4]);
		uint16_t radius = payload[5];
		matrix->fillCircle(x0_coordinate, y0_coordinate, radius, matrix->Color(payload[6], payload[7], payload[8]));
		break;
	}
	case 4:
	{
		//Command 4: DrawPixel

		//Prepare the coordinates
		uint16_t x0_coordinate = int(payload[1] << 8) + int(payload[2]);
		uint16_t y0_coordinate = int(payload[3] << 8) + int(payload[4]);
		matrix->drawPixel(x0_coordinate, y0_coordinate, matrix->Color(payload[5], payload[6], payload[7]));
		break;
	}
	case 5:
	{
		//Command 5: DrawRect

		//Prepare the coordinates
		uint16_t x0_coordinate = int(payload[1] << 8) + int(payload[2]);
		uint16_t y0_coordinate = int(payload[3] << 8) + int(payload[4]);
		int16_t width = payload[5];
		int16_t height = payload[6];

		matrix->drawRect(x0_coordinate, y0_coordinate, width, height, matrix->Color(payload[7], payload[8], payload[9]));
		break;
	}
	case 6:
	{
		//Command 6: DrawLine

		//Prepare the coordinates
		uint16_t x0_coordinate = int(payload[1] << 8) + int(payload[2]);
		uint16_t y0_coordinate = int(payload[3] << 8) + int(payload[4]);
		uint16_t x1_coordinate = int(payload[5] << 8) + int(payload[6]);
		uint16_t y1_coordinate = int(payload[7] << 8) + int(payload[8]);
		matrix->drawLine(x0_coordinate, y0_coordinate, x1_coordinate, y1_coordinate, matrix->Color(payload[9], payload[10], payload[11]));
		break;
	}

	case 7:
	{
		//Command 7: FillMatrix

		matrix->fillScreen(matrix->Color(payload[1], payload[2], payload[3]));
		break;
	}

	case 8:
	{
		//Command 8: Show
		matrix->show();
		break;
	}
	case 9:
	{
		//Command 9: Clear
		matrix->clear();
		break;
	}
	case 10:
	{
		//Command 10: Play
		myMP3.volume(payload[2]);
		delay(10);
		myMP3.play(payload[1]);
		break;
	}
	case 11:
	{
		//Command 11: reset
		ESP.reset();
		break;
	}
	case 12:
	{
		//Command 12: GetMatrixInfo
		StaticJsonBuffer<400> jsonBuffer;
		JsonObject &root = jsonBuffer.createObject();
		root["type"] = "MatrixInfo";
		root["version"] = version;
		root["wifirssi"] = String(WiFi.RSSI());
		root["wifiquality"] = GetRSSIasQuality(WiFi.RSSI());
		root["wifissid"] = WiFi.SSID();
		root["IP"] = WiFi.localIP().toString();
		if (ldrState != 0)
		{
			root["LUX"] = photocell.getCurrentLux();
		}
		else
		{
			root["LUX"] = NULL;
		}

		BMESensor.refresh();
		if (tempState == 1)
		{
			root["Temp"] = BMESensor.temperature;
			root["Hum"] = BMESensor.humidity;
			root["hPa"] = BMESensor.pressure;
		}
		else if (tempState == 2)
		{
			root["Temp"] = htu.readTemperature();
			root["Hum"] = htu.readHumidity();
			root["hPa"] = 0;
		}
		else
		{
			root["Temp"] = 0;
			root["Hum"] = 0;
			root["hPa"] = 0;
		}

		String JS;
		root.printTo(JS);
		sendToServer(JS);
		break;
	}
	case 13:
	{
    //Command 13: SetMatrix Brightness
		matrix->setBrightness(payload[1]);
    //mybrightnessbefore = uint8_t(payload[1]);
		break;
	}
	case 14:
	{
    //Command 14: SaveConfig
		tempState = (int)payload[1];
		audioState = (int)payload[2];
		gestureState = (int)payload[3];
		ldrState = int(payload[4] << 8) + int(payload[5]);
		matrixTempCorrection = (int)payload[6];
		seg7lcdState = (int)payload[7];
    buttonState =  (int)payload[8];
		matrix->clear();
		matrix->setCursor(6, 6);
		matrix->setTextColor(matrix->Color(0, 255, 50));
		matrix->print("SAVED!");
		matrix->show();
		delay(2000);
		if (saveConfig())
		{
			ESP.reset();
		}
		break;
	}
	case 15:
	{
    //Command 15: ResetWifI Setting
		wifiManager.resetSettings();
		ESP.reset();
		break;
	}
	case 16:
	{
    //Command 16: Ping Server
		sendToServer("ping");
		break;
	}
	case 17:
	{
    //Command 17: Set 7-LED Segment Matrix via I2C
    // https://learn.adafruit.com/adafruit-7-segment-led-featherwings/arduino-usage
    lcd7matrix.clear();
		uint8_t brightness = payload[1];
		lcd7matrix.setBrightness(brightness);
		lcd7matrix.blinkRate(uint8_t(payload[2]));
		lcd7matrix.writeDigitNum(uint8_t(0), uint8_t(payload[3]));
		lcd7matrix.writeDigitNum(uint8_t(1), uint8_t(payload[4]));
		lcd7matrix.writeDigitNum(uint8_t(3), uint8_t(payload[5]));
		lcd7matrix.writeDigitNum(uint8_t(4), uint8_t(payload[6]));
    if (int(payload[7]) > 0) {
      lcd7matrix.drawColon(true);
    }
    //lcd7matrix.print(0000);
    //lcd7matrix.clear();
		lcd7matrix.writeDisplay();
		break;
	}
  case 18:
  {
    //Command 18: Set Icon
    //matrix->clear();
    uint16_t x0_coordinate = int(payload[1] << 8) + int(payload[2]);
    uint16_t y0_coordinate = int(payload[3] << 8) + int(payload[4]);
    //matrix->drawPixel(x0_coordinate, y0_coordinate,
    //matrix->drawRGBBitmap(int16_t x, int16_t y, const uint16_t *bitmap, int16_t w, int16_t h)
    matrix->drawRGBBitmap(x0_coordinate, y0_coordinate,  RGB_bmp[uint8_t(payload[5])], 8,8);
		break;
  }
  case 20:
  {
    //Command 20: DrawText with Font and wrap if needed

    //Prepare the coordinates
    uint16_t x_coordinate = int(payload[1] << 8) + int(payload[2]);
    uint16_t y_coordinate = int(payload[3] << 8) + int(payload[4]);

    //matrix->setCursor(x_coordinate + 1, y_coordinate + y_offset); # +1 needed?
    matrix->setCursor(x_coordinate, y_coordinate + y_offset);
    matrix->setTextColor(matrix->Color(payload[5], payload[6], payload[7]));

    String myText = "";
    for (int i = 8; i < length; i++)
    {
      char c = payload[i];
      myText += c;
    }
    matrix->setFont(&TomThumb);
    if (int(payload[8]) == 1) {
      matrix->setFont(&Picopixel);
    }
    if (int(payload[8]) == 2) {
      matrix->setFont(&Tiny3x3a2pt7b);
    }
    if (int(payload[8]) == 3) {
      matrix->setFont(&Org_01);
    }
    matrix->setTextWrap(false);
    if (int(payload[9]) > 0) {
      matrix->setTextWrap(true);
    }

    matrix->print(utf8ascii(myText));
    break;
  }
  case 22:
  {
    //Command 22: Set counter to define flickerrate from bright to blank
    myflickercounter = int(payload[1] << 8) + int(payload[2]);
    break;
  }
	}
}

void callback(char *topic, byte *payload, unsigned int length)
{
	updateMatrix(payload, length);
}

void reconnect()
{
	if (!USBConnection)
	{
		while (!client.connected())
		{
			Serial.println("reconnecting to " + String(awtrix_server));
			String clientId = "Matrix8x32-";
			clientId += String(random(0xffff), HEX);
			hardwareAnimatedSearch(1, 28, 0);
			if (client.connect(clientId.c_str(),mqttuser,mqttpass )) //username, pa
			{
				Serial.println("connected to mqtt-server!");
      	//client.subscribe("matrix1/#"); //change for 2 matrices at one MQTT Server
        client.subscribe(mqttreceivechannel);
        client.publish(mqttfeedbackchannel, "connected"); // change also here

        // New feedback for visual feedback
        matrix->clear();
        matrix->setCursor(6, 6);
        matrix->setTextColor(matrix->Color(0, 255, 50));
        matrix->print("online");
        matrix->show();
        Serial.println("Matrix show update done");
			}
		}
	}
}

void ICACHE_RAM_ATTR interruptRoutine()
{
	isr_flag = 1;
}

void handleGesture()
{
	String control;
	if (apds.isGestureAvailable())
	{
		switch (apds.readGesture())
		{
		case DIR_UP:
			control = "UP";
			break;
		case DIR_DOWN:
			control = "DOWN";
			break;
		case DIR_LEFT:
			control = "LEFT";
			break;
		case DIR_RIGHT:
			control = "RIGHT";
			break;
		case DIR_NEAR:
			control = "NEAR";
			break;
		case DIR_FAR:
			control = "FAR";
			break;
		default:
			control = "NONE";
		}
		StaticJsonBuffer<200> jsonBuffer;
		JsonObject &root = jsonBuffer.createObject();
		root["type"] = "gesture";
		root["gesture"] = control;
		String JS;
		root.printTo(JS);
		sendToServer(JS);
	}
}

uint32_t Wheel(byte WheelPos, int pos)
{
	if (WheelPos < 85)
	{
		return matrix->Color((WheelPos * 3) - pos, (255 - WheelPos * 3) - pos, 0);
	}
	else if (WheelPos < 170)
	{
		WheelPos -= 85;
		return matrix->Color((255 - WheelPos * 3) - pos, 0, (WheelPos * 3) - pos);
	}
	else
	{
		WheelPos -= 170;
		return matrix->Color(0, (WheelPos * 3) - pos, (255 - WheelPos * 3) - pos);
	}
}

void flashProgress(unsigned int progress, unsigned int total)
{
	matrix->setBrightness(80);
	long num = 32 * 8 * progress / total;
	for (unsigned char y = 0; y < 8; y++)
	{
		for (unsigned char x = 0; x < 32; x++)
		{
			if (num-- > 0)
				matrix->drawPixel(x, 8 - y - 1, Wheel((num * 16) & 255, 0));
		}
	}
	matrix->setCursor(0, 6);
	matrix->setTextColor(matrix->Color(255, 255, 255));
	matrix->print("FLASHING");
	matrix->show();
}

void saveConfigCallback()
{
	if (!USBConnection)
	{
		Serial.println("Should save config");
	}
	shouldSaveConfig = true;
}

void configModeCallback(WiFiManager *myWiFiManager)
{
	if (!USBConnection)
	{
		Serial.println("Entered config mode");
		Serial.println(WiFi.softAPIP());
		Serial.println(myWiFiManager->getConfigPortalSSID());
	}
	matrix->clear();
	matrix->setCursor(3, 6);
	matrix->setTextColor(matrix->Color(0, 255, 50));
	matrix->print("Hotspot");
	matrix->show();
}


//
// Record that a short-click happened.
//
void on_short_click()
{
    short_click = true;
}

//
// Record that a long-click happened.
//
void on_long_click()
{
    long_click = true;
}

//
// The callbacks just record the pending state, and here we
// process any of them that were raised.
//
void handlePendingButtons()
{

    //
    // If we have a pending-short-click then handle it
    //
    if (short_click)
    {
        short_click = false;
        StaticJsonBuffer<200> jsonBuffer;
    		JsonObject &root = jsonBuffer.createObject();
    		root["type"] = "button";
    		root["button"] = "short";
    		String JS;
    		root.printTo(JS);
    		sendToServer(JS);
        if (audioState)
        {
          myMP3.play(0);
        }
    }
    //
    // If we have a pending long-click then handle it.
    //
    if (long_click)
    {
        long_click = false;
        StaticJsonBuffer<200> jsonBuffer;
        JsonObject &root = jsonBuffer.createObject();
        root["type"] = "button";
        root["button"] = "long";
        String JS;
        root.printTo(JS);
        sendToServer(JS);
        if (audioState)
        {
          myMP3.play(0);
        }
    }
}


void setup()
{
	delay(2000);
	Serial.setRxBufferSize(1024);
	Serial.begin(115200);
	if (!USBConnection)
	{
		Serial.println("");
		Serial.println(version);
	}



	if (SPIFFS.begin())
	{
		//if file not exists
		if (!(SPIFFS.exists(jsonconfigfile)))
		{
			SPIFFS.open(jsonconfigfile, "w+");
			if (!USBConnection)
			{
				Serial.println("make File...");
			}
		}

		File configFile = SPIFFS.open(jsonconfigfile, "r");
		if (configFile)
		{
			size_t size = configFile.size();
			// Allocate a buffer to store contents of the file.
			std::unique_ptr<char[]> buf(new char[size]);
			configFile.readBytes(buf.get(), size);
			DynamicJsonBuffer jsonBuffer;
			JsonObject &json = jsonBuffer.parseObject(buf.get());
			if (json.success())
			{
				if (!USBConnection)
				{
					Serial.println("\nparsed json");
				}
				strcpy(awtrix_server, json["awtrix_server"]);
				USBConnection = json["usbWifi"].as<bool>();
				audioState = json["audio"].as<int>();
				gestureState = json["gesture"].as<int>();
				ldrState = json["ldr"].as<int>();
				tempState = json["temp"].as<int>();
				MatrixType2 = json["MatrixType"].as<bool>();
				matrixTempCorrection = json["matrixCorrection"].as<int>();
        seg7lcdState = json["seg7lcd"].as<int>();
        buttonState = json["button"].as<int>();
			}
			configFile.close();
		}
	}
	else
	{
		if (!USBConnection)
		{
			Serial.println("mounting not possible");
		}
	}
	Serial.println("Matrix Type");

	if(!MatrixType2)
	{
		matrix = new FastLED_NeoMatrix(leds, 32, 8, NEO_MATRIX_TOP + NEO_MATRIX_LEFT + NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG);
	}
	else
	{
		matrix = new FastLED_NeoMatrix(leds, 32, 8, NEO_MATRIX_TOP + NEO_MATRIX_LEFT + NEO_MATRIX_ROWS + NEO_MATRIX_ZIGZAG);
	}
	switch (matrixTempCorrection)
	{
	case 0:
		FastLED.addLeds<NEOPIXEL, D2>(leds, 256).setCorrection(TypicalLEDStrip);
		break;
	case 1:
		FastLED.addLeds<NEOPIXEL, D2>(leds, 256).setTemperature(Candle);
		break;
	case 2:
		FastLED.addLeds<NEOPIXEL, D2>(leds, 256).setTemperature(Tungsten40W);
		break;
	case 3:
		FastLED.addLeds<NEOPIXEL, D2>(leds, 256).setTemperature(Tungsten100W);
		break;
	case 4:
		FastLED.addLeds<NEOPIXEL, D2>(leds, 256).setTemperature(Halogen);
		break;
	case 5:
		FastLED.addLeds<NEOPIXEL, D2>(leds, 256).setTemperature(CarbonArc);
		break;
	case 6:
		FastLED.addLeds<NEOPIXEL, D2>(leds, 256).setTemperature(HighNoonSun);
		break;
	case 7:
		FastLED.addLeds<NEOPIXEL, D2>(leds, 256).setTemperature(DirectSunlight);
		break;
	case 8:
		FastLED.addLeds<NEOPIXEL, D2>(leds, 256).setTemperature(OvercastSky);
		break;
	case 9:
		FastLED.addLeds<NEOPIXEL, D2>(leds, 256).setTemperature(ClearBlueSky);
		break;
	case 10:
		FastLED.addLeds<NEOPIXEL, D2>(leds, 256).setTemperature(WarmFluorescent);
		break;
	case 11:
		FastLED.addLeds<NEOPIXEL, D2>(leds, 256).setTemperature(StandardFluorescent);
		break;
	case 12:
		FastLED.addLeds<NEOPIXEL, D2>(leds, 256).setTemperature(CoolWhiteFluorescent);
		break;
	case 13:
		FastLED.addLeds<NEOPIXEL, D2>(leds, 256).setTemperature(FullSpectrumFluorescent);
		break;
	case 14:
		FastLED.addLeds<NEOPIXEL, D2>(leds, 256).setTemperature(GrowLightFluorescent);
		break;
	case 15:
		FastLED.addLeds<NEOPIXEL, D2>(leds, 256).setTemperature(BlackLightFluorescent);
		break;
	case 16:
		FastLED.addLeds<NEOPIXEL, D2>(leds, 256).setTemperature(MercuryVapor);
		break;
	case 17:
		FastLED.addLeds<NEOPIXEL, D2>(leds, 256).setTemperature(SodiumVapor);
		break;
	case 18:
		FastLED.addLeds<NEOPIXEL, D2>(leds, 256).setTemperature(MetalHalide);
		break;
	case 19:
		FastLED.addLeds<NEOPIXEL, D2>(leds, 256).setTemperature(HighPressureSodium);
		break;
	case 20:
		FastLED.addLeds<NEOPIXEL, D2>(leds, 256).setTemperature(UncorrectedTemperature);
		break;
	default:
		FastLED.addLeds<NEOPIXEL, D2>(leds, 256).setCorrection(TypicalLEDStrip);
		break;
	}
	photocell.updateResistor(ldrState);

	matrix->begin();
	matrix->setTextWrap(false);
	matrix->setBrightness(20);
	matrix->setFont(&TomThumb);

	if (drd.detect())
	{
		//Serial.println("** Double reset boot **");
		matrix->clear();
		matrix->setTextColor(matrix->Color(255, 0, 0));
		matrix->setCursor(6, 6);
		matrix->print("RESET!");
		matrix->show();
		delay(1000);
		if (SPIFFS.begin())
		{
			delay(1000);
			SPIFFS.remove(jsonconfigfile);
			Serial.println("SPIFFS is fine, now removing /.json file");
			SPIFFS.end();
			delay(1000);
		}
		else
		{
			Serial.println("SPIFFS is broken, resetting WIFI");
		}
		wifiManager.resetSettings();
		ESP.reset();
	}

	wifiManager.setAPStaticIPConfig(IPAddress(172, 217, 28, 1), IPAddress(172, 217, 28, 1), IPAddress(255, 255, 255, 0));
	WiFiManagerParameter custom_awtrix_server("server", "AWTRIX Server", awtrix_server, 16);
  WiFiManagerParameter p_MatrixType2("MatrixType2", "MatrixType 2", "T", 2, "type=\"checkbox\" ", WFM_LABEL_BEFORE);
	WiFiManagerParameter p_USBConnection("USBConnection", "Serial Connection", "T", 2, "type=\"checkbox\" ", WFM_LABEL_BEFORE);
// Just a quick hint
    WiFiManagerParameter p_hint("<small>Please configure your AWTRIX Server IP (without Port), and check MatrixType 2 if you cant read anything on the Matrix<br></small><br><br>");
    WiFiManagerParameter p_lineBreak_notext("<p></p>");

	wifiManager.setSaveConfigCallback(saveConfigCallback);
	wifiManager.setAPCallback(configModeCallback);
	wifiManager.addParameter(&p_hint);
	wifiManager.addParameter(&custom_awtrix_server);
	wifiManager.addParameter(&p_lineBreak_notext);
  wifiManager.addParameter(&p_MatrixType2);
	wifiManager.addParameter(&p_USBConnection);
	wifiManager.addParameter(&p_lineBreak_notext);
	hardwareAnimatedSearch(0, 24, 0);

	if (!wifiManager.autoConnect("AWTRIX Controller", wifipass))
	{
		Serial.println("failed to connect WiFi and hit timeout");
		delay(3000);
		//reset and try again, or maybe put it to deep sleep
		ESP.reset();
		delay(5000);
	}

	Serial.println("connected...yeey, jupp1 :)");
  Serial.println(awtrix_server);
  Serial.println("connected...yeey, jupp2 :)");

	server.on("/", HTTP_GET, []() {
		server.sendHeader("Connection", "close");
		server.send(200, "text/html", serverIndex);
	});
	server.on("/update", HTTP_POST, []() {
      server.sendHeader("Connection", "close");
      server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
      ESP.restart(); }, []() {
      HTTPUpload& upload = server.upload();

      if (upload.status == UPLOAD_FILE_START) {
        Serial.setDebugOutput(true);
        Serial.printf("Update: %s\n", upload.filename.c_str());
        uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
        if (!Update.begin(maxSketchSpace)) { //start with max available size
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_WRITE) {
		  matrix->clear();
		  flashProgress((int)upload.currentSize,(int)upload.buf);
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) { //true to set the size to the current progress
		  server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
          Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
        } else {
          Update.printError(Serial);
        }
        Serial.setDebugOutput(false);
      }
      yield(); });

	server.begin();

	if (shouldSaveConfig)
	{
		if (!USBConnection)
		{
		 Serial.println("saving config");
		}
		strcpy(awtrix_server, custom_awtrix_server.getValue());
  	MatrixType2 = (strncmp(p_MatrixType2.getValue(), "T", 1) == 0);
		USBConnection = (strncmp(p_USBConnection.getValue(), "T", 1) == 0);
		saveConfig();
		ESP.reset();
	}

	hardwareAnimatedCheck(0, 27, 2);

	//Checking periphery
	Wire.begin(I2C_SDA, I2C_SCL); //  Also needed for LCD 7 Segment display
	if (tempState == 1)
	{
		if (BMESensor.begin())
		{
			//temp OK
			hardwareAnimatedCheck(2, 29, 2);
		}
		else
		{
			//temp NOK
			hardwareAnimatedUncheck(2, 27, 1);
		}
	}
	else if (tempState == 2)
	{
		if (htu.begin())
		{
			hardwareAnimatedCheck(2, 29, 2);
		}
		else
		{
			hardwareAnimatedUncheck(2, 27, 1);
		}
	}

	if (audioState)
	{
		mySoftwareSerial.begin(9600);
		myMP3.begin(mySoftwareSerial);
		hardwareAnimatedCheck(3, 29, 2);
	}
	if (gestureState)
	{
		pinMode(APDS9960_INT, INPUT);
		attachInterrupt(APDS9960_INT, interruptRoutine, FALLING);
		if(apds.init()){
			hardwareAnimatedCheck(4, 29, 2);
		}
		else
		{
			hardwareAnimatedUncheck(4, 27, 1);
		}
		apds.enableGestureSensor(true);
	}
	if (ldrState)
	{
		photocell.setPhotocellPositionOnGround(false);
		hardwareAnimatedCheck(5, 29, 2);
	}
  if (seg7lcdState)
  {
    //Serial.begin(9600);
    //Serial.println("HT16K33 test");
    lcd7matrix.begin(0x70);  // pass in the address
    lcd7matrix.clear();
    lcd7matrix.writeDisplay();
    hardwareAnimatedCheck(6, 29, 2);
  }
  if (buttonState)
  {
    //lcd7matrix.begin(0x70);  // pass in the address
    //lcd7matrix.clear();
    //lcd7matrix.writeDisplay();
    pinMode(D8, INPUT_PULLUP);
    //
    // Configure the button-action(s).
    //
    button.attachClick(on_short_click);
    button.attachLongPressStop(on_long_click);

    hardwareAnimatedCheck(7, 29, 2);
  }

	ArduinoOTA.onStart([&]() {
		updating = true;
		matrix->clear();
	});

	ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
		flashProgress(progress, total);
	});

	ArduinoOTA.begin();
	matrix->clear();
	matrix->setCursor(7, 6);

	bufferpointer = 0;

	myTime = millis() - 500;
	myTime2 = millis() - 1000;
	myTime3 = millis() - 500;
	myCounter = 0;
	myCounter2 = 0;

	getLength = true;
	prefix = -5;

	for(int x=32; x>=-100; x--) {
    matrix->clear();
    matrix->setCursor(x, 6);
    matrix->print("Server-IP: " + String(awtrix_server));
		matrix->setTextColor(matrix->Color(0, 0, 127));
	  matrix->show();
    delay(25);
  }

	if (!USBConnection)
	{
		//client.setServer(awtrix_server, 7001);
    client.setServer(awtrix_server, mqttport);
		client.setCallback(callback);
	}
}

void loop()
{
	server.handleClient();
	ArduinoOTA.handle();

	if (firstStart)
	{
		if (!USBConnection)
		{
			if (millis() - myTime > 500)
			{
				serverSearch(myCounter, 0, 28, 0);
				myCounter++;
				if (myCounter == 4)
				{
					myCounter = 0;
				}
				myTime = millis();
			}
		}
		else
		{
			if (millis() - myTime > 100)
			{
				serverSearch(myCounter, 1, 28, 0);
				myCounter++;
				if (myCounter == 13)
				{
					myCounter = 0;
				}
				myTime = millis();
			}
		}
	}

	if (!updating)
	{
		if (USBConnection)
		{
			//third try
			if (Serial.available() > 0)
			{
				//read and fill in ringbuffer
				myBytes[bufferpointer] = Serial.read();
				messageLength--;
				for (int i = 0; i < 14; i++)
				{
					if ((bufferpointer - i) < 0)
					{
						myPointer[i] = 1000 + bufferpointer - i;
					}
					else
					{
						myPointer[i] = bufferpointer - i;
					}
				}
				//prefix from "awtrix" == 6?
				if (myBytes[myPointer[13]] == 0 && myBytes[myPointer[12]] == 0 && myBytes[myPointer[11]] == 0 && myBytes[myPointer[10]] == 6)
				{
					//"awtrix" ?
					if (myBytes[myPointer[9]] == 97 && myBytes[myPointer[8]] == 119 && myBytes[myPointer[7]] == 116 && myBytes[myPointer[6]] == 114 && myBytes[myPointer[5]] == 105 && myBytes[myPointer[4]] == 120)
					{
						messageLength = (int(myBytes[myPointer[3]]) << 24) + (int(myBytes[myPointer[2]]) << 16) + (int(myBytes[myPointer[1]]) << 8) + int(myBytes[myPointer[0]]);
						SavemMessageLength = messageLength;
						awtrixFound = true;
					}
				}

				if (awtrixFound && messageLength == 0)
				{
					byte tempData[SavemMessageLength];
					int up = 0;
					for (int i = SavemMessageLength - 1; i >= 0; i--)
					{
						if ((bufferpointer - i) >= 0)
						{
							tempData[up] = myBytes[bufferpointer - i];
						}
						else
						{
							tempData[up] = myBytes[1000 + bufferpointer - i];
						}
						up++;
					}
					updateMatrix(tempData, SavemMessageLength);
					awtrixFound = false;
				}
				bufferpointer++;
				if (bufferpointer == 1000)
				{
					bufferpointer = 0;
				}
			}
		}

		else
		{
			if (!client.connected())
			{
				reconnect();
			}
			else
			{
        //
        // Process the input-button
        //
        ////button.tick();

				client.loop();
        //
        // Handle any pending clicks here.
        //
        ////handlePendingButtons();

        // new feature to flicker the display
        //if (myflickercounter > 0)
        //{
        //  if ((millis() - myFlickerTime) > myflickercounter)
        //  {
        //    Serial.println(".");
        //    mytoogle = not(mytoogle);
        //    if (mytoogle == true) {
        //        //mybrightnessbefore = matrix->getBrightness();
        //        //matrix->setMaxPowerInVoltsAndMilliamps(uint8_t(5), uint32_t(3000));
        //        matrix->setBrightness(0);
        //    }
        //    else {
        //        matrix->setBrightness(mybrightnessbefore);
        //    }
        //    matrix->show();
        //    myFlickerTime = millis();
        //  }
        //}

			}
		}
		if (isr_flag == 1)
		{
			detachInterrupt(APDS9960_INT);
			handleGesture();
			isr_flag = 0;
			attachInterrupt(APDS9960_INT, interruptRoutine, FALLING);
		}
	}
}
