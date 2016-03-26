/* RethinkAzureSuit.ino - MKR1000 Azure IoT HTTP client with sensors sending data to Azure IoT Suit example

Tested joepvanwyk 11 Mar 2016 - adapted from MKRAzure.ino written by Mohan Palanisamy (http://mohanp.com)
Instructions are here to properly set up the MKR1000 for SSL connections http://mohanp.com/mkr1000-azure-iot-hub-how-to/

Additional code used from sample sketches:
------------------------------------------
Connectthedots - DS18B20.ino					// temperature sensor code outputs degrees Centigrade
Bildr		  - ADXL345_Example.ino			// ADXL345 accelerometer statuses INACTIVITY, ACTIVITY, FALL DETECT, TAP, DOUBLE TAP
Github		  - JsonGeneratorExample.ino	// Benoit Blanchon's Json library example simplifies JSON generation parts of mohanp's code

MKR1000 pinouts:
----------------
LDR analog			A0		2
Battery voltage		A1		3
LED red				D0		9
LED green			D1		10
DS18B20 signal		D2		11
ADXL345 INT1			?
Buzzer				D3		12
Switch				D4		13
MKR1000_LED			D6		15				// internal LED
ADXL345 SDA			D11		21
ADXL345 SCL			D12		22

GND							25
Vcc							26
Vin							27

Rules for posting data to Azure:
--------------------------------
1. Temperature changes by more than 0.5 degrees C
2. Accelerometer combination of statuses defines action
3. Light level changes by more than 100 units
4. Battery voltage falls below 3.3V
5. Switch status changes (door open or close)


Copyright (c) Arduino. All rights reserved.
Licensed under the MIT license. See LICENSE file in the project root for full license information.

*/

#include <SPI.h>
#include <WiFi101.h>
#include <Wire.h>
#include <ADXL345.h>
#include <OneWire.h> 
#include <avr/dtostrf.h>
#include <ArduinoJson.h>

// WiFi Network Config 
char ssid[] = "wlan-ap"; 		//  your network SSID (name)
char pass[] = "qwerty123";		// your network password (use for WPA, or use as key for WEP)

								// Device Explorer - Protocol Gateway Hostname
char hostname[] = "RethinkAzureSensorNode.azure-devices.net";
// IoT Hub Connection String: HostName=RethinkAzureSensorNode.azure-devices.net;SharedAccessKeyName=iothubowner;SharedAccessKey=yypCMvmAHIZDSKN2CYxVilHeliTau42EvlBd/QmSC9o= 

// Device Explorer - Generated SAS
char authSAS[] = "SharedAccessSignature sr=RethinkAzureSensorNode.azure-devices.net&sig=g%2bQHMLYInsPrvbmyeC9QRsA6kutL8dQTpz5ruR%2b%2fiY0%3d&se=1490453142&skn=iothubowner";

char feeduri[] = "/devices/RethinkAzureSuite01/messages/devicebound?api-version=2016-02-03"; // feed URI
char azurePOST_Uri[] = "/devices/RethinkAzureSuite01/messages/events?api-version=2016-02-03"; // feed POST Uri

																							  // message Complete/Reject/Abandon URIs.  "etag" will be replaced with the message id E-Tag recieved from recieve call.
String azureComplete = "/devices/RethinkAzureSuite01/messages/devicebound/etag?api-version=2016-02-03";
String azureReject = "/devices/RethinkAzureSuite01/messages/devicebound/etag?reject&api-version=2016-02-03";
String azureAbandon = "/devices/RethinkAzureSuite01/messages/devicebound/etag/abandon?&api-version=2016-02-03";

char DeviceID[] = "RethinkAzureSuite01";

char buffer[256];		// JSON data buffer


						// Sensor stuff
#define ldrPin		A0			// light sensor
#define batPin		A1			// battery voltage
#define LED_red		0			// red status led
#define LED_green	1			// green status led
#define DS18S20_Pin 2 			// DS18S20 Signal pin
#define Buzzer		3			// sound output
#define swPin		4			// switch input
#define MKR1000_LED 6			// internal LED

float prevtemp = 0;
float prevacc = 0;
float prevlight = 0;
float prevbattery = 0;
int prevswitch = 0;

unsigned long time_last = millis();
unsigned long time_new = millis();
int polltime = 5;						// minimum time between sending to Azure

int status = WL_IDLE_STATUS;

OneWire ds(DS18S20_Pin); 				// instantiate temperature object	 
ADXL345 adxl; 							// variable adxl is an instance of the ADXL345 library
WiFiSSLClient client;					// instantiate wifi object
void setup()
{

	/* serial is for debugging  */
	Serial.begin(9600);
	int i;
	while (!Serial) { if (i++ >= 512) break; }			// wait for serial to connect

														// setup I/Os
	pinMode(LED_red, OUTPUT);			// status LED red
	digitalWrite(LED_red, HIGH);		// high is off
	pinMode(LED_green, OUTPUT); 		// status LED green
	digitalWrite(LED_green, HIGH);	// high is off
	pinMode(Buzzer, OUTPUT);			// sound out
	digitalWrite(Buzzer, LOW);		// low is off
	pinMode(MKR1000_LED, OUTPUT);		// internal LED
	pinMode(swPin, INPUT);			// external switch


									// ADXL345 accelerometer setup
	adxl.powerOn();
	//set activity/ inactivity thresholds (0-255)
	adxl.setActivityThreshold(75); //62.5mg per increment
	adxl.setInactivityThreshold(75); //62.5mg per increment
	adxl.setTimeInactivity(10); // how many seconds of no activity is inactive?
								//look of activity movement on this axes - 1 == on; 0 == off 
	adxl.setActivityX(1);
	adxl.setActivityY(1);
	adxl.setActivityZ(1);
	//look of inactivity movement on this axes - 1 == on; 0 == off
	adxl.setInactivityX(1);
	adxl.setInactivityY(1);
	adxl.setInactivityZ(1);
	//look of tap movement on this axes - 1 == on; 0 == off
	adxl.setTapDetectionOnX(0);
	adxl.setTapDetectionOnY(0);
	adxl.setTapDetectionOnZ(1);
	//set values for what is a tap, and what is a double tap (0-255)
	adxl.setTapThreshold(50); //62.5mg per increment
	adxl.setTapDuration(15); //625μs per increment
	adxl.setDoubleTapLatency(80); //1.25ms per increment
	adxl.setDoubleTapWindow(200); //1.25ms per increment
								  //set values for what is considered freefall (0-255)
	adxl.setFreeFallThreshold(7); //(5 - 9) recommended - 62.5mg per increment
	adxl.setFreeFallDuration(45); //(20 - 70) recommended - 5ms per increment
								  //setting all interupts to take place on int pin 1
								  //I had issues with int pin 2, was unable to reset it
	adxl.setInterruptMapping(ADXL345_INT_SINGLE_TAP_BIT, ADXL345_INT1_PIN);
	adxl.setInterruptMapping(ADXL345_INT_DOUBLE_TAP_BIT, ADXL345_INT1_PIN);
	adxl.setInterruptMapping(ADXL345_INT_FREE_FALL_BIT, ADXL345_INT1_PIN);
	adxl.setInterruptMapping(ADXL345_INT_ACTIVITY_BIT, ADXL345_INT1_PIN);
	adxl.setInterruptMapping(ADXL345_INT_INACTIVITY_BIT, ADXL345_INT1_PIN);
	//register interupt actions - 1 == on; 0 == off  
	adxl.setInterrupt(ADXL345_INT_SINGLE_TAP_BIT, 1);
	adxl.setInterrupt(ADXL345_INT_DOUBLE_TAP_BIT, 1);
	adxl.setInterrupt(ADXL345_INT_FREE_FALL_BIT, 1);
	adxl.setInterrupt(ADXL345_INT_ACTIVITY_BIT, 1);
	adxl.setInterrupt(ADXL345_INT_INACTIVITY_BIT, 1);

	//check for the presence of the shield:
	if (WiFi.status() == WL_NO_SHIELD) {
		// don't continue:
		while (true);
	}

	// attempt to connect to Wifi network:
	while (status != WL_CONNECTED) {
		status = WiFi.begin(ssid, pass);
		// wait 10 seconds for connection:
		delay(10000);
	}

}

void loop()
	{
		// Check for messages from Azure IoT Hub and process them
		azureHttpMessage();

		// Read accelerometer and send to Azure
		int acc = getAccelerometer();
		if (acc > 0) {					    // acceleration action sensed
			if (wifiConnect()) {
				prevacc = acc;
				LED_blink(LED_red, 50, 1);
				httpRequest("POST", azurePOST_Uri, "", buffer); 			// buffer contains JSON data
				delay(polltime);
			}
		}

		// Read temperature and send to Azure
		float temp = getTemp();
		float temp1 = (temp - prevtemp);
		if (abs(temp1) > 1) {				// temperature changed by more than 1 degrees C
			if (wifiConnect()) {
				prevtemp = temp;
				LED_blink(LED_red, 50, 2);
				httpRequest("POST", azurePOST_Uri, "", buffer); 			// buffer contains JSON data
				delay(polltime);				// minimum poll time in seconds
			}
		}

		// Read lightlevel and send to Azure
		float light = getLight();
		float light1 = (light - prevlight);
		if (abs(light1) > 100) {				// light level changed more than 100 units
			if (wifiConnect()) {
				prevlight = light;
				LED_blink(LED_red, 50, 3);
				httpRequest("POST", azurePOST_Uri, "", buffer); 			// buffer contains JSON data
				delay(polltime);				// minimum poll time in seconds
			}
		}

		// Read battery voltage and send to Azure
		float battery = getBatteryV();
		float battery1 = (battery - prevbattery);
		if (abs(battery1) > 100) {			// battery voltage changed more than 0.5V
			if (wifiConnect()) {
				prevbattery = battery;
				LED_blink(LED_red, 50, 4);
				httpRequest("POST", azurePOST_Uri, "", buffer); 			// buffer contains JSON data
				delay(polltime);				// minimum poll time in seconds
			}
		}

		int switchstatus = getSwitch();
		if (!(switchstatus == prevswitch)) {
			if (wifiConnect()) {
				prevswitch = switchstatus;
				LED_blink(LED_red, 50, 5);
				httpRequest("POST", azurePOST_Uri, "", buffer); 			// buffer contains JSON data
				delay(polltime);				// minimum poll time in seconds
			}
		}

		time_new = millis();
		if ((time_new - time_last) > 20000) {
			LED_blink(LED_green, 50, 2);
			time_last = millis();
			azureIoTReceiveMessage();			// send Azure request for message
		}
	}


/************ Read temperature ************/

float getTemp() {
	//returns the temperature from one DS18S20 in Centigrade
	byte data1[12];
	byte addr[8];

	if (!ds.search(addr)) {
		//no more sensors on chain, reset search
		ds.reset_search();
		return -1000;
	}

	if (OneWire::crc8(addr, 7) != addr[7]) {
		return -1000;
	}

	if (addr[0] == 0x10) {
		//Sensor is a DS18S20
	}
	else if (addr[0] == 0x28) {
		//Sensor is a DS18BS20
	}
	else
	{
		return -1000;
	}

	ds.reset();
	ds.select(addr);
	ds.write(0x44, 1); // start conversion, with parasite power on at the end

	byte present = ds.reset();
	ds.select(addr);
	ds.write(0xBE); // Read Scratchpad


	for (int i = 0; i < 9; i++) { // we need 9 bytes
		data1[i] = ds.read();
	}

	ds.reset_search();

	byte MSB = data1[1];
	byte LSB = data1[0];

	float tempRead = ((MSB << 8) | LSB); //using two's compliment
	float TemperatureSum = tempRead / 16;
	// JSON buffer created
	// Create the root of the object tree.
	StaticJsonBuffer<256> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	root["DeviceID"] = DeviceID;
	root["Temperature"] = double_with_n_digits(TemperatureSum, 2);
	// Print to buffer
	root.printTo(buffer, sizeof(buffer));

	return TemperatureSum;

}

/************ Read accelerometer ************/

int getAccelerometer() {
	char action[50];                        // strcat needs a character array
	memset(action, '\0', sizeof(action));
	int interrupt = 0;

	int x, y, z;
	adxl.readAccel(&x, &y, &z); //read the accelerometer values and store them in variables  x,y,z
								//read interrupts source and look for triggerd actions
	byte interrupts = adxl.getInterruptSource();

	// freefall
	if (adxl.triggered(interrupts, ADXL345_FREE_FALL)) {
		strcat(action, "freefall ");
		interrupt = 1;
		//add code here to do when freefall is sensed
	}

	//inactivity
	if (adxl.triggered(interrupts, ADXL345_INACTIVITY)) {
		strcat(action, "inactivity ");
		interrupt = 0;
		//add code here to do when inactivity is sensed
	}

	//activity
	if (adxl.triggered(interrupts, ADXL345_ACTIVITY)) {
		strcat(action, "activity ");
		interrupt = 1;
		//add code here to do when activity is sensed
	}

	//double tap
	if (adxl.triggered(interrupts, ADXL345_DOUBLE_TAP)) {
		strcat(action, "double tap ");
		interrupt = 1;
		//add code here to do when a 2X tap is sensed
	}

	//tap
	if (adxl.triggered(interrupts, ADXL345_SINGLE_TAP)) {
		strcat(action, "tap ");
		interrupt = 1;
		//add code here to do when a tap is sensed
	}

	// JSON buffer created
	// Create the root of the object tree.
	StaticJsonBuffer<256> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	root["DeviceID"] = DeviceID;
	root["Accelerometer"] = action;
	// Print to buffer
	root.printTo(buffer, sizeof(buffer));

	return interrupt;
}

/************ Read lightlevel ************/

float getLight() {
	float lightlevel = analogRead(ldrPin);

	// Create the root of the object tree.
	StaticJsonBuffer<256> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	// JSON buffer created
	root["DeviceID"] = DeviceID;
	root["Lightlevel"] = double_with_n_digits(lightlevel, 2);
	// Print to buffer
	root.printTo(buffer, sizeof(buffer));

	return lightlevel;
}

/************ Read battery voltage ************/

float getBatteryV() {
	float batteryV = analogRead(batPin);

	// Create the root of the object tree.
	StaticJsonBuffer<256> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	// JSON buffer created
	root["DeviceID"] = DeviceID;
	root["Batvolts"] = double_with_n_digits(batteryV, 2);
	// Print to buffer
	root.printTo(buffer, sizeof(buffer));

	return batteryV;
}

/************ Read switch ************/

int getSwitch() {
	int swStatus = digitalRead(swPin);
	char truefalse[5];                        // strcat needs a character array
	memset(truefalse, '\0', sizeof(truefalse));

	if (swStatus == 0) {
		strcat(truefalse, "OFF");
	}
	else
	{
		strcat(truefalse, "ON");
	}

	// Create the root of the object tree.
	StaticJsonBuffer<256> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	// JSON buffer created
	root["DeviceID"] = DeviceID;
	root["Switch"] = truefalse;
	// Print to buffer
	root.printTo(buffer, sizeof(buffer));

	return swStatus;
}

/********************* Blink ***********************/

void LED_blink(int portnum, int timedelay, int flashnum) {
	for (int i = 0; i < flashnum; i++) {
		digitalWrite(portnum, LOW);
		delay(timedelay);
		digitalWrite(portnum, HIGH);
		delay(timedelay);
	}

}

//******************* Open WiFi connection ****************/

int wifiConnect() {
	// attempt to connect to Wifi network:
	while (status != WL_CONNECTED) {
		status = WiFi.begin(ssid, pass);
		// wait 1 seconds for connection:
		delay(1000);
	}
	return 1;
}

/**************************** Azure related methods **************************/


/************* this method checks for messages from the Aure IoT Hub and processes them ***************/

void azureHttpMessage() {
	String response = "";
	char c;
	///read response if WiFi Client is available
	while (client.available()) {
		c = client.read();
		response.concat(c);
	}

	if (!response.equals(""))
	{
		//if there are no messages in the IoT Hub Device queue, Azure will return 204 status code. 
		if (response.startsWith("HTTP/1.1 204"))
		{
			//turn off onboard LED
			digitalWrite(MKR1000_LED, LOW);
		}
		else
		{
			//turn on onboard LED
			digitalWrite(MKR1000_LED, HIGH);

			// get the ETag from the received message response 
			String eTag = getHeaderValue(response, "ETag");

			// get the payload from the message response
			String command = getResponsePayload(response);

			azureIoTCompleteMessage(eTag);

			if (command == "beepon") {
				//turn on buzzer
				digitalWrite(Buzzer, HIGH);
			}

			if (command == "beepoff") {
				//turn off buzzer
				digitalWrite(Buzzer, LOW);
			}
		}
	}
}

/************* Receive Azure IoT Hub "cloud-to-device" message ***************/

void azureIoTReceiveMessage()
{
	httpRequest("GET", feeduri, "", "");
}

/************* Tells Azure IoT Hub that the message with the msgLockId is handled and it can be removed from the queue ***************/

void azureIoTCompleteMessage(String eTag)
{
	String uri = azureComplete;
	uri.replace("etag", trimETag(eTag));

	httpRequest("DELETE", uri, "", "");
}

/************* Azure POST, GET, DELETE requests ***************/

void httpRequest(String verb, String uri, String contentType, String content)
{
	if (verb.equals("")) return;
	if (uri.equals("")) return;

	// close any connection before send a new request.
	// This will free the socket on the WiFi shield
	// client.stop();

	// if there's a successful connection:
	if (client.connect(hostname, 443)) {
		client.print(verb); //send POST, GET or DELETE
		client.print(" ");
		client.print(uri);  // any of the URI
		client.println(" HTTP/1.1");
		client.print("Host: ");
		client.println(hostname);  					//with hostname header
		client.print("Authorization: ");
		client.println(authSAS);  					//Authorization SAS token obtained from Azure IoT device explorer
		client.println("Connection: close");

		if (verb.equals("POST"))
		{
			client.print("Content-Type: ");
			client.println(contentType);
			client.print("Content-Length: ");
			client.println(content.length());
			client.println();
			client.println(content);

		}
		else
		{
			client.println();
		}
	}
}

/************* To get only the message header from http response ***************/

String getHeaderValue(String response, String headerName)
{
	String headerSection = getHeaderSection(response);
	String headerValue = "";

	int idx = headerSection.indexOf(headerName);

	if (idx >= 0)
	{
		int skip = 0;
		if (headerName.endsWith(":"))
			skip = headerName.length() + 1;
		else
			skip = headerName.length() + 2;

		int idxStart = idx + skip;
		int idxEnd = headerSection.indexOf("\r\n", idxStart);
		headerValue = response.substring(idxStart, idxEnd);
	}

	return headerValue;
}

/********** Azure IoT sets ETag string enclosed in double quotes, not in sync with its other endpoints - need to remove the double quotes ************/

String trimETag(String value)
{
	String retVal = value;

	if (value.startsWith("\""))
		retVal = value.substring(1);

	if (value.endsWith("\""))
		retVal = retVal.substring(0, retVal.length() - 1);

	return retVal;
}

/************* To get all the headers from the HTTP Response ***************/

String getHeaderSection(String response)
{
	int idxHdrEnd = response.indexOf("\r\n\r\n");

	return response.substring(0, idxHdrEnd);
}

/************* To get only the message payload from http response ***************/

String getResponsePayload(String response)
{
	int idxHdrEnd = response.indexOf("\r\n\r\n");

	return response.substring(idxHdrEnd + 4);
}