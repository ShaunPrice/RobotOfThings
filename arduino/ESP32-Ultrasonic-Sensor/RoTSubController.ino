/*
Name:	RoTController.ino
		Created: 10/02/2019 3:17 PM
		Author: Shaun Price
*/
#include <ArduinoJson.h>
#include <WiFi.h>

//#define DEBUG

#define FRONT_TRIGGER 5
#define FRONT_ECHO 18
#define REAR_TRIGGER 23
#define REAR_ECHO 19
#define LED_BUILTIN 22
#define FPM_SLEEP_MAX_TIME 0xFFFFFFF

volatile float frontDistance = 0;
volatile float rearDistance = 0;

volatile uint32_t frontDistanceArray[5] = {0,0,0,0,0};
volatile int frontStartPtr = 0;

volatile uint32_t rearDistanceArray[5] = { 0,0,0,0,0 };
volatile int rearStartPtr = 0;

unsigned long startTime = 0;

// Inside the brackets, 200 is the capacity of the memory pool in bytes.
// Don't forget to change this value to match your JSON document.
// Use arduinojson.org/v6/assistant to compute the capacity.
StaticJsonDocument<200> control;

void TaskUltrasonic(void *pvParameters);
void TaskComms(void *pvParameters);

TaskHandle_t ultrasonicTask;
TaskHandle_t commsTask;

SemaphoreHandle_t frontUltrasonicSemaphore;
SemaphoreHandle_t rearUltrasonicSemaphore;

/// Moving Average
/// array = The array to process (mus be 5 elements)
/// startPtr = the start index of the first element
/// returns the moving averrage of the array
float movingAverage(volatile uint32_t array[], int startPtr)
{
	int length = 5;

	String data = "";
	float averageEchoTime = 0;

	for (int x = 0; x < length; x++)
	{
		int ptr = startPtr + x;
		if (ptr >= length) ptr -= length; // Loop ptr around

		averageEchoTime += (float)array[ptr] * (0.3 - ptr * 0.05);
	}
	return averageEchoTime;
}

void setup()
{
	pinMode(LED_BUILTIN, OUTPUT);

	pinMode(FRONT_TRIGGER, OUTPUT);
	pinMode(FRONT_ECHO, INPUT);
	pinMode(REAR_TRIGGER, OUTPUT);
	pinMode(REAR_ECHO, INPUT);
	
	digitalWrite(LED_BUILTIN, LOW);

	// Put WiFi to sleep
	WiFi.mode(WIFI_OFF);

#ifdef DEBUG
	Serial.begin(115200);
#endif
	Serial2.begin(115200);
	
	// Semaphore cannot be used before a call to xSemaphoreCreateMutex().
   // This is a macro so pass the variable in directly.
	frontUltrasonicSemaphore = xSemaphoreCreateMutex();
	rearUltrasonicSemaphore = xSemaphoreCreateMutex();

	xTaskCreatePinnedToCore(TaskUltrasonic, "TaskUltrasonic", 1024, NULL, 2, &ultrasonicTask, 0);
	xTaskCreatePinnedToCore(TaskComms, "TaskComms", 1024, NULL, 3, &commsTask, 1);
}

void loop()
{
	// Everything is run in tasks
}

void TaskComms(void *pvParameters)
{
	(void)pvParameters;

	for (;;)
	{
		// Wait here until a request for data
		if (Serial2.available())
		{
			String request = Serial2.readStringUntil(10);

			// Deserialize the JSON document
			DeserializationError error = deserializeJson(control, request);
			
			if (!error)
			{	
				// We have a request for data
				if (control["control"] == "xxx")
				{
				}
			}
		}

		// Send data
		digitalWrite(LED_BUILTIN, HIGH);

		// Read Front Sensor
		// Send the requested data
		String doc;
		doc = "{\"front_ultrasonic\":\"";

		xSemaphoreTake(frontUltrasonicSemaphore, 5);
		doc += String(frontDistance, 2);
		xSemaphoreGive(frontUltrasonicSemaphore);

		doc += "\",\"rear_ultrasonic\":\"";

		xSemaphoreTake(rearUltrasonicSemaphore, 5);
		doc += String(rearDistance, 2);
		xSemaphoreGive(rearUltrasonicSemaphore);

		doc += "\"}";
		Serial2.println(doc);
#ifdef DEBUG
		Serial.println(doc);
#endif
		digitalWrite(LED_BUILTIN, LOW);

		vTaskDelay(50);
	}
}

void TaskUltrasonic(void *pvParameters)
{
	(void)pvParameters;
	for (;;)
	{
		// Read Front Sensor
		//########################################
		digitalWrite(FRONT_TRIGGER, HIGH);
		vTaskDelay(10);
		digitalWrite(FRONT_TRIGGER, LOW);

		startTime = micros();

		while (digitalRead(FRONT_ECHO) != HIGH && (micros() - startTime) < 10000)
		{
			taskYIELD();
		}

		// Get the start time
		startTime = micros();

		// Wait for the timing to go low
		while (digitalRead(FRONT_ECHO) != LOW && (micros() - startTime) < 26000)
		{
			taskYIELD();
		}

		xSemaphoreTake(frontUltrasonicSemaphore, 5);
		if (micros() - startTime < 26000)
		{
			// Calculate the distance
			frontDistanceArray[frontStartPtr] = (float)(micros() - startTime);
			frontDistance = movingAverage(frontDistanceArray, frontStartPtr) * 343.0 / 1000000.0 / 2.0;

			// increment the pointer
			frontStartPtr = (frontStartPtr == 4) ? 0 : ++frontStartPtr;
		}
		else
		{
			frontDistance = 5; // Out of range between 0.02 and 4.5 meters
		}
		xSemaphoreGive(frontUltrasonicSemaphore);

		// Read Rear sensor
		//########################################
		digitalWrite(REAR_TRIGGER, HIGH);
		vTaskDelay(10);
		digitalWrite(REAR_TRIGGER, LOW);

		startTime = micros();

		while (digitalRead(REAR_ECHO) != HIGH && (micros() - startTime) < 10000)
		{
			taskYIELD();
		}

		// Get the start time
		startTime = micros();

		// Wait for the timing to go low
		while (digitalRead(REAR_ECHO) != LOW && (micros() - startTime) < 26000)
		{
			taskYIELD();
		}

		xSemaphoreTake(rearUltrasonicSemaphore, 5);
		if (micros() - startTime < 26000)
		{
			// Calculate the distance
			rearDistanceArray[rearStartPtr] = (float)(micros() - startTime);
			rearDistance = movingAverage(rearDistanceArray, rearStartPtr) * 343.0 / 1000000.0 / 2.0;

			// increment the pointer
			rearStartPtr = (rearStartPtr == 4) ? 0 : ++rearStartPtr;
		}
		else
		{
			rearDistance = 5; // Out of range between 0.02 and 4.5 meters
		}
		xSemaphoreGive(rearUltrasonicSemaphore);
	}
	vTaskDelay(1);
}