/*
 * Based on AnalogRead_DigitalRead example from: https://github.com/feilipu/Arduino_FreeRTOS_Library
 * Modified by: Frederic Pillon <frederic.pillon (at) st.com>
 */
#include <STM32FreeRTOS.h>
#include <Arduino.h>
#include <Wire.h>
#include<SparkFunMPL3115A2.h>
#include <SparkFun_Si7021_Breakout_Library.h>

// If no default pin for user button (USER_BTN) is defined, define it
#ifndef USER_BTN
#define USER_BTN 2
#endif
//sensores asco//
const byte WSPEED = D3;
const byte RAIN = D2;
const byte WDIR = A0;


// Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
// It will be used to ensure only only one Task is accessing this resource at any time.
SemaphoreHandle_t xSerialSemaphore;
SemaphoreHandle_t xEncoderSemaphore;
MPL3115A2 myPressure;
Weather Humedad;


//intento sensores viento y lluvia
//******************************************************
byte minutes;
volatile float rainHour[60];
float rainin = 0;
float windspeedmph = 0;
volatile unsigned long raintime, rainlast, raininterval, rain;
long lastWindCheck = 0;
volatile long lastWindIRQ = 0;
volatile byte windClicks = 0;
volatile float dailyrainin = 0;

//**************************************************************************
// define two Tasks for DigitalRead & AnalogRead
void TaskDigitalRead( void *pvParameters );
void TaskAnalogRead( void *pvParameters );
void Vientosp( void *pvParameters );
void altitud( void *pvParameters );
void confhum( void *pvParameters );
void Hum( void *pvParameters );
void rainIRQ();
void wspeedIRQ();
void speedwind( void *pvParameters );
//************************************************************************
//Funciones de fabricante
void rainIRQ()
// Count rain gauge bucket tips as they occur
// Activated by the magnet and reed switch in the rain gauge, attached to input D2
{
  raintime = millis(); // grab current time
  raininterval = raintime - rainlast; // calculate interval between this and last event

  if (raininterval > 10) // ignore switch-bounce glitches less than 10mS after initial edge
  {
    dailyrainin += 0.011; //Each dump is 0.011" of water
    rainHour[minutes] += 0.011; //Increase this minute's amount of rain

    rainlast = raintime; // set up for next event
  }
}

void wspeedIRQ(){
// Activated by the magnet in the anemometer (2 ticks per rotation), attached to input D3

      lastWindIRQ = millis(); 
      windClicks++;



}
void get_wind_direction(void* arg)
{
  unsigned int adc;

  adc = analogRead(WDIR); // get the current reading from the sensor

  // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
  // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
  // Note that these are not in compass degree order! See Weather Meters datasheet for more information.
/*
  if (adc < 380) return (113);
  if (adc < 393) return (68);
  if (adc < 414) return (90);
  if (adc < 456) return (158);
  if (adc < 508) return (135);
  if (adc < 551) return (203);
  if (adc < 615) return (180);
  if (adc < 680) return (23);
  if (adc < 746) return (45);
  if (adc < 801) return (248);
  if (adc < 833) return (225);
  if (adc < 878) return (338);
  if (adc < 913) return (0);
  if (adc < 940) return (293);
  if (adc < 967) return (315);
  if (adc < 990) return (270);
  //return (-1); // error, disconnected?*/
}
//******************************************************************************
// the setup function runs once when you press reset or power the board
void setup() {
//*****************************
//configuracion sensores asco
  pinMode(WSPEED, INPUT_PULLUP); // input from wind meters windspeed sensor
  pinMode(RAIN, INPUT_PULLUP);
    // attach external interrupt pins to IRQ functions
  attachInterrupt(D0, rainIRQ, FALLING);
  attachInterrupt(D3, wspeedIRQ, FALLING);
 interrupts();
 
//********************************
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
    
    Wire.begin();
    // Join i2c bus
   // Start serial for output
  


  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }

  // Semaphores are useful to stop a Task proceeding, where it should be paused to wait,
  // because it is sharing a resource, such as the Serial port.
  // Semaphores should only be used whilst the scheduler is running, but we can set it up here.
  if ( xSerialSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerialSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xSerialSemaphore ) != NULL )
      xSemaphoreGive( ( xSerialSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }
  if ( xEncoderSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xEncoderSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xEncoderSemaphore ) != NULL )
      xSemaphoreGive( ( xEncoderSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }
  // Now set up two Tasks to run independently.


  /*xTaskCreate(
    TaskAnalogRead
    ,  (const portCHAR *) "AnalogRead"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );*/

 

  xTaskCreate(
    altitud
    ,  (const portCHAR *) "sensor"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );

  /*xTaskCreate(
    confhum
    ,  (const portCHAR *) "adios"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );*/

 xTaskCreate(
    Hum
    ,  (const portCHAR *) "sensor de humedad"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );

 xTaskCreate(
    speedwind
    ,  (const portCHAR *) "velocidad del viento"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );

  // start scheduler
  vTaskStartScheduler();
  Serial.println("Insufficient RAM");
  while(1);
}

void loop()
{
  
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/
//**********************************************************************************************

//**********************************************************************************************
void TaskDigitalRead( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  /*
    DigitalReadSerial
    Reads a digital input on pin defined with USER_BTN, prints the result to the serial monitor
    This example code is in the public domain.
  */

  // defined USER_BTN digital pin  has a pushbutton attached to it. Give it a name:
  uint8_t pushButton = D2;

  // make the pushbutton's pin an input:
  pinMode(pushButton, INPUT);

  for (;;) // A Task shall never return or exit.
  {
    // read the input pin:
    int buttonState = digitalRead(pushButton);

    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      // We were able to obtain or "Take" the semaphore and can now access the shared resource.
      // We want to have the Serial Port for us alone, as it takes some time to print,
      // so we don't want it getting stolen during the middle of a conversion.
      // print out the state of the button:
      Serial.print("Button state: ");
      Serial.print(buttonState);

      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }

    vTaskDelay(1000);  // one tick delay (15ms) in between reads for stability
  }
}

void TaskAnalogRead( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

  for (;;)
  {
    // read the input on analog pin 0:
    int sensorValue = analogRead(A0);

    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      // We were able to obtain or "Take" the semaphore and can now access the shared resource.
      // We want to have the Serial Port for us alone, as it takes some time to print,
      // so we don't want it getting stolen during the middle of a conversion.
      // print out the value you read:
 

      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }

    vTaskDelay(1000);  // one tick delay (15ms) in between reads for stability
  }
}
/****************************************************/

/***************************************************/
/*///////////////////////////////////////////////////////////////////////////////////////*/

void altitud( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

  for (;;)
  {
   
    myPressure.begin();
    myPressure.setModeAltimeter();
    myPressure.setOversampleRate(128);
    myPressure.enableEventFlags();
    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      // We were able to obtain or "Take" the semaphore and can now access the shared resource.
      // We want to have the Serial Port for us alone, as it takes some time to print,
      // so we don't want it getting stolen during the middle of a conversion.
      // print out the value you read:
      float pressure = myPressure.readPressure();
      Serial.print("Pressure(Pa):");
      Serial.print(pressure, 2);

      float altitude = myPressure.readAltitude();
      Serial.print(",Altitude(m):");
      Serial.print(altitude, 2);

      float temperature = myPressure.readTemp();
      Serial.print(",Temp(c):");
      Serial.print(temperature, 2);
      
      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }

    vTaskDelay(1000);  // one tick delay (15ms) in between reads for stability
  }
}
/*//////////////////////////////////////////////////////////////////////////////////////*/
/*-------------------------------------------------------------------------------------*/
void confhum( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

  for (;;)
  {
    

    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      // We were able to obtain or "Take" the semaphore and can now access the shared resource.
      // We want to have the Serial Port for us alone, as it takes some time to print,
      // so we don't want it getting stolen during the middle of a conversion.
      // print out the value you read:

  

      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }

    vTaskDelay(1000);  // one tick delay (15ms) in between reads for stability
  }
}

/*------------------------------------------------------------------------------------*/
/*777777777777777777777777777777777777777777777777777777777777777777777777777777777777*/
void Hum( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

  for (;;)
  {
    

    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      // We were able to obtain or "Take" the semaphore and can now access the shared resource.
      // We want to have the Serial Port for us alone, as it takes some time to print,
      // so we don't want it getting stolen during the middle of a conversion.
      // print out the value you read:
    
     float humidity = Humedad.getRH();
    Serial.print(",Humidity:");
    Serial.print(humidity);
    Serial.println("%");

      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }

    vTaskDelay(1000);  // one tick delay (15ms) in between reads for stability
  }
}

/*777777777777777777777777777777777777777777777777777777777777777777777777777777777777*/
//989898989898989898989898989898989898989898

void speedwind( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

  for (;;)
  {
    

    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      // We were able to obtain or "Take" the semaphore and can now access the shared resource.
      // We want to have the Serial Port for us alone, as it takes some time to print,
      // so we don't want it getting stolen during the middle of a conversion.
      // print out the value you read:
    
  float deltaTime = millis() - lastWindCheck; //750ms

  deltaTime /= 1000.0; //Covert to seconds

  float windSpeed = (float)windClicks / deltaTime; //3 / 0.750s = 4

  windClicks = 0; //Reset and start watching for new wind
  lastWindCheck = millis();

  windSpeed *= 1.492; //4 * 1.492 = 5.968MPH

 
    Serial.print(",Windspeed:");
    Serial.print(windSpeed);

      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }

    vTaskDelay(1000);  // one tick delay (15ms) in between reads for stability
  }
}
//9898989898989898989898898989898989898989898