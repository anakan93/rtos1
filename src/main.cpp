
#include <STM32FreeRTOS.h>
#include <Arduino.h>
#include <Wire.h>
#include<SparkFunMPL3115A2.h>
#include <SparkFun_Si7021_Breakout_Library.h>


//sensores asco//
const byte WSPEED = D3;
const byte RAIN = D2;
const byte WDIR = A0;


// Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
// It will be used to ensure only only one Task is accessing this resource at any time.
SemaphoreHandle_t xSerialSemaphore;
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
volatile float windir=0;
float pressure = 0;
float altitude =0;
float temperature =0;
float humidity =0;
float windSpeed=0;
//**************************************************************************
// define two Tasks for DigitalRead & AnalogRead
void TaskDigitalRead( void *pvParameters );
void TaskAnalogRead( void *pvParameters );
void Vientosp( void *pvParameters );
void altitud( void *pvParameters );
void RainToday( void *pvParameters );
void Hum( void *pvParameters );
void rainIRQ();
void wspeedIRQ();
void speedwind( void *pvParameters );
void Impresion( void *pvParameters );
//************************************************************************
//Funciones de fabricante
void rainIRQ()
// Count rain gauge bucket tips as they occur
// Activated by the magnet and reed switch in the rain gauge, attached to input D2
{
  raintime = millis(); // grab current time
  raininterval = raintime - rainlast; // calculate interval between this and last event

  
    dailyrainin += 0.011; //Each dump is 0.011" of water
    rainHour[minutes] += 0.011; //Increase this minute's amount of rain

    rainlast = raintime; // set up for next event
  
}

void wspeedIRQ(){
// Activated by the magnet in the anemometer (2 ticks per rotation), attached to input D3

      lastWindIRQ = millis(); 
      windClicks++;



}

//******************************************************************************
// the setup function runs once when you press reset or power the board
void setup() {
//*****************************
//configuracion sensores ANALOGOS DIGITALES
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
  
  // Now set up two Tasks to run independently.


  xTaskCreate(
    TaskAnalogRead
    ,  (const portCHAR *) "direccion del viento"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );

 

  xTaskCreate(
    altitud
    ,  (const portCHAR *) "sensor de altitud,presion y temperatura"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );

  xTaskCreate(
    RainToday
    ,  (const portCHAR *) "sensor de lluvia"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );

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
 xTaskCreate(
    Impresion
    ,  (const portCHAR *) "Tarea de impresion de datos"
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
//*****************************************************

void TaskAnalogRead( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

  for (;;)
  {
    // read the input on analog pin 0:
    int adc = analogRead(A0);

    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      // We were able to obtain or "Take" the semaphore and can now access the shared resource.
      // We want to have the Serial Port for us alone, as it takes some time to print,
      // so we don't want it getting stolen during the middle of a conversion.
      // print out the value you read:
      
    if (adc < 380) {windir = (113);}
 else if (adc < 393) {windir = (68);}
 else if (adc < 414) {windir = (90);}
 else if (adc < 456) {windir = (158);}
 else if (adc < 508) {windir = (135);}
 else if (adc < 551) {windir = (203);}
 else if (adc < 615) {windir = (180);}
 else if (adc < 680) {windir = (23);}
 else if (adc < 746) {windir = (45);}
 else if (adc < 801) {windir = (248);}
 else if (adc < 833) {windir = (225);}
 else if (adc < 878) {windir = (338);}
 else if (adc < 913) {windir = (0);}
 else if (adc < 940) {windir = (293);}
 else if (adc < 967) {windir = (315);}
 else if (adc < 990) {windir = (270);}

  //Serial.print( " ,Winddir:");
  //Serial.print(windir);
      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }

    vTaskDelay(1000);  // one tick delay (15ms) in between reads for stability
  }
}
/****************************************************/

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
       pressure = myPressure.readPressure();
      //Serial.print("Pressure(Pa):");
      //Serial.print(pressure, 2);

      altitude = myPressure.readAltitude();
      //Serial.print(",Altitude(m):");
      //Serial.print(altitude, 2);

      temperature = myPressure.readTemp();
      //Serial.print(",Temp(c):");
      //Serial.print(temperature, 2);
      
      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }

    vTaskDelay(1000);  // one tick delay (15ms) in between reads for stability
  }
}
/*//////////////////////////////////////////////////////////////////////////////////////*/
/*-------------------------------------------------------------------------------------*/
void  RainToday( void *pvParameters __attribute__((unused)) )  // This is a Task.
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
      rainin = 0;
      for (int i = 0 ; i < 60 ; i++)
      rainin += rainHour[i];
  

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
    
     humidity = Humedad.getRH();
   // Serial.print(",Humidity:");
   // Serial.print(humidity);
    //Serial.println("%");

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

     windSpeed = (float)windClicks / deltaTime; //3 / 0.750s = 4

     windClicks = 0; //Reset and start watching for new wind
     lastWindCheck = millis();

     windSpeed *= 1.492; //4 * 1.492 = 5.968MPH

 
     // Serial.print(",Windspeed:");
     // Serial.print(windSpeed);

      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }

    vTaskDelay(1000);  // one tick delay (15ms) in between reads for stability
  }
}
//9898989898989898989898898989898989898989898
//**********************************************************************************************
void Impresion( void *pvParameters __attribute__((unused)) )  // This is a Task.
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
      Serial.print("@");
      Serial.print("Windspeed:");
      Serial.print(windSpeed);
      Serial.print(":WindDir: ");
      Serial.print(windir); 
      Serial.print(":Pressure: ");
      Serial.print(pressure);  
      Serial.print(":Altitude: ");
      Serial.print(altitude);
      Serial.print(":Temperature: ");
      Serial.print(temperature);
      Serial.print(":Humedad R: ");
      Serial.print(humidity);
      Serial.print(":Rainin: ");
      Serial.print(rainin);
      Serial.print(":DailyRainIn: ");
      Serial.print(dailyrainin);
      Serial.println(":#");                           


      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }

    vTaskDelay(1000);  // one tick delay (15ms) in between reads for stability
  }
}

//**********************************************************************************************
