
#include <STM32FreeRTOS.h>
#include <Arduino.h>
#include <Wire.h>
#include<SparkFunMPL3115A2.h>
#include <SparkFun_Si7021_Breakout_Library.h>


//Configuracion de sensores analogos//
const byte WSPEED = D3;
const byte RAIN = D2;
const byte WDIR = A0;


// declaracion de librerias.
SemaphoreHandle_t xSerialSemaphore;
MPL3115A2 myPressure;
Weather Humedad;


//variables de funcionamiento y claculo de cantidades
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
float temph=0;
//**************************************************************************
//Prototipo de tareas 
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
//Funciones de calculo
void rainIRQ()
{
  raintime = millis(); //variable para calcular el tiempo
  raininterval = raintime - rainlast; // calculo del intervalo de lluvia

  
    dailyrainin += 0.011; //Each dump is 0.011" of water
    rainHour[minutes] += 0.011; //Increase this minute's amount of rain

    rainlast = raintime; // set up for next event
  
}

void wspeedIRQ(){


      lastWindIRQ = millis(); 
      windClicks++;



}

//******************************************************************************

void setup() {
//*****************************
//configuracion sensores ANALOGOS DIGITALES
  pinMode(WSPEED, INPUT_PULLUP); // input from wind meters windspeed sensor
  pinMode(RAIN, INPUT_PULLUP);
    // funciones de interrupcion para sensor de velocidad del viento y lluvia
  attachInterrupt(D0, rainIRQ, FALLING);
  attachInterrupt(D3, wspeedIRQ, FALLING);
 interrupts();
 
//********************************
  //inicio de comunicacion seria a 9600:
  Serial.begin(9600);
    
    Wire.begin();
 
  


  while (!Serial) {
    ;
  }

  //
  if ( xSerialSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerialSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xSerialSemaphore ) != NULL )
      xSemaphoreGive( ( xSerialSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }
  
//configuracion de tareas


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

    
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      
      
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

    vTaskDelay(10000/portTICK_PERIOD_MS);  
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
    
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      
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

    vTaskDelay(10000/portTICK_PERIOD_MS);  
  }
}
/*//////////////////////////////////////////////////////////////////////////////////////*/


/*777777777777777777777777777777777777777777777777777777777777777777777777777777777777*/


void speedwind( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

  for (;;)
  {
    

    
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      
    
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

    vTaskDelay(10000/portTICK_PERIOD_MS);  
  }
}

//979797979797979797979797979797979797797979797979797979797977979797979
/*-------------------------------------------------------------------------------------*/
void  RainToday( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

  for (;;)
  {
    
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      
      rainin = 0;
      for (int i = 0 ; i < 60 ; i++)
      rainin += rainHour[i];
  

      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }

    vTaskDelay(10000/portTICK_PERIOD_MS);  
  }
}

/*------------------------------------------------------------------------------------*/
/*777777777777777777777777777777777777777777777777777777777777777777777777777777777777*/
void Hum( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

  for (;;)
  {
    
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
    
     humidity = Humedad.getRH();
     temph=Humedad.getTemp();
   // Serial.print(",Humidity:");
   // Serial.print(humidity);
    //Serial.println("%");

      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }

    vTaskDelay(10000/portTICK_PERIOD_MS); 
  }
}

/*777777777777777777777777777777777777777777777777777777777777777777777777777777777777*/

//**********************************************************************************************
void Impresion( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

  for (;;)
  {
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      Serial.print("@");
      //Serial.print();
      Serial.print("Windspeed:");
      Serial.print(windSpeed,2);
      Serial.print(":WindDir: ");
      Serial.print(windir,2); 
      Serial.print(":Pressure: ");
      Serial.print(pressure,2);  
      Serial.print(":Altitude: ");
      Serial.print(altitude,2);
      Serial.print(":Temperature: ");
      Serial.print(temperature,2);
      Serial.print(":TemperatureHH: ");
      Serial.print(temph,2);      
      Serial.print(":Humedad R: ");
      Serial.print(humidity,2);
      Serial.print(":Rainin: ");
      Serial.print(rainin,2);
      Serial.print(":DailyRainIn: ");
      Serial.print(dailyrainin,2);
      Serial.println(":#");                           


      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }

    vTaskDelay(10000/portTICK_PERIOD_MS); 
  }
}

//**********************************************************************************************
