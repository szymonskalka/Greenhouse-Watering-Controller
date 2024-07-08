#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <DHT.h> 
#include <DHT_U.h> 
#include <LiquidCrystal_I2C.h>

//pin setup
#define buttonPin 2
#define relayPin 4  
#define DHTPIN 7
int soilSensorPin = A2;

// relay timings all in seconds
// WATERING is the preset watering time
// ADJUSTED_WATERING is the actual watering time after adjusting for conditions
// LOOP is total loop time 
float WATERING  = 45;
float ADJUSTED_WATERING  = WATERING;
float LOOP  = 86400; //24h so that watering happens once per day

#define DHTTYPE DHT11
DHT_Unified dht(DHTPIN,DHTTYPE);

LiquidCrystal_I2C lcd(0x27, 16, 4);

float readSoilHumidity(){
  Serial.print(analogRead(soilSensorPin));
  Serial.print(" ");
  return ( 100 - ( (analogRead(soilSensorPin)/1023.00) * 100 ));
}

void toggleRelay(float time){
  digitalWrite(relayPin,HIGH);
  delay(time*1000); 
  digitalWrite(relayPin,LOW);
}

void printAirData(){
  sensors_event_t event;

  dht.temperature().getEvent(&event);
  lcd.setCursor(0,0);
  if (isnan(event.temperature)) {
    lcd.println(F("Error!"));
  }  
  else {
    lcd.print("Temperature:  ");
    lcd.print(event.temperature, 1);    
    lcd.print("C ");
  }

  dht.humidity().getEvent(&event);
  lcd.setCursor(0,1);
  if (isnan(event.relative_humidity)) {
    lcd.println(F("Error!"));
  }
  else{
    lcd.print(" Humidity:    ");
    lcd.print( event.relative_humidity , 1);
    lcd.print("% ");
  }
}

void toggleWateringTime(){
  if(digitalRead(buttonPin) == HIGH){
    WATERING+=5;  
  }
    Serial.print(digitalRead(buttonPin));
  if(WATERING > 60)
    WATERING=20;  
}

void  adjustWateringForConditions(){
  ADJUSTED_WATERING = WATERING;

  sensors_event_t event;
  dht.temperature().getEvent(&event);
  dht.humidity().getEvent(&event);

  if(readSoilHumidity() < 30)
    ADJUSTED_WATERING*=1.3;
  if(readSoilHumidity() > 70)
    ADJUSTED_WATERING*=0.7;

  if(event.relative_humidity < 30)
    ADJUSTED_WATERING*=1.1;

  if(event.temperature > 25){
    ADJUSTED_WATERING*=1.2;
      if(event.temperature > 30){
        ADJUSTED_WATERING*=1.2;
      }
  }   

}


void setup() {
  Serial.begin(9600);

  lcd.init();
  lcd.noBacklight();
  dht.begin();

  pinMode(soilSensorPin, INPUT);
  pinMode(buttonPin, INPUT);
  pinMode(relayPin, OUTPUT);

  digitalWrite(relayPin,LOW);
}

//used for turning on the screen
bool backlight = 0;
float backlightTimer;

void loop() {;
  adjustWateringForConditions();
  toggleRelay( ADJUSTED_WATERING);  

  if(backlight == 1)
    backlightTimer-=ADJUSTED_WATERING*2;

  for(float i=0.0f;i<=2*LOOP-2*ADJUSTED_WATERING; i+=1.0f){
    // enable changing watering time if backlight is on
    if(backlight == 1){
      toggleWateringTime();
      backlightTimer--;
    }
    // if backlightTimer has ran out disable backlight
    if(backlightTimer <= 0){
      lcd.noBacklight();
      backlight = 0;
    } 
    // if button is pressed reset backlight timer to 15s
    if(digitalRead(buttonPin) == HIGH){
      lcd.backlight();
      backlight = 1;
      backlightTimer = 30;
    }

    printAirData();

    lcd.setCursor(0,2);
    lcd.print("Watering time: ");
    lcd.print(WATERING, 1);
    lcd.print("s ");

    lcd.setCursor(0,3);
    lcd.print("Soil Humidity: ");
    lcd.print(readSoilHumidity(), 1);
    lcd.print("% ");

    // check for input and display sensor reading every 500ms
    delay(500);
  } 
}
