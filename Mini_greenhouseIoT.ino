
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DallasTemperature.h>
#include "DHT.h"


#define DHTPIN D4  
#define DHTTYPE DHT11 


// NodeMCU pin layout
#define LCD_SCL_PIN D1          // GPI04
#define LCD_SDA_PIN D2          // GPI05
#define ONE_WIRE_BUS D3         // GPI00
#define BUTTON_DOWN_PIN D5      // GPIO14
#define BUTTON_UP_PIN D6        // GPI012
#define PUMP_PIN D7             // GPIO13
#define FAN_PIN D8             
#define BUTTON_BACKLIGHT_PIN D9 // GPIO3

// Global defines
#define T_TARGET_MIN 0           // Minimum target temperature (in degrees Celsius)
#define T_TARGET_MAX 100           // Maximum target temperature (in degrees Celsius)
#define H_TARGET_MIN 0
#define H_TARGET_MAX 100 
#define COLOR_RED "#E3233F"       // Color of the red LED in Blynk app
#define COLOR_GREEN "#68FC79"     // Color of the green LED in Blynk app
#define TIMER_CHECK_BLYNK  11000L // Frequency in ms to check Blynk connection 
//#define TIMER_TURN_OFF_LCD 60000L // Frequency in ms to turn off LCD backlight

DHT dht(DHTPIN, DHTTYPE);

// Setup a oneWire instance to communicate with the DS1820 temperature Sensors
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature Sensors(&oneWire);

// Setup Blynk timer, used to periodically check the Blynk connection
BlynkTimer Timer1;

// Setup Blynk timer, used to turn off LCD backlight after some time
BlynkTimer Timer2;

// Construct an LCD object and pass it the
// I2C address, width (in characters) and
// height (in characters). Depending on the
// Actual device, the IC2 address may change.
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Important: enter your own data below!
char MySSID[]           = "ICECY";                 // SSID of your WiFi network
char MyPassword[]       = "49118806";                  // Password of your WiFi network
char BlynkAuthToken[] = "NRXqQZMtjpCllUNlTZqPJ4Ejor2r_Vnp"; // Your Auth Token in the Blynk App, see the Project Settings (nut icon).

const char* resource = "/trigger/T&H&S_datalogger/with/key/bZSKizBnuHxaMvGgHoxW8n";
const char* server = "maker.ifttt.com";

float TempAir;                                              // Air temperature
float HumAir;                                              // Air temperature
float TempSoil;                                              // Air temperature
float Soilmoisture;                                             // Soil temperature
int   TempTarget;                                           // Target soil temperature
int   HumTarget; 
bool  Connected2Blynk;                                      // Blynk connection status
bool  ActuatorHeat;                                         // Actuator heating status
bool  ActuatorFan;
bool  ErrorStatus;                                          // Error status
String text_In;

// ------------------------FUNCTIONS------------------------------------------------------
BLYNK_WRITE(V2)
// This function will be calLED every time Slider Widget
// in Blynk app writes values to the Virtual Pin V2
{
  TempTarget = param.asInt();                            // Assigning incoming value from pin V2 to variable
  Blynk.virtualWrite(V5, TempTarget);
  Serial.println(TempTarget);
  lcd.backlight();                                       // Turn on the backlight
}


BLYNK_WRITE(V6)
// This function will be calLED every time Slider Widget
// in Blynk app writes values to the Virtual Pin 
{
  HumTarget = param.asInt();                            
  Blynk.virtualWrite(V7, HumTarget);
  Serial.println(HumTarget);
  lcd.backlight();                                       // Turn on the backlight
}


WidgetLED LED(V3);
// Register LED in Blynk app to Virtual Pin V3

WidgetLED LED2(V13);
// Register LED in Blynk app to Virtual Pin V13

BLYNK_WRITE(V11) {
  text_In = param.asStr();
  Blynk.virtualWrite(V12, text_In);
}




void setup() {
  // --------- Step 1: initialize the device
  InitDevice();

  // --------- Step 2: start WiFi
  InitWiFi();

  // --------- Step 3: connect to Blynk
  InitBlynk();

  // --------- Step 4: initialize LCD display
  InitLCD();

  

  
}

void loop() {
  // --------- Step 1: get sensor data
  GetSensorData();

  // --------- Step 2: get manual input
  GetManualInput();

  // --------- Step 3: take action if required
  Check4Action();

  // --------- Step 4: update Blynk
  ProcessBlynk();

  // --------- Step 5: update LCD display
  UpdateLCD();

  makeIFTTTRequest();
}

void InitDevice()
// Initialize device hardware and software
{
  Serial.begin(115200);
  delay(10);

  Serial.println("Initializing device");

  pinMode(BUTTON_UP_PIN, INPUT_PULLUP);                // Activate 'up button' pin as input including internal pullup
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);              // Activate 'down button' pin as input including internal pullup
  pinMode(BUTTON_BACKLIGHT_PIN, INPUT_PULLUP);         // Activate 'backlight button' pin as input including internal pullup
  pinMode(PUMP_PIN, OUTPUT);                           // Activate 'heat relay' pin as output
  pinMode(FAN_PIN, OUTPUT);

  
  // Start by turning heat relay OFF
  ActuatorHeat = false;
  ActuatorFan = false;
  digitalWrite(PUMP_PIN, LOW); // Turn heat relay off FAN_PIN
  digitalWrite(FAN_PIN, LOW);
  //Initialize global variables
  TempTarget = T_TARGET_MIN;
  HumTarget = H_TARGET_MIN;
  Connected2Blynk = false;
  ErrorStatus = false;

  // Setup the LCD display. The begin call takes the width and height.
  // This should match the number provided to the constructor.
  Wire.begin(LCD_SDA_PIN, LCD_SCL_PIN);
  lcd.begin();                                     // Initialize LCD display
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Blynk Thermostat");
  lcd.setCursor(0, 1);
  lcd.print("T&H&S Greenhouse");
  delay(2000);
  lcd.setCursor(0, 0);
  lcd.print("Initializing    ");
  lcd.setCursor(0, 1);
  lcd.print("Sensors...      ");
  // Start up the temperatur sensor library
  Sensors.begin();
  dht.begin();
  
}

void InitWiFi()
//Start WiFi connection
{
  // Check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // Don't continue:
    while (true);
  }

  // Connect to WiFi
  Serial.print("Connecting to ");
  Serial.println(MySSID);
  lcd.setCursor(0, 1);
  lcd.print("WiFi...         ");

  WiFi.begin(MySSID, MyPassword);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
    if (!DebounceKey(BUTTON_BACKLIGHT_PIN)) ESP.reset();  // Sometimes Wifi connection fails, use backlight button as reset button
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void InitBlynk()
// Connect to Blynk
{
  Serial.println("Connecting to Blynk");
  lcd.setCursor(0, 1);
  lcd.print("Blynk...        ");

  Blynk.config(BlynkAuthToken);
  // line below needs to be BEFORE Blynk.connect()
  Timer1.setInterval(TIMER_CHECK_BLYNK, CheckConnection); // Periodically check if Blynk still connected
  Blynk.connect();
  if (Blynk.connected())
  {
    Connected2Blynk = true;
    Serial.println("Connected to Blynk server");

    // Inform Blynk about initial status
    Blynk.setProperty(V3, "color", COLOR_RED);
    Blynk.setProperty(V3, "label", "Off");
    Blynk.virtualWrite(V4, ActuatorHeat);
    Blynk.virtualWrite(V2, TempTarget);
    Blynk.virtualWrite(V5, TempTarget);
    Blynk.virtualWrite(V6, HumTarget);
    Blynk.virtualWrite(V7, HumTarget);
    Blynk.virtualWrite(V9, ActuatorFan);
    Blynk.setProperty(V13, "color", COLOR_RED);
    Blynk.setProperty(V13, "label", "Off");
    
    LED.on();
    LED2.on();
    LED.setValue(255);
    LED2.setValue(255);
  }
}

void InitLCD()
// Show labels on LCD display
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T Air  Soil% Trgt");
  lcd.setCursor(0, 1);
  lcd.print("C               ");
  //Timer2.setInterval(TIMER_TURN_OFF_LCD, TurnOffLCD); // Periodically turn LCD backlight off
}

void GetSensorData()
// Get data from all temperature Sensors
{
  // Issue a global temperature request to all devices on the bus
  Serial.print("Requesting temperatures...");
  
  Sensors.requestTemperatures();   // Send the command to get temperatures
  Soilmoisture = map(analogRead(A0),0,1023,0,100);
  TempAir = dht.readTemperature();
  HumAir = dht.readHumidity();
  TempSoil = Sensors.getTempCByIndex(1);
  if (isnan(HumAir) || isnan(TempAir))   
  {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  Serial.println("done");
  Serial.print("Soil Moisture via AnalogA0 is: ");
  Serial.println(Soilmoisture);
  Serial.print("Air temperature via device 2 (index 1) is: ");
  Serial.println(TempAir);
  Serial.print("Air Humidity  is: ");
  Serial.println(HumAir);
  Serial.print("Soil Temperature  is: ");
  Serial.println(TempSoil);
  

  // Check if Sensors still work properly
  if ((Soilmoisture < -1) or (TempAir < -1) or (Soilmoisture > 100) or (TempAir > 100))
  {
    Serial.println("Invalid temperature(s) detected, sensor error");
    ErrorStatus = true;
  }
  else ErrorStatus = false;
}

void  GetManualInput()
// Check for manual input
{
  if (!DebounceKey(BUTTON_DOWN_PIN))
  {
    // Button to decrease target temperature is pressed
    Serial.println("Button down pressed");
    lcd.backlight();                  // Turn on the backlight
    TempTarget--;
    if (TempTarget < T_TARGET_MIN) TempTarget = T_TARGET_MIN;
    UpdateBlynk();
  }
  if (!DebounceKey(BUTTON_UP_PIN))
  {
    // button to increase target temperature is pressed
    Serial.println("Button up pressed");
    lcd.backlight();                  // Turn on the backlight
    TempTarget++;
    if (TempTarget > T_TARGET_MAX) TempTarget = T_TARGET_MAX;
    UpdateBlynk();
  }
 

  if (!DebounceKey(BUTTON_DOWN_PIN))
  {
    // Button to decrease target Humidity is pressed
    Serial.println("Button down pressed");
    lcd.backlight();                  // Turn on the backlight
    HumTarget--;
    if (HumTarget < H_TARGET_MIN) HumTarget = H_TARGET_MIN;
    UpdateBlynk();
  }
  if (!DebounceKey(BUTTON_UP_PIN))
  {
    // button to increase target Humidity is pressed
    Serial.println("Button up pressed");
    lcd.backlight();                  // Turn on the backlight
    HumTarget++;
    if (HumTarget > H_TARGET_MAX) HumTarget = H_TARGET_MAX;
    UpdateBlynk();
  }
  if (!DebounceKey(BUTTON_BACKLIGHT_PIN))
  {
    // button to turn on LCD backlight is pressed
    Serial.println("Button LCD backlight pressed");
    lcd.backlight();                  // Turn on the backlight
  }
}

void Check4Action()
// See if it is time to turn on or off the heat
{
  if (ErrorStatus)TurnHeatOn(false);
  else
  {
    if (!ActuatorHeat && (TempTarget > TempAir)){
      TurnHeatOn(true);
      }
    if ( ActuatorHeat && (TempTarget < TempAir)){
      TurnHeatOn(false);
      }
  }

  if (ErrorStatus)TurnFanOn(false);
  else
  {
    if (!ActuatorFan && (HumTarget > HumAir)){
      TurnFanOn(true);
      }
    if ( ActuatorFan && (HumTarget < HumAir)){
      TurnFanOn(false);
      }
  }
}

void TurnHeatOn(bool TurnOn)
// Turn heat on or off
{
  if (TurnOn)
  {
    Serial.println("Turning heat on");
    ActuatorHeat = true;
    // Turn heat relay on
    digitalWrite(PUMP_PIN, HIGH);
    // Inform Blynk
    Blynk.setProperty(V3, "color", COLOR_GREEN);
    Blynk.setProperty(V3, "label", "On");
    UpdateBlynk();
  }
  else
  {
    Serial.println("Turning heat off");
    ActuatorHeat = false;
    // Turn heat relay off
    digitalWrite(PUMP_PIN, LOW);
    // Inform Blynk
    Blynk.setProperty(V3, "color", COLOR_RED);
    Blynk.setProperty(V3, "label", "Off");
    UpdateBlynk();
  }
}


void TurnFanOn(bool TurnOn)
// Turn fan on or off
{
  if (TurnOn)
  {
    Serial.println("Turning fan on");
    ActuatorFan = true;
    // Turn fan relay on
    digitalWrite(FAN_PIN, HIGH);
    // Inform Blynk
    Blynk.setProperty(V13, "color", COLOR_GREEN);
    Blynk.setProperty(V13, "label", "On");
    UpdateBlynk();
  }
  else
  {
    Serial.println("Turning fan off");
    ActuatorFan = false;
    // Turn fan relay off
    digitalWrite(FAN_PIN, LOW);
    // Inform Blynk
    Blynk.setProperty(V13, "color", COLOR_RED);
    Blynk.setProperty(V13, "label", "Off");
    UpdateBlynk();
  }
}

void ProcessBlynk()
// Process Blynk
{
  if (Connected2Blynk) {
    Blynk.run();                    // Process incoming commands and perform housekeeping of Blynk connection
  }
  Timer1.run();                      // Poll timer1 to see if it is time for action
  Timer2.run();                      // Poll timer2 to see if it is time for action
}

void UpdateLCD()
// Show data on LCD display
{
  lcd.setCursor(0, 1);
  if (ErrorStatus)
  {
    lcd.backlight();                // Turn on the backlight
    lcd.print("! SENSOR ERROR !");
  }
  else
  {
    lcd.print("C               ");
    lcd.setCursor(2, 1);
    lcd.print(TempAir, 1);
    lcd.setCursor(7, 1);
    lcd.print(Soilmoisture, 1);
    lcd.setCursor(13, 1);
    lcd.print(TempTarget);
  }
}

void CheckConnection()
// Timer function to periodically check if we are still connected to Blynk server
{
  if (!Blynk.connected())
  {
    Serial.println("Not connected to Blynk server");
    Connected2Blynk = false;
    Blynk.connect();  // Try to connect to server with default timeout
  }
  else
  {
    Serial.println("Connected to Blynk server");
    Connected2Blynk = true;
    UpdateBlynk();   // Time to update data in the Blynk app
  }
}

void UpdateBlynk()
// Write device data to Blynk app via virtual pins
{
  Blynk.virtualWrite(V0, TempAir);
  Blynk.virtualWrite(V1, Soilmoisture);
  Blynk.virtualWrite(V2, TempTarget);
  Blynk.virtualWrite(V4, ActuatorHeat);
  Blynk.virtualWrite(V5, TempTarget);
  Blynk.virtualWrite(V6, HumTarget);
  Blynk.virtualWrite(V7, HumTarget);
  Blynk.virtualWrite(V9, ActuatorFan);
  Blynk.virtualWrite(V10, HumAir);
  
  
}

void TurnOffLCD()
{
  lcd.noBacklight(); // Turn off LCD backlight, can be switched on manually again
}

boolean DebounceKey(int pin)
// Deboucing a key
{
  boolean State;
  boolean PreviousState;
  const int DebounceDelay = 30;

  PreviousState = digitalRead(pin);
  for (int Counter = 0; Counter < DebounceDelay; Counter++)
  {
    delay(1);
    State = digitalRead(pin);
    if (State != PreviousState)
    {
      Counter = 0;
      PreviousState = State;
    }
  }
  return State;
}


void makeIFTTTRequest() {
  Serial.print("Connecting to "); 
  Serial.print(server);
  
  WiFiClient client;
  int retries = 5;
  while(!!!client.connect(server, 80) && (retries-- > 0)) {
    Serial.print(".");
  }
  Serial.println();
  if(!!!client.connected()) {
    Serial.println("Failed to connect...");
  }
  
  Serial.print("Request resource: "); 
  Serial.println(resource);

  // Temperature in Celsius
  String jsonObject = String("{\"value1\":\"") + dht.readTemperature() + "\",\"value2\":\"" + dht.readHumidity()
                      + "\",\"value3\":\"" + map(analogRead(A0),0,1023,0,100) + "\"}";
                      
  // Comment the previous line and uncomment the next line to publish temperature readings in Fahrenheit                    
  /*String jsonObject = String("{\"value1\":\"") + (1.8 * bme.readTemperature() + 32) + "\",\"value2\":\"" 
                      + (bme.readPressure()/100.0F) + "\",\"value3\":\"" + bme.readHumidity() + "\"}";*/
                      
  client.println(String("POST ") + resource + " HTTP/1.1");
  client.println(String("Host: ") + server); 
  client.println("Connection: close\r\nContent-Type: application/json");
  client.print("Content-Length: ");
  client.println(jsonObject.length());
  client.println();
  client.println(jsonObject);
        
  int timeout = 5 * 10; // 5 seconds             
  while(!!!client.available() && (timeout-- > 0)){
    delay(100);
  }
  if(!!!client.available()) {
    Serial.println("No response...");
  }
  while(client.available()){
    Serial.write(client.read());
  }
  
  Serial.println("\nclosing connection");
  client.stop(); 
}
