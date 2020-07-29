

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "I2Cdev.h"
#include "MPU9250.h"

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1305.h>


// Used for I2C or SPI
#define OLED_RESET D3

// software SPI
//Adafruit_SSD1305 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
// hardware SPI
//Adafruit_SSD1305 display(OLED_DC, OLED_RESET, OLED_CS);

// Used for software SPI
#define OLED_CLK 13
#define OLED_MOSI 11

// Used for software or hardware SPI
#define OLED_CS 10
#define OLED_DC 8

// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU9250 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
//#define OUTPUT_READABLE_ACCELGYRO

// I2C
Adafruit_SSD1305 display(OLED_RESET);

#define DEBUG

#ifdef DEBUG
 #define DEBUG_PRINT(x)  Serial.print (x)
 #define DEBUG_PRINTLN(x)  Serial.println (x)
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTLN(x)
#endif

#define USE_ACCEL 3
#define USE_GYRO 3
#define USE_MAGN 3

#define LED_PIN 13

bool blinkState = false;

uint16_t lastReport;
const int numReadings = 25;

#if defined(USE_ACCEL) && defined(USE_GYRO) && defined(USE_MAGN)
const int numAxis = USE_ACCEL + USE_GYRO + USE_MAGN;
const int AX = 0;
const int AY = 1;
const int AZ = 2;
const int GX = 3;
const int GY = 4;
const int GZ = 5;
const int MX = 6;
const int MY = 7;
const int MZ = 8;
#elif defined(USE_ACCEL)
const int numAxis = USE_ACCEL;
const int AX = 0;
const int AY = 1;
const int AZ = 2;
#elif defined(USE_GYRO)
const int numAxis = USE_GYRO;
const int GX = 0;
const int GY = 1;
const int GZ = 2;
#endif

double magnetField;

int32_t readings[numAxis][numReadings];  // the reading history
int32_t readIndex[numAxis];              // the index of the current reading
int32_t total[numAxis];                  // the running total
int32_t average[numAxis];                // the average

boolean flat = false;
uint32_t flatStarted = 0;
uint32_t flatDuration = 0;
uint32_t flatLastEnded = 0;

boolean north = false;
uint32_t northStarted = 0;
uint32_t northDuration = 0;
uint32_t northLastEnded = 0;

boolean south = false;
uint32_t southStarted = 0;
uint32_t southDuration = 0;
uint32_t southLastEnded = 0;

boolean east = false;
uint32_t eastStarted = 0;
uint32_t eastDuration = 0;
uint32_t eastLastEnded = 0;

boolean tiltEast = false;
uint32_t tiltEastStarted = 0;
uint32_t tiltEastDuration = 0;
uint32_t tiltEastLastEnded = 0;
uint32_t lastWasTiltedEast = 0;

boolean west = false;
uint32_t westStarted = 0;
uint32_t westDuration = 0;
uint32_t westLastEnded = 0;

boolean tiltWest = false;
uint32_t tiltWestStarted = 0;
uint32_t tiltWestDuration = 0;
uint32_t tiltWestLastEnded = 0;
uint32_t lastWasTiltedWest = 0;

boolean vertical = false;
uint32_t verticalStarted = 0;
uint32_t verticalDuration = 0;
uint32_t verticalLastEnded = 0;


boolean onTop = false;
uint32_t onTopStarted = 0;
uint32_t onTopDuration = 0;
uint32_t onTopLastEnded = 0;

boolean onSide = false;
uint32_t onSideStarted = 0;
uint32_t onSideDuration = 0;
uint32_t onSideLastEnded = 0;

boolean onBottom = false;
uint32_t onBottomStarted = 0;
uint32_t onBottomDuration = 0;
uint32_t onBottomLastEnded = 0;

boolean glowing = false;
uint32_t glowEnd = -1;
const uint32_t glowDuration = 2500;
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define PIN            D5

// How many Neostrip are attached to the Arduino?
#define NUMPIXELS      24

// When we setup the NeoPixel library, we tell it how many strip, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

int delayval = 500; // delay for half a second
unsigned long lastTimeLEDUpdatedNorth = 0;
unsigned long lastTimeLEDUpdatedSouth = 0;
unsigned long lastTimeLEDUpdatedEast = 0;
unsigned long lastTimeLEDUpdatedWest = 0;

//boolean LEDUpdate = false;
int northLED[]= {3,5,6,8};//{0,2,21,23};
int southLED[]= {15,17,18,20};
int eastLED[]= {0,2,21,23};
int westLED[]= {9,11,12,14};//{15,17,18,20};

/*int *northColor[3];
int *southColor[3];
int *eastColor[3];
int *westColor[3];*/


int blue[] = {0, 0, 150};
int blueValue = 150;
int red[] = {150, 0, 0};
int redValue = 9830400;
int green[] = {0, 150, 0};
int greenValue = 38400;
int yellow[] = {150, 150, 0};
int yellowValue = 9868800;


int *northColor;
int *southColor;
int *eastColor;
int *westColor; 

int LEDCounterNorth = 0;
int LEDCounterSouth = 0;
int LEDCounterEast = 0;
int LEDCounterWest = 0;


const char* WIFI_SSID =  "F.K.Knudsens iPhone";    
const char* WIFI_PWD =  "Adgangskode123";
const char* toDoStateURI = "Cubes/cube2/state/todo";
const char* inProgressStateURI = "Cubes/cube2/state/inprogress";
const char* doneStateURI = "Cubes/cube2/state/done";
const char* shareSendURI = "Cubes/cube2/share/send";
const char* shareReceiveURI = "Cubes/cube2/share/receive";
const char* mergeReceiveURI = "Cubes/cube2/merge/receive";
const char* mergeSendURI = "Cubes/cube2/merge/send";

const char* getTasksURI = "Cubes/cube2/gettasks/";
const char* switchURI = "Cubes/cube2/switch/";
const char* callbackURI = "Cubes/cube2/+";


String labelID = "5bbf0bbb9c16fb124a57451d";
String memberID = "5ba0e97eca344126c573b51c";

String Card1 = "5be56aef4250ed39d43c5335";
String Card2 = "5be56af46c0b2b31699750c1";
String Card3 = "5be56af7f45a753fb2df8969";
String activeCardDescription;
String activeCardListName;
String activeCardName;

boolean startUp = false;
boolean readyToShare = false;
boolean receiveTask = false;
boolean sendTask = false;
boolean readyToMerge = false;
boolean receiveMerge = false;
boolean sendMerge = false;

int vibrationCounter = 0;

String payloadID;

String activeCardTest;
int switchCounterLeft = 1;
int switchCounterRight = 1;

String shareTaskId = Card2;

String activeCard = Card1;
String oldActiveCard = Card1;

int displayCallbackCounter = 0;

DynamicJsonBuffer jsonResource;
JsonArray& resource = jsonResource.createArray();


DynamicJsonBuffer JSONbuffer;
char JSONmessageBuffer[500];


const int UPDATE_INTERVAL_SECS = 10; // Update every 10 minutes



IPAddress server(165, 227, 175, 8);
WiFiClient wclient;
PubSubClient client(wclient);

int motorPin = D7;

boolean displayInverse = true;

void HomeKit(const char* URI, const char* message) {
    if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) {
      if (client.connect("Cube2")) {
        client.publish("Status","Cube2 is online");
      }
    }
    if (client.connected()){  
        Serial.println("Sending");
        client.publish(URI,  message);   
        client.loop();
    }
      
  } else {
    Serial.println("No wifi");
    }
    Serial.println("Sending completed");
}

void setup() {

  strip.begin(); // This initializes the NeoPixel library.
  for(int i=0;i<4;i++){

    // strip.Color takes RGB values, from 0,0,0 up to 255,255,255
    strip.setPixelColor(northLED[i], strip.Color(50,50,50)); // Moderately bright green color.
    
  }
  for(int i=0;i<4;i++){

    // strip.Color takes RGB values, from 0,0,0 up to 255,255,255
    strip.setPixelColor(southLED[i], strip.Color(50,50,50)); // Moderately bright green color.
    
  }
  for(int i=0;i<4;i++){

    // strip.Color takes RGB values, from 0,0,0 up to 255,255,255
    strip.setPixelColor(eastLED[i], strip.Color(50,50,50)); // Moderately bright green color.
    
  }
  for(int i=0;i<4;i++){

    // strip.Color takes RGB values, from 0,0,0 up to 255,255,255
    strip.setPixelColor(westLED[i], strip.Color(50,50,50)); // Moderately bright green color.
    
  }
  strip.show();
  lastReport = millis();
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  setUpJsonBuffer(activeCard, shareTaskId, oldActiveCard);
  Serial.begin(115200);
  //Serial.begin(38400);


  // initialize device
  DEBUG_PRINTLN("Initializing I2C devices...");
  accelgyro.initialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  // 1688 factory default for my test chip

    
  // verify connection
  DEBUG_PRINTLN("Testing device connections...");
  DEBUG_PRINTLN(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // zero-fill all the arrays:
  for (int axis = 0; axis < numAxis; axis++) {
        readIndex[axis] = 0;
        total[axis] = 0;
        average[axis] = 0;
        for (int i = 0; i<numReadings; i++){
            readings[axis][i] = 0;
        }
    }

    while(!Serial) {}
  Serial.println("SSD1305 OLED test");

  display.begin();
  display.display();
  delay(1000);
  display.clearDisplay();
 
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PWD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  client.setServer(server, 1883);
  client.setCallback(callback);
  client.subscribe(callbackURI);
northColor = blue;
southColor = yellow;
eastColor = green;
westColor = red; 

pinMode(motorPin, OUTPUT);
}


void loop() {

if(millis()>8000 && !startUp){
  startUp = true;
  for (JsonArray::iterator it=resource.begin(); it!=resource.end(); ++it) {
    resource.remove(it);
  }
  delay(100);
  HomeKit(getTasksURI, JSONmessageBuffer);
  
}

getAccel_Data();
  checkFlat();
  checkWest();
  checkTiltWest();
  checkEast();
  checkTiltEast();
  checkNorth();
  checkSouth();
  //reportStates();
  checkCubeOnTop();
  checkCubeOnSide();
  checkCubeOnBottom();

 
  //Switch actuve task right
  if(wasTiltedEast()){
    shortVibration(100);
  Serial.println("Tilt East!!!");
  switchActiveCardRight();
  HomeKit(switchURI, JSONmessageBuffer);
  
  }
  //Switch active task left
  if(wasTiltedWest()){
    shortVibration(100);
  Serial.println("Tilt West!!!");
  //HomeKit(getTasksURI, JSONmessageBuffer);
  switchActiveCardLeft();
  HomeKit(switchURI, JSONmessageBuffer);
  
  
  }

   if(flat){
      vibrationCounter = 0;
      //Serial.println("Flat");
  }

  //Set active task in TODO
  if(west && vibrationCounter==0){
    Serial.println("Flat west");
    HomeKit(toDoStateURI, JSONmessageBuffer);
    vibrationCounter++;
    eastColor = westColor;
    northColor = westColor;
    southColor = westColor;
    shortVibration(100);
  }

  //Set active task in inprogress
  if(south && vibrationCounter==0 && !onSide && !readyToShare){
    Serial.println("Flat south");
    HomeKit(inProgressStateURI, JSONmessageBuffer);
    vibrationCounter++;
    eastColor = southColor;
    northColor = southColor;
    westColor = southColor;
    shortVibration(100);

  }

  //Set active task in done
  if(east && vibrationCounter==0){
    Serial.println("Flat east");
    HomeKit(doneStateURI, JSONmessageBuffer);
    vibrationCounter++;
    southColor = eastColor;
    northColor = eastColor;
    westColor = eastColor;
    shortVibration(100);
  }

  


// //   //Share a task
// //   if(digitalRead(D7)== LOW){
if(readyToShare && south && !receiveTask && !flat){
  receiveTask = true;
  Serial.println("receive Task once");
  for (JsonArray::iterator it=resource.begin(); it!=resource.end(); ++it) {
     resource.remove(it);
   }  
   HomeKit(shareReceiveURI, JSONmessageBuffer);
   //delay(300);
   doubleVibration(100);
  }


if(readyToShare && north && !sendTask && !flat){
  sendTask = true;
  Serial.println("send Task once");
  for(JsonArray::iterator it=resource.begin(); it!=resource.end(); ++it) {
     resource.remove(it);
   }  
   HomeKit(shareSendURI, JSONmessageBuffer);
   //delay(300);
   doubleVibration(100);
}

if(readyToMerge && !readyToShare && !receiveMerge && onTop){
  Serial.println("receive merge");
  receiveMerge = true;
  for (JsonArray::iterator it=resource.begin(); it!=resource.end(); ++it) {
     resource.remove(it);
   }  
   HomeKit(mergeReceiveURI, JSONmessageBuffer);
   //delay(300);
   doubleVibration(100);
}

if(readyToMerge && !readyToShare && !sendMerge && onBottom){
  Serial.println("send merge");
  sendMerge = true;
  for (JsonArray::iterator it=resource.begin(); it!=resource.end(); ++it) {
     resource.remove(it);
   }  
   HomeKit(mergeSendURI, JSONmessageBuffer);
   //delay(300);
   doubleVibration(100);
}

 client.loop();
 if (!client.connected()) {
  client.connect("Cube2");
  client.subscribe(callbackURI);

 }
  if(flat && flatDuration>100 && !onSide && !onBottom && !onTop /*|| flat && flatDuration>100 && !onTop || flat && flatDuration>100 && !onBottom*/){
    northColor = blue;
    southColor = yellow;
    eastColor = green;
    westColor = red;
    readyToShare = false;
    receiveTask = false;
    sendTask = false;
    readyToMerge = false;
    receiveMerge = false;
    sendMerge = false;
  }
  
  if(startUp){
    northFade(northColor, 1, northLED);
  
    southFade(southColor, 1, southLED);
  
    eastFade(eastColor, 1, eastLED);
 
    westFade(westColor, 1, westLED);
  }

  if(onSide && flat && onSideDuration > 800){
    southColor = blue;
    eastColor = blue;
    westColor = blue;
    if(strip.getPixelColor(southLED[1]) == blueValue){
      Serial.println("Ready to share");
      readyToShare = true;
    }
  }
  if(onTop && flat && onTopDuration > 800){
    southColor = blue;
    eastColor = blue;
    westColor = blue;
    if(strip.getPixelColor(southLED[1]) == blueValue){
      Serial.println("Ready to merge");
      readyToMerge = true;
    }
  }
  if(onBottom && flat && onBottomDuration > 800){
    southColor = blue;
    eastColor = blue;
    westColor = blue;
    if(strip.getPixelColor(southLED[1]) == blueValue){
      Serial.println("Ready to merge");
      readyToMerge = true;
    }
  }
}

//Callback from pi, here we pass payload, and read the needed values
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String stringTopic = String((char*) topic);
  JsonObject& root = JSONbuffer.parseObject(payload);
  if(stringTopic == "Cubes/cube2/callback"){
    moveLabelSwitchCallback(root);
  }
  if(stringTopic == "Cubes/cube2/callback2"){

    getCallback(root);
  } 
  if(stringTopic == "Cubes/cube2/callbackError"){
    display.setTextSize(1.2);
    displayTask();
    display.setCursor(0,0);
    display.println("Something went wrong! Try again");
    display.display();
  }
}

//For setting up the JSON buffer with updated values, which we send to the pi
void setUpJsonBuffer(String activeCardId, String shareTaskId, String oldActiveId){
  JSONbuffer.clear();
  JsonObject& JSONencoder = JSONbuffer.createObject();
  JSONencoder["Cube"] = "cube2";
  JSONencoder["Member"] = memberID;
  JSONencoder["Label"] = labelID;
  JSONencoder["ActiveId"] = activeCardId;
  JSONencoder["ShareTaskID"] = shareTaskId;
  JSONencoder["OldActiveID"] = oldActiveCard;


  JsonArray& values = JSONencoder.createNestedArray("taskIDs");
 
  values.add(Card1);
  values.add(Card2);
  values.add(Card3);
 
  
  JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
}

//display text
void updateDisplay(){
  display.setTextSize(1.2);
  displayTask();
  display.setCursor(2,0);
  display.println("Owner: Nicolai");
  display.setTextSize(1);
  display.setCursor(2,10);
  display.println("Task: "+ activeCardName);
  display.setCursor(2,30);
  display.println("In lane: " + activeCardListName);
  display.setCursor(2,40);
  display.println("Desc: " + activeCardDescription);
  display.display();
}

//Switch left between tasks
void switchActiveCardLeft() {
  int size = resource.size();
  for(int i = 0; i < resource.size(); i++)
  {
    if((resource.get<JsonObject>(i)).get<String>("id") == activeCard){
      oldActiveCard = activeCard;
      if(i == 0){
        activeCard = (resource.get<JsonObject>(size-1)).get<String>("id");
        oldActiveCard = (resource.get<JsonObject>(i)).get<String>("id");
        setUpJsonBuffer(activeCard, shareTaskId, oldActiveCard);
      }  
      if(i>0) {
        activeCard = (resource.get<JsonObject>((i-1))).get<String>("id");  
        setUpJsonBuffer(activeCard, shareTaskId, oldActiveCard);
      }   
      break;
    }
  }
}

//Swtich right between tasks
void switchActiveCardRight(){
    for(int i = 0; i < resource.size(); i++){
    if((resource.get<JsonObject>(i)).get<String>("id") == activeCard){
      oldActiveCard = activeCard;
      activeCard = (resource.get<JsonObject>((i+1))).get<String>("id");
       if(i+1 == resource.size()){
        activeCard = (resource.get<JsonObject>((0))).get<String>("id");
        oldActiveCard = (resource.get<JsonObject>(i)).get<String>("id");
        setUpJsonBuffer(activeCard, shareTaskId, oldActiveCard);
      }
      setUpJsonBuffer(activeCard, shareTaskId, oldActiveCard);
      break;
    }
  }
}

//Gets the needed variables and updates our buffer accordingly
void getCallback(JsonObject& root){
    const char* cardIDData = root["data"][1];
    const char* descriptionData = root["data"][0];
    const char* listNameData = root["data"][2];
    const char* callbackActiveCard = root["data"][3];
    const char* isActive = root["data"][4];
    const char* name = root["data"][5];
    String isActiveString = String((char*)isActive);
    String cardID = String((char*)cardIDData);
    activeCardName = String((char*)name);
    activeCardDescription = String((char*)descriptionData);
    activeCardListName = String((char*)listNameData);
    activeCard = String((char*)callbackActiveCard);
    oldActiveCard = activeCard;
    setUpJsonBuffer(activeCard, shareTaskId, oldActiveCard);

    JsonObject& id = resource.createNestedObject();
    id["id"] = cardID;
    id["description"] = activeCardDescription;
    id["listname"] = activeCardListName;
    //Set display for active card
    if(isActiveString == "true"){
      displayCallbackCounter ++;
      display.clearDisplay();
      updateDisplay();
    } 
    resource.printTo(Serial);
}

void moveLabelSwitchCallback(JsonObject& root){
   const char* activeCardData = root["data"][1];
  //const char* activeCardData = root["id"];
  const char* activeDescriptionData = root["data"][0];
  const char* activeCardListData = root["data"][2];
  const char* name = root["data"][3];
  activeCardName = String((char*)name);
  String activeCard = String((char*)activeCardData);
  activeCardDescription = String((char*)activeDescriptionData);
  activeCardListName = String((char*)activeCardListData);
  Serial.println(activeCard);
  setUpJsonBuffer(activeCard, shareTaskId, oldActiveCard);
  Serial.println();
  display.clearDisplay();
  updateDisplay();
}

void checkFlat(){
    #ifdef USE_ACCEL
    if( abs(average[AZ]) > 18000 && abs(average[AY]) < 2000 ){
        if(!flat){
            flatStarted = millis();
        }
        flatLastEnded = millis();
            
        flatDuration = millis() - flatStarted;
        
        flat = true;
    } else {
        flat = false;
    }
    #endif
}


void checkNorth(){
    #ifdef USE_ACCEL
    boolean AZ_in_range = 0 < average[AZ] && average[AZ] < 8000;
    boolean AY_in_range = -2000 < average[AY] && average[AY] < 2000;
    boolean AX_in_range = 20000 > average[AX] && average[AX] > 13500;
    if( AZ_in_range && AX_in_range && AY_in_range ){
        if(!north){
            northStarted = millis();
        }
        northLastEnded = millis();
            
        northDuration = millis() - northStarted;
        
        north = true;
    } else {
        north = false;
    }
    #endif
}

void checkSouth(){
    #ifdef USE_ACCEL
    boolean AZ_in_range = 0 < average[AZ] && average[AZ] < 8000;
    boolean AY_in_range = -2000 < average[AY] && average[AY] < 2000;
    boolean AX_in_range = -20000 < average[AX] && average[AX] < -13500;
    if( AZ_in_range && AX_in_range && AY_in_range ){
        if(!south){
            southStarted = millis();
        }
        southLastEnded = millis();
            
        southDuration = millis() - southStarted;
        
        south = true;
    } else {
        south = false;
    }
    #endif
}

void checkEast(){
    #ifdef USE_ACCEL
    boolean AZ_in_range = 0 < average[AZ] && average[AZ] < 8000;
    boolean AX_in_range = -2000 < average[AX] && average[AX] < 2000;
    boolean AY_in_range = -20000 < average[AY] && average[AY] < -13500;
    if( AZ_in_range && AX_in_range && AY_in_range ){
        if(!east){
            eastStarted = millis();
        }
        eastLastEnded = millis();
            
        eastDuration = millis() - eastStarted;
        
        east = true;
    } else {
        east = false;
    }
    #endif
}

void checkTiltEast(){
    #ifdef USE_ACCEL
    boolean AZ_in_range = 20000 > average[AZ] && average[AZ] > 14000;
    boolean AX_in_range = -2000 < average[AX] && average[AX] < 2000;
    boolean AY_in_range = -6000 < average[AY] && average[AY] < -2000;
    
    
    if( AZ_in_range && AX_in_range && AY_in_range ){
        if(!tiltEast){
            tiltEastStarted = millis();
        }
        tiltEastLastEnded = millis();
            
        tiltEastDuration = millis() - tiltEastStarted;
        
        tiltEast = true;
    } else {
        tiltEast = false;
    }
    #endif
}

void checkTiltWest(){
    #ifdef USE_ACCEL
    boolean AZ_in_range = 20000 > average[AZ] && average[AZ] > 14000;
    boolean AX_in_range = -2000 < average[AX] && average[AX] < 2000;
    boolean AY_in_range = 6000 > average[AY] && average[AY] > 2000;
    
    if( AZ_in_range && AX_in_range && AY_in_range ){
        if(!tiltWest){
            tiltWestStarted = millis();
        }
        tiltWestLastEnded = millis();
            
        tiltWestDuration = millis() - tiltWestStarted;
        
        tiltWest = true;
    } else {
        tiltWest = false;
    }
    #endif
}

void checkWest(){
    #ifdef USE_ACCEL
    boolean AZ_in_range = 0 < average[AZ] && average[AZ] < 8000;
    boolean AX_in_range = -2000 < average[AX] && average[AX] < 2000;
    boolean AY_in_range = 20000 > average[AY] && average[AY] > 13500;
    if( AZ_in_range && AX_in_range && AY_in_range ){
        if(!west){
            westStarted = millis();
        }
        westLastEnded = millis();
            
        westDuration = millis() - westStarted;
        
        west = true;
    } else {
        west = false;
    }
    #endif
}

boolean wasTiltedEast(){
    if(millis()-eastLastEnded > 2000 && flat && flatDuration > 60 && millis() - tiltEastLastEnded < 500 && 100 < tiltEastDuration && tiltEastDuration < 1500 && millis() - lastWasTiltedEast > 1000 ){
        // Sword of Omens, Give Me Sight Beyond Sight!
        lastWasTiltedEast = millis();
        return true;
    }

    return false;
}

boolean wasTiltedWest(){
    if(millis()-westLastEnded > 2000 && flat && flatDuration > 60 && millis() - tiltWestLastEnded < 500 && 100 < tiltWestDuration && tiltWestDuration < 1500 && millis() - lastWasTiltedWest > 1000 ){
        // Sword of Omens, Give Me Sight Beyond Sight!
        lastWasTiltedWest = millis();
        return true;
    }

    return false;
}

void checkCubeOnTop(){
  if(magnetField > 730 && magnetField < 790){
    if(!onTop){
            onTopStarted = millis();
        }
        onTopLastEnded = millis();
            
        onTopDuration = millis() - onTopStarted;
        
        onTop = true;
    } else {
        onTop = false;
    } 
}

void checkCubeOnSide(){
  if(magnetField > 550 && magnetField < 725){
    if(!onSide){
            onSideStarted = millis();
        }
        onSideLastEnded = millis();
            
        onSideDuration = millis() - onSideStarted;
        
        onSide = true;
    } else {
        onSide = false;
    } 
}

void checkCubeOnBottom(){
  if(magnetField > 160 && magnetField < 240){
    if(!onBottom){
            onBottomStarted = millis();
        }
        onBottomLastEnded = millis();
            
        onBottomDuration = millis() - onBottomStarted;
        
        onBottom = true;
    } else {
        onBottom = false;
    } 
}


void reportStates(){
    DEBUG_PRINT("flat: ");
    DEBUG_PRINT(flat);
    DEBUG_PRINT(" duration: ");
    DEBUG_PRINT(flatDuration);
    DEBUG_PRINT(" since last: ");
    DEBUG_PRINT(millis() - flatLastEnded);

    DEBUG_PRINT(" | tiltEast: ");
    DEBUG_PRINT(tiltEast);
    DEBUG_PRINT(" duration: ");
    DEBUG_PRINT(tiltEastDuration);
    DEBUG_PRINT(" since last: ");
    DEBUG_PRINT(millis() - tiltEastLastEnded);

    /*
    DEBUG_PRINT(" | north: ");
    DEBUG_PRINT(north);
    DEBUG_PRINT(" duration: ");
    DEBUG_PRINT(northDuration);
    DEBUG_PRINT(" since last: ");
    DEBUG_PRINT(millis() - northLastEnded);

    DEBUG_PRINT(" | south: ");
    DEBUG_PRINT(south);
    DEBUG_PRINT(" duration: ");
    DEBUG_PRINT(southDuration);
    DEBUG_PRINT(" since last: ");
    DEBUG_PRINT(millis() - southLastEnded);

    DEBUG_PRINT(" | east: ");
    DEBUG_PRINT(east);
    DEBUG_PRINT(" duration: ");
    DEBUG_PRINT(eastDuration);
    DEBUG_PRINT(" since last: ");
    DEBUG_PRINT(millis() - eastLastEnded);

    DEBUG_PRINT(" | west: ");
    DEBUG_PRINT(west);
    DEBUG_PRINT(" duration: ");
    DEBUG_PRINT(westDuration);
    DEBUG_PRINT(" since last: ");
    DEBUG_PRINT(millis() - westLastEnded);*/
    
    

    DEBUG_PRINT(" Glowing?: ");
    DEBUG_PRINTLN(glowing);
}

void getAccel_Data(void)
{
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  #ifdef USE_ACCEL
        smooth(AX, ax);
        smooth(AY, ay);
        smooth(AZ, az);
    #endif

    #ifdef USE_GYRO
        smooth(GX, gx);
        smooth(GY, gy);
        smooth(GZ, gz);
    #endif

    #ifdef USE_MAGN
        smooth(MX, mx);
        smooth(MY, my);
        smooth(MZ, mz);
    #endif

    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values

        // #ifdef USE_ACCEL
            DEBUG_PRINT(average[AX]);
            DEBUG_PRINT("\t");

            DEBUG_PRINT(average[AY]);
            DEBUG_PRINT("\t");

            DEBUG_PRINT(average[AZ]);
            DEBUG_PRINT("\t");
        // #endif

        // #ifdef USE_GYRO
            DEBUG_PRINT(average[GX]);
            DEBUG_PRINT("\t");

            DEBUG_PRINT(average[GY]);
            DEBUG_PRINT("\t");

            DEBUG_PRINT(average[GZ]);
            DEBUG_PRINT("\t");
        // #endif
            DEBUG_PRINT(mx);
            DEBUG_PRINT("\t");

            DEBUG_PRINT(my);
            DEBUG_PRINT("\t");

            DEBUG_PRINT(mz);
            DEBUG_PRINT("\t");

        DEBUG_PRINT(millis());
        DEBUG_PRINT("\t");

        DEBUG_PRINTLN(millis() - lastReport);
        lastReport = millis();

    #endif

magnetField = sqrt(average[MX]*average[MX] + average[MY]*average[MY] + average[MZ]*average[MZ]);
    
}
void smooth(int axis, int32_t val) {
    // pop and subtract the last reading:
    total[axis] -= readings[axis][readIndex[axis]];
    total[axis] += val;

    // add value to running total
    readings[axis][readIndex[axis]] = val;
    readIndex[axis]++;

    if(readIndex[axis] >= numReadings)
        readIndex[axis] = 0;

    // calculate the average:
    average[axis] = total[axis] / numReadings;
}

void northFade(int color[], uint8_t wait, int side[]) {
  uint32_t toColor = strip.Color(color[0],color[1],color[2]);
  if(strip.getPixelColor(side[1]) != toColor){
    //lastTimeLEDUpdatedNorth = millis();
      uint8_t startR, startG, startB;
      uint32_t startColor = strip.getPixelColor(side[LEDCounterNorth]); // get the current colour
      startB = startColor & 0xFF;
      startG = (startColor >> 8) & 0xFF;
      startR = (startColor >> 16) & 0xFF;  // separate into RGB components

      if ((startR != color[0]) || (startG != color[1]) || (startB != color[2])){  // while the curr color is not yet the target color
        if (startR < color[0]) startR = startR+10; else if (startR > color[0]) startR = startR-10;  // increment or decrement the old color values
        if (startG < color[1]) startG = startG+10; else if (startG > color[1]) startG = startG-10;
        if (startB < color[2]) startB = startB+10; else if (startB > color[2]) startB = startB-10;
        strip.setPixelColor(side[LEDCounterNorth], startR, startG, startB);  // set the color
        
        // delay(1);  // add a delay if its too fast
      }
      strip.show();
      LEDCounterNorth++;
      if(LEDCounterNorth==4){
        LEDCounterNorth=0;
      }
      //delay(wait);
  
  }
}

void southFade(int color[], uint8_t wait, int side[]) {
  uint32_t toColor = strip.Color(color[0],color[1],color[2]);
  if(strip.getPixelColor(side[1]) != toColor){
    //lastTimeLEDUpdatedSouth = millis();
      uint8_t startR, startG, startB;
      uint32_t startColor = strip.getPixelColor(side[LEDCounterSouth]); // get the current colour
      startB = startColor & 0xFF;
      startG = (startColor >> 8) & 0xFF;
      startR = (startColor >> 16) & 0xFF;  // separate into RGB components

      if ((startR != color[0]) || (startG != color[1]) || (startB != color[2])){  // while the curr color is not yet the target color
        if (startR < color[0]) startR = startR+10; else if (startR > color[0]) startR = startR-10;  // increment or decrement the old color values
        if (startG < color[1]) startG = startG+10; else if (startG > color[1]) startG = startG-10;
        if (startB < color[2]) startB = startB+10; else if (startB > color[2]) startB = startB-10;
        strip.setPixelColor(side[LEDCounterSouth], startR, startG, startB);  // set the color
        
        // delay(1);  // add a delay if its too fast
      }
      strip.show();
      LEDCounterSouth++;
      if(LEDCounterSouth==4){
        
        LEDCounterSouth=0;
      }
      //delay(wait);
  
  }
}

void eastFade(int color[], int wait, int side[]) {
  uint32_t toColor = strip.Color(color[0],color[1],color[2]);
  
  if(strip.getPixelColor(side[1]) != toColor){
    //lastTimeLEDUpdatedEast = millis();
      uint8_t startR, startG, startB;
      uint32_t startColor = strip.getPixelColor(side[LEDCounterEast]); // get the current colour
      startB = startColor & 0xFF;
      startG = (startColor >> 8) & 0xFF;
      startR = (startColor >> 16) & 0xFF;  // separate into RGB components

      if ((startR != color[0]) || (startG != color[1]) || (startB != color[2])){  // while the curr color is not yet the target color
        if (startR < color[0]) startR = startR+10; else if (startR > color[0]) startR = startR-10;  // increment or decrement the old color values
        if (startG < color[1]) startG = startG+10; else if (startG > color[1]) startG = startG-10;
        if (startB < color[2]) startB = startB+10; else if (startB > color[2]) startB = startB-10;
        strip.setPixelColor(side[LEDCounterEast], startR, startG, startB);  // set the color
        
        // delay(1);  // add a delay if its too fast
      }
      strip.show();
      LEDCounterEast++;
      if(LEDCounterEast==4){
        //LEDUpdate = true;
        LEDCounterEast=0;
      }
      //delay(wait);
  
  }
}

void westFade(int color[], int wait, int side[]) {
  uint32_t toColor = strip.Color(color[0],color[1],color[2]);
  if(strip.getPixelColor(side[1]) != toColor){
    //lastTimeLEDUpdatedWest = millis();
      uint8_t startR, startG, startB;
      uint32_t startColor = strip.getPixelColor(side[LEDCounterWest]); // get the current colour
      startB = startColor & 0xFF;
      startG = (startColor >> 8) & 0xFF;
      startR = (startColor >> 16) & 0xFF;  // separate into RGB components

      if ((startR != color[0]) || (startG != color[1]) || (startB != color[2])){  // while the curr color is not yet the target color
        if (startR < color[0]) startR = startR+10; else if (startR > color[0]) startR = startR-10;  // increment or decrement the old color values
        if (startG < color[1]) startG = startG+10; else if (startG > color[1]) startG = startG-10;
        if (startB < color[2]) startB = startB+10; else if (startB > color[2]) startB = startB-10;
        strip.setPixelColor(side[LEDCounterWest], startR, startG, startB);  // set the color
        
        // delay(1);  // add a delay if its too fast
      }
      strip.show();
      LEDCounterWest++;
      if(LEDCounterWest==4){
        //LEDUpdate = true;
        LEDCounterWest=0;
      }
      //delay(wait);
  
  }
}

void shortVibration(int delayValue){
  analogWrite(motorPin, 1024);
  delay(delayValue);
  analogWrite(motorPin, 0);
}

void doubleVibration(int delayValue){
  analogWrite(motorPin, 1024);
  delay(delayValue);
  analogWrite(motorPin, 0);
  delay(delayValue);
  analogWrite(motorPin, 1024);
  delay(delayValue);
  analogWrite(motorPin, 0);
}

void displayTask(){
  display.clearDisplay();
  if(displayInverse){
    display.fillScreen(WHITE);
    display.setTextColor(BLACK);
  }

  if(!displayInverse){
    display.setTextColor(WHITE);  }

   displayInverse = !displayInverse;
}
