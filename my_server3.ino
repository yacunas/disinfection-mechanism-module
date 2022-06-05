//Yacunas, Ronnel James M.
// Disinfection Module

// Import required libraries
#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include "DHT.h"
#define DHTPIN 4     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11   // DHT 11
#include <Adafruit_MLX90614.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Arduino_JSON.h> 
#include <SPIFFS.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#define API_KEY "AIzaSyDUd8LrNRFRAiT4A5MN93uSooKGlJkYapM"
#define DATABASE_URL "https://disinfection-module-default-rtdb.asia-southeast1.firebasedatabase.app/" 


FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
bool signupOK = false;
int sessions;
int sessionid=0;

// Replace with your network credentials
const char* ssid = "Djhouse";
const char* password = "Dante102";

// Web Server HTTP Authentication credentials
const char* http_username = "admin";
const char* http_password = "admin";

static int time_elapsed = 0;

DHT dht(DHTPIN, DHTTYPE);

float last_temp = 0;
float last_hum = 0;

Servo myservo_left;  // create servo object to control a servo
const int servopin_left = 13;

Servo myservo_right;  
const int servopin_right = 12;
 
//LEFT DISTANCE SENSOR 5V 
const int trigPin_left = 5;
const int echoPin_left = 18;

//RIGHT DISTANCE SENSOR 5V
const int trigPin_right = 16;
const int echoPin_right = 17;

//define sound speed in cm/uS
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

long duration;
float distance;

// JSONVar dist_readings;
// JSONVar angle_readings;

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

const int output = 14;       // Output socket

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncEventSource events("/events");

const char* PARAM_INPUT_1 = "state";

String outputState(int gpio){
  if(digitalRead(gpio)){
    return "checked";
  }
  else {
    return "";
  }
}

String processor(const String& var){
  //Serial.println(var);
  if(var == "BUTTONPLACEHOLDER"){
    String buttons;
    String outputStateValue = outputState(output);
    buttons+="<div class=\"card card-switch\"><h4><i class=\"fas fa-wind\"></i> AIR COMPRESSOR</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"controlOutput(this)\" id=\"output\" " + outputStateValue + "><span class=\"slider\"></span></label></div>";
   
    return buttons;
  }
  
  return String();
}

String readDHTtemp(){
  float temp = dht.readTemperature();
  Serial.println("Temperature: "+ String(temp));
  if (isnan(temp)) {    
    Serial.println("Failed to read from DHT sensor!");
    Serial.println("Last Temperature: "+ String(last_temp));
    return String(last_temp);
  }
  else {
    last_temp = temp;
    return String(temp);
  }
}

String readDHThum(){
  float hum = dht.readHumidity();
    Serial.println("Humidity: "+ String(hum));

  if (isnan(hum)) {    
    Serial.println("Failed to read from DHT sensor!");
    Serial.println("Last Humidity: "+ String(last_hum));
    return String(last_hum);
  }
  else {
    last_hum = hum;
    return String(hum);
  }
}

float readMLXobj(){
  float obj = mlx.readObjectTempC();
   Serial.println("Obj Temperature: "+ String(obj));

  if (isnan(obj)) {    
    Serial.println("Failed to read from MLX sensor!");
    return 0.0;
  }
  else {
    return obj;
  }
}

float readDistance(int trigPin, int echoPin){
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin_left on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Reads the echoPin_left, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    distance = (duration * SOUND_SPEED/2)/100;
    // Calculate the distance
    return distance;

}

int calculateAngle(float dist, bool left){
  if(left){
    if(dist > 1){
      return 180;
    }

    else if(dist > 0.8) {
      return 150;
    }

    else if(dist > 0.3){
      return 130;
    }

    else if(dist > 0.01){
      return 115;
    }
    else {
      return 150;
    }
  }
  else{
    if(dist > 1){
      return 0;
    }

    else if(dist > 0.8) {
      return 30;
    }

    else if(dist > 0.3){
      return 50;
    }

    else if(dist > 0.01){
      return 65;
    }
    else {
      return 30;
    }
  }
}


int displayAngle(float dist, bool left){
  int angle = calculateAngle(dist, left);
  if(!left){
    if(dist > 1){
      return 180;
    }

    else if(dist > 0.8) {
      return 150;
    }

    else if(dist > 0.3){
      return 130;
    }

    else if(dist > 0.01){
      return 115;
    }
    else {
      return 180;
    }
  }
  else{
    return angle;
  }
}

void getSessionFirebase(){
  if (Firebase.RTDB.getInt(&fbdo, "/sessions")) {
      if (fbdo.dataType() == "int") {
        sessions = fbdo.intData();
        Serial.println(sessions);
        sessions+=1;
      }
      Firebase.RTDB.setInt(&fbdo, "/sessions", sessions);
    }
    else {
      Serial.println(fbdo.errorReason());
      Firebase.RTDB.setInt(&fbdo, "/sessions", 1);
    }
}

void sendFirebaseFloat(float value, String location){
    if (Firebase.RTDB.setFloat(&fbdo, location, value)){
      // Serial.println("PASSED");
    }
    else {
      // Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }
}

void sendFirebaseInt(int value, String location){
    if (Firebase.RTDB.setInt(&fbdo, location, value)){
      // Serial.println("PASSED");
    }
    else {
      // Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }
}

// Logged out web page
const char logout_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
</head>
<body>
  <p>Logged out or <a href="/">return to homepage</a>.</p>
  <p><strong>Note:</strong> close all web browser tabs to complete the logout process.</p>
</body>
</html>
)rawliteral";

void setup(){
  // Serial port for debugging purposes
  Serial.begin(115200);
    
  dht.begin();
  Serial.println("DHT11 Started...");
  
  mlx.begin();
  Serial.println("MLX90614 Started...");

  myservo_left.attach(servopin_left);  // attaches the servo on the servoPin to the servo object
  myservo_right.attach(servopin_right);  // attaches the servo on the servoPin to the servo object
  Serial.println("Servos Started...");

  //Left Distance Sensor
  pinMode(trigPin_left, OUTPUT); // Sets the trigPin_left as an Output
  pinMode(echoPin_left, INPUT); // Sets the echoPin_left as an Input


  //Right Distance Sensor
  pinMode(trigPin_right, OUTPUT); // Sets the trigPin_right as an Output
  pinMode(echoPin_right, INPUT); // Sets the echoPin_right as an Input
  Serial.println("Distance Sensor Started...");


  // initialize the LED pin as an output
  pinMode(output, OUTPUT);
  Serial.println("Relay Started...");

    if(!SPIFFS.begin()){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());

 config.api_key = API_KEY;
 config.database_url = DATABASE_URL;

   /* Sign up */
  if (Firebase.signUp(&config, &auth, "", "")){
    Serial.println("ok");
    signupOK = true;
  }
  else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  
  getSessionFirebase();
  Serial.println("firebase started");

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
   if(!request->authenticate(http_username, http_password))
      return request->requestAuthentication();
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });
  server.on("/logged-out", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", logout_html, processor);
  });
  server.on("/logout", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(401);
  });


  // Send a GET request to control output socket <ESP_IP>/output?state=<inputMessage>
  server.on("/output", HTTP_GET, [] (AsyncWebServerRequest *request) {
    if(!request->authenticate(http_username, http_password))
      return request->requestAuthentication();
    String inputMessage;
    // GET gpio and state value
    if (request->hasParam(PARAM_INPUT_1)) {
      inputMessage = request->getParam(PARAM_INPUT_1)->value();
      digitalWrite(output, inputMessage.toInt());
      request->send(200, "text/plain", "OK");
    }
    request->send(200, "text/plain", "Failed");
  });

  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis and set reconnect delay to 1 second
    client->send("hello!",NULL,millis(),1000);
  });
  server.addHandler(&events);
  
  // Start server
  server.begin();
}
 
void loop(){
  static unsigned long lastEventTime = millis();
  static const unsigned long EVENT_INTERVAL_MS = 2500;

  static unsigned long lastEventTime2 = millis();
  static const unsigned long EVENT_INTERVAL_MS2 = 1000;


  if ((millis() - lastEventTime) > EVENT_INTERVAL_MS) {
    events.send("ping",NULL,millis());
    float roomTemp = readDHTtemp().toFloat();
    float roomHum = readDHThum().toFloat();
    float objTemp = readMLXobj();
    float distance_left_m = readDistance(trigPin_left, echoPin_left);
    int nozzle_angle_left = calculateAngle(distance_left_m, true);
    float distance_right_m = readDistance(trigPin_right, echoPin_right);
    int nozzle_angle_right = calculateAngle(distance_right_m, false);
    int nozzle_angle_right_disp = displayAngle(distance_right_m, false);

    

    if(objTemp > roomTemp){
    events.send(String(objTemp).c_str(),"body_temperature",millis());
    Serial.println(String("Body Temp: "+ String(objTemp)));
    }
    else{
      objTemp = 0;
      events.send(String(" ").c_str(),"body_temperature",millis());
    }
    
    events.send(String(roomTemp).c_str(),"temperature",millis());
    Serial.println(String("Room Temp: "+ String(roomTemp)));

    events.send(String(roomHum).c_str(),"humidity",millis());
    Serial.println(String("Room Hum: "+ String(roomHum)));

    events.send(String(distance_left_m).c_str(),"distance_left",millis());
    Serial.println(String("Distance Left: "+ String(distance_left_m)));

    events.send(String(distance_right_m).c_str(),"distance_right",millis());
    Serial.println(String("Distance Right: "+ String(distance_right_m)));


       //140 ubos 120 taas
    //140 if greater than 1m
    //130 if greater than 0.8m
    //120 if greater than 0.3m
    //LEFT---------------------------------
//    myservo_left.attach(servopin_left);
    events.send(String(nozzle_angle_left).c_str(),"nozzle_angle_left",millis());
    Serial.println(String("Angle Left: "+ String(nozzle_angle_left)));
    myservo_left.write(nozzle_angle_left);
    
    //RIGHT---------------------------------
//    myservo_right.attach(servopin_right);
    events.send(String(nozzle_angle_right_disp).c_str(),"nozzle_angle_right",millis());
    Serial.println(String("Angle Right: "+ String(nozzle_angle_right_disp)));
    myservo_right.write(nozzle_angle_right);

    Serial.println(String(sessionid).c_str());

    sendFirebaseFloat(roomTemp, "session_"+String(sessions)+"/"+String(sessionid)+"/room_temp/");
    sendFirebaseFloat(roomHum, "session_"+String(sessions)+"/"+String(sessionid)+"/room_hum/");
    sendFirebaseFloat(objTemp, "session_"+String(sessions)+"/"+String(sessionid)+"/body_temp/");
    sendFirebaseFloat(distance_left_m, "session_"+String(sessions)+"/"+String(sessionid)+"/distance_left/");
    sendFirebaseFloat(distance_right_m, "session_"+String(sessions)+"/"+String(sessionid)+"/distance_right/");
    sendFirebaseInt(nozzle_angle_left, "session_"+String(sessions)+"/"+String(sessionid)+"/angle_left/");
    sendFirebaseInt(nozzle_angle_right, "session_"+String(sessions)+"/"+String(sessionid)+"/angle_right/");

    sessionid+=1;
    
    lastEventTime = millis();
    
  }

  
 if ((millis() - lastEventTime2) > EVENT_INTERVAL_MS2) {
    time_elapsed = millis() / 1000;

    events.send(String(time_elapsed).c_str(),"time_elapsed",millis());

    lastEventTime2 = millis();

  }
}
