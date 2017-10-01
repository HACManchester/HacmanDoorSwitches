#include <ESP8266WiFi.h>
#include <PubSubClient.h>

//WiFi Details
const char* ssid = "";
const char* password = "";
const char* mqtt_server = "172.16.0.5";

//Pin Information
const int OVERRIDE_PIN = 5; //D1
const int DOORBELL_PIN = 2; //D4
const int STATE_PIN = 4; //D2
const int BUZZER_PIN = 0; //D3
const int STATUS_PIN = 14; // D5

//Indicators of an event we care about
volatile int ohfuck = 0; //Override 
volatile int ohhello = 0; //Doorbell

//Flags for when the override is triggered - flashing light without delay
volatile int shitFlag = 0;
const int shitFlagReset = 1000000;
volatile int shitFlagCount = 0;
int ledState = LOW;

//Normal flashing light
unsigned long previousMillis = 0;        // will store last time LED was updated
const long interval = 400; 


unsigned volatile long lastOverride = 0;
unsigned volatile long lastDoorbell = 0;
unsigned volatile long doorState = 0;

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

void setup() {
  //Set up pins
  pinMode(OVERRIDE_PIN, INPUT_PULLUP); // D1 input - override
  pinMode(DOORBELL_PIN, INPUT_PULLUP); // D2 Input - doorbell
  pinMode(STATE_PIN, INPUT_PULLUP); // D3 input - open sensor
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(STATUS_PIN, OUTPUT);

  //Indicate system is alive
  digitalWrite(BUZZER_PIN, LOW); 
  digitalWrite(STATUS_PIN, HIGH); 
  delay(3000);
  digitalWrite(STATUS_PIN, LOW); 

  //Set up interrupts
  attachInterrupt(digitalPinToInterrupt(OVERRIDE_PIN), ohshit, RISING);
  attachInterrupt(digitalPinToInterrupt(DOORBELL_PIN), ohhi, RISING);

  //Do the dance
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  
}

void setup_wifi() {

  delay(10);
  
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(STATUS_PIN, HIGH); 
    delay(500);
    digitalWrite(STATUS_PIN, LOW); 
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  digitalWrite(STATUS_PIN, HIGH); 
  delay(500);
  digitalWrite(STATUS_PIN, LOW); 
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    digitalWrite(STATUS_PIN, HIGH); 
    // Attempt to connect
    if (client.connect("ESP8266Client", "system/keysensor/state", 1, true, "online")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("system/keysensor/state", "online");
      delay(1000);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
    digitalWrite(STATUS_PIN, LOW); 
  }
}


void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  if (ohfuck) {
    client.publish("door/outer/opened/username", "MANUAL OVERRIDE KEY");
    ohfuck = 0;
    chirp();
  }
  if (ohhello) {
    client.publish("door/outer/doorbell", "");
    ohhello = 0;
    chirp();
  }

  int live_door_state = digitalRead(STATE_PIN);
  if(live_door_state != doorState){
    Serial.println("DOOR STATE");
    Serial.println(live_door_state);
    client.publish("door/outer/state", live_door_state?"opened":"closed");
    doorState = live_door_state;
    chirp();
  }

  
  
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
      digitalWrite(STATUS_PIN, HIGH);
      delay(10);
      digitalWrite(STATUS_PIN, LOW);
    } else {
      ledState = LOW;
    }
    if(shitFlag) digitalWrite(BUZZER_PIN, ledState);
    
  }

  if(shitFlag){
    shitFlagCount ++;
  
    if(shitFlagCount > shitFlagReset){
      shitFlag = 0;
      shitFlagCount = 0; 
      digitalWrite(BUZZER_PIN, LOW); 
    }
  }
}

// Called when an override happens on the switch
void ohshit() {
  Serial.println("OVERRIDE");
  unsigned long now = millis();
  if (now - lastOverride >= 1000) {
    ohfuck = 1;
    shitFlag = 1;
    lastOverride = now;
  }
}

//Called when the doorbell is rung
void ohhi() {
  Serial.println("DOORBELL"); 
  unsigned long now = millis();
  if (now - lastDoorbell >= 5000) {
    ohhello = 1;
    lastDoorbell = now;
  }
}

//Flashes the buzzer pin to indicate a thing happened
void chirp() {
  Serial.println("chirp");
  digitalWrite(BUZZER_PIN, HIGH); 
  delay(500);
  digitalWrite(BUZZER_PIN, LOW); 
}

