#include <ESP8266WiFi.h>
#include <PubSubClient.h>

typedef void (*GeneralMessageFunction) ();


/**
 * Chirp Class manages emitting a chirp on the speaker for a given pin, length and frequency 
 */
class Chirp {
  private:
    int _chirpUntil = 0;
    int _chirpFrequency;
    int _chirpPin;
    int _chirpTime;
    int _ledPin;
    
  public:
    Chirp(int pin, int t, int frequency, int ledPin =-1) : _chirpFrequency(frequency),  _chirpPin(pin), _chirpTime(t), _ledPin(ledPin) {}
    
    virtual void set() {
      _chirpUntil = millis() + _chirpTime;
      tone(_chirpPin, _chirpFrequency); 
      if(_ledPin > -1) { digitalWrite(_ledPin, HIGH);}
    }

    virtual void loop(){
      if(millis() > _chirpUntil) {
        noTone(_chirpPin);
        if(_ledPin > -1) { digitalWrite(_ledPin, LOW);}
      }
      
    }  
};

/**
 * StatusLED manages flashing an LED at a set interval to indicate life, or lack thereof.
 */
class StatusLED {
  private:
    int _on;
    int _off;
    int _pin;
    int _thisOn; 
    int _thisOff;

  public:
    StatusLED(int pin, int on, int off) : _off(off), _on(on), _pin(pin) {}

    virtual void loop() {
      if (millis() < _thisOn){
        digitalWrite(_pin, HIGH);
      }else{
        digitalWrite(_pin, LOW);
      }

      if(millis() > _thisOff) {
        _thisOn = millis() + _on;
        _thisOff = millis() + _on + _off;
      }

    }

};

/**
 * Debounces a button. 
 * Given a pin of the input, a state that the button desires to be in, debounce, and then a retry delay
 * Pin = the pin of the button/input
 * DesireState = what state the debounce should activate
 * Debounce = ms the input must be in desire state before register
 * Retry = ms that the button is no longer active after an activtion
 * 
 */
class Debounce {
private:
  int _pin;
  int _desireState;
  int _debounce;
  int _retry;
  int _prevState;
  int _fired;
  int _thisState;
  int _thisDebounce;
  int _thisRetry;

public:
  Debounce(int pin, int desireState, int debounce, int retry) : _pin(pin), _desireState(desireState), _debounce(debounce), _retry(retry) {}
  
  virtual void init(){
    _prevState = digitalRead(_pin);  
    _fired = 0;
  }
  
  virtual bool loop() {

    _thisState = digitalRead(_pin);
    
    if(_thisState == _desireState){

      if(_thisState != _prevState && (millis() > _thisRetry)){
        Serial.println("Updating");
        _prevState = _thisState;
        _thisDebounce = millis() + _debounce;
        _thisRetry = millis() + _debounce + _retry;
      }

      //in the disired state
      if(millis() < _thisDebounce){Serial.println("not yet"); return false;}
      if(millis() > _thisRetry) {Serial.println("too far"); return false;}

      if(_fired) return false;
      
      _fired = 1;
      return true;

    }else{
      if(millis() < _thisRetry){_thisDebounce = 0; _thisRetry = 0; _fired = 0;}
      _prevState = _thisState;
      return false;
    }
  }
};


//WiFi Details
const char* ssid = "";
const char* password = "";
const char* mqtt_server = "172.16.0.5";

//Pin Details
const int NUM_PINS = 7;
const int pins [NUM_PINS] = {5,2,4,0,14,12,13 };

//State variables
volatile int doorState = 0;

//Nice names 
const int OVERRIDE_PIN = pins[0];
const int DOORBELL_PIN = pins[1];
const int DOOR_PIN = pins[2];
const int WARNLED_PIN = pins[3];
const int STATUS_PIN = pins[4];
const int SOUNDER_PIN = pins[5];
const int AUX_PIN = pins[6];

Chirp chirp(SOUNDER_PIN, 900, 900, WARNLED_PIN);
StatusLED statusLED(STATUS_PIN, 80, 800);
Debounce doorbellDebounce(DOORBELL_PIN, LOW, 50, 5000);
Debounce overrideDebounce(OVERRIDE_PIN, LOW, 50, 2000);
Debounce auxDebounce(AUX_PIN, LOW, 1000, 5000);

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

void setup() {


  pinMode(OVERRIDE_PIN, INPUT_PULLUP); 
  pinMode(DOORBELL_PIN, INPUT_PULLUP); 
  pinMode(DOOR_PIN, INPUT_PULLUP);
  pinMode(AUX_PIN, INPUT_PULLUP);
     
  pinMode(WARNLED_PIN, OUTPUT); 
  pinMode(STATUS_PIN, OUTPUT); 
  pinMode(SOUNDER_PIN, OUTPUT); 


  doorbellDebounce.init();
  overrideDebounce.init();
  auxDebounce.init();
 
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
    digitalWrite(WARNLED_PIN, HIGH);
    Serial.print(".");
    delay(100);
    digitalWrite(WARNLED_PIN, LOW);
    delay(100);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");

    // Attempt to connect
    if (client.connect("ESP8266Client", "system/keysensor/state", 2, true, "offline")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("system/keysensor/state", "online", true);
      delay(1000);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop() {

  if (!client.connected()) reconnect();
  client.loop();
  chirp.loop();
  statusLED.loop();
  
  
  if(doorbellDebounce.loop()){
    client.publish("door/outer/doorbell", "");
    chirp.set();
  }

  if(overrideDebounce.loop()){
    client.publish("door/outer/opened/key", "");
    chirp.set();
  }

  if(auxDebounce.loop()){
    client.publish("door/outer/aux", "");
    chirp.set();
  }

  
  int live_door_state = digitalRead(DOOR_PIN);
  if(live_door_state != doorState){
    Serial.println("DOOR STATE");
    Serial.println(live_door_state);
    client.publish("door/outer/state", live_door_state?"opened":"closed");
    doorState = live_door_state;
    chirp.set();
  }

}
