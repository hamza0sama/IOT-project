#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <ESPSupabase.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>

#define SERVO_PIN 14
#define LDR_PIN 34
#define BUZZER_PIN 12
#define IR_PIN 32
#define LED_RED 26
#define LED_GREEN 27
#define BUTTON_PIN 25
LiquidCrystal_I2C lcd(0x27, 16, 2);
bool servo= false;
unsigned long lastMsg = 0;



const char* ssid = "DESKTOP-UNPO4BQ 0832";
const char* password = "53790)Hg";

const char* mqtt_server = "e0df0135069d46a8b31a96e05d9613d7.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_user = "threeshold";
const char* mqtt_pass = "HamzaYoussefMohanad3";

const int PWM_CHANNEL = 0;
const int PWM_FREQ = 50;       // 50Hz for servo
const int PWM_RESOLUTION = 16; // 16-bit

Supabase db;
String supabase_url = "https://pisnlmcsvljbmjtnbzko.supabase.co";
String anon_key = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6InBpc25sbWNzdmxqYm1qdG5iemtvIiwicm9sZSI6ImFub24iLCJpYXQiOjE3NTU2Mzg4ODcsImV4cCI6MjA3MTIxNDg4N30.FA-JDaO5dQkNwlbe8E72fqOyN5PVdB5xOvkxGuxXHvU";
String table = "IR";

WiFiClientSecure espClient;
PubSubClient client(espClient);

void writeServo(int angle) {
  int duty = map(angle, 0, 180, 1638, 7864); 
  ledcWrite(PWM_CHANNEL, duty);
}

void insertrow(int IRvalue){
  String JSON = "{\"id\":1, \"value\": " + String(IRvalue) + "}";
  db.insert(table,JSON,false);
}

void updateLCD(const char* msg, int delayTime = 0) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(msg);
  if (delayTime > 0) {
    delay(delayTime);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Threeshold farm");
  }
}

void connectWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");
  db.begin(supabase_url, anon_key);
}

void connectMQTT() {
    while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP32Client",mqtt_user,mqtt_pass)) {
      Serial.println("connected.");
      client.subscribe("buzzer");
      client.subscribe("servo");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 2s...");
      delay(2000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");

  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);

  // If the topic is for the servo, set the angle
  if (String(topic) == "servo") {
    int angle = message.toInt();
    if(angle ==180){
    writeServo(angle);
    digitalWrite(BUZZER_PIN,LOW);
    servo=true;
    Serial.print("Servo moved to: ");
    Serial.println(angle);
    updateLCD("Hello", 5000);
    }
  }

  if(String(topic) == "buzzer"){
    int status = message.toInt();
    if(status == 1){
    digitalWrite(BUZZER_PIN,HIGH);
    }
  }
}

void setup(){
  Serial.begin(115200);
  pinMode(LDR_PIN, INPUT);
  pinMode(IR_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(SERVO_PIN, PWM_CHANNEL);


  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Threeshold farm");
  connectWiFi();
  espClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);


}

void loop(){
  if (!client.connected()) {
    connectMQTT();
  }
  client.loop();

  // قراءة LDR
  int ldrValue = analogRead(LDR_PIN);
  if (ldrValue < 1000) { // نور فصل
    if (servo) {
      digitalWrite(LED_GREEN, HIGH);
      digitalWrite(LED_RED, LOW);
    } else {
      digitalWrite(LED_RED, HIGH);
      digitalWrite(LED_GREEN, LOW);
    }
  }else{
    digitalWrite(LED_RED,LOW);
    digitalWrite(LED_GREEN,LOW);
  }

  // قراءة زرار
  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(200); // Debounce
    if (servo){
      writeServo(0);  // وضعية مقفولة
      servo = false;
      updateLCD("Thanks", 5000);

    }
  }

  if (millis() - lastMsg > 1000) {
    lastMsg = millis();
    int irValue = analogRead(IR_PIN);
    insertrow(irValue);
  }
}
