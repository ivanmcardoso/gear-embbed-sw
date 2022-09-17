#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>

/* Region --------------- PINS --------------*/
const byte ledPin = 2;
/* endregion */

/* region --------------- WIFI --------------*/
const char* ssid = "";
const char* password = "";

WiFiClientSecure espClient;
/* endregion */

/* Region --------------- MQTT --------------*/
const char* mqtt_server = "";
const int mqtt_port = 8883;
const char* mqtt_topic = "";
const char* mqtt_suid =  "";
const char* mqtt_pass = "";

PubSubClient client(mqtt_server, mqtt_port, espClient);
/* endregion */

void setup() {
  Serial.begin(115200);

  setup_wifi();
  espClient.setInsecure();
  client.setCallback(mqtt_callback);

  pinMode(ledPin, OUTPUT);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("esp32-my-cid", mqtt_suid, mqtt_pass)) {
      Serial.println("connected");
      client.subscribe(mqtt_topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void mqtt_callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();
  if (String(topic) == mqtt_topic) {
    Serial.print("Changing output to ");
    if (messageTemp == "on") {
      Serial.println("on");
      digitalWrite(ledPin, HIGH);
    }
    else if (messageTemp == "off") {
      Serial.println("off");
      digitalWrite(ledPin, LOW);
    }
  }
}
