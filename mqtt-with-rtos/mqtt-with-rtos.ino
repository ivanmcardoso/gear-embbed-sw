#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"

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

/* Region --------------- EventGroups --------------*/
EventGroupHandle_t connectionEventGroup;
const int WIFI_CONNECTED_BIT = BIT0;
const int MQTT_CONNECTED_BIT = BIT1;
/* endregion */

/* Region --------------- Semaphores --------------*/
SemaphoreHandle_t xSemaphore = NULL;
/* endregion */

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  connectionEventGroup = xEventGroupCreate();
  xSemaphore = xSemaphoreCreateMutex();
  // cria tarefas no nucleo 0
  xTaskCreatePinnedToCore(connectWifiTask, "connectWifiTask", 4096, NULL, 4, NULL, 1 );
  xTaskCreatePinnedToCore(connectMqtt, "connectMqtt", 4096, NULL, 3, NULL, 1 );
  xTaskCreatePinnedToCore(mqttLoopTask, "mqttLoopTask", 4096, NULL, 2, NULL, 1 );
  xTaskCreatePinnedToCore(publishDataToMqttTask, "publishDataToMqttTask", 4096, NULL, 2, NULL, 1 );

}

void loop() {}

/* Region --------------- Tasks --------------*/
void connectWifiTask( void * pvParameters ) {
  Serial.println("Task wifi created");
  // Loop infinito até conectar
  setup_wifi();
  // configura conexão sem certificados de segurança
  espClient.setInsecure();
  // sinaliza a conexao de wifi para o event group
  xEventGroupSetBits( connectionEventGroup, WIFI_CONNECTED_BIT);
  // tarefa se auto destroi já que não tem mais utilidade
  vTaskDelete(NULL);
}

void connectMqtt( void * pvParameters ) {
  Serial.println("Task mqtt created");

  // seta a função de callback do subscribe
  client.setCallback(mqtt_callback);

  for (;;) {
    /**
       bloqueia a task até que haja conexao com wifi, não limpa a flag ao ser notificado
      EventBits_t xEventGroupWaitBits(    EventGroupHandle_t xEventGroup,
                                           const EventBits_t uxBitsToWaitFor,
                                           const BaseType_t xClearOnExit,
                                           const BaseType_t xWaitForAllBits,
                                           const TickType_t xTicksToWait );
    */
    xEventGroupWaitBits (connectionEventGroup, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY );
    // Verifica se o client está conectado ao broker
    if (!client.connected()) {
      Serial.println("try reconect..");

      // Se está desconectado limpa bit de conexao para bloquear as tasks dependentes da conexao
      xEventGroupClearBits( connectionEventGroup, MQTT_CONNECTED_BIT);
      // Loop infinito até conectar
      int connStatus = reconnect();
      if (connStatus) {
        Serial.println(connStatus);
        // sinaliza a conexao com o broker para o event group
        xEventGroupSetBits( connectionEventGroup, MQTT_CONNECTED_BIT);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void mqttLoopTask( void * pvParameters ) {
  Serial.println("Task mqtt loop created");


  for (;;)  {
    // bloqueia task até que a conexao wifi e mqtt estejam ativas
    xEventGroupWaitBits (connectionEventGroup, WIFI_CONNECTED_BIT | MQTT_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY );
    // verifica se pode obter o mutex, espera até 100ms por isso
    if (xSemaphoreTake( xSemaphore, pdMS_TO_TICKS(100)) == pdTRUE ) {
      // funçao loop sempre será chamada a tempo de manter a conexão graças ao RTOS :pray_hands:
      client.loop();
      // libera o mutex
      xSemaphoreGive( xSemaphore );
    }
  }
}

void publishDataToMqttTask( void * pvParameters ) {
  Serial.println("Task publish created");
  int state = 0;
  for (;;) {
    // bloqueia task até que a conexao wifi e mqtt estejam ativas
    xEventGroupWaitBits (connectionEventGroup, WIFI_CONNECTED_BIT | MQTT_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY );
    // envia mensagem a cada 10s

    // verifica se pode obter o mutex, espera até 100ms por isso
    if (xSemaphoreTake( xSemaphore, pdMS_TO_TICKS(100)) == pdTRUE ) {
      // logica boba pra fazer o led piscar pelo mqtt
      state = state == 1 ? 0 : 1;
      mqtt_publish_data(state == 1 ? "on" : "off");
      // libera o mutex
      xSemaphoreGive( xSemaphore );
    }
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}
/* endregion */

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

int reconnect() {
  if (client.connected()) return 0;
  Serial.print("Attempting MQTT connection...");
  if (client.connect("esp32-my-cid", mqtt_suid, mqtt_pass)) {
    Serial.println("connected");
    client.subscribe(mqtt_topic);
    return 1;
  } else {
    Serial.print("failed, rc=");
    Serial.print(client.state());
    Serial.println(" try again in 5 seconds");
    return 0;
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

void mqtt_publish_data(const char* message) {
  client.publish(mqtt_topic, message);
}
