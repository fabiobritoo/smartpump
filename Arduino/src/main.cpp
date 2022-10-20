#include <arduino.h>
#include <esp8266wifi.h>
#include <math.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>

#include <PubSubClient.h>                          // Envio por MQTT

#include <ArduinoJson.h>                          // Criar o JSon para envio
#include <time.h>                                 // Enviar a Hora com o Payload

const char* ssid     = "HOME";
const char* password = "12345678";   


#define RELAYPIN 14                               // Pino D5 (Recebimento de Mensagens)

#define DHTPIN 4 //Pino digital D2 (GPIO5) conectado ao DHT11 MAYBE16
#define DHTTYPE DHT11 //Tipo do sensor DHT11

DHT dht(DHTPIN, DHTTYPE); //Inicializando o objeto dht do tipo DHT passando como parâmetro o pino (DHTPIN) e o tipo do sensor (DHTTYPE)

float temperatura; //variável para armazenar a temperatura
float umidade; //Variável para armazenar a umidade


int wifiStatus;

int timezone = -3 * 3600;                     // Definição da TimeZone
int dst = 0;                                  //

unsigned long previousMillis = 0;             // Variáveis para intervalo de tempo
const long interval = 2000;                   // intervalo
 
//**************************************
 
//__ Configurações para o IBM Watson
 
//**************************************
 
//__ Informações do dispositivo
 
#define DEVICE_TYPE  "smartroom"
#define DEVICE_ID    "roomcontrol"

// Variáves para Inscrição de Tópico

#define DEVICE_ID_IN    "room"                                                              // Id do tópico de entrada
char topicIn[]    = "room/light/britohome";   // Tópico de entrada
 
//__ Informações da conexão com o servidor
 
#define ORG     "4je244"

//__ Dados da API

char authMeth[] = "a-4je244-4vdymohwui";
#define TOKEN   "OoAthjNwopAQ4xqX_X"

//__ Variáveis de conexão com o servidor (Não customizaveis)
 
char host[]   = "test.mosquitto.org";
char topic[]    = "room/temp/britohome";
char token[]    = TOKEN;
char clientId[] = "a:" ORG ":" DEVICE_ID;

//__ Inicia WiFi
 
WiFiClient wifiClient;
PubSubClient client(host, 1883, NULL, wifiClient);

DynamicJsonDocument doc(1024);                    // Variável para Converter o Json recebido

unsigned long oldTime;

///////////////////////////////////////////////////

// Método que atua no valor recebido pelo tópico de entrada
void checkLight(int val){

  Serial.print("Light: "); 
  Serial.println(val); 

  if (val == 1){
    digitalWrite(RELAYPIN, HIGH);
  } else if( val == 0 ){
    digitalWrite(RELAYPIN, LOW);
  }
 
}

// Função que é chamada quando é recebida mensagem no tópico inscrito
void callback(char* topicIn, byte* payload, unsigned int length) {

  Serial.print("Message arrived [");
  Serial.print(topicIn);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  
  deserializeJson(doc, payload);

  int lightValue = doc["d"]["light"];

  checkLight(lightValue);

}

// Cálculo da Data Atual
String dataAtual(){
  time_t now = time(nullptr);
  struct tm* p_tm = localtime(&now);
  String Ano      =  String((p_tm->tm_year + 1900));
  String Mes;
  String Dia;
  String Hora;
  String Minuto;
  String Segundo;

  if ((p_tm->tm_mon + 1) < 10)  Mes = "0";
  Mes  =  Mes + String((p_tm->tm_mon + 1));

  if ((p_tm->tm_mday) < 10)  Dia ="0";
  Dia  =  Dia + String((p_tm->tm_mday)); 

  if ((p_tm->tm_hour) < 10)  Hora ="0";
  Hora  =  Hora + String((p_tm->tm_hour));

  if ((p_tm->tm_min) < 10)  Minuto ="0";
  Minuto  =  Minuto + String((p_tm->tm_min));

  if ((p_tm->tm_sec) < 10)  Segundo = "0";
  Segundo  =  Segundo + String((p_tm->tm_sec));  

  
  String datetime = Ano + "-" + Mes + "-" + Dia + "T" 
                + Hora + ":" + Minuto + ":" + Segundo;
  
  return datetime;

}

// Preparar o payload para o envio de dados
String preparePayload (float data1, float data2){
  
  DynamicJsonDocument doc(1024);
  doc["d"]["temp"]  = data1;
  doc["d"]["hum"]  = data2;
  doc["d"]["datetime"] = dataAtual();
  String payload;
  serializeJson(doc, payload);
  return payload;

}
//__ Envia os dados para o topico


void enviaDado(float dado1, float dado2){
 
  String payload = preparePayload(dado1, dado2);
  Serial.println(payload);

 //__ Envia o dado
 
  if (client.publish(topic, (char*) payload.c_str())) {
    Serial.println("Publish ok");
  } else {
    Serial.println("Publish failed");
  }
 
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(clientId, authMeth, token)) {
      Serial.println("connected");
      // ... and resubscribe
      client.subscribe(topicIn);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      // Wait 2 seconds before retrying
      delay(2000);
    }
  }
}


void setup() { 

  pinMode(RELAYPIN, OUTPUT);
  dht.begin(); //Inicializa o sensor DHT11

  //__ Inicializa a serial
  
  Serial.begin(9600);
  Serial.println();
  Serial.print("Conectando-se na rede ");
  Serial.print(ssid);
  
  //__ Conecta-se na rede WIFI
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  
  Serial.print("Conectado, endereço de IP: ");
  Serial.println(WiFi.localIP());

  // Configuração da Data

  configTime(timezone, dst, "pool.ntp.org","time.nist.gov");
  Serial.println("\nWaiting for Internet time");

  while(!time(nullptr)){
     Serial.print("*");
     delay(1000);
  }
  Serial.println("\nTime response....OK");  

  // Define função callback - Chamada quando recebe informação no tópico
  client.setCallback(callback); 

  // Allow the hardware to sort itself out
  delay(1500);
  
}

void check_temp() {

  if ((millis() - oldTime) > 1000)   // Only process counters once per second
  { 
    oldTime = millis();
    temperatura = dht.readTemperature();  //Realiza a leitura da temperatura
    umidade = dht.readHumidity(); //Realiza a leitura da umidade
    Serial.print("Temperatura: ");
    Serial.print(temperatura); //Imprime no monitor serial o valor da temperatura lida
    Serial.println(" ºC");
    Serial.print("Umidade: ");
    Serial.print(umidade); //Imprime no monitor serial o valor da umidade lida
    Serial.println(" %");
    enviaDado(temperatura,umidade);
  }
  
}

 
void loop() {

  if (!client.connected()) {
    checkLight(0);
    reconnect();
  }

  client.loop(); 
  check_temp();

}