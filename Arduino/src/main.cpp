#include <arduino.h>
#include <esp8266wifi.h>
#include <math.h>

#include <PubSubClient.h>                          // Envio por MQTT

#include <Adafruit_Sensor.h>                       // Biblioteca DHT Sensor Adafruit 
#include <DHT.h>
#include <DHT_U.h>

#include <ArduinoJson.h>                          // Criar o JSon para envio
#include <time.h>                                 // Enviar a Hora com o Payload

const char* ssid     = "HOME";
const char* password = "12345678";   

#define FLOWPIN 16                                // Pino D0
#define RELAYPIN 14                               // Pino D5 (Recebimento de Mensagens)
 
int wifiStatus;

int timezone = -3 * 3600;                     // Definição da TimeZone
int dst = 0;                                  //

unsigned long previousMillis = 0;             // Variáveis para intervalo de tempo
const long interval = 2000;                   // intervalo
 
//**************************************
 
//__ Configurações para o IBM Watson
 
//**************************************
 
//__ Informações do dispositivo
 
#define DEVICE_TYPE  "smartpump"
#define DEVICE_ID    "flowsensor"

// Variáves para Inscrição de Tópico

#define DEVICE_ID_IN    "pump"                                                              // Id do tópico de entrada
char topicIn[]    = "iot-2/type/" DEVICE_TYPE "/id/" DEVICE_ID_IN "/evt/1-anl/fmt/json";   // Tópico de entrada
 
//__ Informações da conexão com o servidor
 
#define ORG     "4je244"

//__ Dados da API

char authMeth[] = "a-4je244-4vdymohwui";
#define TOKEN   "OoAthjNwopAQ4xqX_X"

//__ Variáveis de conexão com o servidor (Não customizaveis)
 
char host[]   = ORG ".messaging.internetofthings.ibmcloud.com";
char topic[]    = "iot-2/type/" DEVICE_TYPE "/id/" DEVICE_ID "/evt/1-anl/fmt/json";
char token[]    = TOKEN;
char clientId[] = "a:" ORG ":" DEVICE_ID;

//__ Inicia WiFi
 
WiFiClient wifiClient;
PubSubClient client(host, 1883, NULL, wifiClient);

DynamicJsonDocument doc(1024);                    // Variável para Converter o Json recebido



// Variáveis do sensor de Vazão
void ICACHE_RAM_ATTR handleInterrupt();

#define PULSE_PIN D2  //gpio4
#define FLOW_CALIBRATION 8.2 

volatile long pulseCount = 0;
float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;
float totalLitres;
float totalLitresold;
unsigned long oldTime;

///////


///////////////////////////////////////////////////

// Método que atua no valor recebido pelo tópico de entrada
void checkPump(int val){

  Serial.print("Pump: "); 
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

  int pumpValue = doc["d"]["pump"];

  checkPump(pumpValue);

}

// Cálculo da Data Atual
String dataAtual(){
  time_t now = time(nullptr);
  struct tm* p_tm = localtime(&now);
  String Ano      =  String((p_tm->tm_year + 1900));
  String Mes      =  String((p_tm->tm_mon + 1));
  String Dia      =  String((p_tm->tm_mday));
  String Hora     =  String((p_tm->tm_hour));
  String Minuto   =  String((p_tm->tm_min));
  String Segundo;

  if ((p_tm->tm_sec) < 10)  Segundo = Segundo + "0";


  Segundo  =  Segundo + String((p_tm->tm_sec));
  

  
  String datetime = Ano + "-" + Mes + "-" + Dia + "T" 
                + Hora + ":" + Minuto + ":" + Segundo;
  
  return datetime;

}

// Preparar o payload para o envio de dados
String preparePayload (float data){
  
  DynamicJsonDocument doc(1024);
  doc["d"]["flow"]  = data;
  doc["d"]["datetime"] = dataAtual();
  String payload;
  serializeJson(doc, payload);
  return payload;

}
//__ Envia os dados para a cloud


void enviaDado(float dado){
 
  String payload = preparePayload(dado);
  Serial.println(payload);

 //__ Envia o dado
 
  if (client.publish(topic, (char*) payload.c_str())) {
    Serial.println("Publish ok");
  } else {
    Serial.println("Publish failed");
  }
 
}

/// Sensor de Vazão

void ICACHE_RAM_ATTR pulseCounter()
{
  pulseCount++;
}

void flow()
{

  if ((millis() - oldTime) > 1000)   // Only process counters once per second
  {
    detachInterrupt(PULSE_PIN);
    flowRate = ((1000.0 / (millis() - oldTime)) * pulseCount) / FLOW_CALIBRATION;
    oldTime = millis();
    flowMilliLitres = (flowRate / 60) * 1000;
    totalMilliLitres += flowMilliLitres;
    totalLitres = totalLitresold + totalMilliLitres * 0.001;
    unsigned int frac;

    // Print the flow rate for this second in liters / minute
    Serial.print("flowrate: ");
    Serial.print(int(flowRate));  // Print the integer part of the variable

    Serial.print(".");             // Print the decimal point
    frac = (flowRate - int(flowRate)) * 10; // Determine the fractional part. The 10 multiplier gives us 1 decimal place.
    Serial.print(frac, DEC) ;      // Print the fractional part of the variable
    Serial.print("L/min");


    // Serial.print("  Current Liquid Flowing: ");  // Print the number of liters flowed in this second
    // Serial.print(flowMilliLitres);
    // Serial.print("mL/Sec");

    // Serial.print("  Output Liquid Quantity: ");  // Print the cumulative total of liters flowed since starting
    // Serial.print(totalLitres);
    // Serial.println("L");
    //flowRate = roundf(flowRate * 100) / 100; 
    pulseCount = 0;  // Reset the pulse counter so we can start incrementing again

    attachInterrupt(PULSE_PIN, pulseCounter, FALLING);    // Enable the interrupt again now that we've finished sending output
    enviaDado(flowRate);
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

  
  // Define função callback
  client.setCallback(callback);

  // Sensor de Vazão

    pulseCount        = 0;
  flowRate          = 0.0;
  flowMilliLitres   = 0;
  totalMilliLitres  = 0;
  oldTime           = 0;
  totalLitresold = 0;

  pinMode(PULSE_PIN, INPUT);  // Initialization of the variable "PULSE_PIN" as INPUT (D2 pin)

  attachInterrupt(PULSE_PIN, pulseCounter, FALLING);


  // Allow the hardware to sort itself out
  delay(1500);

  
}



 
void loop() {

  //unsigned long currentMillis = millis();
  if (!client.connected()) {
    checkPump(0);
    reconnect();
  }

  client.loop(); 

  flow();

  //__ Le Sensores
  //flow = LeTemperatura();

//   sensors_event_t event;                        // inicializa o evento da Temperatura
//   dht.temperature().getEvent(&event);           // faz a leitura da Temperatura
//   if (isnan(event.temperature))                 // se algum erro na leitura
//   {
//     Serial.println("Erro na leitura da Temperatura!");
//   }

//  else 
//   {
//    flow =  event.temperature;

//    //__ Envia um dado para a cloud
  
//   if (currentMillis - previousMillis >= interval) {
//     previousMillis = currentMillis;
//     enviaDado(flow);
//   }

  //}

}