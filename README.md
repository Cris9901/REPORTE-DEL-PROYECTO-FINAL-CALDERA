# REPORTE-DEL-PROYECTO-FINAL-CALDERA

Dentro de este repositorio se muestra la manera de programar una ESP32 con el sensor DHT11, el sensor Ultrasonico y además, que los datos obtenidos se vean reflejados en el programa Node-RED a través de una conexión WIFI y asi simular el funcionamiento de una caldera.

## Introducción

Este proyecto propone una forma de simular el funcionamiento de una caldera mediante dos sensores y un microcontrolador, la programacion del código nos permite simular y medir el nivel del agua dentro de la caldera, asi como tambien la temperatura que va subiendo hasta llegar al punto máximo de calor permitido, una vez que llega la temperatura al máximo establecido, ésta se apaga automaticamente. Es importante realizar todo lo indicado en este repositorio para poder aprovechar este sistema y realizar diversas tareas; como lo es el poder monitoriar de manera eficaz la productividad de una caldera en cualquier lugar donde se encuentre.

### Descripción

La ```Esp32``` la utilizamos en un entorno de adquision de datos, lo cual en esta practica ocuparemos un sensor ```DTH11``` para adquirir la temperatura y la humedad dentro de la caldera, el sensor ```HC-SR04``` para el registro del nivel del agua. Tambien utilizaremos el programa ```Node-red``` para trabajar de manera visual y esquematica la programación grafica del flujo de la información; podremos observar los resultados de la programación y simulación a través de la interfaz *HMI* de ```Node-Red```; cabe aclarar que en esta practica se usara un simulador llamado [WOKWI](https://https://wokwi.com/) y un programa llamado [Node-RED](http://localhost:1880/).

## Material Necesario

Para realizar esta practica se necesita lo siguiente:
- Simulador [WOKWI](https://https://wokwi.com/)
- Programa [Node-RED](http://localhost:1880/)
- Tarjeta ESP32
- Sensor de temperatura y humedad DHT11
- Sensor Ultrasonico HC-SR04

## Requisitos previos

Para poder usar este repositorio necesitas entrar a la plataforma [WOKWI](https://https://wokwi.com/) y tener instalado correctamente el programa [Node-RED](http://localhost:1880/).

## Instrucciones

1. Abrir la terminal de programación y colocar la siguente programación:

```
#include <ArduinoJson.h>
#include <WiFi.h>
#include <PubSubClient.h>
#define BUILTIN_LED 2
#include "DHTesp.h"
const int DHT_PIN = 15;
const int Trigger = 4;   //Pin digital 2 para el Trigger del sensor
const int Echo = 2;   //Pin digital 3 para el Echo del sensor
DHTesp dhtSensor;
String OFF;
String ON;
// Update these with values suitable for your network.

const char* ssid = "Wokwi-GUEST";
const char* password = "";
const char* mqtt_server = "18.193.219.109";
String username_mqtt="educatronicosiot";
String password_mqtt="12345678";

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   
    // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  
    // Turn the LED off by making the voltage HIGH
  }

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), username_mqtt.c_str() , password_mqtt.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);
  pinMode(Trigger, OUTPUT); //pin como salida
  pinMode(Echo, INPUT);  //pin como entrada
  digitalWrite(Trigger, LOW);//Inicializamos el pin con 0
}

void loop() {


delay(1000);
TempAndHumidity  data = dhtSensor.getTempAndHumidity();
long t; //timepo que demora en llegar el eco
long d; //distancia en centimetros

digitalWrite(Trigger, HIGH);
delayMicroseconds(10);          //Enviamos un pulso de 10us
digitalWrite(Trigger, LOW);
  
t = pulseIn(Echo, HIGH); //obtenemos el ancho del pulso
d = t/59;             //escalamos el tiempo a una distancia en cm
  
Serial.print("Distancia: ");
Serial.print(d);      //Enviamos serialmente el valor de la distancia
Serial.print("cm");
Serial.println();
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
    //++value;
    //snprintf (msg, MSG_BUFFER_SIZE, "hello world #%ld", value);

    StaticJsonDocument<128> doc;

    

    if (dhtSensor.getTemperature() < 50)
{
  doc["DEVICE"] = "ESP32";
    //doc["Anho"] = 2022;
    doc["DISTANCIA"] = String(d);
    doc["TEMPERATURA"] = String(data.temperature, 1);
    doc["HUMEDAD"] = String(data.humidity, 1);
    doc["CALDERA"] = "ON";
}
else
if (dhtSensor.getTemperature() >= 50)
{
   doc["DEVICE"] = "ESP32";
    //doc["Anho"] = 2022;
    doc["DISTANCIA"] = String(d);
    doc["TEMPERATURA"] = String(data.temperature, 1);
    doc["HUMEDAD"] = String(data.humidity, 1);
    doc["CALDERA"] = "OFF";
}


    String output;
    
    serializeJson(doc, output);

    Serial.print("Publish message: ");
    Serial.println(output);
    Serial.println(output.c_str());
    client.publish("PROYECTOFINAL", output.c_str());
  }
}
```

2. Abrir el simulador [WOKWI](https://https://wokwi.com/) e instalar las librerias de **DHT sensor library for ESPx**, **PubSubClient** y **ArduinoJson**  como se muestra en la siguente imagen.

![](https://github.com/Cris9901/REPORTE-DEL-PROYECTO-FINAL-CALDERA/blob/main/IMAGEN%201.jpg)

3. Colocar y hacer la conexion del sensor **DHT11** con la tarjeta **ESP32** como se muestra en la siguente imagen.

![](https://github.com/Cris9901/REPORTE-DEL-PROYECTO-FINAL-CALDERA/blob/main/IMAGEN%202.jpg)

4. Insertar y hacer la conexion del sensor ultrasónico **HC-SR04** con la tarjeta **ESP32** como se muestra en la siguente imagen.

![](https://github.com/Cris9901/REPORTE-DEL-PROYECTO-FINAL-CALDERA/blob/main/IMAGEN%203.jpg)

5. Abrir el programa de **Node-RED** e instalar los nodos ```node-red-dashboard``` y ```node-red-node-mysql```, que se encuentran en la opcion de **Manage palette**.

![](https://github.com/Cris9901/REPORTE-DEL-PROYECTO-FINAL-CALDERA/blob/main/IMAGEN%204.jpg)

6. Colocar el bloque de ```mqtt in```.

![](https://github.com/Cris9901/REPORTE-DEL-PROYECTO-FINAL-CALDERA/blob/main/IMAGEN%205.jpg)

7. Configurar el bloque ```mqtt in``` en el puerto del servidor con el ip ```18.193.219.109``` y el *topic* ```PROYECTOFINAL``` como se muestra en la imagen.

![](https://github.com/Cris9901/REPORTE-DEL-PROYECTO-FINAL-CALDERA/blob/main/IMAGEN%206.jpg)

8. Colocar el bloque ```json```.

![](https://github.com/Cris9901/REPORTE-DEL-PROYECTO-FINAL-CALDERA/blob/main/IMAGEN%207.jpg)

9. Configurar el bloque con la acción de ```Always convert to JavaScript Object```  como se muestra en la imagen.

![](https://github.com/Cris9901/REPORTE-DEL-PROYECTO-FINAL-CALDERA/blob/main/IMAGEN%208.jpg)

10. Colocar cuatro bloques ```function```.

![](https://github.com/Cris9901/REPORTE-DEL-PROYECTO-FINAL-CALDERA/blob/main/IMAGEN%209.jpg)

11. Los configuramos con el siguiente codigo, dependiendo de cada funcion.

**CALDERA**

```
msg.payload = msg.payload.CALDERA;
msg.topic = "CALDERA";
return msg;
```

![](https://github.com/Cris9901/REPORTE-DEL-PROYECTO-FINAL-CALDERA/blob/main/IMAGEN%2010.jpg)

**TEMPERATURA**

```
msg.payload = msg.payload.TEMPERATURA;
msg.topic = "TEMPERATURA";
return msg;
```

![](https://github.com/Cris9901/REPORTE-DEL-PROYECTO-FINAL-CALDERA/blob/main/IMAGEN%2011.jpg)

**HUMEDAD**

```
msg.payload = msg.payload.HUMEDAD;
msg.topic = "HUMEDAD";
return msg;
```

![](https://github.com/Cris9901/REPORTE-DEL-PROYECTO-FINAL-CALDERA/blob/main/IMAGEN%2012.jpg)

**NIVEL**

```
msg.payload = msg.payload.DISTANCIA;
msg.topic = "DISTANCIA";
return msg;
```

![](https://github.com/Cris9901/REPORTE-DEL-PROYECTO-FINAL-CALDERA/blob/main/IMAGEN%2013.jpg)

11. Colocamos un bloque ```Guage``` a cada una de las funciones de **TEMPERATURA**, **HUMEDAD** y **DISTANCIA**.

![](https://github.com/Cris9901/REPORTE-DEL-PROYECTO-FINAL-CALDERA/blob/main/IMAGEN%2014.jpg)

12. Los configuramos de la siguiente manera, dependiendo de cada funcion.

**TEMPERATURA**

![](https://github.com/Cris9901/REPORTE-DEL-PROYECTO-FINAL-CALDERA/blob/main/IMAGEN%2015.jpg)

**HUMEDAD**

![](https://github.com/Cris9901/REPORTE-DEL-PROYECTO-FINAL-CALDERA/blob/main/IMAGEN%2016.jpg)

**NIVEL**

![](https://github.com/Cris9901/REPORTE-DEL-PROYECTO-FINAL-CALDERA/blob/main/IMAGEN%2017.jpg)

13. Colocamos un bloque ```text``` a la funcion de **CALDERA**.

![](https://github.com/Cris9901/REPORTE-DEL-PROYECTO-FINAL-CALDERA/blob/main/IMAGEN%2018.jpg)

14. Configuramos el bloque ```text``` como en la foto.

![](https://github.com/Cris9901/REPORTE-DEL-PROYECTO-FINAL-CALDERA/blob/main/IMAGEN%2019.jpg)

15. Colocamos un bloque ```chart``` a la funcion de **NIVEL**.

![](https://github.com/Cris9901/REPORTE-DEL-PROYECTO-FINAL-CALDERA/blob/main/IMAGEN%2020.jpg)

16. Configuramos el bloque ```chart``` como en la foto.

![](https://github.com/Cris9901/REPORTE-DEL-PROYECTO-FINAL-CALDERA/blob/main/IMAGEN%2021.jpg)

17. Conectamos todas las funciones de la siguiente manera.

![](https://github.com/Cris9901/REPORTE-DEL-PROYECTO-FINAL-CALDERA/blob/main/IMAGEN%2022.jpg)

18. Por ultimo, en la pestaña de *Layout* crearemos otro tabulador llamado **PROYECTO FINAL**, dentro de el añadiremos tres grupos uno para los indicadores, otro para las graficas y el ultimo para el texto; de igual manera colocaremos espaciadores de temperatura, humedad, nivel y texto, los colocaremos segun sea el caso y la especificación.

![](https://github.com/Cris9901/REPORTE-DEL-PROYECTO-FINAL-CALDERA/blob/main/IMAGEN%2023.jpg)

### Instrucciónes de operación

1. Iniciar simulador en [WOKWI](https://https://wokwi.com/).
2. Visualizar los datos en el monitor serial.
3. Colocar la temperatura y humedad dando *doble click* al sensor **DHT11**.
4. Colocar la distancia dando *doble click* al sensor **HC-SR04**.
5. Iniciar el simulador en [Node-RED](http://localhost:1880/) dando *click izquierdo* en el botón **Deploy** y despues abrir la interfaz dando *click izquierdo* en el boton de exportar.
6. Visualizar la interfaz.

## Resultados

Cuando haya funcionado verás los valores dentro del monitor serial y la interfaz como se muestra en las siguentes imagenes.

Resultados en **Node-red**

![](https://github.com/Cris9901/REPORTE-DEL-PROYECTO-FINAL-CALDERA/blob/main/IMAGEN%2025.jpg)

![](https://github.com/Cris9901/REPORTE-DEL-PROYECTO-FINAL-CALDERA/blob/main/IMAGEN%2026.jpg)

![](https://github.com/Cris9901/REPORTE-DEL-PROYECTO-FINAL-CALDERA/blob/main/IMAGEN%2027.jpg)

![](https://github.com/Cris9901/REPORTE-DEL-PROYECTO-FINAL-CALDERA/blob/main/IMAGEN%2028.jpg)

Resultados en **WOKWI**

![](https://github.com/Cris9901/REPORTE-DEL-PROYECTO-FINAL-CALDERA/blob/main/IMAGEN%2024.jpg)

## Conclusión



# Créditos

## Desarrollado por:

- Ing. Cristian Montañez Mejia. [GITHUB](https://github.com/Cris9901)
- Ing. Roberto Miguel Patiño Lozano. [GITHUB](https://github.com/robertopatino42)




























