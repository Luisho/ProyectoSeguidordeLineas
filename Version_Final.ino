#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PN532.h>
#include "ThingsBoard.h"
#include <WiFi.h>
#include <ArduinoJson.h>
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

#define WIFI_AP             "POCOPHONE"
#define WIFI_PASSWORD       "748e045de1da"
#define TOKEN               "esp32grupo2.2"
#define THINGSBOARD_SERVER  "demo.thingsboard.io"

#define PN532_IRQ   (17)
#define PN532_RESET (16)

#define sensorPin (33)
#define sensorPin2 (32)

int motor1Pin1 = 27; 
int motor2Pin1 = 25; 
int enable1Pin = 14; 
int enable2Pin = 26;

// Setting PWM properties
const int freq = 30000;
const int pwmChannel_1 = 0;
const int pwmChannel_2 = 1;
const int resolution = 8;
int dutyCycle = 160;
int giroCycle = 140;

int status = WL_IDLE_STATUS;

int semaforoNfc = 0;

String destino = "";
char c = 0;
boolean newLine = false;
boolean destinoEncontrado = false;

Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);

WiFiClient espClient;
ThingsBoard tb(espClient);

void setup(){  
    Serial.begin(115200);
    SerialBT.begin("PROMEESP32"); //Bluetooth device name
    Serial.println("Bluetooth activado");
  
    WiFi.begin(WIFI_AP, WIFI_PASSWORD);
    InitWiFi();
  
    // Configurar para leer etiquetas RFID
    nfc.begin();
    nfc.setPassiveActivationRetries(0xFF);
    nfc.SAMConfig();
  
    //Configuración cny70
    pinMode(sensorPin , INPUT);  //definir pin como entrada
    pinMode(sensorPin2, INPUT); //definir pin2 como entrada
  
   //Configuración motores
   // sets the pins as outputs:
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor2Pin1, OUTPUT);
    pinMode(enable1Pin, OUTPUT);
    pinMode(enable2Pin, OUTPUT);
    
    // configure LED PWM functionalitites
    ledcSetup(pwmChannel_1, freq, resolution);
    ledcSetup(pwmChannel_2, freq, resolution);  
    
    // attach the channel to the GPIO to be controlled
    ledcAttachPin(enable1Pin, pwmChannel_1);
    ledcAttachPin(enable2Pin, pwmChannel_2);
   
    // testing
    Serial.print("Testing DC Motor...");  
}

void loop(){
    int value = 0;
    int value2 = 0;
    int pin18 = 0;
    value = digitalRead(sensorPin );  //lectura digital de pin
    value2 = digitalRead(sensorPin2);  //lectura digital de pin2

    //----------------------------------CONEXIÓN THINGSBOARD----------------------------------
        if (WiFi.status() != WL_CONNECTED) { 
            reconnect();
        }
        if (!tb.connected()) {
            // Connect to the ThingsBoard
            Serial.print("Connecting to: ");
            Serial.print(THINGSBOARD_SERVER);
            Serial.print(" with token ");
            Serial.println(TOKEN);
            if (!tb.connect(THINGSBOARD_SERVER, TOKEN)) {
                Serial.println("Failed to connect");
                return;
          }
        }
    
    if(!destinoEncontrado){ //No ha recibido un destino el coche, por lo tanto busca en la "base de datos". 
       semaforoNfc = 0;
       digitalWrite(motor1Pin1, LOW);
       digitalWrite(motor2Pin1, LOW);
       while(SerialBT.available() > 0){
         Serial.println("Esperando destino..."); 
         c = SerialBT.read();
         if(c == '\r')
           continue;
         else if(c == '\n'){
           newLine = true;
           break;  
         }
         else destino = destino+c;
       }
       if(newLine){
         if(destino.equals("5AT4")or destino.equals("PPP0")or destino.equals("1234")){
           Serial.println("DESTINO CORRECTO");
           destinoEncontrado = true;    
         }
         else{
           Serial.println("No existe este destino");
           destino = "";
           newLine = false;
         }
       }
    }
    else{ //El coche tiene un destino al que ir.
        String valorStr = "";
        //Serial.println("Coche activado");
        char x[4] = {'0', '0', '0', '0'};
        String valor;
        tb.sendTelemetryString("DESTINO", "EN CAMINO...");
        //----------------------------------MOTORES + CNY70----------------------------------

        if (value == HIGH and value2 == HIGH){ // Los dos sensores están fuera de la linea => giramos a la derecha del robot
            Serial.println("GIRO DERECHA");
            // GiroDerecha the DC motor
            digitalWrite(motor1Pin1, HIGH);
            digitalWrite(motor2Pin1, HIGH);
            ledcWrite(pwmChannel_1, giroCycle);
            ledcWrite(pwmChannel_2, dutyCycle);
        }
        else if ((value == LOW and value2 == HIGH) or (value == LOW and value2 == LOW)){ // Los dos sensores están dentro de la linea o el central está fuera => giramos a la izquierda del robot
            Serial.println("GIRO IZQUIERDA");
            // GiroIzquierda the DC motor
            digitalWrite(motor1Pin1, HIGH);
            digitalWrite(motor2Pin1, HIGH);
            ledcWrite(pwmChannel_1, dutyCycle);
            ledcWrite(pwmChannel_2, giroCycle);
        }

        else { // Si el sensor central está en la linea y el otro fuera => Mantenemos dirección.
            Serial.println("MANTENER DIRECCION");
            // PA'LANTE the DC motor
            digitalWrite(motor1Pin1, HIGH);
            digitalWrite(motor2Pin1, HIGH);
            ledcWrite(pwmChannel_1, dutyCycle);
            ledcWrite(pwmChannel_2, dutyCycle);
        }
    //----------------------------------LECTURA NFC----------------------------------
        if((devolverClave(x)) and (semaforoNfc==0)){
            int j;
            /*for(j=0; j<sizeof(x); j++){
            Serial.println(x[j]);
            }*/
            valor = String(x);  
            Serial.println("Sending data...");
            tb.sendTelemetryString("KEY", valor.c_str());
            if(destino.equals(valor.c_str())){
                destinoEncontrado = false;
                destino = "";
                newLine = false;
                tb.sendTelemetryString("DESTINO", "ALCANZADO");
            }
            valorStr = valor.c_str();
            valor = "";
            digitalWrite(motor1Pin1, LOW);
            digitalWrite(motor2Pin1, LOW);
            for(j=0; j<4; j++){
                x[j] = '0';
            }
        }
        if((valorStr.equals("5AT4")or valorStr.equals("PPP0")or valorStr.equals("1234"))){
            semaforoNfc = 7;
        }  
        tb.loop();
    }
    if(semaforoNfc > 0)
        semaforoNfc = semaforoNfc - 1;
}
void InitWiFi(){
    Serial.println("Connecting to AP ...");
    // attempt to connect to WiFi network
  
    WiFi.begin(WIFI_AP, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Connected to AP");
}

void reconnect() {
    // Loop until we're reconnected
    status = WiFi.status();
    if ( status != WL_CONNECTED) {
        WiFi.begin(WIFI_AP, WIFI_PASSWORD);
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }
        Serial.println("Connected to AP");
    }
}

boolean devolverClave (char x[4]){
    uint8_t success;
    uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };
    uint8_t uidLength;
  
    success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
    if (success){
        uint8_t keya[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
        success = nfc.mifareclassic_AuthenticateBlock(uid, uidLength, 4, 1, keya); // uint8_t * uid, uint8_t uidLen, uint32_t blockNumber, uint8_t keyNumber, uint8_t * keyData);
        delay(100);
        if (success){
            uint8_t data[16];
            uint8_t A[16];
            uint8_t B[16];
            uint8_t C[4] = { 0xFF, 0xFF, 0xFF, 0xFF};      
            success = nfc.mifareclassic_ReadDataBlock(4, data);
            if (success){
                int j;
                for (j = 1; j < 16; j++) {
                    A[j] = data[j];
                }
                int k;
                for (k = 1; k < 11; k++) {
                    B[k] = data[k];
                }
                int z;
                int cont = 0;
                for (z = 1; z < 16; z++) {
                    if (A[z] != B[z]) {
                        C[cont] = A[z];
                        cont++;
                    }
                }
                int i;
                for(i=0; i<4; i++){
                    x[i] = (char) C[i];
                }
                //valor = String(x);
            }
            else Serial.println("Fallo al leer tarjeta");
        }
        else Serial.println("Fallo autentificar tarjeta");
        return true;
    }
    return false;
}




    
