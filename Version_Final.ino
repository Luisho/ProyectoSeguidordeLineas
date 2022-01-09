#include <Arduino.h>
#include <Adafruit_PN532.h>
#include "ThingsBoard.h"
#include <WiFi.h>
#include "BluetoothSerial.h"
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
BluetoothSerial SerialBT;

//-------------------OTA--------------------------------
const char* host = "esp32";
const char* ssid = "POCOPHONE";
const char* password = "748e045de1da";

//--------------------THINGSBOARD----------------------
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
int dutyCycle = 200;
int giroCycle = 0;
int status = WL_IDLE_STATUS;

int semaforoNfc = 0;


char c = 0;
boolean newLine = false;
boolean destinoEncontrado = false;

Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);

WiFiClient espClient;
ThingsBoard tb(espClient);
WebServer server(591);


TaskHandle_t Task2;
boolean movimiento = false;
boolean cambio = true;
String destino = "";

//---------------------------------PAGINA WEB---------------------------------------
const char* loginIndex = 
 "<form name='loginForm'>"
    "<table width='20%' bgcolor='A09F9F' align='center'>"
        "<tr>"
            "<td colspan=2>"
                "<center><font size=4><b>ESP32 Login Page</b></font></center>"
                "<br>"
            "</td>"
            "<br>"
            "<br>"
        "</tr>"
        "<td>Username:</td>"
        "<td><input type='text' size=25 name='userid'><br></td>"
        "</tr>"
        "<br>"
        "<br>"
        "<tr>"
            "<td>Password:</td>"
            "<td><input type='Password' size=25 name='pwd'><br></td>"
            "<br>"
            "<br>"
        "</tr>"
        "<tr>"
            "<td><input type='submit' onclick='check(this.form)' value='Login'></td>"
        "</tr>"
    "</table>"
"</form>"
"<script>"
    "function check(form)"
    "{"
    "if(form.userid.value=='admin' && form.pwd.value=='admin')"
    "{"
    "window.open('/serverIndex')"
    "}"
    "else"
    "{"
    " alert('Error Password or Username')/*displays error message*/"
    "}"
    "}"
"</script>";
 
/*
 * Server Index Page
 */
 
const char* serverIndex = 
"<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
   "<input type='file' name='update'>"
        "<input type='submit' value='Update'>"
    "</form>"
 "<div id='prg'>progress: 0%</div>"
 "<script>"
  "$('form').submit(function(e){"
  "e.preventDefault();"
  "var form = $('#upload_form')[0];"
  "var data = new FormData(form);"
  " $.ajax({"
  "url: '/update',"
  "type: 'POST',"
  "data: data,"
  "contentType: false,"
  "processData:false,"
  "xhr: function() {"
  "var xhr = new window.XMLHttpRequest();"
  "xhr.upload.addEventListener('progress', function(evt) {"
  "if (evt.lengthComputable) {"
  "var per = evt.loaded / evt.total;"
  "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
  "}"
  "}, false);"
  "return xhr;"
  "},"
  "success:function(d, s) {"
  "console.log('success!')" 
 "},"
 "error: function (a, b, c) {"
 "}"
 "});"
 "});"
 "</script>";

//------------------------------USO DEL SEGUNDO PROCESADOR DEL ESP32----------------------------------
void loop2(void *parameter){
    for(;;){
    int value = 0;
    int value2 = 0;
    value = digitalRead(sensorPin );  //lectura digital de pin
    value2 = digitalRead(sensorPin2);  //lectura digital de pin2
    //----------------------------------MOTORES + CNY70----------------------------------
    if(movimiento == true){
        cambio = true;
        if (value == HIGH and value2 == HIGH){ // Los dos sensores están fuera de la linea => giramos a la izquierda del robot
            digitalWrite(motor1Pin1, HIGH);
            digitalWrite(motor2Pin1, HIGH);
            ledcWrite(pwmChannel_1, giroCycle);
            ledcWrite(pwmChannel_2, dutyCycle);
        }
        else if ((value == LOW and value2 == HIGH) or (value == LOW and value2 == LOW)){ // Los dos sensores están dentro de la linea o el central está fuera => giramos a la derecha del robot
            digitalWrite(motor1Pin1, HIGH);
            digitalWrite(motor2Pin1, HIGH);
            ledcWrite(pwmChannel_1, dutyCycle);
            ledcWrite(pwmChannel_2, giroCycle);
        }

        else { // Si el sensor central está en la linea y el otro fuera => Mantenemos dirección.
            digitalWrite(motor1Pin1, HIGH);
            digitalWrite(motor2Pin1, HIGH);
            ledcWrite(pwmChannel_1, dutyCycle);
            ledcWrite(pwmChannel_2, dutyCycle);
        }
  }
    else if(cambio){ //Dispositivo en parada
        digitalWrite(motor1Pin1, LOW);
        digitalWrite(motor2Pin1, LOW);
        cambio = false;
    }
    vTaskDelay(10);
  }
}
void setup() {
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

 //Configuración OTA
  if (!MDNS.begin(host)) { //http://esp32.local
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");
  //return index page which is stored in serverIndex 
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", loginIndex);
  });
  server.on("/serverIndex", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex);
  });
  //handling uploading firmware file 
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      // flashing firmware to ESP
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });
  server.begin();

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
          Serial.println(ssid);
          Serial.print("IP address: ");
          Serial.println(WiFi.localIP());
      }

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
 
  // set up loop2 
  xTaskCreatePinnedToCore(loop2,"Task_2",4096,NULL,1,&Task2,0);
}

void loop() {
    if(!destinoEncontrado){ //No ha recibido un destino el coche, por lo tanto busca en la "base de datos". 
       semaforoNfc = 0;
       movimiento = false;
       server.handleClient();
       while(SerialBT.available() > 0){
         Serial.println("Esperando destino..."); 
         tb.sendTelemetryString("DESTINO", "ESPERANDO DESTINO...");  
         tb.sendTelemetryString("PARADA", "NULL");
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
       server.close();
        String valorStr = "";
        movimiento = true;
        Serial.println("Coche activado");
        char x[4] = {'0', '0', '0', '0'};
        String valor;
        tb.sendTelemetryString("PARADA", destino.c_str());
        tb.sendTelemetryString("DESTINO", "EN CAMINO...");
        
    //----------------------------------LECTURA NFC----------------------------------
        if((devolverClave(x)) and (semaforoNfc==0)){
            int j;
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
