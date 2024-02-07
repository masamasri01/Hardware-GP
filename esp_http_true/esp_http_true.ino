#include <ESPAsyncWebSrv.h>
#include "WiFi.h"

#define RXp2 16
#define TXp2 17

// const char *ssid = "MASA's LAPTOP";
// const char *password = "masa1234";
const char *ssid = "rahaf";
const char *password = "rahaf1234";

AsyncWebServer server(80);

float temperature = 0;
float humidity = 0;
int waterSensorData = 0;
int smokeSensorData = 0;
int currentCell = 0;


void handleNavigation(AsyncWebServerRequest *request) {
  // Extract the cell index from the query parameter
  if (request->hasParam("cell")) {
    String cellIndexStr = request->getParam("cell")->value();
    int cellIndex = cellIndexStr.toInt();
    
    // Here, you can add your navigation logic based on the cell index
    // For demonstration, let's just print the received cell index
    Serial2.print("NavigationCommand:");
    Serial2.println(cellIndex);
    
    request->send(200, "text/plain", "Navigation command received");
  } else {
    request->send(400, "text/plain", "Missing cell parameter");
  }
}


void handleLoc(AsyncWebServerRequest *request) {
    Serial2.println("LocCommand");
    request->send(200, "text/plain", String(currentCell));
  
}

void setup() {
    Serial.begin(115200);
  //  Serial.print("iiiiiiiiiiii");
    Serial2.begin(9600);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  WiFi.begin(ssid, password);
  Serial.println(WiFi.localIP());
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi :)");
 server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hello, world");
  });
 server.on("/navigate", HTTP_GET, handleNavigation);
   //server.on("/water", HTTP_GET, handleWaterSensor);
  server.on("/locate", HTTP_GET, handleLoc);
  server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", String(temperature));
  });

  server.on("/humidity", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", String(humidity));
  });

  server.on("/water", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", String(waterSensorData));
  });

  server.on("/smoke", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", String(smokeSensorData));
  });
  server.on("/cell", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", String(currentCell));
  });
Serial.println(WiFi.localIP());Serial.println(WiFi.localIP());Serial.println(WiFi.localIP());Serial.println(WiFi.localIP());

  server.begin();
}

int c=0;
int cellNumberToMobile =-1;
void loop() {
  c++;
  int n = WiFi.scanNetworks();
  if (n == 0) {
      Serial2.println("No networks found");
  } else {
      Serial2.print(n);
      Serial2.println(" networks found");
      Serial2.println("Nr | SSID | RSSI");

      for (int i = 0; i < n; ++i) {
        if(strstr(WiFi.SSID(i).c_str(), "ESPap1")||strstr(WiFi.SSID(i).c_str(), "ESPap2")||strstr(WiFi.SSID(i).c_str(), "ESPap3")){
          Serial2.printf("%2d | %-32.32s | %4d\n", i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i));
          delay(10);
        }
      }
  }
  Serial2.println("");
  //Serial.println(WiFi.localIP());
   if (Serial2.available() > 0) {
    // Read data from Arduino until newline character
    String line = Serial2.readStringUntil('\n');
//Serial.println("yes"); 
//Serial.println(line);
    // Check the keyword and extract the corresponding value
    if (line.startsWith("Temperature:")) {
       temperature = line.substring(12).toFloat();
    //  Serial.print("Temperature: ");
  //    Serial.println(temperature);
    } else if (line.startsWith("Humidity:")) {
       humidity = line.substring(9).toFloat();
   ///   Serial.print("Humidity: ");
  //    Serial.println(humidity);
    } else if (line.startsWith("Water Sensor:")) {
       waterSensorData = line.substring(13).toInt();
 ///     Serial.print("Water Sensor: ");
  //    Serial.println(waterSensorData);
    } else if (line.startsWith("Smoke Sensor:")) {
       smokeSensorData = line.substring(13).toInt();
  //    Serial.print("Smoke Sensor: ");
  //    Serial.println(smokeSensorData);
    }

   else  if (line.startsWith("Current cell:")) {
       currentCell = line.substring(13).toInt();
      
     Serial.print("currentCell: ");
     Serial.println(currentCell);
    }
 }

  delay(1000);
  }
