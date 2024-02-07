////this is the code to consider
#include <Wire.h>
#include <Keypad.h>
#include <LiquidCrystal.h>
#include <MPU6050_light.h>
#include <NewPing.h>  // Include the NewPing library for ultrasonic sensor
#include <DHT22.h>
#define TRIGGER_PIN 6
#define ECHO_PIN 7
#define MAX_DISTANCE 200 // Maximum distance to measure in centimeters

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing object

const int LED_PIN=8;

#define DHTPIN 9        // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11   // Change to DHT22 or DHT21 if you have a different sensor

const int smokeSensorPin = A1;
const int waterSensorPin = A0;  // Analog pin connected to the water sensor

 
  int BUZZER_PIN=11;
#define pinDATA 9 // SDA, or almost any other I/O pin
DHT22 dht22(pinDATA);


/////////////////////////////////////////////////////navigation variables and functions//////////////////////////////////////////
const int rows = 3;
const int cols = 3;
const int inf = 9999; // Infinity
MPU6050 mpu(Wire);

const int leftMotorPin1 = 2;   
const int leftMotorPin2 = 3;   
const int rightMotorPin1 = 4;  
const int rightMotorPin2 = 5;

int grid[rows][cols] = {
  {0, 1, 2},
  {3, 4, 5},
  {6, 7, 8}
};

// Define directions
const int NORTH = 0;
const int EAST = 1;
const int SOUTH = 2;
const int WEST = 3;

// Assume initial direction is "North"
int currentDirection;

void moveForward() {
  // analogWrite(leftMotorPin1, 255);
  // analogWrite(leftMotorPin2, LOW);
  // analogWrite(rightMotorPin1, 255);
  // analogWrite(rightMotorPin2, LOW);
  // // if(size!=0 && (path[i-1]==TURN_LEFT ||path[i-1]==TURN_RIGHT)) {delay(790);}
  // // else {delay(835);}
  // delay(510);
  // Additional code to control the motors as needed
   unsigned long startTime = millis();
   int duration =510;
  while (millis() - startTime <duration ) {
    analogWrite(leftMotorPin1, 255);
    analogWrite(leftMotorPin2, LOW);
    analogWrite(rightMotorPin1, 255);
    analogWrite(rightMotorPin2, LOW);
    
  //  Check for obstacle detection while moving forward
    // if ((sonar.ping_cm()) < 25 & (sonar.ping_cm() > 0 )) { // Adjust threshold distance as needed
    //   stopMotors();
    //   delay(100);
    //  duration+=100;
    //   digitalWrite( LED_PIN  , HIGH);
    // //  break;
    // }
    //  digitalWrite( LED_PIN  , LOW);
   }
}

void turnLeft() {
  int initialOrientation = mpu.getAngleZ();
  int targetOrientation = initialOrientation + 90;
  while (mpu.getAngleZ() < targetOrientation) {
    mpu.update();
    //Serial.println(mpu.getAngleZ());
    analogWrite(leftMotorPin1, LOW);
    analogWrite(leftMotorPin2, 155);
    analogWrite(rightMotorPin1, 155);
    analogWrite(rightMotorPin2, LOW);
  }
}

void turnRight() {
  int initialOrientation = mpu.getAngleZ();
  int targetOrientation = initialOrientation - 90;
  while (mpu.getAngleZ() > targetOrientation) {
    mpu.update();
    //Serial.println(abs(mpu.getAngleZ()));
    analogWrite(leftMotorPin1, 155);
    analogWrite(leftMotorPin2, LOW);
    analogWrite(rightMotorPin1, LOW);
    analogWrite(rightMotorPin2, 155);
    }
}

void stopMotors() {
  analogWrite(leftMotorPin1, 0);
  analogWrite(leftMotorPin2, 0);
  analogWrite(rightMotorPin1, 0);
  analogWrite(rightMotorPin2, 0);
}
bool isValid(int x, int y) {
  return (x >= 0 && x < rows && y >= 0 && y < cols);
}

int dijkstra(int startGrid, int targetGrid, int distance[], int parent[], int shortestPath[]) {
  const int MAX_PATH_LENGTH = rows * cols;

  bool visited[rows * cols] = {false};

  for (int i = 0; i < rows * cols; ++i) {
    distance[i] = inf;
    parent[i] = -1;
  }

  distance[startGrid] = 0;

  for (int count = 0; count < rows * cols - 1; ++count) {
    int u = -1;
    // Find the vertex with the minimum distance value
    for (int v = 0; v < rows * cols; ++v) {
      if (!visited[v] && (u == -1 || distance[v] < distance[u])) {
        u = v;
      }
    }

    visited[u] = true;

    int uRow = u / cols;
    int uCol = u % cols;

    // Update distance value of the adjacent vertices with opposite directions
    int dx[] = {1, -1, 0, 0};
    int dy[] = {0, 0, 1, -1};
    for (int i = 0; i < 4; ++i) {
      int vRow = uRow + dx[i];
      int vCol = uCol + dy[i];
      int v = vRow * cols + vCol;

      if (isValid(vRow, vCol) && !visited[v] && distance[u] + 1 < distance[v]) {
        distance[v] = distance[u] + 1;
        parent[v] = u;
      }
    }
  }

  // Build the path
  int pathIndex = 0;
  int currentGrid = targetGrid;
  while (currentGrid != -1 && pathIndex < MAX_PATH_LENGTH) {
    shortestPath[pathIndex] = currentGrid;
    currentGrid = parent[currentGrid];
    pathIndex++;
  }

  // Reverse the path to get it from start to target
  for (int i = 0; i < pathIndex / 2; ++i) {
    int temp = shortestPath[i];
    shortestPath[i] = shortestPath[pathIndex - i - 1];
    shortestPath[pathIndex - i - 1] = temp;
  }

  return pathIndex;
}
void printPath(int path[], int pathLength) {
  Serial.print("Shortest Path: ");
  for (int i = 0; i < pathLength; ++i) {
    Serial.print(path[i]);
    Serial.print(" ");
  }
  Serial.println();
}
void convertPathToMovements(int path[],int length) {
  for (int i = 0; i < length-1; ++i) {
    int currentGrid = path[i];
    int nextGrid = path[i + 1];

    int currentRow = currentGrid / cols;
    int currentCol = currentGrid % cols;
    int nextRow = nextGrid / cols;
    int nextCol = nextGrid % cols;
    //Serial.println(currentDirection);

    // Determine changes in direction based on the difference in coordinates
    int rowDiff = currentRow - nextRow;
    int colDiff = currentCol - nextCol;
 
    switch(currentDirection) {
      case NORTH: {
        if(colDiff==1){
          Serial.println("Turn Left");
          turnLeft();
          currentDirection = (currentDirection + 3) % 4;
        }
        else if(colDiff==-1){
          Serial.println("Turn Right");
          turnRight();
          currentDirection = (currentDirection + 1) % 4;
        }
        else if(rowDiff==1){
          Serial.println("No adjustments");
        }
        else if(rowDiff==-1){
          Serial.println("Turn Right");
          Serial.println("Turn Right");
          turnRight();
          turnRight();
          currentDirection = (currentDirection + 1) % 4;
          currentDirection = (currentDirection + 1) % 4;
        }
        break;
      }
      case SOUTH: {
        if(colDiff==-1){
          Serial.println("Turn Left");
          turnLeft();
          currentDirection = (currentDirection + 3) % 4;
        }
        else if(colDiff==1){
          Serial.println("Turn Right");
          turnRight();
          currentDirection = (currentDirection + 1) % 4;
        }
        else if(rowDiff==-1){
          Serial.println("No adjustments");
        }
        else if(rowDiff==1){
          Serial.println("Turn Right");
          Serial.println("Turn Right");
          turnRight();
          turnRight();
          currentDirection = (currentDirection + 1) % 4;
          currentDirection = (currentDirection + 1) % 4;
        }
        break;
      }
      case EAST: {
        if(rowDiff==1){
          Serial.println("Turn Left");
          turnLeft();
          currentDirection = (currentDirection + 3) % 4;
        }
        else if(rowDiff==-1){
          Serial.println("Turn Right");
          turnRight();
          currentDirection = (currentDirection + 1) % 4;
        }
        else if(colDiff==-1){
          Serial.println("No adjustments");
        }
        else if(colDiff==1){
          Serial.println("Turn Right");
          Serial.println("Turn Right");
          turnRight();
          turnRight();
          currentDirection = (currentDirection + 1) % 4;
          currentDirection = (currentDirection + 1) % 4;
        }
        break;
      }
      case WEST: {
        if(rowDiff==-1){
          Serial.println("Turn Left");
          turnLeft();
          currentDirection = (currentDirection + 3) % 4;
        }
        else if(rowDiff==1){
          Serial.println("Turn Right");
          turnRight();
          currentDirection = (currentDirection + 1) % 4;
        }
        else if(colDiff==1){
          Serial.println("No adjustments");
        }
        else if(colDiff==-1){
          turnRight();
          turnRight();
          Serial.println("Turn Right");
          Serial.println("Turn Right");
          currentDirection = (currentDirection + 1) % 4;
          currentDirection = (currentDirection + 1) % 4;
        }
        break;
      }
    }

    //Serial.println(currentDirection);
    moveForward();
    Serial.println("Move Forward");



  }
  stopMotors();

}
/////////////////////////////////////////////////////////////end//////////////////////////////////////////////
bool navigationCommandFlag=false;
int currentGrid=1;
int targetGridFromMobile;
int cellNumberToMobile =0;
const int rs = 37, en = 36, d4 = 35, d5 = 34, d6 = 33, d7 = 32;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Define the keypad parameters
const byte ROW_NUM    = 4; // number of rows on the keypad
const byte COLUMN_NUM = 4; // number of columns on the keypad

char keys[ROW_NUM][COLUMN_NUM] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

byte pin_rows[ROW_NUM] = {22, 23, 24, 25}; //connect to the row pinouts of the keypad
byte pin_column[COLUMN_NUM] = {26, 27, 28, 29}; //connect to the column pinouts of the keypad

Keypad keypad = Keypad( makeKeymap(keys), pin_rows, pin_column, ROW_NUM, COLUMN_NUM);
const int MAX_READINGS = 1; 
// Variables for AP1
String ap1SSID;
int ap1RSSI[MAX_READINGS];
int ap1ReadingCount = 0;
int ap1RSSIavg=0;

// Variables for AP2
String ap2SSID;
int ap2RSSI[MAX_READINGS];
int ap2ReadingCount = 0;
int ap2RSSIavg=0;

// Variables for AP3
String ap3SSID;
int ap3RSSI[MAX_READINGS];
int ap3ReadingCount = 0;
int ap3RSSIavg=0;

int printLoc=0;
int printLocLCD=0;

  int startGrid=0;
  int targetGrid=0;


void setup() {
  //motors setup
   currentDirection = NORTH;
  pinMode(LED_PIN, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  //serial
  Serial.begin(9600, SERIAL_8N1);
  //mpu setup
  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(100);
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
  //lcd
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Press key 1 to");
  lcd.setCursor(0, 1);
  lcd.print("localize:");
  delay(1500);
  lcd.setCursor(0, 0);
  lcd.print("Press key 2 to");
  lcd.setCursor(0, 1);
  lcd.print("navigate:");
  delay(1500);
  lcd.setCursor(0, 0);
  lcd.print("Press key 3 to");
  lcd.setCursor(0, 1);
  lcd.print("sensor values:");
  delay(1500);
}

void loop() {
//digitalWrite( LED_PIN  , LOW);
    //Serial.println(mpu.getAngleZ());
  //  delay(500);
  float temperature = dht22.getTemperature();
  float humidity = dht22.getHumidity();
  int smokeSensorValue = analogRead(smokeSensorPin);
  int waterSensorValue = analogRead(waterSensorPin);
  
  if (Serial.available() > 0) {
   // Serial.println("yesss");
    String input = Serial.readStringUntil('\n');
 //Serial.println(input);
      if (input.indexOf("NavigationCommand:") != -1){
      String cellidx = input.substring (18);
      int cellInt =cellidx.toInt();
      cellInt=cellInt-1;
       targetGridFromMobile = cellInt;
      navigationCommandFlag=true;
      Serial.print("recieved from esp, cell: ");
      Serial.println(cellidx);
    }
       else  if (input.indexOf("LocCommand") != -1){
        Serial.print("Current cell:");
  Serial.println((cellNumberToMobile));
     printLoc=true;
    
    }
  else  if (input.indexOf("ESPap1") != -1) {
      ap1SSID = getValue(input, '|', 1);
      ap1RSSI[ap1ReadingCount] = getValue(input, '|', 2).toInt();
      ap1ReadingCount++;

      if (ap1ReadingCount >= MAX_READINGS) {
        printAverage("AP1", ap1SSID, ap1RSSI, ap1ReadingCount);
        ap1ReadingCount = 0;
      }
    } else if (input.indexOf("ESPap2") != -1) {
      ap2SSID = getValue(input, '|', 1);
      ap2RSSI[ap2ReadingCount] = getValue(input, '|', 2).toInt();
      ap2ReadingCount++;

      if (ap2ReadingCount >= MAX_READINGS) {
        printAverage("AP2", ap2SSID, ap2RSSI, ap2ReadingCount);
        ap2ReadingCount = 0;
      }
    } else if (input.indexOf("ESPap3") != -1) {
      ap3SSID = getValue(input, '|', 1);
      ap3RSSI[ap3ReadingCount] = getValue(input, '|', 2).toInt();
      ap3ReadingCount++;

      if (ap3ReadingCount >= MAX_READINGS) {
        printAverage("AP3", ap3SSID, ap3RSSI, ap3ReadingCount);
        ap3ReadingCount = 0;
      }
    }
    
Serial.print("Temperature:");
Serial.println(temperature);
Serial.print("Humidity:");
Serial.println(humidity);
Serial.print("Water Sensor:");
Serial.println(waterSensorValue);
Serial.print("Smoke Sensor:");
Serial.println(smokeSensorValue);
 Serial.print("Current cell:");
  Serial.println((cellNumberToMobile));

// Serial.print("Current cell:");
// Serial.println(calculatedGridNo);
  }

  // if (waterSensorValue > 800) {
  //   lcd.clear();
  //   lcd.setCursor(0, 0);
  //   lcd.print(" High Water Level! ");
  // }

  // // Check if smoke is detected
  if (smokeSensorValue >800) {
    digitalWrite(BUZZER_PIN, HIGH);  // Turn on the LED
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(" Detected Smoke! ");
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);  // Turn on the LED

  }

   // int distance = sonar.ping_cm();
///     Serial.println(distance);
    //  delay(5000);
    // if (distance < 10) {
    //   stopMotors();
    //   delay(1000);
    // }
  
  //mpu.update();
  char key = keypad.getKey(); // read the keypad

  if(key == '1') {
   // Serial.print("key 1 pressed;");
    if ((Serial.available()>0)==false){
       lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Waiting Serial");
    }
    printLocLCD=1;
    printLoc = 1;
  }
  else if (key=='2'){
    printLocLCD=0;
   printLoc = 0;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Next cell no:");
    //delay(4000);
    char key1 = NO_KEY;
    startGrid = currentGrid;

 int cellInt = -1;
    do {
      key1 = keypad.getKey();
    } while (key1 == NO_KEY);
    if (key1) {
      cellInt = key1 - '0' - 1;
      targetGrid = cellInt;
    }
    Serial.println(targetGrid);
    lcd.clear();


    int distance[rows * cols];
    int parent[rows * cols];
    int shortestPath[rows * cols];

    int length = dijkstra(startGrid, targetGrid, distance, parent, shortestPath);

    if (distance[targetGrid] != inf) {
      Serial.println("Shortest path exists!");
      Serial.print("Shortest Path: ");
      printPath(shortestPath, length);
      Serial.println();

      // Convert the path to movements
      convertPathToMovements(shortestPath, length);
      currentGrid = targetGrid;
    } else {
      Serial.println("No path found.");
    }
  
    lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Press key 1 to");
  lcd.setCursor(0, 1);
  lcd.print("localize:");
  delay(1500);
  lcd.setCursor(0, 0);
  lcd.print("Press key 2 to");
  lcd.setCursor(0, 1);
  lcd.print("navigate:");
  delay(1500);
  lcd.setCursor(0, 0);
  lcd.print("Press key 3 to");
  lcd.setCursor(0, 1);
  lcd.print("sensor values:");
  delay(1500);
  } else if (key == '3') {
   printLoc = 0;
   printLocLCD=0;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Water: ");
    lcd.print(waterSensorValue);
    lcd.setCursor(0, 1);
    lcd.print("Smoke: ");
    lcd.print(smokeSensorValue);
    delay(2000);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(temperature);
    lcd.print(" C");
    lcd.setCursor(0, 1);
    lcd.print("Humidity: ");
    lcd.print(humidity);
    lcd.print(" %");
    delay(2000);
  }

}

String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void printAverage(String apName, String ssid, int *rssiArray, int count) {
  int sum = 0;
  for (int i = 0; i < count; i++) {
    sum += rssiArray[i];
  }

  int average = sum / count;

  Serial.print(apName + " SSID: ");
  Serial.print(ssid);
  Serial.print("  Average RSSI: ");
  Serial.println(average);
  average = average * -1;

  //if(printLoc == 1) {
    if (ssid.indexOf("ESPap1") != -1) {
      ap1RSSIavg = average;
    } else if (ssid.indexOf("ESPap2") != -1) {
      ap2RSSIavg = average;
    } else if (ssid.indexOf("ESPap3") != -1) {
      ap3RSSIavg = average;
    }
    trilaterate(ap1RSSIavg, ap2RSSIavg, ap3RSSIavg);
   
 // }
 
}


float calculateDistanceAP1(int avgRSSI) {
  float distance = -948.7578 + 257.7917 * log(avgRSSI);
  // Serial.println("distance1= " );
  // Serial.println(distance);
    if(distance>135) distance = 135;
     else if(distance <0) distance=0;
  return distance;
}

float calculateDistanceAP2(int avgRSSI) {
 float distance = -948.7578 + 257.7917 * log(avgRSSI);
  //  Serial.println("distance2= " ); Serial.println(distance);
     if(distance>135) distance = 135;
      else if(distance <0) distance=0;
  return distance;
}

float calculateDistanceAP3(int avgRSSI) {
   float distance = -932.2568 + 264.0196 * log(avgRSSI);
  //  Serial.println("distance3= "); Serial.println(distance);
   if(distance>135) distance = 135;
     else if(distance <0) distance=0;
  return distance;
}


void trilaterate(int avgRSSI1, int avgRSSI2, int avgRSSI3) {
  float x1 = 90.0;
  float y1 = 0.0;

  float x2 = 0.0;
  float y2 = 135.0;

  float x3 = 135.0;
  float y3 = 135.0;

  float d1 = calculateDistanceAP1(avgRSSI1);
  float d2 = calculateDistanceAP2(avgRSSI2);
  float d3 = calculateDistanceAP3(avgRSSI3);

  float A = 2 * x2 - 2 * x1;
  float B = 2 * y2 - 2 * y1;
  float C = d1 * d1 - d2 * d2 - x1 * x1 + x2 * x2 - y1 * y1 + y2 * y2;
  float D = 2 * x3 - 2 * x2;
  float E = 2 * y3 - 2 * y2;
  float F = d2 * d2 - d3 * d3 - x2 * x2 + x3 * x3 - y2 * y2 + y3 * y3;

  float x = (C * E - F * B) / (E * A - B * D);
  float y = (C * D - A * F) / (B * D - A * E);
//  Serial.println("");
//   Serial.print("Trilateration Result - X: ");
//   Serial.print(x);
//   Serial.print(", Y: ");
//   Serial.println(y);
  printCurrentCell(x,y);

}
int mapNumber(int num) {
    switch (num) {
        case 7:
            return 1;
        case 8:
            return 2;
        case 9:
            return 3;
        case 4:
            return 4;
        case 5:
            return 5;
        case 6:
            return 6;
        case 1:
            return 7;
        case 2:
            return 8;
        case 3:
            return 9;
        default:
            return -1; // Return -1 if the number is not in the mapping
    }
}
void printCurrentCell(float x, float y) {
  // Define the grid dimensions
  int gridRows = 3;
  int gridCols = 3;

  // Define the size of each cell in the grid
  float cellWidth = 135.0 / gridCols;
  float cellHeight = 135.0 / gridRows;

  int cellRow = (int(y / cellHeight)); //1
  int cellCol = int(x / cellWidth);//0

  // Calculate the corresponding cell number (0-based index)
  int cellNumber = cellRow * gridCols + cellCol ;  

  // Print the result
  int currnt = mapNumber(cellRow * gridCols + cellCol +1);
  Serial.print("Current cell:");
  Serial.println(currnt);
  cellNumberToMobile= currnt;
  if(printLocLCD == 1){
      lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Current Cell:");
  lcd.setCursor(0, 1);
  lcd.print(currnt);
  }
 
}