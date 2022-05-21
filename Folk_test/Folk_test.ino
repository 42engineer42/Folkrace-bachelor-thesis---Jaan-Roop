/*
 Kui midagi muud head välja ei mõtle siis võta PD kood 06.05.2022
 Name:    Folk_test.ino
 Created: 11/20/2021 12:23:23 AM
 Author:  Jaan
*/

#include <Arduino.h>
#include <math.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050_tockn.h>
#include <VNH3SP30.h>
#include <splash.h>
#include <Servo.h>
#include <WireKinetis.h>
#include <WireIMXRT.h>
#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <SoftwareSerial.h>
#include <ezButton.h>
#include <FastLED.h>
#include <bits/stdc++.h>
#include <I2CScanner.h>
#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX

using namespace std;

#define NUM_LEDS 19
#define DATA_PIN 6

I2CScanner scanner;

#define ServoPWM 0
#define LPn2 1
#define INT1 2
#define LPn1 3
#define ENC1 4
#define ENC2 5
#define BluetoothRX 7
#define BluetoothTX 8
#define Bumper 9
#define SW2 10
#define SW1 11
#define RST6 12
#define LPn3 13
#define INT2 14
#define RST2 15
#define SCL1 16
#define SDA1 17
#define SDA0 18
#define SCL0 19
#define RST1 20
#define Cell1 23
#define Cell2 22
#define Cell3 21
#define SCL2 24
#define SDA2 25
#define Enable 26
#define Current 27
#define INTimu 28
#define MotorPWM 29
#define INB 30
#define INA 31
#define INT6 32
#define LPn5 33
#define INT5 34
#define RST5 35
#define LPn6 36
#define INT3 37
#define RST3 38
#define LPn4 39
#define INT4 40
#define RST4 41

///////////////MOTOR FEEDBACK/////////////////////
float DrivingSpeed = 200;

long encoder1 = 0;
bool driving_direction = 0;
void Encoder1() {
    encoder1 = encoder1 + (driving_direction * 1);         // liidab või lahutab ühe enkoodri counterist
    Dynamics();
}


long encoder2 = 0;

void Encoder2() {
    encoder2 = encoder2 + (driving_direction * 1);         // liidab või lahutab ühe enkoodri counterist
    Dynamics();
}


long totalClicks = 0;
float rotations = 0;
float clicksPerRotation = (464.64)/2.49;   /* Mitu enkoodri klikki peab olema enne kui mootori gearboxi 
                                           valjundvoll teeb yhe taispoorde */ 
float wheelDiameter = 30.4;                // Millimetrites ratta diameeter
float motorToWheelsTransmission = 1;       /* Ylekandetegur mootoritelt ratastele. NT 1 on 1:1 ; 2 on 2:1 ehk mootor teeb 2 pooret 
                                           enne kui ratas yhe; 0.5 on 1:2 ehk mootor teeb yhe poorde samal ajal kui rattad 2 */
float elapsedDistance = 0;                 // Labitud vahemaa mm enkoodri info baasil libisemist arvestamata

void Dynamics() {
    totalClicks = encoder1 + encoder2;
    rotations = totalClicks / clicksPerRotation;
    elapsedDistance = (rotations * wheelDiameter) / motorToWheelsTransmission;
}



/////////////BLUETOOTH/////////////
String data = "";
char info = 0;

void Bluetooth() {
   while (Serial2.available()) { 
        info = Serial2.read();
        Serial.println(info);
        if (info != '#') { data.concat(info); } // Add the received character to the receive buffer 
   }
   if (info == '#' && data.length() > 0) { //If end of message character recieved, move on.
           /*Do some action with recived message*/
           Serial.println();
           Serial.println(data); //Output the message
           bluetoothActions(data);
           delay(100);
       }
}

void BluetoothSend(float msgToSend) {
   if (Serial2.available()) { 
        Serial2.print(msgToSend);
        
   }
}

bool kitMode = false;
bool motorON = false;
bool steeringON = false;
bool isAttached = false;

//////////OLED/////////////
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire2, OLED_RESET);

void OLED() {
    //SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        while(1){} // Don't proceed, loop forever
    }

    // Show initial display buffer contents on the screen --
    // the library initializes this with an Adafruit splash screen.
    display.display();
    delay(2000); // Pause for 2 seconds

    // Clear the buffer
    display.clearDisplay();
}


///////////////IMU///////////////////////
MPU6050 mpu6050(Wire2);
unsigned long IMUlastTime = 0;
unsigned long TimeElapsed = 0;
void IMU() {
  IMUlastTime = millis();
  mpu6050.update(); 
    
  Serial.println("=======================================================");
  Serial.print("temp : "); Serial.println(mpu6050.getTemp());
  Serial.print("accX : "); Serial.print(mpu6050.getAccX());                                  //positiivne vasakule ja negatiivne paremale
  Serial.print("\taccY : "); Serial.print(mpu6050.getAccY());                                //negatiivne on otse s�it ja positiivne tagurpidi
  Serial.print("\taccZ : "); Serial.println(mpu6050.getAccZ());                              //upside down or not
  Serial.print("\tangleZ : "); Serial.println(mpu6050.getAngleZ());                           //LAP counter
  Serial.println("=======================================================\n");
  
  IMUspeed();
}

float SpeedXfromIMU = 0;
float SpeedYfromIMU = 0;
float SpeedZfromIMU = 0;
void IMUspeed(){
  SpeedXfromIMU = mpu6050.getAccX() * (millis() - IMUlastTime) / 1000;
  SpeedYfromIMU = mpu6050.getAccY() * (millis() - IMUlastTime) / 1000;
  SpeedZfromIMU = mpu6050.getAccZ() * (millis() - IMUlastTime) / 1000;  
}

void DATAtoPC() {
  Serial.print("Enkooder 1: ");
  Serial.println(encoder1);
  Serial.print("Enkooder 2: ");
  Serial.println(encoder2);

  Serial.print("Motor rotations: ");
  Serial.println(rotations);
  Serial.print("Elapsed distance: ");
  Serial.println(elapsedDistance);
}

long InterruptTimer = 0;

void Interrupts() { 
    //attachInterrupt(digitalPinToInterrupt(INTimu), IMU, FALLING);
    attachInterrupt(digitalPinToInterrupt(ENC1), Encoder1, FALLING);
    attachInterrupt(digitalPinToInterrupt(ENC2), Encoder2, FALLING);
    
    if (millis() - InterruptTimer > 50) {
        DATAtoPC();
        InterruptTimer = millis(); 
    }
}

////////// KIT ///////////
CRGB leds[NUM_LEDS];
bool BatteryNeedsCharging = false;
bool once = false;
void kit(bool countdown, bool battary, long waitTime) {
    int FPS = 10;
    long startTime = 0;
    Serial.println();
    if (countdown){
      startTime = millis();
    }
    while (kitMode) {
        if (countdown){
          Serial.print("countdown: ");
          Serial.print(startTime + waitTime - millis());
          Serial.println();
          if (startTime + waitTime < millis()){
            kitMode = false;
            once = true;
          }
        }
        else{
          Bluetooth();
          Battery();
        }
        
        
        if (battary){
          if(BatteryNeedsCharging == false){kitMode = false;}
        }
        Serial.print("kitmode: ");
          Serial.print(kitMode);
          Serial.print(!once);
          Serial.println();
        int i = 0;
        if (kitMode == true && !once) {
          Serial.print("KIT");
            if (i == 0) {
                for (i; i < NUM_LEDS; i++) {
                    if (i > 2) {
                        leds[i - 3] = CRGB(0, 0, 0);
                        leds[i - 2] = CRGB(0, 230, 0);
                        leds[i - 1] = CRGB(0, 215, 0);
                        leds[i] = CRGB(0, 200, 0);
                        FastLED.show();
                        delay(FPS);
                    }
                    else {
                        if (i == 0) {
                            leds[i] = CRGB(0, 200, 0);
                            FastLED.show();
                            delay(FPS);
                        }
                        else if (i == 1) {
                            leds[i - 1] = CRGB(0, 215, 0);
                            leds[i] = CRGB(0, 200, 0);
                            FastLED.show();
                            delay(FPS);
                        }
                        else if (i == 2) {
                            leds[i - 2] = CRGB(0, 230, 0);
                            leds[i - 1] = CRGB(0, 215, 0);
                            leds[i] = CRGB(0, 200, 0);
                            FastLED.show();
                            delay(FPS);
                        }
                    }
                }
            }
            if (i == NUM_LEDS) {
                for (i; i > -1; i--) {
                    if (i < NUM_LEDS - 2) {
                        leds[i + 3] = CRGB(0, 0, 0);
                        leds[i + 2] = CRGB(0, 230, 0);
                        leds[i + 1] = CRGB(0, 215, 0);
                        leds[i] = CRGB(0, 200, 0);
                        FastLED.show();
                        delay(FPS);
                    }
                    else {
                        if (i == NUM_LEDS - 1) {
                            leds[i] = CRGB(0, 200, 0);
                            FastLED.show();
                            delay(FPS);
                        }
                        else if (i == NUM_LEDS - 2) {
                            leds[i + 1] = CRGB(0, 215, 0);
                            leds[i] = CRGB(0, 200, 0);
                            FastLED.show();
                            delay(FPS);
                        }
                        else if (i == NUM_LEDS - 3) {
                            leds[i + 2] = CRGB(0, 230, 0);
                            leds[i + 1] = CRGB(0, 215, 0);
                            leds[i] = CRGB(0, 200, 0);
                            FastLED.show();
                            delay(FPS);
                        }
                    }
                }
            }
        }
        else { break; }
    }
}

float Cell1analog = 0;
float Cell2analog = 0;
float Cell3analog = 0;
float BatCell1Voltage = 0;
float BatCell2Voltage = 0;
float BatCell3Voltage = 0;
float TotalVoltage = 0;
float Cell1Percentage = 0;
float Cell2Percentage = 0;
float Cell3Percentage = 0;
float TotalBatteryPercentage = 0;
float CutoffVoltage = 3.3;
const int LastValues = 15;
float Cell1LastValues [LastValues] = {};
float Cell2LastValues [LastValues] = {};
float Cell3LastValues [LastValues] = {};
int LastCounter = 0;
long LastDisplayTime = 0;
long DisplayShowInterval = 20000; //1000 means 1 second delay

void Battery() {
  
  for(int i = 0; i < LastValues; i++){
    Cell1LastValues [i] = {analogRead(Cell1)};
    Cell2LastValues [i] = {analogRead(Cell2)};
    Cell3LastValues [i] = {analogRead(Cell3)};
  }
  
  Cell1analog = 0;
  Cell2analog = 0;
  Cell3analog = 0;
  
  for(int x = 0; x < LastValues; x++) {
    Cell1analog = Cell1analog + Cell1LastValues [x];
    Cell2analog = Cell2analog + Cell2LastValues [x];
    Cell3analog = Cell3analog + Cell3LastValues [x];  
  }
  Cell1analog = Cell1analog / LastValues;
  Cell2analog = Cell2analog / LastValues;
  Cell3analog = Cell3analog / LastValues;

  //Pingejaguri tõttu on vaja analoogväärtus korrutada maksimaalse sisenpingega 3V ja pingejaguri suhtega ning jagada ADC resolutsiooniga
  BatCell1Voltage = (Cell1analog * 0.004756); 
  BatCell2Voltage = (Cell2analog * 0.009545) - BatCell1Voltage;
  if(BatCell2Voltage < 0){BatCell2Voltage = 0;}
  BatCell3Voltage = (Cell3analog * 0.01374) - BatCell1Voltage - BatCell2Voltage;
  if(BatCell3Voltage < 0){BatCell3Voltage = 0;}
  if(BatCell1Voltage < CutoffVoltage || BatCell2Voltage < CutoffVoltage || BatCell3Voltage < CutoffVoltage){BatteryNeedsCharging = true;}
  else{BatteryNeedsCharging = false;}
  TotalVoltage = BatCell1Voltage + BatCell2Voltage + BatCell3Voltage;

  Cell1Percentage = ((BatCell1Voltage - CutoffVoltage) / (4.2 - CutoffVoltage)) * 100;
  if(Cell1Percentage < 0){Cell1Percentage = 0;}
  Cell2Percentage = ((BatCell2Voltage - CutoffVoltage) / (4.2 - CutoffVoltage)) * 100;
  if(Cell2Percentage < 0){Cell2Percentage = 0;}
  Cell3Percentage = ((BatCell3Voltage - CutoffVoltage) / (4.2 - CutoffVoltage)) * 100;
  if(Cell3Percentage < 0){Cell3Percentage = 0;}
  TotalBatteryPercentage = (Cell1Percentage + Cell2Percentage + Cell3Percentage) / 3;

  //BatteryPrint();
  BatteryOLED();
}

void BatteryPrint() {
    Serial.print("CELL 1:   ");
    Serial.println(Cell1analog);
    Serial.print("Voltage 1:   ");
    Serial.println((int)floor(BatCell1Voltage));
    Serial.print("Percentage Cell 1:   ");
    Serial.println(Cell1Percentage);
    Serial.print("CELL 2:   ");
    Serial.println(Cell2analog);
    Serial.print("Voltage 2:   ");
    Serial.println(BatCell2Voltage);
    Serial.print("Percentage Cell 2:   ");
    Serial.println(Cell2Percentage);
    Serial.print("CELL 3:   ");
    Serial.println(Cell3analog);
    Serial.print("Voltage 3:   ");
    Serial.println(BatCell3Voltage);
    Serial.print("Percentage Cell 3:   ");
    Serial.println(Cell3Percentage);

    Serial.print("Total Voltage:   ");
    Serial.println(TotalVoltage);
    Serial.print("Total Percentage:   ");
    Serial.println(TotalBatteryPercentage);
    Serial.print("NEEDS CHARGING? ");
    if(BatteryNeedsCharging == 1){Serial.println("YES");}
    else{Serial.println("NO");}
}

void BatteryOLED() {
    display.clearDisplay();
    display.setTextSize(1);             // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);        // Draw white text
    display.setCursor(0,0);             // Start at top-left corner
    display.println(F("C1:"));
    display.setCursor(20,0);
    char CellChar [6] = {' '};
    sprintf(CellChar, "%.2f",(round(BatCell1Voltage * 100))/100);
    display.println(CellChar);
    display.setCursor(45,0);
    display.println(F("V"));
    display.setCursor(75,0);             // Start at top-left corner
    display.println(F("C2:"));
    display.setCursor(95,0);
    sprintf(CellChar, "%.2f",(round(BatCell2Voltage * 100))/100);
    display.println(CellChar);
    display.setCursor(120,0);
    display.println(F("V"));
    display.setCursor(0,8);             // Start at top-left corner
    display.println(F("C3:"));
    display.setCursor(20,8);
    sprintf(CellChar, "%.2f",(round(BatCell3Voltage * 100))/100);
    display.println(CellChar);
    display.setCursor(45,8);
    display.println(F("V"));
    display.setCursor(75,8);             // Start at top-left corner
    display.println(F("B%:"));
    display.setCursor(95,8);
    sprintf(CellChar, "%.1f",(round(TotalBatteryPercentage) * 10)/10);
    display.println(CellChar);
    display.setCursor(120,8);
    display.println(F("%"));
    display.setTextSize(2);             // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);        // Draw white text
    display.setCursor(0,16); 
    if(BatteryNeedsCharging == 1){display.println(F("CHARGE ME"));}
    else{display.println(F("BAT. OKAY"));}
    display.display(); 
}

/////////////// DISTANCE SENSORS ////////////////////////

int imageResolution = 0; //Used to pretty print output
int imageWidth = 0; //Used to pretty print output

SparkFun_VL53L5CX Sensor1;
int sensorAddress1 = 0x26; //New address of unit without a wire. Valid: 0x08 <= address <= 0x77
int sensorReset1 = 15; //GPIO that is connected to the Reset pin on sensor 1
VL53L5CX_ResultsData measurementData1;

SparkFun_VL53L5CX Sensor2;
int sensorAddress2 = 0x27; //New address of unit without a wire. Valid: 0x08 <= address <= 0x77
int sensorReset2 = 20; //GPIO that is connected to the Reset pin on sensor 2
VL53L5CX_ResultsData measurementData2;

SparkFun_VL53L5CX Sensor3;
int sensorAddress3 = 0x28; //New address of unit without a wire. Valid: 0x08 <= address <= 0x77
int sensorReset3 = 38; //GPIO that is connected to the Reset pin on sensor 3
VL53L5CX_ResultsData measurementData3;

SparkFun_VL53L5CX Sensor4;
int sensorAddress4 = 0x29; //Default VL53L5CX - this is the unit we'll hold in reset (has the wire soldered)
int sensorReset4 = 41; //GPIO that is connected to the Reset pin on sensor 4
VL53L5CX_ResultsData measurementData4;

SparkFun_VL53L5CX Sensor5;
int sensorAddress5 = 0x28; //New address of unit without a wire. Valid: 0x08 <= address <= 0x77
int sensorReset5 = 12; //GPIO that is connected to the Reset pin on sensor 5
VL53L5CX_ResultsData measurementData5;

SparkFun_VL53L5CX Sensor6;
int sensorAddress6 = 0x29; //Default VL53L5CX - this is the unit we'll hold in reset (has the wire soldered)
int sensorReset6 = 35; //GPIO that is connected to the Reset pin on sensor 6
VL53L5CX_ResultsData measurementData6;


void CameraSetup () {
  pinMode(sensorReset6, OUTPUT);
  digitalWrite(sensorReset6, HIGH); //Hold sensor 6 in reset while we configure sensor 5, 4, 2, 1

  pinMode(sensorReset5, OUTPUT);
  digitalWrite(sensorReset5, HIGH); //Hold sensor 5 in reset while we configure sensor 2, 1

  pinMode(sensorReset4, OUTPUT);
  digitalWrite(sensorReset4, HIGH); //Hold sensor 4 in reset while we configure sensor 3
  
  pinMode(sensorReset2, OUTPUT);
  digitalWrite(sensorReset2, HIGH); //Hold sensor 6 in reset while we configure sensor 1

  
  pinMode(sensorReset1, OUTPUT);
  digitalWrite(sensorReset1, HIGH); //Reset sensor 1
  delay(100);
  digitalWrite(sensorReset1, LOW); //Sensor 1 should now be available at default address 0x29
  delay(5);

  Serial.println(F("Initializing sensor 1. This can take up to 10s. Please wait."));
  while (Sensor1.begin() == false) {
    Serial.println(F("Sensor 1 not found. Check wiring. Trying again..."));
    pinMode(sensorReset1, OUTPUT);
    digitalWrite(sensorReset1, HIGH); //Reset sensor 1
    delay(100);
    digitalWrite(sensorReset1, LOW); //Sensor 1 should now be available at default address 0x29
    delay(5);
  }

  Serial.print(F("Setting sensor 1 address to: 0x"));
  Serial.println(sensorAddress1, HEX);
  while (Sensor1.setAddress(sensorAddress1) == false || Sensor1.setAddress(sensorAddress1) == false) {
    Serial.println(F("Sensor 1 failed to set new address. Trying again..."));
    while (Sensor1.begin() == false) {
      Serial.println(F("Sensor 1 not found. Check wiring. Trying again..."));
      pinMode(sensorReset1, OUTPUT);
      digitalWrite(sensorReset1, HIGH); //Reset sensor 1
      delay(100);
      digitalWrite(sensorReset1, LOW); //Sensor 1 should now be available at default address 0x29
      delay(5);
    }
  }

  int newAddress = Sensor1.getAddress();

  Serial.print(F("New address of sensor 1 is: 0x"));
  Serial.println(newAddress, HEX);
  display.clearDisplay();
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F("1OK"));
  display.display();

  digitalWrite(sensorReset2, HIGH); //Reset sensor 2
  delay(100);
  digitalWrite(sensorReset2, LOW); //Sensor 2 should now be available at default address 0x29
  delay(5);

  Serial.println(F("Initializing sensor 2. This can take up to 10s. Please wait."));
  while (Sensor2.begin() == false) {
    Serial.println(F("Sensor 2 not found. Check wiring. Trying again..."));
    pinMode(sensorReset2, OUTPUT);
    digitalWrite(sensorReset2, HIGH); //Reset sensor 2
    delay(100);
    digitalWrite(sensorReset2, LOW); //Sensor 2 should now be available at default address 0x29
    delay(5);
  }

  Serial.print(F("Setting sensor 2 address to: 0x"));
  Serial.println(sensorAddress2, HEX);
  while (Sensor2.setAddress(sensorAddress2) == false) {
    Serial.println(F("Sensor 2 failed to set new address. Trying again..."));
    while (Sensor2.begin() == false) {
      Serial.println(F("Sensor 2 not found. Check wiring. Trying again..."));
      pinMode(sensorReset2, OUTPUT);
      digitalWrite(sensorReset2, HIGH); //Reset sensor 2
      delay(100);
      digitalWrite(sensorReset2, LOW); //Sensor 2 should now be available at default address 0x29
      delay(5);
    }
  }

  newAddress = Sensor2.getAddress();

  Serial.print(F("New address of sensor 2 is: 0x"));
  Serial.println(newAddress, HEX);
  display.setCursor(43,0);   
  display.println(F("2OK"));
  display.display();

  digitalWrite(sensorReset3, HIGH); //Reset sensor 3
  delay(100);
  digitalWrite(sensorReset3, LOW); //Sensor 3 should now be available at default address 0x29
  delay(5);

  Serial.println(F("Initializing sensor 3. This can take up to 10s. Please wait."));
  while (Sensor3.begin(0x29, Wire1) == false) {
    Serial.println(F("Sensor 3 not found. Check wiring. Trying again..."));
    pinMode(sensorReset3, OUTPUT);
    digitalWrite(sensorReset3, HIGH); //Reset sensor 3
    delay(100);
    digitalWrite(sensorReset3, LOW); //Sensor 3 should now be available at default address 0x29
    delay(5);
  }

  Serial.print(F("Setting sensor 3 address to: 0x"));
  Serial.println(sensorAddress3, HEX);
  while (Sensor3.setAddress(sensorAddress3) == false) {
    Serial.println(F("Sensor 3 failed to set new address. Trying again..."));
    while (Sensor3.begin(0x29, Wire1) == false) {
      Serial.println(F("Sensor 3 not found. Check wiring. Trying again..."));
      pinMode(sensorReset3, OUTPUT);
      digitalWrite(sensorReset3, HIGH); //Reset sensor 3
      delay(100);
      digitalWrite(sensorReset3, LOW); //Sensor 3 should now be available at default address 0x29
      delay(5);
    }
  }

  newAddress = Sensor3.getAddress();

  Serial.print(F("New address of sensor 3 is: 0x"));
  Serial.println(newAddress, HEX);
  display.setCursor(85,0);   
  display.println(F("3OK"));
  display.display();
  
  Serial.println(F("Initializing sensor 4. This can take up to 10s. Please wait."));
  digitalWrite(sensorReset4, LOW); //Release sensor 4 from reset

  while (Sensor4.begin(0x29, Wire1) == false) {
    Serial.println(F("Sensor 4 not found. Check wiring. Trying again..."));
    pinMode(sensorReset4, OUTPUT);
    digitalWrite(sensorReset4, HIGH); //Reset sensor 4
    delay(100);
    digitalWrite(sensorReset4, LOW); //Sensor 4 should now be available at default address 0x29
    delay(5);
  }
  display.setCursor(0,16);   
  display.println(F("4OK"));
  display.display();


  pinMode(sensorReset5, OUTPUT);
  digitalWrite(sensorReset5, HIGH); //Reset sensor 5
  delay(100);
  digitalWrite(sensorReset5, LOW); //Sensor 5 should now be available at default address 0x29
  delay(5);

  Serial.println(F("Initializing sensor 5. This can take up to 10s. Please wait."));
  while (Sensor5.begin() == false) {
    Serial.println(F("Sensor 5 not found. Check wiring. Trying again..."));
    pinMode(sensorReset5, OUTPUT);
    digitalWrite(sensorReset5, HIGH); //Reset sensor 5
    delay(100);
    digitalWrite(sensorReset5, LOW); //Sensor 5 should now be available at default address 0x29
    delay(5);
  }

  Serial.print(F("Setting sensor 5 address to: 0x"));
  Serial.println(sensorAddress5, HEX);
  while (Sensor5.setAddress(sensorAddress5) == false) {
    Serial.println(F("Sensor 5 failed to set new address. Trying again..."));
    while (Sensor5.begin() == false) {
      Serial.println(F("Sensor 5 not found. Check wiring. Trying again..."));
      pinMode(sensorReset5, OUTPUT);
      digitalWrite(sensorReset5, HIGH); //Reset sensor 5
      delay(100);
      digitalWrite(sensorReset5, LOW); //Sensor 5 should now be available at default address 0x29
      delay(5);
    }
  }

  newAddress = Sensor5.getAddress();

  Serial.print(F("New address of sensor 5 is: 0x"));
  Serial.println(newAddress, HEX);
  display.setCursor(43,16);   
  display.println(F("5OK"));
  display.display();

  Serial.println(F("Initializing sensor 6. This can take up to 10s. Please wait."));
  digitalWrite(sensorReset6, LOW); //Release sensor 6 from reset

  while (Sensor6.begin() == false) {
    Serial.println(F("Sensor 6 not found. Check wiring. Trying again..."));
    pinMode(sensorReset6, OUTPUT);
    digitalWrite(sensorReset6, HIGH); //Reset sensor 6
    delay(100);
    digitalWrite(sensorReset6, LOW); //Sensor 6 should now be available at default address 0x29
    delay(5);
  }
  display.setCursor(85,16);   
  display.println(F("6OK"));
  display.display();
  Serial.println(F("DONE"));  
}

void SensorResolutionStuff(SparkFun_VL53L5CX sensor){
  int SensorFrequency = 60; //4*4 max 60, 8*8 max 15
  for( int i = 0; i < 6; i++){
    sensor.setResolution(4 * 4);
    sensor.setRangingFrequency(SensorFrequency);
    sensor.setTargetOrder(SF_VL53L5CX_TARGET_ORDER::CLOSEST);
    sensor.setRangingMode(SF_VL53L5CX_RANGING_MODE::CONTINUOUS);
    sensor.setSharpenerPercent(0);
    sensor.startRanging();
  }
}
  
void CameraInit(){ 
  SparkFun_VL53L5CX mySensors[]={Sensor1, Sensor2, Sensor3, Sensor4, Sensor5, Sensor6}; 
  for(int i = 0; i < 6; i++){ 
    SensorResolutionStuff( mySensors[i] ); 
  } 
}

void CameraGetData() {
  //Poll sensor for new data
  if (Sensor1.isDataReady() == true) { Sensor1.getRangingData(&measurementData1); } //Read distance data into array
  if (Sensor2.isDataReady() == true) { Sensor2.getRangingData(&measurementData2); } //Read distance data into array    
  if (Sensor3.isDataReady() == true) { Sensor3.getRangingData(&measurementData3); } //Read distance data into array
  if (Sensor4.isDataReady() == true) { Sensor4.getRangingData(&measurementData4); } //Read distance data into array    
  if (Sensor5.isDataReady() == true) { Sensor5.getRangingData(&measurementData5); } //Read distance data into array
  if (Sensor6.isDataReady() == true) { Sensor6.getRangingData(&measurementData6); } //Read distance data into array 
  
}

void PrintNum(int num){
  Serial.print("\t");
  Serial.print(num);
  Serial.print(":");
}
void PrintCameraData(int num, VL53L5CX_ResultsData sensor){
  for (int x = 0 ; imageWidth > x ; x++) {
    for (int y = 0 ; y <= imageWidth * (imageWidth - 1) ; y += imageWidth) {
      PrintNum(num);
      Serial.print(sensor.distance_mm[x + y]);
    }
    Serial.println();
  }
  Serial.println();
}
void PrintAllCameraData() {
  imageResolution = Sensor1.getResolution(); //Query sensor for current resolution - either 4x4 or 8x8
  imageWidth = sqrt(imageResolution); //Calculate printing width
  VL53L5CX_ResultsData measurementDatas[]={measurementData1, measurementData2, measurementData3, measurementData4, measurementData5, measurementData6};

  for(int i = 1; i <= 6; i++){
    PrintCameraData(i, measurementDatas[i-1]);
  }
}



int SensorLine = 0;   //2 Kasutab kolmandat rida 
int SensorCount = 6;
int UsingDataPerLine = 3;
int StartLed = (NUM_LEDS - SensorCount * UsingDataPerLine) / 2;
bool SensorStart = 0;
int SensorLedLine3 [19] = {0};

void OneDataLineArray() { 
  for(int y = StartLed; (SensorCount * UsingDataPerLine + StartLed) > y; y++) {
    for (int z = 0; SensorCount > z; z++) {
      if (z == 1 || z == 3 || z == 5){SensorStart = 1;}
      else {SensorStart = 0;}
      for (int x = (0 + SensorStart); (UsingDataPerLine + SensorStart) > x; x++) {
        switch (z) {
          case 0:
            SensorLedLine3 [y] = {measurementData6.distance_mm[SensorLine + x]};
            y++;
          break;
          case 1:
            SensorLedLine3 [y] = {measurementData5.distance_mm[SensorLine + x]};
            y++;
          break;
          case 2:
            SensorLedLine3 [y] = {measurementData3.distance_mm[SensorLine + x]};
            y++;
          break;
          case 3:
            SensorLedLine3 [y] = {measurementData4.distance_mm[SensorLine + x]};
            y++;
          break;
          case 4:
            SensorLedLine3 [y] = {measurementData2.distance_mm[SensorLine + x]};
            y++;
          break;
          case 5:
            SensorLedLine3 [y] = {measurementData1.distance_mm[SensorLine + x]};
            y++;
          break;
        }
      }  
    }
  }
  Serial.print("Line 3: ");
  for (int z = 0; NUM_LEDS > z; z++) { 
    if(SensorLedLine3 [z] > 2550){SensorLedLine3 [z] = {2550};}
    Serial.print(SensorLedLine3 [z]); 
    Serial.print(" ");  
  }
  Serial.println();  
}



#define LedBrightness 255
float RedLed = 0;
float GreenLed = 0;

void LedCamera() {
  for (int z = 0; NUM_LEDS > z; z++) { 
    if(SensorLedLine3[z] == 0) {
      RedLed = 0;
      GreenLed = 0;
    }
    else {
      RedLed = (SensorLedLine3 [z]/(2550.0/LedBrightness));
      GreenLed = LedBrightness - (SensorLedLine3 [z]/(2550.0/LedBrightness));
    }
    leds[z] = CRGB(RedLed, GreenLed, 0);  
  }
  FastLED.show();
}


int SensorLedLineTest[NUM_LEDS] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18};
const int WindowSize = 3;
const int WindowsAmount = NUM_LEDS/WindowSize;
int posCenter = NUM_LEDS / 2; //Start postition for front wheels (center)
int WindowsArray [WindowsAmount][WindowSize];
int FavorablePosition = 0;

int bestPos = posCenter;

void BestDirection(){
  int pointer = 0;
  int WindowPointer = 0;
  int Window = 0;
  /*for (int z = 0; NUM_LEDS > z; z += 3) {
    if (WindowPointer == WindowSize){
      WindowPointer = 0;
      Window++;
    }
    for(WindowPointer; WindowPointer < WindowSize; WindowPointer++){ WindowsArray[Window][WindowPointer] = SensorLedLine3[z + WindowPointer];} //reaalsuses kasuta SensorLedLine3
  }
  for (int i = 0; i < WindowsAmount; i++){
    for (int j = 0; j < WindowSize; j++){
      Serial.print(WindowsArray[i][j]);
      Serial.print(" ");
    }
  }
  Serial.println();*/
  bestPos = NUM_LEDS - 1 - findBestPositionRida(); //RIDA
  //Serial.print("Best pos: ");
  //Serial.print(bestPos);
  //Serial.println();
  //Serial.print("Best values: ");
  //Serial.print((SensorLedLine3[bestPos] + SensorLedLine3[bestPos + 1])/2);
  //Serial.println();
  return bestPos;
}
int findBestPositionRida(){
  int currPosition = 1;
  int bestPosition = NUM_LEDS / 2;
  float greatestAverage = 0;
  for (currPosition; currPosition < NUM_LEDS / 2; currPosition++){
    float leftAverage = SensorLedLine3[currPosition - 1] + SensorLedLine3[currPosition] / 2.0;
    float rightAverage = SensorLedLine3[NUM_LEDS - 1 - currPosition] + SensorLedLine3[NUM_LEDS - currPosition] / 2.0;
    if (leftAverage > rightAverage){
        if (leftAverage > greatestAverage){
            bestPosition = currPosition - 1;
            greatestAverage = leftAverage;
        }
    }
    else{
        if (rightAverage > greatestAverage){
            bestPosition = NUM_LEDS - currPosition;
            greatestAverage = rightAverage;
        }
    }
    
  }
  return bestPosition;
}


int findBestPositionAken(){
  int currPosition = 1;
  int bestPosition = 0;
  int greatestAverage = 0;
  for (currPosition; currPosition < WindowsAmount - 1; currPosition++){
    int currAverage = WindowAverage(WindowsArray[currPosition-1], WindowsArray[currPosition]);
    if (currAverage > greatestAverage){
      bestPosition = currPosition;
      greatestAverage = currAverage;
    }
  }
  return bestPosition;
}

int WindowAverage(int Window1[3], int Window2[3]){
  float WindowSum = 0;
  for (int i = 0; i < WindowSize; i++){
    WindowSum += Window1[i] + Window2[i];
  }
  return WindowSum/(WindowSize*2);
}

//////////////////TURN////////////
Servo aservo;
float turnCenter = 88.0;
float turn = turnCenter;   
int rightMax = 50;
int leftMax = 127;
float turnMultiplexer = 6.0; // PD parameetrid max kiiruse korral (P = 6 ja D = 4.5
float IMultiplexer = 0;
float DMultiplexer = 4.5;
int lastError = 0;
int error = posCenter;
float Pturn = 7.6;
float Iturn = 0;
float Dturn = 4.4;
long elapseTime = 0;
unsigned long elapseTimeStart = 0;

void Steering(bool steeringON) {
  if (steeringON){
    if(not isAttached){
      Serial.print("Steering ON");
      aservo.attach(ServoPWM);
      isAttached = true;
    }

    if(bestPos == posCenter){
      elapseTime = 0;
      elapseTimeStart = 0;
    }
    else{
      if (elapseTimeStart == 0){
        elapseTimeStart = millis();
      }
      elapseTime = (millis() - elapseTimeStart) / 1000.0;
    }

    error = bestPos - posCenter;
    Pturn = (turnMultiplexer * (error)); //P kontroller
    Iturn = error * elapseTime * IMultiplexer; //I kontroller
    Dturn = (error - lastError) * DMultiplexer; // D kontroller
    lastError = error;
    //Serial.print("Pturn: ");
    //Serial.print(Pturn);
    //Serial.print("  PMultix: ");
    //Serial.print(turnMultiplexer);
    //Serial.println();

    //Serial.print("Iturn: ");
    //Serial.print(Iturn);
    //Serial.print("  IMultix: ");
    //Serial.print(IMultiplexer);
    //Serial.println();

    //Serial.print("Dturn: ");
    //Serial.print(Dturn);
    //Serial.print("  DMultix: ");
    //Serial.print(DMultiplexer);
    //Serial.println();


    turn = turnCenter - Pturn - Iturn - Dturn;
    String emptyS = "";
    BluetoothSend(turn);
    if(turn > leftMax){turn = leftMax;}
    if(turn < rightMax){turn = rightMax;}
    aservo.write(turn);
    //Serial.print("turn: ");
    //Serial.println(turn);
  }
  else{
    Serial.print("Steering OFF");
    aservo.write(turnCenter);
    delay(50);
    aservo.detach();
    isAttached = false;
  }
}

////////DRIVING///////////////

bool isReverse = false;
bool isStuck = false;
unsigned long startStuck = 0;
int isStuckCount = 0;
float stuckTreshold = 70;
int reverseSpeed = 100;
long stuckTimeTreshold = 1500;
long reverseDuration = 1000;
long reverseStartTime = 0;
long straightReverseTime = 500;
bool doStraightReverse = false;

void getStuck(){
  for (int i = 0; i < NUM_LEDS; i++){
    if(i == NUM_LEDS - 1){
      isStuck = false;
      isStuckCount = 0;
      break;
    }
    else if(SensorLedLine3[i] <= stuckTreshold){
      isStuck = true;
      isStuckCount++;
      break;
    }
  }

  if(isStuckCount == 1){
      startStuck = millis();
      isReverse = true;
      doStraightReverse = true;
  }
  if (millis() > startStuck + stuckTimeTreshold and isStuck == true){
      reverseStartTime = millis();
      Reverse();
  }
}

void Reverse(){
  if (doStraightReverse){
    aservo.write(turnCenter);
    delay(50);
    driving_direction = 1;
    digitalWrite(INA, 0);
    digitalWrite(INB, HIGH);
    analogWrite(MotorPWM, reverseSpeed);
    delay(500);
    doStraightReverse = false;
  }
  while(millis() < reverseStartTime + reverseDuration){
    CameraGetData();
    OneDataLineArray();
    LedCamera();
    BestDirection();
    bestPos = WindowsAmount - bestPos;
    Steering(steeringON);
    isStuck = false;
    isStuckCount = 0;
  }
  Driving(motorON);
}
int original = DrivingSpeed;
int FrontTreshold = 500;
int SideTreshold = 250;
float SensorBasedSpeed = 0;
int speed_value = 150;
int CutoffSpeed = 90;

void VariableSpeed(){
  SensorBasedSpeed = 0;
  for (int z = 0; (NUM_LEDS - 1) > z; z++) { 
    if(z < 6 || z > 11){
      SensorBasedSpeed = SensorBasedSpeed + (min(SensorLedLine3 [z], SideTreshold) / SideTreshold);
    }
    else{
      SensorBasedSpeed = SensorBasedSpeed + (min(SensorLedLine3 [z], FrontTreshold) / FrontTreshold);  
    }
  }
  //Serial.print("Sensor based speed: ");
  //Serial.println(SensorBasedSpeed);
  //Serial.print("BTspeed: ");
  //Serial.println(speed_value);
  DrivingSpeed = (SensorBasedSpeed / 18) * speed_value;
  if(DrivingSpeed < CutoffSpeed){DrivingSpeed = CutoffSpeed;}
  //Serial.print("Drivingspeed: ");
  //Serial.println(DrivingSpeed);
}

void Driving(bool motorON) {
    VariableSpeed();
    driving_direction = 0;
    analogWrite(MotorPWM, DrivingSpeed);
    if (motorON){
      digitalWrite(Enable, 1);
    }
    else{
      digitalWrite(Enable, 0);
    }
    
    if (driving_direction == 0) {
          digitalWrite(INA, HIGH);
          digitalWrite(INB, 0);
      }
    else if (driving_direction == 1) {
          digitalWrite(INA, 0);
          digitalWrite(INB, HIGH);
      }
}

void BatteryDead(){
  while(BatteryNeedsCharging == 1){
    motorON = false;
    Driving(motorON);
    steeringON = false;
    Steering(steeringON);
    Battery();
    kitMode = true;
    kit(false, true, 0L);
  }
}

//////////////BUTTONS//////////////
ezButton LeftButton(SW2);  
ezButton RightButton(SW1);
ezButton BumperSwitch(Bumper); 
unsigned long LeftCount = 0;
unsigned long RightCount = 0;
unsigned long BumperCount = 0;
int LeftButtonState = 0;
int RightButtonState = 0;
int BumperSwitchState = 0;

void Buttons() {
  LeftButton.loop(); // MUST call the loop() function first
  RightButton.loop(); // MUST call the loop() function first
  BumperSwitch.loop(); // MUST call the loop() function first
  
  LeftCount = LeftButton.getCount();
  RightCount = RightButton.getCount();
  BumperCount = BumperSwitch.getCount();

  LeftButtonState = LeftButton.getState();
  RightButtonState = RightButton.getState();
  BumperSwitchState = BumperSwitch.getState();
}

void PrintButtonData(){
  Serial.print("Left button state: ");
  Serial.println(LeftButtonState);
  Serial.print("Left button COUNT: ");
  Serial.println(LeftCount);
  Serial.print("Right button state: ");
  Serial.println(RightButtonState);
  Serial.print("Right button COUNT: ");
  Serial.println(RightCount);
  Serial.print("Bumper state: ");
  Serial.println(BumperSwitchState);
  Serial.print("Bumper COUNT: ");
  Serial.println(BumperCount);

  if(LeftButton.isPressed()){Serial.println("The Left button is pressed");} 
  if(LeftButton.isReleased()){Serial.println("The Left button is released");}
  if(RightButton.isPressed()){Serial.println("The Right button is pressed");}
  if(RightButton.isReleased()){Serial.println("The Right button is released");}
  if(BumperSwitch.isPressed()){Serial.println("The Bumper is pressed");}
  if(BumperSwitch.isReleased()){Serial.println("The Bumper is released");}
}

void ButtonsMotorServoOnOff(){
  if(LeftButton.isPressed()){
    steeringON = not steeringON;
    Steering(steeringON);
  }
  if(RightButton.isPressed()){
    motorON = not motorON;
    Driving(motorON);
  }
}

void startButton(){
  if(LeftButton.isPressed()){
    kitMode = true;
    if(!once){
      kit(true, false, 5000);
      motorON = not motorON;
      steeringON = not steeringON;
    }
    else{
      while(true){
        BatteryDead();      //Lylitab servo ja veomootori valja kui aku pinge on liiga madal voi kui aku pole yhendatud
        Bluetooth();        //Loob yhenduse nutiseadmega bluetooth abil andmete saatmiseks
        Driving(motorON);   //Kaivitab veomootori eelnevalt maaratud konstantse kiirusega
        CameraGetData();    //Kysib kaugusanduritelt uued andmed
        OneDataLineArray(); //Teeb kaugusandurite info pohjal yhe horisontaalse rea maatriksi
        //PrintAllCameraData(); //Kuvab arvuti ekraanile reaalajas koikide kaugusandurite andmed
        LedCamera();        /*Kaugusandurite info kuvatakse LED ribale, kus iga LED tahistab yhte anduri pikslit, mida 
                             //lahemal on objekt seda punasem ja mida kaugemal on ojekt seda rohelisem on see konkreetne LED*/
        BestDirection();    //Leiab andurite info pohjal parima suuna kuhu soita
        Steering(steeringON); //Parima suuna pohjal keeratakse rattad soovitud liikumissuunda
        getStuck();         //Kontrollitakse, kas robot on kinni jaanud. Sellisel juhul tagurdab ja keerab sinna kus on rohkem ruumi
        
        if(millis() > (LastDisplayTime + DisplayShowInterval)) { //Kontrollib iga kindla aja tagant aku seisundit
          LastDisplayTime = millis();
          Battery();
        }
      }
    }
  }
  if(RightButton.isPressed()){
    motorON = not motorON;
    Driving(motorON);
  }
}

////////////BLUETOOTH//////////

void bluetoothActions(String msg) {
    data = ""; //clear the buffer/message
    if (msg == "kit"){
      kitMode = not kitMode;
        kit(false, false, 0L);
    }
    else if (msg == "steering"){
      steeringON = not steeringON;
      Steering(steeringON);
    }
    else if (msg == "motor"){
      motorON = not motorON;
      Driving(motorON);
    }
    else if (msg.indexOf("speed:") >= 0){
      speed_value = atoi(msg.substring(6).c_str());
      Serial.println();
      Serial.print("Kiirus: ");
      Serial.print(speed_value);
      DrivingSpeed = float(speed_value);
      analogWrite(MotorPWM, DrivingSpeed);
    }
    else if (msg.indexOf("TP:") >= 0){
      float P_value = msg.substring(3).toFloat();
      Serial.println();
      Serial.print("P_value: ");
      Serial.print(P_value);
      turnMultiplexer = P_value;
    }
    else if (msg.indexOf("TI:") >= 0){
      float I_value = msg.substring(3).toFloat();
      Serial.println();
      Serial.print("I_value: ");
      Serial.print(I_value);
      IMultiplexer = I_value;
    }
    else if (msg.indexOf("TD:") >= 0){
      float D_value = msg.substring(3).toFloat();
      Serial.println();
      Serial.print("D_value: ");
      Serial.print(D_value);
      DMultiplexer = D_value;
    }
}

////////////SETUP//////////////
void setup() {

    Serial2.begin(9600);
    Serial.begin(115200);
    scanner.Init();
    
    //Servo
    //aservo.attach(ServoPWM);

    //IMU
    pinMode(INTimu, INPUT_PULLUP);
    Wire2.begin();
    Wire2.setSCL(SCL2);                   //change the SCL pin
    Wire2.setSDA(SDA2);                   //change the SDA pin
    Wire2.setClock(100000);              //change I2C 2 speedF
    mpu6050.writeMPU6050(56, 1);
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);
    OLED();
    
    //Buttons init
    LeftButton.setDebounceTime(50); // set debounce time to 50 milliseconds
    RightButton.setDebounceTime(50); // set debounce time to 50 milliseconds
    BumperSwitch.setDebounceTime(50); // set debounce time to 50 milliseconds
    LeftButton.setCountMode(COUNT_FALLING);
    RightButton.setCountMode(COUNT_FALLING);
    BumperSwitch.setCountMode(COUNT_FALLING);

    //Camera 1,2,3,4,5,6
    Wire.begin();
    Wire.setSCL(SCL);                   //change the SCL pin
    Wire.setSDA(SDA);                   //change the SDA pin
    Wire.setClock(400000);              //change I2C speed
    Wire1.begin();
    Wire1.setSCL(SCL1);                   //change the SCL pin
    Wire1.setSDA(SDA1);                   //change the SDA pin
    Wire1.setClock(400000);              //change I2C speed
    Serial.println();
    //scanner.Scan();
    CameraSetup();
    CameraInit();
    
    //Encoder 1 & Encoder 2
    pinMode(ENC1, INPUT_PULLDOWN);
    pinMode(ENC2, INPUT_PULLDOWN);

    //Motor Driver
    pinMode(Enable, OUTPUT);
    pinMode(MotorPWM, OUTPUT);
    pinMode(INA, OUTPUT);
    pinMode(INB, OUTPUT);
    pinMode(Current, INPUT);

    //Fill battery arrays with real values
    for(int i = 0; i < LastValues; i++){
      Battery(); 
    }

    //LED strip setup
    FastLED.addLeds<WS2812B, DATA_PIN>(leds, NUM_LEDS);
}

/////////ENDLESS LOOP/////////////////
void loop() {
    //kitMode = true;
    //kit(true, false, 5000);
    Buttons();
    startButton();
    //Interrupts();     //Loeb mootori poordeid
    
    
    //Buttons();          //Kontrolib nuppude seisundit
    //ButtonsMotorServoOnOff(); //Vastavalt nuppude asenditele lylitab servo ja veomootori sisse voi valja
    
}
