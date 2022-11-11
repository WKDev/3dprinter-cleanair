#include <Arduino.h>
#include "BluetoothSerial.h"
#include <Preferences.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
int particlePeriod = 0;
String message = ""; // accumulated incoming string
char incomingChar;   // incoming string
float dust = 0;
int working_mode = 0; // working mode : 0 - auto, 1 - low , 2- middle, 3- high

Preferences prfs;

#define DIGITAL_LED_WHITE 26
#define ANALOG_BLACK 25
#define PWM_FAN 13
#define RPM_FAN 27
//
//  DustSensor_Nano_v1.ino (2017.11.13)
//
//  Sensor (GP2Y1010AU) - Arduino Nano
//
//  1 (V_IR LED)        - 150 Ohm & 220 uF
//  2 (GND_IR LED       - GND
//  3 (Drive_IR LED)    - A0
//  4 (GND)             - GND
//  5 (Output)          - A1
//  6 (VCC)             - VCC (5V)
//

int pinDrive = DIGITAL_LED_WHITE; // Sensor IR LED Drive
int pinADC = ANALOG_BLACK;        // Reading Sensor Output
int pinLED = 15;                  // LED Indicattion of Dust Density
boolean pinState = 1;

int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680; // Pulse Interval (0.28 + 0.04 + 9.68 = 10 ms)

unsigned long measureInterval = 20; // Time Interval of Measurement (ms)
unsigned long blinkInterval = 3000; // Time Interval for LED Indicator

unsigned long prevMeasure; // Previous Time of Measure
unsigned long prevBlink;   // Previous Time of LED ON or OFF

float voltageADC = 0.0;      // Instantaneous Voltage (ADC)
float maADC = 0.1030;        // Moving Average Voltage
float voltageClean = 0.1030; // Voltage for Clean Air (Zero Setting)

unsigned long pmOntime;
unsigned long pmOfftime;

unsigned long fanOntime;
unsigned long fanOfftime;

const int pwmPin               = PWM_FAN;
const int pwmFreq              = 25000;
const int pwmChannel           = 0;
const int pwmResolution        = 8;
const int fanMaxRPM            = 2600;  

const int tachoPin                             = RPM_FAN;
const int tachoUpdateCycle                     = 1000; // how often tacho speed shall be determined, in milliseconds
const int numberOfInterrupsInOneSingleRotation = 2;    // Number of interrupts ESP32 sees on tacho signal on a single fan rotation. A

unsigned long lastworked = millis();

uint8_t ctrl_factor = 0;


static volatile int counter_rpm = 0;
int last_rpm = 0;
unsigned long millisecondsLastTachoMeasurement = 0;

// Interrupt counting every rotation of the fan
// https://desire.giesecke.tk/index.php/2018/01/30/change-global-variables-from-isr/
void IRAM_ATTR rpm_fan() {
  counter_rpm++;
}

void initTacho(void) {
  pinMode(tachoPin, INPUT);
  digitalWrite(tachoPin, HIGH);
  attachInterrupt(digitalPinToInterrupt(tachoPin), rpm_fan, FALLING);
  Serial.printf("  Fan tacho detection sucessfully initialized.\r\n");
}

void updateTacho(void) {
  // start of tacho measurement
  if ((unsigned long)(millis() - millisecondsLastTachoMeasurement) >= tachoUpdateCycle)
  { 
    // detach interrupt while calculating rpm
    detachInterrupt(digitalPinToInterrupt(tachoPin)); 
    // calculate rpm
    last_rpm = counter_rpm * ((float)60 / (float)numberOfInterrupsInOneSingleRotation) * ((float)1000 / (float)tachoUpdateCycle);
    // Log.printf("fan rpm = %d\r\n", last_rpm);

    // reset counter
    counter_rpm = 0; 
    // store milliseconds when tacho was measured the last time
    millisecondsLastTachoMeasurement = millis();

    // attach interrupt again
    attachInterrupt(digitalPinToInterrupt(tachoPin), rpm_fan, FALLING);
  }
}

int bittopercent(float percent){

  int ret = int((185.0/100.0)*percent + 60.0);
  // Serial.println("currentworking : " + String ( ret));

  return ret;
}


void setup()
{
  pinMode(pinDrive, OUTPUT);
  pinMode(pinLED, OUTPUT);
  pinMode(PWM_FAN, OUTPUT);


  Serial.begin(115200);

  SerialBT.begin("3DPrinter Air Purifier"); // Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");

  prfs.begin("working-mode", false);
  working_mode = prfs.getUInt("mode");

  Serial.println("Working mode is loaded : " + String(working_mode));

  prevMeasure = millis();
  prevBlink = millis();
  particlePeriod = millis();


   initTacho();
ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(pwmPin, pwmChannel);

  delay(500);
  Serial.println("initiating pwm fan with lowest speed...");
    ledcWrite(pwmChannel, bittopercent(0.0));

    delay(5000);   

      Serial.println("initiating pwm fan done.");




}

void loop()
{
  if (SerialBT.available())
  {
    // Serial.write(SerialBT.read());

    // char incomingChar = SerialBT.read();
    // // while (incomingChar != '\n'){
    // //   Serial.println(incomingChar);
    // //   message += String(incomingChar);
    // // }

    // Serial.write(incomingChar);
    // working_mode = incomingChar;
    // SerialBT.flush();
    // message = "";

    // Serial.println("Working mode is set and stored : " + incomingChar);
    // prfs.putUInt("mode", incomingChar);
    // SerialBT.flush();

    //   if (message =="0"){
    //   // digitalWrite(ledPin, HIGH);

    //   // PWM Setting here
    // }
    // else if(message =="1"){

    // }
    // else if(message =="2"){
    // }
    // else if(message =="3"){
    // }
  }

  int availableBytes = SerialBT.available();

  if (availableBytes)
  {

    for (int i = 0; i < availableBytes; i++)
    {
      int red = SerialBT.read();
      if (red >= 48 && red <= 72)
      {
        red -= 48;
        message += red;

        working_mode = message.toInt();
    SerialBT.flush();
    Serial.println(message);
    Serial.println("Working mode is set and stored : " + String(working_mode));
    prfs.putUInt("mode", working_mode);
    message = "";
            break;

      }
      else{
        break;
      }
    }

    
  }

  if (millis() - prevMeasure > measureInterval)
  {
    prevMeasure = millis();
    
    
    digitalWrite(pinDrive, LOW); // Pulse ON

    // if(millis()-last_millis  == 28 )
    delayMicroseconds(samplingTime); // 0.28 ms

    voltageADC = analogRead(pinADC) * (3.3 / 4095.0); // ADC to Voltage Convertion

    delayMicroseconds(deltaTime); // 0.04 ms
    digitalWrite(pinDrive, HIGH); // Pulse OFF
    delayMicroseconds(sleepTime); // 9.68 ms

    maADC = voltageADC * 0.005 + maADC * 0.995; // Exponential Moving Average

    // Serial.print("* V_instant = ");
    // Serial.print(voltageADC, 4);
    // Serial.print(",   * V_ma = ");
    // Serial.print(maADC, 4);

    dust = (0.17 * maADC) * 1000;

    // Serial.print(",   * dust(ug/m3) = ");
    // Serial.println(dust, 4);
  }

  // sending dust data periodically,
  if ((millis() - particlePeriod) > 500)
  {
    SerialBT.println(String(working_mode)+ ","+ dust);
    particlePeriod = millis();

    Serial.print("* V_instant = ");
    Serial.print(voltageADC, 4);
    Serial.print(",   * V_ma = ");
    Serial.print(maADC, 4);
    Serial.print(",   * dust(ug/m3) = ");
    Serial.println(dust, 4);
  }


  blinkInterval = long(50.0 / (maADC - voltageClean));

  if (blinkInterval > 5000)
  {
    blinkInterval = 5000;
    pinState = 1;
  }

  if (millis() - prevBlink > blinkInterval)
  {
    prevBlink = millis();
    digitalWrite(pinLED, pinState);
    pinState = !pinState;
  }

  updateTacho();

  // Serial.println("last_rpm : " + String(last_rpm));


  if(working_mode == 0){ // off
  ledcWrite(pwmChannel,0);   

  }
  else if(working_mode == 1){ // auto

    if(dust < 30 ){
      ctrl_factor = 0.0;

    }
    else if (dust > 100){
      ctrl_factor = 100.0;

    }
    else{
      float dust_new = dust;

      if (dust_new >= 100.0){
        dust_new = 100.0;
      }
      ctrl_factor = (dust_new / 2.0) + 50.0;
      // Serial.println(ctrl_factor);
    }

    // 30 ~ 100 --> 15 ~ 50

    ledcWrite(pwmChannel, bittopercent(ctrl_factor));   
  }

  else if(working_mode == 2){ // low
  ledcWrite(pwmChannel, bittopercent(0.0));   
  }
  
  else if(working_mode == 3){ // mid
  ledcWrite(pwmChannel, bittopercent(80.0));   
  }
  
  else if(working_mode == 4){ // high
    ledcWrite(pwmChannel, bittopercent(100.0));   
  }
}



// void setup(){

//   Serial.begin(115200);
//   // pinMode(PWM_FAN, OUTPUT);
//   initTacho();
// ledcSetup(pwmChannel, pwmFreq, pwmResolution);
//   ledcAttachPin(pwmPin, pwmChannel);


//   fanOfftime = millis();
//   fanOntime = millis();

//   for(int dutyCycle = 255; dutyCycle >= 0; dutyCycle--){
//     // changing the LED brightness with PWM
//     ledcWrite(pwmChannel, dutyCycle);   
//     delay(15);
//   }
// }

// void loop(){


  // for(int dutyCycle = 0; dutyCycle <= 255; dutyCycle++){   
  //   // changing the LED brightness with PWM
  //   ledcWrite(pwmChannel, dutyCycle);
  //   delay(15);
  // }

  // decrease the LED brightness
  // for(int dutyCycle = 255; dutyCycle >= 0; dutyCycle--){
  //   // changing the LED brightness with PWM
  //   ledcWrite(pwmChannel, dutyCycle);   
  //   delay(15);
  // }
  // delay(1000);

  //   updateTacho();

  // Serial.println("last_rpm : " + String(last_rpm));

  // ledcWrite(pwmChannel, 125);   
  //   delay(15);
  //     ledcWrite(pwmChannel, 125);   

  //         delay(15);




  // Serial.println(String(millis()%3000));

  // if((millis()%10000) < 1500){
  // ledcWrite(pwmChannel, 0);
  // lastworked = millis();
  // delay(1);
  
  // }
  // else if ((millis()%10000) < 3000){
  //     // Serial.println(0);

  //     ledcWrite(pwmChannel, 255);
  //       delay(1);

  // }
// }
  


  // delay(2000);

  // ledcWrite(pwmChannel, 255);

//   delay(2000);

// }








// float dustVal=0;
// float dustDensity = 0;
// int delayTime=280;
// int delayTime2=40;
// float offTime=9680;

// void setup(){
//   Serial.begin(115200);
//   //  pinMode(ANALOG_BLACK,OUTPUT);
//    pinMode(DIGITAL_LED_WHITE, OUTPUT);
//   }

// void loop(){
// digitalWrite(DIGITAL_LED_WHITE,LOW); // power on the LED 
// delayMicroseconds(delayTime);
// //먼지 센서를 값을 0.0V~3.3V 을 0~1024 값으로 변경해 줌. 
// dustVal=analogRead(DIGITAL_LED_WHITE); // read the dust value  
// delayMicroseconds(delayTime2);
// digitalWrite(DIGITAL_LED_WHITE,HIGH); // turn the LED off 
// delayMicroseconds(offTime);
// delay(3000);

// float voltage=0;
// float v0=0;
// float dust=0;// dust = (0.17*voltage)*1000;

// //입력된 볼트를 계산해 줌 
// voltage=dustVal*(3.3/1024);
// // 볼트를 기준으로 미세먼지 값으로 변환해 줌, 데이터 시트을 참고 오차가 존재합니다. 
// dust = (0.17*voltage)*1000;
// // 미세먼지 수치 출력 
// Serial.print("Dust density(ug/m3) = ");
// Serial.print(dust);
// //아날로그로 읽어 들인 0-1024 수치 
// Serial.print(", dustVal = ");
// Serial.print(dustVal);
// // 볼트로 계산해서 출력 
// Serial.print(", voltage = ");
// Serial.println(voltage);
// }