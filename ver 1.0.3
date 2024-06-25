#include <ModbusMaster.h>
 //RS485 //DI  TX0 //DE  pin3 //RE  pin2//R0  RX0
//ENCODER 
//RS485
#define MAX485_DE      3
#define MAX485_RE_NEG  2
// instantiate ModbusMaster object
ModbusMaster node;
void preTransmission()   {digitalWrite(MAX485_RE_NEG, 1);digitalWrite(MAX485_DE, 1);}
void postTransmission()   {digitalWrite(MAX485_RE_NEG, 0);digitalWrite(MAX485_DE, 0);}
uint8_t result,j;
uint16_t data[6];
//RS485
volatile unsigned int temp; //this variable will increase or decrease depending on the rotation of encoder
int counter = 0; // initial point of rotary encoder
int candleLenght = 1640; // Lenght -> signal  for cutting candle
int RELAY1 = 8; // relay for something else
int x; // var for operate RELAY1 on/off
boolean runMotorOneTime = false; //for running motor once


  //LED
  const int ledPin40 =  40; //cut signal millis
  const int ledPin41 =  41; //watchdog green 
  int ledState40 = HIGH; // ledState used to set the LED
  int ledState41 = LOW; // ledState used to set the LED
   long previousMillisWDOG = 0;
   long watchDogInterval = 250;
   long previousMillisCutInterval;
   
   long previousMillisCutLED = 250;
   long CutLedInterval = 250;
  
   
     int S1; //button status for candle L
     int S2; //button status for candle L
     int S3; //button status for candle L

// ENCODER     
// Motor encoder output pulses per 360 degree revolution (measured manually)
#define ENC_COUNT_REV 600
 
// Encoder output to Arduino Interrupt pin. Tracks the pulse count.
#define ENC_IN_RIGHT_A 20
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_RIGHT_B 21

// True = Forward; False = Reverse
boolean Direction_right = true;
 
// Keep track of the number of right wheel pulses
volatile long right_wheel_pulse_count = 0;
 
// One-second interval for measurements
int interval = 00;
  
// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;
 
// Variable for RPM measuerment
float rpm_right = 0;
 
// Variable for angular velocity measurement
float ang_velocity_right = 0;
float ang_velocity_right_deg = 0;
 
const float rpm_to_radians = 0.10471975512;
const float rad_to_deg = 57.29578;
 
void setup() {

 
  // Set pin states of the encoder
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);
 
  // Every time the pin goes high, this is a pulse
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_pulse, RISING);

    //RS 485
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
  //RS 485
  Serial.begin (9600);
  Serial1.begin (38400);
  //Serial1.begin (9600);
  Serial.println("Serail Ready");
  
  pinMode(RELAY1, OUTPUT);
  digitalWrite(RELAY1, HIGH);   //RELAY 1 is OFF

  // Modbus slave ID 1
  node.begin(1, Serial1);
  // Callbacks allow us to configure the RS485 transceiver correctly
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);


   writeMotorParams();
    delay(500);
    readMotorParams();
   delay(500);

//  S1 = digitalRead(31);
//  S2 = digitalRead(32);
//  S3 = digitalRead(33);

//  S1 = 0;
//  S2 = 0;
//  S3 = 0;
}
 
void loop() {
 
  // Record the time
  currentMillis = millis();
   S1 = digitalRead(31);
  S2 = digitalRead(32);
  S3 = digitalRead(33);
  runMotor();
    //  Serial.print(" Pulses: ");
    //Serial.println(right_wheel_pulse_count);
  
//  // If one second has passed, print the number of pulses
//  if (currentMillis - previousMillis > interval) {
// 
//    previousMillis = currentMillis;
// 
//    // Calculate revolutions per minute
//    rpm_right = (float)(right_wheel_pulse_count * 60 / ENC_COUNT_REV);
//    ang_velocity_right = rpm_right * rpm_to_radians;   
//    ang_velocity_right_deg = ang_velocity_right * rad_to_deg;
//     
//   // Serial.print(" Pulses: ");
//    //Serial.println(right_wheel_pulse_count);
////    Serial.print(" Speed: ");
////    Serial.print(rpm_right);
////    Serial.println(" RPM");
////    Serial.print(" Angular Velocity: ");
////    Serial.print(rpm_right);
////    Serial.print(" rad per second");
////    Serial.print("\t");
////    Serial.print(ang_velocity_right_deg);
////    Serial.println(" deg per second");
////    Serial.println();
// 
//    //right_wheel_pulse_count = 0;
//   
//  }

}
 
// Increment the number of pulses by 1
void right_wheel_pulse() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if(val == LOW) {
    
    Direction_right = false; // Reverse
    
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
   
    right_wheel_pulse_count++;

    if (right_wheel_pulse_count %  candyL(S1,S2,S3) == 0 ) { //activate cutCandle for RUN motor
        //if (right_wheel_pulse_count  == candyL(S1,S2,S3) ) { //activate cutCandle for RUN motor
      x = !x;  
      cutCandle(x);//cut candle RUN motor + var y for relay to do something else
    
    }
    
  }
  else {
    //right_wheel_pulse_count--;
  }
}





/*****************************************************************************
 *************************** CUT CANDLE **************************************
 *****************************************************************************/
int cutCandle(unsigned int x) {
  
  if (x == 1) { 
   // Serial.println(counter);
 
     //digitalWrite(RELAY1, HIGH);  //relay for something else OFF
     // Serial.println(currentTime);
     //Serial.println("activate cutting solenoid OFF"); //<--lagging if uncomment
  runMotorOneTime = false;
     counter = 0;
     right_wheel_pulse_count =0;
        
      // Serial.print("MOTOR => CUT ON"); 
  }
  if (x == 0) {
   // Serial.println(counter);
   
    //digitalWrite(RELAY1, LOW);  //relay for something else ON
  
    //Serial.println(currentTime);

    //Serial.println("activate cutting solenoid ON"); //<--lagging if uncomment
    runMotorOneTime = false;
    counter = 0; 
    right_wheel_pulse_count =0;
       
      //Serial.print("MOTOR => CUT ON");   
  
  }

}


/*****************************************************************************
 *************************** MOTOR DATA **************************************
 *****************************************************************************/
void writeMotorParams() { //Setup the motor driver after hard reset
  //308 = 0x0134 fan of driver 1off
  //901 = 0x0385
  //902 = 0x0386
  //903 = 0x0387 RPM
  result = node.writeSingleRegister(0x0004, 8); delay(100);
  result = node.writeSingleRegister(0x0134, 1); delay(200); //fan off
  result = node.writeSingleRegister(0x0134,0); delay(100); //fan on for test RS 485
  result = node.writeSingleRegister(0x0385, 0); delay(100);
  result = node.writeSingleRegister(0x0386, 1000); delay(100);
  result = node.writeSingleRegister(0x0387, 600); delay(100);

}

void readMotorParams() {
  result = node.readHoldingRegisters(0x0386, 1); // Control mode selection
  
    Serial.println("***SOME PARAMETER : ***" );
    for (j = 0; j < 1; j++) {
      data[j] = node.getResponseBuffer(j);
      Serial.print("Control Mode selection P00.04 => "); Serial.print(j); Serial.println(data[j]);
    
  } 
}




void runMotor(){ //MOTOR ON
 
 if (runMotorOneTime == false){  
 
    Serial.println("MOTOR => CUT ON");     
    ledCutSignal();
    ledWatchDog();
    result = node.writeSingleRegister(0x0384, 1);
     node.clearResponseBuffer();//if not clear can lagging
     Serial1.flush();//if not clear can lagging
     runMotorOneTime = true;//stop motor exit from method
     
    //Serial.println(millis()); 
    //delay(25);
    }
  }



  void ledWatchDog(){
    
    if(currentMillis - previousMillisWDOG > watchDogInterval) {
    // save the last time you blinked the LED 
    previousMillisWDOG = currentMillis;   

    // if the LED is off turn it on and vice-versa:
    if (ledState41 == LOW)
      ledState41 = HIGH;
    else
      ledState41 = LOW;

    // set the LED with the ledState of the variable:
    digitalWrite(ledPin41, ledState41);
  }
  }


  
  void ledCutSignal(){
   
   if(currentMillis - previousMillisCutLED > CutLedInterval) {
    // save the last time you blinked the LED 
    previousMillisCutLED = currentMillis;   

    // if the LED is off turn it on and vice-versa:
    if (ledState40 == LOW)
      ledState40 = HIGH;
    else
      ledState40 = LOW;


    // set the LED with the ledState of the variable:
    digitalWrite(ledPin40, ledState40);
  }
  }



/*****************************************************************************
 *************************** CANDLE LENGHT ***********************************
 *****************************************************************************/
  int candyL(int s1, int s2, int s3 ){
    long l;
    
    if(s1 == 0 && s2 == 0 &&  s3 == 0) { // 0-0-0
      l = 1040; }
      
      if(s1 == 1 && s2 == 0 &&  s3 == 0){ // 1-0-0
        l = 1140;}
        
        if(s1 == 1 && s2 == 0 &&  s3 == 1){ // 1-0-1 euro 0.20 dia samij malenijkij No1 + po 0.50 dlinna dia 6mm
          l = 1240;}
          
          if(s1 == 1 && s2 == 1 &&  s3 == 0){ // 1-1-0
            l = 1340;}
            
            if(s1 == 1 && s2 == 1 &&  s3 == 1){ // 1-1-1
              l = 1440;}
              
              if(s1 == 0 && s2 == 1 &&  s3 == 1){ // 0-1-1
                l = 1540;}
                
                if(s1 == 0 && s2 == 0 &&  s3 == 1) {// 0-0-1 euro 1,5 dia 7mm
                  l = 1700;}
                  
                  if (s1 == 0 && s2 == 1 &&  s3 == 0){ // 0-1-0
                    l = 2000;}
                    
                    return l;
  }
 /*****************************************************************************
 *************************** CANDLE LENGHT ***********************************
 *****************************************************************************/
 
