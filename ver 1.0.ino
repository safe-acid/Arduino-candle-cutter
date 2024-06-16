// vsersion 003
//RS485
//DI  TX0
//DE  3
//RE  2
//R0  RX0
#include <ModbusMaster.h>
#define MAX485_DE      3
#define MAX485_RE_NEG  2
// instantiate ModbusMaster object
ModbusMaster node;
void preTransmission() .  {digitalWrite(MAX485_RE_NEG, 1);digitalWrite(MAX485_DE, 1);}
void postTransmission()   {digitalWrite(MAX485_RE_NEG, 0);digitalWrite(MAX485_DE, 0);}
//RS485
volatile unsigned int temp; //This variable will increase or decrease depending on the rotation of encoder
int counter = 1040;
volatile int candleLenght = 0; // Lenght -> signal  for cutting candle
int RELAY1 = 7; // relay for cutting solenoid


int y; // var for operate solenoid on/off
volatile int switchPin8, switchPin9;
void setup() {
  //RS 485
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
  //RS 485

  Serial.begin (9600);
  pinMode(RELAY1, OUTPUT);
  digitalWrite(RELAY1, HIGH);  // initiate solenoid OFF
  pinMode(2, INPUT_PULLUP); // internal pullup input pin 2
  pinMode(3, INPUT_PULLUP); // internal pullup input pin 3
  //Setting up interrupt A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2.
  attachInterrupt(0, ai0, RISING);
  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3.
  attachInterrupt(1, ai1, RISING);
  // Modbus slave ID 1
  node.begin(1, Serial);
  // Callbacks allow us to configure the RS485 transceiver correctly
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);


  //getParams_P00();
  readMotorParams();
  writeMotorParams();
  //getParams_P02();

}

void loop() {

  // Send the value of counter
  if ( counter != temp ) {
    candleLenghtButton();
    temp = counter;
  }

}


/***********************************
 ******* CANDLE L BUTTONS ********
 ***********************************/
void candleLenghtButton() { // to be adjusted via analogue value later
  candleLenght = 1040;
}


/***********************************
 ******* ANGLE ENCODER *************
 ***********************************/
void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if (digitalRead(3) == LOW) {
    counter++;
    Serial.println(counter);
    if (counter % candleLenght == 0 ) { // activate cutting solenoid @ candleLenght
      y = !y;
      cutCandle(y);//cut candle function

    }
  } else {
    //counter--; //we do not want count anticlockwise
  }
}

void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if (digitalRead(2) == LOW) {
    //counter--; //we do not want count anticlockwise
  } else {
    counter++;
    if (counter % candleLenght == 0 ) { // activate cutting solenoid @ candleLenght
      y = !y;
      cutCandle(y);//cut candle function

    }
  }
}


/***********************************
 ******* CUT CANDLE SOLENOID *******
 ***********************************/
int cutCandle(volatile unsigned int x) {
  if (x == 1) {
    digitalWrite(RELAY1, HIGH);  //solenoid OFF ->immitation of signal or use solenoid
    runMotor();//cut candle motor ON
    delay(1); //avoid lagging
    Serial.println("activate cutting solenoid OFF");
    counter = 0;
    Serial.println(x + 10);


  }
  if (x == 0) {
    digitalWrite(RELAY1, LOW);  //solenoid ON ->immitation of signal or use solenoid
     runMotor();//cut candle motor ON
    delay(1); //avoid lagging
    Serial.println("activate cutting solenoid ON");
    counter = 0;
    runMotor();
    Serial.println(x);

  }

}

/***********************************
 ******* MOTOR DATA *******
 ***********************************/

void readMotorParams() {
  uint8_t result, j;
  uint16_t data[6];
  result = node.readHoldingRegisters(0x387, 1); // Control mode selection
  if (result == node.ku8MBSuccess)
  { Serial.println("***P00 PARAMETERS***" );
    for (j = 0; j < 1; j++) {
      data[j] = node.getResponseBuffer(j);
      Serial.print("Control Mode selection P00.04 =>"); Serial.print(j); Serial.println(data[j]);
    }
  } delay(1200);
}

void writeMotorParams() {
  //901 = 0x385
  //902 = 0x386
  //903 = 0x387 RPM
  uint8_t result, j;
  uint16_t data[6];
  result = node.writeSingleRegister(0x4, 8); delay(100);
  result = node.writeSingleRegister(0x385, 0); delay(100);
  result = node.writeSingleRegister(0x386, 1000); delay(100);
  result = node.writeSingleRegister(0x387, 300); delay(100);

}

void runMotor(){
  uint8_t result;
  uint16_t data[6];
  //900 start 1, stop 0
  result = node.writeSingleRegister(0x384, 1);
  Serial.println("Cut Signal for MOTOR => CUT");
  }
