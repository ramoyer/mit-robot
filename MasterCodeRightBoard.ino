#include <QTRSensors.h>
#include <SoftwareServo.h>

/*
 TO DO: nothing!
*/
#define K 2// base turn rate, in PWM form. 4º for right board, -4º for left  Altering 5-03 to 2 for testing.  2 goes for final version
#define C 3 // turn rate correction.  3 worked well on the 4th, 1 on the 5th
#define EDGE_FOLLOW_DELAY 10// delay in ms for line finding loop
#define T_0 4000// wait time before starting.  Light on pin 13 flashes here. 
#define T_1 1800// time for dead reckoning travel
#define T_2 2000// total edge finding time in ms
#define T_3 4000// MCM ramp up time
#define T_4 15000// Full power time at turbine
#define T_5 150 // Back up time to release from turbines
#define SIDE 1     // TENTATIVE: Right side of board: -1  Left side of board: 1   
#define LED_PIN 13 //Set 13 as LED_PIN

#define NUMBER_OF_SENSORS 1
#define IR_HIGH 600// Calibrated min value
#define IR_LOW 20// Calibrated max value
#define FORWARD 169  //1260
#define REVERSE 11 //1760
#define AFORWARD 65// Autonomous forward speed in degrees
#define AREVERSE 113// Autonomous reverse speed in degrees
#define TFORWARD 78// Turbine spinning forward speed (76,8 gives slight right towards end)
#define DIFFERENTIAL 6// turn differential in degrees during turbine spin up. 8
#define NEUTRAL 90  //1510
#define SERVORPIN 6 // Right Servo
#define SERVOLPIN 7 // Left Servo
#define MCM_IN 8 // Control signal from RC for MCM
#define MCM_OUT 9 // to MCM
#define PHOTORESISTOR 0 // photoresistor pin
#define PHOTORESISTOR_TRIGGER 950// value at which to start run.  Input ranges from 0-1023


unsigned int sensors[NUMBER_OF_SENSORS];
unsigned char readMode = QTR_EMITTERS_ON;
int counter;
int duty = 0;
int pulse = 0;
float delta = 0;// Reading change from center value, range from -TURN_RATE to TURN_RATE
float old_delta = 1;// Last reading of delta
float k = 1;
boolean dontStart = true;
// create servo class
SoftwareServo servoL;        // initialize left servo
SoftwareServo servoR;        // init right servo
// create a digital IR sensor object, set on pins 14,15.  2 pins.
QTRSensorsRC qtr((unsigned char[]) {3}, NUMBER_OF_SENSORS);



void setup() {
  // setup servos
  pinMode (SERVOLPIN, OUTPUT);
  pinMode (SERVORPIN, OUTPUT);
  servoL.attach(SERVOLPIN);   
  servoR.attach(SERVORPIN);
  servoL.write(NEUTRAL);     // set servo to neutral  
  servoR.write(NEUTRAL);    // set servo to neutral
                 
  Serial.begin(57600);
  counter = 0;
  duty = 0;
  k = 0.5;
}

void loop() {
  // Wait for LED start light:
  while(dontStart){
    // check photoresistor
    if(analogRead(PHOTORESISTOR)>PHOTORESISTOR_TRIGGER){
      dontStart = false;
    }
    delay(100);
    Serial.println("waiting for LIGHT!!!");
    Serial.println("IR Value: ");
    //Serial.println(analogRead(PHOTORESISTOR));
    qtr.read(sensors,readMode);
    Serial.print(sensors[0]);
    
  }

  for(int i = 0; i < 10; i++){
    delay(T_0/20);
    digitalWrite(LED_PIN,HIGH);
    delay(T_0/20);
    digitalWrite(LED_PIN,LOW);
  }
  
  // Travel forwards to edge
  for(int i = 0; i < 50; i++){
    SoftwareServo::refresh();
    servoR.write(AFORWARD-C);//REMOVED -3
    servoL.write(AFORWARD+C);// removed +3
    delay(T_1/50);
  }
  
  
  while(counter*EDGE_FOLLOW_DELAY < T_2){
    //Serial.println("Dunno what I'm doing, but I SHOULD be running the wheels!!!");
    old_delta = delta;
    qtr.read(sensors,readMode);  // sensors[0] = value of sensor 0 between 1 and 4000, raw data

    // This last section added late.  Delete if needed.  Sets sensor value to the expected range if the value is below expected.
    if(sensors[0]< IR_LOW){
      sensors[0] = IR_LOW; 
    }
    //Serial.println(sensors[0]);
    

    
    delta = map(sensors[0],IR_LOW,IR_HIGH,-K,K);
  
    if(old_delta*delta<0){
      k = 0.5*k;  //Reduce k by half and switch direction if edge is crossed  
    }
    else{
      k = 1.1*k; // Else, speed up the turn by multiplying k
    }

    if(abs(k)>3){
      k = 2;
    }

    
    
    SoftwareServo::refresh();
    servoR.write(int(AFORWARD-C+k*delta));
    servoL.write(int(AFORWARD+C-k*delta));
    delay(EDGE_FOLLOW_DELAY);
    counter++;
    //digitalWrite(LED_PIN,HIGH);
    Serial.println(delta);
    Serial.println(k);
  }

  // Spin Turbine: 
  
  for(int i = 50; i < 100; i++){
    analogWrite(MCM_OUT,2.55*i);
    SoftwareServo::refresh();
    servoR.write(TFORWARD+DIFFERENTIAL);// Left servo should be going faster, but motors are mounted "backwards"
    servoL.write(TFORWARD-DIFFERENTIAL);
    delay(T_3/50);
  }
  
  //delay(T_4);
  analogWrite(MCM_OUT,255);
  for(int i = 1; i < 100; i++){
    SoftwareServo::refresh();
    servoR.write(TFORWARD+DIFFERENTIAL);// Left servo should be going faster, but motors are mounted "backwards"
    servoL.write(TFORWARD-DIFFERENTIAL);
    delay((T_4)/100);
  }
  for(int i = 0; i < 50; i++){
    SoftwareServo::refresh();
    servoL.write(AREVERSE);     // set servo to forward.  
    servoR.write(AREVERSE);    // set servo to mid-point
    delay(T_5/50);
  }
  
  // switch to RC control:
  digitalWrite(SERVORPIN,LOW);
  digitalWrite(SERVOLPIN,LOW);
  pinMode(SERVORPIN,INPUT);
  pinMode(SERVOLPIN,INPUT);
  
  while(true){
    // recieve input from digital pin, write to MCM_OUT
    pulse = pulseIn(MCM_IN,HIGH);

    // If pulse is out of normal range, set to normal range enpoints.
    if(pulse<1550){
      pulse = 1550;
    }
    if(pulse>1800){
      pulse = 1800;
    }
  
    duty = map(pulse,1550,1800,0,255);
    analogWrite(MCM_OUT,duty);
    SoftwareServo::refresh();
    delay(20);
    //Serial.print("Pulse In: ");
  //Serial.println(pulse);
  //Serial.print("Duty: ");
  //Serial.println(duty);
  
  }
  
}
