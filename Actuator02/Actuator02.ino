// from http://playground.arduino.cc/Main/RotaryEncoders

#include <PID_v1.h>  //PID loop from http://playground.arduino.cc/Code/PIDLibrary

unsigned long previousMillis = 0;
const long interval = 20;

#define encoder0PinA 2
#define encoder0PinB 3

volatile long encoder0Pos = 0;

double Pk1 = 0.35;  //speed it gets there
double Ik1 = 0;
double Dk1 = 0.08;
double Setpoint1, Input1, Output1, Output1a;    // PID variables wheel

int read1;
char ident;

PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);

void setup() {

  Serial.begin (57600);

  PID1.SetMode(AUTOMATIC);              
  PID1.SetOutputLimits(-255, 255);
  PID1.SetSampleTime(20);

  pinMode(9, OUTPUT);   //  motor PWMs
  pinMode(10, OUTPUT);

  pinMode(encoder0PinA, INPUT_PULLUP); 
  pinMode(encoder0PinB, INPUT_PULLUP); 
 
// encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, doEncoderA, CHANGE);
  
// encoder pin on interrupt 1 (pin 3)
  attachInterrupt(1, doEncoderB, CHANGE);  
    
}

void loop(){ //Do stuff here

       unsigned long currentMillis = millis();
       if (currentMillis - previousMillis >= interval) {  // start timed event
          previousMillis = currentMillis;

          Serial.print(encoder0Pos); 

          if (Serial.available()>2) {  // check for enough serial data
              // read the incoming byte:
              read1 = Serial.parseInt();
              ident = Serial.read();         

              if (ident == 'a') {
                  Setpoint1 = read1;
              }
              
          } // end of serial read

          Input1 = encoder0Pos;
          PID1.Compute();

          Serial.print(" , ");
          Serial.println(Output1);

          // drive motor
          
          if (Output1 < 0)                                       // decide which way to turn the motor
          {
            Output1a = abs(Output1);
            analogWrite(10, Output1a);                           // set PWM pins 
            analogWrite(9, 0);
          }
          else if (Output1 > 0)                                  // decide which way to turn the motor
          { 
            Output1a = abs(Output1);
            analogWrite(9, Output1a);  
            analogWrite(10, 0);
          } 
          else
          {
            analogWrite(9, 0);  
            analogWrite(10, 0);
          }                  
             
                      
      }      // end of timed event
  
}  // end of main loop



// encoder 1

void doEncoderA(){  

  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinB) == HIGH) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  } 
}

void doEncoderB(){  

  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinA) == LOW) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  } 
}







