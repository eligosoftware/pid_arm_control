// importing libraries
#include "MPU9250.h" // sensor library
#include "math.h"  // math operations

//define pins
#define MOTOR D3   // pin for motor control
#define PIN_POT  A0  //pin for potentiometer value reading

MPU9250 mpu;  // sensor instance

// initial control values
float kp=6.8;
float ki=0.1;
float kd=1.8;
float multiplier=1; // multiplier variable is used to magnitude the P I D values at the same time by the same factor
float error;
float ki_error_range=10;
float desired_roll=38.0;
float pError=0.0;
float current_roll=0.0;
float PID_p, PID_i, PID_d, PID_total;
// time parameters for setting the frequency of reading sensor values
int period = 50; // milliseconds    
float tme;

// serial input value
String serialInput;


void setup() {
  Serial.begin(115200); 
  Wire.begin();
  // connection to MPU sensor
  if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example. Trying to reconnect...");
//            delay(5000);
          if (mpu.setup(0x68)){
            break;
          }
        }
      
        
    }

  // motor and potentiometer to output and input
  pinMode(MOTOR, OUTPUT);
  pinMode(PIN_POT, INPUT);
  // set desired roll to the value, read from potentiometer
  set_desired_roll();
  Serial.println("Setup finished");
  tme=millis(); 
}


void set_desired_roll(){
  // -55 lowest, 20 max, 0 center, total 75
  // read potentiometer value, range is [1024-10]
  int rot_1024= analogRead(PIN_POT);
  // convert to 75 units system
  int rot_75 = 75*(1024 - rot_1024)/1014;
  // set desired roll
  desired_roll=rot_75-55;
  }

void loop() {
  // set desired yaw in accordance to the last read from potentiometer
  set_desired_roll();  

  // read input from serial monitor
  // format: <variable>=<float value>
  // example: kp=1.5
  if (Serial.available()> 0){ // check if there is an input
    
      serialInput = Serial.readString(); //read input as a string
      int index = serialInput.indexOf('='); // find index of =
      String variable = serialInput.substring(0,index); // find the first part of substring, meaning the variable name
      float value = serialInput.substring(index+1, serialInput.length()).toFloat(); // find the second part of substring, meaning the variable value and convert it to float
    // check variable name and assign the value to the corresponding variable
    if (variable=="kp"){
        kp=value;
    }
    else if (variable=="ki"){
        ki=value;
    }

    else if (variable=="kd"){
        kd=value;
    }
    else if (variable=="kier"){
        ki_error_range=value;
    }
    }

    // check the sensor data
  if (mpu.update()) {     
        if (millis() > tme + period) { // if more than period seconds passed since last read
          
            tme=millis(); // set tme variable to current time in milliseconds

            // read current roll angle
             current_roll=mpu.getRoll();
             
             // error calculation 
            error=desired_roll-current_roll;
                    
            // P calculation    
            PID_p = kp * multiplier* error;

            // I calculation
            // I component starts to accumulate and hence to affect the PID total only if it
            // is in range of ki error range 
            if(abs(error) < ki_error_range){
            PID_i = PID_i + (ki *multiplier* error);
            
            } else { // else it is set to zero
              PID_i=0;
           }

            // D calculation
            // pError is previous value of error
            PID_d = kd*multiplier*((error - pError)/(period));

            // Total PID calculation
            PID_total = PID_p + PID_i + PID_d;
            
            // trim the PID value if it is outside of [0-255] range
            
            if (PID_total > 255){
              PID_total =255;
            }

            if (PID_total < 0){
              PID_total =0;
            }

            // print PID and other variables' values
            print_pid();

            // send final PID value to motor
            analogWrite(MOTOR,PID_total);

            // set pError value to current error value
            pError = error;
    }
}
}

// print variable values to Serial Monitor
void print_pid() {
    Serial.print("Current Roll: ");
    Serial.println(current_roll, 2);
    
    Serial.print("Desired Roll: ");
    Serial.println(desired_roll, 2);
    Serial.print("Absolute error: ");
    Serial.println(abs(error), 2);
    Serial.print("KP ki ki_error_range kd: ");
    Serial.print(kp);
    Serial.print(" ");
    Serial.print(ki);
    Serial.print(" ");
    Serial.print(ki_error_range);
    Serial.print(" ");
    Serial.println(kd);
    
    Serial.print("PID_Total, P, I, D: ");
    Serial.print(PID_total, 2);
    Serial.print(", ");
    Serial.print(PID_p, 2);
    Serial.print(", ");
    Serial.print(PID_i, 2);
    Serial.print(", ");
    Serial.println(PID_d, 2);
}

// not used
// sending values to PC
// may be useful in future
void sendToPC(int* data)
{
  byte* byteData = (byte*)(data);
  Serial.write(byteData, 2);
}

void sendToPC(float* data)
{
  byte* byteData = (byte*)(data);
  Serial.write(byteData, 4);
}
