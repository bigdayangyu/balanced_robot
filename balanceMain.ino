#include <MatrixMath.h>
#include <Wire.h>
#include <ADXL345.h>
#include <stdarg.h>
ADXL345 accelerometer;
/*-------Encoder initialization------*/
int pin1 = 2;
int pin2 = 3;
int counter;
boolean goingUp = false;
boolean goingDown = false;

/*-------Motor initializationy------*/
//Motor one
int enA = 10;
int in1 = 8;
int in2 = 9;
// motor two
int enB = 5;
int in3 = 6;
int in4 = 7;

/*-------PID initializaiton------*/
double Kp = 25, Ki = 0.1*1, Kd =0.01;
float sampleTime = 0.001;
unsigned long timer = millis(); // Time now c
unsigned long  increment = 0;
unsigned long motorWrite = 0;
float pwmMax = 100; // PWM saturation 
float pwmMin = 0;
float equilibrium_x = 0.0;
float equilibrium_theta = 0.0  ;
float equilibrium_x_dot = 0.0;
float equilibrium_theta_dot = 0.0;

float dt = 0.01;

/*---------pole Placement initialization______*/
unsigned long pre_distance = 0; 
unsigned long pre_froll = 0;
float k1 = -0.05, k2 = -3, k3 = -20, k4 = -3;


/*-------------Setup------------*/
void setup()
{
/*-------motor setup------*/
  // set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
/*-------Encoder setup------*/
  counter = 0;
  //Serial prints for debugging and testing
  Serial.begin(9600);

// Setup encoder pins as inputs 
  pinMode(pin1, INPUT); 
  pinMode(pin2, INPUT);  

// encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, decoder, FALLING);
    
/*-------Accelerometer setup------*/
 // Initialize ADXL345
  Serial.println("Initialize ADXL345");

  if (!accelerometer.begin())
  {
    Serial.println("Could not find a valid ADXL345 sensor, check wiring!");
    delay(500);
  }
  // Set measurement range
  // +/-  2G: ADXL345_RANGE_2G
  // +/-  4G: ADXL345_RANGE_4G
  // +/-  8G: ADXL345_RANGE_8G
  // +/- 16G: ADXL345_RANGE_16G
  accelerometer.setRange(ADXL345_RANGE_16G);
}

/*-------Main loop ------*/
void loop()
{

/*-------Encoder Reading------*/
  while(goingUp==1) // CW motion in the rotary encoder
  {
    goingUp=0; // Reset the flag
    counter ++;
  }

  while(goingDown==1) // CCW motion in rotary encoder
  {
    goingDown=0; // clear the flag
    counter --;
  }
    float distance = 0.21*counter/(334);// distance robot moved 

//    Serial.println(distance);   
 
/*-------Accelerometer Reading------*/
  // Read normalized values
  Vector norm = accelerometer.readNormalize();

  // Low Pass Filter to smooth out data. 0.1 - 0.9
  Vector filtered = accelerometer.lowPassFilter(norm, 0.5);

  // Calculate Pitch & Roll (Low Pass Filter)
  int fpitch = -(atan2(filtered.XAxis, sqrt(filtered.YAxis*filtered.YAxis + filtered.ZAxis*filtered.ZAxis))*180.0)/M_PI;
  int froll  = (atan2(filtered.YAxis, filtered.ZAxis)*180.0)/M_PI;

  Serial.println(froll);
//  Serial.print(" \t");
//  Serial.print(distance);   
//  Serial.println(" ");

/*------------Motor Control ---------------*/

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW); 
//Serial.println(froll);
//Serial.print("\t");
/*------------PID ---------------*/

  if(dt >= sampleTime)
    {

//      float increment = 0;
      // Kp*E + Ki*E*dt + Kd*E/dt
      float error1 = equilibrium_x - distance;
      float error2 = equilibrium_theta - froll;
//      Serial.print("Error");
//      Serial.print("\t");
//      Serial.println(error1);
//      Serial.print("\t");

     float kiTerm = Ki*error2*dt;
     kiTerm = constrain(kiTerm, pwmMin, pwmMax);
     float motorSetting = Kp*error2 + kiTerm + Kd*error2/dt;
//     float motorWrite = increment + motorWrite;
     if(motorSetting >=0)
     {
        if(motorSetting >=  255 )
          {
            motorWrite = 255;
           }
        else{motorWrite = motorSetting;}
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(enA, motorWrite);  
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW); 
      analogWrite(enB, motorWrite);
      delay(10);
       
      }// if motor setting positive

      if(motorSetting < 0)
      {
        motorSetting = -1*motorSetting;
        if(motorSetting >= 255)
         {
           motorWrite = 255;
          }   
        else{motorWrite = motorSetting;}
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(enA, motorWrite);  
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH); 
      analogWrite(enB, motorWrite);
      delay(10);

       }// flip sign for negtive motor write

//     Serial.println(motorWrite);

    }// if dt> sample time 
/*-------------------Pole placement--------------*/
//      float distance_dot = (distance - pre_distance)/dt;
//      float theta_dot = (froll - pre_froll)/dt;
//      float error1 = equilibrium_x - distance;
//      float error2 = equilibrium_x_dot -distance_dot;
//      float error3 = equilibrium_theta - froll;
//      float error4 = equilibrium_theta_dot -theta_dot;
//      pre_distance = distance;
//      pre_froll = froll;
//
//      float increment = 0;
//
//      
// if(dt >= sampleTime)
//   {
//
//      float increment = 0;
//      float motorSetting = k1*error1 + k2*error2 + k3*error3 +k4*error4;
//     if(motorSetting >=0)
//     {
//        if(motorSetting >=  250 )
//          {
//            increment = 250;
//           }
//        else{increment = motorSetting;}
//      digitalWrite(in1, HIGH);
//      digitalWrite(in2, LOW);
//      analogWrite(enA, increment);  
//      digitalWrite(in3, HIGH);
//      digitalWrite(in4, LOW); 
//      analogWrite(enB, increment);
//      delay(10);
//       
//      }// if motor setting positive
//
//      if(motorSetting < 0)
//      {
//        motorSetting = -1*motorSetting;
//      
//        if(motorSetting >= 250)
//         {
//           increment = 250;
//          }   
//        else{increment = motorSetting;}
//      digitalWrite(in1, LOW);
//      digitalWrite(in2, HIGH);
//      analogWrite(enA, increment);  
//      digitalWrite(in3, LOW);
//      digitalWrite(in4, HIGH); 
//      analogWrite(enB, increment);
//      delay(10);
//
//       }// flip sign for negtive motor write
////
////     Serial.println(increment);
//
//    }// if dt> sample time 
//      
//

}//main loop

void decoder()
{

  if (digitalRead(pin1) == digitalRead(pin2))
  {
      goingUp = 1; //if encoder channels are the same, direction is CW
  }
  else
  {
    goingDown = 1; //if they are not the same, direction is CCW
  } 
}


