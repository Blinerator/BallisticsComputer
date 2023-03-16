#define SERIAL_INTERVAL 500
#define SERIAL_INTERVAL2 400

#include <Encoder.h>
#include <Wire.h>
#include <MPU6050.h>
MPU6050 mpu;
Encoder myEnc(A4, A5);
long long int oldPosition=100;

unsigned long long t1 = 0, t2=0;

int joystick_enable = 0, automated_motor_enable = 0, mp = 0, yaw = 0, pitch = 0;
int accel_vals[2];

const int dirPin = 13, stepPin = 12, stepsPerRevolution = 2000;
const int LNup = 6, LNdn = 5;  //linear actuator pins
const int vry = A0, vrx = A1;
const int motor_enable = 4, FIRE=0, Data_Trnsmt=8;

float degrees = 0, v=0;

String incoming, command_array[4];

void setup() {
  accel_vals[0]=-15000;

  Serial.begin(115200);
  Serial.setTimeout(0.2);

  Serial.println("Initialize MPU6050");

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  pinMode(motor_enable, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(LNup, OUTPUT);
  pinMode(LNdn, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(FIRE, INPUT_PULLUP);
  pinMode(Data_Trnsmt, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);

  read_pitch();
}

void loop() {
  // Serial.println(digitalRead(1));
  // Serial.println(digitalRead(11));
  // delay(100);
  // Serial.print("Number 1:");
  // Serial.println(digitalRead(1));
  // Serial.print("Number 2:");
  // Serial.println(digitalRead(2));
  //Serial.println(digitalRead(8));
  if (millis() >= t1) {
    t1 += SERIAL_INTERVAL;
    int stringStart = 0, arrayIndex = 0;

    if(digitalRead(Data_Trnsmt)==LOW){
      int avg_r=0;
      for(int i=0; i<10; i++){//finding the average of ten roll measurements
        read_pitch();
        delay(50);
        avg_r=(avg_r+accel_vals[1])/(i+1);//average
      }


      //arduino sends pitch/roll to main pc twice/second   
      Serial.print(accel_vals[0]);
      Serial.print(",");
      Serial.print(avg_r);
      Serial.println(",");
    }
    // Serial.println(v);
    int initial_pitch=accel_vals[0];
    int initial_yaw=read_yaw();

    if (Serial.available() > 0)
      incoming = Serial.readString();
      for (int i = 0; i < incoming.length(); i++) {
        if (incoming.charAt(i) == ',') {
          command_array[arrayIndex] = "";
          command_array[arrayIndex] = incoming.substring(stringStart, i);
          stringStart = (i + 1);
          arrayIndex++;
        }
        yaw = command_array[0].toInt();
        pitch = command_array[1].toInt();  
        joystick_enable = command_array[2].toInt();
        automated_motor_enable = command_array[3].toInt();
      }
  }
  if (joystick_enable == 0) {
    manual_control();
  } 
  if (automated_motor_enable == 1) {
      buzzer();
      set_yaw(read_yaw(),yaw);
      for(int i=0;i<2;i++){//how many iterations to run on the angle adjustment.  more iterations may mean more accurate
        read_pitch();
        int pitch_i = accel_vals[0];  //current pitch
        int pitch_f = pitch;               //final pitch
        set_pitch(pitch_i, pitch_f);
        analogWrite(LNup,0);
        analogWrite(LNdn,0);
        delay(2000);
      }
    buzzer_f();
    buzzer_f();  
    Serial.println(accel_vals[0]);   
  }
  if(digitalRead(FIRE)==LOW)
  fire();
}

void set_pitch(int pitch_i, int pitch_f){
  if(pitch_f<100)
    return;
  if(pitch_f>15000)
    return;
  int pi_rnd=round_int(pitch_i,100);
  int pf_rnd=round_int(pitch_f,100);
  if(pi_rnd!=pf_rnd){//is current equal to final?
    signed long int dTheta=abs(pitch_i-pitch_f); //the range over which the angle will be adjusted
    signed long int dTheta_c=dTheta; //initializing current error to be dTheta.  This value will change as angle is adjusted
    int PWMm=map(dTheta,0,16384,100,200); //calculating maximum PWM.  Note 50 is the lowest possible PWM max.  16384 = 90 degrees, a.k.a. the maximum delta theta.
    int pitch_c=pitch_i;//initializing current pitch to initial pitch
    int fi=0;
    int PWM=0;
    while(dTheta_c > 10){
      fi=abs(dTheta-dTheta_c);//starts at zero, grows until reaches max value then recedes back down
      PWM=PWMm*sin(fi*3.1415/dTheta)+30; //dTheta defines period, fi defines where in the period we are
      if(PWM<0)//error checking and ensuring slow adjtment
      return;
      if(dTheta<1000)
      PWM=25;
      if(pitch_c-pitch_f<0){
      analogWrite(LNup,PWM);
      analogWrite(LNdn,0);
      }
      else{
      analogWrite(LNdn,PWM);
      analogWrite(LNup,0);
      }

      read_pitch();
      pitch_c=(accel_vals[0]+pitch_c)/2;
      dTheta_c=abs(pitch_c-pitch_f);
    }
    return;
    }
  }


void set_yaw(int yaw_c, int yaw_f){
  int dTheta = abs(yaw_c-yaw_f);
  int speed = 8000; //higher value gives lower speed.  Keep it between 10000 and 1500.
  while(dTheta!=0){//margin is 5, a.k.a. 0.3 degrees

    if(yaw_c-yaw_f>0)//setting direction
    digitalWrite(dirPin, HIGH);
    else
    digitalWrite(dirPin, LOW);

    digitalWrite(stepPin, HIGH);  //start rotation.  replace this with a function in the future
    delayMicroseconds(speed);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(speed);
    
    yaw_c=read_yaw();
    dTheta=abs(yaw_c-yaw_f);

  }

}

int round_int(int x,int rnd){
  int y=x%rnd;
  if(x>=rnd/2)
  x=x+(rnd-y);
  else
  x=x-y;
  return x;
}
void manual_control() {
  int js = analogRead(vry);
  int jsup = analogRead(vrx);
  digitalWrite(motor_enable, HIGH);
  if (jsup > 590) {
    mp = map(jsup, 590, 1024, 10, 100);
    analogWrite(LNup, mp);
  } else {
    analogWrite(LNup, 0);
  }
  if (jsup < 450) {
    mp = map(jsup, 450, 0, 10, 100);
    analogWrite(LNdn, mp);
  } else {
    analogWrite(LNdn, 0);
  }
  if (js > 590) {
    mp = map(js, 590, 1024, 10000, 1500);
    digitalWrite(dirPin, HIGH);   //set direction
    digitalWrite(stepPin, HIGH);  //start rotation
    delayMicroseconds(mp);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(mp);
  }
  if (js < 450) {
    mp = map(js, 450, 0, 10000, 1500);
    digitalWrite(dirPin, LOW);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(mp);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(mp);
  }
}

long read_yaw() {
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    //degrees = newPosition;
    //degrees = degrees / 2400 * 360; //this encoder has 2400 steps/rev
  }
  return oldPosition;
}

void buzzer() {
  digitalWrite(9, HIGH);
  delay(500);
  digitalWrite(9, LOW);
  delay(500);
}
void buzzer_f(){
  digitalWrite(9,HIGH);
  delay(250);
  digitalWrite(9,LOW);
  delay(250);
}

void read_pitch() {//2
  if (millis() >= t2) { //update array vals if it's been long enough, else last vals are kept
    Vector rawAccel = mpu.readRawAccel();
    int accelerometer_y=rawAccel.YAxis;
    int accelerometer_x=rawAccel.XAxis;

    // if(accelerometer_y<accel_vals[0]-1000) return;
    // else if(accelerometer_y<0) return;
    // else{ 
    accel_vals[0]=accelerometer_y; //pitch
    accel_vals[1]=accelerometer_x;//roll
    // }
return;
}
}

void fire(){
  buzzer_f();
  buzzer_f();  
  delay(500);
  digitalWrite(7,HIGH);
  if((digitalRead(1)==HIGH)&&(digitalRead(11)==HIGH)){
    delay(10);
    take_v();
    digitalWrite(7,LOW);
  }
  else{
    delay(250);
    digitalWrite(7,LOW);
  }
}

void take_v(){
  unsigned long int t=0;
  v=0;
  bool passed_first=false;
  while(1==1){
    bool r1=digitalRead(11);
    bool r2=digitalRead(1);
    if((r1==LOW)&&passed_first==false){
      //Serial.println("First");
      t=micros();
      //Serial.println(t);
      passed_first=true;
    }
    else if((r2==LOW)&&passed_first==true){
      //Serial.println("Sec");
      t=micros()-t;
      //Serial.println(t);
      v=71.00/t*1000;
      Serial.println(v);
      return;
    }
  }
}
