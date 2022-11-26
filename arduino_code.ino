#define SERIAL_INTERVAL 50
#define SERIAL_INTERVAL2 400

#include <Encoder.h>
#include <Wire.h>

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
long long int oldPosition=100;
const int dirPin = 13;
const int stepPin = 12;
const int stepsPerRevolution = 2000;
const int LNup = 6, LNdn = 5;  //linear actuator pins
const int vry = A0;
const int vrx = A1;
const int button = 7;
const int motor_enable = 4;
int mp = 0;
int joystick_enable = 0, automated_motor_enable = 0;
float degrees = 0;
int yaw = 0, pitch = 0;
unsigned long long t1 = 0, t2=0;
String incoming, command_array[4];
Encoder myEnc(A4, A5);
int accel_vals[2];
void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  Serial.begin(115200);
  Serial.setTimeout(0.2);

  pinMode(motor_enable, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(LNup, OUTPUT);
  pinMode(LNdn, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(7,OUTPUT);
}

void loop() {
  if (millis() >= t1) {
    t1 += SERIAL_INTERVAL;
    int stringStart = 0, arrayIndex = 0;
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
    delay(1000);
    digitalWrite(7,HIGH);
    delay(500);
    digitalWrite(7,LOW);    
  }
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
    t2 = millis() + SERIAL_INTERVAL2;  
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
    Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
    Wire.requestFrom(MPU_ADDR, 2*2, true); // request a total of 3*2=6 registers
    // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
    accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
    accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
    //Serial.println(accelerometer_y);
    if(accelerometer_y<accel_vals[0]-1000){
      read_pitch();//yes, this is a terrible idea
    }
    else if(accelerometer_y<0){
      read_pitch(); //oh boy...  what could go wrong here...
    }
    else{ 
    accel_vals[0]=accelerometer_y; //pitch
    accel_vals[1]=accelerometer_x;//roll
    }
    //Serial.print("aX = "); Serial.print(convert_int16_to_str(accelerometer_x));
    //Serial.print(" | aY = "); Serial.print(convert_int16_to_str(accelerometer_y));
    //Serial.print(" | aZ = "); Serial.print(convert_int16_to_str(accelerometer_z));
    //Serial.println();
}
return;
}
