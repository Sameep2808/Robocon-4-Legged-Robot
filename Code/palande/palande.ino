// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge0, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

int Interruptpin=20;
int abc=0;                        
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */



// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT



#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

int stable =1000;
int a1=0;
float diff=0;
int frontRight1=22,frontRight2=23,frontRighten=24,frontRightpwm=6,frontLeft1=25,frontLeft2=26,frontLeften=27,frontLeftpwm=3,backRight1=28,backRight2=29,backRighten=30,backRightpwm=7;
int backLeft1=31,backLeft2=32,backLeften=33,backLeftpwm=5;
int proportional,high_speed=50,high_speed1=70,high_speed2=80,low_speed=25,KP=70,pwm;
int weight_index;
int jc=0,jc1=0,a=0,qq=0;
int stablede = 1200;
int stable1 = 1000;
int weights[13]={0,
                  4,
                  6,
                   8,
                   10,
                   12,
                   15,
                   20,
                   25,
                   20,
                   25,
                   30,
                   35
                        };

void setup() {
 //   attachInterrupt(digitalPinToInterrupt(Interruptpin),hardstop,LOW);
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

                        
    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
     for(byte i=40;i<=47;i++) {
    pinMode(i,INPUT);
  }

  
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    /*// wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
*/
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    pinMode(LED_PIN, OUTPUT);
    
digitalWrite(20,HIGH);
  pinMode(frontRight1,OUTPUT);  
  pinMode(frontRight2,OUTPUT);
  pinMode(frontRighten,OUTPUT);
  pinMode(frontRightpwm,OUTPUT);
  pinMode(frontLeft1,OUTPUT);
  pinMode(frontLeft2,OUTPUT);
  pinMode(frontLeften,OUTPUT);
  pinMode(frontLeftpwm,OUTPUT);
  pinMode(backRight1,OUTPUT);
  pinMode(backRight2,OUTPUT);
  pinMode(backRighten,OUTPUT);
  pinMode(backRightpwm,OUTPUT);
  pinMode(backLeft1,OUTPUT);
  pinMode(backLeft2,OUTPUT);
  pinMode(backLeften,OUTPUT);
  pinMode(backLeftpwm,OUTPUT);
    // configure LED for output
   
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
            stable--;
            if(stable==2)
            {
              a1=ypr[0] * 180/M_PI;
              abc=a1;
              Serial.print(" A1  ");
              Serial.println(a1);
            }
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

        if(stable<0){ 
          Serial.println("Working");
          a=ypr[0] * 180/M_PI;
          Serial.print(a);
          diff=a1-a;
          Serial.print(diff);
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
        
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
   
     Serial.println(" ");
  Serial.print(digitalRead(40));
  Serial.print(digitalRead(41));
  Serial.print(digitalRead(42));
  Serial.print(digitalRead(43));
  Serial.print(digitalRead(44));
  Serial.print(digitalRead(45));
  Serial.print(digitalRead(46));
  Serial.print(digitalRead(47));
  Serial.println();

      
  
   if(digitalRead(40) && digitalRead(41) && digitalRead(42) && digitalRead(43) && digitalRead(44) && digitalRead(45))

{    jc=jc+1;
  
  Serial.println("Junction Count\n");
  //digitalWrite(20,LOW);
  if(jc==1)
  Back();
  Serial.println(jc);
}
   
    
   else if(abs(diff)<=0.2){
    qq=0;
    Serial.println(qq);}
    else if(abs(diff)>0.2 && abs(diff)<=0.4){
    qq=1;
    Serial.println(qq);}
    else if(abs(diff)>0.4 && abs(diff)<=0.6){
     qq=2;
    Serial.println(qq);}
    else if(abs(diff)>0.6 && abs(diff)<=0.8){
    qq=3;
    Serial.println(qq);}
    else if(abs(diff)>0.8 && abs(diff)<=1.2){
    qq=4;
    Serial.println(qq);}
    else if(abs(diff)>1.2 && abs(diff)<=1.6){
    qq=5;
    Serial.println(qq);}
    else if(abs(diff)>1.6 && abs(diff)<=1.8){
    qq=6;
    Serial.println(qq);}
   else if(abs(diff)>1.8 && abs(diff)<=2.2){
    qq=7;
    Serial.println(qq);}
    else if(abs(diff)>2.2 && abs(diff)<=2.8){
    qq=8;
    Serial.println(qq);}
    else if(abs(diff)>2.8 && abs(diff)<=3.6){
    qq=9;
    Serial.println(qq);
    }
    else if(abs(diff)>3.6 && abs(diff)<=4.6){
    qq=10;
    Serial.println(qq);}
    else if(abs(diff)>4.6 && abs(diff)<=7){
    qq=11;
    Serial.println(qq);
    }
    else if (abs(diff)>7){
    qq=12;
    Serial.println(qq);}
    
    switch(qq){
    case 0:
    Serial.print("case 0:");
    pwm=weights[0];
   Forward(high_speed+pwm,high_speed-pwm,high_speed+pwm,high_speed-pwm);
   Serial.println("FORARD");
    break;
case 1:
    Serial.print("case 1:");
    pwm=weights[1];
    if(diff<0)
    {
    Left(high_speed+pwm,high_speed-pwm,high_speed+pwm,high_speed-pwm);
   Serial.println("Left");
    }
    if(diff>0)
    {
    Right(high_speed-pwm,high_speed+pwm,high_speed-pwm,high_speed+pwm);
    Serial.println("Right");
    }
    break;

   case 2:
    Serial.print("case 2:");
    pwm=weights[2];
    if(diff<0)
    {
    Left(high_speed+pwm,high_speed-pwm,high_speed+pwm,high_speed-pwm);
   Serial.println("Left");
    }
    if(diff>0)
    {
    Right(high_speed-pwm,high_speed+pwm,high_speed-pwm,high_speed+pwm);
    Serial.println("Right");
    }
    break;

     case 3:
      Serial.print("case 3:");
    pwm=weights[3];
    if(diff<0)
    {
    Left(high_speed+pwm,high_speed-pwm,high_speed+pwm,high_speed-pwm);
   Serial.println("Left");
    }
    if(diff>0)
    {
    Right(high_speed-pwm,high_speed+pwm,high_speed-pwm,high_speed+pwm);
    Serial.println("Right");
    }
    break;
    case 4:
     Serial.print("case 4:");
    pwm=weights[4];
    if(diff<0)
    {
    Left(high_speed+pwm,high_speed-pwm,high_speed+pwm,high_speed-pwm);
   Serial.println("Left");
    }
    if(diff>0)
    {
    Right(high_speed-pwm,high_speed+pwm,high_speed-pwm,high_speed+pwm);
    Serial.println("Right");
    }
    break;
    case 5:
     Serial.print("case 5:");
    pwm=weights[5];
    if(diff<0)
    {
    Left(high_speed+pwm,high_speed-pwm,high_speed+pwm,high_speed-pwm);
   Serial.println("Left");
    }
    if(diff>0)
    {
    Right(high_speed-pwm,high_speed+pwm,high_speed-pwm,high_speed+pwm);
    Serial.println("Right");
    }
    break;
    
    case 6:
     Serial.print("case 6:");
    pwm=weights[6];
   if(diff<0)
    {
    Left(high_speed+pwm,high_speed-pwm,high_speed+pwm,high_speed-pwm);
   Serial.println("Left");
    }
    if(diff>0)
    {
    Right(high_speed-pwm,high_speed+pwm,high_speed-pwm,high_speed+pwm);
    Serial.println("Right");
    }
    break;
    
    case 7:
 Serial.print("case 7:");   
    pwm=weights[7];
    if(diff<0)
    {
    Left(high_speed+pwm,high_speed-pwm,high_speed+pwm,high_speed-pwm);
   Serial.println("Left");
    }
    if(diff>0)
    {
    Right(high_speed-pwm,high_speed+pwm,high_speed-pwm,high_speed+pwm);
    Serial.println("Right");
    }
    break;
    case 8:
     Serial.print("case 8:");
    pwm=weights[8];
    if(diff<0)
    {
    Left(high_speed+pwm,high_speed-pwm,high_speed+pwm,high_speed-pwm);
   Serial.println("Left");
    }
    if(diff>0)
    {
    Right(high_speed-pwm,high_speed+pwm,high_speed-pwm,high_speed+pwm);
    Serial.println("Right");
    }
    break;
    case 9:
     Serial.print("case 9:");
    pwm=weights[9];
    if(diff<0)
    {
    Left(high_speed+pwm,high_speed-pwm,high_speed+pwm,high_speed-pwm);
   Serial.println("Left");
    }
    if(diff>0)
    {
    Right(high_speed-pwm,high_speed+pwm,high_speed-pwm,high_speed+pwm);
    Serial.println("Right");
    }
    break;
    case 10:
     Serial.print("case 10:");
    pwm=weights[10];
    if(diff<0)
    {
    Left(high_speed+pwm,high_speed-pwm,high_speed+pwm,high_speed-pwm);
   Serial.println("Left");
    }
    if(diff>0)
    {
    Right(high_speed-pwm,high_speed+pwm,high_speed-pwm,high_speed+pwm);
    Serial.println("Right");
    }
    break;
    case 11:
     Serial.print("case 11:");
    pwm=weights[11];
    if(diff<0)
    {
    Left(high_speed+pwm,high_speed-pwm,high_speed+pwm,high_speed-pwm);
   Serial.println("Left");
    }
    if(diff>0)
    {
    Right(high_speed-pwm,high_speed+pwm,high_speed-pwm,high_speed+pwm);
    Serial.println("Right");
    }
    break;
    case 12:
     Serial.print("case 12:");
    pwm=weights[12];
    if(diff<0)
    {
    Left(high_speed+pwm,high_speed-pwm,high_speed+pwm,high_speed-pwm);
   Serial.println("Left");
    }
    if(diff>0)
    {
    Right(high_speed-pwm,high_speed+pwm,high_speed-pwm,high_speed+pwm);
    Serial.println("Right");
    }
    break;
    default:
    break;
    }
    Serial.print("PWM:");
    Serial.print(pwm);
  
  
}

    }  }  


void Right(int FRPWM,int FLPWM,int BRPWM,int BLPWM){
  digitalWrite(frontRight1,LOW);
       digitalWrite(frontRight2,LOW);
       digitalWrite(frontRighten,LOW);
       digitalWrite(frontLeft1,HIGH);
       digitalWrite(frontLeft2,LOW);
       digitalWrite(frontLeften,LOW);
         digitalWrite(backLeft1,LOW);     
         digitalWrite(backLeft2,LOW);
       digitalWrite(backLeften,LOW);
       digitalWrite(backRight1,HIGH);
       digitalWrite(backRight2,LOW);
       digitalWrite(backRighten,LOW);
      analogWrite(frontRightpwm,FRPWM);
         analogWrite(frontLeftpwm,FLPWM); 
          analogWrite(backLeftpwm,BLPWM);
          analogWrite(backRightpwm,BRPWM);
           Serial.println(FRPWM);
          Serial.println(FLPWM);
          Serial.println(BRPWM);
          Serial.println(BLPWM);
          Serial.print("\nRight PWM:");
  }
 void Left(int FRPWM,int FLPWM,int BRPWM,int BLPWM){
   digitalWrite(frontRight1,LOW);
       digitalWrite(frontRight2,LOW);
       digitalWrite(frontRighten,LOW);
       digitalWrite(frontLeft1,HIGH);
       digitalWrite(frontLeft2,LOW);
       digitalWrite(frontLeften,LOW);
         digitalWrite(backLeft1,LOW);
       digitalWrite(backLeft2,LOW);
       digitalWrite(backLeften,LOW);
       digitalWrite(backRight1,HIGH);
       digitalWrite(backRight2,LOW);
       digitalWrite(backRighten,LOW);
     analogWrite(frontRightpwm,FRPWM);
         analogWrite(frontLeftpwm,FLPWM); 
          analogWrite(backLeftpwm,BLPWM);
          analogWrite(backRightpwm,BRPWM);
           Serial.println(FRPWM);
          Serial.println(FLPWM);
          Serial.println(BRPWM);
          Serial.println(BLPWM);
            Serial.print("\nLeft PWM:");
  }
  /*void SlideRight(int FLPWM,int BRPWM){
    digitalWrite(frontLeft1,LOW);
  digitalWrite(frontLeft2,HIGH);
  digitalWrite(frontLeften,LOW);
  analogWrite(frontLeftpwm,FLPWM);
    digitalWrite(backRight1,HIGH);
  digitalWrite(backRight2,LOW);
  digitalWrite(backRighten,LOW);
  analogWrite(backRightpwm,BRPWM);
 }
   void SlideLeft(int FLPWM,int BRPWM){
    digitalWrite(frontLeft1,HIGH);
  digitalWrite(frontLeft2,LOW);
  digitalWrite(frontLeften,LOW);
  analogWrite(frontLeftpwm,FLPWM);
    digitalWrite(backRight1,LOW);
  digitalWrite(backRight2,HIGH);
  digitalWrite(backRighten,LOW);
  analogWrite(backRightpwm,BRPWM);
 }*/
 void Forward(int FRPWM,int FLPWM,int BRPWM,int BLPWM){
digitalWrite(frontRight1,LOW);
       digitalWrite(frontRight2,LOW);
       digitalWrite(frontRighten,LOW);
       digitalWrite(frontLeft1,HIGH);
       digitalWrite(frontLeft2,LOW);
       digitalWrite(frontLeften,LOW);
         digitalWrite(backLeft1,LOW);
       digitalWrite(backLeft2,LOW);
       digitalWrite(backLeften,LOW);
       digitalWrite(backRight1,HIGH);
       digitalWrite(backRight2,LOW);
       digitalWrite(backRighten,LOW);
         
         analogWrite(frontRightpwm,FRPWM);
         analogWrite(frontLeftpwm,FLPWM); 
          analogWrite(backLeftpwm,BLPWM);
          analogWrite(backRightpwm,BRPWM);
          Serial.println(FRPWM);
          Serial.println(FLPWM);
          Serial.println(BRPWM);
          Serial.println(BLPWM);
          Serial.print("\nUp pwm:");
         
  }
  void Back()
  {
 

   digitalWrite(frontRight1,LOW);
       digitalWrite(frontRight2,LOW);
       digitalWrite(frontRighten,LOW);
       digitalWrite(frontLeft1,HIGH);
       digitalWrite(frontLeft2,LOW);
       digitalWrite(frontLeften,LOW);
         digitalWrite(backLeft1,LOW);
       digitalWrite(backLeft2,LOW);
       digitalWrite(backLeften,LOW);
       digitalWrite(backRight1,HIGH);
       digitalWrite(backRight2,LOW);
       digitalWrite(backRighten,LOW);
      analogWrite(frontRightpwm,0);
         analogWrite(frontLeftpwm,0); 
          analogWrite(backLeftpwm,0);
          analogWrite(backRightpwm,0);

          Serial.print("\nDown PWM:");
          
        
        digitalWrite(20,HIGH);
        delay(5000);
        
        while(true)
        {
          
            // if programming failed, don't try to do anything
      if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
            stable--;
            if(stable==2)
            {
              a1=ypr[0] * 180/M_PI;
              Serial.print(" A1  ");
              Serial.println(a1);
            }
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

        if(stable<0){ 
          Serial.println("Working");
          a=ypr[0] * 180/M_PI;
          Serial.print(a);
          diff=a1-a;
          Serial.print(diff);
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
        
        // blink LED to indicate activity
        blinkState = !blinkState;
high_speed=85;
if( digitalRead(42) && digitalRead(43) && digitalRead(44) )
{ 
  jc1=jc1+1;
  
  Serial.println("\nJC1");
  //digitalWrite(20,LOW);
  Serial.println(jc1);
  delay(1000);
  if (jc1==3)
  { digitalWrite(frontRight1,LOW);
       digitalWrite(frontRight2,LOW);
       digitalWrite(frontRighten,LOW);
       digitalWrite(frontLeft1,HIGH);
       digitalWrite(frontLeft2,LOW);
       digitalWrite(frontLeften,LOW);
         digitalWrite(backLeft1,LOW);
       digitalWrite(backLeft2,LOW);
       digitalWrite(backLeften,LOW);
       digitalWrite(backRight1,HIGH);
       digitalWrite(backRight2,LOW);
       digitalWrite(backRighten,LOW);
      analogWrite(frontRightpwm,0);
         analogWrite(frontLeftpwm,0); 
          analogWrite(backLeftpwm,0);
          analogWrite(backRightpwm,0);

          Serial.print("\nDown PWM:");
          
        
        digitalWrite(20,HIGH);
        delay(5000);
    return;
    }
}
     if(abs(diff)<=0.2){
    qq=0;
    Serial.println(qq);}
    if(abs(diff)>0.2 && abs(diff)<=0.8){
    qq=1;
    Serial.println(qq);}
    if(abs(diff)>0.8 && abs(diff)<=1.2){
     qq=2;
    Serial.println(qq);}
     if(abs(diff)>1.2 && abs(diff)<=1.6){
    qq=3;
    Serial.println(qq);}
    if(abs(diff)>1.6 && abs(diff)<=2){
    qq=4;
    Serial.println(qq);}
     if(abs(diff)>2 && abs(diff)<=2.4){
    qq=5;
    Serial.println(qq);}
    if(abs(diff)>2.4 && abs(diff)<=2.8){
    qq=6;
    Serial.println(qq);}
   if(abs(diff)>2.8 && abs(diff)<=3.2){
    qq=7;
    Serial.println(qq);}
     if(abs(diff)>3.2 && abs(diff)<=3.8){
    qq=8;
    Serial.println(qq);}
    if(abs(diff)>3.8 && abs(diff)<=4.6){
    qq=9;
    Serial.println(qq);
    }
    if(abs(diff)>4.6 && abs(diff)<=5.6){
    qq=10;
    Serial.println(qq);}
    if(abs(diff)>5.6 && abs(diff)<=7){
    qq=11;
    Serial.println(qq);
    }
     if (abs(diff)>7){
    qq=12;
    Serial.println(qq);}
    
    switch(qq){
    case 0:
    Serial.print("case 0:");
    pwm=weights[0];
   Forward1(high_speed+pwm,high_speed-pwm,high_speed+pwm,high_speed-pwm);
   Serial.println("FORARD12");
    break;
    case 1:
    Serial.print("case 1:");
    pwm=weights[1];
    if(diff<0)
    {
    Left1(high_speed+pwm,high_speed+pwm,high_speed-pwm,high_speed-pwm);
   Serial.println("Left");
    }
    if(diff>0)
    {
    Right1(high_speed-pwm,high_speed-pwm,high_speed+pwm,high_speed+pwm);
    Serial.println("Right");
    }
    break;

   case 2:
    Serial.print("case 2:");
    pwm=weights[2];
    if(diff<0)
    {
    Left1(high_speed+pwm,high_speed+pwm,high_speed-pwm,high_speed-pwm);
   Serial.println("Left");
    }
    if(diff>0)
    {
    Right1(high_speed-pwm,high_speed-pwm,high_speed+pwm,high_speed+pwm);
    Serial.println("Right");
    }
    break;

     case 3:
      Serial.print("case 3:");
    pwm=weights[3];
    if(diff<0)
    {
    Left1(high_speed+pwm,high_speed+pwm,high_speed-pwm,high_speed-pwm);
   Serial.println("Left");
    }
    if(diff>0)
    {
    Right1(high_speed-pwm,high_speed-pwm,high_speed+pwm,high_speed+pwm);
    Serial.println("Right");
    }
    break;
    case 4:
     Serial.print("case 4:");
    pwm=weights[4];
    if(diff<0)
    {
    Left1(high_speed+pwm,high_speed+pwm,high_speed-pwm,high_speed-pwm);
   Serial.println("Left");
    }
    if(diff>0)
    {
    Right1(high_speed-pwm,high_speed-pwm,high_speed+pwm,high_speed+pwm);
    Serial.println("Right");
    }
    break;
    case 5:
     Serial.print("case 5:");
    pwm=weights[5];
    if(diff<0)
    {
    Left1(high_speed+pwm,high_speed+pwm,high_speed-pwm,high_speed-pwm);
   Serial.println("Left");
    }
    if(diff>0)
    {
    Right1(high_speed-pwm,high_speed-pwm,high_speed+pwm,high_speed+pwm);
    Serial.println("Right");
    }
    break;
    
    case 6:
     Serial.print("case 6:");
    pwm=weights[6];
   if(diff<0)
    {
    Left1(high_speed+pwm,high_speed+pwm,high_speed-pwm,high_speed-pwm);
   Serial.println("Left");
    }
    if(diff>0)
    {
    Right1(high_speed-pwm,high_speed-pwm,high_speed+pwm,high_speed+pwm);
    Serial.println("Right");
    }
    break;
    
    case 7:
 Serial.print("case 7:");   
    pwm=weights[7];
    if(diff<0)
    {
    Left1(high_speed+pwm,high_speed+pwm,high_speed-pwm,high_speed-pwm);
   Serial.println("Left");
    }
    if(diff>0)
    {
    Right1(high_speed-pwm,high_speed-pwm,high_speed+pwm,high_speed+pwm);
    Serial.println("Right");
    }
    break;
    case 8:
     Serial.print("case 8:");
    pwm=weights[8];
    if(diff<0)
    {
    Left1(high_speed+pwm,high_speed+pwm,high_speed-pwm,high_speed-pwm);
   Serial.println("Left");
    }
    if(diff>0)
    {
    Right1(high_speed-pwm,high_speed-pwm,high_speed+pwm,high_speed+pwm);
    Serial.println("Right");
    }
    break;
    case 9:
     Serial.print("case 9:");
    pwm=weights[9];
    if(diff<0)
    {
    Left1(high_speed+pwm,high_speed+pwm,high_speed-pwm,high_speed-pwm);
   Serial.println("Left");
    }
    if(diff>0)
    {
    Right1(high_speed-pwm,high_speed-pwm,high_speed+pwm,high_speed+pwm);
    Serial.println("Right");
    }
    break;
    case 10:
     Serial.print("case 10:");
    pwm=weights[10];
    if(diff<0)
    {
    Left1(high_speed+pwm,high_speed+pwm,high_speed-pwm,high_speed-pwm);
   Serial.println("Left");
    }
    if(diff>0)
    {
    Right1(high_speed-pwm,high_speed-pwm,high_speed+pwm,high_speed+pwm);
    Serial.println("Right");
    }
    break;
    case 11:
     Serial.print("case 11:");
    pwm=weights[11];
    if(diff<0)
    {
    Left1(high_speed+pwm,high_speed+pwm,high_speed-pwm,high_speed-pwm);
   Serial.println("Left");
    }
    if(diff>0)
    {
    Right1(high_speed-pwm,high_speed-pwm,high_speed+pwm,high_speed+pwm);
    Serial.println("Right");
    }
    break;
    case 12:
     Serial.print("case 12:");
    pwm=weights[12];
    if(diff<0)
    {
    Left1(high_speed+pwm,high_speed+pwm,high_speed-pwm,high_speed-pwm);
   Serial.println("Left");
    }
    if(diff>0)
    {
    Right1(high_speed-pwm,high_speed-pwm,high_speed+pwm,high_speed+pwm);
    Serial.println("Right");
    }
    break;
    default:
    break;
    }

    
        }
        
    }}
  }

 void Right1(int FRPWM,int FLPWM,int BRPWM,int BLPWM){
  digitalWrite(frontRight1,LOW);
       digitalWrite(frontRight2,LOW);
       digitalWrite(frontRighten,LOW);
       digitalWrite(frontLeft1,LOW);
       digitalWrite(frontLeft2,HIGH);
       digitalWrite(frontLeften,LOW);
         digitalWrite(backLeft1,LOW);
       digitalWrite(backLeft2,LOW);
       digitalWrite(backLeften,LOW);
       digitalWrite(backRight1,LOW);
       digitalWrite(backRight2,HIGH);
       digitalWrite(backRighten,LOW);
      analogWrite(frontRightpwm,FRPWM);
         analogWrite(frontLeftpwm,FLPWM); 
          analogWrite(backLeftpwm,BLPWM);
          analogWrite(backRightpwm,BRPWM);
           Serial.println(FRPWM);
          Serial.println(FLPWM);
          Serial.println(BRPWM);
          Serial.println(BLPWM);
          Serial.print("\nRight PWM:");
          Serial.print("RIGHT1");
  }
 void Left1(int FRPWM,int FLPWM,int BRPWM,int BLPWM){
  digitalWrite(frontRight1,LOW);
       digitalWrite(frontRight2,LOW);
       digitalWrite(frontRighten,LOW);
       digitalWrite(frontLeft1,LOW);
       digitalWrite(frontLeft2,HIGH);
       digitalWrite(frontLeften,LOW);
         digitalWrite(backLeft1,LOW);
       digitalWrite(backLeft2,LOW);
       digitalWrite(backLeften,LOW);
       digitalWrite(backRight1,LOW);
       digitalWrite(backRight2,HIGH);
       digitalWrite(backRighten,LOW);
     analogWrite(frontRightpwm,FRPWM);
         analogWrite(frontLeftpwm,FLPWM); 
          analogWrite(backLeftpwm,BLPWM);
          analogWrite(backRightpwm,BRPWM);
           Serial.println(FRPWM);
          Serial.println(FLPWM);
          Serial.println(BRPWM);
          Serial.println(BLPWM);
            Serial.print("\nLeft PWM:");
            Serial.print("LEFT1");
  }
  /*void SlideRight(int FLPWM,int BRPWM){
    digitalWrite(frontLeft1,LOW);
  digitalWrite(frontLeft2,HIGH);
  digitalWrite(frontLeften,LOW);
  analogWrite(frontLeftpwm,FLPWM);
    digitalWrite(backRight1,HIGH);
  digitalWrite(backRight2,LOW);
  digitalWrite(backRighten,LOW);
  analogWrite(backRightpwm,BRPWM);
 }
   void SlideLeft(int FLPWM,int BRPWM){
    digitalWrite(frontLeft1,HIGH);
  digitalWrite(frontLeft2,LOW);
  digitalWrite(frontLeften,LOW);
  analogWrite(frontLeftpwm,FLPWM);
    digitalWrite(backRight1,LOW);
  digitalWrite(backRight2,HIGH);
  digitalWrite(backRighten,LOW);
  analogWrite(backRightpwm,BRPWM);
 }*/
 void Forward1(int FRPWM,int FLPWM,int BRPWM,int BLPWM){
digitalWrite(frontRight1,LOW);
       digitalWrite(frontRight2,LOW);
       digitalWrite(frontRighten,LOW);
       digitalWrite(frontLeft1,LOW);
       digitalWrite(frontLeft2,HIGH);
       digitalWrite(frontLeften,LOW);
         digitalWrite(backLeft1,LOW);
       digitalWrite(backLeft2,LOW);
       digitalWrite(backLeften,LOW);
       digitalWrite(backRight1,LOW);
       digitalWrite(backRight2,HIGH);
       digitalWrite(backRighten,LOW);
         
         analogWrite(frontRightpwm,FRPWM+15);
         analogWrite(frontLeftpwm,FLPWM); 
          analogWrite(backLeftpwm,BLPWM);
          analogWrite(backRightpwm,BRPWM);
          Serial.println(FRPWM);
          Serial.println(FLPWM);
          Serial.println(BRPWM);
          Serial.println(BLPWM);
          Serial.print("\nUp pwm:");
          Serial.print("FORWARD1");
         
  }  

       void Backward1(int FRPWM,int FLPWM,int BRPWM,int BLPWM){
        while(true)
        {
digitalWrite(frontRight1,HIGH);
       digitalWrite(frontRight2,LOW);
       digitalWrite(frontRighten,LOW);
       digitalWrite(frontLeft1,LOW);
       digitalWrite(frontLeft2,LOW);
       digitalWrite(frontLeften,LOW);
         digitalWrite(backLeft1,HIGH);
       digitalWrite(backLeft2,LOW);
       digitalWrite(backLeften,LOW);
       digitalWrite(backRight1,LOW);
       digitalWrite(backRight2,LOW);
       digitalWrite(backRighten,LOW);
         
         analogWrite(frontRightpwm,FRPWM);
         analogWrite(frontLeftpwm,FLPWM); 
          analogWrite(backLeftpwm,BLPWM);
          analogWrite(backRightpwm,BRPWM);
          Serial.print("Backward1");
          Serial.println(FRPWM);
          Serial.println(FLPWM);
          Serial.println(BRPWM);
          Serial.println(BLPWM);
          Serial.print("\nUp pwm:");
         
  }}

  
  
