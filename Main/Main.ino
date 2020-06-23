#include <Wire.h>
#include "PID_v1.h"
#include "LMotorController.h"
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define LOG_INPUT 0
#define MOVE_BACK_FORTH 0
#define UNIT_SPEED 2
#define MIN_ABS_SPEED 30

//MPU

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container// phương hướng
VectorFloat gravity;    // [x, y, z]            gravity vector// gia tốc
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


double originalSetpoint = 181.4;// 182
double setpoint = originalSetpoint;
double movingAngleOffset = 0.15;// 0.3- OK, 0.15 - OK
double input, output;
int moveState=0; //0 = balance; 1 = back; 2 = forth
//kp->ki->kd// giá trị sẽ được thiết lập mặc định
float kp = 39;
float ki = 1100;
float kd = 1.38;
PID pid(&input, &output, &setpoint, kp, ki, kd, DIRECT);// time 5ms & 10ms, sometimes Kp(17.35, 16.86) Ki(302.05, 301.05) Kd(1.21)
//MOTOR CONTROLLER

int ENA = 6;
int IN1 = 8;
int IN2 = 9;
int IN3 = 10;
int IN4 = 11;
int ENB = 7;


//control
int steps1=0; //number of steps for Left Stepper each loop
int steps2=0;
int roll,pitch; //roll and pitch sent from Android device
int pad_x,pad_y; //control pad values sent from Andorid device
char BluetoothData; // the Bluetooth data received


LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, 1, 1);


void Uart_Recieve()
{
  if(Serial1.available())
  {
    BluetoothData=Serial1.read(); //Get next character from bluetooth
    //**** Control Pad on Right -  Sends 'X__,Y___*' every 150ms
    if(BluetoothData=='X'){
      pad_x=Serial1.parseInt();
      while (BluetoothData!='*'){
        if (Serial1.available()){
          BluetoothData = Serial1.read(); //Get next character from bluetooth
          if(BluetoothData=='Y') pad_y = -Serial1.parseInt();
        }
      }
      //Algorithm to convert pad position to number of steps for each motor
      float mag = sqrt(pad_y * pad_y + pad_x * pad_x);
      if (mag > 15) mag = 15;
      if (pad_y<0) mag = 0-mag;
      steps1 = steps2 = mag;
      if (pad_x > 0){ //turning right
        steps2 = steps2 - mag*pad_x/5.0;
      }else{ //turnign left
        steps1 = steps1 + mag*pad_x/5.0;
      }
      Serial.println(steps1);
      Serial.println("|||");
      Serial.println(steps2);
    }
    //**** Control Pad on Left
    if(BluetoothData=='0') {
      steps1 = steps2 = 0;
      setpoint = originalSetpoint;
    } //Release 
    if(BluetoothData=='1') setpoint = originalSetpoint + 1.5; //Up
    if(BluetoothData=='3') setpoint = originalSetpoint - 1.5; //Down
    if(BluetoothData=='4') { //Left
      steps1=-50;
      steps2=50; 
    }
    if(BluetoothData=='2') { //Right
      steps1=50;
      steps2=-50; 
    }  
  }
}

void setupPID(){
  //setup PID
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(5);// 10 - OK, 5 - GOOD, 1- CHANGE PID
  pid.SetOutputLimits(-255, 255);// 80 - OK Strong enough  
}

void setup()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(200);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    setupMPU();
    setupPID();
    Serial1.begin(9600);
}

void loop()
{
    if (!dmpReady) return;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize){
          fifoCount = mpu.getFIFOCount();
        }

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)//đọc thêm từ fifo mà ko cần chờ ngắt
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);// chuyển động Yaw, Pitch, Roll
        #if LOG_INPUT
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);//yaw//chuyển động xoay
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);//pitch
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);//rool// nghiên về hai bên
        #endif
        input = ypr[2]*180/M_PI+180;//lấy chuyển động pitch // ngả về trước hoặc về sau
        Serial.println(abs(input-originalSetpoint));
        if (abs(input-originalSetpoint)<30){
          pid.Compute();
          motorController.move(output + steps1, output + steps2,MIN_ABS_SPEED);
        }
        else {
          digitalWrite(IN1, LOW);
          digitalWrite(IN2, LOW);
          digitalWrite(IN3, LOW);
          digitalWrite(IN4, LOW);
        }
   } 
   Uart_Recieve();
}

void setupMPU(){
    // load and configure the DMP
    //Digital Motion Processor bộ xử lí chuyển động kĩ thuật số
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity
    //cung cấp bù đắp cho con quay hồi chuyển
    mpu.setXGyroOffset(39);
    mpu.setYGyroOffset(14);
    mpu.setZGyroOffset(6);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed// tải bộ nhớ không thành công
        // 2 = DMP configuration updates failed // cập nhật DMP thất bại
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}
