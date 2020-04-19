#include <FlexiTimer2.h>
#include <BalanceCar.h>
#include <KalmanFilter.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

//TB6612FNG Drive module control signal
#define IN1M 6
#define IN2M 7
#define IN3M 12
#define IN4M 13
#define PWMA 4
#define PWMB 5
#define STBY 8

//Encoder interrupt
#define PinA_right 18  
#define PinA_left 2    

//Bluetooth protocol function code
#define  Left_Enoder     0x01
#define  Right_Enoder    0x02
#define  Left_PWM        0x03
#define  Right_PWM       0x04
#define  Balance_Angle   0x05
#define  Upright_Kp      0x06
#define  Upright_Ki      0x07
#define  Upright_Kd      0x08
#define  Speed_Kp        0x09
#define  Speed_Ki        0x10
#define  Speed_Kd        0x0A
#define  Rotate_Kp       0x0B
#define  Rotate_Ki       0x0C
#define  Rotate_Kd       0x0D
#define  Contrl_val      0x0E
#define  angle_output    0x0F
#define  speed_output    0x11

MPU6050 mpu; 
BalanceCar balancecar;
KalmanFilter kalmanfilter;
int16_t ax, ay, az, gx, gy, gz;

//Declare custom variables
double Outputs = 0;                         //Speed ​​DIP set point, input, output

double kp = 29.525, ki = 0.0, kd = 0.2765;                              //Need you to modify the parameters
double kp_speed = 5.0225, ki_speed = 0.1528, kd_speed = 0.0;            // Need you to modify the parameters
double kp_turn = 23.1625, ki_turn = 0, kd_turn = 0.277;                 //Rotate PID setting


//Turn to PID parameters
double setp0 = 0; 


//********************angle data*********************//
float K1 = 0.05; //The weight of the accelerometer value
float angle0 = 1.928; //Mechanical balance angle
//********************angle data*********************//


//***************Kalman_Filter*********************//
float Q_angle = 0.001, Q_gyro = 0.005; //Angle data confidence, angular velocity data confidence
float R_angle = 0.5 , C_0 = 1;
float timeChange = 5; //Filter sampling interval milliseconds
float dt = timeChange * 0.001; //Note: The value of dt is the filter sampling time
//***************Kalman_Filter*********************//


//*********************************************
//******************** speed count ************
//*********************************************
volatile long count_right = 0;//The volatile lon type is used for external interrupt pulse count values ​​to ensure that the value is valid when used in other functions
volatile long count_left = 0;//The volatile lon type is used for external interrupt pulse count values ​​to ensure that the value is valid when used in other functions
int speedcc = 0;
int timer = 0;

//////////////////////Pulse calculation/////////////////////////
int lz = 0;
int rz = 0;
int rpluse = 0;
int lpluse = 0;
int sumam;
/////////////////////Pulse calculation////////////////////////////

//////////////Turn, rotate parameters///////////////////////////////
int turncount = 0; //Turn intervention time calculation
float turnoutput = 0;
//////////////Turn, rotate parameters///////////////////////////////

//////////////Bluetooth control volume///////////////////
int front = 0;//Forward variables
int back = 0;//Backward variables
int turnl = 0;//Turn left sign
int turnr = 0;//Turn right sign
int spinl = 0;//Left rotation sign
int spinr = 0;//Right rotation sign
//////////////Bluetooth control volume///////////////////


////////////////Bluetooth transceiver////////////
unsigned char a[12]={0xAA,0xAA,0xAA,0xAA,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF};
unsigned char RxBuf[12]={0};
unsigned int Ctrl_val;
int Speed_val = 85;        //Straight speed setting
char i=0;

void Uart_Send(unsigned char function,unsigned int value)
{
  unsigned int temp;
  i=0;
  temp=value;
  a[5]=function;
  a[6]=(value>>8) & 0xFF;
  a[7]=temp & 0xFF;
  while(i!=12)
  {
    Serial.write(a[i]);
    i++;
  }
}

void Uart_Recieve()
{
  if(Serial.available()>=12)
  {
    i=0;
    while(i!=12)
    {
      RxBuf[i]=Serial.read();
      i++;
    }
    Ctrl_val=RxBuf[6]*0x100+RxBuf[7];
    switch(RxBuf[5])
    {
      case Upright_Kp : kp=(double)Ctrl_val/1000;break;         
      case Upright_Ki :break;           
      case Upright_Kd : kd=(double)Ctrl_val/1000;break;
      case Speed_Kp :kp_speed=(double)Ctrl_val/1000;break;       
      case Speed_Ki :ki_speed=(double)Ctrl_val/1000;break;     
      case Speed_Kd :break;
      case Rotate_Kp :kp_turn=(double)Ctrl_val/1000;break;
      case Rotate_Ki :break;
      case Rotate_Kd :kd_turn=(double)Ctrl_val/100;break;
      case Contrl_val:
        switch(RxBuf[7])
        {
          case 0x01: front = Speed_val;   break;                                     
          case 0x02: back = -Speed_val;   break;                                    //
          case 0x03: turnl = 1;front = 0;back = 0;   break;                          //
          case 0x04: turnr = 1;front = 0;back = 0;   break;                          //
          case 0x05: spinl = 1;front = 0;back = 0;   break;                       //
          case 0x06: spinr = 1;front = 0;back = 0;break;                          //
          case 0x07: spinl = 0; spinr = 0;  front = 0; back = 0;  turnl = 0; turnr = 0;  break;                  //Make sure the button is released for parking operation
          case 0x08: front = 0; back = 0; turnl = 0; turnr = 0; spinl = 0; spinr = 0; turnoutput = 0; break;       // Make sure the button is released for parking operation
          default: front = 0; back = 0; turnl = 0; turnr = 0; spinl = 0; spinr = 0; turnoutput = 0; break;
        }break;
      default :break;
    }
    while(Serial.read()>= 0){}
  }
}



void countpluse()
{

  lz = count_left;
  rz = count_right;
  
  count_left = 0;
  count_right = 0;

  lpluse = lz;
  rpluse = rz;

  if ((balancecar.pwm1 < 0) && (balancecar.pwm2 < 0))                     //Judgment of the direction of movement of the car back (PWM that is negative motor voltage) pulse number is negative
  {
    rpluse = -rpluse;
    lpluse = -lpluse;
  }
  else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 > 0))                 //When the direction of movement of the car to judge forward (PWM that is positive motor voltage) pulse number is negative
  {
    rpluse = rpluse;
    lpluse = lpluse;
  }
  else if ((balancecar.pwm1 < 0) && (balancecar.pwm2 > 0))                 //When the direction of movement of the car to judge forward (PWM that is positive motor voltage) pulse number is negative
  {
    rpluse = -rpluse;
    lpluse = lpluse;
  }
  else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 < 0))               //The direction of movement of the car to determine the left turn right pulse number is negative left pulse number is positive
  {
    rpluse = rpluse;
    lpluse = -lpluse;
  }

  //Filed judgment
  balancecar.stopr += rpluse;
  balancecar.stopl += lpluse;

  //Every 5ms into the interrupt, the number of pulses superimposed
  balancecar.pulseright += rpluse;
  balancecar.pulseleft += lpluse;
  sumam=balancecar.pulseright+balancecar.pulseleft;
}
////////////////////Pulse calculation///////////////////////


//////////////////angle PD////////////////////
void angleout()
{
  balancecar.angleoutput = -kp * (kalmanfilter.angle + angle0) - kd * kalmanfilter.Gyro_x;//PD Angle loop control
}
//////////////////angle PD////////////////////

//////////////////////////////////////////////////////////
//////////////////Interrupt timing 5ms timer interrupt////////////////////
/////////////////////////////////////////////////////////
void inter()
{
  sei();                                           
  countpluse();                                     //Pulse superposition function
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);     //IIC Get MPU6050 six-axis data ax ay az gx gy gz
  kalmanfilter.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro,R_angle,C_0,K1);                                      //Get angle angle and Kaman filter
  angleout();                                       //Angle ring PD control

  
  speedcc++;
  if (speedcc >= 8)                                //40ms into the speed loop control  
  {
    Outputs = balancecar.speedpiout(kp_speed,ki_speed,kd_speed,front,back,setp0);
    speedcc = 0;
  }
  turncount++;
  if (turncount > 4)                                //20ms into the rotation control
  {
    turnoutput = balancecar.turnspin(turnl,turnr,spinl,spinr,kp_turn,kd_turn,kalmanfilter.Gyro_z);                                    //�
    turncount = 0;
  }
  balancecar.posture++;
  balancecar.pwma(Outputs,turnoutput,kalmanfilter.angle,kalmanfilter.angle6,turnl,turnr,spinl,spinr,front,back,kalmanfilter.accelz,IN1M,IN2M,IN3M,IN4M,PWMA,PWMB);                     
  
  timer++;
}

void Bluetooth()
{
  if(timer==200)
  {
    Uart_Send(Left_Enoder,(unsigned int)lz);
    delay(2);
    Uart_Send(Right_Enoder,(unsigned int)rz);
    delay(2);
    Uart_Send(Left_PWM,abs(balancecar.pwm2));
    delay(2);
    Uart_Send(Right_PWM,abs(balancecar.pwm1));
    delay(2);
    Uart_Send(Balance_Angle,(unsigned int)kalmanfilter.angle*10);
    delay(2);
    Uart_Send(angle_output,(unsigned int)balancecar.angleoutput*10);        //PID
    delay(2);
    Uart_Send(speed_output,(unsigned int)Outputs*10);  
    timer=0;
  }
}


void setup() {
  // TB6612 Drive module control signal initialization
  pinMode(IN1M, OUTPUT);                         //Control the direction of motor 1, 01 is forward, 10 is reversed
  pinMode(IN2M, OUTPUT);
  pinMode(IN3M, OUTPUT);                        //Control the direction of the motor 2, 01 is forward, 10 is reversed
  pinMode(IN4M, OUTPUT);
  pinMode(PWMA, OUTPUT);                        //Left motor PWM
  pinMode(PWMB, OUTPUT);                        //Right motor PWM
  pinMode(STBY, OUTPUT);                        //TB6612FNG
  digitalWrite(IN1M, 0);
  digitalWrite(IN2M, 1);
  digitalWrite(IN3M, 1);
  digitalWrite(IN4M, 0);
  digitalWrite(STBY, 1);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  pinMode(PinA_left, INPUT);  
  pinMode(PinA_right, INPUT);
  

    
  Wire.begin();                           
  Serial.begin(115200);                       
  delay(500);
  Serial.println("AT+NAMESainSmart");
  delay(1000);
  
  mpu.initialize();                       
  delay(2);
  balancecar.pwm1 =0;
  balancecar.pwm2 =0;
 
  FlexiTimer2::set(5, inter);    //5ms
  FlexiTimer2::start();

}

void loop() {
  
  //The main function of the cycle detection and superimposed pulse measurement car speed change using both the pulse into the pulse superimposed to increase the number of pulses motor to ensure the accuracy of the car.
  attachInterrupt(5, Code_right, CHANGE);
  attachInterrupt(0, Code_left, CHANGE);
  Uart_Recieve();
  Bluetooth();
}


void Code_left() {

  count_left ++;

} //左测速码盘计数



void Code_right() {

  count_right ++;

} 




