#include <mbed.h>
#include "QEI.h"
#include "ros.h"
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

int OdomCount = 0;

int OdomWait = 3; // wait 3
// unsigned long timeold=0;

int LMotor = 1;
int RMotor = 0;

bool left_forward;
bool right_forward;

int MotorNum[2] = {LMotor, RMotor};

double Vels[2] = {0, 0};
int CVEL[2] = {0, 0};
int Mspeeds[2] = {0, 0};

//Use X4 encoding.
QEI wheel_r(PA_2, PA_0, NC, 600, QEI::X4_ENCODING);
QEI wheel_l(PA_1, PA_0, NC, 600, QEI::X4_ENCODING);

double WCS[2] = {wheel_l.getPulses(), wheel_r.getPulses()};

long EncoderVal[2] = {0, 0};
double DDis[2] = {0, 0};
long Time[2] = {0, 0};
long Time_fb = 0;
double DDis_fb = 0;

float WheelSeparation = 0.62; //0.17
float WheelDiameter = 0.15;  //60 to fast
int TPR_forward = 600;    // 1210
int TPR_backward = 600;   // 1210

volatile int counterL_forward = 0;
volatile int counterR_forward = 0;
volatile int counterL_backward = 0;
volatile int counterR_backward = 0;

int AccParam = 3;  //3; //acceleration multiplie8Ur.


int bot_vel;
int count = 0;

// int dir1 = PA_6, pwm1 = PA_7, dir2 = PA_9,  pwm2 = PA_8;
DigitalOut dir1(PA_6);
PwmOut pwm1 (PA_7) ;
DigitalOut dir2(PA_9);
PwmOut pwm2 (PA_8) ;

int motor_speed_l = 0, motor_speed_r = 0;



ros::NodeHandle  nh;

geometry_msgs::Twist odom_msg;
geometry_msgs::Twist odom_msg2;

ros::Publisher Pub ("stm_odom", &odom_msg);


void countL_forward() {
  if (WCS[0]  > 0)
    counterL_forward++;
}

void countR_forward() {
  if (WCS[1]  > 0)
    counterR_forward++;
}

void countL_backward() {
  if (WCS[0] < 0)
    counterL_backward--;
}

void countR_backward() {
  if (WCS[1] < 0)
    counterR_backward--;
}

void right() {
  dir1 = 0 ;
  dir2 = 1 ;
}


void left() {
  dir1 = 1 ;
  dir2 = 0 ;
}

void forward() {
  dir1 = 0 ;
  dir2 = 0 ;
}

void backward() {
  dir1 = 1 ;
  dir2 = 1 ;
}
void stop() {
  pwm1.write(0.0f);
  pwm2.write(0.0f);
}

void velCallback( const geometry_msgs::Twist& CVel) {
  double vel_x = CVel.linear.x;
  double vel_th = CVel.angular.z;
  double right_vel = 0.0;
  double left_vel = 0.0;

  // turning
  if (vel_x == 0) {
    right_vel = vel_th * WheelSeparation / 2.0;
    left_vel = (-1) * right_vel;
  }
  // forward / backward
  else if (vel_th == 0) {
    left_vel = right_vel = vel_x;
  }
  // moving doing arcs
  else {
    right_vel = vel_x + vel_th * WheelSeparation / 2.0;
    left_vel = vel_x - vel_th * WheelSeparation / 2.0;
  }
  //write new command speeds to global vars
  WCS[0] = left_vel;
  WCS[1] = right_vel;

}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel" , &velCallback);

double TicksToMeters(int Ticks, int mode) {
  if (mode == 0) //forward
    return (Ticks * 3.14 * WheelDiameter) / TPR_forward;
  else
    return (Ticks * 3.14 * WheelDiameter) / TPR_backward;
}

//motor write speed - in motor units
double MWS[2] = {0, 0};

double CorrectedSpeed_FB(double CVel) { //only forward, backward
  if (Time_fb == 0) {
    Time_fb = us_ticker_read();
    return 0;
  }

  if (CVel >= 0) {
    EncoderVal[0] = counterL_forward;
    EncoderVal[1] = counterL_forward;
    counterL_forward = counterR_forward = 0;
  }
  else if (CVel < 0) {
    EncoderVal[0] = counterL_backward;
    EncoderVal[1] = counterL_backward;
    counterL_backward = counterR_backward = 0;
  }

  long T = us_ticker_read();
  int DTime = T - Time_fb;
  Time_fb = T;

  if (CVel >= 0)
    DDis_fb = TicksToMeters(EncoderVal[0], 0);
  else
    DDis_fb = TicksToMeters(EncoderVal[0], 1);

  double EVel = (DDis_fb / DTime) * 1000;

  Vels[0] = EVel;
  Vels[1] = EVel;

  EVel = abs(EVel);
  CVel = abs(CVel);

  double dif = EVel - CVel;

  MWS[0] = MWS[0] - (dif * (AccParam));

  if (MWS[0] < 0)  {
    MWS[0] = 0;
    MWS[1] = 0;
  }

  /* if(CVel == 0) {
     MWS[0] = 0;
     MWS[1] = 0;
    } */

  //DEBUG
  CVEL[0] = MWS[0];

  return MWS[0];
}

double CorrectedSpeed(int M, double CVel) {
  if (Time[0] == 0 && Time[1] == 0) {
    Time[0] = us_ticker_read();
    Time[1] = us_ticker_read();
    return 0;
  }

  if (M == 0) {
    if (CVel >= 0) {
      EncoderVal[0] = counterL_forward;
      counterL_forward = 0;
    }
    else if (CVel < 0) {
      EncoderVal[0] = counterL_backward;
      counterL_backward = 0;
    }
  }
  else if (M == 1) {
    if (CVel >= 0) {
      EncoderVal[1] = counterR_forward;
      counterR_forward = 0;
    }
    else if (CVel < 0) {
      EncoderVal[1] = counterR_backward;
      counterR_backward = 0;
    }
  }


  long T = us_ticker_read();
  int DTime = T - Time[M];
  Time[M] = T;


  if (CVel >= 0)
    DDis[M] = TicksToMeters(EncoderVal[M], 0);
  else
    DDis[M] = TicksToMeters(EncoderVal[M], 1);


  double EVel = (DDis[M] / DTime) * 1000;

  Vels[M] = EVel;

  EVel = abs(EVel);
  CVel = abs(CVel);

  double dif = EVel - CVel;

  MWS[M] = MWS[M] - (dif * (AccParam));

  if (MWS[M] < 0)
    MWS[M] = 0;

  if (CVel == 0)  //HERE
    MWS[M] = 0;   //HERE

  //DEBUG
  CVEL[M] = MWS[M];

  return MWS[M];

}

void motorGo(uint8_t l, uint8_t r) {
  if (WCS[0] > 0 && WCS[1] > 0)
    forward();
  else if (WCS[0] < 0 && WCS[1] < 0)
    backward();
  else if (WCS[0] > 0 && WCS[1] < 0)
    right();
  else if (WCS[0] < 0 && WCS[1] > 0)
    left();

  pwm1.write(l/255);
  pwm2.write(r/255);
}


void MotorWrite() {
  int DIR;
  int min_speed;
  double MSpeed;

  if ( ((WCS[0] > 0 && WCS[1] > 0)  || (WCS[0] < 0 && WCS[1] < 0)) && (WCS[0] == WCS[1])  ) {
    //forward/backward
    motor_speed_l = CorrectedSpeed_FB(WCS[0]);
    motor_speed_r = motor_speed_l;
  }
  else {   //left / right / arc
    for (int i = 0; i < 2; i++) {
      MSpeed = CorrectedSpeed(i, WCS[i]);

      if (i == 0)
        motor_speed_l = MSpeed;
      else if (i == 1)
        motor_speed_r = MSpeed;
    }
  }

  motorGo(motor_speed_l, motor_speed_r);
}

int main() {

  // put your setup code here, to run once:
  nh.initNode();
  nh.advertise(Pub);
  nh.subscribe(sub);

  while (1) {
    // put your main code here, to run repeatedly:
    nh.spinOnce();

    if ( OdomCount > OdomWait) { // update 1 second  OdomCount > OdomWait[3]


      if (Vels[0] == 0 and Vels[1] == 0) {
        odom_msg.linear.x = 0;
        odom_msg.linear.y = 0;
      }
      else if (WCS[0] == WCS[1]) {
        odom_msg.linear.x = Vels[0];
        odom_msg.linear.y = Vels[0];
      }
      else {
        odom_msg.linear.x = Vels[0];
        odom_msg.linear.y = Vels[1];
      }
      // timeold = millis();
      Pub.publish(&odom_msg);
    } else {
      OdomCount ++ ;
    }

    MotorWrite(); //Takes WCS and corrects speed of motors with encoders
  }
}