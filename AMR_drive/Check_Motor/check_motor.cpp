#include "mbed.h"

DigitalOut dir1(PA_6); PwmOut pwm1 (PA_7) ; DigitalOut dir2(PA_9); PwmOut pwm2 (PA_8) ;
Serial pc (USBTX , USBRX , 115200) ;
int pwm  = 0 ;

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

int main () {
  while (1)
  {
    /* code */
    char str = pc.getc() ;
    if (str == 'q') {
      pwm += 10 ;
      pc.printf("PWM : ");
      pc.printf("%d" , pwm);
    }
    else if (str == 'e') {
      pwm -= 10 ;
      pc.printf("PWM : ");
      pc.printf("%d" , pwm);
    }
    else if (str == 'r') {
      pwm = 50 ;
      pc.printf("PWM : ");
      pc.printf("%d" , pwm);
    }
    switch (str)
    {
      case 'w':
        forward() ;
        pc.printf("forward");
        break ;

      case 'a':
        left();
        pc.printf("turnleft");
        break ;

      case 'd':
        right() ;
        pc.printf("turnright");
        break ;

      case 's':
        stop() ;
        pc.printf("Stop");
        break ;

      case 'x':
        backward();
        pc.printf("backward");
        break ;

      default:
        stop();
        pc.printf("PWM : ");
        pc.printf("%d" , pwm);
        break;
    }
    pwm1.write(pwm / 255);
    pwm2.write(pwm / 255);
  }

}