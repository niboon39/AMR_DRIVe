#include "mbed.h"

DigitalOut dir1(D12); 
PwmOut pwm1 (D11) ; 
DigitalOut dir2(D5); 
PwmOut pwm2 (D4) ;

Serial pc(USBTX , USBRX , 57600) ;
float pwm  = 0.0 ;

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
  pc.printf("Program start..\n");
  while (1)
  {
    /* code */
    char str = pc.getc() ;
    
    switch (str)
    {
      case 'q':
        pwm += 10.0 ;
        pc.printf("PWM : %.2f \n" , pwm);
        break;

      case 'e':
        pwm -= 10.0 ;
        pc.printf("PWM : %.2f \n" , pwm);
        break ; 

      case 'r':
        pwm = 50.0 ;
        pc.printf("PWM : %.2f \n" , pwm);
        break ;
        
      case 'w':
        forward() ;
        pwm1.write(pwm/255);
        pwm2.write(pwm/255);
        pc.printf("forward\n");
        break ;

      case 'a':
        left();
        pwm1.write(pwm/255);
        pwm2.write(pwm/255);
        pc.printf("turnleft\n");
        break ;

      case 'd':
        right() ;
        pwm1.write(pwm/255);
        pwm2.write(pwm/255);
        pc.printf("turnright\n");
        break ;

      case 's':
        stop() ;
        pc.printf("Stop\n");
        break ;

      case 'x':
        backward();
        pwm1.write(pwm/255);
        pwm2.write(pwm/255);
        pc.printf("backward\n");
        break ;

      default:
        stop();
        pc.printf("PWM : %.2f \n" , pwm);
        break;
    }
  // pwm1.write(pwm/255);
  // pwm2.write(pwm/255);
    // pc.printf("%f\n" , pwm);
  }

}