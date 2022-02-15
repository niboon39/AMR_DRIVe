#include <mbed.h>
#include "QEI.h"
#include "ros.h"

Serial pc (USBTX , USBRX , 9600); 

//Use X4 encoding.
QEI wheel(PA_2, PA_0, NC, 600, QEI::X4_ENCODING);

int main() {

  // put your setup code here, to run once:

  while(1) {
    // put your main code here, to run repeatedly:
    wait(0.1);
    pc.printf("Pulses is: %i\n", wheel.getPulses());
  }
}