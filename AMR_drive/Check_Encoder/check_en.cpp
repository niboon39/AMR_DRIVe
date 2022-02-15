#include "mbed.h"
#include "QEI.h"

//Use X4 encoding.
QEI wheel_r(PA_2, PA_0, NC, 600, QEI::X4_ENCODING);
QEI wheel_l(PA_1, PA_0, NC, 600, QEI::X4_ENCODING);

Serial pc (USBTX , USBRX , 115200) ; 

int main(){
    while (1)
    {
        /* code */
        wait(0.1);
        pc.printf("Pulses_R is: %i\n", wheel_r.getPulses());
        pc.printf("Pulses_L is: %i\n", wheel_l.getPulses());

    }
    
}