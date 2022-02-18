#include "mbed.h"
Serial pc (USBTX , USBRX , 57600) ; 

int main(){
    while (1)
    {
        /* code */
        long time = us_ticker_read();
        pc.printf("%ld\n" , time) ; 
    }
    
}