#include <16F688.h>
#device adc=10

#FUSES NOWDT                    //No Watch Dog Timer
#FUSES INTRC                 //Internal RC Osc, no CLKOUT
#FUSES NOPROTECT                //Code not protected from reading
#FUSES NOBROWNOUT                 //Reset when brownout detected
#FUSES MCLR                   //Master Clear pin used for I/O
#FUSES NOCPD                    //No EE protection
#FUSES NOPUT                    //No Power Up Timer
#FUSES IESO                     //Internal External Switch Over mode enabled
#FUSES FCMEN                    //Fail-safe clock monitor enabled

#use delay(clock=8000000)

#use rs232(baud=38400,parity=N,xmit=PIN_C4,rcv=PIN_C5,bits=8,errors)

void get_gyro_offset();
void read_gyro();
