#include <HeadTracking.h>
#define canal_AD 0
#define ajuste_drift_h 2
#define ajuste_drift_l -2
#define pin_test pin_c2

short bit_test=true;
long value_gyro;
float gyro_offset;
signed long pos_gyro=0;
float ADC_Samples[4];
float integracion=0;
float theta=0, theta_old=0;
signed long f1,f2;
long contador=0;
int ciclos=6;
#define a 0.2
#define aa 0.001   // aa is the filter time-constant to take care of the gyro drift
#define vref 3300.
#define samples 1023.
#define scale 0.00067 //0.67 mv/deg/s

/*
you can integrate in discrete domain

Suppose the angular rate is da, it's time integral is a. k is the discrete step number.

a(k) = a(k-1) + T*0.5*(da(k) + da(k-1))

for example, da(k) is current angular rate reading. da(k-1) is previoud angular rate reading. a(k-1) is the previous step's integration value(rotation angle)

T is sampling rate. if the sensor outputs in every 1 mili second, T becomes 0.001

you can use this formula when k>0 The initial value, a(0), must be given.
*/

 //RUTINAS DE INTERRUPCION
#INT_TIMER0
void interrupcion(void)
{
  int8 i;
 // signed long deriva;
  long adc_tmp=0;
  set_timer0(99);                //carga TMR0

    //   ciclos--;
    //   if (ciclos==0){
    //   if (bit_test) output_high(pin_test);
    //   else output_low(pin_test);
    //   bit_test=~bit_test;
    output_high(pin_test);
    set_adc_channel(canal_AD); 
    delay_us(15);
    adc_tmp=read_adc();
    delay_us(15);
    i=4;
    while (i>0){
      ADC_Samples[i-1]=ADC_Samples[i-2];
      i--;
    }
    ADC_samples[0]=(adc_tmp*vref/samples);
    ADC_samples[0]-=gyro_offset;
    //Integracion Runge-kutta
    theta = theta_old + (((ADC_samples[3]+(2*ADC_samples[2])+(2*ADC_samples[1])+ADC_samples[0])/6)*0.01/0.67);
    theta_old = theta;
    output_low(pin_test);
    contador++;
   
   

  }
 


void get_gyro_offset()          //Calcula gyro offset
/*
      // filter the sensor output
      Gyro_value = SensorValue(GyroSensor);
      wait1Msec(100);
       GyroBias = (1-a)*GyroBias + a*Gyro_value; */
{ 
    int8 i;
    int iNumLec = 90;
    long iTemp;
    long Gyro_value;

    set_adc_channel(canal_AD);   //Leer Gyro 
    delay_us(15);               //Settle time
    iTemp = 0;
    for (i=0;i<iNumLec;i++)    //Captura 100 lecturas
    {
      gyro_value=read_adc();
      iTemp = itemp + gyro_value; 
      delay_ms(10);
  //    printf("itemp:%Ld \n\r",gyro_value);
    }
    iTemp = iTemp / iNumLec;  //Media de 100 lecturas
 //   printf("Offset:%Ld \n\r",itemp);
    gyro_offset = (iTemp*vref/samples);
 //   printf("Offset:%f2 \n\r",gyro_offset);
 //   gyro_offset = (float)(gyro_offset*0.00322);
 //   printf("Offset:%f3 \n\r",gyro_offset);
    
} 

void read_gyro(){
  int8 i;
 // signed long deriva;
  long adc_tmp=0;
  set_adc_channel(canal_AD); 
  delay_us(15);
  adc_tmp=read_adc();
  delay_us(15);
  i=4;
  while (i>0){
   ADC_Samples[i-1]=ADC_Samples[i-2];
   i--;
  }
  ADC_samples[0]=(adc_tmp*vref/samples);
//  printf("ADC%F2 \n\r",ADC_Samples[0]);
 // printf("Offset:%f2 \n\r",gyro_offset);
  ADC_samples[0]-=gyro_offset;
     //Integracion Runge-kutta
     theta = theta_old + (((ADC_samples[3]+(2*ADC_samples[2])+(2*ADC_samples[1])+ADC_samples[0])/6)*0.05/0.67);
     theta_old = theta;
     
      printf("%F3 \n",theta);
//   printf("ADC_samples[1]:%F3 \n\r",ADC_Samples[1]);
//   printf("ADC_samples[2]:%F3 \n\r",ADC_Samples[2]);
//   printf("ADC_samples[3]:%F3 \n\r",ADC_Samples[3]);


     
}

void main()
{
   setup_oscillator(OSC_8MHZ);
   setup_adc_ports(sAN0|VSS_VDD);
   setup_adc(ADC_CLOCK_INTERNAL);
   setup_comparator(NC_NC_NC_NC);
   setup_vref(FALSE);
   setup_timer_0(T0_INTERNAL|T0_DIV_128|T0_8_BIT);
   delay_ms(1000);
   printf("Radikaldesig.com Gyro Control Head Tracking\r\n");
   get_gyro_offset(); 
   delay_ms(100);
   enable_interrupts(global);
   ENABLE_INTERRUPTS(INT_TIMER0);
   set_timer0(99);                //carga TMR0
   ADC_Samples[0]=0;
   ADC_Samples[1]=0;
   ADC_Samples[2]=0;
   ADC_Samples[3]=0;
    theta_old = 0;
    f1=0;

   while(contador<15000){
  // printf("Offset:%Ld \n\r",gyro_offset);
   printf("%F3 \n",theta);
   //delay_ms(10);

   }
}


