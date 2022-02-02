

/*Servo Driver for Attiny1616
 * Maulion Charles 02/01/2020
 * Use Megatinycore library
 * Internal clock 10mhz
 * !!!!!!!!!!use millis TCB0!!!!!!!!!!!!! 
 * 
 * PA4 : hall effect
 * PA2 : servo signal
 * 
 * For now 2 outputs : 
 * PB4/PB5 = -motor
 * PC1/PC2 = +motor
 * 
 * for the pcb 
 * PB2/PB3/PB4/PB5 = -motor
 * PB0/PC0/PC1/PC2 = +motor
 * 
 * adjust Steps => adjust output frequency => 64 steps arround 2khz
 * 
 * 
 * 
 */
 
// Install Pin change interrupt for a pin, can be called multiple times

int pciTime = 0;
byte prev_pinState = 0;
unsigned long pwmPeriod = 0 ;
int pwmTimer = 0;

int PW = 1500;
byte  pwmFlag = 0 ;


/*Pwm count*/
volatile unsigned char _time_count=0;      // time++
int steps = 8; // lower steps control but higher frequency output motor 8,16,32,64,128


int PWM1=0;
int PWM2=0;
int duration =0;
int setpoint =0;


int srv_min_read=1000;//850
int srv_max_read=2000;
int srv_mid_val=1500;
int delta = 150;
int angle_out_min=512-delta;//450
int angle_out_max=512+delta;//450
int srv_value=1500;




/*byte debug*/
byte debug = 0;


/**********servo**********/
int ADC_SetPoint_S1 = 0;
int ADC_SetPoint_S1_in = 0;
int ADC_SetPointOld_S1 = 0;
int ADC_ServoPoti_S1 = 512;
int ADC_ServoPotiOld_S1 = 512;
int dutyCycle_S1 = 10; // 10 - 255
int ADCdiff_S1 = 0;
int timeDiff_S1 = 0;





//Change values below to adapt your motor
//Set MAX_DUTYCYCLE_S1 and MAX_DUTYCYCLE_S2 to 75 for the first test run!

#define P_FRACTION_S1 4          //0.0 - 10.0 (0.3)JJKWAng use =1
#define I_FRACTION_S1 4          //0.0 - 10.0 (0.3)JJKWAng use =1
#define D_FRACTION_S1 4          //0.0 - 10.0 (4.0)JJKWAng use =1
#define V_WINDOW_S1 4             //10 - 1000 (25)
#define MIN_DUTYCYCLE_S1 steps/2     //0 - 255 (125)
#define MAX_DUTYCYCLE_S1 steps     //0 - 255 (255)
#define SOFT_START_S1 1.00          //0.00 - 1.00 (0.30) 1.00 = OFF
#define EMERGENCY_SHUTDOWN_S1 0  //0 - 1000 (300), 0 - OFF, Stops motor if blocked



#define SERVO_SENSOR_S1 0       //Analog input servo sensor A4



/* Pin interrupt servo, read servo pulse Input*/
ISR (PORTA_PORT_vect) // handle pin change interrupt for A2
{
PORTA.INTFLAGS=4; //we know only PB0 has an interrupt, so that's the only flag that could be set. Px0=1 / Px2=4 etc

pciTime = micros();  
    if(prev_pinState == 0 )
        {          // and the pin state has changed from LOW to HIGH (start of pulse)
        prev_pinState = 1;                                     // record pin state
        pwmPeriod = pciTime - pwmTimer;                     // calculate the time period, micro sec, between the current and previous pulse
        pwmTimer = pciTime;                                    // record the start time of the current pulse
        }
      else if (prev_pinState == 1 )
        { // or the pin state has changed from HIGH to LOW (end of pulse)
        prev_pinState = 0;                                     // record pin state
        PW = pciTime - pwmTimer;                            // calculate the duration of the current pulse
        pwmFlag = HIGH;   // flag that new data is available
      //  if(i+1 == RC_inputs) RC_data_rdy = HIGH;
        }



}  



/*Interrupt PWM*/
ISR(TCB1_INT_vect)
{
  /*
  static uint8_t t;
  if ( t < PWM1 ) PORTB.OUTSET = _BV(4) | _BV(5);
  else            PORTB.OUTCLR = _BV(4) | _BV(5);
  if ( t < PWM2 ) PORTC.OUTSET = _BV(1) | _BV(2);
  else            PORTC.OUTCLR = _BV(1) | _BV(2);
  t += 4;     // 64 steps
  TCB1.INTFLAGS = TCB_CAPT_bm;
  */

  
  if(_time_count<steps)
  {
    
//TCB1.INTFLAGS= TCB_CAPT_bm; //clear interrupt flag


if ( _time_count < PWM1 ) PORTB.OUTSET = _BV(2) |_BV(3) |_BV(4) | _BV(5);
else            PORTB.OUTCLR = _BV(2) |_BV(3) |_BV(4) | _BV(5);
if ( _time_count < PWM2 ) 
{
  PORTC.OUTSET = _BV(0) |_BV(1) | _BV(2);
  PORTB.OUTSET = _BV(0);
}
else            
{
  PORTC.OUTCLR = _BV(0) |_BV(1) | _BV(2);
  PORTB.OUTCLR = _BV(0);
}
_time_count++;           // 변수 증가  0~255 > 0~255 반복
  }
  

  else 
  {
    _time_count=0;

  }
TCB1.INTFLAGS= TCB_CAPT_bm; //clear interrupt flag

}


 
 

 
void setup() {  
PORTB.DIR = 0b00111101;  // set PORTB PB0/PB2/PB3/PB4/PB5
//PORTB.DIR = 0b00110000;  // set PORTB PB4/PB5
PORTC.DIR = 0b00000111; // set PORTB PC0/PC1/PC2
//PORTC.DIR = 0b00000110; // set PORTB PC1/PC2
PORTA.DIR = 0b00000000; //PA2 in servo / PA4 in feedback

/*input Servo pin  interrupt*/
PORTA.PIN2CTRL=0b00001001; //PULLUPEN=1, ISC=1 trigger both


if (debug == 1)
{
Serial.begin(115200);
delay(10);
Serial.println("Startup");
}


/*configure timer TCB1 for pwm */

TCB1.CTRLA= TCB_ENABLE_bm;
TCB1.CNT=0;
TCB1.CTRLB=0; // periodic interrupt mode
TCB1.CCMP = 78*2; // 
TCB1.INTCTRL=TCB_CAPT_bm;
//TCB1.CTRLA= TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;

delay(2000);

}

void servo1()
{
  //Servo 1 ----------------------------------------------------------
  ADC_ServoPoti_S1 = analogRead(SERVO_SENSOR_S1);     // reads the servo sensor (between 0 and 1023) 

  ADCdiff_S1 = srv_value - ADC_ServoPoti_S1  ;
  
  dutyCycle_S1 = abs(ADCdiff_S1) * P_FRACTION_S1;
  dutyCycle_S1 += timeDiff_S1 * I_FRACTION_S1;
  dutyCycle_S1 += abs(ADC_SetPointOld_S1 - ADC_SetPoint_S1) * D_FRACTION_S1;
    
  if(SOFT_START_S1 * timeDiff_S1 < 1){
    dutyCycle_S1 = dutyCycle_S1 * (SOFT_START_S1 * timeDiff_S1);
  }
  
  timeDiff_S1++;
  
  if(dutyCycle_S1 < MIN_DUTYCYCLE_S1 && dutyCycle_S1 > 0){
    dutyCycle_S1 = MIN_DUTYCYCLE_S1;
  }
  
  if(dutyCycle_S1 > MAX_DUTYCYCLE_S1){
    dutyCycle_S1 = MAX_DUTYCYCLE_S1;
  }
  
  if(dutyCycle_S1 < 0){
    dutyCycle_S1 = 0;
  }
  
  
  if(abs(ADCdiff_S1) < V_WINDOW_S1){
    dutyCycle_S1 = 0;
    timeDiff_S1 = 0;
  }

  if(dutyCycle_S1 == MAX_DUTYCYCLE_S1 && timeDiff_S1 > EMERGENCY_SHUTDOWN_S1 && EMERGENCY_SHUTDOWN_S1 > 0){
  PWM1 = 0;
  PWM2 = 0;
    //delay(1000);
    timeDiff_S1 = 0;
  }
  else{  
    if(ADCdiff_S1 > 0){
      PWM1=0;
      PWM2 = dutyCycle_S1;
    }
    if(ADCdiff_S1 < 0){
      PWM1 = dutyCycle_S1;
      PWM2 = 0;
    }
  }
  

  ADC_SetPointOld_S1 = ADC_SetPoint_S1;
  ADC_ServoPotiOld_S1 = srv_value;


  
}
 
 
void loop() {
/*
if (PW < srv_min_read) // a check
{
  PW = srv_min_read;
}
if (PW > srv_max_read)
{
  PW = srv_max_read;
}
*/

//srv_value = map(PW,850,2000,300,550);// no logic but thats run, need to be check
srv_value = map(PW,srv_min_read,srv_max_read,angle_out_min,angle_out_max);// no logic but thats run, need to be check

 if ( debug == 0)
 {
delay(1);
 }
 

 servo1();

 if ( debug == 1)
 {
   Serial.print(PW);  Serial.print("\t"); Serial.print(_time_count);  Serial.print("\t"); Serial.print(srv_value); Serial.print("\t");  Serial.print(ADC_ServoPoti_S1);

 }



}
