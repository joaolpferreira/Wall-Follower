#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "serial_printf.h"
#include "lcd1602.h"
#include "i2c.h"


//Motors
#define IN1 PD7
#define ENA PD6
#define ENB PD5
#define IN2 PD4
#define IN3 PD3
#define IN4 PD2
//Trigs
#define	TrigF PB0
#define TrigL PB3
#define TrigR PB2
//Echos
#define EchoF PB1
#define EchoL PB5
#define EchoR PB4

#define LEFT_MIN_DIST 8
#define LEFT_MED_DIST 12
#define LEFT_MAX_DIST 20

#define MIN_DIST 30
#define TURN_TIME 250
#define DELAY_TIME 5000
#define MAX_SPEED 170
#define MED_SPEED 160
#define MIN_SPEED 155

#define BUILTIN_LED_PIN PB5
#define VREF 5

//LCD
void init_adc(void) {
 	// Definir Vref=AVcc
 	ADMUX = ADMUX | (1<<REFS0);
 	// Desativar buffer digital em PC0
 	DIDR0 = DIDR0 | (1<<PC0);
 	// Pré-divisor em 128 e ativar ADC
	ADCSRA = ADCSRA | (7<<ADPS0)|(1<<ADEN);
}

unsigned int analogread(unsigned char chan) {
 	// escolher o canal...
 	ADMUX = (ADMUX & 0xF0) | (chan & 0x0F);
 	// iniciar a conversão
 	// em modo manual (ADATE=0)
 	ADCSRA |= (1<<ADSC);
 	// esperar pelo fim da conversão
 	while(ADCSRA & (1<<ADSC));
 	return ADC;
}

void reverse(char* str, int len) 
{ 
    int i = 0, j = len - 1, temp; 
    while (i < j) { 
        temp = str[i]; 
        str[i] = str[j]; 
        str[j] = temp; 
        i++; 
        j--; 
    } 
} 

int intToStr(int x, char str[], int d) 
{ 
    int i = 0; 
    while (x) { 
        str[i++] = (x % 10) + '0'; 
        x = x / 10; 
    } 
    // If number of digits required is more, then 
    // add 0s at the beginning 
    while (i < d) 
        str[i++] = '0'; 
  
    reverse(str, i); 
    str[i] = '\0'; 
    return i; 
} 
  
  void ftoa(float n, char* res) 
{ 
    // Extract integer part 
   	int ipart = (int)n; 
  	// convert integer part to string 
    int j = intToStr(ipart, res, 0); 
} 

int printbattery() 
{
	DDRC=0b00000000;
	PORTC=0b00000000;
	unsigned int sensorvalue = analogread(4);
	//unsigned int sensorvalue = PINC4;
 	float voltage =sensorvalue*VREF/1024;
 	float bat = (voltage * 100)/4.5;
 	char bateria[20];
 	ftoa(bat,bateria);
  	lcd1602_goto_xy(0,0);
 	lcd1602_send_string(bateria);
  	lcd1602_send_char('%');
	return voltage;
 }

//Robõ
volatile unsigned short hc_sr04_cntF, hc_sr04_cntL, hc_sr04_cntR; 		//Reading results
uint8_t count; 															//Reading count
volatile uint16_t Time_right, Time_left;								//Turn timers
volatile uint8_t flag;													//decide wich ultrasonic to read
int delta;																//Difference between R and L


void motors_init()   {
     //Motors
    DDRD |= (1 << IN1) | (1 << ENA) | (1 << ENB) | (1 << IN2) | (1 << IN3) | (1 << IN4);
    PORTD = 0b00000000;
}

void Ultrasonic_init()
{	
	//Front
	DDRB |= (1 << TrigF);   
	PORTB &= ~(1 << TrigF);    
	DDRB &= ~(1 << EchoF);   
	PORTB &= ~(1 << EchoF);  

	//Left
	DDRB |= (1 << TrigL); 
	PORTB &= ~(1 << TrigL);
	DDRB &= ~(1 << EchoL);
	PORTB &= ~(1 << EchoL);

	//Right
	DDRB |= (1 << TrigR); 
	PORTB &= ~(1 << TrigR);
	DDRB &= ~(1 << EchoR);
	PORTB &= ~(1 << EchoR);

	PCICR |= (1 << PCIE0);     	// set PCIE0 to enable PCMSK0 scan
    PCMSK0 |= (1 << PCINT1);	// set PCINT1 to trigger an interrupt on state change
	PCMSK0 |= (1 << PCINT4);
	PCMSK0 |= (1 << PCINT5);   
    sei();                    	// turn on interrupts
}

//TIMERS
 	//PMW motors
void Timer0_init()      {
    //Activating OC0A and OC0B in Fast PWM
	TCCR0A |= (1 << COM0A1) | (1 << COM0B1) | (1 << WGM00) | (1 << WGM01);
	// set up timer with prescaler 8
	TCCR0B |= (1 << CS01);
    //Initializing Bottom and Top
    TCNT0 = 0;
	OCR0A = 0;
	OCR0B = 0;
}

	//TCNT1 with 4us ticks
void Timer1_init(){
	TCCR1B = (0<<CS12) | (1<<CS11) | (1<<CS10);
	sei();
}
	//1ms ticks
void Timer2_init()  {   //timer 1ms
    OCR2A = 250;                 // Compare value
    TCCR2A |= (1 << WGM21);     // Set to CTC Mode
    TIMSK2 |= (1 << OCIE2A);    // Set interrupt on compare match
    TCCR2B |= (1 << CS22);      // Set prescaler to 64
    sei();                      // Enable interrupts
}

//Echo Interrupts
ISR( PCINT0_vect )
{	
	if(PINB & (1 << EchoF)) {TCNT1 = 0; flag=1;} //Clear Timer counter 
	else  if(flag==1){hc_sr04_cntF = TCNT1; flag=0;}
	
	else if(PINB & (1 << EchoL)) {TCNT1 = 0; flag=2;} //Clear Timer counter 
	else if(flag==2) {hc_sr04_cntL = TCNT1; flag=0;}
	
	else if(PINB & (1 << EchoR)) {TCNT1 = 0, flag=3;}/* Clear Timer counter */
	else if(flag==3) {hc_sr04_cntR = TCNT1; flag=0;}
}

//1ms Interrupt
ISR (TIMER2_COMPA_vect)
{
	if(Time_left>0){
        cli();
        Time_left--;
        sei(); 
    }

	if(Time_right>0){
        cli();
        Time_right--;
        sei(); 
    }
}

//Motors Movements
/*
void motor_foward(){
	uint8_t p=0;
    if(delta<0){
		if(-20<delta && delta<=-10) p=2;
		if(-30<delta && delta<=-20) p=3;
		if(-200<delta && delta<=-30) p=4;
		OCR0A=MAX_SPEED-p*delta;
		OCR0B=MAX_SPEED;
	}
	 if(delta>=0){
		if(10<delta && delta<=20) p=1;
		if(20<delta && delta<=30) p=2;
		if(30<delta && delta<=200) p=3;
		OCR0A=MAX_SPEED;
		OCR0B=MAX_SPEED-p*delta;
	}
    PORTD |=   (1 << IN1) | (1 << IN3);
    PORTD &= ~((1 << IN2) | (1 << IN4));
}
*/

void turn_right(){
	OCR0A=170;
    OCR0B=170;
	PORTD |=   (1 << IN1) | (1 << IN4);
	PORTD &= ~((1 << IN2) | (1 << IN3));
}

void turn_left(){
	OCR0A=170;
    OCR0B=170;
    PORTD |=   (1 << IN2) | (1 << IN3);
    PORTD &= ~((1 << IN1) | (1 << IN4));
}

void motor_foward_left(uint16_t smL){
	if(smL>LEFT_MIN_DIST && smL<LEFT_MED_DIST) 			{OCR0A=MAX_SPEED-5; OCR0B=MAX_SPEED-7;}
	else if(smL>LEFT_MED_DIST && smL<LEFT_MAX_DIST) 	{OCR0A=MIN_SPEED; OCR0B=MED_SPEED-4;}
	else if(smL>25) 									{OCR0A=MED_SPEED-1; OCR0B=MIN_SPEED;}
	else if(smL<LEFT_MIN_DIST)							{OCR0A=MED_SPEED; OCR0B=MIN_SPEED;}
	PORTD |=   (1 << IN1) | (1 << IN3);
    PORTD &= ~((1 << IN2) | (1 << IN4));
}

void turn_left_cont(){
	OCR0A=160;
    OCR0B=180;
    PORTD |=   (1 << IN1) | (1 << IN3);
    PORTD &= ~((1 << IN2) | (1 << IN4));
}

void turn_right_cont(){
	OCR0A=170;
    OCR0B=100;
	PORTD |=   (1 << IN1) | (1 << IN3);
	PORTD &= ~((1 << IN2) | (1 << IN4));
}

void turn_left_cont2(){
	OCR0A=150;
    OCR0B=180;
    PORTD |=   (1 << IN1) | (1 << IN3);
    PORTD &= ~((1 << IN2) | (1 << IN4));
}

void motor_stop(){
	OCR0A=0;
	OCR0B=0;
}


//Reading Distances
unsigned char hc_sr04_measF( void )
{
	//Trig
	hc_sr04_cntF = 0;
	TCNT1=0;
	PORTB |=  (1 << TrigF);
	while(TCNT1<3);
	PORTB &= ~(1 << TrigF);
	//Echo
	TCNT1 = 0;
	while( (hc_sr04_cntF == 0) ){
		if(TCNT1>3500) return 250;
	}
	if (hc_sr04_cntF<4000) {return 0.000004 * hc_sr04_cntF/2 * 34300;} else {return 250;}
}

unsigned char hc_sr04_measL( void )
{	
	//Trig
	hc_sr04_cntL = 0;
	TCNT1=0;
	PORTB |=  (1 << TrigL);
	while(TCNT1<3);
	PORTB &= ~(1 << TrigL);
	//Echo
	TCNT1 = 0;
	while( (hc_sr04_cntL == 0)){
		if(TCNT1>3500) return 250;
	}
	if (hc_sr04_cntL<4000) {return 0.000004 * hc_sr04_cntL/2 * 34300;} else {return 250;}	
}

unsigned char hc_sr04_measR( void )
{
	//Trig
	hc_sr04_cntR = 0;
	TCNT1=0;
	PORTB |=  (1 << TrigR);
	while(TCNT1<3);
	PORTB &= ~(1 << TrigR);
	//Echo
	TCNT1 = 0;
	while( (hc_sr04_cntR == 0)){
		if(TCNT1>3500) return 250;
	}
	if (hc_sr04_cntR<4000) {return 0.000004 * hc_sr04_cntR/2 * 34300;} else {return 250;}	
}




//MAIN CODE
int main() {

	printf_init();
	Ultrasonic_init();
	motors_init();
   	Timer0_init();
    Timer1_init();
	Timer2_init();
//LCD
	int valor=0;
   	lcd1602_init();
    init_adc();
    lcd1602_clear();
	valor=printbattery();
    printf("QQcoisa %d\n", valor);

	uint16_t smF=0, smL=0, smR=0, smF_old=0, smL_old=0, smR_old=0;
	uint8_t main_st=1, n_count=0, lap=0;

//LOOP
while(1){
	//Measures	
		//Front
		while(n_count<10){
			smF += hc_sr04_measF();
			n_count++;
		}
		n_count=0;	
		//Right
		while(n_count<10){
			smR += hc_sr04_measR();
			n_count++;
		}	
		n_count=0;
		//Left
		while(n_count<10){
			smL += hc_sr04_measL();
			n_count++;
		}	
		n_count=0;
		//average of each measure on a 10 count base
		smR=smR/10;
		smL=smL/10;
		smF=smF/10;
		
		printf("Tempo FRONT %u\n", smF);
		printf("Tempo RIGHT %u\n", smR);
		printf("Tempo LEFT %u\n", smL);
		delta=smR-smL;
		printf("DELTA %d\n", delta);
		printf("st %d\n\n", main_st);

/*
		if(main_st==1 && smF<20)				{main_st=2; Time_left=DELAY_TIME;}
		else if(main_st==2 && !(Time_left))		{main_st=3; Time_left=TURN_TIME;}
		else if(main_st==3 && !Time_left)		{main_st=4;}
		else if(main_st==4 && smF<20)			{main_st=5; Time_left=DELAY_TIME;}
		else if(main_st==5 && !(Time_left))		{main_st=6; Time_left=TURN_TIME;}
		else if(main_st==6 && !(Time_left))		{main_st=7; }

		if(main_st==1)		{motor_foward_left(smL);}
		else if(main_st==2)	{motor_stop();}
		else if(main_st==3) {turn_right();}
		else if(main_st==4)	{motor_foward_left(10);}
		else if(main_st==5) {motor_stop();}
		else if(main_st==6)	{turn_right();}
		else if(main_st==7)	{motor_stop();}
		*/

//State Machine
	//Conditions	
		if		(main_st==1 && smL_old+MIN_DIST<smL)       			{main_st=2; 	}
    	else if	(main_st==2 && smL<40)  							{main_st=3;		}
    	else if	(main_st==3 && smF<25)  							{main_st=4;		} 
    	else if	(main_st==4 && smF>30)  							{main_st=5; 	}
		else if	(main_st==5 && smL_old+MIN_DIST<smL)  				{main_st=6; Time_left=1800;	}
		else if	(main_st==6 && !Time_left)  						{main_st=7;	}
		else if	(main_st==7 && smF<20)  							{main_st=8;		}
		else if	(main_st==8 && smF>30)  							{main_st=9; 	}
		else if	(main_st==9 && smL_old+MIN_DIST<smL)  				{main_st=10; 	}
		else if	(main_st==10 && smL<30)  							{main_st=11;	}
		else if	(main_st==11 && smF<20)       						{main_st=12; 	}
	
	//Outputs
		if(main_st==1)	{motor_foward_left(smL);}
		if(main_st==2)	{turn_left_cont();}
		if(main_st==3)	{motor_foward_left(smL);}
		if(main_st==4)	{turn_right_cont();}
		if(main_st==5)	{motor_foward_left(smL);}
		if(main_st==6)	{turn_left_cont2();}
		if(main_st==7) 	{motor_foward_left(smL);}
		if(main_st==8) 	{turn_right_cont();}
		if(main_st==9) 	{motor_foward_left(smL);}
		if(main_st==10)	{turn_left_cont();}
		if(main_st==11)	{motor_foward_left(smL);}
		if(main_st==12)	{motor_stop();}

/*
//State Machine
		if(main_st==1 && smL_old+MIN_DIST<smL)       		{main_st=2; Time_left=DELAY_TIME;	}
    	else if(main_st==2 && !(Time_left))  				{main_st=3; Time_left=TURN_TIME;	}
    	else if(main_st==3 && !(Time_left))  				{main_st=4; Time_left=DELAY_TIME;						} //1 - Turn_Left completed
    	else if(main_st==4 && !(Time_left))  				{main_st=5; 	}
    	else if(main_st==5 && smF<10)  						{main_st=6;	Time_left=DELAY_TIME;						} //2 - Turn_Right completed
		else if(main_st==6 && !(Time_left))  				{main_st=7; Time_left=TURN_TIME;	}	
		else if(main_st==7 && !(Time_left))  				{main_st=8; Time_left=DELAY_TIME;	} //3 - Turn_Left completed
		else if(main_st==8 && !(Time_left))  				{main_st=9; Time_left=TURN_TIME;	}
		else if(main_st==9 && !(Time_left))  				{main_st=10; Time_left=DELAY_TIME;						} //4 - Turn_Left completed
		else if(main_st==10 && smF<10)  					{main_st=11; Time_left=TURN_TIME;	}
		else if(main_st==11 && !(Time_left))  				{main_st=12; Time_left=DELAY_TIME;						} //5 - Turn_Right completed
		else if(main_st==12 && smL_old+MIN_DIST<smL)  		{main_st=13; Time_left=DELAY_TIME;	}
		else if(main_st==13 && !(Time_left))  				{main_st=14; Time_left=TURN_TIME;	}
		else if(main_st==14 && !(Time_left))  				{main_st=1; count++;				} //6 - Turn_Left completed
		if(count==2) {count=0; lap++;}


		if(main_st==1)	{motor_foward_left(smL);}
		if(main_st==2)	{motor_stop();}
		if(main_st==3)	{turn_left();}
		if(main_st==4)	{motor_stop();}
		if(main_st==5)	{motor_foward_left(smL);}
		if(main_st==6)	{motor_stop();}
		if(main_st==7)	{turn_right();}
		if(main_st==8)	{motor_stop();}
		if(main_st==9)	{motor_foward_left(smL);}
		if(main_st==10)	{motor_stop();}
		if(main_st==11)	{turn_left();}
		if(main_st==12)	{motor_stop();}
		if(main_st==13)	{motor_foward_left(10);}
		if(main_st==14)	{motor_stop();}
		if(main_st==15)	{turn_left();}
		if(main_st==16)	{motor_stop();}
		if(main_st==17)	{motor_foward_left(smL);}
		if(main_st==18)	{motor_stop();}
		if(main_st==19)	{turn_right();}
		if(main_st==20)	{motor_stop();}
		if(main_st==21)	{motor_foward_left(smL);}
		if(main_st==22)	{motor_stop();}
		if(main_st==23)	{turn_left();}

*/
		
		smF_old=smF;
		smL_old=smL;
		smR_old=smR;

		smF=0;
		smL=0;
		smR=0;

		//wait 250ms
		//TCNT1=0;
		//while(TCNT1<62500);
		
	}
}
