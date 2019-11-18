// V2
// Status: A-syncronous stepper motor contol 
// 				- Multiple loading
//				- Sorting correctly, taking shortest path
//				- Acceleration is added

// For those that are still having major problems, I've seen about 1/3 of the class with major problems in 
// code structure. If you are still having major problems with your code, it's time to do a VERY quick overhaul.
// I've provided a skeleton structure with an example using two input capture interrupts on PORTDA0 and A3
// Please try this in the debugger.

// Create a watch variable on STATE. To do this right click on the variable STATE and then
// Add Watch 'STATE'. You can see how the variable changes as you click on PINDA0 or PINDA3. Note that the interrupt
// catches a rising edge. You modify this to suit your needs.

//#define F_CPU 8000000UL

volatile int future_dir, current_dir, current_delay, predicted_steps, future_steps;

#include <stdlib.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <avr/io.h>
#include <util/delay.h>

#include "lcd.h"
#include "linked_queue.h"
#include "defs.h"

//#define LCD_DEBUG // Have LCD show current sorting progress

typedef enum
{
	WAITING_FOR_FIRST,
	MOVING_ITEM_TO_GATE,
	GATE_CHECK,
	PAUSE,
	RAMPDOWN
} state_t;

// Global Variables
volatile state_t state;

volatile uint16_t ADC_count;
volatile uint16_t reflect_min;

volatile uint8_t BLK_COUNT;
volatile uint8_t WHITE_COUNT;
volatile uint8_t STEEL_COUNT;
volatile uint8_t ALUM_COUNT;

volatile uint8_t BUCKET_COUNT;
volatile uint8_t GATE_COUNT;

volatile uint8_t is_paused;

// item queue
link * head, * tail, * new_link, * rtn_link, * unknown_item;

const unsigned char STEPPER_ARRAY[] = {0b110110,0b101110,0b101101,0b110101};
int8_t current_stepper;
int16_t stepper_pos;

int current_pos;

#define STEEL_BOUND 260
#define WHITE_BOUND 700
#define BLACK_BOUND 884

#define DUTY_CYCLE 0x30 // Expressed as ratio of 0xff (i.e. 0x80 = 50% duty)

#define ACCELERATION  1
#define DECELERATION  1
#define NUM_ACCEL_STEPS 12
#define NUM_DECEL_STEPS 12

#define FULL_SPEED 6
#define DROP_STEP 14

volatile uint8_t CURRENT_DELAY = FULL_SPEED + ( ACCELERATION * NUM_ACCEL_STEPS );

//--------------------  set_belt  --------------------

inline void set_belt(char is_on)
{
	if (is_on)
		PORTD = 0b00010000;
	else
		PORTD = 0;
}


//--------------------  mTimer  --------------------

void mTimer (int count)
{
   int i;

   i = 0;

   TCCR1B |= _BV (CS10);  //  sets prescalar to DIV 1
   /* Set the Waveform gen. mode bit description to clear
     on compare mode only */
   TCCR1B |= _BV(WGM12);
   /* Set output compare register for 1000 cycles, 1ms */
   OCR1A = 0x03E8;
   /* Initialize Timer 1 to zero */
   TCNT1 = 0x0000;
   /* Clear the timer interrupt flag and begin timing */
   TIFR1 |= _BV(OCF1A);

   /* Poll the timer to determine when the timer has reached 1ms */
   while (i < count)
   {
      while ((TIFR1 & 0x02) != 0x02);
	  /* Clear the interrupt flag by WRITING a ONE to the bit */
	  TIFR1 |= _BV(OCF1A);
	  i++;
   } /* while */
   TCCR1B &= ~_BV (CS10);  //  disable prescalar
   return;
}

//--------------------  stepper_movement  --------------------

inline void stepper_movement ()
{

	int8_t j = current_stepper;

	j++;
	if(j==4)
		j=0;

	PORTA = STEPPER_ARRAY[j];
	mTimer(20);

	current_stepper = j;
}

//--------------------  categorize  --------------------

inline material_t categorize(uint16_t reflect_min)
{
	if (reflect_min >= BLACK_BOUND)
	{
		BLK_COUNT++;
		return BLACK;
	}
	else if (reflect_min < BLACK_BOUND && reflect_min >= WHITE_BOUND)
	{
		WHITE_COUNT++;
		return WHITE;
	}
	else if (reflect_min < WHITE_BOUND && reflect_min >= STEEL_BOUND)
	{
		STEEL_COUNT++;
		return STEEL;
	}
	else
	{
		ALUM_COUNT++;
		return ALUM;
	}
}

//--------------------  lcd_message  --------------------

inline void lcd_message(const char * msg)
{
	LCDClear();
	LCDWriteString(msg);
	mTimer(20);
}

//--------------------   zero_tray   --------------------
void zero_tray()
{
	lcd_message("Calibrating tray..");

	while ((PIND & 0b1000) != 0)
	{
		stepper_movement();
	}
	stepper_pos = 0;
	LCDClear();

	lcd_message("Waiting for item..");
}

//--------------------     pause    --------------------
void pause()
{
	set_belt(0);
}

//--------------------   unpause    --------------------
void unpause()
{
	set_belt(1);
}

//--------------------   rampdown   --------------------

void rampdown()
{


}

//############################## MAIN ##############################

int main(){

	cli();	// Disables all interrupts

	// Going to set up interrupt0 on PD0

	// pin F1 RL
	// pin D0 OR
	// pin D2 EX
	// pin D3 HE
	DDRA = 0xFF;
	DDRB |= 0x80; // Sets OC0A pin to output (B7 pin)
	DDRC = 0xFF;	// just use as a display
	DDRD |= 0xf0;	// DC motor control out and 
	DDRE = 0x00; // Button input

	set_belt(0); // disable DC motor

// ----------   CONFIGURING INTERRUPTS    ----------  

	// OR / ADC sensor
	EIMSK |= (_BV(INT0)); // enable INT0
	EICRA |= (_BV(ISC01) | _BV(ISC00)); // rising edge interrupt
	// OI /  Initial detection
	EIMSK |= (_BV(INT1)); // enable INT1
	EICRA |= (_BV(ISC11)); // falling edge interrupt
	// EX / Bucket
	EIMSK |= (_BV(INT2)); // enable INT2
	EICRA |= (_BV(ISC21)); // falling edge interrupt
	//EICRA |= (_BV(ISC21) | _BV(ISC20)); // falling edge interrupt

	// Pause button
//	EIMSK |= (_BV(INT6)); // enable INT6
//	EICRB |= (_BV(ISC61)); // falling edge interrupt

	// Ramp-down button
//	EIMSK |= (_BV(INT7)); // enable INT7
//	EICRB |= (_BV(ISC71)); // falling edge interrupt

	// Set Interrupt sense control to catch a rising edge
//	EICRA |= _BV(ISC01) | _BV(ISC00);
//	EICRA |= _BV(ISC31) | _BV(ISC30);

//	EICRA &= ~_BV(ISC01) & ~_BV(ISC00); /* These lines would undo the above two lines */
//	EICRA &= ~_BV(ISC31) & ~_BV(ISC30); /* Nice little trick */

	// See page 96 - EIFR External Interrupt Flags...notice how they reset on their own in 'C'...not in assembly
//	EIMSK |= 0x09;

	// config ADC =========================================================
	// the ADC input (analog input is set to be ADC1 / PORTF1
	ADCSRA |= _BV(ADEN); // enable ADC
	ADCSRA |= _BV(ADIE); // enable interrupt of ADC
	ADMUX |= _BV(REFS0); 
	ADMUX |= 0b1;


// ----------   Starting Program   ----------  

	//Initialize LCD module
	InitLCD(LS_BLINK|LS_ULINE);

	lcd_message("Starting prog...");

	// PWM 
	TCCR0A |= _BV(WGM01)|_BV(WGM00);
	TCCR0A |= _BV(COM0A1); // Rests at top
	TCCR0B|=_BV(CS01); // Prescale /8
	OCR0A = DUTY_CYCLE; // Sets the OCRA value 

	// Steppper motor timer
	TCCR3B |= _BV (CS30);  //  sets prescalar to DIV 1
   	OCR3A = CURRENT_DELAY * 0x03E8; // use this to adjust timer 3 countdown value (20ms rn)
   	TCNT3 = 0x0000;
	TIMSK3 |= 0x2; // use this to enable/disable COMPA interrupt
   	TCCR3B |= _BV(WGM32);
	TIFR3 |= _BV(OCF3A); // clear interrupt flag and start timer

	zero_tray(); // set initial tray position

	// Enable all interrupts

	sei();	// Note this sets the Global Enable for all interrupts

	set_belt(1); // start DC motor

	// init linked queue
	setup(&head,&tail);
	initLink(&new_link);
	new_link->e.item_type = DUMMY;
	new_link->e.item_min = 123;
	enqueue(&head,&tail,&new_link);
	unknown_item = head;

	state = WAITING_FOR_FIRST;

	// main program loop starts
	while(1)
	{
		switch (state)
		{
			case WAITING_FOR_FIRST:
				set_belt(1);

				material_t head_type;				
				ATOMIC_BLOCK(ATOMIC_FORCEON)
				{
					head_type = head->e.item_type;
				}
				
			//	if (head_type != DUMMY && head_type != UNKNOWN)
			//		state = MOVING_ITEM_TO_GATE;
			break;			

			case MOVING_ITEM_TO_GATE:;

				element head_item;
				ATOMIC_BLOCK(ATOMIC_FORCEON)
				{
					head_item = head->e;
				}
	
				const material_t future_item_type = head_item.item_type;
				const uint16_t item_min = head_item.item_min;

				int16_t target_position;
				switch(future_item_type)
				{
					case (WHITE):
						target_position = 100;
					break;
					case (BLACK):
						target_position = 0;
					break;
					case (STEEL):
						target_position = 50;
					break;
					case (ALUM):
						target_position = 150;
					break;
					default:
						target_position = stepper_pos;
					break;
				}

				future_steps = target_position - stepper_pos;
				if (future_steps > 100)
				{
					future_steps = - 200 + future_steps;
				}
				else if (future_steps < -100)
				{
					future_steps =  200 - abs(future_steps);
				}

				state = GATE_CHECK;


			//	set_belt(1);

			break;

			case GATE_CHECK:;

				int curr_future_steps;
				ATOMIC_BLOCK(ATOMIC_FORCEON)
				{
					curr_future_steps =  future_steps;
				}

				if (abs(curr_future_steps) > DROP_STEP)
				{
					set_belt(0);
					break;
				}
				else
				{
					set_belt(1);

					if (curr_future_steps == 0)
					{
						BUCKET_COUNT++;
						ATOMIC_BLOCK(ATOMIC_FORCEON)
						{
							dequeue(&head,&tail,&rtn_link);
						}

#ifdef LCD_DEBUG // print LCD debug output??
						LCDClear();
						switch(rtn_link->e.item_type) // should do a null check...
						{
							case (WHITE):
								LCDWriteString("WHITE");
							break;
							case (BLACK):
								LCDWriteString("BLACK");
							break;
							case (STEEL):
								LCDWriteString("STEEL");
							break;
							case (ALUM):
								LCDWriteString("ALUM");
							break;
							case (UNKNOWN):
								LCDWriteString("UNKNOWN");
							break;
							default:
								LCDWriteString("GARBAGE");
							break;
						}
						LCDWriteIntXY(0,1,rtn_link->e.item_min,4);
						LCDWriteIntXY(14,0,size(&head,&tail),2);
						LCDWriteIntXY(6,1,ADC_count,5);
						mTimer(20);
#endif

						free(rtn_link);
					
						material_t head_type;				
						ATOMIC_BLOCK(ATOMIC_FORCEON)
						{
							head_type = head->e.item_type;
						}
				
						if (head_type != DUMMY && head_type != UNKNOWN)
						{	
							// is real item at head
						 	if(BUCKET_COUNT == GATE_COUNT){
								set_belt(1);
								state = WAITING_FOR_FIRST;
							}
							else
								state = MOVING_ITEM_TO_GATE;
						
						
							//mTimer(80);
						
						}
						else
						{
							state = WAITING_FOR_FIRST;
						}
					}
				}
			break;
			default:
			break;
		}
		// get first element of queue


	}
/*
	PAUSE_STAGE:
		LCDClear();
		lcd_message("Paused!");	
		set_belt(0);
		while(is_paused);
		LCDClear();
		set_belt(1);
		STATE = POLLING;
		goto POLLING_STAGE;

	RAMP_STAGE:
		LCDClear();
		lcd_message("Rampdown!");	
		while(is_paused);
		LCDClear();
		mTimer(20);		
		STATE = POLLING;
		goto POLLING_STAGE;
*/
	return(0);

}

// the interrupt will be trigured if the ADC is done ========================


//-------------------- ADC --------------------
ISR(ADC_vect)
{
	uint16_t ADC_result = 0;
	uint8_t ADC_low = ADCL;
	ADC_result = ADCH;
	ADC_result <<= 8;
	ADC_result |= ADC_low;
	
	if(reflect_min > ADC_result)
		reflect_min = ADC_result;

	ADC_count++;

	if ((PIND & 0x01) > 0)
	{
		ADCSRA |= _BV(ADSC);
	}
	else

	{
//	 	if(unknown_item->e.item_type == DUMMY )
//			update_ready = 1;

		// classify unknown piece
		unknown_item->e.item_min = reflect_min;
		unknown_item->e.item_type = categorize(unknown_item->e.item_min);
		unknown_item = unknown_item->next;
	
	}
}

//-------------------- OR --------------------
// Resets the ADC Count and starts conversion

ISR(INT0_vect)
{
	reflect_min = 1023;
	ADC_count = 0;
	ADCSRA |= _BV(ADSC);
}

//-------------------- OI --------------------
// Creating linked list and adding it to que

ISR(INT1_vect)
{
	if(head == tail){
		unknown_item->e.item_type = DUMMY;

	}

	initLink(&new_link);
	new_link->e.item_type = UNKNOWN;
	

	enqueue(&head,&tail,&new_link);
}

//-------------------- EX --------------------
// Starts the Dequeuing process ( item at exit gate)
ISR(INT2_vect)
{
	if(BUCKET_COUNT == GATE_COUNT)
		state = MOVING_ITEM_TO_GATE;
	else
		state = GATE_CHECK;

	GATE_COUNT++;
}

//-------------------- PAUSE --------------------
// Starts or stops the belt

// Will need to also stop to pause the stepper motor as well.
 
ISR(INT6_vect)
{
// debounce

	if (is_paused)
	{
		is_paused = 0;	
	}
	else
	{
		is_paused = 1;
		state = PAUSE;
	}	
}

//-------------------- RAMP_DOWN --------------------
// Finishes what ever items have been added to the belt

ISR(INT7_vect)
{
// debounce?
	mTimer(20);
	state = RAMPDOWN;
}

//-------------------- TIMER 3 --------------------
// Moves one step in the stepper.

ISR(TIMER3_COMPA_vect)
{


	int j = current_stepper;
	
	//Clockwise movement
	if(future_steps < 0){

		j++;
		if(j==4)
			j=0;

		PORTA = STEPPER_ARRAY[j];

		stepper_pos--;
		future_steps++;

		if (stepper_pos < 0)
			stepper_pos = 199;



	}	// Counter-Clockwise movement
	if(future_steps > 0){
		
		j--;
		if(j==-1)
			j=3;

		PORTA = STEPPER_ARRAY[j];
			
			
		stepper_pos++;
		future_steps--;
		
		if (stepper_pos >= 200)
			stepper_pos = 0;
		
	}
	if( abs(future_steps) < NUM_DECEL_STEPS ){
		CURRENT_DELAY = CURRENT_DELAY + DECELERATION;
	}
	else if(CURRENT_DELAY > FULL_SPEED) {
		CURRENT_DELAY = CURRENT_DELAY - ACCELERATION;
	}
	

	current_stepper = j;

	if (future_steps == 0)
	{
		CURRENT_DELAY = FULL_SPEED + ( ACCELERATION * NUM_ACCEL_STEPS );
	}

 	OCR3A = CURRENT_DELAY * 0x03E8;
	TIFR3 |= _BV(OCF3A);
}

// If an unexpected interrupt occurs (interrupt is enabled and no handler is installed,
// which usually indicates a bug), then the default action is to reset the device by jumping 
// to the reset vector. You can override this by supplying a function named BADISR_vect which 
// should be defined with ISR() as such. (The name BADISR_vect is actually an alias for __vector_default.
// The latter must be used inside assembly code in case <avr/interrupt.h> is not included.
ISR(BADISR_vect)
{
    // user code here
}
