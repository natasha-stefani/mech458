// Status: A-syncronous stepper motor contol
// 				- Multiple loading
//				- Sorting correctly, taking shortest path
//				- Acceleration is added

// Create a watch variable on STATE. To do this right click on the variable STATE and then
// Add Watch 'STATE'. You can see how the variable changes as you click on PINDA0 or PINDA3. Note that the interrupt
// catches a rising edge. You modify this to suit your needs.

# define F_CPU 8000000UL // suppress compiler warnings

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
	DROPPING_ITEM,
	PAUSE_STAGE,
	RAMPDOWN,
	BADISR
} state_t;

// Global Variables

const uint16_t ACCEL_TABLE[] =
{
       14000,
       13680,
       13375,
       13084,
       12197,
       11423,
        9859,
        8671,
        7739,
        6988,
        6370,
        5852,
        5412,
        5033,
        4705,
        4585,
        4471,
        4438,
        4405,
        4373


/*
// Best profile so far
       14000,
       13680,
       13375,
       13084,
       12197,
       11423,
        9859,
        8671,
        7739,
        6988,
        6370,
        5852,
        5412,
        5033,
        4897,
        4767,
        4729,
        4692,
        4656
*/
};

const uint16_t * DECEL_TABLE = ACCEL_TABLE;

const uint8_t ACCEL_TABLE_SIZE = sizeof(ACCEL_TABLE) / sizeof(uint16_t);
const uint8_t DECEL_TABLE_SIZE = sizeof(DECEL_TABLE) / sizeof(uint16_t);
volatile int8_t accel_idx = 0;  // initializing index for acceleration

// State transition control
volatile state_t state;
volatile uint8_t BUCKET_COUNT;
volatile uint8_t GATE_COUNT;
volatile uint8_t PAUSE_flag;
volatile uint8_t PAUSE_belt;
volatile uint8_t RAMPDOWN_flag;
volatile uint8_t rampdown_time_reached;

// Item queue
link * head, * tail, * rtn_link, * unknown_item, * new_link;

// Stepper motor control
const unsigned char STEPPER_ARRAY[] = {0b110110,0b101110,0b101101,0b110101};
volatile int8_t stepper_table_pos;
volatile int16_t stepper_pos, initial_position, target_position, initial_future_steps;
//volatile int16_t future_steps = 0;
#define REVERSAL_DELAY 55000
#define REVERSAL_COUNTDOWN_MS 225
#define DROP_STEP 55
//#define ZEROING_OFFSET 12
#define ZEROING_OFFSET 0
volatile uint16_t CURRENT_DELAY = 14000;

// Item categorization
#define STEEL_BOUND 200
#define WHITE_BOUND 700
#define BLACK_BOUND 911

//Controlling bucket count
volatile material_t SORTING_type;
volatile uint8_t BLK_BUCKET_COUNT;
volatile uint8_t WHITE_BUCKET_COUNT;
volatile uint8_t STEEL_BUCKET_COUNT;
volatile uint8_t ALUM_BUCKET_COUNT;
volatile uint8_t UK_BUCKET_COUNT;

volatile uint8_t BLK_COUNT;
volatile uint8_t WHITE_COUNT;
volatile uint8_t STEEL_COUNT;
volatile uint8_t ALUM_COUNT;

volatile uint16_t ADC_count;
volatile uint16_t reflect_min;

// Belt control
#define SORTING_DUTY_CYCLE 63 //0x38 56 // Expressed as ratio of 0xff (i.e. 0x80 = 50% duty)

// Timer control
#define TIMER0_PRESCALE _BV(CS01) | _BV(CS00) // Prescale /64 -> PWM timer
#define TIMER1_PRESCALE _BV(CS12) // Prescale /256 -> Countdown timer
#define TIMER2_PRESCALE _BV(CS22) | _BV(CS21) | _BV(CS20) // Prescale 1024 -> Rampdown timer
#define TIMER3_PRESCALE _BV (CS31)  //  Prescale /8 -> Stepper timer
volatile uint8_t countdown_reached = 1;

// timer 1 countdown
// input value must be less than ~2100ms
void restart_countdown(uint16_t ms)
{
    /* Initialize Timer 1 to zero */
    countdown_reached = 0;

    OCR1A = ms * 31;

    // Countdown timer
    TCNT1 = 0x0000;
    TIFR1 |= _BV(OCF1A);
	TCCR1B |= TIMER1_PRESCALE;
}

// timer 2 countdown - hard-coded for 5 seconds
void start_rampdown_timer()
{
    /* Initialize Timer 1 to zero */
    rampdown_time_reached = 0;

    OCR2A = 255;

    // Countdown timer
    TCNT2 = 0x0000;
    TIFR2 |= _BV(OCF2A);
	TCCR2B |= TIMER2_PRESCALE;
}

//--------------------  set_belt  --------------------

inline void set_belt(char is_on)
{
	if (is_on)
		PORTD = 0b00010000;
	else
		PORTD = 0;
}

//--------------------  stepper_movement  --------------------

static inline void stepper_movement (direction_t dir)
{

    if (dir == STEPPER_CW)
    {
    	stepper_table_pos++;
    	if(stepper_table_pos==4)
    		stepper_table_pos=0;
    }
    else if (dir == STEPPER_CCW)
    {
		stepper_table_pos--;
		if(stepper_table_pos==-1)
			stepper_table_pos=3;
    }
	
    PORTA = STEPPER_ARRAY[stepper_table_pos];
	_delay_ms(20);
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
//--------------------  Adjust bucket  --------------------

inline void adjust_bucket()
{
	BUCKET_COUNT++;

		switch(SORTING_type)
        {
			case (WHITE):
				WHITE_BUCKET_COUNT ++;
			break;
			case (BLACK):
				BLK_BUCKET_COUNT ++;
			break;
			case (STEEL):
				STEEL_BUCKET_COUNT ++;
			break;
			case (ALUM):
				ALUM_BUCKET_COUNT ++;
			break;
			default:
				UK_BUCKET_COUNT ++;
			break;
				}

}

//--------------------  lcd_message  --------------------

static inline void lcd_message(const char * msg)
{
	LCDClear();
	LCDWriteString(msg);
	_delay_ms(20);
}

//--------------------   zero_tray   --------------------
void zero_tray()
{
	lcd_message("Calibrating tray..");

	while ((PIND & 0b1000) != 0)
		stepper_movement(STEPPER_CW);

    for (int i = 0; i < ZEROING_OFFSET; i++)
    {
        stepper_movement(STEPPER_CCW);
    }
	
    stepper_pos = 0;
	LCDClear();

	lcd_message("Waiting ");
}

//--------------------     pause    --------------------
void pause()
{
  //store state of the belt
    if(PIND)
        PAUSE_belt = 1;
    else
        PAUSE_belt = 0;

	set_belt(0);

    PAUSE_flag = 1;

 // Pause the stepper motor timer 
}

//--------------------   unpause    --------------------
void unpause()
{

// reset the acceleration index
    accel_idx = 0;
    CURRENT_DELAY = ACCEL_TABLE[accel_idx];

// unpasue the stepper motor timer


//restore the state of the belt
    if(PAUSE_belt != 0 )
        set_belt(1);
    else
    {
        set_belt(0);
        state = MOVING_ITEM_TO_GATE; 
     }

// change the state
    PAUSE_flag = 0;

}

//--------------------   rampdown   --------------------

void rampdown()
{

}

void stepper_move(uint16_t target_pos)
{
	// calculate which way you need to go
	// CCW -> Positive future steps
	// CW  -> Negative future steps

    uint16_t CURRENT_DELAY = 20000;
	int16_t future_steps = target_pos - stepper_pos;
	if (future_steps > 100)
		future_steps = - 200 + future_steps;
	else if (future_steps < -100)
		future_steps =  200 - abs(future_steps);

    int accel_idx = 0;
    while (future_steps != 0)
    {
        if (abs(future_steps) < DECEL_TABLE_SIZE)
        {
            accel_idx = 0;
            CURRENT_DELAY = DECEL_TABLE[abs(future_steps)];
        }
        else
        {
            CURRENT_DELAY = ACCEL_TABLE[accel_idx];
    		if (accel_idx < ACCEL_TABLE_SIZE-1)
                accel_idx++;
        }

    	if(future_steps < 0)
    	{
    		stepper_table_pos++;
    		if(stepper_table_pos==4)
    			stepper_table_pos=0;

    		PORTA = STEPPER_ARRAY[stepper_table_pos];

    		stepper_pos--;

    		if (stepper_pos < 0)
    			stepper_pos = 199;
            future_steps++;

    	}	// Counter-Clockwise movement
    	else if(future_steps > 0)
    	{
    		stepper_table_pos--;
    		if(stepper_table_pos==-1)
    			stepper_table_pos=3;

    		PORTA = STEPPER_ARRAY[stepper_table_pos];

    		stepper_pos++;

    		if (stepper_pos >= 200)
    			stepper_pos = 0;
            future_steps--;
    	}

        _delay_us(CURRENT_DELAY);
    }

    _delay_ms(200);
}

void set_stepper_target(int16_t target)
{

	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		target_position = target;

		initial_future_steps = target_position - stepper_pos;
		if (initial_future_steps > 100)
		{
			initial_future_steps = - 200 + initial_future_steps;
		}
		else if (initial_future_steps < -100)
		{
			initial_future_steps =  200 - abs(initial_future_steps);
		}

		initial_position = stepper_pos;
	}
}

void wait_for_stepper()
{
    while(1)
    {
        _delay_ms(1);
        uint16_t step_pos;
    	ATOMIC_BLOCK(ATOMIC_FORCEON)
    	{
            step_pos = stepper_pos;
    	}
        if (target_position == step_pos)
            break;
    }
    _delay_ms(500);
}
//############################## MAIN ##############################

int main(){

	cli();	// Disables all interrupts

    // Disable system clock prescaling - run at full 8MHz
    CLKPR=(1<<CLKPCE);
    CLKPR=0;

	// pin F1 RL
	// pin D0 OR
	// pin D2 EX
	// pin D3 HE
	DDRA = 0xFF;
	DDRB |= 0x80; // Sets OC0A pin to output (B7 pin)
	DDRC = 0xFF;	// just use as a display
	DDRD |= 0xf0;	// DC motor control out and
	DDRE = 0x00; // Button input, E6 and E7 output
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
    EIMSK |= (_BV(INT6)); // enable INT6
    EICRB |= (_BV(ISC61)); // falling edge interrupt

	// Ramp-down button
    EIMSK |= (_BV(INT7)); // enable INT7
	EICRB |= (_BV(ISC71)); // falling edge interrupt

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
	ADCSRA |= _BV(ADPS2) | _BV(ADPS1); // Enable prescale 64x (8MHz -> 125kHz)
	//ADCSRA |= _BV(ADPS2) | _BV(ADPS0); // Enable prescale 32x (8MHz -> 250kHz)
	ADCSRB |= _BV(ADHSM);
	ADMUX |= _BV(REFS0);
	ADMUX |= 0b1;

	//Initialize LCD module
	InitLCD(LS_BLINK|LS_ULINE);

	lcd_message("Starting prog...");

	// PWM
	TCCR0A |= _BV(WGM01)|_BV(WGM00);
	TCCR0A |= _BV(COM0A1); // Rests at top
	TCCR0B |= TIMER0_PRESCALE;
	OCR0A = SORTING_DUTY_CYCLE; // Sets the OCRA value

	// Countdown timer
    TCCR1B |= _BV(WGM12); // CTC mode
	TIMSK1 |= 0x2; // use this to enable/disable COMPA interrupt

    // Rampdown timer (2)
	TCCR2B |= _BV(WGM22);
	TIMSK2 |= 0x2; // use this to enable/disable COMPA interrupt

	// Steppper motor timer
   	OCR3A = 0x03E8; // use this to adjust timer 3 countdown value (1ms rn)
   	TCNT3 = 0x0000;
	TIMSK3 |= 0x2; // use this to enable/disable COMPA interrupt
   	TCCR3B |= _BV(WGM32);
	TIFR3 |= _BV(OCF3A); // clear interrupt flag and start timer
	TCCR3B |= TIMER3_PRESCALE;

	// Enable all interrupts
	sei();	// Note this sets the Global Enable for all interrupts

	// init linked queue
	setup(&head,&tail);
	initLink(&new_link);
	new_link->e.item_type = DUMMY;
	new_link->e.item_min = 123;
	enqueue(&head,&tail,&new_link);
	unknown_item = head;

	// set initial state
	state = WAITING_FOR_FIRST;

	zero_tray(); // set initial tray position

/*
    stepper_move(150);
    stepper_move(100);
    stepper_move(50);
    stepper_move(0);
    stepper_move(150);
    stepper_move(100);
    stepper_move(50);
    stepper_move(0);
    stepper_move(150);
    stepper_move(100);
    stepper_move(50);
    stepper_move(0);
    stepper_move(150);
    stepper_move(100);
    stepper_move(50);
    stepper_move(0);

    stepper_move(50);
    stepper_move(100);
    stepper_move(150);
    stepper_move(0);
    stepper_move(50);
    stepper_move(100);
    stepper_move(150);
    stepper_move(0);
    stepper_move(50);
    stepper_move(100);
    stepper_move(150);
    stepper_move(0);
    stepper_move(50);
    stepper_move(100);
    stepper_move(150);
    stepper_move(0);


    while(1);

    _delay_ms(1000);
    set_stepper_target(150);
    wait_for_stepper();
    set_stepper_target(100);
    wait_for_stepper();
    set_stepper_target(50);
    wait_for_stepper();
    set_stepper_target(0);
    wait_for_stepper();
    set_stepper_target(150);
    wait_for_stepper();
    set_stepper_target(100);
    wait_for_stepper();
    set_stepper_target(50);
    wait_for_stepper();
    set_stepper_target(0);
    wait_for_stepper();
    set_stepper_target(150);
    wait_for_stepper();
    set_stepper_target(100);
    wait_for_stepper();
    set_stepper_target(50);
    wait_for_stepper();
    set_stepper_target(0);
    wait_for_stepper();
    set_stepper_target(150);
    wait_for_stepper();
    set_stepper_target(100);
    wait_for_stepper();
    set_stepper_target(50);
    wait_for_stepper();
    set_stepper_target(0);
    wait_for_stepper();
    //
    _delay_ms(1000);
    set_stepper_target(50);
    wait_for_stepper();
    set_stepper_target(100);
    wait_for_stepper();
    set_stepper_target(150);
    wait_for_stepper();
    set_stepper_target(0);
    wait_for_stepper();
    set_stepper_target(50);
    wait_for_stepper();
    set_stepper_target(100);
    wait_for_stepper();
    set_stepper_target(150);
    wait_for_stepper();
    set_stepper_target(0);
    wait_for_stepper();
    set_stepper_target(50);
    wait_for_stepper();
    set_stepper_target(100);
    wait_for_stepper();
    set_stepper_target(150);
    wait_for_stepper();
    set_stepper_target(0);
    wait_for_stepper();
    set_stepper_target(50);
    wait_for_stepper();
    set_stepper_target(100);
    wait_for_stepper();
    set_stepper_target(150);
    wait_for_stepper();
    set_stepper_target(0);
    wait_for_stepper();

    while(1);
*/

	set_belt(1); // start DC motor

	// main program loop starts
	while(1)
	{
        if(RAMPDOWN_flag !=0 && rampdown_time_reached)
        {
           	EICRA &= ~_BV(ISC01) | ~_BV(ISC00); // disable OR interrupt

            if (head == tail)
                state = RAMPDOWN;
        }

        _delay_ms(1);
		switch(state)
		{
			case WAITING_FOR_FIRST:
				set_belt(1);


			break;

			case MOVING_ITEM_TO_GATE:;

				element head_item;
				ATOMIC_BLOCK(ATOMIC_FORCEON)
				{
					head_item = head->e;
				}

				const material_t future_item_type = head_item.item_type;
               
                SORTING_type = future_item_type;

				int16_t future_target_position;
				switch(future_item_type)
				{
					case (WHITE):
						future_target_position = 100;
					break;
					case (BLACK):
						future_target_position = 0;
					break;
					case (STEEL):
						future_target_position = 50;
					break;
					case (ALUM):
						future_target_position = 150;
					break;
					default:
						future_target_position = stepper_pos;
					break;
				}

				set_stepper_target(future_target_position);

                LCDClear();


				state = GATE_CHECK;
			break;

			case GATE_CHECK:;
				int16_t remaining_steps;
				ATOMIC_BLOCK(ATOMIC_FORCEON)
				{
					remaining_steps = target_position - stepper_pos;
					if (remaining_steps > 100)
						remaining_steps = - 200 + remaining_steps;
					else if (remaining_steps < -100)
						remaining_steps =  200 - abs(remaining_steps);
				}

				if (abs(remaining_steps) > DROP_STEP)
				{
					set_belt(0);
					break;
				}
                else if (CURRENT_DELAY > ACCEL_TABLE[5] && abs(remaining_steps) != 0)
                {
                    set_belt(0);
                    break;
                }
				else
				{
                    if (BUCKET_COUNT + 1 == GATE_COUNT)
        			{	

                        set_belt(1);
                        if (remaining_steps == 0)
                        {
                            _delay_ms(100);
                        }
                    }

					if (remaining_steps == 0)
					{
                        adjust_bucket();
	
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
						_delay_ms(20);
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
						}
						else
						{
							state = WAITING_FOR_FIRST;
						}
					}
				}
			break;
			case BADISR:

    			set_belt(0);
    			lcd_message("Congrats, u suck");
    			cli();

    			while (1);

    		break;

            case PAUSE_STAGE:;
        		LCDClear();
        		lcd_message("Paused!");
                _delay_ms(1000);
        		//set_belt(0);
                uint8_t blk_belt = BLK_COUNT - BLK_BUCKET_COUNT;
                uint8_t white_belt = WHITE_COUNT - WHITE_BUCKET_COUNT;
                uint8_t steel_belt = STEEL_COUNT - STEEL_BUCKET_COUNT;
                uint8_t alum_belt = ALUM_COUNT - ALUM_BUCKET_COUNT;
                
                //what is in the bucket
                LCDClear();
                LCDWriteString(" B");
                LCDWriteIntXY(2,0,BLK_BUCKET_COUNT,2);
                LCDWriteString(" W");
                LCDWriteIntXY(6,0,WHITE_BUCKET_COUNT,2);
                LCDWriteString(" S");
                LCDWriteIntXY(10,0,STEEL_BUCKET_COUNT,2);
                LCDWriteString(" A");
                LCDWriteIntXY(14,0,ALUM_BUCKET_COUNT,2);

                // what is on the belt
                LCDWriteIntXY(2,1,blk_belt,2);
                LCDWriteIntXY(6,1,white_belt,2);
                LCDWriteIntXY(10,1,steel_belt,2);
                LCDWriteIntXY(14,1,alum_belt,2);

				_delay_ms(500);

        		while(PAUSE_flag);

        		LCDClear();

            	state = WAITING_FOR_FIRST;

            break;
            case RAMPDOWN:;
                set_belt(0);
                cli();

                LCDClear();
            	LCDWriteString("Rampdown!");
                LCDWriteStringXY(0,1," B");
                LCDWriteIntXY(2,1,BLK_BUCKET_COUNT,2);
                LCDWriteString(" W");
                LCDWriteIntXY(6,1,WHITE_BUCKET_COUNT,2);
                LCDWriteString(" S");
                LCDWriteIntXY(10,1,STEEL_BUCKET_COUNT,2);
                LCDWriteString(" A");
                LCDWriteIntXY(14,1,ALUM_BUCKET_COUNT,2);

                _delay_ms(1000);

                while(1);

            break;
			default:
			break;



		}



    	

	}




/*
	RAMP_STAGE:
		LCDClear();
		lcd_message("Rampdown!");
		while(is_paused);
		LCDClear();
		_delay_ms(20);
		STATE = POLLING;
		goto POLLING_STAGE;
*/
	return(0);

}

// the interrupt will be trigured if the ADC is done ========================


//-------------------- ADC --------------------
ISR(ADC_vect)
{
	const uint8_t ADC_low = ADCL;
	uint16_t ADC_result = 0;
	ADC_result = ADCH;
	ADC_result <<= 8;
	ADC_result |= ADC_low;

	if(reflect_min > ADC_result)
		reflect_min = ADC_result;

	ADC_count++;

	if ((PIND & 0x01) > 0)
		ADCSRA |= _BV(ADSC);
	else
	{
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
 
	if(head == tail)
		unknown_item->e.item_type = DUMMY;

	initLink(&new_link);
	new_link->e.item_type = UNKNOWN;

	enqueue(&head,&tail,&new_link);
}

//-------------------- EX --------------------
// Starts the Dequeuing process ( item at exit gate)
ISR(INT2_vect)
{
	set_belt(0);

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
	if (PAUSE_flag != 0)
        unpause();

	else
        pause();
  
    
    state = PAUSE_STAGE;
}

//-------------------- RAMP_DOWN --------------------
// Finishes what ever items have been added to the belt

ISR(INT7_vect)
{
    RAMPDOWN_flag = 1;
    start_rampdown_timer();    
}

//-------------------- TIMER 3 --------------------
// Moves one step in the stepper.

ISR(TIMER3_COMPA_vect)
{
	static direction_t curr_direction = STOP;
	static direction_t prev_direction = STOP;
	// calculate which way you need to go
	// CCW -> Positive future steps
	// CW  -> Negative future steps

    // checking for the timer to be complete between switching directions

    static uint8_t stop_switch_flag = 0;
    if(stop_switch_flag)
    {
        if (countdown_reached)
        {
            countdown_reached = 0;
            stop_switch_flag = 0;
        }
        else
            return;
    }
    
    // Determining future steps
	int16_t future_steps = target_position - stepper_pos;
	if (future_steps > 100)
		future_steps = - 200 + future_steps;
	else if (future_steps < -100)
		future_steps =  200 - abs(future_steps);

    // Determining direction based on future steps
	if (future_steps < 0)
		curr_direction = STEPPER_CW;
	else if (future_steps > 0)
		curr_direction = STEPPER_CCW;
	else // stopping
    {
		curr_direction = STOP;
        //stop_switch_flag = 1;
        //restart_countdown(REVERSAL_COUNTDOWN_MS);
    }

   

    if (curr_direction == prev_direction) // cruising or stopped
	{
        if (curr_direction == STOP)
            goto ISR_TIMER_RESET;

        if (abs(future_steps) < DECEL_TABLE_SIZE)
        {
            CURRENT_DELAY = DECEL_TABLE[abs(future_steps)];
        }
        else
        {
    		if (accel_idx < ACCEL_TABLE_SIZE - 1)
                accel_idx++;
            if (accel_idx < 0)
                accel_idx = 0;
            CURRENT_DELAY = ACCEL_TABLE[accel_idx];
        }
	}
	else if (prev_direction == STOP) // starting
	{
    	accel_idx = 0;

    	CURRENT_DELAY = REVERSAL_DELAY;
        goto ISR_TIMER_RESET;
	}
	else if (curr_direction == STOP) // stopping
	{
		accel_idx = 0;
		CURRENT_DELAY = ACCEL_TABLE[0];
        goto ISR_TIMER_RESET;
	}
    else // switching directions
    {
        accel_idx = -1;
        stop_switch_flag = 1;
        restart_countdown(REVERSAL_COUNTDOWN_MS);
        CURRENT_DELAY = REVERSAL_DELAY;
        goto ISR_TIMER_RESET;
    }

	if(future_steps < 0)
	{
		stepper_table_pos++;
		if(stepper_table_pos==4)
			stepper_table_pos=0;

		PORTA = STEPPER_ARRAY[stepper_table_pos];

		stepper_pos--;

		if (stepper_pos < 0)
			stepper_pos = 199;

	}	// Counter-Clockwise movement
	else if(future_steps > 0)
	{
		stepper_table_pos--;
		if(stepper_table_pos==-1)
			stepper_table_pos=3;

		PORTA = STEPPER_ARRAY[stepper_table_pos];

		stepper_pos++;

		if (stepper_pos >= 200)
			stepper_pos = 0;
	}

ISR_TIMER_RESET:
	prev_direction = curr_direction;
	TCNT3 = 0;
    OCR3A = CURRENT_DELAY;
}

//-------------------- TIMER 1 --------------------
// Async countdown timer
ISR(TIMER1_COMPA_vect)
{
    countdown_reached = 1;
    TCCR1B &= ~(TIMER1_PRESCALE);  //  disable timer
}

//-------------------- TIMER 2 --------------------
// Async countdown timer
ISR(TIMER2_COMPA_vect)
{
    static uint16_t ramp_timer_count = 0;

    ramp_timer_count++;
    if (ramp_timer_count == 60)
       	EICRA &= ~_BV(ISC11); // disable OI interrupt
    if (ramp_timer_count < 75)
        return;

    rampdown_time_reached = 1;
    TCCR2B &= ~(TIMER2_PRESCALE);  //  disable timer
}

// If an unexpected interrupt occurs (interrupt is enabled and no handler is installed,
// which usually indicates a bug), then the default action is to reset the device by jumping
// to the reset vector. You can override this by supplying a function named BADISR_vect which
// should be defined with ISR() as such. (The name BADISR_vect is actually an alias for __vector_default.
// The latter must be used inside assembly code in case <avr/interrupt.h> is not included.
ISR(BADISR_vect)
{
    // user code here
	state = BADISR;
}
