/* Mech 458 Project

Status: Final demo candidate

Andres Martinez
Natasha Stefani
Ao Li

*/

# define F_CPU 8000000UL // suppress compiler warnings

#include <stdlib.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <avr/io.h>
#include <util/delay.h>

#include "lcd.h"
#include "linked_queue.h"
#include "defs.h"

//#define LCD_DEBUG // Have LCD show most recent item classification

// State machine values
typedef enum
{
    WAITING_FOR_FIRST,
    MOVING_ITEM_TO_GATE,
    GATE_CHECK,
    PAUSE_STAGE,
    RAMPDOWN,
    BADISR,
    BADITEM
} state_t;

// Global Variables

// Stepper motor acceleration profile
const uint16_t ACCEL_TABLE[] =
{
       14000,
       13680,
       13375,
       13084,
       12197,
       11423,
       9699,
       8428,
       7451,
       6677,
       6048,
       5528,
       5363,
       5208,
       5163,
       5119,
       5076,
       5033,
       4992,
       4950,
       4910
};
// Stepper motor deceleration profile
const uint16_t * DECEL_TABLE = ACCEL_TABLE;

// Calculate the size of the stepper motor acceleration table
const uint8_t ACCEL_TABLE_SIZE = sizeof(ACCEL_TABLE) / sizeof(uint16_t);
const uint8_t DECEL_TABLE_SIZE = sizeof(DECEL_TABLE) / sizeof(uint16_t);
volatile int8_t accel_idx;  // initializing index for acceleration

// State transition control
volatile state_t state; // State machine tracking
volatile uint8_t bucket_count; // Number of dropped items
volatile uint8_t gate_count; // Number of items that have passed EX
volatile uint8_t pause_flag; // Pause button has been pressed
volatile uint8_t rampdown_flag; // Rampdown button has been pressed
volatile uint8_t rampdown_time_reached; // Rampdown timer has counted down
#define DEBOUNCE_DELAY_MS 50 // Pause and rampdown button debounce delay

// Item queue
link * head, * tail, * unknown_item; // Item queue tracking

// Stepper motor control
const unsigned char STEPPER_ARRAY[] = {0b110110,0b101110,0b101101,0b110101};
volatile int8_t stepper_table_pos; // Stepper motor table increment
volatile uint8_t stepper_pos, target_position; // Current and target stepper position
// Delay on stepper motor reversal
#define REVERSAL_DELAY 14500 
#define REVERSAL_COUNTDOWN_MS 60
// Metal and plastic drop timing parameters
#define METAL_DROP_STEP 55
#define PLASTIC_DROP_STEP 58
#define METAL_IDX 2
#define PLASTIC_IDX 0
#define ZEROING_OFFSET 9
// Current stepper motor delay
volatile uint16_t CURRENT_DELAY = 14000;

// ADC categorization value boundaries
#define STEEL_BOUND 350
#define WHITE_BOUND 800
#define BLACK_BOUND 969

// Item drop and categorization tracking
material_t sorting_type; // Currently item type
// Bucket counts for different item types
uint8_t blk_bucket_count;
uint8_t white_bucket_count;
uint8_t steel_bucket_count;
uint8_t alum_bucket_count;
uint8_t uk_bucket_count; // Unclassified item count
// Categorization counts for different item types
volatile uint8_t blk_count;
volatile uint8_t white_count;
volatile uint8_t steel_count;
volatile uint8_t alum_count;

#ifdef LCD_DEBUG
// Number of ADC measurements for current item categorization
volatile uint16_t adc_count;
#endif
// Minimum ADC measurement for current item categorization
volatile uint16_t reflect_min;

// Belt speed control
#define SORTING_DUTY_CYCLE 80 // (i.e. 0x80 = 50% duty)

// Timer prescale values
#define TIMER0_PRESCALE _BV(CS01) | _BV(CS00) // Prescale /64 -> PWM timer
#define TIMER1_PRESCALE _BV(CS12) // Prescale /256 -> Countdown timer
#define TIMER2_PRESCALE _BV(CS22) | _BV(CS21) | _BV(CS20) // Prescale 1024 -> Rampdown timer
#define TIMER3_PRESCALE _BV (CS31)  //  Prescale /8 -> Stepper timer
// Countdown timer flag
volatile uint8_t countdown_reached = 1;

//--------------------  restart_countdown --------------------
// Starts asynchronous millisecond countdown timer
// Input: Timer countdown in milliseconds
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

//--------------------  start_rampdown_timer --------------------
// Starts asynchronous 5-second countdown timer
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
// Starts or stops the conveyor belt
// Input: Boolean value (1 or 0) corresponding to whether the belt is on or off
inline void set_belt(char is_on)
{
    if (is_on)
        PORTD = 0b00010000;
    else
        PORTD = 0;
}

//--------------------  stepper_movement  --------------------
// Moves the stepper motor by one increment in the specified direction
// Input: Movement direction
static inline void stepper_movement (direction_t dir)
{

    if (dir == STEPPER_CW)
    {
        if(stepper_table_pos==3)
            stepper_table_pos=0;
        else
            ++stepper_table_pos;
    }
    else if (dir == STEPPER_CCW)
    {
        if(stepper_table_pos==0)
            stepper_table_pos=3;
        else
            --stepper_table_pos;
    }
    
    PORTA = STEPPER_ARRAY[stepper_table_pos];
    _delay_ms(20);
}

//--------------------  categorize  --------------------
// Converts the raw ADC minimum to an item classification
// Input: Minimum ADC reflectivity
// Output: Item category
inline material_t categorize(uint16_t reflect_min)
{
    if (reflect_min >= BLACK_BOUND)
    {
        ++blk_count;
        return BLACK;
    }
    else if (reflect_min < BLACK_BOUND && reflect_min >= WHITE_BOUND)
    {
        ++white_count;
        return WHITE;
    }
    else if (reflect_min < WHITE_BOUND && reflect_min >= STEEL_BOUND)
    {
        ++steel_count;
        return STEEL;
    }
    else
    {
        ++alum_count;
        return ALUM;
    }
}

//--------------------  Adjust bucket  --------------------
// Updates the dropped item counts based on the item at the head of the queue
inline void adjust_bucket()
{
    ++bucket_count;

    switch(sorting_type)
    {
        case (WHITE):
            ++white_bucket_count;
        break;
        case (BLACK):
            ++blk_bucket_count;
        break;
        case (STEEL):
            ++steel_bucket_count;
        break;
        case (ALUM):
            ++alum_bucket_count;
        break;
        default:
            ++uk_bucket_count;
        break;
    }
}

//--------------------  lcd_message  --------------------
// Clears LCD screen and outputs a single line string
// Input: char buffer with the string message
static inline void lcd_message(const char * msg)
{
    LCDClear();
    LCDWriteString(msg);
    _delay_ms(20);
}

//--------------------   zero_tray   --------------------
// Calibrates the stepper motor position based on the hall effect sensor
// and shifts the motor zero position to a hardcoded offset
void zero_tray()
{
    lcd_message("Calibrating tray..");

    while ((PIND & 0b1000) != 0)
    stepper_movement(STEPPER_CW);

    for (int i = 0; i < ZEROING_OFFSET; i++)
        stepper_movement(STEPPER_CCW);
    
    stepper_pos = 0;
    LCDClear();
    
    lcd_message("Waiting..");
}

//--------------------   Set_stepper_target   --------------------
// Sets the stepper motor to a new target position
// Input: Target position (0 to 150)
inline void set_stepper_target(uint8_t target)
{
    target_position = target;
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
    EICRB |= (_BV(ISC61) | _BV(ISC60)); // rising edge interrupt

    // Ramp-down button
    EIMSK |= (_BV(INT7)); // enable INT7
    EICRB |= (_BV(ISC71)); // falling edge interrupt

    // config ADC =========================================================
    // the ADC input (analog input is set to be ADC1 / PORTF1
    ADCSRA |= _BV(ADEN); // enable ADC
    ADCSRA |= _BV(ADIE); // enable interrupt of ADC
    ADCSRA |= _BV(ADPS2) | _BV(ADPS1); // Enable prescale 64x (8MHz -> 125kHz)
    //ADCSRA |= _BV(ADPS2) | _BV(ADPS0); // Enable prescale 32x (8MHz -> 250kHz)
    ADCSRB |= 0x80; // ADHSM flag
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

    // Initialize item queue
    setup(&head,&tail);
    link * new_link;
    initLink(&new_link);
    new_link->e.item_type = DUMMY;
    new_link->e.item_min = 123;
    enqueue(&head,&tail,&new_link);
    unknown_item = head;

    // Set initial state
    state = WAITING_FOR_FIRST;

    zero_tray(); // set initial tray position
    set_belt(1); // start DC motor
	
	// Current item drop step and target drop speed
    uint8_t drop_step = 0, drop_idx = 0;

	// Main program loop
    while(1)
    {
		// Check if rampdown has been triggered and rampdown timer has elapsed
        if(rampdown_flag !=0 && rampdown_time_reached)
        {
            EICRA &= ~_BV(ISC01) | ~_BV(ISC00); // disable OR interrupt

            if (head == tail) // If no items left in queue, then proceed to terminal rampdown state
                state = RAMPDOWN;
        }

        _delay_ms(1);
	    // State machine
        switch(state)
        {
			// System is currently waiting for a classified item to be placed on the queue
            case WAITING_FOR_FIRST:
            set_belt(1);
            break;

			// The linked queue head has been updated and the system is re-targeting the
			// stepper motor for the new item
            case MOVING_ITEM_TO_GATE:;

            material_t future_item_type;
            ATOMIC_BLOCK(ATOMIC_FORCEON)
            {
				// Get new item type from the head of the queue
                future_item_type = head->e.item_type;
            }
            
            sorting_type = future_item_type;

            uint8_t future_target_position = stepper_pos;
            state = GATE_CHECK;
			// Adjust the stepper motor posisition based on the item type
			// and reset the drop step and drop index
            switch(future_item_type)
            {
                case (WHITE):
                    future_target_position = 100;
                    drop_step = PLASTIC_DROP_STEP;
                    drop_idx = PLASTIC_IDX;
                break;
                case (BLACK):
                    future_target_position = 0;
                    drop_step = PLASTIC_DROP_STEP;
                    drop_idx = PLASTIC_IDX;
                break;
                case (STEEL):
                    future_target_position = 50;
                    drop_step = METAL_DROP_STEP;
                    drop_idx = METAL_IDX;
                break;
                case (ALUM):
                    future_target_position = 150;
                    drop_step = METAL_DROP_STEP;
                    drop_idx = METAL_IDX;
                break;              
				// Error states to denote that head item is unclassified or has nonsense value
                case (UNKNOWN):
                    LCDClear();
                    LCDWriteString("UNKNOWN");
                    state = BADITEM;
                break;
                case (DUMMY):
                    LCDClear();
                    LCDWriteString("DUMMY");
                    state = BADITEM;
                break;
                default:
                    LCDClear();
                    LCDWriteString("GARBAGE");
                    state = BADITEM;
                break;
            }

			// Set new stepper motor position
            set_stepper_target(future_target_position);
            break;

            case GATE_CHECK:;
			
            // Calculate the remaining number of steps from the current stepper position
			// to the target position
			int16_t remaining_steps;
            ATOMIC_BLOCK(ATOMIC_FORCEON)
            {
                remaining_steps = target_position - stepper_pos;
            }
            if (remaining_steps > 100)
                remaining_steps = - 200 + remaining_steps;
            else if (remaining_steps < -100)
                remaining_steps =  200 - abs(remaining_steps);

			// If the stepper position is not positioned within the drop window, then stop the belt
            if (abs(remaining_steps) > drop_step)
            {
                set_belt(0);
                break;
            }
			// Else if the stepper is within the drop window, but not moving fast enough for a centered
			// drop, then stop the belt
            else if (CURRENT_DELAY > ACCEL_TABLE[drop_idx] && abs(remaining_steps) != 0)
            {
                set_belt(0);
                break;
            }
            else
            {
				// If an item has passed the gate and is ready to be dropped..
                if (bucket_count + 1 == gate_count)
                {
                    set_belt(1);
					// Give the item time to fall
                    if (remaining_steps == 0)
                    {
                        _delay_ms(150);
                    }
                }

				// If item has dropped
                if (remaining_steps == 0)
                {
					// Update the bucket count
                    adjust_bucket();
                    
                    link * rtn_link;
                    ATOMIC_BLOCK(ATOMIC_FORCEON)
                    {
						// Take the item off the queue
                        dequeue(&head,&tail,&rtn_link);
                    }

                    #ifdef LCD_DEBUG 
                    LCDClear();
					// Print the recently dropped item type to the LCD
                    switch(rtn_link->e.item_type)
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
					// Print the dropped item reflectivity
                    LCDWriteIntXY(0,1,rtn_link->e.item_min,4);
					// Print the queue size after dropping
                    LCDWriteIntXY(14,0,size(head),2);
					// Print the ADC measurement count for the dropped item
                    LCDWriteIntXY(6,1,adc_count,5);
                    _delay_ms(20);
                    #endif

					// Free the memory allocated for the dropped item
                    free(rtn_link);

					// Determine the next state based on the item type and whether
					// there is another piece already past the gate
                    material_t head_type;
                    ATOMIC_BLOCK(ATOMIC_FORCEON)
                    {
                        head_type = head->e.item_type;
                    }
                    // If a classified item is at the head of the queue...
                    if (head_type == ALUM || head_type == STEEL || head_type == WHITE || head_type == BLACK)
                    {
						// ... and there is not another item past the gate
                        if(bucket_count == gate_count)
						{
                            set_belt(1);
                            state = WAITING_FOR_FIRST;
                        }
						// ... and there is another item past the gate
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
			// If bad ISR state triggered, then stop the system and display error message
            case BADISR:

            set_belt(0);
            lcd_message("--Bad ISR--");
            cli();
            while (1);

            break;

			// If bad item classification, then stop the system and display error message
            case BADITEM:

            set_belt(0);
            LCDWriteStringXY(0,1,"--Bad item--");
            
            break;

			// If system has been paused, then display paused message, 
			// bucket counts, and classification counts
            case PAUSE_STAGE:;
            LCDClear();
            lcd_message("Paused!");
            _delay_ms(1000);

            uint8_t blk_belt = blk_count - blk_bucket_count;
            uint8_t white_belt = white_count - white_bucket_count;
            uint8_t steel_belt = steel_count - steel_bucket_count;
            uint8_t alum_belt = alum_count - alum_bucket_count;
            
            //what is in the bucket
            LCDClear();
            LCDWriteString(" B");
            LCDWriteIntXY(2,0,blk_bucket_count,2);
            LCDWriteString(" W");
            LCDWriteIntXY(6,0,white_bucket_count,2);
            LCDWriteString(" S");
            LCDWriteIntXY(10,0,steel_bucket_count,2);
            LCDWriteString(" A");
            LCDWriteIntXY(14,0,alum_bucket_count,2);

            // what is on the belt
            LCDWriteIntXY(1,1,blk_belt,3);
            LCDWriteIntXY(5,1,white_belt,3);
            LCDWriteIntXY(9,1,steel_belt,3);
            LCDWriteIntXY(13,1,alum_belt,3);

            _delay_ms(500);
			// Spin the CPU until the pause flag has been unset
            while(pause_flag);

            LCDClear();

            state = WAITING_FOR_FIRST;
            break;

			// If rampdown has been triggered, the timer has elapsed, 
			// and no more items are in the queue...
            case RAMPDOWN:;
			// Stop the system
            set_belt(0);
            cli();

			// Display final bucket counts
            LCDClear();
            LCDWriteString("Rampdown!");
            LCDWriteStringXY(0,1," B");
            LCDWriteIntXY(2,1,blk_bucket_count,2);
            LCDWriteString(" W");
            LCDWriteIntXY(6,1,white_bucket_count,2);
            LCDWriteString(" S");
            LCDWriteIntXY(10,1,steel_bucket_count,2);
            LCDWriteString(" A");
            LCDWriteIntXY(14,1,alum_bucket_count,2);

            _delay_ms(1000);

            while(1);

            break;
            default:
            break;
        }
    }
    return(0);
}

//-------------------- ADC --------------------
// Collects ADC conversion result
ISR(ADC_vect)
{
	// Find the new minimum
    if(reflect_min > ADC)
        reflect_min = ADC;

    #ifdef LCD_DEBUG
    ++adc_count;
    #endif

	// If item is still in front of the ADC, then keep taking measurements
    if ((PIND & 0x01) > 0)
        ADCSRA |= _BV(ADSC);
    // Else classify the unknown piece
    else
    {
        unknown_item->e.item_min = reflect_min;
        unknown_item->e.item_type = categorize(unknown_item->e.item_min);
        unknown_item = unknown_item->next;
    }
}

//-------------------- OR --------------------
// Resets the ADC Count and starts conversion
ISR(INT0_vect)
{
	// reset reflectivity minimum for next item
    reflect_min = 1023;

    #ifdef LCD_DEBUG
	// reset measurement count
    adc_count = 0;
    #endif
	// start first measurement
    ADCSRA |= _BV(ADSC);
}

//-------------------- OI --------------------
// Creating linked list and adding it to queue
ISR(INT1_vect)
{
    
    if(head == tail)
        unknown_item->e.item_type = DUMMY;

    link * new_link;
    initLink(&new_link);
    new_link->e.item_type = UNKNOWN;

    enqueue(&head,&tail,&new_link);
}

//-------------------- EX --------------------
// Starts the Dequeuing process ( item at exit gate)
ISR(INT2_vect)
{
    set_belt(0);

    if(bucket_count == gate_count)
        state = MOVING_ITEM_TO_GATE;
    else
        state = GATE_CHECK;

    ++gate_count;
}

//-------------------- PAUSE --------------------
// Starts or stops the belt
ISR(INT6_vect)
{
    static state_t old_state;
    static uint8_t PAUSE_belt;
    
    if (pause_flag == 0)
    {
        set_belt(0);
        _delay_ms(DEBOUNCE_DELAY_MS);
        pause_flag = 1;       
        old_state = state;
        state = PAUSE_STAGE;
        
        //store state of the belt
        if(PIND)
            PAUSE_belt = 1;
        else
            PAUSE_belt = 0;
    }        
    else
    {
        // reset the acceleration index
        accel_idx = 0;
        CURRENT_DELAY = ACCEL_TABLE[accel_idx];

        _delay_ms(DEBOUNCE_DELAY_MS);
        //restore the state of the belt
        if(PAUSE_belt != 0 )
            set_belt(1);
        else
            set_belt(0);
            
        // change the state
        pause_flag = 0;     
        state = old_state;   
    } 
}

//-------------------- RAMP_DOWN --------------------
// Finishes what ever items have been added to the belt
ISR(INT7_vect)
{
    //debounce
    _delay_ms(DEBOUNCE_DELAY_MS);
    
    rampdown_flag = 1;
    start_rampdown_timer();
}

//-------------------- TIMER 3 --------------------
// Asynchronous stepper motor control
// Calculates time delay based on acceleration profile and 
// moves the stepper by one increment
ISR(TIMER3_COMPA_vect)
{
    static direction_t curr_direction = STOP; // current stepper travel direction
    static direction_t prev_direction = STOP; // previous stepper travel direction
    // calculate which way you need to go
    // CCW -> Positive future steps
    // CW  -> Negative future steps

	// Timer used to ensure that stepper has sufficient delay before changing direction
    static uint8_t stop_switch_flag = 0;
    if(stop_switch_flag)
    {
		// Check that countdown has finished before trying to restart stepper
        if (countdown_reached)
        {
			// reset flags
            countdown_reached = 0;
            stop_switch_flag = 0;
        }
        else
            return;
    }

    // Determining number of future steps needed to reach destination
    int16_t future_steps = target_position - stepper_pos;
    if (future_steps > 100)
        future_steps = - 200 + future_steps;
    else if (future_steps < -100)
        future_steps =  200 - abs(future_steps);
    
	// If making a 180 degree movement, then keep the stepper moving in
	// the same direction to avoid an unnecessary stop-and-reverse delay
    if (future_steps == 100 && prev_direction == STEPPER_CW)
        future_steps = -100;
    else if (future_steps == -100 && prev_direction == STEPPER_CCW)
        future_steps = 100;

    // Determining direction based on future steps
    if (future_steps < 0)
    {
        curr_direction = STEPPER_CW;
    }        
    else if (future_steps > 0)
    {
        curr_direction = STEPPER_CCW;
    }        
    else // stopping
    {
        curr_direction = STOP;
    }

    if (curr_direction == prev_direction) // cruising or stopped
    {
		// If stepper still stopped, then do nothing
        if (curr_direction == STOP)
            goto ISR_TIMER_RESET;
		// If nearing end of movement, then decelerate
        if (abs(future_steps) < DECEL_TABLE_SIZE)
        {
            if (CURRENT_DELAY < DECEL_TABLE[abs(future_steps)])
                CURRENT_DELAY = DECEL_TABLE[abs(future_steps)];
        }
		// Else adjust velocity based on acceleration profile position
        else
        {
            if (accel_idx < ACCEL_TABLE_SIZE - 1)
                ++accel_idx;
            if (accel_idx < 0)
                accel_idx = 0;
            CURRENT_DELAY = ACCEL_TABLE[accel_idx];
        }
    }
    else if (prev_direction == STOP) // starting
    {
		// If starting, then set velocity to the start of the profile
        accel_idx = 0;

        CURRENT_DELAY = REVERSAL_DELAY;
        goto ISR_TIMER_RESET;
    }
    else if (curr_direction == STOP) // stopping
    {
		// If stopping, then reset acceleration tracking
        accel_idx = 0;
        CURRENT_DELAY = ACCEL_TABLE[0];
        goto ISR_TIMER_RESET;
    }
    else // switching directions
    {
		// If switching direction, then start reversal delay countdown
        accel_idx = -1;
        stop_switch_flag = 1;
        restart_countdown(REVERSAL_COUNTDOWN_MS);
        CURRENT_DELAY = REVERSAL_DELAY;
        goto ISR_TIMER_RESET;
    }

	// Clockwise stepper movement
    if(future_steps < 0) 
    {
        if(stepper_table_pos==3)
            stepper_table_pos=0;
        else
            ++stepper_table_pos;

        PORTA = STEPPER_ARRAY[stepper_table_pos];

        if (stepper_pos == 0)
            stepper_pos = 199;
        else
            --stepper_pos;

    }	
	// Counter-Clockwise stepper movement    
	else if(future_steps > 0) 
    {
        if(stepper_table_pos==0)
            stepper_table_pos=3;
        else
            --stepper_table_pos;

        PORTA = STEPPER_ARRAY[stepper_table_pos];

        if (stepper_pos == 199)
            stepper_pos = 0;
        else
            ++stepper_pos;
    }

	// Adjust stepper motor timer and direction tracking
    ISR_TIMER_RESET:
    prev_direction = curr_direction;
    TCNT3 = 0;
    OCR3A = CURRENT_DELAY;
}

//-------------------- TIMER 1 --------------------
// Async countdown timer for stepper reversal
ISR(TIMER1_COMPA_vect)
{
    countdown_reached = 1;
    TCCR1B &= ~(TIMER1_PRESCALE);  //  disable timer
}

//-------------------- TIMER 2 --------------------
// Async countdown timer for rampdown
ISR(TIMER2_COMPA_vect)
{
    static uint16_t ramp_timer_count = 0;

    ++ramp_timer_count;
    if (ramp_timer_count == 60)
        EICRA &= ~_BV(ISC11); // disable OI interrupt
    if (ramp_timer_count < 75)
        return;

    rampdown_time_reached = 1;
    TCCR2B &= ~(TIMER2_PRESCALE);  //  disable timer
}

//--------------------  
// If an unexpected interrupt occurs
ISR(BADISR_vect)
{
    // user code here
    state = BADISR;
}
