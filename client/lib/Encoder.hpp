#ifndef ENCODER_HPP
#define ENCODER_HPP

#include "constants.h"

/*  Quadrature encoders used on each motor....
 *
 *  Going forwards (count++)
 *  Channel one
 *     _____     _____
 *    |     |   |     | 
 * ___|     |___|     |__
 *
 *  Channel two
 *        _____     _____
 *       |     |   |     | 
 *    ___|     |___|     |__
 *
 *
 *
 *  Going Backwards (count--)
 *  Channel one
 *        _____     _____
 *       |     |   |     | 
 *    ___|     |___|     |__
 *  Channel two
 *     _____     _____
 *    |     |   |     | 
 * ___|     |___|     |__
 */


// Variables --------------------------------

/* Count list
 * 0 - A
 * 1 - B
 */
static volatile int32  count[2]; 

/* Pin values
 * 0 - A1
 * 1 - A2 
 * 2 - B1
 * 3 - B2
 */
static volatile uint8  pins[4];

/* Speed variables
 * old_count -> keeps track of last count value
 * old_time  -> keeps track of time since last update
 * speed     -> keeps track of counts/second
 */
static volatile int32  old_count[2];
static volatile elapsedMillis old_time[2];
static volatile int32  speed[2];

/* Interrupt routine for Motor A channel one
 */
ISR(INT0_vect)
{
	cli();
	pins[0]   = digitalReadFast(M_AENC1);
	count[0] += ((pins[0] ^ pins[1]) << 1) - 1; 
	sei();
}

/* Interrupt routine for Motor A channel two
 */
ISR(INT1_vect)
{
	cli();
	pins[1]   = digitalReadFast(M_AENC2);
	count[0] += ((pins[0] & pins[1]) << 1) - 1; 
	sei();	
}

/* Interrupt routine for Motor B channel one
 */
ISR(INT2_vect)
{
	cli();
	pins[2]   = digitalReadFast(M_BENC1);
	count[0] += ((pins[0] ^ pins[1]) << 1) - 1; 
	sei();
}

/* Interrupt routine for Motor B channel one
 */
ISR(INT3_vect)
{
	cli();
	pins[1]   = digitalReadFast(M_AENC2);
	count[0] += ((pins[0] & pins[1]) << 1) - 1; 
	sei();	
}

/* Timer Based Interrupt - (To calculate speed)
 */
void update_speed()
{
	speed[0] = (count[0] - old_count[0])/(old_time[0]);
	speed[1] = (count[1] - old_count[1])/(old_time[1]);

	old_time[0] = 0;
	old_time[1] = 0;
}

// Encoder class -----------------------------
class EncoderInterface
{
	IntervalTimer enc_timer;

	public:
		EncoderInterface();
		~EncoderInterface() {};

		uint32 get_count(Motor m);
		uint32 get_speed(Motor m);
};

#endif
