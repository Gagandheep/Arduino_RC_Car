/*
 * EYantra
 *
 * Created: 2/8/2020 6:54:03 PM
 * Author : #254
 */ 


#define F_CPU 16000000UL		// Define Crystal Frequency of Uno Board
#define USART0_ENABLED

#include <avr/io.h>				// Standard AVR IO Library
#include <util/delay.h>			// Standard AVR Delay Library
#include <avr/interrupt.h>		// Standard AVR Interrupt Library
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include "UART.h"

#define PIN_MOTOR_RIGHT_SPEED		PB3	// 11
#define PIN_MOTOR_LEFT_SPEED		PD3 // 3
#define PIN_STRIKER_1				PB4 // 12
#define PIN_STRIKER_2				PB5 // 13
#define PIN_MOTOR_L_1				PD4 // 4
#define PIN_MOTOR_L_2				PD5 // 5
#define PIN_MOTOR_R_3				PD6 // 6
#define PIN_MOTOR_R_4				PD7 // 7
#define PIN_BUZZER					PD2 // 2
#define PIN_WL_LEFT					PC0 // A0
#define PIN_WL_RIGHT				PC1 // A1
#define PIN_WL_CENTER				PC2 // A2

#define SPEED_FAST					255
#define SPEED_SLOW					100
#define SPEED_TURN_FAST				255
#define SPEED_TURN_SLOW				100
#define SPEED_TURN_FAST_2			100
#define SPEED_TURN_SLOW_2			60
#define SPEED_ROTATION				75
#define WL_LEFT_THRESHOLD			70
#define WL_MIDDLE_THRESHOLD			100
#define WL_RIGHT_THRESHOLD			70

/* size of RX/TX buffers */
#define UART_RX0_BUFFER_MASK (UART_RX0_BUFFER_SIZE - 1)

#define UART_TX0_BUFFER_MASK (UART_TX0_BUFFER_SIZE - 1)

#if (UART_RX0_BUFFER_SIZE & UART_RX0_BUFFER_MASK)
	#error RX0 buffer size is not a power of 2
#endif
#if (UART_TX0_BUFFER_SIZE & UART_TX0_BUFFER_MASK)
	#error TX0 buffer size is not a power of 2
#endif

#if defined(__AVR_ATmega328P__)
	/* TLS-Added 48P/88P/168P/328P */
	/* ATmega with one USART */
	#define ATMEGA_USART0
	#define UART0_RECEIVE_INTERRUPT   USART_RX_vect
	#define UART0_TRANSMIT_INTERRUPT  USART_UDRE_vect
	#define UART0_STATUS   UCSR0A
	#define UART0_CONTROL  UCSR0B
	#define UART0_DATA     UDR0
	#define UART0_UDRIE    UDRIE0
#else
	#error "no UART definition for MCU available"
#endif

/*
 *  Module global variables
 */

#if defined(USART0_ENABLED)
	#if defined(ATMEGA_USART) || defined(ATMEGA_USART0)
		static volatile uint8_t UART_TxBuf[UART_TX0_BUFFER_SIZE];
		static volatile uint8_t UART_RxBuf[UART_RX0_BUFFER_SIZE];

		#if defined(USART0_LARGE_BUFFER)
			static volatile uint16_t UART_TxHead;
			static volatile uint16_t UART_TxTail;
			static volatile uint16_t UART_RxHead;
			static volatile uint16_t UART_RxTail;
			static volatile uint8_t UART_LastRxError;
		#else
			static volatile uint8_t UART_TxHead;
			static volatile uint8_t UART_TxTail;
			static volatile uint8_t UART_RxHead;
			static volatile uint8_t UART_RxTail;
			static volatile uint8_t UART_LastRxError;
		#endif

	#endif
#endif


#if defined(USART0_ENABLED)

#if defined(AT90_UART) || defined(ATMEGA_USART) || defined(ATMEGA_USART0)

ISR(UART0_RECEIVE_INTERRUPT)
/*************************************************************************
Function: UART Receive Complete interrupt
Purpose:  called when the UART has received a character
**************************************************************************/
{
    uint16_t tmphead;
    uint8_t data;
    uint8_t usr;
    uint8_t lastRxError;

    /* read UART status register and UART data register */
    usr  = UART0_STATUS;
    data = UART0_DATA;

    /* */
#if defined(AT90_UART)
    lastRxError = (usr & (_BV(FE)|_BV(DOR)));
#elif defined(ATMEGA_USART)
    lastRxError = (usr & (_BV(FE)|_BV(DOR)));
#elif defined(ATMEGA_USART0)
    lastRxError = (usr & (_BV(FE0)|_BV(DOR0)));
#elif defined (ATMEGA_UART)
    lastRxError = (usr & (_BV(FE)|_BV(DOR)));
#endif

    /* calculate buffer index */
    tmphead = (UART_RxHead + 1) & UART_RX0_BUFFER_MASK;

    if (tmphead == UART_RxTail) {
        /* error: receive buffer overflow */
        lastRxError = UART_BUFFER_OVERFLOW >> 8;
    } else {
        /* store new index */
        UART_RxHead = tmphead;
        /* store received data in buffer */
        UART_RxBuf[tmphead] = data;
    }
    UART_LastRxError = lastRxError;
}


ISR(UART0_TRANSMIT_INTERRUPT)
/*************************************************************************
Function: UART Data Register Empty interrupt
Purpose:  called when the UART is ready to transmit the next byte
**************************************************************************/
{
    uint16_t tmptail;

    if (UART_TxHead != UART_TxTail) {
        /* calculate and store new buffer index */
        tmptail = (UART_TxTail + 1) & UART_TX0_BUFFER_MASK;
        UART_TxTail = tmptail;
        /* get one byte from buffer and write it to UART */
        UART0_DATA = UART_TxBuf[tmptail];  /* start transmission */
    } else {
        /* tx buffer empty, disable UDRE interrupt */
        UART0_CONTROL &= ~_BV(UART0_UDRIE);
    }
}


/*************************************************************************
Function: uart0_init()
Purpose:  initialize UART and set baudrate
Input:    baudrate using macro UART_BAUD_SELECT()
Returns:  none
**************************************************************************/
void uart0_init(uint16_t baudrate)
{
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		UART_TxHead = 0;
		UART_TxTail = 0;
		UART_RxHead = 0;
		UART_RxTail = 0;
	}

#if defined(AT90_UART)
	/* set baud rate */
	UBRR = (uint8_t) baudrate;

	/* enable UART receiver and transmitter and receive complete interrupt */
	UART0_CONTROL = _BV(RXCIE)|_BV(RXEN)|_BV(TXEN);

#elif defined (ATMEGA_USART)
	/* Set baud rate */
	if (baudrate & 0x8000) {
		UART0_STATUS = (1<<U2X);  //Enable 2x speed
		baudrate &= ~0x8000;
	}
	UBRRH = (uint8_t) (baudrate>>8);
	UBRRL = (uint8_t) baudrate;

	/* Enable USART receiver and transmitter and receive complete interrupt */
	UART0_CONTROL = _BV(RXCIE)|(1<<RXEN)|(1<<TXEN);

	/* Set frame format: asynchronous, 8data, no parity, 1stop bit */
#ifdef URSEL
	UCSRC = (1<<URSEL)|(3<<UCSZ0);
#else
	UCSRC = (3<<UCSZ0);
#endif

#elif defined (ATMEGA_USART0)
	/* Set baud rate */
	if (baudrate & 0x8000) {
		UART0_STATUS = (1<<U2X0);  //Enable 2x speed
		baudrate &= ~0x8000;
	}
	UBRR0H = (uint8_t)(baudrate>>8);
	UBRR0L = (uint8_t) baudrate;

	/* Enable USART receiver and transmitter and receive complete interrupt */
	UART0_CONTROL = _BV(RXCIE0)|(1<<RXEN0)|(1<<TXEN0);

	/* Set frame format: asynchronous, 8data, no parity, 1stop bit */
#ifdef URSEL0
	UCSR0C = (1<<URSEL0)|(3<<UCSZ00);
#else
	UCSR0C = (3<<UCSZ00);
#endif

#elif defined (ATMEGA_UART)
	/* set baud rate */
	if (baudrate & 0x8000) {
		UART0_STATUS = (1<<U2X);  //Enable 2x speed
		baudrate &= ~0x8000;
	}
	UBRRHI = (uint8_t) (baudrate>>8);
	UBRR   = (uint8_t) baudrate;

	/* Enable UART receiver and transmitter and receive complete interrupt */
	UART0_CONTROL = _BV(RXCIE)|(1<<RXEN)|(1<<TXEN);

#endif

} /* uart0_init */


/*************************************************************************
Function: uart0_getc()
Purpose:  return byte from ringbuffer
Returns:  lower byte:  received byte from ringbuffer
          higher byte: last receive error
**************************************************************************/
uint16_t uart0_getc(void)
{
	uint16_t tmptail;
	uint8_t data;

	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		if (UART_RxHead == UART_RxTail) {
			return UART_NO_DATA;   /* no data available */
		}
	}

	/* calculate / store buffer index */
	tmptail = (UART_RxTail + 1) & UART_RX0_BUFFER_MASK;

	UART_RxTail = tmptail;

	/* get data from receive buffer */
	data = UART_RxBuf[tmptail];

	return (UART_LastRxError << 8) + data;

} /* uart0_getc */

/*************************************************************************
Function: uart0_peek()
Purpose:  Returns the next byte (character) of incoming UART data without
          removing it from the ring buffer. That is, successive calls to
		  uartN_peek() will return the same character, as will the next
		  call to uartN_getc()
Returns:  lower byte:  next byte in ring buffer
          higher byte: last receive error
**************************************************************************/
uint16_t uart0_peek(void)
{
	uint16_t tmptail;
	uint8_t data;

	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		if (UART_RxHead == UART_RxTail) {
			return UART_NO_DATA;   /* no data available */
		}
	}

	tmptail = (UART_RxTail + 1) & UART_RX0_BUFFER_MASK;

	/* get data from receive buffer */
	data = UART_RxBuf[tmptail];

	return (UART_LastRxError << 8) + data;

} /* uart0_peek */

/*************************************************************************
Function: uart0_putc()
Purpose:  write byte to ringbuffer for transmitting via UART
Input:    byte to be transmitted
Returns:  none
**************************************************************************/
void uart0_putc(uint8_t data)
{

#ifdef USART0_LARGE_BUFFER
	uint16_t tmphead;
	uint16_t txtail_tmp;

	tmphead = (UART_TxHead + 1) & UART_TX0_BUFFER_MASK;

	do {
		ATOMIC_BLOCK(ATOMIC_FORCEON) {
			txtail_tmp = UART_TxTail;
		}
	} while (tmphead == txtail_tmp); /* wait for free space in buffer */
#else
	uint16_t tmphead;

	tmphead = (UART_TxHead + 1) & UART_TX0_BUFFER_MASK;

	while (tmphead == UART_TxTail); /* wait for free space in buffer */
#endif

	UART_TxBuf[tmphead] = data;
	UART_TxHead = tmphead;

	/* enable UDRE interrupt */
	UART0_CONTROL |= _BV(UART0_UDRIE);

} /* uart0_putc */


/*************************************************************************
Function: uart0_puts()
Purpose:  transmit string to UART
Input:    string to be transmitted
Returns:  none
**************************************************************************/
void uart0_puts(const char *s)
{
	while (*s) {
		uart0_putc(*s++);
	}

} /* uart0_puts */


/*************************************************************************
Function: uart0_puts_p()
Purpose:  transmit string from program memory to UART
Input:    program memory string to be transmitted
Returns:  none
**************************************************************************/
void uart0_puts_p(const char *progmem_s)
{
	register char c;

	while ((c = pgm_read_byte(progmem_s++))) {
		uart0_putc(c);
	}

} /* uart0_puts_p */



/*************************************************************************
Function: uart0_available()
Purpose:  Determine the number of bytes waiting in the receive buffer
Input:    None
Returns:  Integer number of bytes in the receive buffer
**************************************************************************/
uint16_t uart0_available(void)
{
	uint16_t ret;

	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		ret = (UART_RX0_BUFFER_SIZE + UART_RxHead - UART_RxTail) & UART_RX0_BUFFER_MASK;
	}
	return ret;
} /* uart0_available */

/*************************************************************************
Function: uart0_flush()
Purpose:  Flush bytes waiting the receive buffer. Actually ignores them.
Input:    None
Returns:  None
**************************************************************************/
void uart0_flush(void)
{
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		UART_RxHead = UART_RxTail;
	}
} /* uart0_flush */

#endif

#endif /* defined(USART0_ENABLED) */


// Initialize hardware
void striker_init(){
	DDRB	|= (1 << PIN_STRIKER_1);
	DDRB	|= (1 << PIN_STRIKER_2);
	PORTB	&= ~(1 << PIN_STRIKER_1);
	PORTB	&= ~(1 << PIN_STRIKER_2);
}

void buzzer_init(){
	DDRD    |= (1 << PIN_BUZZER);
	PORTD   |= (1 << PIN_BUZZER);
}

void motor_init(void){
	DDRB    |= (1 << PIN_MOTOR_LEFT_SPEED);
	PORTB   |= (1 << PIN_MOTOR_LEFT_SPEED);
	DDRD	|= (1 << PIN_MOTOR_RIGHT_SPEED);
	PORTD	|= (1 << PIN_MOTOR_RIGHT_SPEED);
	DDRD	|= 0b11110000;
	PORTD	&= ~0b11110000;
}

void white_line_sensor_init(){
	DDRC &= ~0b00000111;
	PORTC &= ~0b00000111;
}

// Movement functions
// Timer 2 initialized in PWM mode for brightness control
// Prescale:64
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer2_init()
{
	cli(); //disable all interrupts

	TCCR2B = 0x00;	//Stop

	TCNT2 = 0xFF;	//Counter higher 8-bit value to which OCR2A value is compared with

	OCR2A = 0xFF;	//Output compare register low value
	OCR2B = 0xFF;

	//  Clear OC2A, on compare match (set output to low level)
	TCCR2A |= (1 << COM2A1) | (1 << COM2B1);
	TCCR2A &= ~(1 << COM2A0);

	// FAST PWM 8-bit Mode
	TCCR2A |= (1 << WGM20);
	TCCR2A |= (1 << WGM21);
	TCCR2B &= ~(1 << WGM22);

	// Set Prescalar to 64
	TCCR2B &= ~((1 << CS21) | (1 << CS20));
	TCCR2B |= (1 << CS22);

	sei(); //re-enable interrupts
}

//use this function to initialize all devices
void init_devices (void) {
	buzzer_init();
	white_line_sensor_init();
	motor_init();
	striker_init();
	timer2_init();
	uart0_init(UART_BAUD_SELECT(9600, F_CPU));
	uart0_flush();
}

// Buzzer functions
void buzzer_start(){
	PORTD	&= ~(1 << PIN_BUZZER);
}

void buzzer_stop(){
	PORTD   |= (1 << PIN_BUZZER);
}

// UART functions
char uart0_readByte(void){

	uint16_t rx;
	uint8_t rx_status, rx_data;

	rx = uart0_getc();
	rx_status = (uint8_t)(rx >> 8);
	rx = rx << 8;
	rx_data = (uint8_t)(rx >> 8);

	if(rx_status == 0 && rx_data != 0){
		return rx_data;
		} else {
		return -1;
	}

}

// Function for speed control of Wheels
void motor_speed_left (unsigned char speed){
	OCR2B = (unsigned char)speed;
}


void motor_speed_right (unsigned char speed){
	OCR2A = (unsigned char)speed;
}

// Movement functions
void movement_forward(unsigned char speed){
	motor_speed_left(speed);
	motor_speed_right(speed);
	PORTD |= 0b01010000;
	PORTD &= ~0b10100000;
}

void movement_backward(unsigned char speed){
	motor_speed_left(speed);
	motor_speed_right(speed);
	PORTD |= 0b10100000;
	PORTD &= ~0b01010000;
}

void movement_stop(){
	PORTD &= ~0b11110000;
	motor_speed_left(0);
	motor_speed_right(0);
}

void movement_right(int reversed, int slow){
	if (slow == 0)
	{
		motor_speed_left(SPEED_TURN_FAST);
		motor_speed_right(SPEED_TURN_SLOW);
	}
	else
	{
		motor_speed_left(SPEED_TURN_FAST_2);
		motor_speed_right(SPEED_TURN_SLOW_2);
	}

	if (reversed){
		PORTD |= 0b10100000;
		PORTD &= ~0b01010000;
	}
	else{
		PORTD |= 0b01010000;
		PORTD &= ~0b10100000;
	}
}

void movement_left(int reversed, int slow){
	if (slow == 0)
	{
		motor_speed_left(SPEED_TURN_SLOW);
		motor_speed_right(SPEED_TURN_FAST);
	}
	else
	{
		motor_speed_left(SPEED_TURN_SLOW_2);
		motor_speed_right(SPEED_TURN_FAST_2);
	}

	if (reversed)
	{
		PORTD |= 0b10100000;
		PORTD &= ~0b01010000;
	}
	else
	{
		PORTD |= 0b01010000;
		PORTD &= ~0b10100000;
	}
}

void movement_rotate_right(){
	motor_speed_left(SPEED_ROTATION);
	motor_speed_right(SPEED_ROTATION);
	PORTD |= 0b01100000;
	PORTD &= ~0b10010000;
}

void movement_rotate_left(){
	motor_speed_left(SPEED_ROTATION);
	motor_speed_right(SPEED_ROTATION);
	PORTD |= 0b10010000;
	PORTD &= ~0b01100000;
}

// Striking mechanism function
void strike(){
	PORTB |= (1 << PIN_STRIKER_2);
	_delay_ms(200);
	PORTB &= ~(1 << PIN_STRIKER_2);
	PORTB |= (1 << PIN_STRIKER_1);
	_delay_ms(200);
	PORTB &= ~(1 << PIN_STRIKER_1);
}

// White line sensor ADC function
int ADCsingleREAD(uint8_t adc_pin)
{
	int ADCval;

	ADMUX = adc_pin;
	ADMUX |= (1 << REFS0);    // use AVcc as the reference
	ADMUX &= ~(1 << ADLAR);   // clear for 10 bit resolution

	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);    // 128 prescale for 8Mhz
	ADCSRA |= (1 << ADEN);    // Enable the ADC

	ADCSRA |= (1 << ADSC);    // Start the ADC conversion

	while(ADCSRA & (1 << ADSC));      // Wait for the ADC to finish


	ADCval = ADCL;
	ADCval = (ADCH << 8) + ADCval;    // Read ADCH

	return ADCval;
}



//Main Function
int main(){
	init_devices();
	char rx_byte;
	int left_sensor, right_sensor, middle_sensor, sensor_data;
	int move = 0;
	int debug_sensor = 0;
	DDRB	|= 0b00000111;
	PORTB	|= 0b00000000;


	while(1){
		left_sensor		= ADCsingleREAD(0);
		middle_sensor	= ADCsingleREAD(1);
		right_sensor	= ADCsingleREAD(2);

		if (debug_sensor){
			uart0_putc(left_sensor);
			uart0_putc(middle_sensor);
			uart0_putc(right_sensor);
			uart0_putc('\n');
			_delay_ms(50);
		}

		// Sets value as 1 if sensor detects white color else 0 based on threshold set
		left_sensor		= (left_sensor > WL_LEFT_THRESHOLD) ? 0 : 1;
		middle_sensor	= (middle_sensor > WL_MIDDLE_THRESHOLD) ? 0 : 1;
		right_sensor	= (right_sensor > WL_RIGHT_THRESHOLD) ? 0 : 1;

		sensor_data		= (left_sensor << 2) | (middle_sensor << 1) | right_sensor;
		PORTB			= sensor_data;
		rx_byte = uart0_readByte();

		if (rx_byte != 0xFF)
		{
			switch (rx_byte)
			{
				case 'w':{
					move = 1;
					break;
				}
				case 'W':{
					move = 2;
					break;
				}
				case 's':{
					move = 3;
					break;
				}
				case 'S':{
					move = 4;
					break;
				}
				case 'q':{
					move = 0;
					movement_stop();
					break;
				}
				case 'h':{
					strike();
					break;
				}
				case 'd':{
					debug_sensor = (debug_sensor) ? 0 : 1;
					break;
				}
				case 'b':{
					buzzer_start();
					_delay_ms(500);
					buzzer_stop();
					break;
				}
				case 'B':{
					buzzer_start();
					_delay_ms(250);
					buzzer_stop();
					_delay_ms(250);
					buzzer_start();
					_delay_ms(250);
					buzzer_stop();
					_delay_ms(200);
					break;
				}
				case 'f':{
					buzzer_start();
					_delay_ms(5000);
					buzzer_stop();
					break;
				}
				case 'x':{
					move = 5;
					break;
				}
				case 'z':{
					move = 6;
					break;
				}
				case 't':{
					move = 7;
					break;
				}
				case 'g':{
					move = 8;
					break;
				}
				case 'c':{
					move = 0;
					movement_rotate_left();
					_delay_ms(400);
					movement_stop();
					_delay_ms(100);
					break;
				}
				case 'C':{
					move = 0;
					movement_rotate_right();
					_delay_ms(400);
					movement_stop();
					_delay_ms(100);
					break;
				}
			}
		}

		if (move == 1)
		{
			if (sensor_data == 0x02 || sensor_data == 0x05 || sensor_data == 0x07)
			{
				movement_forward(SPEED_FAST);
			}
			else if (sensor_data == 0x03 || sensor_data == 0x01)
			{
				movement_right(0, 0);
			}
			else if (sensor_data == 0x06 || sensor_data == 0x04)
			{
				movement_left(0, 0);
			}
			else if (sensor_data == 0x00)
			{
				movement_stop();
			}
		}

		else if (move == 3)
		{
			if (sensor_data == 0x02 || sensor_data == 0x05 || sensor_data == 0x07)
			{
				movement_backward(SPEED_FAST);
			}
			else if (sensor_data == 0x03 || sensor_data == 0x01)
			{
				movement_right(1, 0);
			}
			else if (sensor_data == 0x06 || sensor_data == 0x04)
			{
				movement_left(1, 0);
			}
			else if (sensor_data == 0x00)
			{
				movement_stop();
			}
		}

		else if (move == 2)
		{
			if (sensor_data == 0x02 || sensor_data == 0x05 || sensor_data == 0x07)
			{
				movement_forward(SPEED_SLOW);
			}
			else if (sensor_data == 0x03 || sensor_data == 0x01)
			{
				movement_right(0, 1);
			}
			else if (sensor_data == 0x06 || sensor_data == 0x04)
			{
				movement_left(0, 1);
			}
			else if (sensor_data == 0x00)
			{
				movement_stop();
			}
		}

		else if (move == 4)
		{
			if (sensor_data == 0x02 || sensor_data == 0x05 || sensor_data == 0x07)
			{
				movement_backward(SPEED_SLOW);
			}
			else if (sensor_data == 0x03 || sensor_data == 0x01)
			{
				movement_right(1, 1);
			}
			else if (sensor_data == 0x06 || sensor_data == 0x04)
			{
				movement_left(1, 1);
			}
			else if (sensor_data == 0x00)
			{
				movement_stop();
			}
		}
		
		else if (move == 5) 
		{
			movement_rotate_right();
		}
		
		else if (move == 6)
		{
			movement_rotate_left();
		}
		
		else if (move == 7)
		{
			movement_forward(SPEED_ROTATION);
		}
		
		else if (move == 8)
		{
			movement_backward(SPEED_ROTATION);
		}

		//if (sensor_data == 0x00)
		//{
			//movement_rotate_left();
		//}
		
		_delay_ms(1);
		movement_stop();
	}
}
