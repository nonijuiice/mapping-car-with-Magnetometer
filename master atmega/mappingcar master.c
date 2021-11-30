#include<avr\io.h>
#include<avr\interrupt.h>
#include<util\delay.h>
#include<math.h>

// basic function
void go();
void left();
void right();
void stop();
void go_straight();
void geomagnetic_sensor();
void spi_check(); // check barricade

// mode = 1 function

void starting_point(); // coordinate change 
void move(); // navigation function

// mode = 2 function

void i_hate_magnetic_wave(); // find highest magnetic wave direction
void i_find_magnetic_wave(); // turn highest magnetic wave direction
void i_follow_magnetic_wave(); // follow highest magnetic wave direction

// basic variable 

volatile int time_count = 0;
volatile int count = 0;
volatile int t_mode = 0;
volatile int check = 0;
volatile int mode = 0;
volatile int data1, data2, sum = 0;
volatile int i_hate_atmega128 = 1;


// mode = 1 variable

volatile double coordinate[5] = { 0,0,0,0,0 }; // 1 - (+y), 2 - (-x), 3 - (-y), 4 - (+x)
volatile int direction = 1;
volatile int x = 0, y = 0;
volatile int x_position = 0, y_position = 0; // x,y input value(destination)
volatile int start_coordinate;
volatile int check_R = 0, check_F = 0, check_L = 0;

// mode = 2 variable

volatile int tcnt_H = 0; // not use
volatile int tcnt_L = 0; // not use
volatile int max = 0; // highest magnetic wave
volatile int save_time_count = 0; // highest magnetic wave time 
volatile int save_tcnt_H = 0; // not use
volatile int save_tcnt_L = 0; // not use


int main()
{
	cli();

	// GPIO SETTING

	DDRB = (1 << PB7) | (1 << PB6) | (1 << PB5) | (1 << PB4) | (0 << PB3) | (1 << PB2) | (1 << PB1) | (1 << PB0);
	// SPI
	 // PB0 : SLAVE SELECT, PB1: SCK, PB2: MOSI, PB3: MISO
	// PWM 
	 // PB4 : OC0 (MOTOR DRIVER LEFT1) PB5 : OC1A (MOTOR DRIVER LEFT1)
	 // PB4 : OC0 (MOTOR DRIVER RIGHT1) PB5 : OC1A (MOTOR DRIVER RIGHT1)

	DDRF = (0 << PF3) | (1 << PF0) | (1 << PF1) | (1 << PF2);
	// MAGNETOMETER MODULE
	// PF0 : CH1,  PF1 : CH2,  PF2 : CS (x,y,z select)
	// PF3 : magnetometer_module_ouput(ADC)

	DDRD |= (0 << PD2) | (1 << PD3);
	// UART - bluetooth
	// PD2 : RXD1 (INPUT), PD3 : TXD1 (OUTPUT)


	DDRC = (0 << PC0) | (0 << PC1) | (1 << PC5);
	// PC0 : switch0, PC1 : switch1
	// PC5 : buzzer (not use)

  // ADC setting
	ADMUX |= (1 << REFS0) | (0 << ADLAR) | (1 << MUX1) | (1 << MUX0);
	ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (0 << ADPS1) | (0 << ADPS0);
	// ADC PRESCLAER 16


   // TIMER SETTING

	 // TIMER 1 - motor control
	TCCR1A = (1 << COM1A1) | (0 << COM1A0) | (1 << COM1B1) | (0 << COM1B0) | (1 << COM1C1) | (0 << COM1C0) | (0 << WGM11) | (1 << WGM10);
	TCCR1B = (0 << WGM13) | (1 << WGM12) | (1 << CS12) | (0 << CS11) | (1 << CS10);
	// 8-BIT fast PWM
	// PRESCALER 1024
	// clear OCnA/B/C on compare match

	OCR1AH = 0X00; // NOT USE
	OCR1AL = 0X00;
	// RIGHT WHEEL BACKWARD   

	OCR1BH = 0X00; // NOT USE
	OCR1BL = 0X00;
	// LEFT WHEEL FORWARD

	OCR1CH = 0X00; // NOT USE
	OCR1CL = 0X00;
	// LEFT WHEEL BACKWARD

	// TIMER 0 motor control
	TCCR0 = (1 << WGM01) | (1 << WGM00) | (1 << COM01) | (0 << COM00) | (1 << CS02) | (1 << CS01) | (1 << CS00);
	// 8-BIT fast PWM
	 // PRESCALER 1024
	 // clear OCnA/B/C on compare match

	OCR0 = 0X00;
	// RIGHT WHEEL FORWARD 

	// TIMER 3 - use timer

	ETIMSK = (1 << TOIE3);
	// timer3 interrupt enable

	TCCR3B = (1 << CS32) | (0 << CS31) | (1 << CS30);
	// PRESCALER 1024

	TCNT3H = 0XF9;
	TCNT3L = 0X25;
	// 1 sec

  // UART SETTING

	UCSR1C |= (0 << UMSEL1) | (0 << UPM11) | (0 << UPM10) | (0 << USBS1) | (1 << UCSZ11) | (1 << UCSZ10);
	// Asynchronous, not use parity bit, 1-bit stop bit, 8bit
	// use parity bit > EVEN (1 << UPM11) | (0 << UPM10), ODD (1 << UPM11) | (1 << UPM10)

	UCSR1B |= (1 << RXEN1) | (1 << TXEN1);
	// receiver enable, transmitter enable

	UBRR1H |= 0X00;
	UBRR1L |= 0X67;
	// 9600 bps

  // SPI MASTER

	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
	// SPI ENABLE
	// SPI MASTER
	// SCK frequency : Fosc/4 

	sei();

	// To do a 16-bit write, the high byte must be written before the low byte. 
	// For a 16-bit read, the low byte must be read before the high byte.



	do
	{

		if (mode == 0) // start mode 
		{
			while (!((PINC & (1 << PC1)) | (PINC & (1 << PC0)))); //select mode 1 or mode 2

			mode = PINC; // select mode 

			i_hate_magnetic_wave(); // find highest magnetic wave direction

			i_find_magnetic_wave(); // turn highest magnetic wave direction

			i_hate_atmega128 = 1; // i hate atmega128 

			y = coordinate[1] - coordinate[3]; // x-coordinate 
			x = coordinate[4] - coordinate[2]; // y-coordinate

			stop(); // mode0 end 
		}

		if (mode == 1) // follow coordinate value 
		{

			if (i_hate_atmega128 == 1) // one time run 
			{

				x = 0; y = 0; // now coordinate set

				while (!(UCSR1A & (1 << RXC1))); // wait bluetooth
				start_coordinate = UDR1; // destination

				_delay_ms(10);

				x_position = start_coordinate / 16; // input x destination
				y_position = start_coordinate % 16; // input y destination

				_delay_ms(10);

				UDR1 = x_position * 16 + y_position;
				while (!(UCSR1A & (1 << TXC1))); // transmit destination coordinate

				starting_point(); // coordinate change

				i_hate_atmega128 = 0;

			}

			spi_check(); // confirm barricade

			move(); // navigation function
		}
		else if (mode == 2) // follow magnetic wave 
		{
			if (i_hate_atmega128 == 1) // one time run 
			{

				i_follow_magnetic_wave(); // follow highest magnetic wave direction
				i_hate_atmega128 = 0;

			}

			i_hate_magnetic_wave(); // find highest magnetic wave direction
			i_find_magnetic_wave(); // turn highest magnetic wave direction
			i_follow_magnetic_wave(); // follow highest magnetic wave direction

		}

	} while (1);

}


// TIMER3 INTERRUPT 
ISR(TIMER3_OVF_vect)
{
	cli();

	int q, w, e;
	// timer_mode1 : use go_straight function
	// one count 0.1s
	if (t_mode == 1)
	{
		q = 0xF9;
		w = 0x25;
		e = 33;
	}
	// timer_mode1 : use go_straight function
	// one count 1s
	else if (t_mode == 2)
	{
		q = 0xC2;
		w = 0xF6;
		e = 13;
	}

	TCNT3H = q;
	TCNT3L = w;

	time_count++; // if time_count equal variable e, go_straight function and reverse function stop 

	if (time_count <= e)
	{
		count = 1;
	}
	else if (time_count > e)
	{
		count = 0;
	}
	sei();
}


// FUNCTION
void move()
{

	if (direction == 1) // +y - cordinate 
	{

		if ((x == x_position) && (y == y_position))
		{
			stop();
		}
		else if ((y_position > y) && (check_F != 0x02)) // y 현재값 < 목표값 and 앞이 뚫려있는 경우
		{
			go_straight();
		}

		else if (y_position == y)
		{
			if (check_R != 0x01)
			{
				right();
				stop();
			}
			else if (check_R == 0x01)
			{
				reverse();
				stop();
			}
		}
		else if ((y_position > y) && (check_F == 0x02)) // y 현재값 < 목표값 and 앞이 막혀있는 경우
		{
			if ((check_R != 0x01) && (x_position != x))   // 오른쪽 뚫려있는 경우
			{
				right();
				stop();
			}
			else if ((check_R == 0x01) || (x_position == x))   // 오른쪽 막혀있는 경우 
			{
				if ((check_L == 0x04) || (y != 0))   // 왼쪽 막혀있는 경우 or x<0으로 가게 되는 경우 
				{
					reverse();
					stop();
				}
				else      // 왼쪽 뚫려있는 경우
				{
					left();
					stop();
				}
			}
		}
	}

	else if (direction == 2) // -x coordinate
	{
		if (check_L != 0x04)
		{
			left();
			stop();
		}
		else if (check_L == 0x04)
		{
			if (check_F != 0x02)
			{
				go_straight();
			}
		}
	}

	else if (direction == 3) // -y coordinate
	{
		if (check_L != 0x04)
		{
			left();
			stop();
		}
		else if (check_L == 0x04)
		{
			if (check_F != 0x02)
			{
				go_straight();
			}
		}
	}

	else if (direction == 4) // +x coordinate
	{
		if ((x == x_position) && (y == y_position))
		{
			stop();
		}

		else if ((x_position > x) && (check_F != 0x02))
		{
			go_straight();
		}
		else if (x_position == x)
		{
			if (check_L != 0x04)
			{
				left();
				stop();
			}
			else if (check_L == 0x04)
			{
				reverse();
				stop();
			}
		}
		else if ((x_position > x) && (check_F == 0x02))
		{
			if ((check_L != 0x04) && (y_position != y))
			{
				left();
				stop();
			}
			else if ((check_L == 0x04) || (y_position == y))
			{
				if ((check_R == 0x01) || (x != 0))
				{
					reverse();
					stop();
				}
				else
				{
					right();
					stop();
				}
			}
		}
	}

}

void go_straight()
{
	time_count = 0;
	t_mode = 1; // timer mode 1 : 0.1 sec
	count = 1; // when count = 0, go_straight function end

	TCNT3H = 0XF9;
	TCNT3L = 0X25;

	while (count == 1)
	{
		go();
	}
	stop(); // stop 1 sec

	if (mode == 1)
	{
		coordinate[direction] += 0.5;

		y = coordinate[1] - coordinate[3];
		x = coordinate[4] - coordinate[2];

		UDR1 = x; // send x coordinate
		while (!(UCSR1A & (1 << TXC1))); // wait the bluetooth communication end

		UDR1 = y; // send y coordinate 
		while (!(UCSR1A & (1 << TXC1))); // wait the bluetooth communication end
	}
}

void go()
{
	// OCR0 - R F OCR1A - R B OCR1B - L F OCR1C - L B
	OCR0 = 0X6A; // right motor turns forward direction

	OCR1BH = 0X00;
	OCR1BL = 0X6A; // left motor turns forward direction

	OCR1AH = 0X00;
	OCR1AL = 0X00;
	OCR1CH = 0X00;
	OCR1CL = 0X00;

}

void left()
{
	// OCR0 - R F OCR1A - R B OCR1B - L F OCR1C - L B
	OCR0 = 0X6A; // right motor turns forward direction

	OCR1BH = 0X00;
	OCR1BL = 0X00;

	OCR1AH = 0X00;
	OCR1AL = 0X00;
	OCR1CH = 0X00;
	OCR1CL = 0X6A; // left motor turns backward direction

	_delay_ms(3500);


	direction += 1;
	if (direction == 5)
	{
		direction = 1;
	}
}


void right()
{
	// OCR0 - R F OCR1A - R B OCR1B - L F OCR1C - L B
	OCR0 = 0X00;

	OCR1BH = 0X00;
	OCR1BL = 0X6A; // left motor turns forward direction 

	OCR1AH = 0X00;
	OCR1AL = 0X6A; // right motor turns backward direction
	OCR1CH = 0X00;
	OCR1CL = 0X00;
	_delay_ms(3500);



	direction -= 1;
	if (direction == 0)
	{
		direction = 4;
	}
}

void reverse() // 180degree rotate
{
	// OCR0 - R F OCR1A - R B OCR1B - L F OCR1C - L B
	OCR0 = 0X00;

	OCR1BH = 0X00;
	OCR1BL = 0X6A; // left motor turns forward direction 

	OCR1AH = 0X00;
	OCR1AL = 0X6A; // right motor turns backward direction 
	OCR1CH = 0X00;
	OCR1CL = 0X00;


	_delay_ms(5000);
	_delay_ms(2000);

	direction -= 2;
	if (direction < 0)
	{
		direction += 4;
	}
}

void i_hate_magnetic_wave() // find highest magnetic wave
{
	max = 0;

	// OCR0 - R F OCR1A - R B OCR1B - L F OCR1C - L B
	OCR0 = 0X00;

	OCR1BH = 0X00;
	OCR1BL = 0X6A; // left motor turns forward direction 

	OCR1AH = 0X00;
	OCR1AL = 0X6A; // right motor turns backward direction  
	OCR1CH = 0X00;
	OCR1CL = 0X00;

	// 1 sec 
	TCNT3H = 0XC2;
	TCNT3L = 0XF6;

	time_count = 0;
	t_mode = 2; // timer mode2 : 1sec
	count = 1;


	while (count == 1)
	{

		geomagnetic_sensor(); // magnetometer sensing

		if (sum > max) // find 
		{
			max = sum;
			save_time_count = time_count;
		}
	}

	stop();
}


void i_find_magnetic_wave()
{

	// OCR0 - R F OCR1A - R B OCR1B - L F OCR1C - L B
	OCR0 = 0X00;

	OCR1BH = 0X00;
	OCR1BL = 0X6A;

	OCR1AH = 0X00;
	OCR1AL = 0X6A;
	OCR1CH = 0X00;
	OCR1CL = 0X00;

	time_count = 0;
	t_mode = 2;
	count = 1;

	TCNT3H = 0XC2;
	TCNT3L = 0XF9;

	while (count == 1)
	{
		tcnt_L = TCNT3L; // not use
		tcnt_H = TCNT3H; // not use

		if (time_count == save_time_count)
		{
			count = 0;
		}

	}

	stop();
}


void i_follow_magnetic_wave()
{
	spi_check();

	if ((check & 0x02) != 0x02)
	{
		go_straight();
	}
	else
	{
		stop();

		// buzzer mot use 
		PORTC |= (1 << PC5);
		_delay_ms(100);
		PORTC &= (0 << PC5);

		mode = 0;

	}

}


void stop()
{

	// OCR0 - R F OCR1A - R B OCR1B - L F OCR1C - L B
	OCR0 = 0X00;

	OCR1BH = 0X00;
	OCR1BL = 0X00;

	OCR1AH = 0X00;
	OCR1AL = 0X00;
	OCR1CH = 0X00;
	OCR1CL = 0X00;

	_delay_ms(1000);

}

void starting_point()
// destination x_position, y_position
// now coordinate x ,y
{
	int tmp = 0;

	if ((x_position < x) && (y_position > y))
	{
		left();
		tmp = x_position;
		x_position = y_position;
		y_position = tmp;
		direction = 1;
	}
	else if ((x_position < x) && (y_position < y))
	{
		reverse();
		x_position = x_position;
		y_position = y_position;
		direction = 1;
	}
	else if ((x_position > x) && (y_position < y))
	{
		right();
		tmp = x_position;
		x_position = y_position;
		y_position = tmp;
		direction = 1;
	}
	else
	{
		stop();
	}
	x = 0;
	y = 0;
}

void spi_check() // check barricade
{
	SPDR = 0XAA; // slave start code : 0XAA
	 
	while (!(SPSR & (1 << SPIF)));

	PORTB |= (1 << PB0); // SPI STOP

	_delay_ms(5000);
	_delay_ms(3000);

	PORTB &= (0 << PB0); // SPI RESTART

	SPDR = 0XCC; // meaningless value
	while (!(SPSR & (1 << SPIF))); // wait SPI 

	check = SPDR; // barricade position

	check_R = check & 0x01;
	check_F = check & 0x02;
	check_L = check & 0x04;

	UDR1 = check; // transmit barricade position
	while (!(UCSR1A & (1 << TXC1))); // wait bluetooth

}


void geomagnetic_sensor()
{
	PORTF = (1 << PF0) | (1 << PF1) | (1 << PF2); // x_coordinate
	ADCSRA |= (1 << ADSC); // analog to digital convert start

	while (!(ADCSRA & (1 << ADIF))); // wait analog to digital convert 
	data1 = ADCL;
	data2 = ADCH;

	sum = data2 * 256 + data1;

}