#include<avr\io.h>
#include<avr\interrupt.h>
#include<util\delay.h>

// Ultrasonic & Servo motor function
void getecho();
void degree(int n);
void check_direction(int n);

// Ultrasonic & Servo motor variable
volatile int select = 0;
volatile int start = 0;
volatile int result = 0;
volatile int distance = 0;
volatile int toggle = 0;

int main()
{
    cli(); // interrupt disable

   // GPIO SETTING 
    DDRB |= (1 << PB4) | (1 << PB3) | (0 << PB2) | (0 << PB1) | (0 << PB0);
    // PB0, PB1, PB2, PB3 : SPI SLAVE
    // PB4 TIMER0 (PWM)

    DDRD = (0 << PD1) | (1 << PD0);
    // PD1 ULTRASONIC ECHO(Input) - INTERRUPT
    // PD0 ULTRASONIC TRIGGER (Output)

    DDRE = (1 << PE2) | (1 << PE1) | (1 << PE0);
    // USE LED

   // TIMER SETTING 

    // Timer 0 : 8-BIT FAST PWM MODE, prescaler 1024
    TCCR0 |= (1 << WGM01) | (1 << WGM00) | (1 << COM01) | (0 << COM00) | (1 << CS02) | (1 << CS01) | (1 << CS00);
    OCR0 = 0X00;
    // Timer 2 : Normal Mode, prescaler 8
    TCCR2 = (1 << CS21);
    TCNT2 = 0x00;

    // INTERRUPT SETTING
    EIMSK = (1 << INT1); //External Interrupt 1
    EICRA = (1 << ISC11) | (1 << ISC10); //Rising edge

   // SPI SETTING

    SPCR = (1 << SPE) | (1 << SPR0); //spr0 = frequency 16
    SPDR = 0XFF;

    sei(); // interrupt enable

    do {
        while (!(SPSR & (1 << SPIF)));  // SPDR Receive
        start = SPDR;
        result = 0;

        if (start == 0xAA)  // Start Checking Ultrasonic with Servo
        {
            for (int i = 1; i < 4; i++)
            {
                degree(i);  // Degree 0, Degree 90, Degree 180
            }
            OCR0 = 0x16;
            _delay_ms(1000);

            SPDR = result;  // Send a result value of Ultrasonic checking
        }
    } while (1);
}

void degree(int n)
{
    if (n == 1) // Degree 0
    {
        PORTE |= (1 << PE0);
        OCR0 = 0x00;
        _delay_ms(1000); // Delay for exact check
        getecho();
        _delay_ms(1000);
        check_direction(1);
    }
    else if (n == 2) // Degree 90
    {
        PORTE |= (1 << PE1);
        OCR0 = 0x16;
        _delay_ms(1000); // Delay for exact check
        getecho();
        _delay_ms(1000);
        check_direction(2);
    }
    else if (n == 3) // Degree 180
    {
        PORTE |= (1 << PE2);
        OCR0 = 0x25;
        _delay_ms(1000); // Delay for exact check
        getecho();
        _delay_ms(1000);
        check_direction(3);
    }
}

ISR(TIMER2_OVF_vect)
{
    cli();

    TCNT2 = 0x8B; // Distance in cm = echo pulse width in uS/58

    distance += 1;  // Calculate distance between object

    if (distance > 100)
    {
        distance = 0xff;
    }

    sei();
}

void getecho() {
    EICRA = (1 << ISC11) | (1 << ISC10); // Rising edge
    PORTD = (1 << PD0); // Trigger high 
    _delay_us(10); // For Trigger 10us
    PORTD = (0 << PD0); // Trigger low
}

ISR(SIG_INTERRUPT1)
{
    cli();
    if (toggle == 0)  // When echo pulse is rising edge
    {
        distance = 0;
        TIMSK = (1 << TOIE2); //Timer 2 interrupt enable
        EICRA = (1 << ISC11); //Falling edge
        toggle = 1;
    }
    else {  // When echo pulse is falling edge
        toggle = 0;
        TIMSK = (0 << TOIE2);
        distance /= 2;
    }
    sei();
}

void check_direction(int n)  // If the distance is less than 25cm
{
    if ((distance < 25))
    {
        result |= (1 << n - 1); // Save result value for SPI
    }
}