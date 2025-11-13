#include <Arduino.h>
#define F_CPU 16000000UL // 16 MHz clock
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>

#define LIFT_FAN PD4
#define TRIG_PIN PB3  // Trigger pin
#define ECHO_PIN PD2  // Echo pin (PD2) - INT0

void setup() {
    // Init UART
    UBRR0H = 0;
    UBRR0L = 103;  // 9600 baud
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

	// Timer1
    TCCR1A = 0;
    TCCR1B |= (1 << CS11); // Prescaler 8
    TCNT1 = 0;

    // PD4 as output (Lift Fan)
    DDRD |= (1 << PD4);
    // Start with lift fan OFF
    PORTD &= ~(1 << PD4);

    // Configure TRIG_PIN as OUTPUT
    DDRB |= (1 << TRIG_PIN);
    PORTB &= ~(1 << TRIG_PIN);

    // Configure ECHO_PIN as INPUT
    DDRD &= ~(1 << ECHO_PIN);

    // Configure INT0 interrupt for rising edge
    // External Interrupt Control Register A (EICRA) -> Controls when the interrupt triggers (rising edge)
    EICRA |= (1 << ISC01) | (1 << ISC00);  // Rising edge
    // External Interrupt Mask Register (EIMSK) -> Enables specific interrupt (PD2 -> INT0)
    EIMSK |= (1 << INT0);  // Enable INT0
    
    // Enable global interrupts
    sei();
}

// --- US Sensor Functions --- //
// Variables for interrupt handling
volatile uint32_t pulse_start = 0;
volatile uint32_t pulse_width = 0;
volatile uint8_t measurement_ready = 0;

// INT0 interrupt - triggered on both rising and falling edges
ISR(INT0_vect) {
    // Check rising edge (echo pin HIGH -> sent echo pulse can start timer)
    if (PIND & (1 << ECHO_PIN)) {
        // Rising edge - start measurement timer
        pulse_start = TCNT1;
        // Change to falling edge trigger
        EICRA |= (1 << ISC01);
        EICRA &= ~(1 << ISC00);
    } else {
        // Falling edge - end measurement timer -> pulse received back
        uint32_t pulse_end = TCNT1;
        
        // Handle timer overflow
        if (pulse_end < pulse_start) {
            pulse_width = (65536 - pulse_start) + pulse_end;
        } else {
            pulse_width = pulse_end - pulse_start;
        }
        
        // Convert to microseconds (Timer1 with prescaler 8)
        pulse_width = (pulse_width * 8) / (F_CPU / 1000000);
        
        measurement_ready = 1;
        
        // Change back to rising edge trigger for next measurement
        EICRA |= (1 << ISC01) | (1 << ISC00);
    }
}

void sendTriggerPulse() {
    measurement_ready = 0;
    PORTB &= ~(1 << TRIG_PIN); // Make sure trigger pin is LOW
    _delay_us(2);              
    PORTB |= (1 << TRIG_PIN);  // Send trigger pulse HIGH
    _delay_us(10);             // Pulse width
    PORTB &= ~(1 << TRIG_PIN); // Ensure trigger pin is LOW again
}

uint32_t getDistanceLeft() {
    sendTriggerPulse();

    uint16_t timeout = 100000;
    while (!measurement_ready && timeout--) {
        _delay_us(1);
    }

    if (measurement_ready == 1)
    {
        uint32_t duration = pulse_width;
        // Speed of sound is 0.0343 cm/us
        // Distance = (duration * 343) / 20000
        uint32_t distance = (duration * 343) / 20000;
        return distance;
    }

    return 9999;
}

// --- Lift Fan Functions --- //
void startLiftFan() {
	PORTD |= (1<<PD4);
}

void stopLiftFan() {
	PORTD &= ~(1<<PD4);
}

// --- UART Functions --- //
void uartTransmit(char c) {
    while (!(UCSR0A & (1 << UDRE0))); // Wait until buffer is empty
    UDR0 = c;
}

void uartPrint(const char* str) {
  while (*str) {
      uartTransmit(*str++);
  }
}

void uartPrintInt(uint32_t num) {
  char buffer[10];
  itoa(num, buffer, 10);
  uartPrint(buffer);
}

int main() {
	setup();
	uartPrint("Hovercraft Initialized ! \r\n");

	// Main loop
	while (1)
	{
        uint32_t distanceLeft32 = getDistanceLeft();
        if (distanceLeft32 == 9999)
        {
            uartPrint("US Sensor left error");
        } else {
            uartPrintInt(distanceLeft32);
            uartPrint("cm \r\n");
        }

        _delay_ms(500);

        // Test lift fan
		/* startLiftFan();
		uartPrint("Starting...\r\n");
		_delay_ms(5000);
		uartPrint("Stopping...\r\n");
		stopLiftFan();
		_delay_ms(5000); */
	}
}