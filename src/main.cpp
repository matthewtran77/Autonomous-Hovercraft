#include <Arduino.h>
#define F_CPU 16000000UL // 16 MHz clock
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>

#define LIFT_FAN PD4
#define TRIG_PIN PB3  // Trigger pin
#define ECHO_PIN PD2  // Echo pin (PD2) - INT0
#define IR_PIN 0  // ADC channel for IR sensor (A0 / PC0)

#define IMU_ADDR 0x68

// Current gyroscope range setting
float gyro_scale = 131.0f;
float acc_scale = 16384.0f;

int16_t AccX, AccY, AccZ, GyrX, GyrY, GyrZ; // raw
float accX, accY, accZ, gyrX, gyrY, gyrZ; // converted to g or deg
float yaw = 0.0f;
float pitch = 0.0f;
float roll = 0.0f;
float velocityX = 0.0f;   // in m/s
float offset_ax, offset_ay, offset_az, offset_gz;

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

// ---- I2C Functions ---- //
void i2c_init(void) {
    TWSR = 0x00;  // prescaler = 1
    TWBR = 12;    // ~400kHz @ 16MHz
}

void i2c_start(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

void i2c_stop(void) {
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
    _delay_us(100);  // Small delay for stop condition
}

void i2c_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

uint8_t i2c_read_ack(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
  }
  
  uint8_t i2c_read_nack(void) {
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

// --- IR Sensor Functions --- //
void ADC_init() {
    ADMUX = (1 << REFS0); // AVcc 5v as reference
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // enable ADC, prescaler 128 (ADC clock = 125kHz)
}

uint16_t readADC(uint8_t channel) {
    ADMUX = (ADMUX & 0xF8) | (channel & 0x07); // select channel 0 to 7 ADC
    ADCSRA |= (1 << ADSC); // convert values
    while (ADCSRA & (1 << ADSC)); // wait for conversion
    return ADCW; // return 10 bit ADC result
}

uint16_t getDistanceTop() {
    uint16_t sensorValueRaw = readADC(IR_PIN);

    if (sensorValueRaw == 0) {
        uartPrint("Error: ADC returned 0!\r\n");
        return 0;
    }
    
    // IR sensor calibration equation
    float distance = (6787.0 / (sensorValueRaw - 3.0)) - 4.0;

    // range check
    if (distance < 0 || distance > 200) {
        uartPrint("Error: Out of range!\r\n");
        return 0;
    }

    return (uint16_t)distance;
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

// --- IMU Sensor Functions --- //
void mpu6050_set_gyro_range(uint8_t range) {
    i2c_start();
    i2c_write((IMU_ADDR << 1) | 0);
    i2c_write(0x1B);  // GYRO_CONFIG register
    i2c_write(range << 3);  // Bits 4:3 set the range
    i2c_stop();
    
    // Update scale factor based on range
        case GYRO_RANGE_250:
            gyro_scale = 131.0f;
            uartPrint("Gyro range set to ±250°/s\r\n");
            break;
        case GYRO_RANGE_500:
            gyro_scale = 65.5f;
            uartPrint("Gyro range set to ±500°/s\r\n");
            break;
        case GYRO_RANGE_1000:
            gyro_scale = 32.8f;
            uartPrint("Gyro range set to ±1000°/s\r\n");
            break;
        case GYRO_RANGE_2000:
            gyro_scale = 16.4f;
            uartPrint("Gyro range set to ±2000°/s\r\n");
            break;
    }
}

void mpu6050_set_acc_range(uint8_t range) {
    i2c_start();
    i2c_write((IMU_ADDR << 1) | 0);
    i2c_write(0x1C);  // GYRO_CONFIG register
    i2c_write(range);  // Bits 4:3 set the range
    i2c_stop();
    
    // Update scale factor based on range
    switch(range) {
        case ACCEL_RANGE_2G:
            acc_scale = 16384.0f;
            uartPrint("Accel range set to 2g\r\n");
            break;
        case ACCEL_RANGE_4G:
            acc_scale = 8192.0f;
            uartPrint("Accel range set to 4g\r\n");
            break;
        case ACCEL_RANGE_8G:
            acc_scale = 4096.0f;
            uartPrint("Accel range set to 8g\r\n");
            break;
        case ACCEL_RANGE_16G:
            acc_scale = 2048.0f;
            uartPrint("Accel range set to 16g\r\n");
            break;
    }
}

void mpu6050_init(void) {
    uartPrint("Initializing MPU6050...\r\n");
    
    // Wake up MPU-6050
    i2c_start();
    i2c_write((IMU_ADDR << 1) | 0);
    i2c_write(0x6B); // PWR_MGMT_1
    i2c_write(0x00);
    i2c_stop();

    _delay_ms(10);

    // set gyro / acc range
    

    uartPrint("MPU6050 initialized successfully\r\n");
}

int main() {
	setup();
    _delay_ms(5);
    ADC_init();
    uartPrint("Hovercraft Initialized ! \r\n");

	// Main loop
	while (1)
	{
        // IR Sensor
/*         uint16_t distanceIR = getDistanceTop();
        uartPrint("IR Distance\r\n");
        uartPrintInt(distanceIR);
        uartPrint("cm\r\n");
        _delay_ms(500);

        if (distanceIR > 40)
        {
            stopLiftFan();

            // TODO: stop thrust
        } else {
            // TODO: Continue
        } */
        

        // US Sensor test
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
		_delay_ms(15000);
		uartPrint("Stopping...\r\n");
		stopLiftFan();
		_delay_ms(5000); */
	}
}