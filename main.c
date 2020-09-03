/*
 * main.c for kraken_pod_24a_hello_world built in Atmel Studio 7.0
 *
 * The below file tests the GPIOs, Pin Change ISR, ADC, and PWM Ouput Control on the kraken_pod_20200731 using an ATTiny24A
 *
 * Datasheet for Attiny24a: https://www.microchip.com/wwwproducts/en/ATtiny24A
 *
 * Pin Configuration:
 * PA0      : Connected to 7pin Header J2
 * PA1		: Connected to 7pin Header J2 AND (jumpered) to Pull-up Resistor Switch S11 to GND through 2pin J4 Header 
 * PA2      : Connected to 7pin Header J2 AND (jumpered) to LED D10 through 2pin J2 Header
 * PA3/ADC3 : Connected to 7pin Header J2 AND (wired) to ADC3 referenced to 1.1V
 * PA4      : Connected to 7pin Header J2 AND (jumpered) to SCK_TPICLK through 5x2 Header J7
 * PA5      : Connected to 7pin Header J2 AND (jumpered) to MISO_TPIDATA through 5x2 Header J7
 * PA6      : Connected to 7pin Header J1 AND (jumpered) to MOSI through 5x2 Header J7
 * PA7/OC0B : Connected to 7pin Header J1 AND (wired) to LED D10 through Pin 2 of J2 Header using PWM Output
 * PB0      : Connected to 7pin Header J1
 * PB1      : Connected to 7pin Header J1
 * PB2      : Connected to 7pin Header J1
 * PB3      : Connected to 7pin Header J1 AND (optional) to RESET_B through 5x2 Header J7
 *
 * Created: 9/2/2020 11:18:02 AM
 * Author : gemoore
 */ 

#define F_CPU 4000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>


#define SW_LED_RED PINA1
#define LED_RED PINA2
#define LED_PWM PINA7

ISR(PCINT0_vect) {
	if (PINA & _BV(SW_LED_RED))
		PORTA ^= _BV(LED_RED);
}

void pwm_setup (void)
{
	TCCR0B |= (1 << CS01); // Set Timer 0 prescaler to clock/8. (At 4.0 MHz this is 0.5 MHz) See ATtiny24 datasheet, Table 11-9.
	
	TCCR0A |= (1 << WGM01) | (1 << WGM00); // Set to 'Fast PWM' mode
	
	TCCR0A |= (1 << COM0B1); // Clear OC0B output on compare match, upwards counting.
}

void pwm_write (uint8_t val)
{
	OCR0B = val;  //LED Control signal on PA2
}

void adc_setup (void)
{
	ADMUX |= (1 << MUX0) | (1 << MUX1)|(1 << REFS1);//| (1 << MUX2)  //Measure Current Consumption on PA3(ADC3) and Set the VRef = 1.1V (i.e. internal reference)
	
	//ADCSRB |= (1 << ADLAR); //If the result is left adjusted (ADLAR = 1) and no more than 8-bit precision is required, it is sufficient to read ADCH

	ADCSRA |= (1 << ADPS1) | (1 << ADPS0) | (1 << ADEN); // Set the prescaler to clock/8 & enable ADC
}

uint16_t adc_read (void)
{
	uint16_t adc_out = 0;
	ADCSRA |= (1 << ADEN); // Turning ADC Off
	
	ADCSRA |= (1 << ADSC); // Start the conversion

	// Wait for it to finish - blocking
	while (ADCSRA & (1 << ADSC));	// Recommended over "while (!(ADCSRA & (1 << ADIF)));" due to input pin hysteresis (?)
	ADCSRA |= (1 << ADIF); // Manually clearing the ADIF flag by writing Logic 1 (ATTiny24a Sec. 16.13.2)
	
	adc_out = ADCL; //Make sure to read ADCL before ADCH or they won't clear
	adc_out = (ADCH << 8)| adc_out;
	
	ADCSRA &= ~(1 << ADEN); // Turning ADC Off
	return adc_out;
}

int main(void)
{
	uint16_t adc_in;
	
	// LED is an output.
	DDRA |= (1 << LED_PWM);
	
	adc_setup();
	pwm_setup();

	DDRA |= _BV(LED_RED);           // Set port PA2 as output (all others are input)
	
	PCMSK0 |= _BV(SW_LED_RED);            // Set pin change interrupt mask to listen to port PA1
	
	MCUCR = _BV(ISC01) | _BV(ISC00);  // Set interrupt on INT0 pin falling edge (high pulled to low)
	
	GIMSK |= _BV(PCIE0);              // Enable PCINT interrupt

	sei();  	                      // Global interrupts

	set_sleep_mode(SLEEP_MODE_PWR_DOWN);

	PORTA ^= _BV(LED_RED);
	_delay_ms(250);
	while (1)
	{
		adc_in = 0;
		adc_in = adc_read(); // 1/2 battery voltage

		//pwm_write(adc_in);
		if(adc_in > 1){
			//PORTA ^= _BV(LED_RED);
			// Update Output Compare Register (PWM 0-100%)
			for (int i = 0; i < 255 ; i++ ){
				OCR0B = i;
				_delay_ms(5);
			}

			// Update Output Compare Register (PWM 100%-0)
			for (int i = 255 ; i >= 0 ; i-- ){
				OCR0B = i;
				_delay_ms(5);
			}
		}
		sleep_cpu();
		_delay_ms(1000);
	}
}
