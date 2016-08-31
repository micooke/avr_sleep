/*
Power save stuff based off:
Low Power Testing
Spark Fun Electronics 2011
Nathan Seidle
https://www.sparkfun.com/tutorials/309
*/

#include <Arduino.h>
#include <avr/sleep.h> //Needed for sleep_mode
#include <avr/power.h> //Needed for powering down peripherals such as the ADC/TWI and Timers
#include <util/atomic.h>

#if defined(__AVR_ATtiny25__) | defined(__AVR_ATtiny45__) | defined(__AVR_ATtiny85__)
#define INT0_PIN 2
#define MAX_PINS 6
#elif defined(__AVR_ATmega16U4__) | defined(__AVR_ATmega32U4__)
#define INT0_PIN 3 // Note: INT1 is on D2
#define MAX_PINS 18
//#elif defined(__AVR_ATmega168__) | defined(__AVR_ATmega168P__) | defined(__AVR_ATmega328P__)
#else
#define INT0_PIN 2
#define MAX_PINS 18
#endif

#ifndef WDTCSR // i.e. for the ATtiny85
#define WDTCSR WDTCR
#define EIFR GIFR // INTF0
#endif

volatile uint8_t *INT0_PCMSK;
uint8_t INT0_PCMSK_bit;

// ISR : Watchdog Interrupt
ISR(WDT_vect)
{
	//This order of commands is important and cannot be combined (beyond what they are below)
	MCUSR &= ~_BV(WDRF); // Clear the watch dog reset
	// timed sequence
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		WDTCSR |= _BV(WDCE) | _BV(WDE); //Set WD_change enable, set WD enable
		WDTCSR &= ~_BV(WDIE); // Disable watchdog interrupt, disable watchdog
	}
}
// ISR : External Interrupt
ISR(INT0_vect)
{
	EIFR &= ~_BV(INTF0); // Clear the external interrupt flag
	PCMSK = 0x00;
	GIMSK &= ~_BV(INT0);
	GIMSK |= _BV(INT0);
}

static uint32_t WDT_TIME_ms[10] = { 16, 32, 64, 125, 250, 500, 1000, 2000, 4000, 8000 };

class avr_sleep
{
private:
	bool _int0_isEnabled = false;
	bool _wdt_isEnabled = false;
	uint8_t _wdt_Prescalar = 255;

public:
	avr_sleep() : _wdt_Prescalar(255), _int0_isEnabled(false), _wdt_isEnabled(false) {}
	~avr_sleep() {}

public:
	void setup()
	{
		//To reduce power, setup all pins as inputs with no pullups
		// (saves about 1.3uA when used with watchdog timer in this example)
		for (uint8_t x = 0; x < MAX_PINS; ++x)
		{
			pinMode(x, INPUT);
			digitalWrite(x, (x == 2)); // set PB2 / INT0 high
		}
		//Power down various bits of hardware to lower power usage
		disable_peripherals();
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	}

	void sleep_int0()
	{
		//setup wakeup on external trigger
		disable_watchdogInterrupt();
		enable_ExternalInterrupt();
		power_down(); // Go to sleep
	}

	//Sets the watchdog timer to wake us up, but not reset
	//0=16ms, 1=32ms, 2=64ms, 3=125ms, 4=250ms, 5=500ms
	//6=1sec, 7=2sec, 8=4sec, 9=8sec
	void sleep_wdt(uint8_t timerPrescaler = 6)
	{
		disable_ExternalInterrupt();
		enable_watchdogInterrupt(timerPrescaler);
		power_down(); // Go to sleep
	}

	void sleep_ms(uint32_t time_ms)
	{
		int32_t _time_ms = time_ms;
		int8_t ps_Idx = 9;

		disable_ExternalInterrupt();
		while (_time_ms > 0)
		{
			// get the prescalar
			ps_Idx = 9;
			while (((_time_ms - WDT_TIME_ms[ps_Idx]) > 0) && (ps_Idx >= 0))
			{
				--ps_Idx;
			}

			// break once we have slept for sufficient time
			if (ps_Idx < 0)
			{
				break;
			}

			// enable the watchdog timer and sleep
			enable_watchdogInterrupt(ps_Idx);
			power_down(); // Go to sleep

			_time_ms -= WDT_TIME_ms[ps_Idx];
		}
	}

	//Sets the watchdog timer to wake us up, but not reset
	//0=16ms, 1=32ms, 2=64ms, 3=125ms, 4=250ms, 5=500ms
	//6=1sec, 7=2sec, 8=4sec, 9=8sec
	void sleep_wdt_or_int0(uint8_t timerPrescaler = 6)
	{
		enable_ExternalInterrupt();
		enable_watchdogInterrupt(timerPrescaler);
		power_down(); // Go to sleep
	}

private:
	void disable_peripherals()
	{
		ADCSRA &= ~(1 << ADEN); //Disable ADC
		ACSR = (1 << ACD); //Disable the analog comparator

#if defined(__AVR_ATtiny25__) | defined(__AVR_ATtiny45__) | defined(__AVR_ATtiny85__)
		DIDR0 = 0x3F; //Disable digital input buffers on all ADC0-3 pins and disable digital input buffer on AIN0,1
		/*
		power_timer0_disable(); //Needed for delay_ms
		power_timer1_disable();
		power_adc_disable();
		power_usi_disable();
		*/
#elif defined(__AVR_ATmega16U4__) | defined(__AVR_ATmega32U4__)
		DIDR0 = 0xF3; //Disable digital input buffers on all ADC0,1,4-7 pins
		DIDR1 = (1 << AIN0D); //Disable digital input buffer on AIN0
		DIDR2 = 0x3F; //Disable digital input buffers on all ADC8-13 pins
		/*
		power_twi_disable();
		power_spi_disable();
		power_usart0_disable();
		power_timer0_disable(); //Needed for delay_ms
		power_timer1_disable();
		power_timer2_disable(); //Needed for asynchronous 32kHz operation
		*/
#elif defined(__AVR_ATmega168__) | defined(__AVR_ATmega168P__) | defined(__AVR_ATmega328P__)
		DIDR0 = 0x3F; //Disable digital input buffers on all ADC0-5 pins
		DIDR1 = (1 << AIN1D) | (1 << AIN0D); //Disable digital input buffer on AIN0,1
		/*
		power_twi_disable();
		power_spi_disable();
		power_usart0_disable();
		power_timer0_disable(); //Needed for delay_ms
		power_timer1_disable();
		power_timer2_disable(); //Needed for asynchronous 32kHz operation
		*/
#endif

		// disable all peripherals
		power_all_disable();
	}

	void power_down()
	{
		WDTCSR &= ~_BV(WDIF); // Clear the watchdog timer interrupt flag
		EIFR = 0;// &= ~_BV(INTF0); // Clear the external interrupt flag

		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			sleep_enable();
			sleep_bod_disable(); // Doesn't save any power. I assume it uses the fuze settings, which have BOD disabled
		}
		sleep_cpu();
		sleep_disable();
	}

	//Sets the watchdog timer to wake us up, but not reset
	//0=16ms, 1=32ms, 2=64ms, 3=125ms, 4=250ms, 5=500ms
	//6=1sec, 7=2sec, 8=4sec, 9=8sec
	void enable_watchdogInterrupt(uint8_t timerPrescaler)
	{
		// WDTCSR = [  WDIF |  WDIE |  WDP3 |  WDCE |  WDE  |  WDP2 |  WDP1 |  WDP0 ]
		uint8_t WDTCSR_ = (timerPrescaler & 0x07); // reset the watchdog interrupt flag WDIF
		WDTCSR_ |= (timerPrescaler > 7) ? _BV(WDP3) : 0x00; // Set WDP3 if prescalar > 7 (ie. 4.0s, 8.0s)
		WDTCSR_ |= _BV(WDIE); // Enable watchdog interrupt

		//This order of commands is important and cannot be combined (beyond what they are below)
		MCUSR &= ~_BV(WDRF); // Make sure the watch dog interrupt doesnt trigger a reset

		// timed sequence
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			WDTCSR |= _BV(WDCE) | _BV(WDE); //Set WD_change enable, set WD enable
			WDTCSR = WDTCSR_; //Set new watchdog timeout value & enable
		}

		_wdt_isEnabled = true;
	}

	void disable_watchdogInterrupt()
	{
		// WDTCSR = [  WDIF |  WDIE |  WDP3 |  WDCE |  WDE  |  WDP2 |  WDP1 |  WDP0 ]
		if (_wdt_isEnabled)
		{
			//This order of commands is important and cannot be combined (beyond what they are below)
			MCUSR &= ~_BV(WDRF); // Clear the watch dog reset

			// timed sequence
			ATOMIC_BLOCK(ATOMIC_FORCEON)
			{
				WDTCSR |= _BV(WDCE) | _BV(WDE); //Set WD_change enable, set WD enable
				WDTCSR = 0; // Disable watchdog interrupt, disable watchdog
			}

			_wdt_isEnabled = false;
		}
	}

	void enable_ExternalInterrupt()
	{
		pinMode(INT0_PIN, INPUT_PULLUP);

		if (_int0_isEnabled == false)
		{
#if defined(__AVR_ATtiny25__) | defined(__AVR_ATtiny45__) | defined(__AVR_ATtiny85__)
			// MCUCR = [  BODS |  PUD  |  SE   |  SM1  |  SM0  | BODSE | ISC01 | ISC00 ]
			//MCUCR |= _BV(ISC00); // ISC0[1,0] = [1,0] = rising edge detection
			// ISC0[1,0] = [0,1] = pin change detection
			MCUCR &= ~(_BV(ISC01) | _BV(ISC00)); // 0 : low level generates an interrupt
			//MCUCR &= ~_BV(ISC01); MCUCR |= _BV(ISC00); // 1 : logical change
			//MCUCR |= _BV(ISC01); MCUCR &= ~_BV(ISC00); // 2 : falling edge
			//MCUCR |= _BV(ISC01) | _BV(ISC00); // 3 : rising edge
			PCMSK = 0x00;

			// GIMSK = [   -   |  INT0 |  PCIE |   -   |   -   |   -   |   -   |   -   ]
			GIMSK |= _BV(INT0); // Enable External Interrupt (INT0), disable Pin Change Interrupt
		//#elif defined(__AVR_ATmega16U4__) | defined(__AVR_ATmega32U4__)
		//#elif defined(__AVR_ATmega168__) | defined(__AVR_ATmega168P__) | defined(__AVR_ATmega328P__)
#else
			PCICR = 0x00; // Disable Pin Change Interrupts
			PCMSK0 = 0x00;
#if defined (PCMSK1)
			PCMSK1 = 0x00;
#endif
#if defined (PCMSK1)
			PCMSK2 = 0x00;
#endif
			EICRA &= ~(_BV(ISC01) | _BV(ISC00)); // 0 : low level generates an interrupt
			EIMSK = 0x01; // Enable external interrupt on INT0
#endif
			// Enable the pin interrupt for the INT0 pin (Needed?)
			INT0_PCMSK = digitalPinToPCMSK(INT0_PIN);
			INT0_PCMSK_bit = _BV(digitalPinToPCMSKbit(INT0_PIN));
			*INT0_PCMSK |= INT0_PCMSK_bit;

			_int0_isEnabled = true;
		}
	}

	void disable_ExternalInterrupt()
	{
		if (_int0_isEnabled)
		{
#if defined(__AVR_ATtiny25__) | defined(__AVR_ATtiny45__) | defined(__AVR_ATtiny85__)
			PCMSK = 0x00;
			GIMSK = 0x00;
			//#elif defined(__AVR_ATmega16U4__) | defined(__AVR_ATmega32U4__)
			//#elif defined(__AVR_ATmega168__) | defined(__AVR_ATmega168P__) | defined(__AVR_ATmega328P__)
#else
			EIMSK = 0x00;
			PCICR = 0x00; // Disable Pin Change Interrupts
			PCMSK0 = 0x00;
#if defined (PCMSK1)
			PCMSK1 = 0x00;
#endif
#if defined (PCMSK1)
			PCMSK2 = 0x00;
#endif

			// Disable the pin interrupt for the INT0 pin (Needed?)
			*INT0_PCMSK &= ~INT0_PCMSK_bit;
#endif

			_int0_isEnabled = false;
		}
	}
};
