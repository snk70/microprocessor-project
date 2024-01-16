#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <math.h>

#define MAIN_LED PORTB0
#define CALIBRATION_LED PORTB1

#define CALIBRATION_BUTTON PIND7
#define FACTORY_RESET_BUTTON PIND6

#define DEFAULT_DELAY_MS 500
#define SHORT_DELAY_MS 100

#define MAX_ADC 1023
#define DENOISE_VALUE 10

/******************** ثابت ها ********************/
const uint16_t DEFAULT_DRY_HUMIDITY = 512;
//میزان حد تفاوت میان سطح بالا و پایین رطوبت خاک
const uint16_t LIMIT_GAP = 20;
/******************** ثابت ها ********************/


/******************** متغیرها ********************/
uint8_t* eeprom_is_calibrated = (uint8_t*)2;
uint16_t* eeprom_calibrated_value = (uint16_t*)0;


uint8_t is_flashing, is_calibrating;
volatile uint16_t humidity;
uint16_t lower_humidity_limit,
	 	 upper_humidity_limit;
/******************** متغیرها ********************/


void setup();
void adc_start_conversion();
uint16_t set_upper_limit(uint16_t lower_limit);
void calibrate();
void factory_reset();


ISR(TIMER1_COMPA_vect) {
	adc_start_conversion();
}

ISR(ADC_vect) {
	if( is_calibrating && (abs(ADC - humidity) > DENOISE_VALUE) ) {
		humidity = ADC;
	} else if( !is_calibrating ) {
		humidity = ADC;
	}
}

int main(void) {
	setup();
	sei();


    while (1) {
    	if( !(PIND & _BV(CALIBRATION_BUTTON)) ) {
			//فشرده شدن دکمه کالیبره
    		calibrate();
    	}
    	if( (humidity < lower_humidity_limit) && !is_flashing ) {
			//پایین تر رفتن رطوبت از حداقل حد و روشن شدن چراغ
    		PORTB |= _BV(MAIN_LED);
    		is_flashing = 1;
    	}
    	if( (humidity > upper_humidity_limit) && is_flashing ) {
			//بالاتر رفتن رطوبت از بالاترین حد و خاموش شدن چراغ
			PORTB &= ~_BV(MAIN_LED);
    		is_flashing = 0;
    	}
    	if( !(PIND & _BV(FACTORY_RESET_BUTTON)) ) {
			//فشرده شدن دکمه ریست نرم افزاری
    		factory_reset();
    	}
    }

    return 0;
}

void setup() {
	//تنظیم متغیرهای سراسری
	is_flashing = is_calibrating = 0;
	humidity = MAX_ADC;

	if( eeprom_read_byte(eeprom_is_calibrated) == 1 ) {
		lower_humidity_limit = eeprom_read_word(eeprom_calibrated_value);
		upper_humidity_limit = set_upper_limit(lower_humidity_limit);
	} else {
		lower_humidity_limit = DEFAULT_DRY_HUMIDITY;
		upper_humidity_limit = DEFAULT_DRY_HUMIDITY + LIMIT_GAP;
	}

	//تنظیم تایمر
	TCCR1B |= _BV(WGM12);
	TCCR1B |= _BV(CS12);
	OCR1A = 15624;
	TIMSK1 |= _BV(OCIE1A);

	//تنظیم ADC
	ADMUX |= _BV(REFS0);
	ADMUX |= _BV(MUX0) | _BV(MUX2);
	ADCSRA |= _BV(ADEN) | _BV(ADIE);
	ADCSRA |= _BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2);
	//غیرفعال کردن خروجی دیجیتال در پورت ADC
	DIDR0 = _BV(ADC5D);

	//تنظیم ورودی و خروجی:
	DDRB |= _BV(MAIN_LED) | _BV(CALIBRATION_LED);	//outputs
	PORTB = 0x00;	//خاموش شدن
	DDRD = 0x00;	//inputs
	PORTD |= _BV(CALIBRATION_BUTTON) | _BV(FACTORY_RESET_BUTTON);

	//چشمک شدن سه باره
	for( uint8_t i = 0; i < 3; ++i) {
		_delay_ms(SHORT_DELAY_MS);
		PORTB |= _BV(MAIN_LED);
		_delay_ms(SHORT_DELAY_MS);
		PORTB &= ~_BV(MAIN_LED);
		_delay_ms(SHORT_DELAY_MS);
	}

	//ایجاد وقفه
	_delay_ms(DEFAULT_DELAY_MS);

	//شروع اولین بررسی رطوبت
	adc_start_conversion();
}

uint16_t set_upper_limit(uint16_t lower_limit) {
	if(lower_limit < (MAX_ADC - LIMIT_GAP)) {
		return (lower_limit + LIMIT_GAP);
	} else {
		return MAX_ADC;
	}

}

void adc_start_conversion() {
	ADCSRA |= _BV(ADSC);
}

void calibrate() {
	PORTB |= _BV(CALIBRATION_LED);

	is_calibrating = 1;

	is_flashing = 0;
	PORTB &= ~_BV(MAIN_LED);

	_delay_ms(SHORT_DELAY_MS);
	uint16_t calibrated_value = humidity + DENOISE_VALUE;

	//ذخیره مقدار روطبت در حافظه
	eeprom_update_byte(eeprom_is_calibrated, 1);
	eeprom_update_word(eeprom_calibrated_value, calibrated_value);

	//انتظار تا زمان رها کردن دکمه توسط کاربر
	while( !(PIND & _BV(CALIBRATION_BUTTON)) );

	//تنظیم بالاترین و کمترین حد رطوبت مد نظر
	lower_humidity_limit = calibrated_value;
	upper_humidity_limit = calibrated_value + LIMIT_GAP;

	_delay_ms(DEFAULT_DELAY_MS);
	PORTB &= ~_BV(CALIBRATION_LED);

	is_calibrating = 0;
}

void factory_reset() {
	//روشن شدن هر دو چراغ و ریست اطلاعات کالیبره شده
	PORTB |= _BV(MAIN_LED) | _BV(CALIBRATION_LED);

	//پاک کردن مقادیر کالیبره ذخیره شده
	eeprom_update_byte(eeprom_is_calibrated, 0);
	eeprom_update_word(eeprom_calibrated_value, 0);

	//انتظار تا زمان رها کردن دکمه توسط کاربر
	while( !(PIND & _BV(FACTORY_RESET_BUTTON)) );

	_delay_ms(DEFAULT_DELAY_MS);
	PORTB &= ~(_BV(MAIN_LED) | _BV(CALIBRATION_LED));

	//ریست کردن رجیسترهای استفاده شده
	TCCR1B = 0x00;
	TIMSK1 = 0x00;
	ADMUX = 0x00;
	ADCSRA = 0x00;
	DDRB = 0x00;

	setup();
}

