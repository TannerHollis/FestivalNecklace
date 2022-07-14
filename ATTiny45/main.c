#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include "light_ws2812.h"

#define IR_TX_PIN	0
#define LED_DO_PIN	1
#define IR_RX_PIN	2
#define BTTN_PIN	3
#define PWR_EN_PIN	4

#define LEDS_NUM 10

#define TIM_BIT_0		85
#define TIM_BIT_1		170
#define IR_RX_ATTEMPTS	111	//Receive for 1 second
#define IR_TX_ATTEMPTS	5	//Transmit attempts
#define IR_TX_PAUSE		100	//Transmit pause between transmits

#define CLK_PRESCALER_NONE	0x01
#define CLK_PRESCALER_8		0x02
#define CLK_PRESCALER_64	0x03
#define CLK_PRESCALER_256	0x04
#define CLK_PRESCALER_1024	0x05

#define IDLE_TICKS_TIMEOUT	5
#define IDLE_SAVE_NEW_FRIEND_TICKS_TIMEOUT 5

#define CORRECT_COLOR_R 128
#define CORRECT_COLOR_G 255
#define CORRECT_COLOR_B 200

#define SEQUENCE_WAVE_PIXEL_TRAVEL 2

//Type defines
typedef enum {
	SEQUENCE_FADE_IN_SEQUENCE = 0,
	SEQUENCE_FADE_OUT_SEQUENCE,
	SEQUENCE_FADE_INOUT_SEQUENCE,
	SEQUENCE_FLASH,
	SEQUENCE_FADE_IN,
	SEQUENCE_FADE_OUT,
	SEQUENCE_FADE_INOUT,
	SEQUENCE_SEQUENCE,
	SEQUENCE_PULSE,
	SEQUENCE_ROTATE,
	SEQUENCE_CHASE,
	SEQUENCE_WAVE,
	SEQUENCE_RAINBOW,
	SEQUENCE_TX_ALERT,
	SEQUENCE_RX_ALERT
} Sequence_TypeDef;

typedef enum {
	EEPROM_READ = 0,
	EEPROM_WRITE
} EEPROM_Acces_TypeDef;

typedef enum {
	IR_RX = 0,
	IR_TX
}IR_TypeDef;

//LED Functions
static void set_pixel(struct cRGB* led, uint8_t r, uint8_t g, uint8_t b);
static uint8_t is_pixel_same(struct cRGB* a, struct cRGB* b);
static void pixel_correct_color(struct cRGB* a);
static uint8_t interpolate(uint8_t a, uint8_t b, uint16_t c);
static void pixel_blend(struct cRGB* a, struct cRGB* b, struct cRGB* out, uint16_t c);
static void fill_pixel_buffer(struct cRGB* leds_in, uint8_t r, uint8_t g, uint8_t b);
static void sequence_leds(Sequence_TypeDef seq, struct cRGB* leds_in);

//Control Functions
static void acc_user_data(EEPROM_Acces_TypeDef acc);
static void eeprom_access(EEPROM_Acces_TypeDef acc, uint8_t* user_data, uint8_t* startaddr, uint8_t len);
static void tx_rx_ir_data(IR_TypeDef use);
static void enable_gen_timer(uint8_t prescaler);
static void enable_tx_timer();
static void start_timer();
static void stop_timer();
static uint8_t get_bit(uint8_t x, uint8_t n);
static void set_bit(uint8_t* data, uint8_t x, uint8_t n);
static uint8_t get_bit_for_timer(uint8_t x, uint8_t n);
static void initialize_pins();
static uint8_t ir_rx_read();
static void delay(uint16_t time_ms);
static void pwr_off();

//Initialize EEPROM
uint8_t EEMEM leds_memory[] = { 240, 125, 0,
	0, 0, 0,
	0, 0, 0,
	0, 0, 0,
	0, 0, 0,
	0, 0, 0,
	0, 0, 0,
	0, 0, 0,
	0, 0, 0,
	0, 0, 0};
uint8_t EEMEM friends_cnt = 0;

//LED Data
static struct cRGB leds_mem[LEDS_NUM];
static struct cRGB leds_tmp[LEDS_NUM];
static struct cRGB led_tmp;
static struct cRGB leds_out[LEDS_NUM];
static struct cRGB leds_out_rev[LEDS_NUM];
static uint8_t friends_found;
const struct cRGB roygbiv[7] = {
			{ 0xFF, 0x00, 0x00 },
			{ 0xFF, 0x7F, 0x00 },
			{ 0xFF, 0xFF, 0x00 },
			{ 0x00, 0xFF, 0x00 },
			{ 0x00, 0x00, 0xFF },
			{ 0x4B, 0x00, 0x82 },
			{ 0x94, 0x00, 0xD3 }
};
const struct cRGB zero = { 0, 0, 0 };

//Interrupt wait flag, button-pressed flag
static uint8_t isr_wait = 0;
static uint8_t button_pressed = 0;

//IR TX/RX flags and attempts
static uint8_t ir_rx_attempts = 0;
static uint8_t ir_start_detected = 0;

//Global clock prescaler for timer start/stop functions
static uint8_t clock_prescaler;

//Global idle ticks, records every delay in main loop
uint8_t idle_ticks = 0;

//Timer overflow Interrupt routine
ISR(TIM0_OVF_vect) {
	stop_timer();
	if (ir_rx_attempts && !ir_start_detected)
		ir_rx_attempts--;
}

//Pin Change Interrupt routine
ISR(PCINT3_vect) {
	button_pressed = !get_bit(PINB, BTTN_PIN);
}

int main() {
	//Initialize pin states
	initialize_pins();
	
	//Enable interrupts
	sei();

	//Read user memory
	acc_user_data(EEPROM_READ);

	//Show current leds
	Sequence_TypeDef seq = SEQUENCE_FADE_INOUT;
	sequence_leds(seq, &leds_mem[0]);

	//Define maximum sequence value
	uint8_t seq_max;

	while (1) {
		if (button_pressed) {
			button_pressed = 0;
			idle_ticks = 0;
			delay(500);
			if (!get_bit(PINB, BTTN_PIN)) {
				sequence_leds(SEQUENCE_RX_ALERT, &leds_out[0]);
				tx_rx_ir_data(IR_RX);
			}
			else {
				sequence_leds(SEQUENCE_TX_ALERT, &leds_out[0]);
				tx_rx_ir_data(IR_TX);
			}
		}
		else {
			seq_max = (friends_found + SEQUENCE_SEQUENCE > SEQUENCE_RAINBOW) ? SEQUENCE_RAINBOW : friends_found + SEQUENCE_SEQUENCE;
			seq = (seq + 1) % (seq_max + 1);
			delay(2000);
			if (idle_ticks == IDLE_TICKS_TIMEOUT)
				pwr_off();
			else
				idle_ticks++;
		}
		sequence_leds(seq, &leds_mem[0]);
	}
}

static void send_leds(struct cRGB *leds_in, uint16_t len) {
	uint8_t i;
	for (i = 0; i < LEDS_NUM; i++) {
		leds_out_rev[i] = leds_in[LEDS_NUM - i - 1];
		pixel_correct_color(&leds_out_rev[i]);
	}
	ws2812_setleds(&leds_out_rev[0], len);
}

static void set_pixel(struct cRGB* led_in, uint8_t r, uint8_t g, uint8_t b) {
	led_in->r = r;
	led_in->g = g;
	led_in->b = b;
}

static uint8_t is_pixel_same(struct cRGB *a, struct cRGB *b) {
	return(a->r == b->r && a->g == b->g && a->b == b->b);
}

static void pixel_correct_color(struct cRGB* a) {
	a->r = interpolate(0, a->r, CORRECT_COLOR_R);
	a->g = interpolate(0, a->g, CORRECT_COLOR_G);
	a->b = interpolate(0, a->b, CORRECT_COLOR_B);
}

static void pixel_blend(struct cRGB* a, struct cRGB* b, struct cRGB* out, uint16_t c) {
	out->r = interpolate(a->r, b->r, c);
	out->g = interpolate(a->g, b->g, c);
	out->b = interpolate(a->b, b->b, c);
}

static void fill_pixel_buffer(struct cRGB* leds_in, uint8_t r, uint8_t g, uint8_t b) {
	uint8_t i;
	for (i = 0; i < LEDS_NUM; i++) {
		set_pixel(&leds_in[i], r, g, b);
	}
}

static uint8_t interpolate(uint8_t a, uint8_t b, uint16_t c) {
	uint32_t out_0 = ((uint32_t)a << 8) * (255 - (uint32_t)c);
	uint32_t out_1 = ((uint32_t)b << 8) * (uint32_t)c;
	return((uint8_t)((out_0 + out_1) >> 16));
}

static void sequence_leds(Sequence_TypeDef seq, struct cRGB *leds_in) {
	if (seq == SEQUENCE_FADE_IN_SEQUENCE || seq == SEQUENCE_FADE_OUT_SEQUENCE) {
		int8_t dim_direction;
		uint8_t i;
		uint16_t j;
		if (seq == SEQUENCE_FADE_IN_SEQUENCE) {
			dim_direction = 1;
			j = 0;
		}
		if (seq == SEQUENCE_FADE_OUT_SEQUENCE) {
			dim_direction = -1;
			j = 255;
		}
		fill_pixel_buffer(&leds_out[0], 0, 0, 0);
		for (i = 0; i < LEDS_NUM; i++) {
			do {
				if (button_pressed)
					return;
				pixel_blend(&zero, &leds_in[i], &leds_out[i], j);
				send_leds(&(leds_out[0]), LEDS_NUM * 3);
				delay(2*i);
				j += dim_direction;
			} while (j > 0 && j < 255);
		}
	}
	else if (seq == SEQUENCE_FADE_INOUT_SEQUENCE) {
		sequence_leds(SEQUENCE_FADE_IN_SEQUENCE, leds_in);
		sequence_leds(SEQUENCE_FADE_OUT_SEQUENCE, leds_in);
	}
	else if (seq == SEQUENCE_FADE_IN || seq == SEQUENCE_FADE_OUT) {
		int8_t dim_direction;
		uint8_t i;
		uint16_t j;
		if (seq == SEQUENCE_FADE_IN) {
			dim_direction = 1;
			j = 0;
		}
		if (seq == SEQUENCE_FADE_OUT) {
			dim_direction = -1;
			j = 255;
		}
		do {
			for (i = 0; i < LEDS_NUM; i++) {
				if (button_pressed)
					return;
				pixel_blend(&zero, &leds_in[i], &leds_out[i], j);
				j += dim_direction;
			}
			ws2812_setleds(&(leds_out[0]), LEDS_NUM * 3);
			delay(2);
		} while (j > 0 && j < 255);
	}
	else if (seq == SEQUENCE_FADE_INOUT) {
		sequence_leds(SEQUENCE_FADE_IN, leds_in);
		sequence_leds(SEQUENCE_FADE_OUT, leds_in);
	}
	else if (seq == SEQUENCE_SEQUENCE) {
		uint8_t i, j;
		for (j = 0; j < 5; j++) {
			fill_pixel_buffer(&leds_out[0], 0, 0, 0);
			for (i = 0; i < LEDS_NUM; i++) {
				if (button_pressed)
					return;
				leds_out[i] = leds_in[i];
				send_leds(&(leds_out[0]), LEDS_NUM * 3);
				delay(500);
			}
		}
		fill_pixel_buffer(&leds_out[0], 0, 0, 0);
		send_leds(&(leds_out[0]), LEDS_NUM * 3);
	}
	else if (seq == SEQUENCE_PULSE) {
		uint8_t i;
		for (i = 0; i < 5; i++) {
			sequence_leds(SEQUENCE_FADE_IN, leds_in);
			sequence_leds(SEQUENCE_FADE_OUT, leds_in);
		}
	}
	else if (seq == SEQUENCE_FLASH) {
		uint8_t i;
		fill_pixel_buffer(&leds_out[0], 0, 0, 0);
		for (i = 0; i < 5; i++) {
			if (button_pressed)
				return;
			send_leds(&leds_in[0], LEDS_NUM * 3);
			delay(250);
			send_leds(&leds_out[0], LEDS_NUM * 3);
			delay(500);
		}
	}
	else if (seq == SEQUENCE_ROTATE) {
		sequence_leds(SEQUENCE_FADE_IN, leds_in);
		uint8_t i, j;
		uint16_t k;
		for (i = 0; i < 5; i++) {
			for (k = 0; k < 255; k++) {
				for (j = 0; j < LEDS_NUM; j++) {
					if (button_pressed)
						return;
					pixel_blend(&leds_in[(i + j) % LEDS_NUM], &leds_in[(i + j + 1) % LEDS_NUM], &leds_out[i], k);
				}
				send_leds(&leds_out[0], LEDS_NUM * 3);
				delay(5);
			}
		}
		for (i = 0; i < LEDS_NUM; i++) {
			leds_tmp[i] = leds_out[i];
		}
		sequence_leds(SEQUENCE_FADE_OUT, &leds_tmp[0]);
	}
	else if (seq == SEQUENCE_CHASE) {
		fill_pixel_buffer(&leds_out[0], 0, 0, 0);
		uint8_t i, j, active_led, active_led_right;
		uint16_t k;
		for (i = 0; i < 5; i++) {
			for (j = 0; j < LEDS_NUM; j++) {
				active_led = (i + j) % LEDS_NUM;
				active_led_right = (i + j + 1) % LEDS_NUM;
				for (k = 0; k < 255; k++) {
					if (button_pressed)
						return;
					//Blend pixel with nearest neighbor - right and fade to/from zero
					pixel_blend(&leds_in[active_led], &leds_in[active_led_right], &leds_out[active_led], k);
					pixel_blend(&zero, &leds_out[active_led], &leds_out[active_led], 255 - k); //Finish at zero
					pixel_blend(&leds_in[active_led], &leds_in[active_led_right], &leds_out[active_led_right], k);
					pixel_blend(&zero, &leds_out[active_led_right], &leds_out[active_led_right], k); // Start from zero
					send_leds(&leds_out[0], LEDS_NUM * 3);
					delay(2);
				}
			}
		}
	}
	else if (seq == SEQUENCE_WAVE){
		uint8_t i, j, l, active_led_left_0, active_led_left_1, active_led_right_0, active_led_right_1;
		uint16_t k;
		for (i = 0; i < 5; i++) {
			for (j = 0; j < LEDS_NUM; j++) {
				for (l = 0; l < SEQUENCE_WAVE_PIXEL_TRAVEL; l++) {
					fill_pixel_buffer(&leds_out[0], 0, 0, 0);
					active_led_left_0 = (i + j + l) % LEDS_NUM;
					active_led_left_1 = (i + j + l + 1) % LEDS_NUM;
					active_led_right_0 = (i + j - l) % LEDS_NUM;
					active_led_right_1 = (i + j - l - 1) % LEDS_NUM;
					for (k = 0; k < 255; k++) {
						if (button_pressed)
							return;

						//Blend pixel with nearest neighbor - left_0
						pixel_blend(&leds_in[active_led_left_0], &leds_in[active_led_left_1], &leds_out[active_led_left_0], k);
						pixel_blend(&zero, &leds_out[active_led_left_0], &leds_out[active_led_left_0], k); //Finish at zero

						//Blend pixel with nearest neighbor - right_0
						pixel_blend(&leds_in[active_led_right_0], &leds_in[active_led_right_1], &leds_out[active_led_right_0], k);
						pixel_blend(&zero, &leds_out[active_led_right_0], &leds_out[active_led_right_1], k); //Finish at zero
						
						//On last pixel from wave travel, don't artifact the pixel
						if (l != SEQUENCE_WAVE_PIXEL_TRAVEL) {
							//Blend pixel with nearest neighbor - left_1
							pixel_blend(&leds_in[active_led_left_1], &leds_in[active_led_left_0], &leds_out[active_led_left_1], k);
							pixel_blend(&zero, &leds_out[active_led_left_1], &leds_out[active_led_left_1], k); //Start from zero

							//Blend pixel with nearest neighbor - right_1
							pixel_blend(&leds_in[active_led_right_1], &leds_in[active_led_right_0], &leds_out[active_led_right_1], k);
							pixel_blend(&zero, &leds_out[active_led_right_1], &leds_out[active_led_right_1], k); //Start from zero
						}

						send_leds(&leds_out[0], LEDS_NUM * 3);
						delay(1);
					}
				}
			}
		}
	}
	else if (seq == SEQUENCE_RAINBOW) {
		uint8_t i, j, active_led;
		for (i = 0; i < 7; i++) {
			for (j = 0; j < LEDS_NUM; j++) {
				active_led = (i + j) % LEDS_NUM;
				leds_tmp[active_led] = roygbiv[active_led];
			}
			sequence_leds(SEQUENCE_FADE_IN, &leds_tmp[0]);
			sequence_leds(SEQUENCE_WAVE, &leds_tmp[0]);
			sequence_leds(SEQUENCE_FADE_OUT, &leds_tmp[0]);
		}
	}
	else if (seq == SEQUENCE_RX_ALERT) {
		fill_pixel_buffer(&leds_out[0], 0, 120, 0);
		send_leds(&leds_out[0], LEDS_NUM * 3);
		delay(500);
		fill_pixel_buffer(&leds_out[0], 0, 20, 0);
		send_leds(&leds_out[0], LEDS_NUM * 3);
	}
	else if (seq == SEQUENCE_TX_ALERT) {
		fill_pixel_buffer(&leds_out[0], 0, 0, 120);
		send_leds(&leds_out[0], LEDS_NUM * 3);
		delay(250);
		fill_pixel_buffer(&leds_out[0], 0, 0, 20);
		send_leds(&leds_out[0], LEDS_NUM * 3);
	}
}

static void acc_user_data(EEPROM_Acces_TypeDef acc) {
	eeprom_access(acc, (uint8_t*)&leds_memory, &leds_memory, LEDS_NUM*3);
	eeprom_access(acc, (uint8_t*)&friends_found, &friends_cnt, 1);
}

static void eeprom_access(EEPROM_Acces_TypeDef acc, uint8_t* user_data, uint8_t* startaddr, uint8_t len) {
	uint8_t addr;
	for (addr = startaddr; addr < startaddr + len; addr++) {
		if (acc == EEPROM_READ)
			*user_data = eeprom_read_byte((uint8_t*)addr);
		else
			eeprom_write_byte((uint8_t*)addr, *user_data);
		user_data++;
	}
}

static void tx_rx_ir_data(IR_TypeDef use) {
	//Dim leds
	sequence_leds(SEQUENCE_FADE_OUT, &leds_out[0]);

	if (use == IR_RX) {
		enable_gen_timer(CLK_PRESCALER_1024);
		
		//Set rx attempts to defined value
		ir_rx_attempts = IR_RX_ATTEMPTS;
		
		uint8_t data_rx[3] = { 0, 0, 0 };
		uint8_t bit_rx;
		ir_start_detected = 0;
		uint8_t time_stop;
		uint8_t rise_detected, fall_detected;
		uint8_t bit_cnt, byte_cnt;
		uint8_t ir_pin_state = 0;
		for (byte_cnt = 0; byte_cnt < 3; byte_cnt++) {
			for (bit_cnt = 0; bit_cnt < 8; bit_cnt++) {
				rise_detected = 0;
				fall_detected = 0;
				time_stop = 0;

				//Start RX detection sequence
				if (!ir_start_detected) {
					while (ir_rx_attempts) {
						start_timer();
						while (!ir_pin_state && isr_wait) {
							ir_pin_state = ir_rx_read();
						}
						if (ir_pin_state) {
							TCNT0 = 0x00;
							rise_detected = 1;
							ir_start_detected = 1;
							break;
						}
					}
				}

				//Start bit receive sequence
				start_timer();
				if (ir_start_detected) {
					while (isr_wait && ir_rx_attempts) {
						ir_pin_state = ir_rx_read();
						if (ir_pin_state && !rise_detected) {
							TCNT0 = 0x00;
							rise_detected = 1;
						}
						if (!ir_pin_state && !fall_detected && rise_detected) {
							time_stop = TCNT0;
							fall_detected = 1;
						}
					}
					if (!time_stop) {
						return;
					}
					else if (time_stop > 127) {
						bit_rx = 1;
					}
					else {
						bit_rx = 0;
					}
					set_bit(&(data_rx[byte_cnt]), bit_rx, bit_cnt);
				}
			}
		}
		
		//If data is valid and no timeouts occurred
		if (ir_rx_attempts) {
			set_pixel(&led_tmp, data_rx[0], data_rx[1], data_rx[2]);
			uint8_t i;
			uint8_t friend_found = 0;
			uint8_t friend_pos;
			for (i = 1; i < LEDS_NUM; i++) {
				if (is_pixel_same(&(leds_mem[i]), &led_tmp)) {
					friend_found = 1;
					friend_pos = i;
					break;
				}
			}

			//If friend was not previously found
			if (!friend_found) {
				uint8_t flash_ticks = 0;
				uint8_t new_friend_pos = 1;
				while (flash_ticks < IDLE_SAVE_NEW_FRIEND_TICKS_TIMEOUT) {
					fill_pixel_buffer(&leds_tmp[0], 0, 0, 0);
					leds_tmp[new_friend_pos] = led_tmp;
					sequence_leds(SEQUENCE_FLASH, &leds_tmp[0]);
					if (button_pressed) {
						button_pressed = 0;
						new_friend_pos = (new_friend_pos + 1) % (LEDS_NUM - 1) + 1;
						flash_ticks = 0;
					}
					else
						flash_ticks++;
				}
				leds_mem[new_friend_pos] = led_tmp;
				fill_pixel_buffer(&leds_tmp[0], 0, 0, 0);
				leds_tmp[new_friend_pos] = leds_mem[new_friend_pos];
				sequence_leds(SEQUENCE_FADE_INOUT, &leds_tmp[0]);
				
				//Increment friends found
				friends_found++;
			}
			else { //If friend was already found
				fill_pixel_buffer(&leds_tmp[0], 0, 0, 0);
				fill_pixel_buffer(&leds_out[0], 0, 0, 0);
				uint8_t i;
				for (i = 0; i < LEDS_NUM; i++) {
					if(i != friend_pos)
						leds_tmp[i] = leds_mem[i];
				}
				for (i = 0; i < 3; i++) {
					send_leds(&leds_tmp[0], LEDS_NUM * 3);
					delay(1000);
					send_leds(&leds_out[0], LEDS_NUM * 3);
					delay(1000);
				}
			}
		}
		else { // Receive error
			fill_pixel_buffer(&leds_tmp[0], 128, 0, 0);
			sequence_leds(SEQUENCE_FLASH, &leds_tmp[0]);
		}
	}
	else if (use == IR_TX) {
		enable_tx_timer();

		uint8_t *data = (uint8_t *)&leds_mem;
		uint8_t bit_cnt, byte_cnt, tx_cnt;
		for (tx_cnt = 0; tx_cnt < IR_TX_ATTEMPTS; tx_cnt++) {
			for (byte_cnt = 0; byte_cnt < 3; byte_cnt++) {
				for (bit_cnt = 0; bit_cnt < 8; bit_cnt++) {
					OCR0A = get_bit_for_timer(*data, bit_cnt);
					start_timer();
					while (isr_wait) {
						//Do nothing...
					}
				}
				data++;
			}
			delay(IR_TX_PAUSE);
		}
	}
}

static void enable_gen_timer(uint8_t prescaler) {
	TCCR0A = 0x00; //Enable Normal mode
	TCCR0B = 0x00; //Normal timer mode, clock off
	TIMSK = _BV(TOIE0); //Enable Timer overflow interrupt
	clock_prescaler = prescaler;
	start_timer();
}

static void enable_tx_timer() {
	TCCR0A = 0x02 << 5 | 0x03; //Enable Fast PWM and Mode 7 for WGM
	TCCR0B = (0x01 << 3); //Mode 7 for WGM
	TIMSK = _BV(TOIE0); //Enable Timer overflow interrupt
	clock_prescaler = CLK_PRESCALER_1024;
	start_timer();
}

static void start_timer() {
	isr_wait = 1;
	TCCR0B |= clock_prescaler; //Enable clock source, Set prescaler to F_CPU / 1024 = 9375 Hz
}

static void stop_timer() {
	TCCR0B &= ~(clock_prescaler); //Disable clock source
	isr_wait = 0;
}

static uint8_t get_bit(uint8_t x, uint8_t n) {
	return((x & (1 << n)) >> x);
}

static void set_bit(uint8_t *data, uint8_t x, uint8_t n) {
	*data = (*data & ~(x << n)) | (x << n);
}

static uint8_t get_bit_for_timer(uint8_t x, uint8_t n) {
	if (get_bit(x, n))
		return(TIM_BIT_1);
	else
		return(TIM_BIT_0);
}

static void initialize_pins() {
	PORTB = _BV(IR_TX_PIN); //Set IR TX pin as tri-state before high
	DDRB = _BV(PWR_EN_PIN) | _BV(IR_TX_PIN) | _BV(LED_DO_PIN); //Set PWR_EN low, Set IR TX high, Set LED DO low

	GIMSK |= _BV(PCIE);		//Set pin change interrupt enable bit
	MCUCR |= _BV(ISC01);	//Set to falling edge detection
	PCMSK |= _BV(BTTN_PIN);	//Set button pin as interrupt source
}

static void pwr_off() {
	acc_user_data(EEPROM_WRITE);
	DDRB &= ~_BV(PWR_EN_PIN);
}

static uint8_t ir_rx_read() {
	return(get_bit(PINB, IR_RX_PIN));
}

/*
	Each tick is equal to:
		@ clk / 1 = (1 * 256) / 8MHz = 32us
		@ clk / 8 = (8 * 256) / 8MHz = 0.256ms
		@ clk / 64 = (64 * 256) / 8MHz = 2.048ms
		@ clk / 256 = (256 * 256) / 8MHz = 8.192ms
		@ clk / 1024 = (1024 * 256) / 8MHz = 32.768ms
*/
static void delay(uint16_t time_ms) {
	enable_gen_timer(CLK_PRESCALER_64);

	uint16_t ticks, ticks_div;
	switch (clock_prescaler) {
		case CLK_PRESCALER_NONE :
			ticks_div = (uint16_t)(8); //Fixed-point (8.8, 16bit) 0.032 representation
			break;
		case CLK_PRESCALER_8 :
			ticks_div = (uint16_t)(66); //Fixed-point (8.8, 16bit) 0.256 representation
			break;
		case CLK_PRESCALER_64 :
			ticks_div = (uint16_t)(2 << 8 | 12); //Fixed-point (8.8, 16bit) 2.048 representation
			break;
		case CLK_PRESCALER_256 :
			ticks_div = (uint16_t)(8 << 8 | 49); //Fixed-point (8.8, 16bit) 8.192 representation
			break;
		case CLK_PRESCALER_1024 :
			ticks_div = (uint16_t)(32 << 8 | 197); //Fixed-point (8.8, 16bit) 32.768 representation
			break;
		default :
			ticks_div = time_ms;
			break;
	}

	ticks = (uint16_t)((((uint32_t)time_ms << 8) / (uint32_t)ticks_div) >> 16);

	uint8_t ticks_cnt;
	for (ticks_cnt = ticks; ticks_cnt; ticks_cnt--) {
		start_timer();
		while (isr_wait) {
			//Do nothing.
		}
	}
}