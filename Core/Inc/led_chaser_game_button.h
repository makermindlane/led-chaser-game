/*
 * led-chaser-game-button.h
 *
 *  Created on: 02-Jul-2021
 *      Author: pawan
 */

#ifndef INC_LED_CHASER_GAME_BUTTON_H_
#define INC_LED_CHASER_GAME_BUTTON_H_

#include "main.h"
#include "button_event.h"
#include "stm32f4xx_hal_gpio.h"

#define DEBOUNCE_TIME				(0.02)	// 20 ms debounce time
#define SAMPLE_FREQUENCY		(200)		// 200 Hz sample frequency, means button pin is being sampled every 5 ms.
#define MAXIMUM							(DEBOUNCE_TIME * SAMPLE_FREQUENCY)

typedef enum button_state Button_State;

enum button_state {
	BUTTON_STATE_IDLE = 0,
	BUTTON_STATE_PRESSED,
	BUTTON_STATE_RELEASED
};

typedef struct button Button;

struct button {
	uint8_t curr_state;
	uint8_t prev_state;
	Button_State state;
	uint16_t pin;
	uint8_t intgrtr;
	uint8_t output;
	GPIO_TypeDef *port;
};

void button_init(Button *const button, GPIO_TypeDef *port, uint16_t pin);

Button_State button_get_state(Button *const button);
void button_set_state(Button *const button, uint8_t state);

uint16_t button_get_pin(Button *const button);
void button_set_pin(Button *const button, uint16_t pin);

GPIO_TypeDef *button_get_port(Button *const button);
void button_set_port(Button *const button, GPIO_TypeDef *pin_port);

uint8_t button_is_pressed(Button *const button);
uint8_t button_is_released(Button *const button);

uint8_t button_is_state_changed(Button *const button);

#endif /* INC_LED_CHASER_GAME_BUTTON_H_ */
