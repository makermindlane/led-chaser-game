#include "led_chaser_game_button.h"
//#include "SEGGER_SYSVIEW.h"
#include "FreeRTOS.h"

static uint8_t integrator(Button *const button, uint8_t pin_state) {
//	static uint8_t intgrtr = 0;
//	static uint8_t output = 0;
//	SEGGER_SYSVIEW_PrintfTarget("input: %d", pinState);
//	SEGGER_SYSVIEW_PrintfTarget("intgrtr: %d", intgrtr);
	configASSERT(button != NULL);
	if (pin_state == 0) {
		if (button->intgrtr > 0) {
			button->intgrtr--;
		}
	} else if (button->intgrtr < MAXIMUM) {
		button->intgrtr++;
	}

	if (button->intgrtr == 0) {
		button->output = 0;
	} else if (button->intgrtr >= MAXIMUM) {
		button->intgrtr = MAXIMUM;
		button->output = 1;
	}

//	SEGGER_SYSVIEW_PrintfTarget("intgrtr: %d", intgrtr);
//	SEGGER_SYSVIEW_PrintfTarget("output: %d", output);
	return button->output;
}

void button_init(Button *const button, GPIO_TypeDef *port, uint16_t pin) {
	configASSERT(button != NULL);
	button_set_port(button, port);
	button_set_pin(button, pin);
	button->state = BUTTON_STATE_IDLE;
	button->curr_state = 0;
	button->prev_state = 0;
	button->intgrtr = 0;
	button->output = 0;
}

Button_State button_get_state(Button *const button) {
	configASSERT(button != NULL);
 	return button->state;
}

void button_set_state(Button *const button, uint8_t state) {
	configASSERT(button != NULL);
	if (state >= 0 && state <=1) {
		button->prev_state = button->curr_state;
		button->curr_state = integrator(button, state);
		if ((button->curr_state - button->prev_state) > 0) {
			button->state = BUTTON_STATE_PRESSED;
		} else if ((button->curr_state - button->prev_state) < 0) {
			button->state = BUTTON_STATE_RELEASED;
		} else {
			button->state = BUTTON_STATE_IDLE;
		}
	}
}

uint16_t button_get_pin(Button *const button) {
	configASSERT(button != NULL);
	return button->pin;
}

void button_set_pin(Button *const button, uint16_t pin) {
	configASSERT(button != NULL);
	if (IS_GPIO_PIN(pin)) {
		button->pin = pin;
	}
}

GPIO_TypeDef *button_get_port(Button *const button) {
	configASSERT(button != NULL);
	return button->port;
}

void button_set_port(Button *const button, GPIO_TypeDef *pin_port) {
	configASSERT(button != NULL);
	button->port = pin_port;
}

uint8_t button_is_pressed(Button *const button) {
	configASSERT(button != NULL);
	if (button->state == BUTTON_STATE_PRESSED) {
		return true;
	} else {
		return false;
	}
}

uint8_t button_is_released(Button *const button) {
	configASSERT(button != NULL);
	if (button->state == BUTTON_STATE_RELEASED) {
		return true;
	} else {
		return false;
	}
}

uint8_t button_is_state_changed(Button *const button) {
	configASSERT(button != NULL);
	if (button->curr_state != button->prev_state) {
		return true;
	} else {
		return false;
	}
}




