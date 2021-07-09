#include "button_event.h"
#include "FreeRTOS.h"
#include "main.h"


void button_event_init(Button_Event *const button_event, uint32_t id, uint8_t state) {
	button_event_set_id(button_event, id);
	button_event_set_state(button_event, state);
}

void button_event_set_id(Button_Event *const button_event, uint32_t id) {
	configASSERT(button_event != NULL);
	if (button_event_get_id(button_event) != id) {
		button_event->id = id;
	}
}

uint32_t button_event_get_id(Button_Event *const button_event) {
	configASSERT(button_event != NULL);
	return button_event->id;
}

void button_event_set_state(Button_Event *const button_event, uint8_t state) {
	configASSERT(button_event != NULL);
	button_event->state = state;
}

uint8_t button_event_get_state(Button_Event *const button_event) {
	configASSERT(button_event != NULL);
	return button_event->state;
}
