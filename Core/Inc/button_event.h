#ifndef INC_BUTTON_EVENT_H_
#define INC_BUTTON_EVENT_H_

#include "main.h"
#include "led_chaser_game_button.h"



typedef struct button_event {
	uint32_t id;
	uint8_t state;
} Button_Event;

void button_event_init(Button_Event *const button_event, uint32_t id, uint8_t state);

void button_event_set_id(Button_Event *const button_event, uint32_t id);
uint32_t button_event_get_id(Button_Event *const button_event);

void button_event_set_state(Button_Event *const button_event, uint8_t state);
uint8_t button_event_get_state(Button_Event *const button_event);

#endif /* INC_BUTTON_EVENT_H_ */
