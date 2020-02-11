#ifndef INPUT_H_
#define INPUT_H_

#include "magic_math.h"

typedef enum
{
    RELEASED,
    PRESSED,
    HELD
} ButtonState;

typedef struct
{
    ButtonState forward, back, up, down, left, right, cancel;
    ButtonState left_mouse_button;
    ButtonState middle_mouse_button;
    ButtonState right_mouse_button;
    ButtonState shift;
    V3 mouse_pos;
    V3 mouse_delta;
    int mouse_scroll;
} InputState;

InputState *Input(); // Get input state

void InputNewFrame();

// Functions to inform the input system about events.
void InputKeyEvent(bool key_down, int sdl_key_scan_code);
void InputMouseMotion(float pos_x, float pos_y, float rel_x, float rel_y);
void InputMousePress(bool button_down, int button);
void InputMouseWheel(int scroll);

#endif /* end of include guard: INPUT_H_ */

