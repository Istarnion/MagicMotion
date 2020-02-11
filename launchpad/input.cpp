#include "input.h"

#include <SDL.h>

InputState current_state;

// Keyboard mapping:
#define KEY_FORWARD SDL_SCANCODE_W
#define KEY_BACK    SDL_SCANCODE_S
#define KEY_UP      SDL_SCANCODE_E
#define KEY_DOWN    SDL_SCANCODE_Q
#define KEY_LEFT    SDL_SCANCODE_A
#define KEY_RIGHT   SDL_SCANCODE_D
#define KEY_CANCEL  SDL_SCANCODE_ESCAPE
#define KEY_SHIFT   SDL_SCANCODE_LSHIFT

void
InputNewFrame()
{
    current_state.mouse_delta = MakeV3(0, 0, 0);
    current_state.mouse_scroll = 0;

    if(current_state.forward == PRESSED) current_state.forward = HELD;
    if(current_state.back == PRESSED) current_state.back = HELD;
    if(current_state.up == PRESSED) current_state.up = HELD;
    if(current_state.down == PRESSED) current_state.down = HELD;
    if(current_state.left == PRESSED) current_state.left = HELD;
    if(current_state.right == PRESSED) current_state.right = HELD;
    if(current_state.cancel == PRESSED) current_state.cancel = HELD;
    if(current_state.shift == PRESSED) current_state.shift = HELD;
    if(current_state.left_mouse_button == PRESSED) current_state.left_mouse_button = HELD;
    if(current_state.middle_mouse_button == PRESSED) current_state.middle_mouse_button = HELD;
    if(current_state.right_mouse_button == PRESSED) current_state.right_mouse_button = HELD;
}

InputState *
Input()
{
    return &current_state;
}

void
InputKeyEvent(bool key_down, int sdl_key_scan_code)
{
    ButtonState state = (key_down ? PRESSED : RELEASED);
    ButtonState *key_state;

    switch(sdl_key_scan_code)
    {
        case KEY_FORWARD:
            key_state = &current_state.forward;
            break;
        case KEY_BACK:
            key_state = &current_state.back;
            break;
        case KEY_UP:
            key_state = &current_state.up;
            break;
        case KEY_DOWN:
            key_state = &current_state.down;
            break;
        case KEY_LEFT:
            key_state = &current_state.left;
            break;
        case KEY_RIGHT:
            key_state = &current_state.right;
            break;
        case KEY_CANCEL:
            key_state = &current_state.cancel;
            break;
        case KEY_SHIFT:
            key_state = &current_state.shift;
            break;
        default:
            key_state = NULL;
            break;
    }

    if(key_state)
    {
        *key_state = state;
    }
}

void
InputMouseMotion(float pos_x, float pos_y, float rel_x, float rel_y)
{
    current_state.mouse_pos = MakeV3(pos_x, pos_y, 0);
    current_state.mouse_delta = MakeV3(rel_x, rel_y, 0);
}

void
InputMousePress(bool button_down, int button)
{
    ButtonState state = (button_down ? PRESSED : RELEASED);
    ButtonState *button_state;

    switch(button)
    {
        case 1:
            button_state = &current_state.left_mouse_button;
            break;
        case 2:
            button_state = &current_state.middle_mouse_button;
            break;
        case 3:
            button_state = &current_state.right_mouse_button;
            break;
        default:
            button_state = NULL;
            fprintf(stderr, "Unsupported butten pressed: %d\n", button);
            break;
    }

    if(button_state)
    {
        if(state == PRESSED && !!(*button_state))
        {
            *button_state = HELD;
        }
        else
        {
            *button_state = state;
        }
    }
}

void
InputMouseWheel(int scroll)
{
    current_state.mouse_scroll = scroll;
}

