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

void
InputNewFrame()
{
    current_state.mouse_delta = MakeV3(0, 0, 0);
    current_state.mouse_scroll = 0;
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
    switch(sdl_key_scan_code)
    {
        case KEY_FORWARD:
            current_state.forward = state;
            break;
        case KEY_BACK:
            current_state.back = state;
            break;
        case KEY_UP:
            current_state.up = state;
            break;
        case KEY_DOWN:
            current_state.down = state;
            break;
        case KEY_LEFT:
            current_state.left = state;
            break;
        case KEY_RIGHT:
            current_state.right = state;
            break;
        case KEY_CANCEL:
            current_state.cancel = state;
        default:
            break;
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
    switch(button)
    {
        case 0:
            current_state.left_mouse_button = state;
            break;
        case 1:
            current_state.middle_mouse_button = state;
            break;
        case 2:
            current_state.right_mouse_button = state;
            break;
        default:
            break;
    }
}

void
InputMouseWheel(int scroll)
{
    current_state.mouse_scroll = scroll;
}

