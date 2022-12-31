/* 2022 Fall ESE519
Authors: Rongqian Chen, Junpeng Zhao, Qi Xue
==============================================================================*/

#include "main_functions.h"
#include "tusb_hid/hid_src.h"
#include "pico/stdlib.h"

enum TASKS{
  KEYBOARD=0,
  MOUSE
};
int main(int argc, char *argv[]) {
  setup();
  bool mode=0;
  bool click_flag = false; // avoid repeat detection
  // static bool mouse_drag_flag = false; // long press: dragging the mouse, short press: switch mode
  static uint32_t button_start_ms = 0;
  static uint32_t click_start_ms = 0;

  while (true) {
    tud_task(); 
    if(mode == MOUSE){
      if (click_flag == false){
        if(click_detect()){ // a click detected
          click_start_ms = board_millis();
          click_flag = true;
        }
      }
      if (board_millis() - click_start_ms > 1000){// mouse click interval time is 1000ms
        click_flag = false;
      }           
      mouse_abs_position(1); // initial_cursor_flag = 1, read the adundant data which IMU continue generating

      if (gpio_get(KEYBOARD_SWITCH_IO) == false){  // detected a signal 
        if ( board_millis() - button_start_ms > 500) { // avoid repeat detection
          sleep_ms(20); // avoid the error caused by vibration
          if (gpio_get(KEYBOARD_SWITCH_IO) == false){ // button pressed
            button_start_ms = board_millis();   
            while(true){
              mouse_abs_position(0); //  remember to add tud_task because this is a while loop
              if (gpio_get(KEYBOARD_SWITCH_IO) == true){
                if ( board_millis() - button_start_ms < 700){ // button is released and the press time is short
                mode = KEYBOARD;
                uart_puts(uart0, "switch to Keyboard mode\r\n");
                }
                break;
              }
            }
          }
        }
      }
    }
    else // keyboard mode
    {       
      char key =0;
      keyboard_loop(key);
      if(key!=0){
      char str[50];
      sprintf(str, "Key=%c\r\n",key);
      uart_puts(uart0, str);
    }
      if (gpio_get(KEYBOARD_SWITCH_IO) == false){  // detected a signal     
        if ( board_millis() - button_start_ms > 500) { // avoid repeat detection
          sleep_ms(20); // avoid the error caused by vibration
          if (gpio_get(KEYBOARD_SWITCH_IO) == false){ // button pressed
            button_start_ms = board_millis();
            mode = MOUSE;
            uart_puts(uart0, "switch to Mouse mode\r\n");
          }
        }
      }
    }
  }
}
