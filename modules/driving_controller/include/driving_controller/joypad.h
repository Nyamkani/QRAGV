 


#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/joystick.h>



#ifndef KSSBOT_HARDWARE__JOYPAD_H_
#define KSSBOT_HARDWARE__JOYPAD_H_


/**
 * Current state of an axis.
 */
struct axis_state {
    short x, y;
};



int read_event(int fd, struct js_event *event);
size_t get_axis_count(int fd);
size_t get_button_count(int fd);
size_t get_axis_state(struct js_event *event, struct axis_state axes[3]);

#endif