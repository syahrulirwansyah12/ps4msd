# State message of joystick DS4 using USB serial
std_msgs/Header header

# Left: 0, Right: 15
uint8 left_analog_x
# Up: 0, Down: 15
uint8 left_analog_y
uint8 right_analog_x
uint8 right_analog_y

# Released: 0, Pressed: 15
uint8 l2_analog
# Released: 0, Pressed: 15
uint8 r2_analog

# Released: 0, Pressed: 1
bool button_triangle
bool button_circle
bool button_cross
bool button_square
bool button_r1
bool button_r3
bool button_up
bool button_right
bool button_down
bool button_left
bool button_l1
bool button_l3

