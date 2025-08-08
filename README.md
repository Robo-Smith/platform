# Platform created for a wedding ceremony:
Uses Mecanum drive to provide omni directional drivability and rotaion. Code originated and has been refined with ChatGPT prompts. This repository was created to track working versions as new functionality is added and existing code is refined. 

# Latest Prompt
I have a robot with software running on a raspberry pi 4. The Pi is Bluetooth connected to an 8bitdo lite 2 controller. The robot has mecanum wheels attached to Talon SRX speed controllers that are connected to an Arduino Nano through a USB cable. The Talon controllers are connected to the following pins on Arduino for PWM control:
Front Right	14
Rear Right	15
Rear Left	16
Front Left	17
The motors are wired so that with a positive/larger signal will rotate the robot clockwise.

Additionally there are 32 WS2811 LEDs strung around the robot with the start of the strand in the front center between the right and left wheels and wrapping around clockwise back to the start. The WS2811 color control order is green red blue. The string is connected to the Raspberry Pi GPIO physical pin 12.

Using evedev on the Pi, the inputs that I observe for the 8bitdo stick action are: 
For the left stick:
Forward is ABS_Y = 0 and reverse is ABS_Y = 254
Left is ABS_X = 0 and rights is ABS_X = 254
For the right stick:
Forward is ABS_RZ = 0 and reverse is ABS_RZ = 254
Left is ABS_Z = 0 and rights is ABS_Z = 254

Please write the Pi and Arduino software to read inputs from the 8bitdo controller to operate the motors and illuminate the LEDS to support the following actions:

At startup the operator should be able to control the platform with regular mecanum controls for drive strafe and turn, there should be some filters to limit acceleration making for smooth operations. The LEDs should indicate which motors are driving in forward and reverse with some animation by the LED's closest to each motor with forward being green and reverse being red. 

When the operator presses the left trigger button, the robot should go into a restricted operation mode, that only allows turning and only up to 75% of the regular speed. It should also make the LED's lightly pulsate a white color. Pressing the right trigger should take the robot out of restricted mode.

The x button on the 8bitdo controller should turn off and disable LEDs in whatever mode the robot is in, and pressing the y button should re-enable the LEDs. 

Include instructions for setting up the software to run on boot, and when the 8bitdo controller goes to sleep or becomes disconnected to repeatedly attempt to reconnect it. 
