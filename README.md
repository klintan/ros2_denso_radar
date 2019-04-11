# Toyota Radar driver

ROS2 driver for Denso Toyota/Lexus radar.

Adapted from Faraz Khan https://github.com/frk2/opencaret. 

All hard work is done by him and Comma.ai. 

The reason for this repo is that I wanted to be able to have a stand-alone ROS2 driver for this driver. Opencaret is of this date in ROS1 instead.


## How do I wire up this radar?
Heres the pinout to get you started:
![Radar pinout](/images/connector.jpg "Radar connector")

|Pin| Usage|
|---|------|
|3|Car can High|
|2|Car can Low|
|5|Radar can High|
|6|Radar can Low|
|8|12V VCC|
|1|GND|

## Some CAN help please?
I have used these two CAN adaptors successfully:

- Carloop with CAN Hitch: https://www.amazon.com/Carloop-CAN-Hitch-Particle-microcontroller/dp/B06XXRBVFW/ref=sr_1_1?ie=UTF8&qid=1524621189&sr=8-1&keywords=carloop+can+hitch

You'll need to buy a particle photon and flash it with the SocketCAN application they have for it to work with Socketcan on linux/mac

- http://canable.io/ - The default SL-CAN implementation sucks! (it errors out my socket can interface after a while). You'll need to reflash the STM with the new candlelight_fw which  used the gs_usb driver so it drivers without socketcan. In this mode this adaptor is rock solid.

You can also use comma.ai's Panda if you can hunt down the can pins on the OBD connector

And thats it!