# ROS RedBoard Driver
A ROS Noetic driver for the Red Robotics RedBoard+

This is based around [Approx Engineering's Redboard Library](https://github.com/ApproxEng/RedBoard) and is based around [ROS Noetic](http://wiki.ros.org/noetic/).

Clone this repo in to your catkin/src folder and run catkin_make then source the catkin workspace as usual.

This package implements four nodes:
- Motors: Listens to the `/redboard/motors` topic for `Motor` messages and controls m0 and m1. The motor expander board will be added at a future date.
- Servos: Listens to the `/redboard/servo` topic for `Servo` messages.
- Expander: Listens to the `/redboard/expander` topic for `Servo` messages. This currently connects to an expander port with address `0x40`, connections to other expanders to be added at a future date.
- ADC: Published the values from the ADC headers to the `/redboard/adc` as `ADC` messages.

Definitions for each of the message types can be found in the `msg` folder.


You can either run this using the launchfile, customising to your needs, or run the nodes individually. Please note, the PWM expander board address is now set using the `expander_address` ROS parameter:

`roslaunch rosredboard redboard.launch`  

Thanks!
=

A big thanks to my [Patrons](https://www.patreon.com/neaveeng)!

Patrons:  
----
Angie & John Neave  
Bodger

Supporters:
----
Adam Gilmore  
Bret Colloff  
David Shrive  

Followers:
----
Alexander Gutenkunst  
Alister Perrott  
Andy Batey  
Cara S.  
Dave Booth  
Eva Blake  
John Thurmond  
Mandy Berry  
Phoenix Labs Ltd  
Shelagh Lewins
