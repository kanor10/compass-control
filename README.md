# compass-control

This repo contains Viam client code for making a wheeled mobile robot navigate to an array of GPS coordinates by maintaining a compass heading using a PID controller.

gps-tracking-from-array.py is a simple program that imports all of the various dependencies, sets the PID parameters and then iterates through an array of GPS coordinates and calls **gotopoint**. This program will also save the GPS points to a CSV at the end.

In order to use gps-tracking-from-array.py, you should change:

- the [array of GPS coordinates](https://github.com/chris-viam/compass-control/blob/437d096799c35394b0fcf4662268a316d8269c72/gps-tracking-from-array.py#L21-L30) to your desired coordinates
- the [credentials for connecting to your robot](https://github.com/chris-viam/compass-control/blob/437d096799c35394b0fcf4662268a316d8269c72/gps-tracking-from-array.py#L37-L38)
- the [name](https://github.com/chris-viam/compass-control/blob/437d096799c35394b0fcf4662268a316d8269c72/gps-tracking-from-array.py#L50) of the movement sensor that supports GetCompassHeading
- the [name](https://github.com/chris-viam/compass-control/blob/437d096799c35394b0fcf4662268a316d8269c72/gps-tracking-from-array.py#L53) of the movement sensor that supports GetPosition
- the [names](https://github.com/chris-viam/compass-control/blob/437d096799c35394b0fcf4662268a316d8269c72/boxbot.py#L10-L13) of the motors

This code will need to be modified significantly for other types of base (i.e Ackermann steering).

Some ideas:

Individual PID gains need to be set for other types of robot. The typical process for this is to start with zero for all 3 gains and then increase kp gradually until the gross positioning behavior starts to appear, then try applying ki (start with very low gains i.e.0.0002), then kd can help minimize overshooting. Atudy the control output value to help tune these parameters. 

For Ackermann steering - one approach would be to have a dual control scheme for setheading and gotocoord:
In setheading that 1) rotates the wheels to a preset angle, 2) uses the PID controller to adjust the power to the back wheels - such that the robot will move in an arc to obtain the desired heading, finally 3) center the wheels reading for gotocoord.
In gotocoord, use the PID controller to regulate the steering and have a preset forward motion control the forward speed. If the robot starts to deviate from the heading, the steering colum should then compensate for this. This approach will need soft limits on the steering to prevent radical adjustments to the steering. 
