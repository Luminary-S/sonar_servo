sonar_servo
--------
sonar servo is used for driving the sonar_array to extend the detecting angles.

## package function
When the sonar_array comes with error fitting surface, the servo will rotate to the previous angles to avoid the large angles between the window and the sonar array.

## input and output
  
### input: 
1. ros_topic: /sonar
2. angle_pre (from the absolute encoder of the servo motor)
   
### output:
1.  angle (the target of the servo motor)

## start
1. launch ```roslaunch sonar_servo sonar_servo.launch```
2. key file: ```sonar_servo.cpp```
3. adjustable parameters: controller gain and angle threshold

## hardware
1. RMD-s-50(15), co.: GYEMS
2. vcc: 12v
3. 485  

## parameter:
1. encoder/angle : $2^{12}=360^o$
2. positive direction: from up view, clockwise
3. all positions are absolute angles, single circle angle means the angle is within 0~360.
