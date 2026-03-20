# Gesture Controlled Mobile Robot
Wireless Bluetooth gesture-controlled differential drive robot with closed-loop motor speed control.

Te remote has a MPU6050 IMU that measures orientation and sends that data via HC-05(master) bluetooth module to the robots HC-05(slave) bluetooth module.The N20 motors with encoders measure the rotational speed of the wheels.The orientation recieved is converted into speeds of the right and left motors and PID control is applied to achieve these speeds.

![Alt text](https://github.com/d-send/Gesture-Controlled-Mobile-Robot/blob/4572e3aa5a5d190f64d9aa63fdeef998fde009d7/p1.jpg)

## 3D Model
![Alt text](https://github.com/d-send/Gesture-Controlled-Mobile-Robot/blob/4572e3aa5a5d190f64d9aa63fdeef998fde009d7/Fusion%20Files/Model.jpg)
