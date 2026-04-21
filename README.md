# C++ HAL

This project also includes a C++ Based HAL for the Pico, abstracting away some of the complexities of managing GPIO and PWM pins in the Pico C SDK.

Allowing for a very simplifed usecase of IRQ handlers for GPIO pins, Reduced work of initlizing and tracking PWM pins (pins not slices), and uses a OOP approach on what a Pin is. 

# ROS2 

This Uses USB Uart communication from a RPi5 to control motors and send out encoder information and IMU data collected by the Pico. 
