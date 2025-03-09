# Ball Tracking Car with Raspberry Pi and Arduino

This project demonstrates a robotic car that detects a ball using TensorFlow Lite on a Raspberry Pi and controls the car's movement using a PID controller on an Arduino Uno. The goal is to detect the ball in real time, center it within the camera frame, and drive toward it autonomously.

#  Project Overview

A Raspberry Pi runs a real-time object detection model using TensorFlow Lite.  
The camera captures frames, and the TF-Lite model detects the ball's position (center X).  
The Raspberry Pi sends the ball’s X-coordinate to the Arduino.   
A PID controller calculates the error between the ball’s position and the center of the camera frame.  
The Arduino adjusts the motor speeds to align the car with the ball’s position.  
If the ball is aligned, the car moves forward toward the ball.  
An ultrasonic sensor is used to prevent collisions by stopping the car if an object is close.  
