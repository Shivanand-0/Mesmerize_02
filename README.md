<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
</head>
<body>
    <h1>Mesmerize_02</h1>
<p>
A QTR sensor based Line follower made for TechFest of IIT Bombay.
The bot uses QTR sensors to detect the line, and then uses
PID algorithm to follow the line
</p>


<h4>Pins declarations
    <br>
</h4>
<p>QTR Sensors (From left to right of array): 13, 12, 14, 27, 26, 25, 33, 32 (pins of esp32)</p>

<h4>Libraries used: 
</h4>
<p>QtrSensor.h (by polulu)</p>

<h3>Algorithm Description for Line-Following Bot:</h3>
<h4>Sensor Setup & Calibration:</h4>
<p>
Initialize an 8-sensor array using the QTR sensors.
The sensors are calibrated by moving the bot along the line for 10 seconds during the setup phase. <br>

</p>
<h4>Motor Initialization:</h4>

Two motors are controlled using the TB6612FNG motor driver.
Motor pins are initialized, allowing forward and backward movement.

<h4> Buttons for Start and Calibration:</h4>
A button is assigned to start calibration, and another is used to start/stop the robot.
The robot won’t start unless it is calibrated and the start button is pressed.

<h4> PID Control Mechanism:</h4>

The PID control algorithm uses three terms (Proportional, Integral, and Derivative) to compute an optimal correction for motor speeds based on the sensor readings.
The error is calculated as the deviation from the ideal line position.
The correction adjusts the motor speeds to keep the bot centered on the line.

<h4>Movement Commands:</h4>

Depending on the PID output, the bot adjusts its motor speeds to either move straight or turn left/right when deviations are detected.
When the bot loses the line completely, it rotates in the direction it last sensed the line.

The sensor values are converted  (0 or 1000) based on a threshold for easier interpretation of line detection.

<h4> Open-ended PID:</h4>

If the bot leaves the line, it continues rotating until it realigns with the line.
Forward Brake:

A helper function that controls motor direction and speed based on the inputs from the PID control.
</p>
<h2>phases:</h2>
<h4>1) Line-Follower</h4>
<p>
  for line follower i have called only pid_control function.
</p>
<img src='https://github.com/Shivanand-0/Mesmerize_01/blob/main/Resource/Line-Follower-img.jpeg' style="height:300px; width: 400px;">
<h4>1) Maze-Solver</h4>
<p>
  for Maze-solver. i have used <b>SRB(Left-Straight-Right-Back) and RSLB(Right-Straight-Left-Back) priority Algorithm </b>
</p>
<img src='https://github.com/Shivanand-0/Mesmerize_01/blob/main/Resource/maze-Follower_img.png' style="height:300px; width: 400px;">

<h4>Meshmerize Bot</h4>
<img src='https://github.com/Shivanand-0/Mesmerize_01/blob/main/Resource/meshmerize-bot-img.png' style="height:300px; width: 400px;">


</body>
</html>
