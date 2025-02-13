#!/usr/bin/env python3

# Import the necessary libraries
import time
import math
from ev3dev2.motor import *
from ev3dev2.sound import Sound
from ev3dev2.button import Button
from ev3dev2.sensor import *
from ev3dev2.sensor.lego import *
from ev3dev2.sensor.virtual import *

# Create the sensors and motors objects

spkr = Sound()
btn = Button()
radio = Radio()



# Here is where your code starts

#!/usr/bin/env python3

# Import the necessary libraries
import time
import math
from ev3dev2.motor import *
from ev3dev2.sound import Sound
from ev3dev2.button import Button
from ev3dev2.sensor import *
from ev3dev2.sensor.lego import *
from ev3dev2.sensor.virtual import *

# Create the sensors and motors objects
motorA = LargeMotor(OUTPUT_A)
motorB = LargeMotor(OUTPUT_B)
left_motor = motorA
right_motor = motorB
tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)
steering_drive = MoveSteering(OUTPUT_A, OUTPUT_B)

spkr = Sound()
btn = Button()
radio = Radio()

color_sensor_in1 = ColorSensor(INPUT_1)
ultrasonic_sensor_in2 = UltrasonicSensor(INPUT_2)
ultrasonic_sensor_in3 = UltrasonicSensor(INPUT_3)
ultrasonic_sensor_in4 = UltrasonicSensor(INPUT_4)
gyro_sensor_in5 = GyroSensor(INPUT_5)
#gps_sensor_in4 = GPSSensor(INPUT_4)
#pen_in5 = Pen(INPUT_5)

motorC = LargeMotor(OUTPUT_C) # Magnet

# Here is where my code starts


SP = None
gyro = None
PV = None
eroor = None
Kp = 2
Ki = 0.1
Kd = 1.5 
#Kp = 3
#Ki = 0.4
#Kd = 1.5

def gyroturn(SP, times):
    global  gyro, PV, eroor, speedR, speedL
    cum_error = 0 
    last_error = 0
    previous_time = time.time() 
    
    for count in range(times):
        PV = gyro_sensor_in5.angle
        error = SP - PV  # Calculate current error (renamed from 'eroor')
        current_time = time.time()
        elapsed_time = current_time - previous_time  # In seconds
        
        cum_error += error * elapsed_time
        
        # The rate change
        if elapsed_time == 0:
            derivative = 0  # Avoid division by zero
        else:
            rate_error = (error - last_error) / elapsed_time
        
        out = Kp * error + Ki * cum_error + Kd * rate_error  # Updated variable name
        out = max(min(out, 100), -100)  # Clamp output between -100 and 100
        
        speedL = out
        speedR = -out
        
        tank_drive.on(speedL,speedR)
        previous_time = current_time
        last_error = error  # Updated variable name
    print('gyroturn_finished')
    print('angle =',PV)
    gyro_sensor_in5.reset()
    print('gyro has been reseted')

def steer(ang):
    global PV, eroor, speedR, speedL
    gyro_sensor_in5.reset()
    defspeed = 69
    cum_error = 0 
    last_error = 0
    previous_time = time.time() 
    while ultrasonic_sensor_in2.distance_centimeters > 20:
        PV = gyro_sensor_in5.angle
        error = ang - PV  # Calculate current error (renamed from 'eroor')
        current_time = time.time()
        elapsed_time = current_time - previous_time  # In seconds
        
        cum_error += error * elapsed_time
        
        # The rate change
        if elapsed_time == 0:
            derivative = 0  # Avoid division by zero
        else:
            rate_error = (error - last_error) / elapsed_time
        
        out = Kp * error + Ki * cum_error + Kd * rate_error  # Updated variable name
        out = max(min(out, 30), -30)  
        
        speedL = defspeed + out
        speedR = defspeed - out

        
        tank_drive.on(speedL,speedR)
        
        previous_time = current_time
        last_error = error  # Updated variable name
    else:
        # Stop motors if too close to an object
        tank_drive.off(brake=True)
    print('angle =', PV)


def go_us(SP, times):
    global PV, error  # Changed from 'eroor' to 'error'
    cum_error = 0 
    last_error = 0
    previous_time = time.time() 
    for count in range(times):
        PV = ultrasonic_sensor_in2.distance_centimeters
        error = PV - SP  # Calculate current error (renamed from 'eroor')
        current_time = time.time()
        elapsed_time = current_time - previous_time  # In seconds
        
        cum_error += error * elapsed_time
        
        # The rate change
        if elapsed_time == 0:
            derivative = 0  # Avoid division by zero
        else:
            rate_error = (error - last_error) / elapsed_time
        
        out = Kp * error + Ki * cum_error + Kd * rate_error  # Updated variable name
        out = max(min(out, 100), -100)  # Clamp output between -100 and 100
        tank_drive.on(out, out)
    print('finished')
    tank_drive.on(0, 0)


def check_us():
    global PV_front, PV_left, Pv_right
    PV_front = ultrasonic_sensor_in2.distance_centimeters
    PV_left = ultrasonic_sensor_in3.distance_centimeters
    PV_right = ultrasonic_sensor_in4.distance_centimeters
    if PV_front > PV_right and PV_front > PV_left:
        return("front")
    
    elif PV_right > PV_left and PV_right > PV_front:
        return("right")
    
    elif PV_left > PV_right and PV_left > PV_front:
        return("left")


def go_to_dir():
        direction = check_us()
        if direction == "front":
            pass
            steer(0)
        elif direction == "right":
            gyroturn(90, 350)
            steer(0)
        elif direction == "left":
            gyroturn(-90, 350)
            steer(0)

for count in range(100):
    go_to_dir()


