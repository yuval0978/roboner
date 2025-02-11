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


SP = None
gyro = None
PV = None
eroor = None
Kp = 1.5
KI = 0.4
Kd = 0.1
def gyroturn(SP, times):
    global  gyro, PV, eroor, speedR, speedL
    
    for count in range(times):
        PV = gyro_sensor_in5.angle
        eroor = SP - PV
        if eroor > 100:
            eroor = 100
        if eroor < -100:
            eroor = -100
        speedL = eroor
        speedR = -eroor
        tank_drive.on(speedL,speedR)
    
    print('gyroturn_finished')
    print('angle =',PV)
    gyro_sensor_in5.reset()
    print('gyro has been reseted')

def steer(ang):
    global PV, eroor, speedR, speedL
    
    defspeed = 50
    
    
    while ultrasonic_sensor_in2.distance_centimeters > 20:
        current_time = time.time() * 1_000_000  # Convert to microseconds

        PV = gyro_sensor_in5.angle
        eroor = ang - PV
        cum_error += error * current_time
        out = Kp * eroor + KI * cum_error 
        if eroor > 100:
            eroor = 100
        if eroor < -100:
            eroor = -100
        speedL = defspeed +out
        speedR = defspeed -out
        tank_drive.on(speedL,speedR)
    else:
        # Stop motors if too close to an object
        tank_drive.off(brake=True)
    print('angle =', PV)

def go_us(SP, times):
    global PV, eroor
    cum_error = 0 
    last_error = 0
    previous_time = time.time() 
    for count in range(times):
        current_time = time.time()
        elapsed_time = (current_time - previous_time) / 1000000.0
        PV = ultrasonic_sensor_in2.distance_centimeters
        eroor = PV - SP
        cum_error += eroor * elapsed_time
        rate_error = (eroor - last_error)/elapsed_time
        out = Kp*eroor + KI * cum_error + Kd* rate_error
        if out > 100:
            out = 100
        if out < -100:
            out = -100
        tank_drive.on(out, out)
        previous_time = current_time
        last_error = eroor
    print('finished')
    tank_drive.on(0,0)
    

def gyroturn_left(SP, times):
    global  gyro, PV, eroor, speedR, speedL
    cum_error = 0 
    last_error = 0
    previous_time = time.time() 
    for count in range(times):
        current_time = time.time()
        elapsed_time = (current_time - previous_time) / 1000000.0
        PV = ultrasonic_sensor_in2.distance_centimeters
        eroor = PV - SP
        cum_error += eroor * elapsed_time
        rate_error = (eroor - last_error)/elapsed_time
        out = Kp*eroor + KI * cum_error + Kd* rate_error
        if eroor > 100:
            eroor = 100
        if eroor < -100:
            eroor = -100
        speedL = out
        speedR = -out
        tank_drive.on(speedL,speedR)
        previous_time = current_time
        last_error = eroor
    print('gyroturn_finished')
    print('angle =',PV)
    gyro_sensor_in5.reset()
    print('gyro has been reseted')    


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
            gyroturn(90, 500)
            steer(0)
        elif direction == "left":
            gyroturn_left(-90, 500)
            steer(0)


for count in range(1):
    gyroturn_left(-200, 2000)
