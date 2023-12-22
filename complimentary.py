#################################################################################
# complimentary.py 12/21/23 Lesson 48  MPU6050                                  #
# complimentary filter for degrees pitch, roll from accelerometer and gyro      #
# yaw calculated from gyro only;  accurate in short term only due to drift      #
#################################################################################

from imu import MPU6050
from machine import Pin, I2C
from math import atan2, pi
import time

dt = 0  #time secs for each loop thru sensor data
cnt = 0 #counter for printing/plotting pitch, roll 
confidence = .95  #complimentary pitch, roll filter; gyro vs accelerometer
pitch, roll, yaw = 0, 0, 0  #in degrees, complimentary filter for pitch, roll


def calibrate_sensor() ->Tuple(float, float, float, float, float, float):
    '''calibrate accelerometer and gyro with sensor still and on flat surface'''
    print(f'sensor calibrating...keep sensor still and on flat surface...')
    sum_a_x, sum_a_y, sum_a_z, sum_g_x, sum_g_y, sum_g_z = 0,0,0,0,0,0  # totals for  sensor axes to calculate offsets
    for i in range(100):
        sum_a_x += (a_x:=imu.accel.x)
        sum_a_y += (a_y:=imu.accel.y)
        sum_a_z += (a_z:=imu.accel.z)
        sum_g_x += (g_x:=imu.gyro.x)
        sum_g_y += (g_y:=imu.gyro.y)
        sum_g_z += (g_z:=imu.gyro.z)
        time.sleep(0.01)
    offset_a_x = -(sum_a_x/100)
    offset_a_y = -(sum_a_y/100)
    offset_a_z =  1-(sum_a_z/100)
    offset_g_x = -(sum_g_x/100)
    offset_g_y = -(sum_g_y/100)
    offset_g_z = -(sum_g_z/100)
    print(f'calibration complete...\n\n')
    return offset_a_x, offset_a_y, offset_a_z, offset_g_x, offset_g_y, offset_g_z


def calc_pitch_roll_from_accel() ->Tuple(float, float):
    '''pitch_accel, roll_accel calculated from accelerometer'''
    pitch_accel = atan2(a_y, a_z) * 180 / pi
    roll_accel = atan2( a_x, a_z) * 180 / pi
    return pitch_accel, roll_accel
 
 
i2c = I2C(0, sda = Pin(0), scl = Pin(1))
imu = MPU6050(i2c)
offset_a_x, offset_a_y, offset_a_z, offset_g_x, offset_g_y, offset_g_z = calibrate_sensor()

while True:
    start_time = time.ticks_ms()
    a_x, a_y, a_z = imu.accel.xyz
    g_x, g_y, g_z = imu.gyro.xyz
    a_x += offset_a_x  #apply calibration offsets
    a_y += offset_a_y
    a_z += offset_a_z
    g_x += offset_g_x
    g_y += offset_g_y
    g_z += offset_g_z
    pitch_accel, roll_accel =  calc_pitch_roll_from_accel() #pitch, roll in degrees from accelerometer
    pitch = (pitch + g_x*dt) * confidence  + (pitch_accel)* (1-confidence)  #complimentary filter, degrees
    roll =  (roll +  g_y*dt) * confidence  + (roll_accel) * (1-confidence)  #complimentary filter, degrees
    yaw = yaw + g_z*dt  # can only calc yaw from gyro, not accelerometer; short term accuracy only
    cnt +=1
    if not cnt % 10 :  #print every 10th sensor reading
        print(f'{pitch=:.1f}, {roll=:.1f}, {yaw=:.1f}')  
    time.sleep(.005)
    dt = time.ticks_diff(start_time, now := time.ticks_ms())/1000  #time in secs for each loop
    
