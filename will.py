import pyb
import time
from pyb import Pin, Timer, ADC, DAC, LED ,Switch , millis
from array import array			# need this for memory allocation to buffers
from oled_938 import OLED_938	# Use OLED display driver
from MPU6050 import MPU6050
import micropython


"""
#  The following two lines are needed by micropython
#   ... must include if you use interrupt in your program
import micropython
micropython.alloc_emergency_exception_buf(100)
"""
import micropython
micropython.alloc_emergency_exception_buf(100)

#Setting up the two motors
A1 = Pin('X3', Pin.OUT_PP)		# Control direction of motor A
A2 = Pin('X4', Pin.OUT_PP)
PWMA = Pin('X1')				# Control speed of motor A
B1 = Pin('X7', Pin.OUT_PP)		# Control direction of motor B
B2 = Pin('X8', Pin.OUT_PP)
PWMB = Pin('X2')				# Control speed of motor B

tim = Timer(2, freq = 1000)
motorA = tim.channel (1, Timer.PWM, pin = PWMA)
motorB = tim.channel (2, Timer.PWM, pin = PWMB)

#setting up ADC
pot = ADC(Pin("X11"))

# I2C connected to Y9, Y10 (I2C bus 2) and Y11 is reset low active
oled = OLED_938(pinout={'sda': 'Y10', 'scl': 'Y9', 'res': 'Y8'}, height=64,
                   external_vcc=False, i2c_devid=61)
oled.poweron()
oled.init_display()
oled.draw_text(0,0, "Wills Code")
oled.display()

# define ports for microphone, LEDs and trigger out (X5)
mic = ADC(Pin('Y11'))
MIC_OFFSET = 1523		# ADC reading of microphone for silence
dac = pyb.DAC(1, bits=12)  # Output voltage on X5 (BNC) for debugging
b_LED = LED(4)		# flash for beats on blue LED



def A_forward(value):
	A1.low()
	A2.high()
	motorA.pulse_width_percent(value)

def A_back(value):
	A2.low()
	A1.high()
	motorA.pulse_width_percent(value)

def A_stop():
	A1.high()
	A2.high()

def B_forward(value):
	B2.low()
	B1.high()
	motorB.pulse_width_percent(value)

def B_back(value):
	B1.low()
	B2.high()
	motorB.pulse_width_percent(value)

def B_stop():
	B1.high()
	B2.high()
def A_forward(value):
	A1.low()
	A2.high()
	motorA.pulse_width_percent(value)

def A_back(value):
	A2.low()
	A1.high()
	motorA.pulse_width_percent(value)

def A_stop():
	A1.high()
	A2.high()

def B_forward(value):
	B2.low()
	B1.high()
	motorB.pulse_width_percent(value)

def B_back(value):
	B1.low()
	B2.high()
	motorB.pulse_width_percent(value)

def B_stop():
	B1.high()
	B2.high()


trigger = pyb.Switch()
scale = 2.0

while not trigger():
	time.sleep(0.001)
	Kp = pot.read()*8.0 / 4095
	oled.draw_text(0,30 , 'kp = {:5.2f}'.format(Kp))
	oled.display()
while trigger():
	pass

while not trigger():
	time.sleep(0.001)
	Ki = pot.read()*scale / 4095
	oled.draw_text(0,30 , 'ki = {:5.2f}'.format(Ki))
	oled.display()
while trigger():
	pass

while not trigger():
	time.sleep(0.001)
	Kd = pot.read()*scale / 4095
	oled.draw_text(0,30 , 'kd = {:5.2f}'.format(Kd))
	oled.display()
while trigger():
	pass

oled.draw_text(0,30 , 'Trying to balance 2')
oled.display()

alpha = 0.95
pitch = 0
e_int = 0
e_diff = 0
pwm = 0
target = -6
imu = MPU6050(1,False)


# def updatePitch(pitch,dt,alpha):
# 	theta = imu.pitch()
# 	pitch_dot = imu.get_gy()
# 	pitch = alpha*(pitch + pitch_dot*dt*0.00001) + (1-alpha)*theta
# 	return(pitch, pitch_dot)


tic1 = pyb.micros()

while True:
    dt = pyb.micros() - tic1
    if dt > 5000:
        theta = imu.pitch()
        pitch_dot = imu.get_gy()
        pitch = alpha*(pitch + pitch_dot*dt*0.00001) + (1-alpha)*theta
        error = target - pitch
        pwm = (Kp*error + Ki*e_int + Kd*pitch_dot)
        e_int += error * 0.1
        if pwm > 0:
            A_forward(pwm)
            B_forward(pwm)
        if pwm < 0:
            A_back(-pwm)
            B_back(-pwm)

        tic1 = pyb.micros()
        e_diff = error
