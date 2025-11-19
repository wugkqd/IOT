import smbus
import time
import RPi.GPIO as GPIO
import threading
import math
from pygame import mixer
import time

mixer.init()
mixer.music.load("/home/pi/Downloads/sample-3s.mp3")

running = True

def button_callback(channel):
	global running
	running = not running

power_mgmt_1 = 0x6b
accel_x_out = 0x3b
accel_y_out = 0x3d
accel_z_out = 0x3f
gyro_x_out = 0x43
gyro_y_out = 0x45
gyro_z_out = 0x47

GYRO_SCALE = 131.0
ALPHA = 0.98         

roll_angle = 0.0
pitch_angle = 0.0
last_time = time.time()

pin_button = 27
pin_LED_RED = 16
pin_LED_YELLOW = 20
pin_LED_GREEN = 19
buzzer_pin = 1


A = 15
B = 10
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin_button, GPIO.IN, pull_up_down = GPIO.PUD_DOWN) 
GPIO.setup(pin_LED_RED, GPIO.OUT)
GPIO.setup(pin_LED_YELLOW, GPIO.OUT)
GPIO.setup(pin_LED_GREEN, GPIO.OUT)
GPIO.setup(buzzer_pin, GPIO.OUT)
GPIO.add_event_detect(pin_button, GPIO.RISING, callback=button_callback, bouncetime=200)

def infinite_task():
	init_sensor()

	while True:
		if running:
			Run()
		else:
			GPIO.output(pin_LED_RED, GPIO.LOW)
			GPIO.output(pin_LED_YELLOW, GPIO.LOW)
			GPIO.output(pin_LED_GREEN, GPIO.LOW)
		time.sleep(0.01)


bus = smbus.SMBus(1)

def init_sensor():
	bus.write_byte_data(0x68, power_mgmt_1, 0)

def read_word_2c(reg):
    high = bus.read_byte_data(0x68, reg)
    low = bus.read_byte_data(0x68, reg+1)
    val = (high << 8) + low
    return val if val < 0x8000 else val - 65536

#read accel
def read_accel():
	accel_x = read_word_2c(accel_x_out)
	accel_y = read_word_2c(accel_y_out)
	accel_z = read_word_2c(accel_z_out)
	return accel_x, accel_y, accel_z

def read_gyro():
	gyro_x = read_word_2c(gyro_x_out)
	gyro_y = read_word_2c(gyro_y_out)
	gyro_z = read_word_2c(gyro_z_out)
	return gyro_x, gyro_y, gyro_z

def cal_complementary(accel_data, gyro_data):
    global roll_angle, pitch_angle, last_time

    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time

    ax, ay, az = accel_data
    gx_raw, gy_raw, gz_raw = gyro_data
    
    gx_rate = gx_raw / GYRO_SCALE
    gy_rate = gy_raw / GYRO_SCALE

    roll_accel = math.atan2(ay, math.sqrt(ax**2 + az**2)) * (180.0 / math.pi)
    pitch_accel = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * (180.0 / math.pi)

    roll_gyro = roll_angle + gx_rate * dt
    pitch_gyro = pitch_angle + gy_rate * dt

    roll_angle = ALPHA * roll_gyro + (1.0 - ALPHA) * roll_accel
    pitch_angle = ALPHA * pitch_gyro + (1.0 - ALPHA) * pitch_accel
    
    return roll_angle

def cal(accel_x, accel_y, accel_z):
	roll = math.atan2(accel_y, math.sqrt(accel_x ** 2 + accel_z ** 2)) * 180 / math.pi
	pitch = math.atan2(-accel_x, math.sqrt(accel_y ** 2 + accel_z ** 2)) * 180 / math.pi
	return pitch

def Run():
	accel_data = read_accel()
	gyro_data = read_gyro()
	accel_x, accel_y, accel_z = accel_data
	gyro_x, gyro_y, gyro_z = gyro_data
	
	print(f"Accel: X={accel_x}, Y={accel_y}, Z={accel_z}")
	print(f"Gyro Raw: X={gyro_x}, Y={gyro_y}, Z={gyro_z}")
    
	tilt = cal_complementary(accel_data, gyro_data)
    
	#abs_tilt = abs(tilt) - 40
	#abs_tilt = abs(abs_tilt)
	
	tilt = tilt + 60
	print(f"Filtered Pitch Tilt: {tilt:.2f} degrees")
	
	current_time = time.time()

	if (tilt > A):
		if start_time_high_tilt is None:
			start_time_high_tilt = current_time
		if (current_time - start_time_high_tilt) >= 5.0
			GPIO.output(pin_LED_RED, GPIO.HIGH)
			GPIO.output(pin_LED_YELLOW, GPIO.LOW)
			GPIO.output(pin_LED_GREEN, GPIO.LOW)
			GPIO.output(buzzer_pin, GPIO.HIGH)
		else:
			start_time_high_tilt = 0
			continue
	elif (tilt > B):
		GPIO.output(pin_LED_RED, GPIO.LOW)
		GPIO.output(pin_LED_YELLOW, GPIO.HIGH)
		GPIO.output(pin_LED_GREEN, GPIO.LOW)
		GPIO.output(buzzer_pin, GPIO.LOW)
	else:
		GPIO.output(pin_LED_RED, GPIO.LOW)
		GPIO.output(pin_LED_YELLOW, GPIO.LOW)
		GPIO.output(pin_LED_GREEN, GPIO.HIGH)
		GPIO.output(buzzer_pin, GPIO.LOW)

threading.Thread(target=infinite_task, daemon=True).start()

try:
    while True:
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n\nProgram Interrupted. Performing GPIO cleanup...")
    
finally:
    GPIO.cleanup()
    print("GPIO cleanup complete. Pins reset.")