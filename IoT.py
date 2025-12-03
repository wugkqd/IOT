import smbus
import time
import RPi.GPIO as GPIO
import threading
import math

# --- 설정 변수 ---
power_mgmt_1 = 0x6b
accel_x_out = 0x3b
accel_y_out = 0x3d
accel_z_out = 0x3f
gyro_x_out = 0x43
gyro_y_out = 0x45
gyro_z_out = 0x47

GYRO_SCALE = 131.0
ALPHA = 0.98

# 임계값 설정
ROLL_LIMIT_WARNING = 2.5  # 앞뒤 기울기 경고 기준 (B)
ROLL_LIMIT_DANGER = 3.6   # 앞뒤 기울기 위험 기준 (A)
PITCH_LIMIT_WARNING = 8   # 좌우 기울기 경고 기준 (새로 추가됨)
PITCH_LIMIT_DANGER = 13    # 좌우 기울기 위험 기준 (새로 추가됨)

# 핀 설정
pin_button = 27
pin_LED_RED = 16
pin_LED_YELLOW = 20
pin_LED_GREEN = 19
buzzer_pin = 13

# 전역 변수
running = True
start_time_high_tilt = None
PITCH_angle = 0.0
ROLL_angle = 0.0
last_time = time.time()

# 캘리브레이션(영점 조절)용 변수
offset_PITCH = 0.0
offset_ROLL = 0.0
is_calibrated = False

# GPIO 초기화
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin_button, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(pin_LED_RED, GPIO.OUT)
GPIO.setup(pin_LED_YELLOW, GPIO.OUT)
GPIO.setup(pin_LED_GREEN, GPIO.OUT)
GPIO.setup(buzzer_pin, GPIO.OUT)
buzzer_pwm = GPIO.PWM(buzzer_pin, 261) 
buzzer_pwm.start(0)

def button_callback(channel):
    global running, start_time_high_tilt
    running = not running
    # 멈췄다가 다시 시작할 때 타이머 초기화
    if not running:
        start_time_high_tilt = None

GPIO.add_event_detect(pin_button, GPIO.RISING, callback=button_callback, bouncetime=200)

bus = smbus.SMBus(1)

def init_sensor():
    bus.write_byte_data(0x68, power_mgmt_1, 0)

def read_word_2c(reg):
    high = bus.read_byte_data(0x68, reg)
    low = bus.read_byte_data(0x68, reg+1)
    val = (high << 8) + low
    return val if val < 0x8000 else val - 65536

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

# 캘리브레이션 함수: 시작 시 100개 샘플을 읽어 평균을 구함
def calibrate_sensor(samples=100):
    global offset_PITCH, offset_ROLL, PITCH_angle, ROLL_angle, last_time
    
    print("캘리브레이션 중... 의자를 평평하게 유지하세요.")
    
    # [중요 수정] 1. 시작하자마자 현재 가속도 값으로 각도를 '강제 초기화' 합니다.
    # 이렇게 해야 0도에서 서서히 올라가는 현상(Lag)을 막을 수 있습니다.
    accel_data = read_accel()
    ax, ay, az = accel_data
    
    # 가속도 기반 각도 계산 (라디안 -> 도)
    PITCH_angle = math.atan2(ay, math.sqrt(ax**2 + az**2)) * (180.0 / math.pi)
    ROLL_angle = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * (180.0 / math.pi)
    
    print(f"초기 각도 강제 설정 완료: ROLL={ROLL_angle:.2f}, PITCH={PITCH_angle:.2f}")

    # 2. 값 안정화를 위해 루프를 돌며 평균을 구합니다.
    sum_PITCH = 0
    sum_ROLL = 0
    
    for i in range(samples):
        accel_data = read_accel()
        gyro_data = read_gyro()
        
        # 상보 필터 실행
        r, p = cal_complementary(accel_data, gyro_data)
        
        sum_PITCH += r
        sum_ROLL += p
        time.sleep(0.01)
        
    offset_PITCH = sum_PITCH / samples
    offset_ROLL = sum_ROLL / samples
    
    print(f"캘리브레이션 최종 완료! Offset -> PITCH: {offset_PITCH:.2f}, ROLL: {offset_ROLL:.2f}")
def cal_complementary(accel_data, gyro_data):
    global PITCH_angle, ROLL_angle, last_time

    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time

    ax, ay, az = accel_data
    gx_raw, gy_raw, gz_raw = gyro_data
    
    gx_rate = gx_raw / GYRO_SCALE
    gy_rate = gy_raw / GYRO_SCALE

    # PITCH과 ROLL 계산 (라디안 -> 도)
    PITCH_accel = math.atan2(ay, math.sqrt(ax**2 + az**2)) * (180.0 / math.pi)
    ROLL_accel = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * (180.0 / math.pi)

    PITCH_gyro = PITCH_angle + gx_rate * dt
    ROLL_gyro = ROLL_angle + gy_rate * dt

    PITCH_angle = ALPHA * PITCH_gyro + (1.0 - ALPHA) * PITCH_accel
    ROLL_angle = ALPHA * ROLL_gyro + (1.0 - ALPHA) * ROLL_accel
    
    # 두 값을 모두 반환하도록 수정 (좌우, 앞뒤 모두 필요)
    return PITCH_angle, ROLL_angle

def Run():
    global start_time_high_tilt
    
    accel_data = read_accel()
    gyro_data = read_gyro()
    
    # 1. 현재 각도 계산 (PITCH: 좌우, ROLL: 앞뒤)
    raw_PITCH, raw_ROLL = cal_complementary(accel_data, gyro_data)
    
    # 2. 영점 조절 (초기 오프셋 빼기)
    current_PITCH = raw_PITCH - offset_PITCH
    current_ROLL = raw_ROLL - offset_ROLL
    
    # 3. 절대값으로 변환 (앞으로 기울든 뒤로 기울든, 좌로 기울든 우로 기울든 기울기 크기만 중요)
    abs_PITCH = abs(current_PITCH)
    abs_ROLL = abs(current_ROLL)

    print(f"ROLL(앞뒤): {current_ROLL:.2f}, PITCH(좌우): {current_PITCH:.2f}")

    current_time = time.time()

    # 4. 판별 로직: 앞뒤(ROLL) 또는 좌우(PITCH) 중 하나라도 위험 범위를 넘으면 경고
    # 조건 A: 위험 범위 (Danger) 초과
    if abs_ROLL > ROLL_LIMIT_DANGER or abs_PITCH > PITCH_LIMIT_DANGER:
        if start_time_high_tilt is None:
            start_time_high_tilt = current_time
        
        # 5초 이상 지속되었는지 확인
        if (current_time - start_time_high_tilt) >= 5.0:
            GPIO.output(pin_LED_RED, GPIO.HIGH)
            GPIO.output(pin_LED_YELLOW, GPIO.LOW)
            GPIO.output(pin_LED_GREEN, GPIO.LOW)
            buzzer_pwm.ChangeFrequency(392)  # 392Hz (4옥타브 '솔') - 부드러운 소리
            buzzer_pwm.ChangeDutyCycle(10)   # 볼륨 10% (소리가 너무 크면 이 숫자를 줄이세요)
        else:
            # 아직 5초 안됨 -> 노란불 (주의 단계)
            GPIO.output(pin_LED_RED, GPIO.LOW)
            GPIO.output(pin_LED_YELLOW, GPIO.HIGH)
            GPIO.output(pin_LED_GREEN, GPIO.LOW)
            buzzer_pwm.ChangeDutyCycle(0)
            
# 조건 B: 경고 범위 (Warning) 초과 (위험 범위보다는 작음)
    elif abs_ROLL > ROLL_LIMIT_WARNING or abs_PITCH > PITCH_LIMIT_WARNING:
        start_time_high_tilt = None # 타이머 리셋
        # 노란불
        GPIO.output(pin_LED_RED, GPIO.LOW)
        GPIO.output(pin_LED_YELLOW, GPIO.HIGH)
        GPIO.output(pin_LED_GREEN, GPIO.LOW)
        buzzer_pwm.ChangeDutyCycle(0)
        
    # 정상 범위
    else:
        start_time_high_tilt = None # 타이머 리셋
        # 초록불
        GPIO.output(pin_LED_RED, GPIO.LOW)
        GPIO.output(pin_LED_YELLOW, GPIO.LOW)
        GPIO.output(pin_LED_GREEN, GPIO.HIGH)
        buzzer_pwm.ChangeDutyCycle(0)

def infinite_task():
    global is_calibrated
    init_sensor()
    
    # 프로그램 시작 시 최초 1회 캘리브레이션 수행
    if not is_calibrated:
        calibrate_sensor()
        is_calibrated = True

    while True:
        if running:
            Run()
        else:
            # 정지 상태일 때 LED/부저 끄기
            GPIO.output(pin_LED_RED, GPIO.LOW)
            GPIO.output(pin_LED_YELLOW, GPIO.LOW)
            GPIO.output(pin_LED_GREEN, GPIO.LOW)
            buzzer_pwm.ChangeDutyCycle(0)
            start_time_high_tilt = None
        time.sleep(0.01)

threading.Thread(target=infinite_task, daemon=True).start()

try:
    while True:
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n\n프로그램 종료. GPIO 정리 중...")
    
finally:
    buzzer_pwm.stop()
    GPIO.cleanup()
    print("GPIO 정리 완료.")