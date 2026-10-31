import time
import math
import json
import sys
import board
import busio
import digitalio
import pulseio
import pwmio
import rotaryio
import supervisor

# ==============================================================================
# 1. CẤU HÌNH CHÂN (PINOUT) - ĐÁNH DẤU ĐỂ BẠN SỬA THEO ĐẤU DÂY THỰC TẾ
# Thứ tự 4 bánh khớp với C++: [0]: Left Front (LF), [1]: Right Front (RF),
#                             [2]: Left Back  (LB), [3]: Right Back  (RB)
# ==============================================================================
# --- Chân PWM của 4 Motor (Cytron Motion 2350 Pro mặc định GP8 -> GP15) ---
M_LF_A, M_LF_B = board.GP8,  board.GP9   # M1 (Đã test trong ĐA2)
M_RF_A, M_RF_B = board.GP10, board.GP11  # M2 (Sửa nếu đấu khác)
M_LB_A, M_LB_B = board.GP12, board.GP13  # M3 (Sửa nếu đấu khác)
M_RB_A, M_RB_B = board.GP14, board.GP15  # M4 (Sửa nếu đấu khác)

# --- Chân Encoder (A, B) của 4 Motor ---
ENC_LF_A, ENC_LF_B = board.GP4, board.GP5  # Enc 1 (Đã test trong ĐA2)
ENC_RF_A, ENC_RF_B = board.GP2, board.GP3  # Enc 2 (Sửa theo chân bạn cắm)
ENC_LB_A, ENC_LB_B = board.GP16, board.GP17  # Enc 3 (Sửa theo chân bạn cắm)
ENC_RB_A, ENC_RB_B = board.GP18,  board.GP19   # Enc 4 (Sửa theo chân bạn cắm)

# Đảo chiều động cơ/encoder nếu bánh bên phải gắn đối xứng bị quay ngược (1 hoặc -1)
DIR_MOTOR   = [1, 1, 1, 1]  # [LF, RF, LB, RB] -> Đổi thành -1 nếu bánh nào quay ngược
DIR_ENCODER = [1, 1, 1, 1]  # [LF, RF, LB, RB] -> Đổi thành -1 nếu count bị đếm âm khi tiến

# --- Chân MPU6050 (I2C) và HC-SR04 (Đã test trong ĐA2) ---
I2C_SCL, I2C_SDA = board.GP27, board.GP26
HCSR04_TRIG, HCSR04_ECHO = board.GP0, board.GP1

# CHỌN TRỤC XOAY YAW CỦA MPU6050 (Dành cho EKF):
# Nếu đặt MPU6050 nằm phẳng: chọn "Z".
# Nếu dựng đứng MPU6050 (như trong ảnh ĐA2 ACC_X ~ 1.0g): chọn "X" (hoặc "-X")
YAW_GYRO_AXIS = "Z"

# ==============================================================================
# 2. THÔNG SỐ CALIB & ĐIỀU KHIỂN PI (TỪ TIẾN ĐỘ ĐA2)
# ==============================================================================
COUNTS_PER_REV = 2482.47   # Số xung/vòng thực tế đã đo ở ĐA2
RAD_PER_COUNT = (2.0 * math.pi) / COUNTS_PER_REV
PWM_HZ = 1000
PWM_MAX = 0.80             # Giới hạn PWM tối đa (80%) để bảo vệ nguồn/motor

# Hệ số PI bạn đã tìm ra ở Mục II.2.c (60 RPM ~ 34% PWM -> Hệ số Feedforward KF = 0.34/60)
KF = 0.34 / 60.0           # Hệ số bù trước (giúp đáp ứng mọi tốc độ chứ không chỉ 60 RPM)
KP = 0.002
KI = 0.003

CONTROL_PERIOD = 0.033     # Chu kỳ điều khiển PI & gửi dữ liệu: ~30Hz (0.033s) khớp với ekf.yaml
HCSR04_PERIOD  = 0.100     # Chu kỳ đọc siêu âm: 10Hz (0.1s) để tránh làm chậm vòng lặp PI
HCSR04_OFFSET_CM = 1.5     # Bù sai số hụt ~1.5cm của HC-SR04 bạn đã nhận xét trong ĐA2

# ==============================================================================
# 3. KHỞI TẠO PHẦN CỨNG: 4 MOTOR & 4 ENCODER
# ==============================================================================
motors_pwm = [
    (pwmio.PWMOut(M_LF_A, frequency=PWM_HZ, duty_cycle=0), pwmio.PWMOut(M_LF_B, frequency=PWM_HZ, duty_cycle=0)),
    (pwmio.PWMOut(M_RF_A, frequency=PWM_HZ, duty_cycle=0), pwmio.PWMOut(M_RF_B, frequency=PWM_HZ, duty_cycle=0)),
    (pwmio.PWMOut(M_LB_A, frequency=PWM_HZ, duty_cycle=0), pwmio.PWMOut(M_LB_B, frequency=PWM_HZ, duty_cycle=0)),
    (pwmio.PWMOut(M_RB_A, frequency=PWM_HZ, duty_cycle=0), pwmio.PWMOut(M_RB_B, frequency=PWM_HZ, duty_cycle=0)),
]

encoders = [
    rotaryio.IncrementalEncoder(ENC_LF_A, ENC_LF_B, divisor=1),
    rotaryio.IncrementalEncoder(ENC_RF_A, ENC_RF_B, divisor=1),
    rotaryio.IncrementalEncoder(ENC_LB_A, ENC_LB_B, divisor=1),
    rotaryio.IncrementalEncoder(ENC_RB_A, ENC_RB_B, divisor=1),
]

def set_motor_pwm(idx, pwm_val):
    """Điều khiển chiều và xung PWM (-PWM_MAX đến +PWM_MAX) cho từng bánh"""
    pwm_val = pwm_val * DIR_MOTOR[idx]
    pwm_val = max(-PWM_MAX, min(PWM_MAX, pwm_val))
    duty = int(abs(pwm_val) * 65535)
    
    if pwm_val > 0.01:
        motors_pwm[idx][0].duty_cycle = duty
        motors_pwm[idx][1].duty_cycle = 0
    elif pwm_val < -0.01:
        motors_pwm[idx][0].duty_cycle = 0
        motors_pwm[idx][1].duty_cycle = duty
    else:
        motors_pwm[idx][0].duty_cycle = 0
        motors_pwm[idx][1].duty_cycle = 0

def stop_all_motors():
    for i in range(4):
        motors_pwm[i][0].duty_cycle = 0
        motors_pwm[i][1].duty_cycle = 0

# ==============================================================================
# 4. KHỞI TẠO & CALIB MPU6050 (500 MẪU NHƯ ĐA2)
# ==============================================================================
i2c = busio.I2C(I2C_SCL, I2C_SDA, frequency=400000) # Tăng bus lên 400kHz để đọc nhanh hơn
while not i2c.try_lock():
    pass
try:
    devices = i2c.scan()
finally:
    i2c.unlock()

candidates = [x for x in devices if x in (0x68, 0x69)]
mpu_address = candidates[0] if candidates else None

def read_register(register, length):
    data = bytearray(length)
    while not i2c.try_lock():
        pass
    try:
        i2c.writeto_then_readfrom(mpu_address, bytes([register]), data)
    finally:
        i2c.unlock()
    return data

def write_register(register, value):
    while not i2c.try_lock():
        pass
    try:
        i2c.writeto(mpu_address, bytes([register, value]))
    finally:
        i2c.unlock()

def signed_word(data, offset):
    value = (data[offset] << 8) | data[offset + 1]
    return value - 65536 if value >= 32768 else value

gx_offset, gy_offset, gz_offset = 0.0, 0.0, 0.0

if mpu_address is not None:
    write_register(0x6B, 0x00) # Đánh thức MPU6050
    time.sleep(0.2)
    write_register(0x1A, 0x03) # DLPF
    write_register(0x19, 9)    # Sample rate 100Hz
    write_register(0x1B, 0x00) # Gyro +/-250 deg/s
    write_register(0x1C, 0x00) # Acc +/-2g
    time.sleep(0.2)

    # Calib 500 mẫu khi khởi động
    sample_count = 500
    for _ in range(sample_count):
        data = read_register(0x3B, 14)
        gx_offset += signed_word(data, 8) / 131.0
        gy_offset += signed_word(data, 10) / 131.0
        gz_offset += signed_word(data, 12) / 131.0
        time.sleep(0.005)
    gx_offset /= sample_count
    gy_offset /= sample_count
    gz_offset /= sample_count

def read_mpu6050_rads():
    """Đọc MPU6050 và đổi vận tốc góc sang rad/s cho EKF"""
    if mpu_address is None:
        return 0.0, 0.0, 0.0, 0.0
    data = read_register(0x3B, 14)
    ax_g = signed_word(data, 0) / 16384.0
    ay_g = signed_word(data, 2) / 16384.0
    az_g = signed_word(data, 4) / 16384.0

    gx_dps = signed_word(data, 8) / 131.0 - gx_offset
    gy_dps = signed_word(data, 10) / 131.0 - gy_offset
    gz_dps = signed_word(data, 12) / 131.0 - gz_offset

    # Chọn trục Yaw thực tế dựa trên cách lắp MPU6050
    if YAW_GYRO_AXIS == "X":
        yaw_dps = gx_dps
    elif YAW_GYRO_AXIS == "-X":
        yaw_dps = -gx_dps
    elif YAW_GYRO_AXIS == "-Z":
        yaw_dps = -gz_dps
    else:
        yaw_dps = gz_dps

    # Bỏ qua nhiễu tĩnh cực nhỏ (< 0.15 deg/s) để góc Yaw của EKF không bị trôi khi xe đứng yên
    if abs(yaw_dps) < 0.15:
        yaw_dps = 0.0

    yaw_rads = math.radians(yaw_dps)
    return ax_g, ay_g, az_g, yaw_rads

# ==============================================================================
# 5. KHỞI TẠO HC-SR04 (TỐI ƯU TIMEOUT NGẮN ĐỂ KHÔNG TREO PI)
# ==============================================================================
trig = digitalio.DigitalInOut(HCSR04_TRIG)
trig.switch_to_output(value=False)
echo = pulseio.PulseIn(HCSR04_ECHO, maxlen=4, idle_state=False)
echo.pause()

last_distance_cm = 400.0

def read_hcsr04_fast():
    echo.clear()
    echo.resume()
    try:
        trig.value = True
        time.sleep(0.00002)
        trig.value = False
        started = time.monotonic()
        # Chỉ chờ tối đa 15ms (~2.5m) thay vì 100ms để không làm giật động cơ
        while len(echo) == 0:
            if time.monotonic() - started >= 0.015:
                return None
    finally:
        trig.value = False
        echo.pause()

    pulse_us = echo.popleft()
    dist_cm = (pulse_us / 58.0) + HCSR04_OFFSET_CM
    if 2.0 <= dist_cm <= 400.0:
        return round(dist_cm, 1)
    return None

# ==============================================================================
# 6. VÒNG LẶP CHÍNH: NHẬN LỆNH ROS 2 -> CHẠY PI 4 MOTOR -> GỬI JSON LÊN ROS 2
# ==============================================================================
target_rpm = [0.0, 0.0, 0.0, 0.0]   # [LF, RF, LB, RB]
i_term     = [0.0, 0.0, 0.0, 0.0]
prev_count = [enc.position * DIR_ENCODER[i] for i, enc in enumerate(encoders)]
pos_rad    = [0.0, 0.0, 0.0, 0.0]
vel_rads   = [0.0, 0.0, 0.0, 0.0]

prev_control_time = time.monotonic()
prev_hcsr04_time  = time.monotonic()
last_cmd_time     = time.monotonic()
rx_buffer = ""

try:
    while True:
        now = time.monotonic()

        # --- A. ĐỌC LỆNH VẬN TỐC TỪ ROS 2 (NON-BLOCKING) ---
        # ROS 2 gửi xuống: "CMD,v_lf,v_rf,v_lb,v_rb\n" (đơn vị: rad/s)
        while supervisor.runtime.serial_bytes_available:
            ch = sys.stdin.read(1)
            if ch == "\n":
                line = rx_buffer.strip()
                rx_buffer = ""
                if line.startswith("CMD,"):
                    parts = line.split(",")
                    if len(parts) == 5:
                        try:
                            for i in range(4):
                                cmd_rads = float(parts[i + 1])
                                # Đổi rad/s từ ROS 2 sang RPM cho bộ PI
                                target_rpm[i] = cmd_rads * 60.0 / (2.0 * math.pi)
                            last_cmd_time = now
                        except ValueError:
                            pass
            else:
                rx_buffer += ch

        # An toàn (Watchdog): Nếu quá 1.0s không nhận được lệnh từ ROS 2 thì dừng xe
        if now - last_cmd_time > 1.0:
            target_rpm = [0.0, 0.0, 0.0, 0.0]

        # --- B. ĐỌC HC-SR04 MỖI 100ms (10Hz) ---
        if now - prev_hcsr04_time >= HCSR04_PERIOD:
            prev_hcsr04_time = now
            d = read_hcsr04_fast()
            if d is not None:
                last_distance_cm = d

        # --- C. VÒNG LẶP ĐIỀU KHIỂN PI & GỬI DỮ LIỆU LÊN ROS 2 MỖI 33ms (~30Hz) ---
        dt = now - prev_control_time
        if dt >= CONTROL_PERIOD:
            prev_control_time = now

            for i in range(4):
                curr_count = encoders[i].position * DIR_ENCODER[i]
                delta_count = curr_count - prev_count[i]
                prev_count[i] = curr_count

                # Tính vị trí (rad) và vận tốc thực tế (RPM & rad/s)
                pos_rad[i]  = curr_count * RAD_PER_COUNT
                rpm_actual  = (delta_count / COUNTS_PER_REV) * (60.0 / dt)
                vel_rads[i] = rpm_actual * (2.0 * math.pi) / 60.0

                # Nếu lệnh yêu cầu dừng (target ~ 0), ngắt PWM và xóa tích phân I ngay
                if abs(target_rpm[i]) < 0.5:
                    i_term[i] = 0.0
                    set_motor_pwm(i, 0.0)
                else:
                    error = target_rpm[i] - rpm_actual
                    p_term = KP * error
                    next_i = i_term[i] + KI * error * dt
                    base_pwm = KF * target_rpm[i]  # Bù trước theo tốc độ đích

                    trial_pwm = base_pwm + p_term + next_i
                    # Chống bão hòa tích phân (Anti-windup như trong ĐA2)
                    if -PWM_MAX <= trial_pwm <= PWM_MAX:
                        i_term[i] = next_i

                    pwm_out = base_pwm + p_term + i_term[i]
                    set_motor_pwm(i, pwm_out)

            # Đọc MPU6050
            try:
                ax, ay, az, gz_rads = read_mpu6050_rads()
            except OSError:
                ax, ay, az, gz_rads = 0.0, 0.0, 0.0, 0.0

            # Đóng gói JSON gọn nhẹ gửi lên C++ (motion_2350_system.cpp)
            packet = {
                "p": [round(x, 4) for x in pos_rad],    # Position 4 bánh (rad)
                "v": [round(x, 4) for x in vel_rads],   # Velocity 4 bánh (rad/s)
                "gz": round(gz_rads, 5),                # Vận tốc góc Yaw (rad/s) cho EKF
                "d": last_distance_cm                   # Khoảng cách HC-SR04 (cm)
            }
            print(json.dumps(packet))

except Exception:
    stop_all_motors()
finally:
    stop_all_motors()
    echo.deinit()
    trig.deinit()
    i2c.deinit()