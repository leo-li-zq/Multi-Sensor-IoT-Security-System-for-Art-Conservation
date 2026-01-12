# security_app_final.py (V26 - 路径修复版)
import time
import threading
import RPi.GPIO as GPIO
import board
import busio
import io
import os
from picamera2 import Picamera2
from PIL import Image
from pyzbar.pyzbar import decode
from flask import Flask, render_template, jsonify, request, Response
import datetime
import smbus2

# --- 传感器库 ---
import adafruit_sht4x
import adafruit_veml7700
import spl06

# --- 1. 硬件引脚和配置 ---
BUZZER_PIN = 17
SMOKE_PIN = 18

# --- 2. 警报阈值 (敏感度设置 - 保持你之前的设定) ---
PRESSURE_THRESHOLD_HPA = 8.0
LIGHT_THRESHOLD_LUX = 2  # 光强阈值 2.0 (非常灵敏)
DISTANCE_THRESHOLD_MM = 30.0
TEMP_THRESHOLD_C = 3.0
HUMIDITY_THRESHOLD_PCT = 2.0

# --- VL5300 激光配置 ---
VL5300_ADDRESS = 0x6C
VL5300_REG_CMD = 0x0A
VL5300_CMD_START_SINGLE = 0x0E
VL5300_REG_READ_DATA = 0x0C
VL5300_READ_BYTE_COUNT = 32
CALIB_K = 0.862069
CALIB_B = -5.1724

# --- 3. 全局状态变量 ---
app_state = {
    "system_status": "IDLE",  # IDLE, SCANNED, ARMED
    "art_piece_id": "尚未录入",
    "art_piece_info": "暂无详细信息",
    "alarm_status": "NONE",

    # 烟雾状态 (True=安全/无烟, False=有烟)
    "smoke_safe": True,

    # 基准值
    "baseline_pressure": 0,
    "baseline_light": 0,
    "baseline_distance": 0,
    "baseline_temp": 0,
    "baseline_humidity": 0,

    # 实时值
    "current_temp_c": 0,
    "current_humidity": 0,
    "current_pressure": 0,
    "current_light": 0,
    "current_distance": 0,
}

# 历史数据列表
historical_data = []

state_lock = threading.Lock()
picam2_lock = threading.Lock()
i2c_bus_lock = threading.Lock()

# --- 4. 硬件初始化 ---
i2c_busio = None
i2c_smbus = None
sht40 = None
veml7700 = None
spl06_sensor = None
vl5300_ready = False
picam2 = None


def initialize_hardware():
    global i2c_busio, i2c_smbus, sht40, veml7700, spl06_sensor, picam2, vl5300_ready
    print("正在初始化硬件 (V26 路径修复版)...")

    # [! 修改 !] 改为相对路径，文件夹会生成在代码旁边
    if not os.path.exists("evidence"):
        os.makedirs("evidence")
        print("已确保 evidence 文件夹存在")

    try:
        i2c_busio = busio.I2C(board.SCL, board.SDA)
    except:
        return False

    try:
        i2c_smbus = smbus2.SMBus(1)
    except:
        return False

    try:
        sht40 = adafruit_sht4x.SHT4x(i2c_busio)
    except:
        pass
    try:
        veml7700 = adafruit_veml7700.VEML7700(i2c_busio)
    except:
        pass
    try:
        spl06_sensor = spl06.SPL06(i2c_smbus, address=0x76)
    except:
        pass
    try:
        with i2c_bus_lock:
            i2c_smbus.read_byte(VL5300_ADDRESS)
        vl5300_ready = True
    except:
        vl5300_ready = False

    try:
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(BUZZER_PIN, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(SMOKE_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        picam2 = Picamera2()
        # 使用 1024x768 分辨率提升扫码速度
        config = picam2.create_preview_configuration(main={"size": (1024, 768)})
        picam2.configure(config)
        picam2.start()

        # 尝试开启自动对焦
        try:
            picam2.set_controls({"AfMode": 2})
        except:
            pass

        time.sleep(1)
        print("硬件全部就绪。")
        return True
    except Exception as e:
        print(f"硬件初始化失败: {e}")
        return False


# --- 5. 功能函数 ---

def capture_evidence_thread():
    """ 警报触发后，连续拍摄 10 张照片 (10秒) """
    global picam2
    print("!!! 警报触发：开始 120秒 连续取证...")

    # [! 修改 !] 改为相对路径
    evidence_path = "evidence/"

    trigger_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    for i in range(120):
        filename = f"{evidence_path}ALERT_{trigger_time}_sec{i + 1}.jpg"
        try:
            with picam2_lock:
                picam2.capture_file(filename)
            print(f"  -> 证据保存 [{i + 1}/120]: {filename}")
        except Exception as e:
            print(f"拍照失败: {e}")
        time.sleep(1)
    print("--- 取证结束 ---")


def trigger_alarm():
    GPIO.output(BUZZER_PIN, GPIO.HIGH)
    threading.Thread(target=capture_evidence_thread).start()


def vl5300_get_raw_tof(bus):
    try:
        bus.write_byte_data(VL5300_ADDRESS, VL5300_REG_CMD, VL5300_CMD_START_SINGLE)
        time.sleep(0.05)
        data = bus.read_i2c_block_data(VL5300_ADDRESS, VL5300_REG_READ_DATA, VL5300_READ_BYTE_COUNT)
        return (data[13] << 8) | data[12]
    except:
        return None


def vl5300_raw_to_mm(raw):
    if raw is None: return 0
    val = (raw * CALIB_K) + CALIB_B
    return max(0, val)


# --- 6. 核心逻辑 API ---

def scan_qr_code():
    global app_state, picam2
    qr_data = None
    if not picam2: return False, "摄像头未启动"

    for _ in range(5):
        try:
            with picam2_lock:
                buffer = picam2.capture_array()
            decoded = decode(Image.fromarray(buffer))
            if decoded:
                qr_data = decoded[0].data.decode('utf-8')
                break
            time.sleep(0.2)
        except:
            pass

    if qr_data:
        with state_lock:
            app_state["art_piece_id"] = qr_data
            app_state["art_piece_info"] = "Project 1 认证展品"
            app_state["system_status"] = "SCANNED"
        return True, "识别成功"
    else:
        return False, "未识别到二维码"


def set_system_baseline():
    global app_state
    with state_lock:
        if app_state["current_pressure"] <= 0:
            return False, "传感器数据无效"

        # 锁定所有基准
        app_state["baseline_pressure"] = app_state["current_pressure"]
        app_state["baseline_light"] = app_state["current_light"]
        app_state["baseline_distance"] = app_state["current_distance"]
        app_state["baseline_temp"] = app_state["current_temp_c"]
        app_state["baseline_humidity"] = app_state["current_humidity"]

        app_state["system_status"] = "ARMED"
    return True, "布防成功"


def reset_system_state():
    global app_state
    GPIO.output(BUZZER_PIN, GPIO.LOW)
    with state_lock:
        app_state["system_status"] = "IDLE"
        app_state["art_piece_id"] = "尚未录入"
        app_state["alarm_status"] = "NONE"
        app_state["baseline_temp"] = 0
        app_state["baseline_humidity"] = 0
    return True


# --- 7. 监控线程 (带长时记录优化) ---
def sensor_thread():
    global app_state, historical_data
    record_counter = 0

    while True:
        try:
            # 1. 采集数据
            t, h, p, l, d = 0, 0, 0, 0, 0
            if sht40:
                t = sht40.temperature
                h = sht40.relative_humidity
            if veml7700:
                l = veml7700.lux

            with i2c_bus_lock:
                if spl06_sensor:
                    raw_p = spl06_sensor.get_pressure()
                    if raw_p > 0:
                        p = raw_p
                    else:
                        p = app_state["current_pressure"]

                if vl5300_ready:
                    raw_d = vl5300_get_raw_tof(i2c_smbus)
                    if raw_d is not None:
                        d = vl5300_raw_to_mm(raw_d)
                    else:
                        d = app_state["current_distance"]

            # 读取烟雾 (LOW 通常意味着检测到烟雾)
            is_smoke_safe = (GPIO.input(SMOKE_PIN) == GPIO.HIGH)

            # 2. 更新状态
            with state_lock:
                app_state["current_temp_c"] = t
                app_state["current_humidity"] = h
                app_state["current_pressure"] = p
                app_state["current_light"] = l
                app_state["current_distance"] = d
                app_state["smoke_safe"] = is_smoke_safe

                # 3. 历史记录策略 (每5次循环记录一次，约5秒)
                record_counter += 1
                if record_counter >= 5:
                    record_counter = 0
                    historical_data.append({
                        "timestamp": datetime.datetime.now().isoformat(),
                        "temp_c": t,
                        "humidity": h
                    })
                    # 扩大到 3000 条 (约4小时)
                    if len(historical_data) > 3000:
                        historical_data.pop(0)

                # 4. 警报判定
                if app_state["system_status"] == "ARMED" and app_state["alarm_status"] == "NONE":
                    reason = None
                    if not is_smoke_safe:
                        reason = "SMOKE DETECTED"
                    elif abs(l - app_state["baseline_light"]) > LIGHT_THRESHOLD_LUX:
                        reason = "LIGHT CHANGED"
                    elif abs(p - app_state["baseline_pressure"]) > PRESSURE_THRESHOLD_HPA:
                        reason = "PRESSURE LEAK"
                    elif abs(d - app_state["baseline_distance"]) > DISTANCE_THRESHOLD_MM:
                        reason = "MOVEMENT DETECTED"
                    elif abs(t - app_state["baseline_temp"]) > TEMP_THRESHOLD_C:
                        reason = "TEMP ABNORMAL"
                    elif abs(h - app_state["baseline_humidity"]) > HUMIDITY_THRESHOLD_PCT:
                        reason = "HUMIDITY ABNORMAL"

                    if reason:
                        app_state["alarm_status"] = reason
                        trigger_alarm()

        except Exception as e:
            print(f"Error: {e}")
        time.sleep(1)  # 1秒采样率


# --- 8. Web 路由 ---
app = Flask(__name__)


@app.route('/')
def index(): return render_template('index.html')


@app.route('/api/data')
def api_data():
    with state_lock: return jsonify(app_state)


@app.route('/api/history')
def api_history():
    with state_lock: return jsonify(historical_data)


@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


def gen_frames():
    while True:
        with picam2_lock:
            stream = io.BytesIO()
            picam2.capture_file(stream, format='jpeg')
            frame = stream.getvalue()
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.1)


@app.route('/api/scan', methods=['POST'])
def handle_scan():
    success, msg = scan_qr_code()
    return jsonify({"status": "ok" if success else "error", "message": msg})


@app.route('/api/arm', methods=['POST'])
def handle_arm():
    success, msg = set_system_baseline()
    return jsonify({"status": "ok" if success else "error", "message": msg})


@app.route('/api/reset', methods=['POST'])
def handle_reset():
    reset_system_state()
    return jsonify({"status": "ok", "message": "重置成功"})


if __name__ == '__main__':
    initialize_hardware()
    t = threading.Thread(target=sensor_thread)
    t.daemon = True
    t.start()
    app.run(host='0.0.0.0', port=5000, debug=False)