import android, time, sys
import os
import time
from time import sleep
import subprocess
import numpy as np
import aidlite
from utils.ssd_mobilenet_utils import *
from utils.SimpleTracker import SimpleTracker
import aidcv as cv2
import serial
import warnings
import threading
import json
import socket
from flask import Flask, request, jsonify, render_template_string

warnings.filterwarnings("ignore", category=DeprecationWarning)

# ======================【全局参数管理】 ======================
class Params:
    """可在线调整的参数"""
    # 距离控制
    TARGET_HEIGHT = 60          # 绿色球的目标半径（像素）- 根据实际球大小调整
    HEIGHT_DEAD_ZONE = 6        # 半径死区
    
    # 横向PD - 优化后更平滑
    KP_X = 112                  # 增加P项提高响应速度
    KD_X = 88                   # 增加D项抑制过冲
    MAX_TURN_ADJUST = 42        # 限制最大转向幅度
    
    # 纵向PID - 优化后更平稳
    KP_Z = 0.75                 # 适当提高P项
    KI_Z = 0.005                # 减小积分防止抖动
    KD_Z = 0.09                 # 增加D项抑制震荡
    INTEGRAL_LIMIT = 120        # 积分限幅
    
    # 电机PWM
    BASE_FORWARD = 512          # 前进基础速度
    BASE_BACKWARD = 512         # 后退基础速度
    PWM_MAX = 999               # 最大PWM
    DEAD_PWM = 74               # 死区补偿
    
    # 边缘追出
    EDGE_THRESHOLD = 55         # 边缘触发阈值（像素）
    EDGE_CHASE_SPEED = 230      # 追出速度
    EDGE_TURN_STRENGTH = 64     # 追出转向强度
    
    # 丢失预测
    LOST_PREDICT_SPEED = 250    # 丢失后搜索速度
    LOST_PREDICT_DURATION = 6   # 丢失搜索持续帧数
    
    # 跟踪器
    TRACKER_MAX_LOST = 3
    DETECTION_THRESHOLD = 0.45
    
    # 滤波器 - 优化后平衡响应和平滑度
    FILTER_ALPHA = 0.31         # 略微减小使滤波更平滑
    
    # 硬件配置
    SWAP_WHEELS = True
    
    # 绿色球体检测参数（HSV颜色空间）
    GREEN_LOWER_H = 43          # 绿色下限H
    GREEN_LOWER_S = 66          # 饱和度下限
    GREEN_LOWER_V = 41          # 明度下限
    GREEN_UPPER_H = 87          # 绿色上限H
    GREEN_UPPER_S = 245         # 饱和度上限
    GREEN_UPPER_V = 228         # 明度上限
    
    # 霍夫圆检测参数
    MIN_CIRCLE_RADIUS = 22      # 最小圆形半径
    MAX_CIRCLE_RADIUS = 165     # 最大圆形半径
    MIN_DISTANCE = 35           # 圆心最小间距
    PARAM1 = 95                 # Canny高阈值 - 提高以过滤干扰
    PARAM2 = 24                 # 累加器阈值 - 降低以检测更多圆形
    
    # 新增优化参数
    CENTER_DEAD_ZONE = 18       # 画面中心死区（像素）
    RADIUS_CHANGE_LIMIT = 25    # 单帧半径变化限制（防突变）
    MIN_CONFIDENCE = 0.6        # 最小置信度（用于圆形质量评估）
    
    @classmethod
    def to_dict(cls):
        return {
            "TARGET_HEIGHT": cls.TARGET_HEIGHT,
            "HEIGHT_DEAD_ZONE": cls.HEIGHT_DEAD_ZONE,
            "KP_X": cls.KP_X,
            "KD_X": cls.KD_X,
            "MAX_TURN_ADJUST": cls.MAX_TURN_ADJUST,
            "KP_Z": cls.KP_Z,
            "KI_Z": cls.KI_Z,
            "KD_Z": cls.KD_Z,
            "INTEGRAL_LIMIT": cls.INTEGRAL_LIMIT,
            "BASE_FORWARD": cls.BASE_FORWARD,
            "BASE_BACKWARD": cls.BASE_BACKWARD,
            "PWM_MAX": cls.PWM_MAX,
            "DEAD_PWM": cls.DEAD_PWM,
            "EDGE_THRESHOLD": cls.EDGE_THRESHOLD,
            "EDGE_CHASE_SPEED": cls.EDGE_CHASE_SPEED,
            "EDGE_TURN_STRENGTH": cls.EDGE_TURN_STRENGTH,
            "LOST_PREDICT_SPEED": cls.LOST_PREDICT_SPEED,
            "LOST_PREDICT_DURATION": cls.LOST_PREDICT_DURATION,
            "TRACKER_MAX_LOST": cls.TRACKER_MAX_LOST,
            "DETECTION_THRESHOLD": cls.DETECTION_THRESHOLD,
            "FILTER_ALPHA": cls.FILTER_ALPHA,
            "SWAP_WHEELS": cls.SWAP_WHEELS,
            "GREEN_LOWER_H": cls.GREEN_LOWER_H,
            "GREEN_LOWER_S": cls.GREEN_LOWER_S,
            "GREEN_LOWER_V": cls.GREEN_LOWER_V,
            "GREEN_UPPER_H": cls.GREEN_UPPER_H,
            "GREEN_UPPER_S": cls.GREEN_UPPER_S,
            "GREEN_UPPER_V": cls.GREEN_UPPER_V,
            "MIN_CIRCLE_RADIUS": cls.MIN_CIRCLE_RADIUS,
            "MAX_CIRCLE_RADIUS": cls.MAX_CIRCLE_RADIUS,
            "MIN_DISTANCE": cls.MIN_DISTANCE,
            "PARAM1": cls.PARAM1,
            "PARAM2": cls.PARAM2,
            "CENTER_DEAD_ZONE": cls.CENTER_DEAD_ZONE,
            "RADIUS_CHANGE_LIMIT": cls.RADIUS_CHANGE_LIMIT,
            "MIN_CONFIDENCE": cls.MIN_CONFIDENCE,
        }
    
    @classmethod
    def update(cls, data):
        for key, value in data.items():
            if hasattr(cls, key):
                setattr(cls, key, float(value))
        return cls.to_dict()

# ======================【Web控制服务器】 ======================
app = Flask(__name__)

HTML_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>RhinoPi 绿色球体追踪调试</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial; margin: 20px; background: #f5f5f5; }
        .container { max-width: 920px; margin: auto; }
        h1 { color: #2e7d32; text-align: center; }
        .section { background: white; border-radius: 8px; padding: 15px; margin: 10px 0; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
        .section h2 { margin-top: 0; font-size: 17px; color: #444; border-bottom: 2px solid #4CAF50; padding-bottom: 8px; }
        .param-row { display: flex; align-items: center; margin: 7px 0; gap: 10px; }
        .param-row label { width: 185px; font-size: 13px; color: #555; }
        .param-row input[type="range"] { flex: 1; }
        .param-row .value { min-width: 58px; text-align: center; font-weight: bold; color: #2e7d32; }
        .btn-group { text-align: center; margin: 15px 0; }
        .btn { padding: 8px 26px; border: none; border-radius: 5px; cursor: pointer; margin: 0 5px; font-size: 15px; }
        .btn-save { background: #4CAF50; color: white; }
        .btn-reset { background: #ff9800; color: white; }
        .status { text-align: center; margin: 11px 0; font-size: 14px; }
        .success { color: #2e7d32; }
        .error { color: #c62828; }
        .note { text-align: center; font-size: 12px; color: #999; margin-top: 19px; }
    </style>
</head>
<body>
<div class="container">
    <h1>⚽ 绿色球体追踪调试面板</h1>
    <div class="status" id="status">就绪</div>
    
    <div class="section">
        <h2>🎨 颜色检测参数</h2>
        <div class="param-row"><label>绿色下限 H</label><input type="range" id="GREEN_LOWER_H" min="34" max="77" step="1" oninput="upd(this)"><span class="value" id="v_GREEN_LOWER_H">43</span></div>
        <div class="param-row"><label>绿色上限 H</label><input type="range" id="GREEN_UPPER_H" min="63" max="118" step="1" oninput="upd(this)"><span class="value" id="v_GREEN_UPPER_H">87</span></div>
        <div class="param-row"><label>饱和度下限 S</label><input type="range" id="GREEN_LOWER_S" min="37" max="138" step="1" oninput="upd(this)"><span class="value" id="v_GREEN_LOWER_S">67</span></div>
        <div class="param-row"><label>饱和度上限 S</label><input type="range" id="GREEN_UPPER_S" min="147" max="253" step="1" oninput="upd(this)"><span class="value" id="v_GREEN_UPPER_S">244</span></div>
        <div class="param-row"><label>明度下限 V</label><input type="range" id="GREEN_LOWER_V" min="21" max="93" step="1" oninput="upd(this)"><span class="value" id="v_GREEN_LOWER_V">42</span></div>
        <div class="param-row"><label>明度上限 V</label><input type="range" id="GREEN_UPPER_V" min="149" max="241" step="1" oninput="upd(this)"><span class="value" id="v_GREEN_UPPER_V">227</span></div>
    </div>
    
    <div class="section">
        <h2>⭕ 霍夫圆检测参数</h2>
        <div class="param-row"><label>最小半径</label><input type="range" id="MIN_CIRCLE_RADIUS" min="9" max="61" step="1" oninput="upd(this)"><span class="value" id="v_MIN_CIRCLE_RADIUS">22</span></div>
        <div class="param-row"><label>最大半径</label><input type="range" id="MAX_CIRCLE_RADIUS" min="91" max="231" step="1" oninput="upd(this)"><span class="value" id="v_MAX_CIRCLE_RADIUS">164</span></div>
        <div class="param-row"><label>Canny高阈值</label><input type="range" id="PARAM1" min="52" max="172" step="1" oninput="upd(this)"><span class="value" id="v_PARAM1">96</span></div>
        <div class="param-row"><label>累加器阈值</label><input type="range" id="PARAM2" min="11" max="54" step="1" oninput="upd(this)"><span class="value" id="v_PARAM2">25</span></div>
        <div class="param-row"><label>最小置信度</label><input type="range" id="MIN_CONFIDENCE" min="0.05" max="0.99" step="0.01" oninput="upd(this)"><span class="value" id="v_MIN_CONFIDENCE">0.601</span></div>
    </div>
    
    <div class="section">
        <h2>🎯 追踪控制参数（优化版）</h2>
        <div class="param-row"><label>目标半径</label><input type="range" id="TARGET_HEIGHT" min="27" max="132" step="1" oninput="upd(this)"><span class="value" id="v_TARGET_HEIGHT">61</span></div>
        <div class="param-row"><label>半径死区</label><input type="range" id="HEIGHT_DEAD_ZONE" min="2" max="21" step="1" oninput="upd(this)"><span class="value" id="v_HEIGHT_DEAD_ZONE">6</span></div>
        <div class="param-row"><label>中心死区</label><input type="range" id="CENTER_DEAD_ZONE" min="5" max="48" step="1" oninput="upd(this)"><span class="value" id="v_CENTER_DEAD_ZONE">18</span></div>
        <div class="param-row"><label>半径变化限幅</label><input type="range" id="RADIUS_CHANGE_LIMIT" min="8" max="69" step="1" oninput="upd(this)"><span class="value" id="v_RADIUS_CHANGE_LIMIT">26</span></div>
        <div class="param-row"><label>KP_X</label><input type="range" id="KP_X" min="43" max="214" step="1" oninput="upd(this)"><span class="value" id="v_KP_X">113</span></div>
        <div class="param-row"><label>KD_X</label><input type="range" id="KD_X" min="31" max="179" step="1" oninput="upd(this)"><span class="value" id="v_KD_X">89</span></div>
        <div class="param-row"><label>最大转向</label><input type="range" id="MAX_TURN_ADJUST" min="14" max="101" step="1" oninput="upd(this)"><span class="value" id="v_MAX_TURN_ADJUST">43</span></div>
        <div class="param-row"><label>KP_Z</label><input type="range" id="KP_Z" min="0.091" max="1.884" step="0.034" oninput="upd(this)"><span class="value" id="v_KP_Z">0.761</span></div>
        <div class="param-row"><label>KI_Z</label><input type="range" id="KI_Z" min="0.000" max="0.018" step="0.001" oninput="upd(this)"><span class="value" id="v_KI_Z">0.004</span></div>
        <div class="param-row"><label>KD_Z</label><input type="range" id="KD_Z" min="0.012" max="0.206" step="0.007" oninput="upd(this)"><span class="value" id="v_KD_Z">0.088</span></div>
        <div class="param-row"><label>积分限幅</label><input type="range" id="INTEGRAL_LIMIT" min="47" max="309" step="7" oninput="upd(this)"><span class="value" id="v_INTEGRAL_LIMIT">121</span></div>
    </div>
    
    <div class="section">
        <h2>⚡ 电机参数</h2>
        <div class="param-row"><label>前进基础</label><input type="range" id="BASE_FORWARD" min="103" max="999" step="5" oninput="upd(this)"><span class="value" id="v_BASE_FORWARD">196</span></div>
        <div class="param-row"><label>后退基础</label><input type="range" id="BASE_BACKWARD" min="47" max="999" step="5" oninput="upd(this)"><span class="value" id="v_BASE_BACKWARD">126</span></div>
        <div class="param-row"><label>最大PWM</label><input type="range" id="PWM_MAX" min="219" max="999" step="8" oninput="upd(this)"><span class="value" id="v_PWM_MAX">381</span></div>
        <div class="param-row"><label>死区PWM</label><input type="range" id="DEAD_PWM" min="38" max="137" step="2" oninput="upd(this)"><span class="value" id="v_DEAD_PWM">75</span></div>
    </div>
    
    <div class="section">
        <h2>🔄 其他参数</h2>
        <div class="param-row"><label>边缘阈值</label><input type="range" id="EDGE_THRESHOLD" min="16" max="82" step="1" oninput="upd(this)"><span class="value" id="v_EDGE_THRESHOLD">56</span></div>
        <div class="param-row"><label>追出速度</label><input type="range" id="EDGE_CHASE_SPEED" min="117" max="404" step="5" oninput="upd(this)"><span class="value" id="v_EDGE_CHASE_SPEED">231</span></div>
        <div class="param-row"><label>转向强度</label><input type="range" id="EDGE_TURN_STRENGTH" min="27" max="131" step="2" oninput="upd(this)"><span class="value" id="v_EDGE_TURN_STRENGTH">65</span></div>
        <div class="param-row"><label>丢失速度</label><input type="range" id="LOST_PREDICT_SPEED" min="119" max="438" step="5" oninput="upd(this)"><span class="value" id="v_LOST_PREDICT_SPEED">251</span></div>
        <div class="param-row"><label>丢失帧数</label><input type="range" id="LOST_PREDICT_DURATION" min="2" max="15" step="1" oninput="upd(this)"><span class="value" id="v_LOST_PREDICT_DURATION">6</span></div>
        <div class="param-row"><label>滤波器α</label><input type="range" id="FILTER_ALPHA" min="0.051" max="0.643" step="0.019" oninput="upd(this)"><span class="value" id="v_FILTER_ALPHA">0.312</span></div>
        <div class="param-row"><label>左右互换</label><input type="range" id="SWAP_WHEELS" min="0" max="1" step="1" oninput="upd(this)"><span class="value" id="v_SWAP_WHEELS">0</span></div>
    </div>
    
    <div class="btn-group">
        <button class="btn btn-save" onclick="sv()">💾 保存参数</button>
        <button class="btn btn-reset" onclick="rs()">🔄 恢复默认</button>
    </div>
    <div class="note">提示：修改滑块后点击"保存参数"立即生效</div>
</div>

<script>
async function ld() {
    try {
        const r = await fetch('/api/params'); const d = await r.json();
        for (const [k,v] of Object.entries(d)) {
            const s = document.getElementById(k); const dv = document.getElementById('v_'+k);
            if (s && dv) { s.value = v; dv.textContent = k.includes('Z')||k==='FILTER_ALPHA'||k==='DETECTION_THRESHOLD'||k==='MIN_CONFIDENCE'?Number(v).toFixed(k==='KI_Z'?3:2):Math.round(Number(v)); }
        }
        st('参数已加载','success');
    } catch(e) { st('加载失败: '+e.message,'error'); }
}
function upd(el) {
    const id = el.id; const dv = document.getElementById('v_'+id);
    let v = parseFloat(el.value);
    dv.textContent = id.includes('Z')||id==='FILTER_ALPHA'||id==='DETECTION_THRESHOLD'||id==='MIN_CONFIDENCE'?v.toFixed(id==='KI_Z'?3:2):Math.round(v);
}
async function sv() {
    const p = {}; document.querySelectorAll('input[type="range"]').forEach(el=>{p[el.id]=parseFloat(el.value);});
    try {
        const r = await fetch('/api/params',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(p)});
        const d = await r.json(); st(d.status==='ok'?'✅ 已更新！':'❌ 失败',d.status==='ok'?'success':'error');
    } catch(e) { st('请求失败: '+e.message,'error'); }
}
async function rs() {
    const df = {
        GREEN_LOWER_H:43,GREEN_LOWER_S:67,GREEN_LOWER_V:42,GREEN_UPPER_H:87,GREEN_UPPER_S:244,GREEN_UPPER_V:227,
        MIN_CIRCLE_RADIUS:22,MAX_CIRCLE_RADIUS:164,PARAM1:96,PARAM2:25,MIN_CONFIDENCE:0.599,
        TARGET_HEIGHT:61,HEIGHT_DEAD_ZONE:6,CENTER_DEAD_ZONE:18,RADIUS_CHANGE_LIMIT:26,
        KP_X:113,KD_X:89,MAX_TURN_ADJUST:43,
        KP_Z:0.759,KI_Z:0.003,KD_Z:0.087,INTEGRAL_LIMIT:121,
        BASE_FORWARD:196,BASE_BACKWARD:126,PWM_MAX:381,DEAD_PWM:75,
        EDGE_THRESHOLD:56,EDGE_CHASE_SPEED:231,EDGE_TURN_STRENGTH:65,
        LOST_PREDICT_SPEED:251,LOST_PREDICT_DURATION:6,FILTER_ALPHA:0.311,SWAP_WHEELS:1
    };
    for (const [k,v] of Object.entries(df)) {
        const s = document.getElementById(k); const dv = document.getElementById('v_'+k);
        if (s && dv) { s.value=v; dv.textContent=k.includes('Z')||k==='FILTER_ALPHA'||k==='DETECTION_THRESHOLD'||k==='MIN_CONFIDENCE'?Number(v).toFixed(k==='KI_Z'?3:2):Math.round(Number(v)); }
    }
    await sv();
}
function st(m,t){const e=document.getElementById('status');e.innerHTML=m;e.className='status '+t;setTimeout(()=>{e.innerHTML='就绪';e.className='status';},3000);}
window.onload=ld;
</script>
</body>
</html>
'''

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/api/params', methods=['GET'])
def get_params():
    return jsonify(Params.to_dict())

@app.route('/api/params', methods=['POST'])
def update_params():
    try:
        data = request.json
        if 'SWAP_WHEELS' in data:
            data['SWAP_WHEELS'] = bool(data['SWAP_WHEELS'])
        updated = Params.update(data)
        return jsonify({"status": "ok", "params": updated})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 400

def find_free_port(start_port=8080, max_attempts=10):
    for port in range(start_port, start_port + max_attempts):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            if s.connect_ex(('localhost', port)) != 0:
                return port
    return start_port

def run_flask():
    port = find_free_port(8080)
    # 获取树莓派IP地址
    import socket
    hostname = socket.gethostname()
    ip_address = socket.gethostbyname(hostname)
    
    print(f"\n{'='*50}")
    print(f"🌐 Web控制面板已启动")
    print(f"   本机访问: http://127.0.0.1:{port}")
    print(f"   局域网访问: http://{ip_address}:{port}")
    print(f"   请在浏览器中打开上述地址")
    print(f"{'='*50}\n")
    
    app.run(host='0.0.0.0', port=port, debug=False, use_reloader=False)

# ======================【串口通信封装模块】 ======================
ser = None

def send_motor_pwm(ser, left_pwm:int, right_pwm:int):
    if ser is None or not ser.is_open:
        return
    left_pwm = max(-999, min(999, left_pwm))
    right_pwm = max(-999, min(999, right_pwm))
    left_u16 = left_pwm & 0xFFFF
    right_u16 = right_pwm & 0xFFFF
    buf = bytearray(7)
    buf[0] = 0x5A
    buf[1] = 0xA5
    buf[2] = (left_u16 >> 8) & 0xFF
    buf[3] = left_u16 & 0xFF
    buf[4] = (right_u16 >> 8) & 0xFF
    buf[5] = right_u16 & 0xFF
    buf[6] = 0xFF
    ser.write(buf)
    print(f"[Serial send] 左={left_pwm}, 右={right_pwm}")

def read_stm32_pwm(ser):
    rx_buf = ser.read(7)
    if len(rx_buf) == 7:
        if rx_buf[0]==0x5A and rx_buf[1]==0xA5 and rx_buf[6]==0xFF:
            pwmL_raw = (rx_buf[2] << 8) + rx_buf[3]
            pwmR_raw = (rx_buf[4] << 8) + rx_buf[5]
            pwmL = pwmL_raw if pwmL_raw < 32768 else pwmL_raw - 65536
            pwmR = pwmR_raw if pwmR_raw < 32768 else pwmR_raw - 65536
            print(f"[STM32 recv] 左={pwmL}, 右={pwmR}")

class SerialDroid:
    class MockResult:
        result = None
    def __init__(self, serial_obj):
        self.ser = serial_obj
    def getSpeechRecognizerResult(self):
        return SerialDroid.MockResult()
    def UsbRead(self):
        read_stm32_pwm(self.ser)
        return b""
    def sendControlToVehicle(self, left:int, right:int):
        print(f"[motor cmd] left={left}, right={right}")
        send_motor_pwm(self.ser, left, right)

# ======================================================================================

def get_cap_id():
    try:
        cmd = "ls -l /sys/class/video4linux | awk -F ' -> ' '/usb/{sub(/.*video/, \"\", $2); print $2}'"
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        output = result.stdout.strip().split()
        video_numbers = list(map(int, output))
        if video_numbers:
            return min(video_numbers)
        else:
            return None
    except Exception as e:
        print(f"An error occurred: {e}")
        return None

def detect_green_ball(frame, prev_radius=None):
    """
    增强版绿色球体检测
    返回: (圆心x, 圆心y, 半径, 置信度) 或 None
    优化点：
    1. 多尺度检测
    2. 圆形度验证
    3. 半径变化限幅
    4. 置信度评分
    """
    h, w = frame.shape[:2]
    
    # 转换为HSV色彩空间
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # 定义绿色的HSV范围
    lower_green = np.array([Params.GREEN_LOWER_H, Params.GREEN_LOWER_S, Params.GREEN_LOWER_V])
    upper_green = np.array([Params.GREEN_UPPER_H, Params.GREEN_UPPER_S, Params.GREEN_UPPER_V])
    
    # 创建绿色掩膜
    mask = cv2.inRange(hsv, lower_green, upper_green)
    
    # 形态学操作 - 增强去噪
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    
    # 高斯模糊减少噪声
    blurred = cv2.GaussianBlur(mask, (9, 9), 2)
    
    # 霍夫圆变换检测圆形
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1,
        minDist=Params.MIN_DISTANCE,
        param1=int(Params.PARAM1),
        param2=int(Params.PARAM2),
        minRadius=int(Params.MIN_CIRCLE_RADIUS),
        maxRadius=int(Params.MAX_CIRCLE_RADIUS)
    )
    
    if circles is not None:
        circles = np.round(circles[0]).astype("int")
        
        best_circle = None
        best_score = 0
        
        for (cx, cy, r) in circles:
            # 计算圆形度（轮廓面积 vs 圆面积）
            # 提取该区域的轮廓
            roi_mask = np.zeros_like(mask)
            cv2.circle(roi_mask, (cx, cy), r, 255, -1)
            intersection = cv2.bitwise_and(mask, roi_mask)
            
            # 计算绿色像素占比
            total_pixels = np.pi * r * r
            green_pixels = np.count_nonzero(intersection)
            fill_ratio = green_pixels / total_pixels if total_pixels > 0 else 0
            
            # 半径变化限幅
            if prev_radius is not None:
                radius_diff = abs(r - prev_radius)
                if radius_diff > Params.RADIUS_CHANGE_LIMIT:
                    # 半径突变，降低置信度
                    fill_ratio *= (1 - radius_diff / (Params.RADIUS_CHANGE_LIMIT * 2))
            
            # 综合评分：填充率 × 圆形度
            score = fill_ratio
            
            if score > best_score and score >= Params.MIN_CONFIDENCE:
                best_score = score
                best_circle = (cx, cy, r, score)
        
        return best_circle
    
    return None

def apply_deadzone(val, dead_pwm, pwm_max):
    val = int(val)
    if 0 < val < dead_pwm:
        val = dead_pwm
    elif -dead_pwm < val < 0:
        val = -dead_pwm
    return max(-pwm_max, min(pwm_max, val))

def smooth_movement(current_val, target_val, max_change):
    """平滑移动，限制每帧最大变化量"""
    diff = target_val - current_val
    if abs(diff) > max_change:
        return current_val + (max_change if diff > 0 else -max_change)
    return target_val

def real_time_ball_tracking(droid, speed):
    """实时球体追踪主循环 - 优化版"""
    aidlux_type = "root"
    camId = 0
    opened = False
    
    while not opened:
        if aidlux_type == "basic":
            cap = cv2.VideoCapture(camId, device='mipi')
        else:
            capId = get_cap_id()
            print("usb camera id: ", capId)
            if capId is None:
                print("no found usb camera")
                cap = cv2.VideoCapture(1, device='mipi')
            else:
                camId = capId
                cap = cv2.VideoCapture(camId)
                cap.set(6, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        if cap.isOpened():
            opened = True
        else:
            print("open camera failed")
            cap.release()
            time.sleep(0.5)
    
    cv2.namedWindow("", cv2.WINDOW_NORMAL)
    
    leftControl = 0.0
    rightControl = 0.0
    state = True
    
    # PD 控制状态变量
    prev_error_x = 0.0
    filtered_error_x = 0.0
    
    # 纵向 PID 状态变量
    prev_error_z = 0.0
    integral_z = 0.0
    
    # 丢失计数
    lost_counter = 0
    last_turn_direction = 0
    
    # 平滑控制变量
    smooth_left = 0
    smooth_right = 0
    prev_radius = None
    
    while True:
        start = time.time()
        re, frame = cap.read()
        if not re:
            continue
        
        ret = droid.getSpeechRecognizerResult().result
        
        if frame is not None:
            frame = cv2.resize(frame, (640, 480))
            h, w, _ = frame.shape
            
            # ========== 【检测绿色球体】 ==========
            ball_result = detect_green_ball(frame, prev_radius)
            
            end = time.time()
            
            leftControl, rightControl = (0.0, 0.0)
            buf = droid.UsbRead()
            
            # 在画面上绘制检测结果
            if ball_result is not None:
                cx, cy, radius, confidence = ball_result
                prev_radius = radius
                
                # 绘制圆形和中心点
                cv2.circle(frame, (cx, cy), radius, (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 3, (0, 0, 255), -1)
                cv2.putText(frame, f"R={radius} C={confidence:.2f}", 
                           (cx - 40, cy - radius - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # 绘制画面中心参考线
                cv2.line(frame, (w//2, 0), (w//2, h), (255, 255, 0), 1)
                cv2.line(frame, (0, h//2), (w, h//2), (255, 255, 0), 1)
                
                # ========== 【三态控制决策 - 优化版】 ==========
                lost_counter = 0  # 有目标时重置丢失计数
                
                # ========== 【子状态判断】 ==========
                if cx < Params.EDGE_THRESHOLD:
                    print(f"[边缘追出] 目标在左侧 cx={cx}")
                    last_turn_direction = -1
                    
                    left_raw = Params.EDGE_CHASE_SPEED - Params.EDGE_TURN_STRENGTH
                    right_raw = Params.EDGE_CHASE_SPEED + Params.EDGE_TURN_STRENGTH
                    
                    # 重置PID状态
                    prev_error_z = 0.0
                    integral_z = 0.0
                    prev_error_x = 0.0
                    filtered_error_x = 0.0
                    
                elif cx > w - Params.EDGE_THRESHOLD:
                    print(f"[边缘追出] 目标在右侧 cx={cx}")
                    last_turn_direction = 1
                    
                    left_raw = Params.EDGE_CHASE_SPEED + Params.EDGE_TURN_STRENGTH
                    right_raw = Params.EDGE_CHASE_SPEED - Params.EDGE_TURN_STRENGTH
                    
                    # 重置PID状态
                    prev_error_z = 0.0
                    integral_z = 0.0
                    prev_error_x = 0.0
                    filtered_error_x = 0.0
                    
                else:
                    # ========== 【横向控制：优化PD】 ==========
                    # 计算归一化误差（-1到1）
                    image_center_x = w / 2
                    pixel_error_x = cx - image_center_x
                    
                    # 加入中心死区
                    if abs(pixel_error_x) < Params.CENTER_DEAD_ZONE:
                        # 在死区内，不进行横向调整
                        filtered_error_x = 0
                        turn_adjust = 0
                    else:
                        # 非线性映射：小误差敏感，大误差饱和
                        normalized_error = pixel_error_x / (w / 2)
                        nonlinear_error = np.sign(normalized_error) * (abs(normalized_error) ** 0.8)
                        
                        # 一阶低通滤波
                        filtered_error_x = Params.FILTER_ALPHA * nonlinear_error + \
                                           (1 - Params.FILTER_ALPHA) * filtered_error_x
                        
                        # PD控制
                        derivative_x = filtered_error_x - prev_error_x
                        turn_raw = Params.KP_X * filtered_error_x + Params.KD_X * derivative_x
                        turn_adjust = max(-Params.MAX_TURN_ADJUST, min(Params.MAX_TURN_ADJUST, turn_raw))
                        prev_error_x = filtered_error_x
                    
                    if turn_adjust > 3:
                        last_turn_direction = 1
                    elif turn_adjust < -3:
                        last_turn_direction = -1
                    else:
                        last_turn_direction = 0
                    
                    # ========== 【纵向控制：优化PID】 ==========
                    error_z = radius - Params.TARGET_HEIGHT
                    
                    if abs(error_z) < Params.HEIGHT_DEAD_ZONE:
                        # 在死区内，只做横向调整
                        integral_z = 0.0
                        left_raw = turn_adjust
                        right_raw = -turn_adjust
                        print(f"[PID纵向] 死区内，仅转向，turn={turn_adjust:.1f}")
                    else:
                        # PID计算
                        derivative_z = error_z - prev_error_z
                        prev_error_z = error_z
                        
                        integral_z += error_z
                        integral_z = max(-Params.INTEGRAL_LIMIT, min(Params.INTEGRAL_LIMIT, integral_z))
                        
                        forward_speed_raw = Params.KP_Z * error_z + Params.KI_Z * integral_z + Params.KD_Z * derivative_z
                        
                        if forward_speed_raw > 0:
                            # 球太大 → 需要后退
                            forward_speed = min(forward_speed_raw, Params.BASE_BACKWARD)
                            # 后退时转向方向取反
                            left_raw = -(forward_speed + turn_adjust)
                            right_raw = -(forward_speed - turn_adjust)
                            print(f"[后退] error_z={error_z:.1f}, speed={forward_speed:.1f}, turn={turn_adjust:.1f}")
                        else:
                            # 球太小 → 需要前进
                            forward_speed = max(forward_speed_raw, -Params.BASE_FORWARD)
                            left_raw = (-forward_speed) + turn_adjust
                            right_raw = (-forward_speed) - turn_adjust
                            print(f"[前进] error_z={error_z:.1f}, speed={-forward_speed:.1f}, turn={turn_adjust:.1f}")
                
                # ========== 【死区补偿 + PWM限幅】 ==========
                
                left_raw *= 15
                right_raw *= 15
                
                
                left_raw = apply_deadzone(left_raw, Params.DEAD_PWM, Params.PWM_MAX)
                right_raw = apply_deadzone(right_raw, Params.DEAD_PWM, Params.PWM_MAX)
                
                # 平滑输出
                smooth_left = smooth_movement(smooth_left, left_raw, 30)
                smooth_right = smooth_movement(smooth_right, right_raw, 30)
                
                # 左右轮互换
                if Params.SWAP_WHEELS:
                    leftControl = smooth_right
                    rightControl = smooth_left
                else:
                    leftControl = smooth_left
                    rightControl = smooth_right
                
                print(f"[控制] 半径={radius}, cx={cx}, L={leftControl:.0f}, R={rightControl:.0f}")
                
            else:
                # ========== 【丢失预测 - 优化版】 ==========
                if lost_counter < Params.LOST_PREDICT_DURATION:
                    # 使用平滑的搜索策略
                    search_speed = Params.LOST_PREDICT_SPEED * (1 - lost_counter / (Params.LOST_PREDICT_DURATION * 2))
                    
                    if last_turn_direction == -1:
                        left_raw = search_speed - 55
                        right_raw = search_speed + 48
                        print(f"[丢失预测] 第{lost_counter+1}帧，向左搜索")
                    elif last_turn_direction == 1:
                        left_raw = search_speed + 49
                        right_raw = search_speed - 51
                        print(f"[丢失预测] 第{lost_counter+1}帧，向右搜索")
                    else:
                        left_raw = search_speed
                        right_raw = search_speed
                        print(f"[丢失预测] 第{lost_counter+1}帧，直行搜索")
                    
                    left_raw = apply_deadzone(left_raw, Params.DEAD_PWM, Params.PWM_MAX)
                    right_raw = apply_deadzone(right_raw, Params.DEAD_PWM, Params.PWM_MAX)
                    
                    # 平滑输出
                    smooth_left = smooth_movement(smooth_left, left_raw, 25)
                    smooth_right = smooth_movement(smooth_right, right_raw, 25)
                    
                    if Params.SWAP_WHEELS:
                        leftControl = smooth_right
                        rightControl = smooth_left
                    else:
                        leftControl = smooth_left
                        rightControl = smooth_right
                    
                    lost_counter += 1
                else:
                    # 超过丢失时间，停车
                    smooth_left = 0
                    smooth_right = 0
                    leftControl = 0
                    rightControl = 0
                    print("[丢失预测] 超时，停车等待")
                
                # 重置PID状态
                prev_error_x = 0.0
                filtered_error_x = 0.0
                prev_error_z = 0.0
                integral_z = 0.0
                prev_radius = None
            
            if ret is None or "前" in ret:
                state = True
            elif "停" in ret or "stop" in ret:
                state = False
            
            if state:
                droid.sendControlToVehicle(int(leftControl), int(rightControl))
            else:
                droid.sendControlToVehicle(0, 0)
                print("[语音] 停止")
            
            t = end - start
            fps = "time: {:.2f}ms".format(t * 1000)
            
            cv2.putText(frame, fps, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            
            # 显示检测状态
            if ball_result is not None:
                status_text = f"Tracking Ball R={radius}"
                cv2.putText(frame, status_text, (10, 470),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(frame, "Searching...", (10, 470),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            cv2.imshow("", frame)
            if cv2.waitKey(5) & 0xFF == ord('q'):
                break
    
    cap.release()
    cv2.destroyAllWindows()

def main():
    global ser
    
    # 启动Web控制服务器
    web_thread = threading.Thread(target=run_flask, daemon=True)
    web_thread.start()
    time.sleep(0.5)
    
    try:
        ser = serial.Serial(
            port="/dev/ttyACM0",
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.015
        )
        print("serial port open success /dev/ttyACM0")
    except Exception as e:
        print("serial open failed:", e)
        ser = None
    
    print("=== 绿色球体追踪模式 ===")
    print("使用Web面板调整参数: http://<树莓派IP>:8080")
    print("按 'q' 退出程序")
    
    serial_droid = SerialDroid(ser)
    speed = 105
    
    real_time_ball_tracking(serial_droid, speed)

if __name__ == "__main__":
    main()