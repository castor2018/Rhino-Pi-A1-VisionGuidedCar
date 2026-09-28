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

# ======================【串口通信封装模块】 ======================
ser = None

def send_motor_pwm(ser, left_pwm:int, right_pwm:int):
    """
    向主控板发送左右轮PWM值。
    协议：0x5A, 0xA5, 左轮PWM高8位, 左轮PWM低8位, 右轮PWM高8位, 右轮PWM低8位, 0xFF
    PWM范围：-999 ~ 999，正数前进，负数后退
    """
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
    print(f"[Serial send] 左轮PWM={left_pwm}, 右轮PWM={right_pwm}")

def read_stm32_pwm(ser):
    rx_buf = ser.read(7)
    if len(rx_buf) == 7:
        if rx_buf[0]==0x5A and rx_buf[1]==0xA5 and rx_buf[6]==0xFF:
            pwmL_raw = (rx_buf[2] << 8) + rx_buf[3]
            pwmR_raw = (rx_buf[4] << 8) + rx_buf[5]
            pwmL = pwmL_raw if pwmL_raw < 32768 else pwmL_raw - 65536
            pwmR = pwmR_raw if pwmR_raw < 32768 else pwmR_raw - 65536
            print(f"[STM32 recv] 左轮PWM:{pwmL}, 右轮PWM:{pwmR}")

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
        print(f"[origin motor cmd] left={left}, right={right}")
        left_pwm = int(left * 999 / 92)
        right_pwm = int(right * 999 / 92)
        send_motor_pwm(self.ser, left_pwm, right_pwm)

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

def run_detection(fast_interpreter, input_shape, image):
    result = fast_interpreter.set_input_tensor(0, image.data)
    if result != 0:
        print("interpreter set_input_tensor() failed")
    
    result = fast_interpreter.invoke()
    if result != 0:
        print("interpreter invoke() failed")

    classes = fast_interpreter.get_output_tensor(1)
    if classes is None:
        print("sample : interpreter->get_output_tensor(1) failed !")
        return False
    scores = fast_interpreter.get_output_tensor(2)
    if scores is None:
        print("sample : interpreter->get_output_tensor(2) failed !")
        return False
    boxes = fast_interpreter.get_output_tensor(0)
    if boxes is None:
        print("sample : interpreter->get_output_tensor(1) failed !")
        return False
    box = boxes.reshape((10, 4))

    box, scores, classes = np.squeeze(box), np.squeeze(scores), np.squeeze(classes + 1).astype(np.int32)

    out_scores, out_boxes, out_classes = non_max_suppression(scores, box, classes)
    return out_scores, out_boxes, out_classes

def real_time_object_detection(fast_interpreter, droid, speed, input_shape, colors, class_names):
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
    
    leftControl = 0.0
    rightControl = 0.0
    state = True
    tracker = SimpleTracker(max_lost=3)
    threshold = 0.6
    
    # ==================== 【距离控制参数】 ====================
    TARGET_HEIGHT_MIN = 205   # 框高度低于此值，人太远，前进
    TARGET_HEIGHT_MAX = 365   # 框高度高于此值，人太近，后退
    HEIGHT_DEAD_ZONE = 25     # 高度死区，避免频繁切换
    # ========================================================
    
    # ==================== 【横向 PD 控制参数】 ====================
    KP_X = 80                  # 横向比例增益
    KD_X = 110                 # 横向微分增益
    MAX_TURN_ADJUST = 14       # 最大转向调整量
    SWAP_WHEELS = True         # True = 左右轮互换（硬件接反时启用）
    # ========================================================
    
    # ==================== 【纵向 PID 控制参数】 ====================
    KP_Z = 0.35                # 纵向比例增益
    KI_Z = 0.008               # 纵向积分增益
    KD_Z = 0.05                # 纵向微分增益
    MAX_FORWARD_SPEED = 56     # 最大前进速度
    MAX_BACKWARD_SPEED = 28    # 最大后退速度
    INTEGRAL_LIMIT = 500       # 积分限幅，防止积分饱和
    # ========================================================
    
    # ==================== 【边缘追出参数】 ====================
    EDGE_THRESHOLD = 66        # 像素，距画面左右边缘小于此值时进入边缘追出模式
    EDGE_CHASE_SPEED = 50      # 边缘追出时的前进速度
    EDGE_TURN_STRENGTH = 10    # 边缘追出时的转向强度
    # ========================================================
    
    # ==================== 【丢失预测参数】 ====================
    LOST_PREDICT_SPEED = 36    # 目标丢失后短时前进速度
    LOST_PREDICT_DURATION = 8  # 丢失后最多持续预测帧数
    lost_counter = 0           # 丢失计数
    last_turn_direction = 0    # 最后一帧的转向方向（-1左，0直，1右）
    # ========================================================
    
    # 横向 PD 状态变量
    prev_error_x = 0.0
    FILTER_ALPHA = 0.45
    filtered_error_x = 0.0
    
    # 纵向 PID 状态变量
    prev_error_z = 0.0
    integral_z = 0.0
    target_height = (TARGET_HEIGHT_MIN + TARGET_HEIGHT_MAX) / 2
    
    while True:
        start = time.time()
        re, frame = cap.read()
        if not re:
            continue
        
        ret = droid.getSpeechRecognizerResult().result
        
        if frame is not None:
            frame = cv2.resize(frame, (640, 480))
            h, w, _ = frame.shape
            
            image_data = preprocess_image_for_tflite_uint8(frame, model_image_size=300)
            out_scores, out_boxes, out_classes = run_detection(fast_interpreter, input_shape, image_data)
            
            end = time.time()
            
            # 将检测框从归一化坐标转换为像素坐标
            detections_bbox_pixels = []
            bottom_center_x_list = []
            box_height_list = []
            for i in range(len(out_boxes)):
                if (out_classes[i] == 1 and 
                    out_scores[i] > threshold and 
                    out_boxes[i][3] - out_boxes[i][1] > 0.06):
                    
                    y_min = int(out_boxes[i][0] * h)
                    x_min = int(out_boxes[i][1] * w)
                    y_max = int(out_boxes[i][2] * h)
                    x_max = int(out_boxes[i][3] * w)
                    
                    # 计算底部中点
                    bottom_center_x = (x_min + x_max) // 2
                    bottom_center_x_list.append(bottom_center_x)
                    
                    # 计算框高度
                    box_height = y_max - y_min
                    box_height_list.append(box_height)
                    
                    detections_bbox_pixels.append([y_min, x_min, y_max, x_max])
            
            if detections_bbox_pixels:
                detections_bbox = np.array(detections_bbox_pixels).reshape(-1, 4)
            else:
                detections_bbox = np.empty((0, 4))
            
            objects = tracker.update(detections_bbox)
            
            leftControl, rightControl = (0.0, 0.0)
            buf = droid.UsbRead()
            
            # ========== 【三态控制决策】 ==========
            if len(objects) > 0:
                # ----- 状态1：有目标，正常控制 -----
                lost_counter = 0  # 重置丢失计数
                
                for idx, (objectID, centroid) in enumerate(objects.items()):
                    text = "ID {}".format(objectID)
                    print(text, centroid)
                    
                    cv2.putText(frame, text, (centroid[0] - 10, centroid[1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.circle(frame, (centroid[0], centroid[1]), 4, (0, 255, 0), -1)
                    
                    # 使用底部中点
                    if idx < len(bottom_center_x_list):
                        foot_x = bottom_center_x_list[idx]
                    else:
                        foot_x = centroid[0]
                    
                    # 获取框高度
                    if idx < len(box_height_list):
                        box_h = box_height_list[idx]
                    else:
                        box_h = 172
                    
                    # 计算横向误差：[-1, 1]，正数表示目标偏左
                    error_x = 1.0 - 2.0 * foot_x / w
                    
                    # 计算纵向误差（框高度误差）
                    error_z = box_h - target_height  # 正数太近（后退），负数太远（前进）
                    
                    # ========== 【子状态判断：边缘 or 中心】 ==========
                    if foot_x < EDGE_THRESHOLD:
                        # ----- 状态1a：目标在左边缘，进入左追出模式 -----
                        print("[边缘追出] 目标在左侧，向左追出")
                        last_turn_direction = -1
                        
                        left_raw = EDGE_CHASE_SPEED - EDGE_TURN_STRENGTH
                        right_raw = EDGE_CHASE_SPEED + EDGE_TURN_STRENGTH
                        
                        # 边缘模式下也要更新纵向 PID 状态，防止切换回中心时突变
                        prev_error_z = error_z
                        
                    elif foot_x > w - EDGE_THRESHOLD:
                        # ----- 状态1b：目标在右边缘，进入右追出模式 -----
                        print("[边缘追出] 目标在右侧，向右追出")
                        last_turn_direction = 1
                        
                        left_raw = EDGE_CHASE_SPEED + EDGE_TURN_STRENGTH
                        right_raw = EDGE_CHASE_SPEED - EDGE_TURN_STRENGTH
                        
                        prev_error_z = error_z
                        
                    else:
                        # ----- 状态1c：目标在中心区域，PD 横向 + PID 纵向控制 -----
                        # ========== 横向控制：PD ==========
                        filtered_error_x = FILTER_ALPHA * error_x + (1 - FILTER_ALPHA) * filtered_error_x
                        derivative_x = filtered_error_x - prev_error_x
                        turn_adjust_raw = KP_X * filtered_error_x + KD_X * derivative_x
                        turn_adjust = max(-MAX_TURN_ADJUST, min(MAX_TURN_ADJUST, turn_adjust_raw))
                        prev_error_x = filtered_error_x
                        
                        # 记录转向方向
                        if turn_adjust > 1:
                            last_turn_direction = 1
                        elif turn_adjust < -1:
                            last_turn_direction = -1
                        else:
                            last_turn_direction = 0
                        
                        # ========== 纵向控制：PID ==========
                        # 计算微分项
                        derivative_z = error_z - prev_error_z
                        prev_error_z = error_z
                        
                        # 死区判断
                        if abs(error_z) < HEIGHT_DEAD_ZONE:
                            # 在死区内，清零积分，速度为0
                            integral_z = 0
                            forward_speed = 0
                        else:
                            # 积分累加（只在死区外积分）
                            integral_z += error_z
                            # 积分限幅
                            integral_z = max(-INTEGRAL_LIMIT, min(INTEGRAL_LIMIT, integral_z))
                            
                            # PID 合成
                            forward_speed_raw = KP_Z * error_z + KI_Z * integral_z + KD_Z * derivative_z
                            
                            # 根据方向限幅
                            if forward_speed_raw > 0:
                                # 需要后退
                                forward_speed = -min(forward_speed_raw, MAX_BACKWARD_SPEED)
                            else:
                                # 需要前进
                                forward_speed = -min(-forward_speed_raw, MAX_FORWARD_SPEED)
                        
                        # 根据横向误差动态调整纵向速度
                        abs_error_x = abs(filtered_error_x)
                        if abs_error_x > 0.3:
                            # 横向误差大时减速，优先修正方向
                            speed_scale = max(0.4, 1.0 - abs_error_x * 0.8)
                            forward_speed = forward_speed * speed_scale
                            turn_adjust = turn_adjust * 1.2  # 加强转向
                        
                        # 合成左右轮速度
                        if forward_speed >= 0:
                            left_raw = forward_speed + turn_adjust
                            right_raw = forward_speed - turn_adjust
                        else:
                            left_raw = forward_speed - turn_adjust
                            right_raw = forward_speed + turn_adjust
                        
                        print(f"[PID纵向] error_z={error_z:.1f}, P={KP_Z*error_z:.1f}, "
                              f"I={KI_Z*integral_z:.1f}, D={KD_Z*derivative_z:.1f}, "
                              f"forward={forward_speed:.1f}")
                    
                    # 限幅
                    left_raw = max(-76, min(76, left_raw))
                    right_raw = max(-76, min(76, right_raw))
                    
                    # 左右轮互换
                    if SWAP_WHEELS:
                        leftControl = right_raw
                        rightControl = left_raw
                    else:
                        leftControl = left_raw
                        rightControl = right_raw
                    
                    print(f"[控制] 框高={box_h}, foot_x={foot_x}, "
                          f"L={leftControl:.1f}, R={rightControl:.1f}")
                    break
                    
            else:
                # ----- 状态2：目标丢失，短时预测追出 -----
                if lost_counter < LOST_PREDICT_DURATION:
                    if last_turn_direction == -1:
                        left_raw = LOST_PREDICT_SPEED - 8
                        right_raw = LOST_PREDICT_SPEED + 8
                        print(f"[丢失预测] 第{lost_counter+1}帧，沿左方向追出")
                    elif last_turn_direction == 1:
                        left_raw = LOST_PREDICT_SPEED + 8
                        right_raw = LOST_PREDICT_SPEED - 8
                        print(f"[丢失预测] 第{lost_counter+1}帧，沿右方向追出")
                    else:
                        left_raw = LOST_PREDICT_SPEED
                        right_raw = LOST_PREDICT_SPEED
                        print(f"[丢失预测] 第{lost_counter+1}帧，直行追出")
                    
                    left_raw = max(-76, min(76, left_raw))
                    right_raw = max(-76, min(76, right_raw))
                    
                    if SWAP_WHEELS:
                        leftControl = right_raw
                        rightControl = left_raw
                    else:
                        leftControl = left_raw
                        rightControl = right_raw
                    
                    lost_counter += 1
                else:
                    leftControl = 0
                    rightControl = 0
                    print("[丢失预测] 超时，停车等待")
                
                # 目标丢失时重置 PID 状态
                prev_error_x = 0.0
                filtered_error_x = 0.0
                prev_error_z = 0.0
                integral_z = 0.0
            
            result = draw_boxes_and_turn(frame, out_scores, out_boxes, out_classes, class_names, colors)
            
            # 语音控制
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
            
            cv2.imshow("", frame)
            if cv2.waitKey(5) & 0xFF == ord('q'):
                break

def main():
    global ser
    
    try:
        ser = serial.Serial(
            port="/dev/ttyACM0",
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.01
        )
        print("serial port open success /dev/ttyACM0")
    except Exception as e:
        print("serial open failed:", e)
        ser = None
    
    input_shape = [300, 300]
    inShape = [[1, 300, 300, 3]]
    outShape = [[1, 10, 4, 4], [1, 10, 4, 1], [1, 10, 4, 1], [1, 4, 1, 1]]
    model_path = "model_data/ssdlite_mobilenet_v3.tflite"
    
    model = aidlite.Model.create_instance(model_path)
    if model is None:
        print("Create model failed !")
    
    model.set_model_properties(inShape, aidlite.DataType.TYPE_INT8, outShape, aidlite.DataType.TYPE_INT8)
    
    config = aidlite.Config.create_instance()
    config.implement_type = aidlite.ImplementType.TYPE_FAST
    config.framework_type = aidlite.FrameworkType.TYPE_TFLITE
    config.accelerate_type = aidlite.AccelerateType.TYPE_GPU
    config.number_of_threads = 4
    
    fast_interpreter = aidlite.InterpreterBuilder.build_interpretper_from_model_and_config(model, config)
    if fast_interpreter is None:
        print("build_interpretper_from_model_and_config failed !")
    
    result = fast_interpreter.init()
    if result != 0:
        print("interpreter init failed !")
    
    result = fast_interpreter.load_model()
    if result != 0:
        print("interpreter load model failed !")
    print("detect model load success!")
    
    class_names = read_classes('model_data/coco_classes.txt')
    colors = generate_colors(class_names)
    
    print("connecting to usb bot")
    
    serial_droid = SerialDroid(ser)
    speed = 92
    
    real_time_object_detection(fast_interpreter, serial_droid, speed, input_shape, colors, class_names)

if __name__ == "__main__":
    main()