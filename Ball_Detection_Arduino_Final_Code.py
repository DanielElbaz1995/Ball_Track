import cv2
import time
import serial
import threading
import mediapipe as mp
from utils import visualize 
from picamera2 import Picamera2
from flask import Flask, Response
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

app = Flask(__name__)

# Initialize Picamera2
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 400) 
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.set_controls({"FrameDurationLimits": (11111, 11111)})
picam2.start()

# Global variables
detection_result_list = []
detection_frame = None
skip_frame_count = 8  # Only run detection on every 8th frame
frame_counter = 0
ball_detected = True  # Flag to indicate if the ball is detected
video_feed_started = False

# Initialize serial communication with Arduino
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
ser.reset_input_buffer()

# Exponential Moving Average (EMA) parameters
alpha = 0.15  # Smoothing factor
smoothed_x = None

def exponential_moving_average(new_value, smoothed_value, alpha):
    if smoothed_value is None:
        return int(new_value)
    smoothed_value = alpha * new_value + (1 - alpha) * smoothed_value
    return int(smoothed_value)

def send_to_arduino(value):
    command = str(value) + '\n'
    ser.write(command.encode())
    ser.flush()
    
def ball_position():
    global smoothed_x, ball_detected, video_feed_started
    while True:
        time.sleep(1.3) 
        if video_feed_started:
            if ball_detected:
                send_to_arduino(smoothed_x)
                print(f'Ball detected! x_position: {smoothed_x}')
            else:
                send_to_arduino(-1)
                print(f'No ball detected, sending -1')
        
def save_result(result: vision.ObjectDetectorResult, unused_output_image: mp.Image, timestamp_ms: int):
    global detection_result_list, ball_detected
    detection_result_list.append(result)
    ball_detected = bool(result.detections)  
    
# Load the object detection model
base_options = python.BaseOptions(model_asset_path='/home/daniel/Desktop/Robot_Track/My_Robot/best.tflite')
options = vision.ObjectDetectorOptions(
    base_options=base_options,
    running_mode=vision.RunningMode.LIVE_STREAM,
    max_results=5,
    score_threshold=0.65,
    result_callback=save_result
)
detector = vision.ObjectDetector.create_from_options(options)

# Function to run detection in a separate thread
def run_inference(frame, timestamp_ms):
    rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)
    detector.detect_async(mp_image, timestamp_ms)

def generate_frames():
    global detection_frame, frame_counter, smoothed_x

    while True:
        frame = picam2.capture_array()
        frame_counter += 1
        # Run detection on every 'skip_frame_count' frame
        if frame_counter % skip_frame_count == 0:
            threading.Thread(target=run_inference, args=(frame, time.time_ns() // 1_000_000)).start()

        if detection_result_list:
            if detection_result_list[0].detections:
                # Extract the first detected object (assuming it's the ball)
                detection = detection_result_list[0].detections[0]
                bbox = detection.bounding_box
                ball_center_x = int(bbox.origin_x + bbox.width / 2)

                # Apply Exponential Moving Average (EMA) to smooth the x-coordinate
                smoothed_x = exponential_moving_average(ball_center_x, smoothed_x, alpha)
                # Draw detection results on the frame
                frame = visualize(frame, detection_result_list[0])
            detection_frame = frame
            detection_result_list.clear()

        if detection_frame is not None:
            ret, buffer = cv2.imencode('.jpg', detection_frame)
        else:
            ret, buffer = cv2.imencode('.jpg', frame)

        frame = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    global video_feed_started
    video_feed_started = True
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return "Video Streaming is available at /video_feed"

if __name__ == '__main__':
    ball_position_thread = threading.Thread(target=ball_position)
    ball_position_thread.daemon = True 
    ball_position_thread.start()
        
    app.run(host='0.0.0.0', port=5000, debug=False)
    ser.close()