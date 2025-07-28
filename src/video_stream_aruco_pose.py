from flask import Flask, Response, jsonify
import cv2
import numpy as np

app = Flask(__name__)
cap = cv2.VideoCapture("/dev/jetcocam0")

camera_matrix = np.load("/home/jetcobot/dev_ws/src/calibration_matrix.npy")
dist_coeffs = np.load("/home/jetcobot/dev_ws/src/distortion_coefficients.npy")
marker_length = 0.02  # 단위: m

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

latest_tvecs = {}  # {id: tvec}

def generate_frames():
    global latest_tvecs
    while True:
        success, frame = cap.read()
        if not success:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        latest_tvecs = {}
        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)
            for i, marker_id in enumerate(ids.flatten()):
                tvec = tvecs[i][0].tolist()
                latest_tvecs[int(marker_id)] = tvec
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], marker_length * 0.5)

        _, buffer = cv2.imencode('.jpg', frame)
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/')
def index():
    return "<h1>JetCobot ArUco Stream</h1><img src='/video_feed'>"

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/pose')
def pose():
    return jsonify(latest_tvecs)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
