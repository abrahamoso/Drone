import cv2
import mediapipe as mp
import time
from threading import Thread
import queue
import serial
import time
import serial


class ArduinoCommunicator:
    def __init__(self, port='COM3', baud_rate=9600):  # Change COM3 to your Arduino port
        try:
            self.arduino = serial.Serial(port, baud_rate)
            print(f"Connected to Arduino on {port}")
            time.sleep(2)  # Wait for Arduino to reset
        except serial.SerialException as e:
            print(f"Failed to connect to Arduino: {e}")
            self.arduino = None
    
    def send_command(self, command):
        if self.arduino:
            try:
                # Send command as bytes
                self.arduino.write(f"{command}\n".encode())
                print(f"Sent to Arduino: {command}")
            except Exception as e:
                print(f"Error sending command: {e}")
    
    def close(self):
        if self.arduino:
            self.arduino.close()
class VideoStreamThread:
    def __init__(self, src=0):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.q = queue.Queue(maxsize=2)
        self.stopped = False
        
    def start(self):
        Thread(target=self.update, daemon=True).start()
        return self
    
    def update(self):
        while not self.stopped:
            if not self.q.full():
                ret, frame = self.cap.read()
                if not ret:
                    self.stop()
                    break
                while self.q.full():
                    self.q.get()
                self.q.put(frame)
            else:
                time.sleep(0.001)
    
    def read(self):
        return self.q.get() if not self.q.empty() else None
    
    def stop(self):
        self.stopped = True
        self.cap.release()

class HandGestureDetector:
    def __init__(self):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,  # Detecting only one hand for better performance
            min_detection_confidence=0.7,  # Increased confidence threshold
            min_tracking_confidence=0.7
        )
        self.mp_draw = mp.solutions.drawing_utils
    
    def detect_gesture(self, hand_landmarks):
        # Get important finger landmarks
        thumb_tip = hand_landmarks.landmark[4]
        index_tip = hand_landmarks.landmark[8]
        middle_tip = hand_landmarks.landmark[12]
        ring_tip = hand_landmarks.landmark[16]
        pinky_tip = hand_landmarks.landmark[20]
        wrist = hand_landmarks.landmark[0]

        # Get finger bases
        index_base = hand_landmarks.landmark[5]
        middle_base = hand_landmarks.landmark[9]
        ring_base = hand_landmarks.landmark[13]
        pinky_base = hand_landmarks.landmark[17]

        # Define threshold for finger raised/lowered
        RAISED_THRESHOLD = 0.1
        
        def is_finger_raised(tip, base):
            return (base.y - tip.y) > RAISED_THRESHOLD

        # Check hand position
        if wrist.y > index_tip.y:  # Hand is raised
            # LEFT gesture
            if (thumb_tip.x < wrist.x - 0.1 and 
                thumb_tip.y < wrist.y and 
                not is_finger_raised(index_tip, index_base)):
                return "LEFT"
            
            # RIGHT gesture
            elif (thumb_tip.x > wrist.x + 0.1 and 
                  thumb_tip.y < wrist.y and 
                  not is_finger_raised(index_tip, index_base)):
                return "RIGHT"
        
        # STOP gesture
        if (is_finger_raised(index_tip, index_base) and 
            not is_finger_raised(middle_tip, middle_base) and 
            not is_finger_raised(ring_tip, ring_base) and 
            is_finger_raised(pinky_tip, pinky_base)):
            return "STOP"
        
        # UP gesture
        elif (is_finger_raised(index_tip, index_base) and 
              is_finger_raised(middle_tip, middle_base) and 
              not is_finger_raised(ring_tip, ring_base) and 
              not is_finger_raised(pinky_tip, pinky_base)):
            return "UP"
        
        # DOWN gesture
        elif (is_finger_raised(index_tip, index_base) and 
              not is_finger_raised(middle_tip, middle_base) and 
              not is_finger_raised(ring_tip, ring_base) and 
              not is_finger_raised(pinky_tip, pinky_base)):
            return "DOWN"

        return None

    def process_frame(self, frame):
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_rgb.flags.writeable = False
        results = self.hands.process(frame_rgb)
        frame_rgb.flags.writeable = True
        
        gesture = None
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(
                    frame,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_draw.DrawingSpec(color=(255, 0, 0), thickness=2),
                    self.mp_draw.DrawingSpec(color=(0, 255, 0), thickness=2)
                )
                
                gesture = self.detect_gesture(hand_landmarks)
                if gesture:
                    cv2.putText(frame, f"Gesture: {gesture}", (10, 60),
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    
        return frame, gesture

    def close(self):
        self.hands.close()

def main():
    CAMERA_INDEX = 1  # Adjust this based on your camera setup
    
    print("Starting video stream...")
    vs = VideoStreamThread(CAMERA_INDEX).start()
    time.sleep(1.0)

    hand_detector = HandGestureDetector()
    arduino = ArduinoCommunicator()  # Initialize Arduino communication
    
    frame_count = 0
    start_time = time.time()
    last_gesture = None
    gesture_duration = 0

    try:
        while True:
            frame = vs.read()
            if frame is None:
                continue

            try:
                frame, gesture = hand_detector.process_frame(frame)
                
                if gesture:
                    if gesture != last_gesture:
                        print(f"Detected gesture: {gesture}")
                        arduino.send_command(gesture)  # Send gesture to Arduino
                        last_gesture = gesture
                        gesture_duration = time.time()
                elif last_gesture and time.time() - gesture_duration > 2:
                    last_gesture = None

                # Calculate and display FPS
                frame_count += 1
                if frame_count % 30 == 0:
                    fps = frame_count / (time.time() - start_time)
                    cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                cv2.imshow("Hand Gesture Detection", frame)

            except Exception as e:
                print(f"Error during detection: {e}")

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        vs.stop()
        cv2.destroyAllWindows()
        hand_detector.close()
        arduino.close()  # Close Arduino connection when done

if __name__ == "__main__":
    main()