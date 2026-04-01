import cv2
import mediapipe as mp
import time
import serial
import serial.tools.list_ports
import os

# --- Auto COM Port Detection ---
# Detects Silicon Labs (EFM8 built-in) or FTDI (external adapter)
def find_efm8_port():
    ports = serial.tools.list_ports.comports()
    if not ports:
        return None
    
    # Priority 1: Match known Vendor IDs (Silicon Labs=0x10C4, FTDI=0x0403)
    for p in ports:
        if p.vid in [0x10C4, 0x0403]:
            return p.device
            
    # Priority 2: Fallback to name search
    for p in ports:
        if p.description and ('CP210' in p.description or 'FTDI' in p.description or 'USB Serial' in p.description):
            return p.device
            
    # Priority 3: Last resort - pick the very first COM port if one is plugged in
    return ports[0].device

try:
    port = find_efm8_port()
    if port is None:
        raise IOError("No USB COM ports found! Is the USB cable plugged in?")
        
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = 115200
    ser.dtr = False   # Prevents reset loop
    ser.rts = False
    ser.open()
    print(f"[CV Bridge] Auto-detected EFM8 Bridge on {port} at 115200 baud.")
except Exception as e:
    ser = None
    print(f"[CV Bridge] WARNING: Could not connect: {e}\n  --> Running in camera-only test mode.", flush=True)

# --- MediaPipe Setup ---
model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'hand_landmarker.task')
BaseOptions = mp.tasks.BaseOptions
HandLandmarker = mp.tasks.vision.HandLandmarker
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode

latest_result = None

def print_result(result: mp.tasks.vision.HandLandmarkerResult, output_image: mp.Image, timestamp_ms: int):
    global latest_result
    latest_result = result

options = HandLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=model_path),
    running_mode=VisionRunningMode.LIVE_STREAM,
    num_hands=1,
    result_callback=print_result,
    min_hand_presence_confidence=0.5
)

# --- State Variables ---
current_command = 'S'
previous_command = ''
last_sent_time = 0
DEBOUNCE_DELAY = 0.5  # Prevent spamming the EFM8 buffer

# Low-Pass Filter Variables
candidate_command = 'S'
candidate_start_time = time.time()
GESTURE_HOLD_TIME = 0.15  # Seconds to hold pose before committing

# --- Autonomy & Macro Variables ---
global_history = []
macro_sequence = []
is_recording = False
last_toggle_time = 0

is_playing = False
is_returning_home = False
playback_sequence = []
playback_index = 0
current_playback_duration = 0
active_command_start_time = time.time()

# Initialize MediaPipe loop
with HandLandmarker.create_from_options(options) as landmarker:
    cap = cv2.VideoCapture(0)
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print("Starting video stream. Press 'q' to quit.")

    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            break

        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        
        timestamp = int(time.time() * 1000)
        landmarker.detect_async(mp_image, timestamp)
        
        current_time = time.time()

        if is_playing:
            # --- Autopilot Mode ---
            if current_time - active_command_start_time >= current_playback_duration:
                playback_index += 1
                if playback_index >= len(playback_sequence):
                    is_playing = False
                    is_returning_home = False
                    current_command = 'S'
                    print("Playback finished.")
                else:
                    current_command, current_playback_duration = playback_sequence[playback_index]
                    active_command_start_time = current_time
                    print(f"Playing '{current_command}' for {current_playback_duration:.2f}s")
            else:
                current_command, _ = playback_sequence[playback_index]
                
            if is_returning_home:
                cv2.putText(frame, f"RETURN TO HOME: {current_command}", (50, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 3)
            else:
                cv2.putText(frame, f"AUTOPILOT: {current_command}", (50, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

        else:
            # --- Manual Driving Mode ---
            raw_command = 'S' # Default to STOP if no hand is found

            if is_recording:
                cv2.circle(frame, (30, 40), 10, (0, 0, 255), -1)
                cv2.putText(frame, "REC", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            if latest_result and latest_result.hand_landmarks:
                landmarks = latest_result.hand_landmarks[0]
                
                thumb_tip_y = landmarks[4].y
                thumb_mcp_y = landmarks[2].y
                index_tip_y = landmarks[8].y
                index_mcp_y = landmarks[5].y
                middle_tip_y = landmarks[12].y
                middle_mcp_y = landmarks[9].y
                ring_tip_y = landmarks[16].y
                ring_mcp_y = landmarks[13].y
                pinky_tip_y = landmarks[20].y
                pinky_mcp_y = landmarks[17].y
                
                thumb_tip_x = landmarks[4].x
                index_tip_x = landmarks[8].x
                pinky_tip_x = landmarks[20].x

                # --- Gesture Heuristics ---
                if index_tip_y < index_mcp_y + 0.03 and index_tip_x > landmarks[5].x + 0.1:
                    raw_command = 'R'
                elif index_tip_y < index_mcp_y + 0.03 and index_tip_x < landmarks[5].x - 0.1:
                    raw_command = 'L'
                elif index_tip_y < index_mcp_y and middle_tip_y < landmarks[9].y and pinky_tip_y < landmarks[17].y:
                    raw_command = 'S'
                elif index_tip_y > index_mcp_y + 0.04 and thumb_tip_y < thumb_mcp_y - 0.04:
                    raw_command = 'F'
                elif index_tip_y > index_mcp_y + 0.04 and thumb_tip_y >= thumb_mcp_y - 0.04:
                    raw_command = 'B'

                # Draw landmarks for feedback
                for lm in landmarks:
                    h, w, _ = frame.shape
                    x, y = int(lm.x * w), int(lm.y * h)
                    cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)

            # --- Low-Pass Filter (Jitter Prevention) ---
            if raw_command == candidate_command:
                if current_time - candidate_start_time >= GESTURE_HOLD_TIME:
                    current_command = raw_command
            else:
                candidate_command = raw_command
                candidate_start_time = current_time

            # Display committed command to user
            color = (0, 255, 0) if current_command == 'F' else (0,0,255) if current_command == 'B' else (255,0,0) if current_command in ['L','R'] else (0,255,255)
            ui_text = {'F':'FORWARD', 'B':'BACKWARD', 'L':'LEFT', 'R':'RIGHT', 'S':'STOP'}.get(current_command, current_command)
            cv2.putText(frame, ui_text, (50, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

            # Sequence Recorder (Logs exact time held when committed current_command changes)
            if current_command != previous_command:
                duration = current_time - active_command_start_time
                if duration > 0.05: # Filter out sequence micro-jitters
                    step = (previous_command if previous_command != '' else 'S', duration)
                    global_history.append(step)
                    if is_recording:
                        macro_sequence.append(step)
                active_command_start_time = current_time

        # Send command over UART, with debouncing
        if current_command != previous_command or (current_time - last_sent_time > DEBOUNCE_DELAY):
            if ser is not None:
                # IMPORTANT: If Matthew wants raw bits instead of ASCII, change this to bytes
                ser.write(current_command.encode())
            previous_command = current_command
            last_sent_time = current_time

        cv2.imshow('Robot Camera Controller', frame)
        
        # --- Keyboard Inputs ---
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r') and not is_playing:
            is_recording = not is_recording
            if is_recording:
                macro_sequence.clear()
                print("Recording started.")
            else:
                print("Recording stopped.")
        elif key == ord('p') and not is_recording:
            if len(macro_sequence) > 0:
                is_playing = True
                is_returning_home = False
                playback_sequence = macro_sequence.copy()
                playback_index = 0
                current_command, current_playback_duration = playback_sequence[0]
                active_command_start_time = current_time
                print("Playing sequence...")
        elif key == ord('h') and not is_playing and not is_recording:
            # Return to Home (Reverse entire history)
            if len(global_history) > 0:
                print("Executing return-to-home sequence.")
                rev_hist = global_history[::-1] # Read array backwards
                opposite = {'F': 'B', 'B': 'F', 'L': 'R', 'R': 'L', 'S': 'S'}
                
                playback_sequence = []
                for cmd, dur in rev_hist:
                    playback_sequence.append((opposite.get(cmd, 'S'), dur))
                
                is_playing = True
                is_returning_home = True
                playback_index = 0
                current_command, current_playback_duration = playback_sequence[0]
                active_command_start_time = current_time

    cap.release()
    cv2.destroyAllWindows()
    if ser is not None:
        ser.close()
