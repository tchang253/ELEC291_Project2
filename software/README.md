# Computer Vision Robot Controller

This is the Python host script that acts as the "brain" of the robot, using a standard laptop webcam to track hand gestures via MediaPipe and OpenCV.

## How it works:
1. The script accesses your webcam and tracks your hand landmarks.
2. It detects structural gestures (e.g., thumb pointing up, open palm, pointer finger).
3. It maps these gestures to driving commands (`F`orward, `B`ackward, `L`eft, `R`ight, `S`top).
4. **Auto-connection**: The script automatically scans your laptop's USB ports to find the EFM8 Laser Bee (or an FTDI adapter) and instantly connects to it.
5. It streams these ASCII commands over serial to the EFM8 `CV_IR_BRIDGE`, which translates them into IR pulses for the STM32 robot.

## How to run it:
1. Make sure the Python virtual environment is activated:
   ```bash
   venv\Scripts\activate
   ```
2. Plug in the EFM8 Laser Bee or your USB-to-UART FTDI cable.
3. Run the script:
   ```bash
   python robot_controller.py
   ```
4. A webcam window will pop up. 
   - ✋ **Open Palm:** Stop
   - ☝️ **Pointer Finger Up:** Forward
   - 👎 **Thumb Down:** Backward
   - 👈 **Thumb Left:** Left
   - 👉 **Thumb Right:** Right
   
Press `q` on your keyboard to close the window and gracefully disconnect the robot.
