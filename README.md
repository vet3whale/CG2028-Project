# CG2028 Fall Detection System for Long Lie
The mission was the eliminate long-lie incidents by designing a device that detects falls, recognises when help is needed, and triggers timely assistance. We implemented this using the STM32 Board, utilising its multiple peripherals via BSP drivers, and other tools to develop a well-rounded system.

## Files relevant to project: 
1. `fall_combo.py` - main Python bridge between UART <--> (Telegram + Video Capture and Upload)
2. `csvtoFrames.py` - converts a captured IMU CSV into an MP4 video
3. `comms_check.py` - quick UART handshake test (not used for fall detection)
4. `human.obj` - 3d model used for rendering the video to make it more realistic

## Prerequisites 
### Hardware 
- STM32 board
- USB cable (to connect STM32 board to laptop)
- Laptop/VM (VM will be better since it always needs to be turned on)

### Software
- Python 3.10+ recommended (check with `python --version`)
- A working Telegram bot token + chat id (use BotFather on Telegram to configure)
- Terminal/Powershell

## Python dependencies
Install required packages: 
```bash
pip install pyserial requests numpy trimesh scipy pyrender imageio[ffmpeg]
```

## Instructions to use: 
### For STM32 
1. Plug in the STM32 go into STM32CubeIDE to load your workspace and project
2. Once done, Click Run --> Debug As --> Select the C/C++ Application --> Proceed --> Switch. Once you see `Download verified successfully` in your STM32CubeIDE terminal, the STM32 board is ready to run.
### For Python
3. Open `fall_combo.py` and update the BOT_TOKEN, CHAT_ID. Update AMBULANCE_NUMBER only if you want to point it to another contact number.
4. (Optional) To know you have established a UART connection, you can run `python comms_check.py`. If you get `ACK: Python Ready` and `BEEP FA`, then you know that UART has been established
5. Run `python fall_combo.py`, followed by running main.c in STM32CubeIDE. You should see this in the python terminal:
```bash
PS C:\Users\User\Downloads> python .\fall_combo.py
Auto-detected port: COM3 (STMicroelectronics STLink Virtual COM Port (COM3))
Listening on COM3 @ 115200...
Sent: BEEP FA
[STM32] Stand up straight and wear the device
[STM32] Calibrating Barometer... Keep the board stationary
[STM32] Calibration done!
```
The system is now active. 

## STM32 Peripherals Used:
1. Acclerometer - measure linear acceleration and gravity
2. Gyroscope - measure angular velocity
3. Barometer - measure the altitude/fall from height
4. RESET button - to reset the system to detect fall again
5. USER button - when pressed, will notify family members that the faller requires assistance
6. LED - blinks normally at 1 blink per second, and blinks fast when user has fallen

## Add-ons to the system: 
1. Telegram Bot - for users to get notified when someone has fallen at home
2. Telegram Group Chat - to add family members so everyone is notified
3. 3D rendering - generate a 3d visualisation of how the person falls
4. Buzzer - buzzes to notify the faller that either help is on the way or that the family member has dismissed it as a false alarm

