# Competitive Multi-Robot Parking Game  
**Vision-Based Localization and Gesture-Based Remote Control**  
*A fast-paced, interactive parking game using four Zumo robots on a 4x4 ft LED-marked field.*

---

## Key Features  
- **Multi-Robot Competition**: Four Zumo robots compete to occupy 10 randomly generated spots in each round.  
- **Vision-Based Localization**: PixyCam tracks robot positions via unique color tags.  
- **Gesture-Based Control**: IMU-driven remote controllers for intuitive robot steering.  
- **Real-Time Feedback**: Dynamic LED indicators for parking spots, haptic vibration feedback, and OLED status displays.  
- **Modular Architecture**: Built with STM32 NUCLEO microcontrollers for scalability and integration.  

---

## Hardware Components  
### Remote Controller System (NUCLEO-L432KC)  
- **IMU (GY-511)**: Measures pitch/roll for gesture input.  
- **Piezoresistor**: Emergency stop trigger.  
- **Bluetooth (HC-05)**: Transmits control signals.  
- **Micro Vibrator**: Haptic feedback for velocity.  
![cad](https://github.com/user-attachments/assets/2f2345de-ff33-487f-ae92-95de54f677c7)

### Zumo Car System (NUCLEO-F401RE)  
- **IMU (GY-511)**: Measures yaw for PID-based motion control.  
- **Motor Driver**: PWM-controlled wheels with velocity feedback.  
- **Bluetooth (HC-05)**: Receives control signals.  

### Playground System (NUCLEO-L4R5ZI)  
- **PixyCam**: Tracks robot positions via color tags.  
- **LED Strips**: Indicates parking spot availability (white = available, color = occupied).  
- **SPI/UART**: Communicates with Zumo cars and remote controllers.  

---

## System Architecture  
1. **Playground**  
   - Generates 10 random parking spots per round.  
   - Updates LED colors in real-time based on occupancy.  
   - Update scoring by checking robot positions and spot assignments.  

2. **Remote Controller**  
   - Converts IMU gestures (pitch/roll) into speed/direction commands.
   - Converts FSR reading into immediate stop command.  
   - Sends commands via Bluetooth and triggers emergency stops.  

3. **Zumo Car**  
   - Executes PID-controlled motion using IMU yaw data.  
   - Reports velocity feedback to the remote controller system via Bluetooth.
![diagram](https://github.com/user-attachments/assets/5d60b5c6-d284-4428-9f7e-ccc66177fe8c)

---

## Game Rules  
- **Players**: 4 Zumo cars (each with a unique color).  
- **Rounds**: 3 trials for each round; highest total score wins.  
- **Parking Spots**: 10 spots per round (randomly generated, indicated by white LEDs).  
- **Time Limit**: 60 seconds per round.  
- **Scoring**: A robot scores only if it succuessfully occupies an available spot and turns that spot into its own color.  
![real](https://github.com/user-attachments/assets/40f72aa2-bee4-4873-9991-cee585e48b1c)

---

## Getting Started  
1. **Hardware Setup**  
   - Assemble Zumo robots with STM32 boards, and Bluetooth modules.  
   - Deploy LED strips and PixyCam on the 4x4 ft arena.  
   - Pair remote controllers with robots via Bluetooth.  

2. **Software Dependencies**  
   - STM32 HAL libraries for microcontroller programming.  
   - PixyCam Open Source Library (modified for STM usage) source at https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:arduino_api
