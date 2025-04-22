# Competitive Multi-Robot Parking Game  
**Vision-Based Localization and Gesture-Based Remote Control**  
*A fast-paced, interactive parking game using four Zumo robots on a 4x4 ft LED-marked field.*

---

## Key Features  
- **Multi-Robot Competition**: Four Zumo robots compete to park in dynamically generated spots.  
- **Vision-Based Localization**: PixyCam tracks robot positions via unique color tags.  
- **Gesture-Based Control**: IMU-driven remote controllers for intuitive robot steering.  
- **Real-Time Feedback**: Dynamic LED indicators for parking spots, haptic vibration alerts, and OLED status displays.  
- **Modular Architecture**: Built with STM32 microcontrollers for scalability and integration.  

---

## Hardware Components  
### Remote Controller System (NUCLEO-F401RE)  
- **IMU (GY-511)**: Measures pitch/roll for gesture input.  
- **Piezoresistor**: Emergency stop trigger.  
- **Bluetooth (HC-05)**: Transmits control signals.  
- **Micro Vibrator**: Haptic feedback for collisions/errors.  

### Zumo Car System (NUCLEO-L432KC)  
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
   - Validates scoring by checking robot positions and spot assignments.  

2. **Remote Controller**  
   - Converts IMU gestures (pitch/roll) into speed/direction commands.  
   - Sends commands via Bluetooth and triggers emergency stops.  

3. **Zumo Car**  
   - Executes PID-controlled motion using IMU yaw data.  
   - Reports velocity feedback to the playground system.  

---

## Game Rules  
- **Players**: 4 Zumo cars (each with a unique color).  
- **Rounds**: 5 rounds; highest total score wins.  
- **Parking Spots**: 10 spots per round (randomly generated, indicated by white LEDs).  
- **Time Limit**: 30 seconds per round.  
- **Scoring**: A robot scores only if it parks in a spot that matches its assigned color.  
- **Warnings**: Players cannot touch robots, the arena, or game elements during play.  

---

## Getting Started  
1. **Hardware Setup**  
   - Assemble Zumo robots with STM32 boards, and Bluetooth modules.  
   - Deploy LED strips and PixyCam on the 4x4 ft arena.  
   - Pair remote controllers with robots via Bluetooth.  

2. **Software Dependencies**  
   - STM32 HAL libraries for microcontroller programming.  
   - PixyCam Open Source Library (modified for STM usage) source at https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:arduino_api 
