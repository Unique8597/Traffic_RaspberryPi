
# Car Density-based Traffic Control using Raspberry Pi

This project implements a dynamic traffic control system based on real-time car density data collected using a Raspberry Pi and camera. The system aims to optimize traffic flow and reduce congestion by adjusting traffic light durations based on the number of vehicles waiting at each lane.

### Features

* **Real-time car detection and counting:** Uses OpenCV to track and count vehicles in each lane.
* **Dynamic traffic light control:** Adjusts green light duration based on car density, prioritizing lanes with higher traffic.
* **Raspberry Pi integration:** Runs efficiently on a low-cost and compact platform.
* **Modular design:** Easy to adapt and extend for further functionalities.

### Hardware Requirements

* Raspberry Pi 4 or equivalent
* Raspberry Pi Camera Module
* Power supply and cables
* Micro SD card
* Monitor (optional)

### Software Requirements

* Raspbian OS
* OpenCV
* Python 3
* Ultralytics

### Installation and Setup

1. **Set up Raspbian OS:** Install Raspbian OS on your Micro SD card and boot it on your Raspberry Pi.
2. **Enable camera:** Enable the Raspberry Pi Camera Module using `raspi-config`.
3. **Install dependencies:** Install OpenCV using `sudo apt-get install python3-opencv`.
4. **Clone the repository:** Clone this repository to your Raspberry Pi using `git clone https://github.com/Unique8597/Traffic_RaspberryPi.git`.
5. **Install Python dependencies:** Install the required Python libraries using `pip install -r requirements.txt`.

### Running the application

1. Run the main script: `python version4.py`.
2. The script will start processing the video feed from the camera and adjust the traffic light durations based on the detected car density.

### Additional notes

* This project currently uses a simple threshold-based approach for traffic light control. More sophisticated algorithms can be implemented for improved performance.
* The project can be further extended to support multiple intersections and communication between traffic lights.

### Additional Resources

* OpenCV: [https://opencv.org/](https://opencv.org/)
* Raspberry Pi: [https://www.raspberrypi.org/](https://www.raspberrypi.org/)
