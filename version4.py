import cv2
import RPi.GPIO as GPIO
import time 
import os
from ultralytics import YOLO
import supervision as sv
from threading import Thread
import queue

# Load the PyTorch model
model = YOLO("best.pt")
print("Model loaded successfully!")
camera = cv2.VideoCapture(0)

green, yellow, red = 17,27,22
green_1, yellow_1, red_1 = 10, 9, 11
green_2, yellow_2, red_2  = 13, 19, 26
green_3, yellow_3, red_3  = 16, 20, 21
IN1, IN2, IN3, IN4 = 7, 8, 25, 24

GPIO.setmode(GPIO.BCM)
GPIO.setup([IN1, IN2, IN3, IN4, green, yellow, red, green_1, yellow_1, red_1, 
green_2, yellow_2, red_2, green_3, yellow_3, red_3], GPIO.OUT)

SEQUENCE = [
    [1, 0, 0, 1],
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1]
]

# Function to rotate the motor 360 degrees with pauses at each cardinal direction
def clockwise_capture(camera):
    capture_image(camera, 'Lane1')
    time.sleep(2)
    for direction, angle in [('Lane2', 90), ('Lane3', 180), ('Lane4', 270)]:
        for _ in range(128):  # 128 steps for 90 degrees (adjust as needed)
            for step in SEQUENCE:
                set_step(step)
                time.sleep(0.001)  # Adjust delay if needed
        time.sleep(2)  # Pause for 5 seconds at each cardinal direction
        capture_image(camera, direction)
        time.sleep(2)

def anti_clockwise_capture(camera):
    capture_image(camera, 'Lane4')
    time.sleep(2)
        # Rotate in reverse direction
    for direction, angle in [('Lane3', 180), ('Lane2', 90), ('Lane1', 0)]:
        for _ in range(128):  # 128 steps for 90 degrees (adjust as needed)
            for step in reversed(SEQUENCE):  # Reverse the sequence for reverse direction
                set_step(step)
                time.sleep(0.001)  # Adjust delay if needed
        time.sleep(2)  # Pause for 5 seconds at each cardinal direction
        capture_image(camera, direction)
        time.sleep(2)

# Function to capture an image with the camera and save it with a specific name
def capture_image(camera, direction):
    image_name = f"{direction.lower()}.jpg"
    _, frame = camera.read()
    cv2.imwrite(image_name, frame)
    print(f"Captured image: {image_name}")

def set_step(step):
    GPIO.output(IN1, step[0])
    GPIO.output(IN2, step[1])
    GPIO.output(IN3, step[2])
    GPIO.output(IN4, step[3])


def clear_all():
	GPIO.output(green, GPIO.LOW)
	GPIO.output(yellow, GPIO.LOW)
	GPIO.output(red, GPIO.LOW)
	GPIO.output(green_1, GPIO.LOW)
	GPIO.output(yellow_1, GPIO.LOW)
	GPIO.output(red_1, GPIO.LOW)
	GPIO.output(green_2, GPIO.LOW)
	GPIO.output(yellow_2, GPIO.LOW)
	GPIO.output(red_2, GPIO.LOW)
	GPIO.output(green_3, GPIO.LOW)
	GPIO.output(yellow_3, GPIO.LOW)
	GPIO.output(red_3, GPIO.LOW)

def detect(output_queue):
	#list to contain the number of vehicles in easc picture
	vehicle_list = []
	box_annotator = sv.BoxAnnotator(thickness=2,text_thickness=2,text_scale=1)
	# Get the path to the folder containing the images
	folder_path = "./"
	# List all files in the folder
	image_files = [f for f in os.listdir(folder_path) if f.endswith(('.jpg', '.png', '.jpeg'))]
	for image_file in image_files:
		# Construct the full path to the image
		image_path = os.path.join(folder_path, image_file)
		# Read the image
		frame = cv2.imread(image_path)
		# Run predictions on the image
		result = model(frame)[0]
		detections = sv.Detections.from_ultralytics(result)
		num_detections = len(detections)
		vehicle_list.append(num_detections)
	output_queue.put(vehicle_list)

def control(vl1, vl2, vl3, vl4):
	# vl1 = number of vehicles in lane 1
	# vl2 = number of vehicles in lane 2
	# vl3 = number of vehicles in lane 3
	# vl4 = number of vehicles in lane 
	dfg_1, dfg_2, dfg_3, dfg_4 = 5,5,5,5
	
	if vl1 > 5:
		dfg_1 = dfg_1 + vl1*0.5
	if vl2 > 5:
		dfg_2 = dfg_2 + vl2*0.5
	if vl3 > 5:
		dfg_3 = dfg_3 + vl3*0.5
	if vl4 > 5:
		dfg_4 = dfg_4 + vl4*0.5
	
	
	
	clear_all()	
	#Lane 1 Green
	GPIO.output(green, GPIO.HIGH)
	GPIO.output(red_1, GPIO.HIGH)
	GPIO.output(red_2, GPIO.HIGH)
	GPIO.output(red_3, GPIO.HIGH)
	time.sleep(dfg_1)
	#Lane 1 and 2 wait
	GPIO.output(green, GPIO.LOW)
	GPIO.output(red_1, GPIO.LOW)
	GPIO.output(red_2, GPIO.HIGH)
	GPIO.output(red_3, GPIO.HIGH)
	GPIO.output(yellow, GPIO.HIGH)
	GPIO.output(yellow_1, GPIO.HIGH)
	time.sleep(2)
	#lane 2 green and all others red
	GPIO.output(yellow, GPIO.LOW)
	GPIO.output(yellow_1, GPIO.LOW)
	GPIO.output(red, GPIO.HIGH)
	GPIO.output(red_2, GPIO.HIGH)
	GPIO.output(red_3, GPIO.HIGH)
	GPIO.output(green_1, GPIO.HIGH)
	time.sleep(dfg_2)
	#lane 2 and 3 wait and all others red
	GPIO.output(red, GPIO.HIGH)
	GPIO.output(red_2, GPIO.LOW)
	GPIO.output(red_3, GPIO.HIGH)
	GPIO.output(yellow_1, GPIO.HIGH)
	GPIO.output(yellow_2, GPIO.HIGH)
	GPIO.output(green_1, GPIO.LOW)
	time.sleep(2)
	#lane 3 green and all others red
	GPIO.output(green_2, GPIO.HIGH)
	GPIO.output(red, GPIO.HIGH)
	GPIO.output(red_1, GPIO.HIGH)
	GPIO.output(yellow_1, GPIO.LOW)
	GPIO.output(yellow_2, GPIO.LOW)
	GPIO.output(red_3, GPIO.HIGH)
	time.sleep(dfg_3)
	#lane 3 and 4 yellow and all others red
	GPIO.output(yellow_2, GPIO.HIGH)
	GPIO.output(yellow_3, GPIO.HIGH)
	GPIO.output(red, GPIO.HIGH)
	GPIO.output(red_1, GPIO.HIGH)
	GPIO.output(green_2, GPIO.LOW)
	GPIO.output(red_3, GPIO.LOW)
	time.sleep(2)
	#lane 4 green and other red
	GPIO.output(yellow_2, GPIO.LOW)
	GPIO.output(yellow_3, GPIO.LOW)
	GPIO.output(red, GPIO.HIGH)
	GPIO.output(red_1, GPIO.HIGH)
	GPIO.output(green_3, GPIO.HIGH)
	GPIO.output(red_2, GPIO.HIGH)
	GPIO.output(red_3, GPIO.LOW)
	time.sleep(dfg_4)
	#lane 4 and 1 yellow and others red
	GPIO.output(yellow, GPIO.HIGH)
	GPIO.output(yellow_3, GPIO.HIGH)
	GPIO.output(red, GPIO.LOW)
	GPIO.output(red_1, GPIO.HIGH)
	GPIO.output(green_3, GPIO.LOW)
	GPIO.output(red_2, GPIO.HIGH)
	time.sleep(2)
try:
	vl1, vl2, vl3, vl4 = 5,5,5,5
	while True:

		# create a multiprocessing Queue for communication
		output_queue = queue.Queue()
		
		
		# Start capture and control process
		fwdcapture_process = Thread(target=clockwise_capture, args =(camera,))
		control_process = Thread(target=control, args =(vl1,vl2,vl3,vl4))
		
		fwdcapture_process.start()
		control_process.start()
		
		# wait for capture to finish
		
		fwdcapture_process.join()

		# start the detect process
		detect_process = Thread(target=detect, args = (output_queue,))
		detect_process.start()
		 # wait for detect to finish
		detect_process.join()
		
		# Retrieve output of detect process
		vehicle_list = output_queue.get()
		
		vl1=vehicle_list[0]
		vl2=vehicle_list[1]
		vl3=vehicle_list[2]
		vl4=vehicle_list[3]
		control_process.join()
		
		# Reverse movement
		bwdcapture_process = Thread(target=anti_clockwise_capture, args =(camera,))
		control_process = Thread(target=control, args =(vl1,vl2,vl3,vl4))
		
		bwdcapture_process.start()
		control_process.start()
		
		# wait for capture to finish
		
		bwdcapture_process.join()

		# start the detect process
		detect_process = Thread(target=detect, args = (output_queue,))
		detect_process.start()
		 # wait for detect to finish
		detect_process.join()
		
		# Retrieve output of detect process
		vehicle_list = output_queue.get()
		
		vl1=vehicle_list[0]
		vl2=vehicle_list[1]
		vl3=vehicle_list[2]
		vl4=vehicle_list[3]
		control_process.join()
		
		
		
	

except KeyboardInterrupt:
	GPIO.cleanup()
