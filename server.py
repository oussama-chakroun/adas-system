import cv2
import serial
import time
import socket
import pickle
import threading

# Set the IP address and port for the UDP stream
udp_address = "192.168.137.2"  # Replace with the IP address of the receiver PC
udp_port = 9999

# Create a VideoCapture object for capturing video from the webcam
cap = cv2.VideoCapture(0)

# Check if the video capture is successfully opened
if not cap.isOpened():
    print("Failed to open the video capture.")
    exit()

# Set the desired frame size (smaller resolution)
frame_width = 700
frame_height = 500

# Set the JPEG compression parameters
jpeg_quality = 80  # Adjust the quality level as needed

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_frames():
    while True:
        # Read a frame from the video capture
        ret, frame = cap.read()

        # Check if a frame was successfully read
        if not ret:
            print("Failed to read a frame from the video capture.")
         # Resize the frame to the desired size
        frame = cv2.resize(frame, (frame_width, frame_height))

        # Compress the frame as a JPEG image
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality]
        _, encoded_frame = cv2.imencode('.jpg', frame, encode_param)
        data = pickle.dumps(encoded_frame)
    
        # Send the frame over UDP to the receiver IP address and port
        sock.sendto(data, (udp_address, udp_port))

        # Check for the 'q' key press to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
if __name__ == '__main__':
    intialTrackBarVals = [52, 126, 70, 170 ]
    valide = False
    port_names = ['/dev/ttyACM1']
    try:
        ser = serial.Serial(port_names[0], 115200, timeout=1.0)
        time.sleep(3)
        ser.reset_input_buffer()
        print("success")
        valide = True
    except:
        print("Close Serial communication.")
        valide = False
        
        
    if(valide) :
        # Start a separate thread to send frames continuously
        send_thread = threading.Thread(target=send_frames)
        send_thread.start()
        while True:
            # Receive the response message from the receiver
            response, _ = sock.recvfrom(1024)
            latest_response = response.decode()
            if(latest_response == "forward") :
                ser.write("forward\n".encode('utf-8'))
            elif latest_response == "right":
                ser.write("right\n".encode('utf-8'))
            elif latest_response == "left" :
                ser.write("left\n".encode('utf-8'))
            elif latest_response == "rr":
                ser.write("rr\n".encode('utf-8'))
            elif latest_response == "ll":
                ser.write("ll\n".encode('utf-8'))
            elif latest_response == "stop":
                ser.write("stop\n".encode('utf-8'))
            elif latest_response == "Red":
                ser.write("Red\n".encode('utf-8'))
            elif latest_response == "Green":
                ser.write("Green\n".encode('utf-8'))
            elif latest_response == "Orange":
                ser.write("Orange\n".encode('utf-8'))        

            
    else :
        print("error")
    # Release the video capture and close the UDP socket
    cap.release()
    sock.close()
    cv2.destroyAllWindows()



