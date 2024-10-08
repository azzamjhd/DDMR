from flask import Flask, request
from flask_socketio import SocketIO, emit
import socket
import threading
import time
import serial
import serial.tools.list_ports

# Create Flask app and initialize SocketIO for WebSocket communication
app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

# Serial port settings
ser = None
stop_flag = threading.Event()  # Flag to stop the serial port monitoring thread
connected_clients = 0  # Track the number of connected clients

def find_serial_port():
    global ser
    while not stop_flag.is_set():
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            print(f"Found serial port: {port.device}")
            try:
                if ser is None:  # Only try to connect if not already connected
                    ser = serial.Serial(port.device, 115200, timeout=1)
                    print(f"Connected to {port.device}")
                    return True  # Indicate successful connection
            except (OSError, serial.SerialException):
                continue  # Try the next port
        print("No serial port found. Retrying in 5 seconds...")
        time.sleep(5)
    return False  # Indicate failure to connect

# Start searching for serial port in a separate thread
def monitor_serial_port():
    global streaming, ser
    while not stop_flag.is_set():
        if ser is None or not ser.is_open:
            print("Serial port disconnected, stopping streaming.")
            stop_streaming()
            if find_serial_port():  # Try to reconnect
                print("Reconnected to serial port.")
                if connected_clients > 0:  # Auto-restart if clients are connected
                    start_streaming()
        time.sleep(5)

# Start monitoring the serial port in a separate thread
serial_monitor_thread = threading.Thread(target=monitor_serial_port)
serial_monitor_thread.start()

# UDP settings
DEST_PORT = 5005

# Create a UDP socket
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

streaming = False
streaming_thread = None
client_ip = None

def send_udp_data():
    global streaming, ser
    while streaming:
        if ser:
            try:
                if ser.in_waiting > 0:
                    serial_data = ser.readline().decode('utf-8', errors="ignore").strip()
                    timestamp = time.strftime("%H:%M:%S", time.gmtime()) + f":{int(time.time() * 1000) % 1000:03d}"
                    message = f"{timestamp}: {serial_data}"
                    udp_socket.sendto(message.encode(), (client_ip, DEST_PORT))
                    # print(f"Sent UDP data: {message}")
            except serial.SerialException:
                print("Serial port disconnected or error occurred, stopping streaming.")
                stop_streaming()  # Stop streaming if the serial connection fails
                ser = None
                # find_serial_port()  # Try to reconnect
                
                # serial_monitor_thread.join()  # Wait for the serial monitor thread to finish
                # serial_monitor_thread.start()  # Restart the serial monitor thread
                
                if ser:  # If reconnection successful, restart streaming
                    start_streaming()
                break  # Exit the loop to prevent repeated error messages
            except Exception as e:
                print(f"Error reading from serial port: {e}")
        time.sleep(0.01)

def start_streaming():
    global streaming, streaming_thread
    if ser and not streaming:
        streaming = True
        streaming_thread = threading.Thread(target=send_udp_data)
        streaming_thread.start()
        print("Started streaming UDP data")

def stop_streaming():
    global streaming
    if streaming:
        streaming = False  # Just set streaming to False, the thread will stop automatically
        print("Stopped streaming UDP data")

# WebSocket (SocketIO) handler for receiving commands from Godot client
@socketio.on('command')
def handle_command(command):
    print(f"Received command from WebSocket: {command}")
    if command == "start":
        start_streaming()
    elif command == "stop":
        stop_streaming()

@socketio.on('closeLoop')
def handle_closeLoop(data):
    if ser:
        x = data['x'] * 1000
        w = data['w'] * 1000
        command = f"c {int(x)} {int(w)}\n"
        ser.write(command.encode())
        print(f"Sent command: {command}")

@socketio.on('openLoop')
def handle_openLoop(data):
    if ser:
        pwmR = data['pwmR']
        pwmL = data['pwmL']
        command = f"o {pwmR} {pwmL}\n"
        ser.write(command.encode())
        print(f"Sent command: {command}")

@socketio.on('pidGains')
def handle_pidGains(data):
    if ser:
        kp = data['kp']
        ki = data['ki']
        kd = data['kd']
        command = f"p {kp} {ki} {kd}\n"
        ser.write(command.encode())
        print(f"Sent command: {command}")

@socketio.on('message')
def handle_message(message):
    print(f"Received message from WebSocket: {message}")

# WebSocket (SocketIO) handler for client connections
@socketio.on('connect')
def handle_connect():
    global client_ip, connected_clients
    client_ip = request.remote_addr
    connected_clients += 1
    print(f"Client connected via WebSocket: {client_ip}, Total clients: {connected_clients}")
    
    if connected_clients == 1:  # Start streaming only if the first client connects
        if ser:
            start_streaming()

    emit('response', "Connected to WebSocket server")

# WebSocket (SocketIO) handler for client disconnections
@socketio.on('disconnect')
def handle_disconnect():
    global connected_clients
    connected_clients -= 1
    print(f"Client disconnected, Total clients: {connected_clients}")
    
    if connected_clients == 0:  # Stop streaming when the last client disconnects
        stop_streaming()

# Start the Flask-SocketIO server
if __name__ == "__main__":
    print("Starting Flask WebSocket (SocketIO) server...")
    try:
        socketio.run(app, host="0.0.0.0", port=5000)
    finally:
        stop_flag.set()  # Stop the serial port monitoring threads before exiting
        serial_monitor_thread.join()
