from flask import Flask, request
from flask_socketio import SocketIO, emit
import socket
import threading
import time

# Create Flask app and initialize SocketIO for WebSocket communication
app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

## Serial port settings
def find_serial_port():
    import serial.tools.list_ports
    while True:
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            try:
                ser = serial.Serial(port.device, 115200, timeout=1)
                print(f"Connected to {port.device}")
                return ser
            except (OSError, serial.SerialException):
                pass
        print("No serial port found. Retrying in 5 seconds...")
        time.sleep(5)

ser = find_serial_port()

# UDP settings
DEST_PORT = 5005

# Create a UDP socket
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

streaming = False
streaming_thread = None
client_ip = None

def send_udp_data():
    global streaming
    while streaming:
        if ser and ser.in_waiting > 0:
            try:
                serial_data = ser.readline().decode('utf-8', errors="ignore").strip()
                timestamp = time.strftime("%H:%M:%S", time.gmtime()) + f":{int(time.time() * 1000) % 1000:03d}"
                message = f"{timestamp}: {serial_data}"
                udp_socket.sendto(message.encode(), (client_ip, DEST_PORT))
                print(f"Sent UDP data: {message}")
            except Exception as e:
                print(f"Error reading from serial port: {e}")
        # time.sleep(0.01)


# WebSocket (SocketIO) handler for receiving commands from Godot client
@socketio.on('command')
def handle_command(command):
    global streaming, streaming_thread
    print(f"Received command from WebSocket: {command}")
    if command == "start":
        if not streaming:
            streaming = True
            streaming_thread = threading.Thread(target=send_udp_data)
            streaming_thread.start()
            print("Started streaming UDP data")
    elif command == "stop":
        if streaming:
            streaming = False
            streaming_thread.join()
            print("Stopped streaming UDP data")

@socketio.on('closeLoop')
def handle_closeLoop(data):
    x = data['x'] * 1000
    w = data['w'] * 1000
    command = f"c {int(x)} {int(w)}\n"
    ser.write(command.encode())
    print(f"Sent command: {command}")

@socketio.on('openLoop')
def handle_openLoop(data):
    pwmR = data['pwmR']
    pwmL = data['pwmL']
    command = f"o {pwmR} {pwmL}\n"
    ser.write(command.encode())
    print(f"Sent command: {command}")

@socketio.on('pidGains')
def handle_pidGains(data):
    kp = data['kp']
    ki = data['ki']
    kd = data['kd']
    command = f"p {kp} {ki} {kd}\n"
    ser.write(command.encode())
    print(f"Sent command: {command}")

@socketio.on('message')
def handle_message(message):
    print(f"Received message from WebSocket: {message}")
    # You can process the message and send it to the robot or other systems.

# WebSocket (SocketIO) handler for client connections
@socketio.on('connect')
def handle_connect():
    global client_ip
    client_ip = request.remote_addr
    print("Client connected via WebSocket: ", client_ip)
    emit('response', "Connected to WebSocket server")

# WebSocket (SocketIO) handler for client disconnections
@socketio.on('disconnect')
def handle_disconnect():
    print("Client disconnected from WebSocket")

# Start the Flask-SocketIO server
if __name__ == "__main__":
    print("Starting Flask WebSocket (SocketIO) server...")
    socketio.run(app, host="0.0.0.0", port=5000)
