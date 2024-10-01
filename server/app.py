from flask import Flask, request
from flask_socketio import SocketIO, emit
import socket
import threading
import time

# Create Flask app and initialize SocketIO for WebSocket communication
app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

# UDP settings
UDP_IP = "0.0.0.0"  # Listen on all network interfaces
UDP_PORT = 5005      # Port for UDP communication
BUFFER_SIZE = 1024   # Buffer size for UDP packets

DEST_IP = "127.0.0.1"
DEST_PORT = 5005

# Create a UDP socket
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

streaming = False
streaming_thread = None
client_ip = None

def send_udp_data():
    global streaming
    counter = 0
    while streaming:
        message = f"Streaming UDP data: {counter}"
        udp_socket.sendto(message.encode(), (client_ip, DEST_PORT))
        counter += 1
        print(f"Sent UDP data: {message}")
        time.sleep(0.01)


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
