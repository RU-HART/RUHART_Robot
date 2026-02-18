# gps_sender.py
import requests
import socketio
import random
import time

# Create a Socket.IO client
sio = socketio.Client()
SERVER_URL = 'http://127.0.0.1:5000/api/gps'

@sio.event
def connect():
    print("Connected to server!")

@sio.event
def disconnect():
    print("Disconnected from server")

# Connect to the Flask-SocketIO server
sio.connect("http://127.0.0.1:5000")

try:
    while True:
        gps_data = {
            "latitude": round(random.uniform(-90, 90), 6),
            "longitude": round(random.uniform(-180, 180), 6),
            "speed": round(random.uniform(0, 120), 2),
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
        }

        print("Sending GPS:", gps_data)

        try:
            response = requests.post(SERVER_URL, json=gps_data, timeout=2)
            if response.status_code == 200:

                print('hell yea')
            else:
                print("hell naw")
        

        except requests.exceptions.RequestException as e:
            print("bro can't connect?")

            

        time.sleep(2)

except KeyboardInterrupt:
    print("Stopping...")
    sio.disconnect()
