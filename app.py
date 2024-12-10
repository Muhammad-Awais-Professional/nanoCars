import socket
import threading
import math
import time
from flask import Flask, jsonify, render_template, request
from functools import partial

# ---------- CONFIG ----------
HOST = '0.0.0.0'
PORT = 12345
# ----------------------------

clients = []
lock = threading.Lock()

car_states = {}

# Define more robust tricks with carefully timed steps.
# Each trick is a list of (command, duration) steps.
# After each step, a 'S' (stop) command is issued before moving to the next.
TRICKS = {
    "SPIN": [
        ("RL", 2.5) # Rotate left for 2.5 seconds, then stop
    ],
    "FLIP": [
        ("R", 0.5),
        ("F", 0.5),
        ("B", 0.5)
    ],
    "CIRCLE": [  # Move in a circle-like pattern by rotating and going forward
        ("RL", 0.5),
        ("F", 1.0),
        ("RL", 0.5),
        ("F", 1.0),
        ("RL", 0.5),
        ("F", 1.0),
        ("RL", 0.5),
        ("F", 1.0)
    ],
    "DANCE": [  # A playful pattern of movements
        ("F", 0.5),
        ("S", 0.2),
        ("B", 0.5),
        ("S", 0.2),
        ("L", 0.5),
        ("S", 0.2),
        ("R", 0.5)
    ],
    "ZIGZAG": [ # Zigzag pattern: forward+left, stop, forward+right, stop
        ("F", 0.5),
        ("L", 0.5),
        ("S", 0.2),
        ("F", 0.5),
        ("R", 0.5)
    ]
}

def handle_client(conn, addr):
    with lock:
        car_states[addr] = {'x':0.0, 'y':0.0, 'yaw':0.0, 'pitch':0.0, 'roll':0.0, 'distance':-1, 'last_update':time.time()}
    try:
        while True:
            data = conn.recv(1024)
            if not data:
                break
            line = data.decode().strip()
            if line.startswith("DIST:"):
                parts = line.split()
                distance = float(parts[0].replace("DIST:", ""))
                pitch = float(parts[1].replace("P:", ""))
                roll = float(parts[2].replace("R:", ""))
                yaw = float(parts[3].replace("Y:", ""))

                with lock:
                    car = car_states[addr]
                    car['yaw'] = yaw
                    car['pitch'] = pitch
                    car['roll'] = roll
                    car['distance'] = distance
                    car['last_update'] = time.time()
    except:
        pass
    finally:
        with lock:
            if addr in car_states:
                del car_states[addr]
            if conn in clients:
                clients.remove(conn)
        conn.close()

def accept_clients(server_socket):
    while True:
        conn, addr = server_socket.accept()
        with lock:
            clients.append(conn)
        thread = threading.Thread(target=handle_client, args=(conn, addr), daemon=True)
        thread.start()

def broadcast_command(cmd, target=None):
    with lock:
        to_remove = []
        for c in clients:
            c_addr = c.getpeername()
            c_addr_str = f"{c_addr[0]}:{c_addr[1]}"
            if target is None or target == c_addr_str:
                try:
                    c.sendall((cmd+"\n").encode())
                except:
                    to_remove.append(c)
        for c in to_remove:
            clients.remove(c)

def perform_trick(trick_name, target=None):
    steps = TRICKS.get(trick_name, [])
    for (cmd, duration) in steps:
        # If the step is a move command, send it and wait
        if cmd != "S":
            broadcast_command(cmd, target)
            time.sleep(duration)
            broadcast_command("S", target)  # Stop after each movement step
        else:
            # If the step is 'S' itself, just wait the duration
            broadcast_command("S", target)
            time.sleep(duration)

app = Flask(__name__)

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/cars")
def get_cars():
    with lock:
        data = []
        for addr, state in car_states.items():
            addr_str = f"{addr[0]}:{addr[1]}"
            data.append({
                'addr': addr_str,
                'x': state['x'],
                'y': state['y'],
                'yaw': state['yaw'],
                'pitch': state['pitch'],
                'roll': state['roll'],
                'distance': state['distance']
            })
        return jsonify(data)

@app.route("/command", methods=['POST'])
def send_command():
    cmd = request.form.get('cmd', '').strip().upper()
    target = request.form.get('target', '')
    if target == 'all':
        target = None
    if cmd in TRICKS.keys():
        threading.Thread(target=perform_trick, args=(cmd, target), daemon=True).start()
    else:
        broadcast_command(cmd, target)
    return "OK"

def start_flask():
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)

def main():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen(5)

    accept_thread = threading.Thread(target=accept_clients, args=(server_socket,), daemon=True)
    accept_thread.start()

    flask_thread = threading.Thread(target=start_flask, daemon=True)
    flask_thread.start()

    try:
        while True:
            cmd = input("Enter Command: ").strip()
            if cmd:
                if cmd in TRICKS.keys():
                    perform_trick(cmd)
                else:
                    broadcast_command(cmd)
    except KeyboardInterrupt:
        pass
    finally:
        server_socket.close()

if __name__ == "__main__":
    main()
