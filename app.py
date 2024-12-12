import socket
import threading
import time
import math
import uuid  # Import uuid for unique ID generation
from flask import Flask, jsonify, render_template, request

HOST = '0.0.0.0'
PORT = 12345

clients = []
lock = threading.Lock()

car_states = {}
environment_map = {}

CAR_LENGTH = 25.0
CAR_WIDTH = 15.0

TRICKS = {
    "SPIN": [("RL", 2.5)],
    "FLIP": [("R", 0.5), ("F", 0.5), ("B", 0.5)],
    "CIRCLE": [("RL", 0.5), ("F", 1.0), ("RL", 0.5), ("F", 1.0),
               ("RL", 0.5), ("F", 1.0), ("RL", 0.5), ("F", 1.0)],
    "DANCE": [("F", 0.5), ("S", 0.2), ("B", 0.5), ("S", 0.2),
              ("L", 0.5), ("S", 0.2), ("R", 0.5)],
    "ZIGZAG": [("F", 0.5), ("L", 0.5), ("S", 0.2),
               ("F", 0.5), ("R", 0.5)],
    "INIT": [
        ("S", 0.1), ("S", 0.1), ("RL", 0.5), ("S", 0.1),
        ("RL", 0.5), ("S", 0.1), ("RL", 0.5), ("S", 0.1),
        ("RL", 0.5), ("S", 0.1)
    ],
    "SQUARE": [
        ("F", 1.0), ("S", 0.2), ("ROT:90", 1.0),
        ("F", 1.0), ("S", 0.2), ("ROT:90", 1.0),
        ("F", 1.0), ("S", 0.2), ("ROT:90", 1.0),
        ("F", 1.0)
    ],
    "FIG8": [
        ("ROT:45", 1.0), ("F", 1.0), ("ROT:90", 1.0),
        ("F", 1.0), ("ROT:90", 1.0), ("F", 1.0),
        ("ROT:90", 1.0), ("F", 1.0), ("ROT:45", 1.0)
    ],
    "WAVE": [
        ("F", 0.5), ("L", 0.5), ("F", 0.5), ("R", 0.5),
        ("F", 0.5), ("R", 0.5), ("F", 0.5), ("L", 0.5)
    ]
}

app = Flask(__name__)

def handle_client(conn, addr):
    # Generate a unique ID for the car
    car_id = str(uuid.uuid4())

    with lock:
        car_states[car_id] = {
            'id': car_id,  # Store the unique ID
            'conn': conn,  # Reference to the connection
            'x': 0.0,
            'y': 0.0,
            'yaw': 0.0,
            'pitch': 0.0,
            'roll': 0.0,
            'distance': -1,
            'last_update': time.time(),
            'distances_rec': [],
            'init_in_progress': False,
            'stable_x': 0.0,
            'stable_y': 0.0,
            'last_direction': None
        }

    try:
        while True:
            data = conn.recv(1024)
            if not data:
                break
            line = data.decode().strip()

            if line.startswith("DIST:"):
                parts = line.split()
                if len(parts) < 4:
                    continue  # Invalid data format
                try:
                    distance = float(parts[0].replace("DIST:", ""))
                    pitch = float(parts[1].replace("P:", ""))
                    roll = float(parts[2].replace("R:", ""))
                    yaw = float(parts[3].replace("Y:", ""))
                except ValueError:
                    continue  # Invalid numerical values

                with lock:
                    car = car_states.get(car_id)
                    if not car:
                        continue  # Car might have been disconnected

                    car['yaw'] = yaw
                    car['pitch'] = pitch
                    car['roll'] = roll
                    car['distance'] = distance
                    car['last_update'] = time.time()

                    if car.get('init_in_progress', False):
                        if len(car['distances_rec']) < 4:
                            car['distances_rec'].append(distance)
                            if len(car['distances_rec']) == 4:
                                dN, dW, dS, dE = car['distances_rec']
                                env_width = dE + dW + CAR_WIDTH
                                env_height = dN + dS + CAR_LENGTH
                                environment_map[car_id] = {
                                    'width': env_width,
                                    'height': env_height
                                }

                                car['x'] = 0.0
                                car['y'] = 0.0
                                car['stable_x'] = 0.0
                                car['stable_y'] = 0.0
                                car['init_in_progress'] = False

                    env = environment_map.get(car_id, None)
                    if env:
                        dir = get_cardinal_direction(yaw)
                        if dir:
                            if dir == 'E':
                                new_x = (env['width'] / 2) - distance - (CAR_LENGTH / 2)
                                car['x'] = new_x
                                car['y'] = car['stable_y']
                                car['stable_x'] = new_x
                                car['last_direction'] = dir
                            elif dir == 'W':
                                new_x = (-env['width'] / 2) + distance + (CAR_LENGTH / 2)
                                car['x'] = new_x
                                car['y'] = car['stable_y']
                                car['stable_x'] = new_x
                                car['last_direction'] = dir
                            elif dir == 'N':
                                new_y = (env['height'] / 2) - distance - (CAR_LENGTH / 2)
                                car['y'] = new_y
                                car['x'] = car['stable_x']
                                car['stable_y'] = new_y
                                car['last_direction'] = dir
                            elif dir == 'S':
                                new_y = (-env['height'] / 2) + distance + (CAR_LENGTH / 2)
                                car['y'] = new_y
                                car['x'] = car['stable_x']
                                car['stable_y'] = new_y
                                car['last_direction'] = dir
                        else:
                            # Handle non-cardinal directions if needed
                            pass

    except Exception as e:
        print(f"Error handling client {car_id}: {e}")
    finally:
        with lock:
            if car_id in car_states:
                del car_states[car_id]
            if conn in clients:
                clients.remove(conn)
        conn.close()

def get_cardinal_direction(yaw):
    yaw = yaw % 360

    def within(a, b, tol=45):
        diff = min(abs(a - b), 360 - abs(a - b))
        return diff <= tol

    if within(yaw, 0):
        return 'E'
    if within(yaw, 90):
        return 'N'
    if within(yaw, 180):
        return 'W'
    if within(yaw, 270):
        return 'S'
    return None

def accept_clients(server_socket):
    while True:
        conn, addr = server_socket.accept()
        with lock:
            clients.append(conn)
        thread = threading.Thread(target=handle_client, args=(conn, addr), daemon=True)
        thread.start()

def broadcast_command(cmd, target_id=None):
    with lock:
        to_remove = []
        for car_id, car in list(car_states.items()):
            if target_id is None or target_id == car_id:
                try:
                    car['conn'].sendall((cmd + "\n").encode())
                except:
                    to_remove.append(car_id)
        for car_id in to_remove:
            if car_id in car_states:
                del car_states[car_id]

def perform_trick(trick_name, target_id=None):
    steps = TRICKS.get(trick_name, [])
    if trick_name == "INIT":
        with lock:
            for car_id, car in car_states.items():
                if target_id is None or target_id == car_id:
                    car['distances_rec'] = []
                    car['init_in_progress'] = True
    for (cmd, duration) in steps:
        if cmd.startswith("ROT:") or cmd.startswith("MOVE:"):
            broadcast_command(cmd, target_id)
            time.sleep(duration)
        elif cmd != "S":
            broadcast_command(cmd, target_id)
            time.sleep(duration)
            broadcast_command("S", target_id)
        else:
            broadcast_command("S", target_id)
            time.sleep(duration)

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/cars")
def get_cars():
    with lock:
        data = []
        for car_id, state in car_states.items():
            addr_str = f"{state['conn'].getpeername()[0]}:{state['conn'].getpeername()[1]}"
            env = environment_map.get(car_id, {'width': 0, 'height': 0})
            data.append({
                'id': state['id'],  # Unique UUID
                'addr': addr_str,
                'x': state['x'],
                'y': state['y'],
                'yaw': state['yaw'],
                'pitch': state['pitch'],
                'roll': state['roll'],
                'distance': state['distance'],
                'env_width': env['width'],
                'env_height': env['height']
            })
        return jsonify(data)

@app.route("/command", methods=['POST'])
def send_command():
    cmd = request.form.get('cmd', '').strip()
    target = request.form.get('target', '')
    if target == 'all':
        target_id = None
    else:
        target_id = target  # Assume target is the UUID
    cmd_upper = cmd.upper()

    if cmd_upper in TRICKS.keys():
        threading.Thread(target=perform_trick, args=(cmd_upper, target_id), daemon=True).start()
    else:
        broadcast_command(cmd, target_id)
    return "OK"

def start_flask():
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)

def main():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen(5)
    print(f"Server listening on {HOST}:{PORT}")

    accept_thread = threading.Thread(target=accept_clients, args=(server_socket,), daemon=True)
    accept_thread.start()

    flask_thread = threading.Thread(target=start_flask, daemon=True)
    flask_thread.start()

    try:
        while True:
            cmd = input("Enter Command: ").strip()
            if cmd:
                cmd_upper = cmd.upper()
                if cmd_upper in TRICKS.keys():
                    perform_trick(cmd_upper)
                else:
                    broadcast_command(cmd)
    except KeyboardInterrupt:
        print("\nShutting down server...")
    finally:
        with lock:
            for car in clients:
                try:
                    car.close()
                except:
                    pass
            clients.clear()
        server_socket.close()

if __name__ == "__main__":
    main()
