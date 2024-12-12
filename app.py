import socket
import threading
import time
import math
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
    "FLIP": [("R",0.5),("F",0.5),("B",0.5)],
    "CIRCLE": [("RL",0.5),("F",1.0),("RL",0.5),("F",1.0),("RL",0.5),("F",1.0),("RL",0.5),("F",1.0)],
    "DANCE": [("F",0.5),("S",0.2),("B",0.5),("S",0.2),("L",0.5),("S",0.2),("R",0.5)],
    "ZIGZAG": [("F",0.5),("L",0.5),("S",0.2),("F",0.5),("R",0.5)],
    "INIT": [
        ("S",0.1),
        ("S",0.1),
        ("RL",0.5),
        ("S",0.1),
        ("RL",0.5),
        ("S",0.1),
        ("RL",0.5),
        ("S",0.1),
        ("RL",0.5),
        ("S",0.1)
    ],
    "SQUARE": [
        ("F",1.0),("S",0.2),("ROT:90",1.0),("F",1.0),("S",0.2),
        ("ROT:90",1.0),("F",1.0),("S",0.2),("ROT:90",1.0),("F",1.0)
    ],
    "FIG8": [
        ("ROT:45",1.0),("F",1.0),("ROT:90",1.0),("F",1.0),
        ("ROT:90",1.0),("F",1.0),("ROT:90",1.0),("F",1.0),
        ("ROT:45",1.0)
    ],
    "WAVE": [
        ("F",0.5),("L",0.5),("F",0.5),("R",0.5),
        ("F",0.5),("R",0.5),("F",0.5),("L",0.5)
    ]
}

app = Flask(__name__)

def handle_client(conn, addr):
    with lock:
        
        car_states[addr] = {
            'x':0.0, 
            'y':0.0, 
            'yaw':0.0, 
            'pitch':0.0, 
            'roll':0.0,
            'distance':-1, 
            'last_update':time.time(), 
            'distances_rec':[],
            'init_in_progress':False,
            'stable_x':0.0, 
            'stable_y':0.0, 
            'last_direction':None
        }

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

                    
                    if car.get('init_in_progress', False):
                        if len(car['distances_rec']) < 4:
                            car['distances_rec'].append(distance)
                            if len(car['distances_rec']) == 4:
                                dN, dW, dS, dE = car['distances_rec']
                                env_width = dE + dW + CAR_WIDTH
                                env_height = dN + dS + CAR_LENGTH
                                environment_map[addr] = {
                                    'width': env_width,
                                    'height': env_height
                                }
                                
                                car['x'] = 0.0
                                car['y'] = 0.0
                                car['stable_x'] = 0.0
                                car['stable_y'] = 0.0
                                car['init_in_progress'] = False

                    
                    env = environment_map.get(addr, None)
                    if env:
                        dir = get_cardinal_direction(yaw)
                        if dir:
                            
                            if dir == 'E':
                                
                                
                                
                                
                                new_x = (env['width']/2) - distance - (CAR_LENGTH/2)
                                
                                car['x'] = new_x
                                car['y'] = car['stable_y']
                                car['stable_x'] = new_x
                                car['last_direction'] = dir
                            elif dir == 'W':
                                
                                new_x = (-env['width']/2) + distance + (CAR_LENGTH/2)
                                car['x'] = new_x
                                car['y'] = car['stable_y']
                                car['stable_x'] = new_x
                                car['last_direction'] = dir
                            elif dir == 'N':
                                
                                new_y = (env['height']/2) - distance - (CAR_LENGTH/2)
                                car['y'] = new_y
                                car['x'] = car['stable_x']
                                car['stable_y'] = new_y
                                car['last_direction'] = dir
                            elif dir == 'S':
                                
                                new_y = (-env['height']/2) + distance + (CAR_LENGTH/2)
                                car['y'] = new_y
                                car['x'] = car['stable_x']
                                car['stable_y'] = new_y
                                car['last_direction'] = dir
                        else:
                            
                            
                            pass

    except:
        pass
    finally:
        with lock:
            if addr in car_states:
                del car_states[addr]
            if conn in clients:
                clients.remove(conn)
        conn.close()

def get_cardinal_direction(yaw):
    
    yaw = yaw % 360
    
    
    def within(a, b, tol=45):
        diff = min(abs(a-b), 360-abs(a-b))
        return diff <= tol

    if within(yaw, 0): return 'E'
    if within(yaw, 90): return 'N'
    if within(yaw, 180): return 'W'
    if within(yaw, 270): return 'S'
    return None

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
            c_str = f"{c_addr[0]}:{c_addr[1]}"
            if target is None or target == c_str:
                try:
                    c.sendall((cmd+"\n").encode())
                except:
                    to_remove.append(c)
        for c in to_remove:
            clients.remove(c)

def perform_trick(trick_name, target=None):
    steps = TRICKS.get(trick_name, [])
    if trick_name == "INIT":
        with lock:
            for addr, car in car_states.items():
                addr_str = f"{addr[0]}:{addr[1]}"
                if target is None or target == addr_str:
                    car['distances_rec'] = []
                    car['init_in_progress'] = True
    for (cmd, duration) in steps:
        if cmd.startswith("ROT:") or cmd.startswith("MOVE:"):
            
            broadcast_command(cmd, target)
            time.sleep(duration)
        elif cmd != "S":
            broadcast_command(cmd, target)
            time.sleep(duration)
            broadcast_command("S", target)
        else:
            broadcast_command("S", target)
            time.sleep(duration)

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/cars")
def get_cars():
    with lock:
        data = []
        for addr, state in car_states.items():
            addr_str = f"{addr[0]}:{addr[1]}"
            env = environment_map.get(addr, {'width':0,'height':0})
            data.append({
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
        target = None
    cmd_upper = cmd.upper()

    
    if cmd_upper in TRICKS.keys():
        threading.Thread(target=perform_trick, args=(cmd_upper, target), daemon=True).start()
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
                cmd_upper = cmd.upper()
                if cmd_upper in TRICKS.keys():
                    perform_trick(cmd_upper)
                else:
                    broadcast_command(cmd)
    except KeyboardInterrupt:
        pass
    finally:
        server_socket.close()

if __name__ == "__main__":
    main()