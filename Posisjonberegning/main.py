import asyncio
from bleak import BleakScanner
from scipy.optimize import minimize
import numpy as np
import json
from math import sqrt
import websockets
import threading

SPEED_OF_SOUND = 343E3
CPU_FREQUENCY = 16E6
GLOBAL_OFFSET_IN_TICKS = 2191
FROM_TICKS_TO_DISTANCE_COEFF = SPEED_OF_SOUND/CPU_FREQUENCY
BUFFER_SIZE = 10

WIDTH = 3000
HEIGHT = 1455

last_values = [0, 0, 0, 0]
REAL_LENGTH = 2572.36
CALIBRATION = False
STD_CHECK = False

class Node:
    def __init__(self, position, offset):
        assert len(position) == 3
        self.pos = position
        self.offset = offset
        self.distance_buffer = [(np.nan, np.nan) for _ in range(BUFFER_SIZE)]
        self.buffer_counter = 0
        self.timeout = False
        self.last_value = 0
        self.deviation = []

    def get_timeout(self):
        return self.timeout

    def set_timeout(self, val):
        self.timeout = val

    def reset_buffer(self):
        self.distance_buffer = [(np.nan, np.nan) for _ in range(BUFFER_SIZE)]
        self.buffer_counter = 0

    def push(self, counter, ticks):
        if counter not in [point[0] for point in self.distance_buffer] and ticks in range(70000, 209000):
            self.last_value = ticks
            if CALIBRATION:
                self.devation.append((ticks - GLOBAL_OFFSET_IN_TICKS)*FROM_TICKS_TO_DISTANCE_COEFF - REAL_LENGTH) 
            self.distance_buffer[self.buffer_counter % BUFFER_SIZE] = (
                counter,
                (ticks - GLOBAL_OFFSET_IN_TICKS)*FROM_TICKS_TO_DISTANCE_COEFF - self.offset
            )
            self.buffer_counter += 1
            return self.distance_buffer[(self.buffer_counter % BUFFER_SIZE)-1]
        return None

    def get_mean(self):
        return np.nanmean(self.distance_buffer, axis=0)[1]

    def get_std(self):
        return np.nanstd(self.distance_buffer, axis=0, ddof=1)[1]

    def get_last_value(self):
        return self.last_value

    def get_deviation(self):
        return np.nanmean(self.deviation)


def get_position(distances, node_positions):
    def error(x, c, r):
        return sum([(np.linalg.norm(x - c[i]) - r[i]) ** 2 for i in range(len(c))])

    l = len(node_positions)
    S = sum(distances)
    # compute weight vector for initial guess
    W = [((l - 1) * S) / (S - w) for w in distances]
    # get initial guess of point location
    x0 = sum([W[i] * node_positions[i] for i in range(l)])
    # optimize distance from signal origin to border of spheres
    return minimize(error, x0, args=(node_positions, distances), method='Nelder-Mead').x


async def compute_position_loop():
    global position
    while True:
        await asyncio.sleep(.5)
        i = 0
        with nodes_mutex:
            for n in nodes.values():
                if (n.get_last_value() in last_values):
                    n.set_timeout(True)
                else:
                    n.set_timeout(False)
                    last_values[i] = n.get_last_value()
                i += 1
            horizontal_distances = np.array([sqrt(n.get_mean()**2 - HEIGHT**2) for n in nodes.values() if not n.get_timeout()])
            node_positions = np.array([n.pos[:2] for n in nodes.values() if not n.get_timeout()])
        # print(node_positions)
        if len(node_positions) > 2:
            pos = get_position(horizontal_distances, node_positions)
            async with empty_room:
                position = json.dumps({"x": pos[0], "y": pos[1]})
            if CALIBRATION != True:
                print(pos)
        if CALIBRATION:
            print("Deviation: ", [(k, n.get_deviation()) for k, n in nodes.items()])
        if STD_CHECK:
            print("STS: ", [(k, n.get_std()) for k, n in nodes.items()])

def detection_callback(device, advertisement_data):
    if device.name[:4] == "NODE":
        mf_data = advertisement_data.manufacturer_data[89]
        counter = int.from_bytes(mf_data[:4], byteorder="big")
        ticks = int.from_bytes(mf_data[4:], byteorder="big")
        with nodes_mutex:
            node = nodes[device.name]
            distance = node.push(counter, ticks)

async def run():
    scanner = BleakScanner()
    scanner.register_detection_callback(detection_callback)
    await scanner.start()


async def client_handler(websocket, path):
    global readers
    print("New client!")
    async for message in websocket:
        async with readers_mutex:
            readers += 1
            if readers == 1:
                empty_room.acquire()

        print("Sending data", position)
        await websocket.send(position)

        async with readers_mutex:
            readers -= 1
            if readers == 0:
                empty_room.release()

        await asyncio.sleep(0.1)



readers_mutex = asyncio.Lock()
readers = 0
empty_room = asyncio.Semaphore()
position = ""

nodes_mutex = threading.Lock()
nodes = {
    "NODE1": Node(position=(0,     0,     HEIGHT), offset=50.2),
    "NODE2": Node(position=(WIDTH, 0,     HEIGHT), offset=40.0),
    "NODE3": Node(position=(WIDTH, WIDTH, HEIGHT), offset=0.4),
    "NODE4": Node(position=(0,     WIDTH, HEIGHT), offset=0.0),
}

start_server = websockets.serve(client_handler, "localhost", 8765)

loop = asyncio.get_event_loop()
loop.run_until_complete(start_server)
loop.run_until_complete(run())
loop.run_until_complete(compute_position_loop())
loop.run_forever()
# for i in range(5):
#     [node.reset_buffer() for node in nodes.values()]
#     loop.run_until_complete(run())

#     print([(k, n.buffer_counter) for k, n in nodes.items()])
#     horizontal_distances = np.array([sqrt(n.get_mean()**2 - HEIGHT**2) for n in nodes.values()])
#     print(horizontal_distances)
#     for k, v in nodes.items():
#         print(f"{k} mean: {v.get_mean()}\tstd: {v.get_std()}")
#     pos = get_position(horizontal_distances, np.array([n.pos[:2] for n in nodes.values()]))
#     print(json.dumps({"x": pos[0], "y": pos[1]})) #, "z": pos[2]}))
