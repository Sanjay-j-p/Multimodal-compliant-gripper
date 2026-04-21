import paho.mqtt.client as mqtt
from arduino_bridge import ArduinoBridge
import time
import threading

bridge = ArduinoBridge()
bridge.connect()

BROKER = "localhost"
HALL_TOPIC = "/sensor/hall"
FORCE_TOPIC = "/sensor/force"
CMD_TOPIC = "/cmd/gripper"


CURRENT_POS = 2048   
MIN_POS = 1000
MAX_POS = 2200
STEP = 100


def pump_on():
    print("Pump ON")
    bridge.notify("suction", 215)

def pump_off():
    print("Pump OFF")
    bridge.notify("suction", 0)

def on_message(client, userdata, msg):
    global CURRENT_POS
    
    cmd = msg.payload.decode().strip().lower()
    print(f"Command received: {cmd}")

    if cmd == "grip_increase":
        CURRENT_POS = min(CURRENT_POS + STEP, MAX_POS)
        print(f"Increasing grip → {CURRENT_POS}")
        bridge.notify("grasp", CURRENT_POS)

    elif cmd == "grip_decrease":
        CURRENT_POS = max(CURRENT_POS - STEP, MIN_POS)
        print(f"Decreasing grip → {CURRENT_POS}")
        bridge.notify("grasp", CURRENT_POS)

    elif cmd == "release_grip":
        CURRENT_POS = 1000
        print("Reset grip ")
        bridge.notify("grasp", CURRENT_POS)

    elif cmd == "pump_on":
        pump_on()

    elif cmd == "pump_off":
        pump_off()


def publish_sensors(client):
    while True:
        try:
            hall = bridge.call("read_hall_value")

            client.publish(FORCE_TOPIC, round(hall, 3))

        except Exception as e:
            print(f"Sensor error: {e}")

        time.sleep(0.05)


client = mqtt.Client()
client.on_message = on_message

client.connect(BROKER, 1883)
client.subscribe(CMD_TOPIC)
client.loop_start()


sensor_thread = threading.Thread(
    target=publish_sensors, 
    args=(client,), 
    daemon=True
)
sensor_thread.start()

print("Bridge running, waiting for commands...")


while True:
    time.sleep(1)