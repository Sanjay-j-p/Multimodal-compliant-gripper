import paho.mqtt.client as mqtt
 

import time
BROKER = "10.89.94.149"  

FORCE_TOPIC = "/sensor/force"
CMD_TOPIC = "/cmd/gripper"
 
latest_hall = 0
latest_force = 0
 
def on_message(client, userdata, msg):
    global latest_hall, latest_force
   
    
   
    if msg.topic == FORCE_TOPIC:
        latest_force = float(msg.payload.decode())
        print(f"Force: {latest_force}N  Hall: {latest_hall}")
        time.sleep(0.1)
 
client = mqtt.Client()
client.on_message = on_message
client.connect(BROKER, 1883)
client.subscribe(FORCE_TOPIC)
client.loop_start()
 
print("Gripper client ready.")
print("Commands:")
print("  i → grip increase")
print("  d → grip decrease")
print("  r → release (reset)")
print("  p → pump ON")
print("  o → pump OFF")
print("  q → quit")
 
while True:
    cmd = input("Enter command: ").strip().lower()
 
    if cmd == "i":
        client.publish(CMD_TOPIC, "grip_increase")
 
    elif cmd == "d":
        client.publish(CMD_TOPIC, "grip_decrease")
 
    elif cmd == "r":
        client.publish(CMD_TOPIC, "release_grip")
 
    elif cmd == "p":
        client.publish(CMD_TOPIC, "pump_on")
 
    elif cmd == "o":
        client.publish(CMD_TOPIC, "pump_off")
 
    elif cmd == "q":
        print("Exiting...")
        break
 
    else:
        print("Unknown command")