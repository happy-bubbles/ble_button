import paho.mqtt.client as mqtt
import json, os, datetime, time

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, rc):
    print("Connected with result code "+str(rc))
# Subscribing in on_connect() means that if we lose the connection and
# reconnect then subscriptions will be renewed.
    client.subscribe("happy-bubbles/ble/+/raw/+")

def get_sensor(data, rssi):
    # 020104020a0011ffcafebeefddeeaadd000abfbf00000000
    # 020104020a0011ffcafebeef77665544000be8bf00000000
    # 020104020a0011ffcafebeefddeeaadd000abfbf00000000
    out = {}
    out['rssi'] = rssi
    if data.startswith("020104020a0011ff045600001"):
        print(data)
        md = data[24:]
        out['type'] = "bunchie"
        out['force'] = "n/a"
        out['serial_number'] = md[0:8]
        out['b1'] = md[8:10]
        out['b2'] = md[10:12]
        out['b3'] = md[12:14]
        bat = md[14:18]
        if bat == '':
            return {}
        out['battery'] = int(bat, 16)
        out['rand'] = md[18:20]
        return out

    return False

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    #print "Topic: ", msg.topic+'\nMessage: '+str(msg.payload)
    mj = json.loads(msg.payload)
    data = mj['data']
    rssi = mj['rssi']
    bunchie_info = get_sensor(data, rssi)
    if not bunchie_info or bunchie_info == {}:
        return
    #print(bunchie_info)
    timestamp = datetime.datetime.now()
    print("%s - %s %s %s %s %s %s %s rand: %s - rssi: %s\n" % (timestamp, bunchie_info['serial_number'], bunchie_info['b1'], bunchie_info['b2'], bunchie_info['b3'], bunchie_info['battery'], bunchie_info['type'], bunchie_info['force'], bunchie_info['rand'], bunchie_info['rssi']))

time.sleep(1)
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.username_pw_set("moo", "moosheep")
client.connect("home.nemik.net", 1883, 60)

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
client.loop_forever()
