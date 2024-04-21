import paho.mqtt.client as mqtt # paho-mqtt from https://github.com/eclipse/paho.mqtt.python.git
import os
import time
import random

mqtt_user = os.getenv('MQTT_USER')
mqtt_pass = os.getenv('MQTT_PASS')

mqtt_broker = "192.168.0.132"
mqtt_port = 1883

topic_lon = "rover/position/lon"
topic_lat = "rover/position/lat"


def on_connect(mqttc, obj, flags, reason_code, properties):
    print("reason_code: " + str(reason_code))


def on_message(mqttc, obj, msg):
    print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload))


def on_publish(mqttc, obj, mid, reason_code, properties):
    print("mid: " + str(mid))

def on_subscribe(mqttc, obj, mid, reason_code_list, properties):
    print("Subscribed: " + str(mid) + " " + str(reason_code_list))


def on_log(mqttc, obj, level, string):
    print(string)


mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_publish = on_publish
mqttc.on_subscribe = on_subscribe

# mqttc.on_log = on_log

mqttc.username_pw_set(username=mqtt_user,password=mqtt_pass)

mqttc.connect(mqtt_broker, 1883, 60)

mqttc.loop_start()

infot = mqttc.publish(topic_lon, "48.2087842", qos=1)
infot = mqttc.publish(topic_lat, "16.3708649", qos=1)

infot.wait_for_publish()
