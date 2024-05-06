import paho.mqtt.client as mqtt # paho-mqtt from https://github.com/eclipse/paho.mqtt.python.git
import os

mqtt_user = os.getenv('MQTT_USER')
mqtt_pass = os.getenv('MQTT_PASS')

print(mqtt_user, mqtt_pass)

mqtt_broker = "192.168.0.132"
mqtt_port = 1883

topic = {
    "timestamp" : "rover/timestamp",
    "lon" : "rover/position/lon",
    "lat" : "rover/position/lat",
    "heading" : "rover/position/heading",
    "acc_x" : "rover/system/acc_x",
    "acc_y" : "rover/system/acc_y",
    "acc_z" : "rover/system/acc_z",
    "obstacle" : "rover/system/obstacle",
    "brightness" : "rover/environment/brightness",
    "temperature" : "rover/environment/temperature",
    "pressure" : "rover/environment/pressure"
}

if __name__ == '__main__':
    mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    mqttc.on_message = lambda mqttc, obj, msg: (print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload)))
    mqttc.on_connect = lambda mqttc, obj, flags, reason_code, properties: (print("reason_code: " + str(reason_code)))
    mqttc.on_publish = lambda mqttc, obj, mid, reason_code, properties: (print("mid: " + str(mid)))
    mqttc.on_subscribe = lambda mqttc, obj, mid, reason_code_list, properties: (print("Subscribed: " + str(mid) + " " + str(reason_code_list)))

    # mqttc.on_log = lambda mqttc, obj, level, string: (print(string))

    mqttc.username_pw_set(username=mqtt_user,password=mqtt_pass)

    mqttc.connect(mqtt_broker, 1883, 60)

    mqttc.loop_start()

    infot = mqttc.publish(topic["lon"], "48.2087842", qos=1)
    infot = mqttc.publish(topic["lat"], "16.3708649", qos=1)

    infot.wait_for_publish()
