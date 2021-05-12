import serial
import time
import paho.mqtt.client as paho

mqttc = paho.Client()

host = "192.168.43.254"
topic1 = "Mbed1"
topic2 = "Mbed2"
# XBee setting
serdev = '/dev/ttyACM0'
s = serial.Serial(serdev, 9600)

upbound = 10
overtilt = []
num = 0

def countFunc():
    if(countFunc.count < 11):
        countFunc.count += 1
    else:
        countFunc.count = 1
    return countFunc.count

countFunc.count = 0

def on_connect(self, mosq, obj, rc):
    print("Connected rc: " + str(rc))

def on_message(mosq, obj, msg):
    print("[Received] Topic: " + msg.topic + ", Message: " + str(msg.payload) + "\n")
    if msg.topic == 'Mbed1':
        s.write("/broker/run\r\n".encode())
        print("done")
        s.write("/tilt/run\r\n".encode())
        print("finish")

    if msg.topic == 'Mbed2':
        print("mbed2 got")
        if(countFunc() < upbound):
            overtilt.append(str(msg.payload)[2:4])
        else:
            overtilt.append(str(msg.payload)[2:4])
            for i in overtilt:
                print(i)
            s.write("/broker/run\r\n".encode())
    

def on_subscribe(mosq, obj, mid, granted_qos):
    print("Subscribed OK")

def on_unsubscribe(mosq, obj, mid, granted_qos):
    print("Unsubscribed OK")

mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_subscribe = on_subscribe
mqttc.on_unsubscribe = on_unsubscribe

print("Connecting to " + host + "/" + topic1)
mqttc.connect(host, port=1883, keepalive=60)
mqttc.subscribe(topic1, 0)
mqttc.subscribe(topic2, 0)

print("start sending RPC")

s.write("/guesture_IU/run\r\n".encode())
N = 1
keep = 1
while keep:
    
    print('IU start')
    mqttc.loop()
    #= mqttc.on_message.msg.payload
    #s.write("/broker/run\r\n".encode())
    #print(string.decode())
    time.sleep(1)

print("out")

mqttc.loop_forever()


s.close()