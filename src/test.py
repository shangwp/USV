import paho.mqtt.client as mqtt
import struct



def on_connect(client,userdata,flags,rc):
    print("connected with reult code " + str(rc))
    client.subscribe('/pose')
    print("connect success")

def on_message(client,userdata,msg):
    topic = '/ctrl'
    v=0.8
    r=-1
    p=50
    data = struct.pack(">ffB",v,r,p)
    client.publish(topic,payload=data)
    if msg.topic == '/pose' :
        a,b = struct.unpack('>ff',msg.payload)
        print("pose")

client = mqtt.Client()
client.on_connect=on_connect
client.on_message=on_message
client.connect("192.168.1.230",1883,600)
client.loop_forever()
