import paho.mqtt.client as mqtt

global my_payload
my_payload = 10

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("jose")
    client.subscribe("carlos")
    print ("connected")
    #client.publish("jose", payload=None, qos=1, retain=False)
# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print("INFO: ", msg.info)
    print("PAYLOAD: ", msg.payload)
    print("TOPIC: ", msg.topic)
    #print (userdata)
    global my_payload
    my_payload += my_payload
    print("my payload",  my_payload)
    client.publish(topic="mayoral", payload=my_payload)


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect("localhost", 1883, 60)

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
client.loop_forever()
