import paho.mqtt.client as mqtt

# Fungsi callback ketika menerima pesan dari broker MQTT pertama
def on_message_from_mqtt1(client, userdata, message):
    payload = message.payload.decode('utf-8')
    print(f"Received message from MQTT1: {payload}")

# Konfigurasi broker MQTT pertama
mqtt1_broker_address = "www.pakwahyu.my.id"
mqtt1_client = mqtt.Client("mqtt1_subscriber")
mqtt1_client.on_message = on_message_from_mqtt1

# Hubungkan ke broker MQTT pertama
mqtt1_client.connect(mqtt1_broker_address, 1883)
mqtt1_client.subscribe("test/get")

# Loop untuk terus menerima pesan dari broker MQTT pertama
mqtt1_client.loop_forever()
