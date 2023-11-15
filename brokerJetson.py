import paho.mqtt.client as mqtt
import os
import numpy as np
import base64
from PIL import Image
from io import BytesIO

# Konfigurasi broker MQTT
broker_address = "192.168.1.7"  # Alamat IP broker MQTT
broker_port = 1883  # Port MQTT (biasanya 1883)
mqtt_topic = "gambar/image_base64"

# Direktori penyimpanan gambar di Jetson Nano
save_directory = "/home/jetsonfd/CSI-Camera/gambar/"

# Fungsi yang akan dipanggil ketika koneksi ke broker berhasil
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe(mqtt_topic)

# Fungsi yang akan dipanggil ketika pesan MQTT masuk
def on_message(client, userdata, msg):
    print("Menerima gambar Base64...")
    image_base64 = msg.payload.decode("utf-8")

    # Dekode gambar Base64 ke gambar asli
    image_bytes = base64.b64decode(image_base64)
    image = Image.open(BytesIO(image_bytes))

    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = os.path.join(save_directory, f"image_{timestamp}.jpg")

    # Simpan gambar ke direktori yang ditentukan
    image.save(filename)
    print(f"Gambar telah disimpan di: {filename}")

# Inisialisasi klien MQTT
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

# Koneksi ke broker MQTT
client.connect(broker_address, broker_port, 60)

# Loop untuk terus memantau pesan MQTT
client.loop_forever()
