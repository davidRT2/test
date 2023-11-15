import paho.mqtt.client as mqtt
import cv2
import numpy as np
import base64

# Konfigurasi broker MQTT
mqtt_broker = "www.pakwahyu.my.id"
mqtt_port = 1883
mqtt_topic = "test/send"

# Inisialisasi OpenCV
cv2.namedWindow("Received Image", cv2.WINDOW_NORMAL)

# Fungsi yang dipanggil saat gambar diterima
def on_message(client, userdata, message):
    try:
        # Mendekode pesan MQTT dari base64 ke data gambar
        image_data_base64 = message.payload
        image_data = base64.b64decode(image_data_base64)

        # Membaca data gambar ke dalam format numpy array
        nparr = np.frombuffer(image_data, np.uint8)
        image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        # Menampilkan gambar
        cv2.imshow("Received Image", image)
        cv2.waitKey(1)  # Menambahkan penanganan event GUI

    except Exception as e:
        print("Error:", str(e))

# Inisialisasi MQTT client
client = mqtt.Client()
client.on_message = on_message

# Koneksi ke broker MQTT
client.connect(mqtt_broker, mqtt_port, 60)

# Berlangganan ke topik MQTT
client.subscribe(mqtt_topic)

# Loop tak terbatas untuk menerima pesan MQTT
client.loop_forever()
