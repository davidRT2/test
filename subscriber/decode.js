// Mendekode gambar yang dienkripsi ke base64 dari MQTT dengan topik test/kirim
function decodeImageFromMQTT() {
    // Mendapatkan pesan MQTT
    const mqttMessage = "MQTT message with base64 encoded image";
    
    // Mendekode pesan MQTT dari base64 ke binary
    const decodedImage = atob(mqttMessage);
    
    // Mengonversi binary ke Uint8Array
    const uint8Array = new Uint8Array(decodedImage.length);
    for (let i = 0; i < decodedImage.length; i++) {
      uint8Array[i] = decodedImage.charCodeAt(i);
    }
    
    // Membuat blob dari Uint8Array
    const blob = new Blob([uint8Array], { type: "image/jpeg" });
    
    // Membuat URL objek dari blob
    const urlObject = URL.createObjectURL(blob);
    
    // Menampilkan gambar di HTML
    const imgElement = document.createElement("img");
    imgElement.src = urlObject;
    document.body.appendChild(imgElement);
  }
  
  // Menyambungkan dan menangkap payload dari MQTT dengan topik test/kirim
  function connectToMQTT() {
    // Mendapatkan pesan MQTT
    const mqttMessage = "MQTT message with base64 encoded image";
    
    // Membuat koneksi MQTT
    const client = new Paho.MQTT.Client("www.pakwahyu.my.id", Number(8080), "clientId");
    
    // Menghubungkan klien ke broker
    client.connect({ onSuccess: onConnect });
    
    // Fungsi callback saat koneksi berhasil dibuat
    function onConnect() {
      console.log("Koneksi berhasil dibuat!");
      
      // Berlangganan topik test/kirim
      client.subscribe("test/kirim");
      
      // Fungsi callback saat pesan diterima
      client.onMessageArrived = onMessageArrived;
      
      // Fungsi callback saat pesan dikirim
      client.onMessageDelivered = onMessageDelivered;
      
      // Fungsi callback saat koneksi terputus
      client.onConnectionLost = onConnectionLost;
      
      // Mengirim pesan MQTT dengan payload gambar yang dienkripsi ke base64
      const message = new Paho.MQTT.Message(mqttMessage);
      message.destinationName = "test/kirim";
      client.send(message);
      
      console.log("Pesan berhasil dikirim!");
    }
    
    // Fungsi callback saat pesan diterima
    function onMessageArrived(message) {
      console.log("Pesan diterima: " + message.payloadString);
      
      // Mendekode gambar yang dienkripsi ke base64 dari MQTT dengan topik test/kirim
      decodeImageFromMQTT();
      
      console.log("Gambar berhasil didekode!");
    }
    
    // Fungsi callback saat pesan dikirim
    function onMessageDelivered(message) {
      console.log("Pesan berhasil dikirim: " + message.payloadString);
    }
    
    // Fungsi callback saat koneksi terputus
    function onConnectionLost(responseObject) {
      if (responseObject.errorCode !== Paho.MQTT.CONNECTION_LOST) {
        console.log("Koneksi terputus: " + responseObject.errorMessage);
      }
      
      console.log("Koneksi terputus!");
    }
  }
  