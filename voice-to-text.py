import speech_recognition as sr
import bluetooth
import time

# --- Ganti dengan MAC Address ESP32 kamu ---
esp32_mac = "XX:XX:XX:XX:XX:XX"  # contoh: "24:6F:28:1A:2B:3C"
port = 1

print("Mencoba koneksi ke ESP32...")
sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.connect((esp32_mac, port))
print("Terhubung ke ESP32!")

recognizer = sr.Recognizer()
mic = sr.Microphone()

while True:
    with mic as source:
        print("Ucapkan angka (0-9)...")
        audio = recognizer.listen(source)

    try:
        text = recognizer.recognize_google(audio, language="id-ID")
        print("Anda bilang:", text)

        # Ambil angka dari hasil suara
        for ch in text:
            if ch.isdigit():
                sock.send(ch)
                print(f"Perintah dikirim: {ch}")
                break
        else:
            print("Tidak ada angka yang dikenali")

    except Exception as e:
        print("Error:", e)

    time.sleep(1)
