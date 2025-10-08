import serial
# print(serial.__file__)

ser = serial.Serial('COM10', 115200)
print("Mengirim perintah mulai rekam...")
ser.write(b'r')

with open("output.raw", "wb") as f:
    print("Merekam... tekan Ctrl+C untuk berhenti.")
    try:
        while True:
            data = ser.read(1024)
            if data:
                f.write(data)
    except KeyboardInterrupt:
        print("Berhenti merekam.")
        ser.write(b's')
        ser.close()
