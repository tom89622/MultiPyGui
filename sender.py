import serial
import time

ser = serial.Serial('/dev/ttys011', 115200)  # Adjust to one of your virtual ports
time.sleep(2)  # Wait for the connection to establish

for i in range(10):
    message = f"Message {i}\n"
    ser.write(message.encode())
    print(f"Sent: {message.strip()}")
    time.sleep(1)

ser.close()