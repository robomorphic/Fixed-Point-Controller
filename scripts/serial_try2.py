import serial
import time
import math

FIXED_POINT = True

# Define the serial port and baud rate
serial_port = '/dev/cu.usbmodem111201'
baud_rate = 115200
ITERATION_COUNT = 1

# Open the serial connection
ser = serial.Serial(serial_port, baud_rate, timeout=1)

# Wait a moment to establish the connection
time.sleep(2)


# Create a message to send
message = "1000000\n200000\n30000\n400000\n"
message = message.encode()

start = time.time()
ser.write(message)
print("message: ", message)
qcos_0_pi = ser.readline()
qcos_1_pi = ser.readline()
qcos_2_pi = ser.readline()
qcos_3_pi = ser.readline()
data_tau0 = ser.readline()
data_tau1 = ser.readline()
data_tau2 = ser.readline()
runtime = ser.readline()

print("Fixed point32")
data_tau0 = float(data_tau0.decode())
data_tau1 = float(data_tau1.decode())
data_tau2 = float(data_tau2.decode())

runtime = float(runtime.decode())
print("data_taus: ", [data_tau0, data_tau1, data_tau2])

data_tau0 = data_tau0 / 2**30
data_tau1 = data_tau1 / 2**30
data_tau2 = data_tau2 / 2**29

print(f"qcos_0_pi: {qcos_0_pi}")
print(f"qcos_1_pi: {qcos_1_pi}")
print(f"qcos_2_pi: {qcos_2_pi}")
print(f"qcos_3_pi: {qcos_3_pi}")
print(f"data_tau0: {data_tau0}")
print(f"data_tau1: {data_tau1}")
print(f"data_tau2: {data_tau2}")
print(f"Total time: {runtime}")



# Close the serial connection
ser.close()