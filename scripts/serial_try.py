import serial
import time
import math

FIXED_POINT32 = True
FIXED_POINT16 = False

# Define the serial port and baud rate
serial_port = '/dev/cu.usbmodem111201'
baud_rate = 115200
ITERATION_COUNT = 10000

# Open the serial connection
ser = serial.Serial(serial_port, baud_rate, timeout=1)

# Wait a moment to establish the connection
time.sleep(2)

# we will send qcos, qsin, v, and a, each has 6 elements
# std::vector<double> q = {1.97125, -0.372364, 1.64045, -0.674883, 2.38533, 0.727269};
# std::vector<double> v = {-0.329554, 0.536459, -0.444451, 0.10794, -0.0452059, 0.257742};
# std::vector<double> a = {-0.270431, 0.0268018, 0.904459, 0.83239, 0.271423, 0.434594};
# std::vector<double> cos_qpos = {cos(q[0]), cos(q[1]), cos(q[2]), cos(q[3]), cos(q[4]), cos(q[5])};
# std::vector<double> sin_qpos = {sin(q[0]), sin(q[1]), sin(q[2]), sin(q[3]), sin(q[4]), sin(q[5])};
q = [1.97125, -0.372364, 1.64045, -0.674883, 2.38533, 0.727269]
qcos = [math.cos(q[0]), math.cos(q[1]), math.cos(q[2]), math.cos(q[3]), math.cos(q[4]), math.cos(q[5])]
qsin = [math.sin(q[0]), math.sin(q[1]), math.sin(q[2]), math.sin(q[3]), math.sin(q[4]), math.sin(q[5])]
v = [-0.329554, 0.536459, -0.444451, 0.10794, -0.0452059, 0.257742]
a = [-0.270431, 0.0268018, 0.904459, 0.83239, 0.271423, 0.434594]

if FIXED_POINT16:
    qcos = [qcos[i] * 2**14 for i in range(6)]
    qsin = [qsin[i] * 2**14 for i in range(6)]
    v    = [v[i]    * 2**14 for i in range(6)]
    a    = [a[i]    * 2**14 for i in range(6)]

if FIXED_POINT32:
    print("Fixed point32")
    qcos = [qcos[i] * 2**30 for i in range(6)]
    qsin = [qsin[i] * 2**30 for i in range(6)]
    v    = [v[i]    * 2**28 for i in range(6)]
    a    = [a[i]    * 2**28 for i in range(6)]

# Average time: 0.009898534774780273
# Average time: 0.010396080493927002 - double
# Pico reported time: 0.0016806919999999986
# Pico reported time: 0.003630857000000001

all_runtimes = []
for i in range(ITERATION_COUNT):
    # Create a message to send
    runtime = 0.0
    if FIXED_POINT32 or FIXED_POINT16:
        message = f"{int(qcos[0])}\n{int(qcos[1])}\n{int(qcos[2])}\n{int(qcos[3])}\n{int(qcos[4])}\n{int(qcos[5])}\n{int(qsin[0])}\n{int(qsin[1])}\n{int(qsin[2])}\n{int(qsin[3])}\n{int(qsin[4])}\n{int(qsin[5])}\n{int(v[0])}\n{int(v[1])}\n{int(v[2])}\n{int(v[3])}\n{int(v[4])}\n{int(v[5])}\n{int(a[0])}\n{int(a[1])}\n{int(a[2])}\n{int(a[3])}\n{int(a[4])}\n{int(a[5])}\n"
        message = message.encode()

        start = time.time()
        ser.write(message)
        print("message: ", message)
        ####var = ser.readline()
        ####print("var: ", var)
        data_tau0 = ser.readline()
        data_tau1 = ser.readline()
        data_tau2 = ser.readline()
        data_tau3 = ser.readline()
        data_tau4 = ser.readline()
        data_tau5 = ser.readline()
        runtime = ser.readline()
    else:
        message  = f"{qcos[0]}\n{qcos[1]}\n{qcos[2]}\n{qcos[3]}\n{qcos[4]}\n{qcos[5]}\n{qsin[0]}\n{qsin[1]}\n{qsin[2]}\n{qsin[3]}\n{qsin[4]}\n{qsin[5]}\n{v[0]}\n{v[1]}\n{v[2]}\n{v[3]}\n{v[4]}\n{v[5]}\n{a[0]}\n{a[1]}\n{a[2]}\n{a[3]}\n{a[4]}\n{a[5]}\n"
        message = message.encode()

        start = time.time()
        ser.write(message)
        print("message: ", message)
        ####var = ser.readline()
        ####print("var: ", var)
        data_tau0 = ser.readline()
        data_tau1 = ser.readline()
        data_tau2 = ser.readline()
        data_tau3 = ser.readline()
        data_tau4 = ser.readline()
        data_tau5 = ser.readline()
        runtime = ser.readline()


    if FIXED_POINT32:

        print("Fixed point32")
        data_tau0 = float(data_tau0.decode())
        data_tau1 = float(data_tau1.decode())
        data_tau2 = float(data_tau2.decode())
        data_tau3 = float(data_tau3.decode())
        data_tau4 = float(data_tau4.decode())
        data_tau5 = float(data_tau5.decode())

        runtime = float(runtime.decode())
        print("qcos: ", qcos)
        print("qsin: ", qsin)
        print("v: ", v)
        print("a: ", a)
        print("data_taus: ", [data_tau0, data_tau1, data_tau2, data_tau3, data_tau4, data_tau5])

        data_tau0 = data_tau0 / 2**13
        data_tau1 = data_tau1 / 2**14
        data_tau2 = data_tau2 / 2**15
        data_tau3 = data_tau3 / 2**16
        data_tau4 = data_tau4 / 2**17
        data_tau5 = data_tau5 / 2**18
        

        print(f"data_tau0: {data_tau0}")
        print(f"data_tau1: {data_tau1}")
        print(f"data_tau2: {data_tau2}")
        print(f"data_tau3: {data_tau3}")
        print(f"data_tau4: {data_tau4}")
        print(f"data_tau5: {data_tau5}")
        print(f"Total time: {runtime}")
    elif FIXED_POINT16:
        print("Fixed point16")
        data_tau0 = float(data_tau0.decode())
        data_tau1 = float(data_tau1.decode())
        data_tau2 = float(data_tau2.decode())
        data_tau3 = float(data_tau3.decode())
        data_tau4 = float(data_tau4.decode())
        data_tau5 = float(data_tau5.decode())

        runtime = float(runtime.decode())
        print("qcos: ", qcos)
        print("qsin: ", qsin)
        print("v: ", v)
        print("a: ", a)
        print("data_taus: ", [data_tau0, data_tau1, data_tau2, data_tau3, data_tau4, data_tau5])

        data_tau0 = data_tau0 / 2**2
        data_tau1 = data_tau1 / 2**2
        data_tau2 = data_tau2 / 2**4
        data_tau3 = data_tau3 / 2**4
        data_tau4 = data_tau4 / 2**6
        data_tau5 = data_tau5 / 2**7

        print(f"data_tau0: {data_tau0}")
        print(f"data_tau1: {data_tau1}")
        print(f"data_tau2: {data_tau2}")
        print(f"data_tau3: {data_tau3}")
        print(f"data_tau4: {data_tau4}")
        print(f"data_tau5: {data_tau5}")
        print(f"Total time: {runtime}")

    else:
        runtime = float(runtime.decode())
        print(f"data_tau0: {data_tau0.decode()}")
        print(f"data_tau1: {data_tau1.decode()}")
        print(f"data_tau2: {data_tau2.decode()}")
        print(f"data_tau3: {data_tau3.decode()}")
        print(f"data_tau4: {data_tau4.decode()}")
        print(f"data_tau5: {data_tau5.decode()}")
        print(f"Total time: {runtime}")

    all_runtimes.append(runtime)


print(f"Average time: {sum(all_runtimes) / len(all_runtimes)}")




# Close the serial connection
ser.close()