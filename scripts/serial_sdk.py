import serial
import time
import math

FIXED_POINT32 = True
FIXED_POINT16 = False

# Define the serial port and baud rate
serial_port = '/dev/tty.usbmodem21301'
baud_rate = 115200
ITERATION_COUNT = 100

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
        message = f"{int(qcos[0])} {int(qcos[1])} {int(qcos[2])} {int(qcos[3])} {int(qcos[4])} {int(qcos[5])} {int(qsin[0])} {int(qsin[1])} {int(qsin[2])} {int(qsin[3])} {int(qsin[4])} {int(qsin[5])} {int(v[0])} {int(v[1])} {int(v[2])} {int(v[3])} {int(v[4])} {int(v[5])} {int(a[0])} {int(a[1])} {int(a[2])} {int(a[3])} {int(a[4])} {int(a[5])} "
        message = message.encode()

        start = time.time()
        ser.write(message)
        print("message: ", message)
        ####var = ser.readline()
        ####print("var: ", var)

        data = ser.readline()
        data = data.decode()
        print("response: ", data)
        data = data.split(" ")
        data_tau0 = data[0]
        data_tau1 = data[1]
        data_tau2 = data[2]
        data_tau3 = data[3]
        data_tau4 = data[4]
        data_tau5 = data[5]
        runtime = data[6]
    else:
        message  = f"{qcos[0]} {qcos[1]} {qcos[2]} {qcos[3]} {qcos[4]} {qcos[5]} {qsin[0]} {qsin[1]} {qsin[2]} {qsin[3]} {qsin[4]} {qsin[5]} {v[0]} {v[1]} {v[2]} {v[3]} {v[4]} {v[5]} {a[0]} {a[1]} {a[2]} {a[3]} {a[4]} {a[5]} "
        message = message.encode()

        start = time.time()
        ser.write(message)
        print("message: ", message)
        data = ser.readline()
        data = data.decode()
        print("response: ", data)
        data = data.split(" ")
        data_tau0 = data[0]
        data_tau1 = data[1]
        data_tau2 = data[2]
        data_tau3 = data[3]
        data_tau4 = data[4]
        data_tau5 = data[5]
        runtime = data[6]


    if FIXED_POINT32:

        print("Fixed point32")
        data_tau0 = float(data_tau0)
        data_tau1 = float(data_tau1)
        data_tau2 = float(data_tau2)
        data_tau3 = float(data_tau3)
        data_tau4 = float(data_tau4)
        data_tau5 = float(data_tau5)

        runtime = float(runtime)
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
        data_tau0 = float(data_tau0)
        data_tau1 = float(data_tau1)
        data_tau2 = float(data_tau2)
        data_tau3 = float(data_tau3)
        data_tau4 = float(data_tau4)
        data_tau5 = float(data_tau5)

        runtime = float(runtime)
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
        runtime = float(runtime)
        print(f"data_tau0: {data_tau0}")
        print(f"data_tau1: {data_tau1}")
        print(f"data_tau2: {data_tau2}")
        print(f"data_tau3: {data_tau3}")
        print(f"data_tau4: {data_tau4}")
        print(f"data_tau5: {data_tau5}")
        print(f"Total time: {runtime}")

    all_runtimes.append(runtime)


print(f"Average time: {sum(all_runtimes) / len(all_runtimes)}")




# Close the serial connection
ser.close()