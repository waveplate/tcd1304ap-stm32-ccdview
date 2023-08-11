import sys
import serial
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import time

initial_xlim = 0
initial_ylim = 0

def setup_serial(port, baudrate):
    ser = serial.Serial(port=port, baudrate=baudrate, timeout=1)
    return ser

def read_serial(ser):
    buffer = bytearray()

    # start with 4096 to fix the ylim
    numbers = [4096]

    try:
        if ser.is_open:
            while True:
                data = ser.read(ser.in_waiting)
                if data:
                    for byte in data:
                        buffer.append(byte)

                        if len(buffer) >= 2 and buffer[-2:] == b'\xff\xff':
                            # Process the buffer contents as two-byte integers
                            numbers.clear()
                            for i in range(0, len(buffer) - 2, 2):
                                two_bytes = (buffer[i] << 8) + buffer[i + 1]
                                # prevent the graph from getting blown out by glitches
                                if two_bytes > 4096:
                                    two_bytes = 4096
                                numbers.append(two_bytes)

                            # try to reject nonsense values caused by ADC or UART glitches
                            if len(numbers) > 3600:
                                # add 0 to fix the ylim
                                numbers.append(0)
                                update_graph(numbers)

                            # print to console if we get a weird number of pixels
                            if len(numbers) != 3695:
                                print("unexpected number of pixels read:", len(numbers))

                            buffer.clear()
    except KeyboardInterrupt:
        print("keyboard interrupt -- closing the serial connection")
        ser.close()

def update_graph(numbers):
    global initial_xlim, initial_ylim

    current_xlim = ax_plot.get_xlim()
    current_ylim = ax_plot.get_ylim()

    ax_plot.clear()
    x = list(range(len(numbers)))
    y = numbers

    ax_plot.plot(x, y, "b.-")

    if current_xlim != initial_xlim or current_ylim != initial_ylim:
        ax_plot.set_xlim(current_xlim)
        ax_plot.set_ylim(current_ylim)

    plt.draw()
    plt.pause(update_interval)

if __name__ == "__main__":
    port = sys.argv[1]
    baudrate = int(sys.argv[2])
    update_interval = 0.1

    ser = setup_serial(port, baudrate)

    fig, ax_plot = plt.subplots()
    plt.title("pixel amplitude vs index")
    plt.xlabel("index")
    plt.ylabel("amplitude")

    numbers = []

    update_graph(numbers)

    initial_xlim = ax_plot.get_xlim()
    initial_ylim = ax_plot.get_ylim()

    read_serial(ser)
