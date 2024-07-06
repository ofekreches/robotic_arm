import serial
import csv
import struct
import random
import threading
import time

# Constants from configuration.h
HEADER = 200
TAIL = 199
SIZE_OF_RX_DATA = 44  # Updated to match the new size
SIZE_OF_TX_DATA = 24  # 2 headers + 5 joints * 4 bytes + checksum + tail
SERIAL_BAUDRATE = 115200

# Serial port configuration (update the port name accordingly)
SERIAL_PORT = 'COM4'

def read_serial_data(ser, writer):
    while True:
        print("reading")
        data = ser.read(SIZE_OF_RX_DATA)
        if len(data) == SIZE_OF_RX_DATA:
            if data[0] == HEADER and data[1] == HEADER:
                # Unpack the data (little-endian format)
                base_joint_pos = struct.unpack('<f', data[2:6])[0]
                second_joint_pos = struct.unpack('<f', data[6:10])[0]
                third_joint_pos = struct.unpack('<f', data[10:14])[0]
                fourth_joint_pos = struct.unpack('<f', data[14:18])[0]
                fifth_joint_pos = struct.unpack('<f', data[18:22])[0]

                base_joint_vel = struct.unpack('<f', data[22:26])[0]
                second_joint_vel = struct.unpack('<f', data[26:30])[0]
                third_joint_vel = struct.unpack('<f', data[30:34])[0]
                fourth_joint_vel = struct.unpack('<f', data[34:38])[0]
                fifth_joint_vel = struct.unpack('<f', data[38:42])[0]

                checksum = data[42]

                # Compute checksum
                computed_checksum = sum(data[2:42]) & 0xFF

                if checksum == computed_checksum:
                    joint_data = {
                        "positions": [
                            base_joint_pos, second_joint_pos, third_joint_pos, fourth_joint_pos, fifth_joint_pos
                        ],
                        "velocities": [
                            base_joint_vel, second_joint_vel, third_joint_vel, fourth_joint_vel, fifth_joint_vel
                        ]
                    }
                    print("Positions:", joint_data["positions"])
                    print("Velocities:", joint_data["velocities"])

                    # Record the data to CSV
                    writer.writerow(joint_data["positions"] + joint_data["velocities"])

def send_desired_positions(ser):
    while True:
        # Generate random desired positions
        desired_positions = [random.uniform(-180.0, 180.0) for _ in range(5)]
        tx_data = bytearray(SIZE_OF_TX_DATA)
        tx_data[0] = HEADER
        tx_data[1] = HEADER

        struct.pack_into('<f', tx_data, 2, desired_positions[0])
        struct.pack_into('<f', tx_data, 6, desired_positions[1])
        struct.pack_into('<f', tx_data, 10, desired_positions[2])
        struct.pack_into('<f', tx_data, 14, desired_positions[3])
        struct.pack_into('<f', tx_data, 18, desired_positions[4])

        checksum = sum(tx_data[2:22]) & 0xFF
        tx_data[22] = checksum
        tx_data[23] = TAIL

        ser.write(tx_data)
        time.sleep(5)
        # print("Sent desired positions:", desired_positions)
        
        # Use a non-blocking delay by checking elapsed time


def main():
    # Initialize serial connection
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=1)

    with open('joint_data.csv', mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([
            "base_joint_pos", "second_joint_pos", "third_joint_pos", "fourth_joint_pos", "fifth_joint_pos",
            "base_joint_vel", "second_joint_vel", "third_joint_vel", "fourth_joint_vel", "fifth_joint_vel"
        ])

        # Create threads for reading and sending data
        read_thread = threading.Thread(target=read_serial_data, args=(ser, writer))
        send_thread = threading.Thread(target=send_desired_positions, args=(ser,))

        # Start the threads
        read_thread.start()
        send_thread.start()

        # Join the threads to keep the main thread alive
        read_thread.join()
        send_thread.join()

if __name__ == "__main__":
    main()
