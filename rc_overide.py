from pymavlink import mavutil
import time

# Fungsi untuk menghubungkan ke vehicle
def connect_to_vehicle():
    connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
    connection.wait_heartbeat()
    print("Connected to vehicle!")
    return connection

# Fungsi untuk mengirim command long override pada channel RC
def override_rc_channels(connection, rc1_value, rc2_value, rc3_value, rc4_value):
    # Mengirimkan perintah COMMAND_LONG untuk override channel RC1-RC4
    try:
        connection.mav.rc_channels_override_send(
            connection.target_system,  # target_system (ID dari flight controller)
            connection.target_component,  # target_component (biasanya ID dari flight controller)
            rc1_value,  # Nilai override untuk RC channel 1 (1000-2000)
            rc2_value,  # Nilai override untuk RC channel 2 (1000-2000)
            rc3_value,  # Nilai override untuk RC channel 3 (1000-2000)
            rc4_value,  # Nilai override untuk RC channel 4 (1000-2000)
            0, 0, 0, 0  # Set remaining channels (RC5 to RC8) to 0 (no override)
        )
        print(f"RC channels overridden: RC1={rc1_value}, RC2={rc2_value}, RC3={rc3_value}, RC4={rc4_value}")
    except Exception as e:
        print(f"Failed to override RC channels: {e}")

# Menghubungkan ke flight controller
connection = connect_to_vehicle()

# Override nilai RC1-RC4
rc1_value = 1000  # Nilai untuk RC channel 1 (range: 1000-2000)
rc2_value = 1500  # Nilai untuk RC channel 2 (range: 1000-2000)
rc3_value = 1000  # Nilai untuk RC channel 3 (range: 1000-2000)
rc4_value = 1500  # Nilai untuk RC channel 4 (range: 1000-2000)

# Mengirimkan perintah override ke RC channel
try:
    while True:
        override_rc_channels(connection, rc1_value, rc2_value, rc3_value, rc4_value)
        time.sleep(1)  # Mengirimkan override setiap detik
except KeyboardInterrupt:
    print("Stopping RC override.")
finally:
    connection.close()
