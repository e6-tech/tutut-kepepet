import math
import redis
import json
import time
from pymavlink import mavutil

# Menghubungkan ke Redis dengan IP host tertentu
client = redis.StrictRedis(host='103.59.95.141', port=6379, db=0)

# Menghubungkan ke flight controller melalui port USB/serial
print("Connecting to vehicle...")
connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
connection.wait_heartbeat()
print("Connected to vehicle!")

# Fungsi untuk mendapatkan data dari vehicle (flight controller)
def get_vehicle_data():
    # Menerima pesan MAVLink
    msg_gps = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    msg_att = connection.recv_match(type='ATTITUDE', blocking=True)
    msg_bat = connection.recv_match(type='SYS_STATUS', blocking=True)
    msg_vfr = connection.recv_match(type='VFR_HUD', blocking=True)
    msg_rc = connection.recv_match(type='RC_CHANNELS', blocking=True)

    # Coba untuk menerima pesan SERVO_OUTPUT_RAW, tapi tidak wajib blocking=True
    msg_servo = connection.recv_match(type='SERVO_OUTPUT_RAW', blocking=False)

    # Mengambil data GPS (lokasi, ketinggian, dan ground course)
    gps = {
        'lat': msg_gps.lat / 1e7,   # Latitude
        'lon': msg_gps.lon / 1e7,   # Longitude
        'alt': msg_gps.alt / 1e3,   # Altitude (meters)
        'cog': msg_gps.hdg / 100.0  # Course over ground (degrees)
    }

    # Mengambil data status baterai
    batt_status = {
        'voltage': msg_bat.voltage_battery / 1000.0,  # Tegangan baterai dalam volt
        'current': msg_bat.current_battery / 100.0,   # Arus baterai dalam Ampere
        'level': msg_bat.battery_remaining            # Level baterai dalam %
    }

    # Mengambil data attitude (roll, pitch, yaw)
    attitude = {
        'roll': math.degrees(msg_att.roll),
        'pitch': math.degrees(msg_att.pitch),
        'yaw': math.degrees(msg_att.yaw)
    }

    # Mengambil data kecepatan tanah dalam beberapa satuan
    speed = {
        'ground_speed': msg_vfr.groundspeed,    # Groundspeed dalam m/s
        'kmh': msg_vfr.groundspeed * 3.6,       # Groundspeed dalam km/h
        'knot': msg_vfr.groundspeed * 1.94384   # Groundspeed dalam knot
    }

    # Mengambil data tambahan lainnya
    heading = msg_vfr.heading  # Compass heading
    baro = msg_vfr.alt  # Altitude relatif dari barometer

    # Mengambil data RC channel
    rc_channels = {
        'rc1': msg_rc.chan1_raw,
        'rc2': msg_rc.chan2_raw,
        'rc3': msg_rc.chan3_raw,
        'rc4': msg_rc.chan4_raw,
        'rc5': msg_rc.chan5_raw,
        'rc6': msg_rc.chan6_raw
    }

    # Mengambil nilai servo output jika tersedia
    if msg_servo:
        servo_output = {
            'steer': msg_servo.servo1_raw,  # Servo 1
            'th_mid': msg_servo.servo3_raw, # Servo 3
            'th_left': msg_servo.servo5_raw,# Servo 5
            'th_right': msg_servo.servo7_raw# Servo 7
        }
    else:
        # Jika pesan servo tidak ada, set sebagai None atau nilai default
        servo_output = {
            'steer': None,
            'th_mid': None,
            'th_left': None,
            'th_right': None
        }

    # Mendapatkan mode kendaraan
    msg_heartbeat = connection.recv_match(type='HEARTBEAT', blocking=True)
    mode = mavutil.mode_string_v10(msg_heartbeat)

    # Memeriksa apakah kendaraan armed
    is_armed = msg_heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED > 0

    # Memeriksa apakah kendaraan dapat di-arm
    is_armable = (
        mode in ["GUIDED", "AUTO", "STABILIZE"] and
        batt_status['level'] > 20 and  # Level baterai lebih dari 20%
        msg_bat.current_battery >= 0  # Status sistem tidak menunjukkan masalah
    )

    # Mengembalikan semua data dalam satu dictionary
    return {
        "gps": gps,
        "bat_status": batt_status,
        "heading": heading,
        "speed": speed,
        "baro": baro,
        "attitude": attitude,
        "rc_channels": rc_channels,  # Data RC
        "servo_output": servo_output,  # Data Servo
        "mode": mode,                  # Mode kendaraan
        "is_armed": is_armed,           # Status armed
        "is_armable": is_armable,        # Status armable
    }

# Fungsi untuk memposting data ke Redis
def post_data(data):
    # Mengambil data yang relevan untuk dikirim ke Redis
    information = {
        "latitude": data['gps']['lat'],
        "longitude": data['gps']['lon'],
        "cog": data['gps']['cog'],  # Course Over Ground
        "sog_knot": data['speed']['knot'],  # Speed Over Ground dalam knots
        "sog_kmh": data['speed']['kmh'],  # Speed Over Ground dalam km/h
    }

    # Mengirim data ke Redis dengan key 'username'
    client.set('username', json.dumps(information))

# Fungsi untuk arming dan disarming
def set_arm_state(arm: bool):
    mode = "GUIDED" if arm else "STABILIZE"
    connection.mav.set_mode_send(connection.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE, mode)

    if arm:
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # Not in mission
            1,  # Arm
            0, 0, 0, 0, 0, 0  # No additional parameters
        )
    else:
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # Not in mission
            0,  # Disarm
            0, 0, 0, 0, 0, 0  # No additional parameters
        )

# Main loop untuk terus mengambil dan mengirim data
try:
    while True:
        # Mengambil data dari flight controller
        data = get_vehicle_data()

        # Menampilkan data ke console
        print(data)
        print("\n")

        # Memposting data ke Redis
        post_data(data)

        # Menunggu 1 detik sebelum mengambil data lagi
        time.sleep(1)

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    # Menutup koneksi dengan aman
    connection.close()
