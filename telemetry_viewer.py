import sys
import serial
import serial.tools.list_ports
from vpython import (
    canvas, box, compound, label, vector, color, rate, radians
)

# ---------------------------------------------------------------
# AYARLAR
# ---------------------------------------------------------------
SERIAL_PORT = "COM5"      # Aygıt Yöneticisindeki güncel port
BAUD_RATE = 115200
MAX_ALTITUDE_M = 200.0

# ---------------------------------------------------------------
# PORT BAĞLANTISI KONTROLÜ
# ---------------------------------------------------------------
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"{SERIAL_PORT} portu {BAUD_RATE} baud ile acildi. Veri bekleniyor...")
except serial.SerialException as e:
    print("\n" + "=" * 50)
    print(f"[HATA] Seri porta baglanilamadi!")
    print(f"Detay: {e}")
    print("=" * 50)
    print("Mevcut aktif portlar:")
    ports = serial.tools.list_ports.comports()
    for p in ports:
        print(f" -> {p.device}: {p.description}")
    print("\nEger Termite aciksa kapatin. Farkli bir portsa 'SERIAL_PORT' degerini guncelleyin.")
    input("\nKapatmak icin Enter'a basin...")
    sys.exit(1)

# ---------------------------------------------------------------
# SAHNE
# ---------------------------------------------------------------
scene = canvas(title="STM32 Telemetri Gorsellestirme", width=900, height=600)
scene.background = color.gray(0.15)
scene.forward = vector(-1, -0.3, -1)

# ---------------------------------------------------------------
# MODEL
# ---------------------------------------------------------------
fuselage = box(pos=vector(0, 0, 0), size=vector(2.0, 0.3, 0.3), color=color.white)
wing     = box(pos=vector(0.1, 0, 0), size=vector(0.4, 0.05, 2.2), color=color.blue)
tail_v   = box(pos=vector(-0.9, 0.3, 0), size=vector(0.3, 0.6, 0.05), color=color.red)
tail_h   = box(pos=vector(-0.9, 0, 0), size=vector(0.3, 0.05, 0.9), color=color.red)

plane = compound([fuselage, wing, tail_v, tail_h], origin=vector(0, 0, 0))
plane.axis = vector(1, 0, 0)
plane.up = vector(0, 1, 0)

# ---------------------------------------------------------------
# IRTIFA BARI
# ---------------------------------------------------------------
BAR_X = 5
BAR_MAX_HEIGHT = 5.0

bar_frame = box(pos=vector(BAR_X, BAR_MAX_HEIGHT / 2, 0),
                size=vector(0.6, BAR_MAX_HEIGHT, 0.6),
                color=color.gray(0.4), opacity=0.25)

bar = box(pos=vector(BAR_X, 0, 0), size=vector(0.5, 0.01, 0.5), color=color.green)
bar_label = label(pos=vector(BAR_X, BAR_MAX_HEIGHT + 0.8, 0), text="0.0 m", box=False, height=16)


def update_altitude_bar(altitude_m: float) -> None:
    # Scale and update visual altitude indicator
    ratio = max(0.0, min(1.0, altitude_m / MAX_ALTITUDE_M))
    h = max(0.01, ratio * BAR_MAX_HEIGHT)
    bar.size.y = h
    bar.pos.y = h / 2
    bar_label.text = f"{altitude_m:.1f} m"


def apply_orientation(obj, roll_deg: float, pitch_deg: float, yaw_deg: float) -> None:
    # Reset orientation baseline
    obj.axis = vector(1, 0, 0)
    obj.up = vector(0, 1, 0)

    # Z-Y-X Euler rotation sequence
    obj.rotate(angle=radians(yaw_deg), axis=vector(0, 1, 0))

    right = obj.axis.cross(obj.up).norm()
    obj.rotate(angle=radians(pitch_deg), axis=right)

    obj.rotate(angle=radians(roll_deg), axis=obj.axis)


# ---------------------------------------------------------------
# DÖNGÜ
# ---------------------------------------------------------------
while True:
    rate(50)

    try:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
    except Exception as e:
        print(f"Seri port okuma hatasi: {e}")
        continue

    if not line:
        continue

    parts = line.split(",")
    if len(parts) != 4:
        continue

    try:
        roll, pitch, yaw, altitude = map(float, parts)
    except ValueError:
        continue

    apply_orientation(plane, roll, pitch, yaw)
    update_altitude_bar(altitude)