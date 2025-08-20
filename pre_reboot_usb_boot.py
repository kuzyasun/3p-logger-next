Import("env")
import time
from serial.tools import list_ports
import serial

VID_ESPR = 0x303A  # Espressif

def send_boot_cmd():
    # Знаходимо активний COM від ESP32-S2 (CDC)
    ports = [p for p in list_ports.comports() if p.vid == VID_ESPR and p.device and "COM" in p.device.upper()]
    if not ports:
        print("[pre] COM від ESP32 не знайдено; можливо, пристрій уже у bootloader або не підключений")
        return

    port = ports[0].device
    print(f"[pre] Надсилаю 'BOOT' у {port} ...")
    try:
        ser = serial.Serial(port=port, baudrate=115200, timeout=0.2)
        time.sleep(0.2)            # дати Windows створити порт
        ser.write(b"BOOT\n")       # має збігатись із MAGIC у прошивці
        ser.flush()
        ser.close()
        time.sleep(0.6)            # дати часу перейти в bootloader і пере-енумеруватись
    except Exception as e:
        print(f"[pre] Не вдалося надіслати команду: {e}")

send_boot_cmd()
