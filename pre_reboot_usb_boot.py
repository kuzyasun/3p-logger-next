import time
from serial.tools import list_ports
from SCons.Script import Import, BUILD_TARGETS
Import("env")

VID_ESPR = 0x303A  # Espressif

def _ports_by_vid():
    return {p.device: p for p in list_ports.comports() if p.vid == VID_ESPR and p.device}

def _wait_disappear(dev: str, timeout=6.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if dev not in _ports_by_vid():
            return True
        time.sleep(0.05)
    return False

def _wait_reappear(timeout=12.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        now = _ports_by_vid()
        if now:
            # return the first found port (even if it's the same name)
            return sorted(now.keys())[0]
        time.sleep(0.05)
    return None

def send_boot_and_pick_port():
    before_dict = _ports_by_vid()
    before_set  = set(before_dict.keys())
    target = sorted(before_set)[0] if before_set else None

    # Send BOOT to the first available port (as before)
    if target:
        try:
            import serial
            with serial.Serial(port=target, baudrate=115200, timeout=0.2) as ser:
                time.sleep(0.2)
                ser.write(b"BOOT\n")
                ser.flush()
            print(f"[pre] Sent 'BOOT' to {target}")
        except Exception as e:
            print(f"[pre] Failed to send BOOT: {e}")
    else:
        print("[pre] No ESP32 ports found before reboot (maybe already in bootloader)")
    
    if target:
        _wait_disappear(target, timeout=6.0)

    boot_port = _wait_reappear(timeout=12.0)
    if boot_port:
        print(f"[pre] Bootloader COM ready: {boot_port}")
        env.Replace(UPLOAD_PORT=boot_port)
    else:
        print("[pre] Bootloader COM not detected; esptool will autodetect")

    env.Append(UPLOADERFLAGS=["--before", "no_reset", "--after", "soft_reset"])

targets = [str(t) for t in BUILD_TARGETS]

if any(t.startswith("upload") for t in targets):
    send_boot_and_pick_port()
else:
    print("[pre] Skip reboot script")
