import os
import subprocess
import re
from datetime import datetime
import sys

def set_build_env_variables():
    try:
        # ДОДАНО: cwd='..' вказує, що команду треба виконати у батьківській папці
        git_hash = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"], 
            stdout=subprocess.PIPE, 
            text=True, 
            check=True,
            cwd='..' 
        ).stdout.strip()
    except Exception as e:
        print(f"Could not get git hash: {e}")
        git_hash = "N/A"

    # Читаємо версію з файлу version.h (шлях відносно скрипта, тому не міняємо)
    version = "N/A"
    try:
        with open("./include/version.h", "r") as f:
            for line in f:
                if "#define FIRMWARE_VERSION" in line:
                    version = line.split('"')[1]
                    break
    except FileNotFoundError:
        print("Warning: ./include/version.h not found.")

    # Отримуємо поточну дату і час
    build_timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    # Встановлюємо змінні для C++ коду
    os.environ['PIO_VERSION'] = version
    os.environ['GIT_HASH'] = git_hash
    os.environ['BUILD_TIMESTAMP'] = build_timestamp  

pio_command = os.environ.get('PIOCOMMAND')
valid_commands = ['build', 'run', 'upload', 'test']


# if pio_command in valid_commands:
    # set_build_env_variables()
set_build_env_variables()