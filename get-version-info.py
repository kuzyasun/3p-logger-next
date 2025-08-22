import os
import subprocess
import re
from datetime import datetime
import sys
from SCons.Script import Import, BUILD_TARGETS
env = DefaultEnvironment()
def set_build_env_variables():
    try:        
        git_hash = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"], 
            stdout=subprocess.PIPE, 
            text=True, 
            check=True,
            cwd='./' 
        ).stdout.strip()
    except Exception as e:
        print(f"[pre] Could not get git hash: {e}")
        git_hash = "N/A"
    
    version = "N/A"
    try:
        with open("./include/version.h", "r") as f:
            for line in f:
                if "#define FIRMWARE_VERSION" in line:
                    version = line.split('"')[1]
                    break
    except FileNotFoundError:
        print("[pre] Warning: ./include/version.h not found.")
    
    build_timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    print(f"[pre] PIO_VERSION: {version}")
    print(f"[pre] GIT_HASH: {git_hash}")
    print(f"[pre] BUILD_TIMESTAMP: {build_timestamp}")
    env.Append(
        CPPDEFINES=[
            ("FIRMWARE_VERSION", '\\"' + version + '\\"'),
            ("GIT_HASH", '\\"' + git_hash + '\\"'),
            ("BUILD_TIMESTAMP", '\\"' + build_timestamp + '\\"')
        ]
    )


valid_commands = ['build', 'run', 'upload', 'test']

targets = [str(t) for t in BUILD_TARGETS]
print(f"targets: {targets}")

if "upload" in targets or "build" in targets:    
    set_build_env_variables()
else:
    print("[pre] skip version script")
