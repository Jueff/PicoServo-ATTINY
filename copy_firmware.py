import os
import shutil
Import("env")

print(">>> copy_firmware.py wurde geladen!")
print(">>> BUILD_DIR beim Laden:", env.get("BUILD_DIR"))
print(">>> $PROJECT_BUILD_DIR:  ", env.get("PROJECT_BUILD_DIR"))
print(">>> $PIOENV           :  ", env.get("PIOENV"))
def copy_firmware(source, target, env):
    print(">>> copy_firmware wird ausgefuehrt!")
    os.makedirs("./firmware", exist_ok=True)
    uf2_path = str(target[0]).replace(".elf", ".uf2")
    print("Kopiere:", uf2_path)
    shutil.copy(uf2_path, "./firmware/PicoServo-Tiny-v0.2.uf2")

env.AddPostAction("$BUILD_DIR/firmware.elf", copy_firmware)