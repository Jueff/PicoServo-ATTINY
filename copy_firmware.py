# copy_firmware.py

import os
import shutil
import re
import zipfile
import glob
import configparser
from SCons.Script import Import

Import("env")

PROJECT_DIR = env.get("PROJECT_DIR")
PIOENV = env.get("PIOENV")
BUILD_DIR = env.get("BUILD_DIR")
PLATFORMIO_INI = os.path.join(PROJECT_DIR, "platformio.ini")

def read_version():
    # Try to read APP_VERSION from CPPDEFINES first
    cppdefs = env.get("CPPDEFINES") or []
    for d in cppdefs:
        # tuple ('APP_VERSION', 'v0.4') or string 'APP_VERSION="v0.4"'
        if isinstance(d, tuple) and d[0] == "APP_VERSION":
            return str(d[1]).strip()
        if isinstance(d, str) and d.startswith("APP_VERSION"):
            # e.g. APP_VERSION="v0.4" or APP_VERSION=v0.4
            val = d.split("=", 1)[1].strip().strip('"').strip("'")
            return val.strip()

    # Then try BUILD_FLAGS (may be string or list)
    bf = env.get("BUILD_FLAGS")
    if bf:
        if isinstance(bf, (list, tuple)):
            bf_str = " ".join(str(x) for x in bf)
        else:
            bf_str = str(bf)
        m = re.search(r'-DAPP_VERSION=(?:"|\')?(v?[0-9]+(?:\.[0-9]+)*)', bf_str)
        if m:
            return m.group(1).strip()

    # Finally try platformio.ini (legacy fallback)
    try:
        cfg = configparser.RawConfigParser()
        cfg.read(PLATFORMIO_INI)
        section_env = "env:" + (PIOENV or "")
        if cfg.has_option(section_env, "version"):
            return cfg.get(section_env, "version").strip()
        if cfg.has_section("env") and cfg.has_option("env", "version"):
            return cfg.get("env", "version").strip()
    except Exception:
        pass

    return None

def find_artifact(build_dir):
    # get uf2
    candidates = []
    candidates.extend(glob.glob(os.path.join(build_dir, "*.uf2")))
    for c in candidates:
        if os.path.exists(c):
            return c
    return None

def perform_copy(version, artifact_path):
    if not artifact_path:
        print(">>> perform_copy: no artifact found")
        return
    os.makedirs(os.path.join(PROJECT_DIR, "firmware"), exist_ok=True)
    base = "{}-V{}".format(PIOENV, version)
    ext = os.path.splitext(artifact_path)[1]
    dest_name = base + ext
    dest_file = os.path.join(PROJECT_DIR, "firmware", dest_name)
    try:
        shutil.copy(artifact_path, dest_file)
        print(">>> copied artifact to", dest_file)
    except Exception as e:
        print(">>> Error copying artifact:", e)
        return

    # process desc template
    tpl_candidates = glob.glob(os.path.join(PROJECT_DIR, "{}.desc".format(PIOENV)))
    if tpl_candidates:
        tpl = tpl_candidates[0]
        with open(tpl, "r", encoding="utf-8") as f:
            content = f.read()
        content = content.replace("{APP_VERSION}", version)
        
        # Read LICENSE.txt and add as "license-text" to JSON
        license_path = os.path.join(PROJECT_DIR, "LICENSE.txt")
        if os.path.exists(license_path):
            try:
                with open(license_path, "r", encoding="utf-8") as f:
                    license_text = f.read()
                # Assuming content is JSON, parse and add license-text
                import json
                desc_data = json.loads(content)
                desc_data["license-text"] = license_text
                content = json.dumps(desc_data, indent=2, ensure_ascii=False)
                print(">>> added license-text to .desc")
            except Exception as e:
                print(">>> Error reading LICENSE.txt:", e)
        else:
            print(">>> LICENSE.txt not found")
    else:
        content = "Name: {}\nVersion: {}\n".format(PIOENV, version)
        print(">>> no desc template found; creating minimal .desc")

    dest_desc = os.path.join(PROJECT_DIR, "firmware", base + ".desc")
    with open(dest_desc, "w", encoding="utf-8") as f:
        f.write(content)
    print(">>> wrote desc to", dest_desc)

    # create zip (use .zip name regardless of actual extension)
    zip_path = os.path.join(PROJECT_DIR, "firmware", base + ".zip")
    try:
        with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
            zf.write(dest_file, arcname=os.path.basename(dest_file))
            zf.write(dest_desc, arcname=os.path.basename(dest_desc))
        print(">>> created zip", zip_path)
    except Exception as e:
        print(">>> Error creating zip:", e)

# main post-build wrapper compatible with SCons post action signature
def copy_firmware(source, target, env):
    print(">>> copy_firmware post-build running...")
    out_build_dir = env.subst("$BUILD_DIR")
    print(">>> PROJECT_DIR:", PROJECT_DIR)
    print(">>> BUILD_DIR:", out_build_dir)

    artifact = find_artifact(out_build_dir)
    if not artifact:
        print(">>> Error: no UF2 found - aborting copy")
        return
    perform_copy(read_version(), artifact)

# register post actions on multiple targets to increase chance of execution
try:
    env.AddPostAction("upload", copy_firmware)
    print(">>> copy_firmware.py loaded; post actions registered")
except Exception as e:
    print(">>> Warning: failed to register some post actions:", e)

