# The upload of an application writes the 1-Wire bootloader of its MCU
# (custom_bl_env, e.g. attiny84-boot) along with it: an ISP upload
# erases the whole chip, the bootloader would be gone else. The
# bootloader is built first, with the same environment (OWID_ADR,
# OWID_BUS: its default ROM, see owid.py), the application has to end
# below it (custom_bl_start of the bootloader env).
#   OWID_ADR=4 OWID_BUS=1 pio run -e attiny84-bl -t upload
import os
import subprocess

Import("env")

bl_env = env.GetProjectOption("custom_bl_env", "")

if bl_env and "upload" in COMMAND_LINE_TARGETS:
    bl_hex = os.path.join(os.path.dirname(env.subst("$BUILD_DIR")), bl_env, "firmware.hex")
    bl_start = int(env.GetProjectConfig().get("env:" + bl_env, "custom_bl_start"), 0)

    def hex_end(path):
        end = 0
        base = 0
        with open(path) as f:
            for line in f:
                b = bytes.fromhex(line.strip()[1:])
                if b[3] == 0 and b[0]:
                    end = max(end, base + ((b[1] << 8) | b[2]) + b[0])
                elif b[3] in (2, 4):
                    base = ((b[4] << 8) | b[5]) << (4 if b[3] == 2 else 16)
        return end

    def build_bl(source, target, env):
        app = env.subst("$BUILD_DIR/${PROGNAME}.hex")
        end = hex_end(app)
        if end > bl_start:
            print("Error: the application ends at %#x, the bootloader %s starts at %#x"
                  % (end, bl_env, bl_start))
            env.Exit(1)
        print("Bootloader %s, written along with the application" % bl_env)
        subprocess.run([env.subst("$PYTHONEXE"), "-m", "platformio", "run",
                        "-d", env.subst("$PROJECT_DIR"), "-e", bl_env], check=True)

    env.AddPreAction("upload", build_bl)
    env.Append(UPLOADERFLAGS=["-Uflash:w:%s:i" % bl_hex])
