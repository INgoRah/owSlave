# Links the bootloader into the boot section, without the C runtime
# (see start.S). custom_bl_start: BL_START of include/owbootloader.h
# as a byte address, e.g. 0x3800 for the ATmega168.
#
# The upload writes the application of custom_bl_app_env along with
# it: an ISP upload erases the whole chip.
import os

Import("env")

start = env.GetProjectOption("custom_bl_start")
env.Append(LINKFLAGS=[
    "-nostartfiles",
    "-Wl,--section-start=.text=" + start,
])

app_env = env.GetProjectOption("custom_bl_app_env", "")
if app_env:
    app = os.path.join(os.path.dirname(env.subst("$BUILD_DIR")), app_env, "firmware.hex")
    if os.path.isfile(app):
        env.Append(UPLOADERFLAGS=["-Uflash:w:%s:i" % app])
    elif "upload" in COMMAND_LINE_TARGETS:
        print("Warning: no application %s, build env %s first; "
              "only the bootloader is written" % (app, app_env))

# The code has to end below custom_bl_end: the end of the flash, on the
# ATtinys their data page (BL_DATA of owbootloader.h).
end = env.GetProjectOption("custom_bl_end", "")
if end:
    def check_size(target, source, env):
        import subprocess
        out = subprocess.run([env.subst("$SIZETOOL"), "-A", str(target[0])],
                             capture_output=True, text=True).stdout
        size = 0
        for line in out.splitlines():
            f = line.split()
            if f and f[0] == ".text":
                size = int(f[1])
        limit = int(end, 0) - int(start, 0)
        print("Bootloader: %d of %d bytes" % (size, limit))
        if size > limit:
            print("Error: the bootloader does not fit, %d bytes too large" % (size - limit))
            env.Exit(1)

    env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", check_size)
