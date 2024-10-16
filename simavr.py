Import("env")
env.AddCustomTarget(
    "simavr", 
    None, 
    'simavr -v -m atmega168 $BUILD_DIR/${PROGNAME}.elf', 
    title="Test without uploading"
)