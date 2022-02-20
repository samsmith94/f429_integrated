Import("env")

# Custom HEX from ELF
env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    env.VerboseAction(" ".join([
        "$OBJCOPY",
        "-O",
        "ihex",
        "$BUILD_DIR/${PROGNAME}.elf",
        "$BUILD_DIR/${PROGNAME}.hex"
    ]), "Building $BUILD_DIR/${PROGNAME}.hex")
)
# //arm-none-eabi-objcopy -O ihex ${ProjName}.elf ${ProjName}.hex