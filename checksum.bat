rem fill gaps
srec_cat .pio\build\nucleo_f429zi\firmware.hex ^
-Intel ^
-fill 0xFF 0x08080000 0x080E0000 ^
-o filled.hex ^
-Intel ^
-line-length=44

rem sign
srec_cat filled.hex ^
-Intel ^
-crop 0x08080000 0x080DFFFC ^
-STM32 0x080DFFFC ^
-o signed_app.hex ^
-Intel

rem hex to binary
arm-none-eabi-objcopy --input-target=ihex --output-target=binary signed_app.hex application.bin

del filled.hex
del signed_app.hex

tar.exe -a -cf application.zip application.bin

copy application.bin signed_app.bin

@REM pause
