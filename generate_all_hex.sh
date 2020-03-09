# your EasyCon/Firmware path
EasyCon_Firmware_path=../EasyCon/Firmware

make -f atmega16u2.mk clean
make -f atmega16u2.mk
make -f atmega32u4.mk clean
make -f atmega32u4.mk

# copy it from output to EasyCon/Firmware path
cp -f ./atmega16u2/atmega16u2.hex  ${EasyCon_Firmware_path}/ATmega16U2\ v3.hex
cp -f ./atmega32u4/atmega32u4.hex  ${EasyCon_Firmware_path}/ATmega32U4\ v3.hex