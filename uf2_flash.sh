#!/bin/bash

set -o errexit
: "${GPIONUM:=1}"
: "${BOOTPATH:=ITSY840BOOT}"

PROJECT_NAME=`basename $PWD`

cargo objcopy --release --bin $PROJECT_NAME -- -O binary target/toflash.bin
uf2conv target/toflash.bin --base 0x26000 --family 0xADA52840 --output target/toflash.uf2
rm target/toflash.bin

cp target/toflash.uf2 /run/media/erik/$BOOTPATH/

