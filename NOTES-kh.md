git submodule update --init --recursive

npm install --global xpm
xpm install --global @xpack-dev-tools/arm-none-eabi-gcc@latest
Installing globally in '/home/kevinh/opt/xPacks/@xpack-dev-tools/arm-none-eabi-gcc/9.2.1-1.1.1'...

make BOARD=pca10056 all
make BOARD=othernet_ppr all DEBUG=1

pip3 install --user adafruit-nrfutil

# To program with jlink

```

JLinkExe -device nrf52840_xxaa -if swd -speed 4000 -autoconnect 1

objdump -s ./_build/build-othernet_ppr/othernet_ppr_bootloader-0.3.2-108-g718c310-dirty_s140_6.1.1.hex  | less


nrfjprog -e -f nrf52; nrfjprog -f nrf52 --program ./_build/build-othernet_ppr/othernet_ppr_bootloader-0.3.2-*-dirty_s140_6.1.1.hex
```
