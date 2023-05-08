ESPOTA=/home/ianf/.arduino15/packages/esp32/hardware/esp32/2.0.9/tools/espota.py

for i in aviwe catkin fred gobbolino oliver sophie
do
    python3 ${ESPOTA} -r -i catfeeder-$i.local -p 3232 --auth= -f ../.pio/build/esp32dev/firmware.bin
done
