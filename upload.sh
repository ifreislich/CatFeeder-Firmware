ESPOTA=/home/ianf/.platformio/packages/framework-arduinoespressif32/tools/espota.py

for i in aviwe catkin gobbolino oliver sophie
do
    python3 ${ESPOTA} -r -i feeder-$i.lan -p 3232 --auth= -f .pio/build/esp32dev/firmware.bin
done
