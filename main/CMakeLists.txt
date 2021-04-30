set(srcs "main.c"
    "display.c"
    "esp-i2c.c"
    "lib/st7789/st7789.c"
    "lib/st7789/fontx.c"
    "lib/st7789/pngle.c"
    "lib/MPU6050/esp32_i2c_rw.c"
    "lib/MPU6050/mpu6050.c"
    "lib/vesc/crc.c"
    "lib/vesc/buffer.c"
    "lib/vesc/packet.c"
    "lib/ADS1015/src/ADS1015.c"
    "lib/melody/melody.c"
    "lib/melody/melody_notes.c"
    )

# Only ESP32 has enough memory to do jpeg decoding
if(IDF_TARGET STREQUAL "esp32")
    list(APPEND srcs "lib/st7789/decode_image.c")
endif()

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS ".;./lib/st7789;./lib/MPU6050;./lib/vesc;./lib/ADS1015/src;./lib/melody")


# Create a SPIFFS image from the contents of the 'font' directory
# that fits the partition named 'storage'. FLASH_IN_PROJECT indicates that
# the generated image should be flashed when the entire project is flashed to
# the target with 'idf.py -p PORT flash
spiffs_create_partition_image(storage ../filesystem FLASH_IN_PROJECT)
