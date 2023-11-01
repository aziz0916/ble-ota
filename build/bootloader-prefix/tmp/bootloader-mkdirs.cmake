# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/aziz/esp/esp-idf/components/bootloader/subproject"
  "C:/Users/aziz/Desktop/ESP-IDF/ESP32H2/esp32-ota-ble-main-add-service-light-sleep-spiffs-json-refactoring/build/bootloader"
  "C:/Users/aziz/Desktop/ESP-IDF/ESP32H2/esp32-ota-ble-main-add-service-light-sleep-spiffs-json-refactoring/build/bootloader-prefix"
  "C:/Users/aziz/Desktop/ESP-IDF/ESP32H2/esp32-ota-ble-main-add-service-light-sleep-spiffs-json-refactoring/build/bootloader-prefix/tmp"
  "C:/Users/aziz/Desktop/ESP-IDF/ESP32H2/esp32-ota-ble-main-add-service-light-sleep-spiffs-json-refactoring/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/aziz/Desktop/ESP-IDF/ESP32H2/esp32-ota-ble-main-add-service-light-sleep-spiffs-json-refactoring/build/bootloader-prefix/src"
  "C:/Users/aziz/Desktop/ESP-IDF/ESP32H2/esp32-ota-ble-main-add-service-light-sleep-spiffs-json-refactoring/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/aziz/Desktop/ESP-IDF/ESP32H2/esp32-ota-ble-main-add-service-light-sleep-spiffs-json-refactoring/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/aziz/Desktop/ESP-IDF/ESP32H2/esp32-ota-ble-main-add-service-light-sleep-spiffs-json-refactoring/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
