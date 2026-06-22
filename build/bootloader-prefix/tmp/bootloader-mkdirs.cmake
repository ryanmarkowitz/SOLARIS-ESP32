# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/ryan/.espressif/v5.5.2/esp-idf/components/bootloader/subproject"
  "/home/ryan/Documents/SOLARIS_ESP32/SOLARIS-ESP32/build/bootloader"
  "/home/ryan/Documents/SOLARIS_ESP32/SOLARIS-ESP32/build/bootloader-prefix"
  "/home/ryan/Documents/SOLARIS_ESP32/SOLARIS-ESP32/build/bootloader-prefix/tmp"
  "/home/ryan/Documents/SOLARIS_ESP32/SOLARIS-ESP32/build/bootloader-prefix/src/bootloader-stamp"
  "/home/ryan/Documents/SOLARIS_ESP32/SOLARIS-ESP32/build/bootloader-prefix/src"
  "/home/ryan/Documents/SOLARIS_ESP32/SOLARIS-ESP32/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/ryan/Documents/SOLARIS_ESP32/SOLARIS-ESP32/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/ryan/Documents/SOLARIS_ESP32/SOLARIS-ESP32/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
