cmake_minimum_required(VERSION 3.16)

# Ajustes locales opcionales fuera de git (p. ej. pines, baudrate de pruebas)
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/sdkconfig.defaults.local")
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.defaults.local")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(jonas_firmware)
