cmake_minimum_required(VERSION 3.10)
project(MotorControlSim)

# Usa C++17 o posterior
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Archivos fuente
set(SOURCES
    RotaryMotor.cpp
    Encoder.cpp
    main.cpp
)

# Incluye directorio actual para headers
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Ejecutable final
add_executable(motor_control_sim ${SOURCES})

# Linkea librerías estándar si fuera necesario (normalmente no)
# target_link_libraries(motor_control_sim ...)
