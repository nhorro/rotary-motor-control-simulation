Estudio de control de motor rotatorio
=====================================

Este proyecto contiene jupyter notebooks y código python de una simulación de motor rotatorio y su controlador de posición.
El objetivo es contar con un modelo simulado para probar controladores de aplicaciones que hagan Pan&Tilt.
También incluyen algunos de los componentes [migrados a C++](./cpp), para su uso en simuladores.

![penguin-with-radar](./assets/portable-radar-sketch.png)

**Cuadernos**

- [01 - Modelo de control rotatatorio básico](./01-MotorModel.ipynb)
- [02 - Controlador de velocidad PID](02-SpeedController.ipynb)
- [03 - Control de posición y planificación de trayectoria](03-PositionController.ipynb)
- 4: pendiente: modelo de transferencia electromecánica.
- [05 - Prueba de modelo de motor en C++](05-CppModelTest.ipynb).

