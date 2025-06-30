Simulación de motor en C++
==========================

Instrucciones para compilar:

~~~bash
mkdir build
cd build
cmake ..
make 
~~~

Ejecución:

~~~bash
./motor_control_sim
~~~

El resultado de una ejecución es un `.csv` con las variables de estado del modelo al aplicarle una señal de potencia trapezoidal, en sentido horario y antihorario.