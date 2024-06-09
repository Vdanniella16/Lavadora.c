<img width="457" alt="image" src="https://github.com/Vdanniella16/Lavadora.c/assets/161171728/23ae7593-d2f8-491c-b3d8-b4a64a274f34"># Lavadora.c
Lavadora Microprocesadores y Microcontroladores, 
```lavadora.c``` es un programa que controla una lavadora. La lavadora tiene ciclos de lavado, enjuague y centrifugado, y se inicia presionando un botón en el panel frontal. El programa se ejecuta en un microcontrolador STM32L053.

**Estructura del programa**

El programa se compone principalmente de las siguientes partes:

**Inclusión de archivos de librerías** ```#include``` - Se incluyen los archivos de encabezado que definen las funciones de biblioteca necesarias. En este programa, se incluyen ```stdint.h``` y ```stm32l053xx.h```
**Variables globales** - Se declaran las variables que se utilizan en todo el programa. En este programa, se declaran las siguientes variables:
```delay``` - Almacena el tiempo de retardo ```uint32_t```
```lavado``` - Bandera que indica si se ha iniciado el ciclo de lavado ```volatile uint8_t```
```clock_base_counter``` - Contador base de 1 segundo ```volatile uint8_t```
```seconds_unit```- Número de segundos del ciclo de lavado ```volatile uint8_t```
**Funciones** 
Funciones que implementan la funcionalidad individual del programa. En este programa, se definen las siguientes funciones:
```timer2``` - Inicializa un temporizador de 1 segundo
```timer6``` - Inicializa un temporizador de 500 milisegundos
```timer21``` - Inicializa un temporizador de 250 milisegundos
```print_bcd_7_segment_decoder_CC``` - Muestra un número en el display de 7 segmentos
```delay_ms``` - Retrasa un número específico de milisegundos 
```USART2_write``` - Envía un byte de datos a través de USART2 
```USART2_putstring``` - Imprime una cadena de caracteres a través de USART2
```USART2_putstring_E``` - Imprime una cadena de caracteres a través de USART2 y luego envía códigos de retorno  ```CR``` y de línea nueva ```LF```
```EXTI4_15_IRQHandler``` - Rutina de servicio de interrupción para la interrupción externa ```PA5```
```main``` - Punto de entrada del programa
**Flujo del programa**

**Inicialización**

Configuración del reloj (HSI 16MHz como reloj del sistema)
Configuración de puertos ```GPIO (PA, PB, PC)```
```PA2```, ```PA3``` se configuran como función alternativa para ```USART2```
```PB8``` ~ ```PB9```, ```PA0``` se configuran como pines de salida para el display de 7 segmentos
```PC3``` ~ ```PC9```, ```PA5``` se configuran como pines de control para la lavadora
Configuración de ```USART2``` (9600 baudios)
Configuración de la interrupción externa se genera una interrupción cuando se presiona ```PA5```
Configuración de los temporizadores ```timer2```: 1 segundo, ```timer6```: 500 ms, ```timer21```: 250 ms
Bucle principal

Si el ciclo de lavado no ha comenzado ```(lavado es 0)```, espera a que se presione el botón del panel frontal ```PA5```.
**Cuando se presiona** ```PA5```, ```(lavado se convierte en 1)```, comienza el ciclo de lavado.
El ciclo de lavado se controla mediante interrupciones del temporizador ```(timer2, timer6, timer21)```
Las interrupciones del temporizador realizan las siguientes acciones:
Llenado del tanque de lavado con agua ```1 segundo```
<img width="880" alt="image" src="https://github.com/Vdanniella16/Lavadora.c/assets/161171728/9af943bc-9245-436a-91ed-bebe28652198">

```Enjuague``` (500ms)
```Centrifugado``` (250ms)
```Conteo del tiempo de lavado``` (2 segundos)
Cuando el tiempo de lavado alcanza un cierto valor ```11 segundos```, el ciclo de lavado finaliza, se suena un timbre ```se enciende un LED``` y se notifica que el lavado ha terminado: <img width="457" alt="image" src="https://github.com/Vdanniella16/Lavadora.c/assets/161171728/d7e6486b-a555-4af6-a3c1-933b5b392c29">
**Y de esta manera se muestra que el ciclo ha concluido, mostrando la letra F en el display de 7 segmentos utilizado.**

Luego cuando todo el ciclo ha concluido, se ejecuta la función en el monitor serial indicando el mensaje que se puede abrir la tapa de la lavadora.
<img width="582" alt="image" src="https://github.com/Vdanniella16/Lavadora.c/assets/161171728/860bb599-9ae7-4758-a0a1-2581c4f187de">

Respuestas de 
A continuación se comparte un pequeño video con la explicación sobre el código y funcionamiento de la lavadora.

Se incluye la parte teórica evaluada en el examen final.
https://drive.google.com/file/d/1aGXfAV9W-YxE7bpt3fyB3X3y8inQ5C_G/view?usp=sharing 
<img width="157" alt="image" src="https://github.com/Vdanniella16/Lavadora.c/assets/161171728/1d15aed7-b84d-4100-9bfd-b07f4db155ed">



