## Librería LMiC para PSoC5LP

Colaboración entre [jnavarro7](https://github.com/jnavarro7) y [C47D](https://github.com/C47D).

### Herramientas utilizadas
- PSoC Creator 4.0
- kit CY8CKIT-059 (PSoC 5LP).


### Librería LMiC

La librería LMiC es desarrollada por IBM para dar soporte a LoRaWAN MAC en sistemas embebidos. El codigo esta escrito en C, y da soporte a radios que utilizan los chips SX1272 y SX1276 de Semtech.

Los cambios necesarios para hacer funcionar esta librería en dispositivos PSoC5LP se realizaron principalmente en los archivos hal.c y debug.c (funcionalidad opcional, pero muy util).

Los cambios necesarios para el funcionamiento de esta librería en nuevas plataformas se realizan en el archivo hal.c y se describen en la sección 3 Hardware Abstraction Layer del documento LMiC-v1.6 (disponible en el directorio doc de este repositorio).


#### Problemas resueltos
* En el fichero lmic.c se hacia referencia a la propiedad rxDelay
que no esta mas en los archivos fuente, se busco el fichero
lmic.c en otros proyectos y se actualizo el de este proyecto.


### Ajustes en los Build settings del proyecto:

* defines del preprocesador:

En el documento LMiC-v1.6 se enlistan los defines del preprocesador que se deben de configurar para el funcionamiento de la librería.
En Compiler -> General -> Preprocessor Definitions se añadió:
- CFG_DEBUG = Habilita la librería de debug integrada en lmic, se utiliza un LED y UART configurada 115200 8n1.
- CFG_us915 = Banda utilizada en NorteAmérica.
- CFG_sx1272_radio = Radio LoRa utilizado.


Basados en otros proyectos encontrados en GitHub (concretamente en:
github.com/wklenk/lmic_rpi_lora_gps_hat) se añadieron flags a la
compilación de este proyecto.
- CFG_DEBUG_LMIC
- CFG_DEBUG_HAL


* flags del compilador:
Se añadio el flag -std=gnu99 para poder inicializar variables en loops for. En Compiler -> command line.


### Estado:

El proyecto compila exitosamente, hay código remanente del port con
microcontroladores STM32 que hay que sustituir por el código pertinente
para que funcione en el PSoC5LP.

Proyecto compilado en modo DEBUG, optimizacion -Og.
