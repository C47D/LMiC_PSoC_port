## Port de la librería LMiC para funcionar con microcontroladores PSoC5LP


Utilizando PSoC Creator 4.0 y el kit CY8CKIT-059 (PSoC 5LP).


### Cambios en la librería:

    En el fichero lmic.c se hacia referencia a la propiedad rxDelay
        que no esta mas en los archivos fuente, se busco el fichero
        lmic.c en otros proyectos y se actualizo el de este proyecto.

    Fichero hal.c contiene las funciones que se basan en el PSoC, se
        intento solo cambiar este archivo y evitar modificar los
        ficheros de la librería.


### Ajustes en los Build settings del proyecto:

Basados en otros proyectos encontrados en GitHub (concretamente en:
github.com/wklenk/lmic_rpi_lora_gps_hat) se añadieron flags a la
compilación de este proyecto.


    defines del preprocesador:
        En Compiler -> General -> Preprocessor Definitions se añadió:
            - CFG_DEBUG
            - CFG_eu868
            - CFG_sx1272_radio
            - CFG_DEBUG_LMIC
            - CFG_DEBUG_HAL


    flags del compilador:
        Se añadio el flag -std=gnu99 para poder inicializar variables
        en loops for. En Compiler -> command line.


### Estado:

El proyecto compila exitosamente, hay código remanente del port con
microcontroladores STM32 que hay que sustituir por el código pertinente
para que funcione en el PSoC5LP.

Proyecto compilado en modo DEBUG, optimizacion -Og.
