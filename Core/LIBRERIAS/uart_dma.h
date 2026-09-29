/*
 * uart_dma.h
 *
 * Librería para administrar comunicaciones UART mediante DMA.
 *
 * Esta librería será utilizada en el control de tierra para:
 *
 * - USART1: comunicación principal con el módulo XBee.
 * - UART5: comunicación con la pantalla Nextion.
 *
 * La recepción funciona continuamente mediante DMA circular
 * y detección de pausas por medio del evento IDLE.
 *
 * La transmisión también utiliza DMA para evitar que el
 * microcontrolador quede detenido esperando que termine
 * el envío de todos los bytes.
 */

#ifndef UART_DMA_H_
#define UART_DMA_H_


/* Incluye las definiciones de STM32 HAL, entre ellas:
 *
 * - UART_HandleTypeDef
 * - HAL_StatusTypeDef
 * - HAL_OK
 * - HAL_ERROR
 * - HAL_BUSY
 */
#include "main.h"


/* Permite utilizar tipos con tamaños definidos:
 *
 * - uint8_t
 * - uint16_t
 * - uint32_t
 */
#include <stdint.h>


/* Permite utilizar size_t para representar cantidades
 * de bytes y tamaños de búferes.
 */
#include <stddef.h>


/* ================================================================
 * TAMAÑOS DE LOS BÚFERES
 * ================================================================
 */


/* Tamaño del búfer utilizado directamente por el DMA receptor.
 *
 * El DMA almacena aquí los bytes que llegan por USART1 o UART5.
 * Como el DMA está configurado en modo circular, al llegar a la
 * última posición vuelve automáticamente a la posición cero.
 *
 * Este tamaño es suficiente para recibir una trama completa
 * del protocolo del USV.
 */
#define UART_DMA_RX_BUFFER_SIZE       256U


/* Tamaño del búfer circular utilizado por el programa.
 *
 * Los bytes nuevos se copian desde el búfer DMA hacia este FIFO.
 * Su tamaño es mayor que el búfer DMA para permitir almacenar
 * temporalmente más de una trama.
 *
 * Esto evita perder datos cuando el programa principal todavía
 * está procesando una trama anterior.
 */
#define UART_DMA_FIFO_SIZE            512U


/* Tamaño máximo permitido para una transmisión DMA.
 *
 * Debe ser suficiente para contener:
 *
 * - Una trama completa de comandos para el bote.
 * - Una trama de telemetría.
 * - Un comando para la pantalla Nextion.
 */
#define UART_DMA_TX_BUFFER_SIZE       256U


/* Tamaño máximo de una línea recibida.
 *
 * Las tramas de comunicación del XBee terminan con:
 *
 * "\r\n"
 *
 * Los bytes se acumulan hasta detectar el carácter '\n'.
 */
#define UART_DMA_LINE_BUFFER_SIZE     256U


/*
 * REQUISITOS DE USO CON EL UART_DMA.C CORREGIDO
 *
 * - Inicializar primero USART1, UART5 y sus canales GPDMA.
 * - Configurar RX circular y TX normal, con datos DMA de un byte,
 *   para UART de 8 bits sin paridad.
 * - Habilitar las IRQ UART y DMA utilizadas por HAL.
 * - Para cada UART y su DMA RX, usar la misma prioridad de
 *   preempcion, evitando callbacks RX que se interrumpan entre si.
 * - Conservar los eventos de media transferencia, fin e IDLE.
 * - Llamar las funciones publicas desde el programa principal,
 *   no desde interrupciones; usar un solo consumidor por UART.
 * - No repetir la inicializacion mientras los DMA estan activos.
 *
 * Los buffers absorben pausas cortas, pero no evitan toda perdida:
 * el FIFO de 512 posiciones permite guardar 511 bytes pendientes.
 * Una linea admite hasta 255 bytes, reservando uno para '\0'.
 * Hay que procesar los datos y las interrupciones a tiempo para
 * evitar que el DMA sobrescriba bytes antes de poder copiarlos.
 *
 * IDLE informa una pausa; no demuestra que una trama este completa.
 * Esta libreria transporta bytes. trama_usv.c construye, verifica
 * el CRC-8 y decodifica los valores del protocolo del proyecto.
 *
 * Estos tres callbacks HAL se implementan una sola vez en uart_dma.c:
 *
 * HAL_UARTEx_RxEventCallback()
 * HAL_UART_TxCpltCallback()
 * HAL_UART_ErrorCallback()
 *
 * No duplicarlos en main.c ni en otra libreria.
 */

/* ================================================================
 * FUNCIONES PÚBLICAS
 * ================================================================
 */


/* Inicializa la recepción mediante DMA de USART1 y UART5.
 *
 * Esta función debe llamarse una sola vez después de que CubeMX
 * inicialice los periféricos.
 *
 * Orden recomendado dentro de main.c:
 *
 * MX_GPIO_Init();
 * MX_GPDMA1_Init();
 * MX_ADC1_Init();
 * MX_ICACHE_Init();
 * MX_UART5_Init();
 * MX_USART1_UART_Init();
 *
 * UART_DMA_Iniciar();
 *
 * La función inicia:
 *
 * - Recepción circular de USART1 para el XBee.
 * - Recepción circular de UART5 para Nextion.
 * - Detección de pausas mediante el evento IDLE.
 *
 * Retorna:
 *
 * HAL_OK    = Las dos UART comenzaron correctamente.
 * HAL_ERROR = Existe un error en algún manejador o canal DMA.
 * HAL_BUSY  = Alguna UART o canal DMA ya estaba ocupado.
 */
/*
 * Una segunda llamada retorna HAL_BUSY sin borrar los buffers.
 * Si falta un manejador o DMA RX, corregir CubeMX antes de continuar.
 * Si los contextos se prepararon pero un arranque HAL fallo,
 * UART_DMA_Procesar() puede reintentar esa recepcion.
 * El retorno de Iniciar siempre debe comprobarse.
 */

HAL_StatusTypeDef UART_DMA_Iniciar(void);


/* Procesa solicitudes pendientes de recuperación.
 *
 * Cuando ocurre un error UART, la interrupción activa una
 * bandera. Esta función detiene y reinicia la recepción DMA
 * desde el programa principal.
 *
 * Debe llamarse periódicamente dentro del while(1):
 *
 * while (1)
 * {
 *     UART_DMA_Procesar();
 * }
 *
 * No debe llamarse desde una interrupción.
 */
/*
 * Tambien descarta datos pendientes tras una perdida de bytes.
 * La recuperacion RX no aborta un envio TX que sigue activo.
 * Una reserva TX pendiente solo se libera cuando HAL devuelve
 * el transmisor a HAL_UART_STATE_READY.
 */

void UART_DMA_Procesar(void);


/* Consulta cuántos bytes existen disponibles en el FIFO
 * de recepción de una UART.
 *
 * Parámetro:
 *
 * huart = Dirección del manejador de la UART.
 *
 * Ejemplos:
 *
 * UART_DMA_DatosDisponibles(&huart1);
 * UART_DMA_DatosDisponibles(&huart5);
 *
 * Retorna:
 *
 * Cantidad de bytes pendientes de lectura.
 * Retorna 0 si no hay datos o si la UART no es válida.
 */
/*
 * Tambien retorna cero si hay una perdida o recuperacion pendiente.
 * El resultado es una instantanea: pueden llegar mas bytes despues.
 */

size_t UART_DMA_DatosDisponibles(
        UART_HandleTypeDef *huart);


/* Lee bytes desde el búfer circular de recepción.
 *
 * Esta función puede utilizarse para recibir datos binarios
 * o mensajes que no necesariamente terminan en '\n'.
 *
 * Será especialmente útil para procesar las respuestas binarias
 * y eventos táctiles enviados por la pantalla Nextion.
 *
 * Parámetros:
 *
 * huart:
 * UART desde donde se leerán los datos.
 *
 * destino:
 * Búfer donde se copiarán los bytes recibidos.
 *
 * capacidad:
 * Cantidad máxima de bytes que pueden copiarse al destino.
 *
 * Retorna:
 *
 * Cantidad real de bytes copiados.
 * Retorna 0 cuando no existen datos disponibles.
 */
/*
 * No agrega '\0': usar unicamente la cantidad de bytes retornada.
 * Si detecta una discontinuidad, descarta la lectura y retorna 0;
 * no procesar contenido de destino cuando ese retorno sea cero.
 * Ante una perdida en UART5, reiniciar tambien el decodificador
 * de respuestas binarias de Nextion para no unir fragmentos.
 */

size_t UART_DMA_Leer(
        UART_HandleTypeDef *huart,
        uint8_t *destino,
        size_t capacidad);


/* Busca una línea completa terminada en el carácter '\n'.
 *
 * Esta función está diseñada principalmente para las tramas
 * intercambiadas mediante el XBee.
 *
 * Ejemplos:
 *
 * $CMD,...*A5\r\n
 * $TEL,...*3F\r\n
 *
 * La función conserva los caracteres '\r' y '\n' dentro
 * de la cadena entregada.
 *
 * También agrega automáticamente el carácter nulo '\0'
 * al final, permitiendo utilizar el resultado como una
 * cadena normal de lenguaje C.
 *
 * Parámetros:
 *
 * huart:
 * UART desde donde se desea obtener la línea.
 *
 * destino:
 * Arreglo de caracteres donde se copiará la trama.
 *
 * capacidad:
 * Tamaño total disponible en el arreglo destino.
 *
 * Retorna:
 *
 * Longitud de la línea recibida, sin contar el carácter '\0'.
 * Retorna 0 si todavía no existe una línea completa.
 */
/*
 * En el proyecto actual las tramas se identifican con:
 *
 * $PUSVU = comandos desde tierra hacia el bote.
 * $PUSVD = telemetria desde el bote hacia tierra.
 *
 * Los ejemplos $CMD y $TEL anteriores se conservan como comentarios.
 * La funcion no verifica CRC: pasar la linea a trama_usv.c.
 * Puede entregar una de varias lineas recibidas en la misma pausa.
 * Descarta lineas demasiado largas o que contienen un NUL interno.
 * Tras una perdida, ignora el fragmento hasta encontrar el siguiente LF.
 * Con parametros validos deja destino vacio si no entrega una linea.
 * No alternar Leer y LeerLinea en la misma UART mientras se acumula
 * una linea: ambas funciones consumen los bytes del mismo FIFO.
 */

size_t UART_DMA_LeerLinea(
        UART_HandleTypeDef *huart,
        char *destino,
        size_t capacidad);


/* Transmite un bloque de bytes mediante DMA.
 *
 * Antes de iniciar la transmisión, la función copia los datos
 * recibidos hacia un búfer interno.
 *
 * Esta copia es importante porque HAL_UART_Transmit_DMA()
 * continúa utilizando el búfer después de que la función
 * retorna.
 *
 * Parámetros:
 *
 * huart:
 * UART utilizada para realizar la transmisión.
 *
 * datos:
 * Dirección del primer byte que se desea transmitir.
 *
 * longitud:
 * Cantidad de bytes que se transmitirán.
 *
 * Retorna:
 *
 * HAL_OK:
 * La transmisión comenzó correctamente.
 *
 * HAL_BUSY:
 * La UART todavía está transmitiendo un mensaje anterior.
 *
 * HAL_ERROR:
 * Los parámetros son inválidos o el mensaje supera
 * UART_DMA_TX_BUFFER_SIZE.
 */
/*
 * Requiere un DMA TX enlazado. Comprueba tanto la reserva interna
 * como gState para respetar envios iniciados por otra libreria.
 * HAL_OK indica que el envio comenzo, no que el bote ya lo recibio.
 * HAL_BUSY no encola el mensaje: el llamador decide cuando reintentar.
 */

HAL_StatusTypeDef UART_DMA_Transmitir(
        UART_HandleTypeDef *huart,
        const uint8_t *datos,
        uint16_t longitud);


/* Transmite una cadena de texto mediante DMA.
 *
 * La función calcula automáticamente la longitud utilizando
 * strlen(), por lo que el texto debe terminar con '\0'.
 *
 * Ejemplo para enviar una trama al XBee:
 *
 * UART_DMA_TransmitirTexto(&huart1, trama_comando);
 *
 * Ejemplo para enviar un comando a Nextion:
 *
 * UART_DMA_TransmitirTexto(&huart5, comando_nextion);
 *
 * Parámetros:
 *
 * huart = UART utilizada para transmitir.
 * texto = Cadena terminada en carácter nulo.
 *
 * Retorna:
 *
 * HAL_OK, HAL_BUSY o HAL_ERROR.
 */
/*
 * En el .c corregido la longitud se busca con un recorrido limitado,
 * en lugar del strlen() de la version descrita en el comentario anterior.
 * texto sigue teniendo que ser una cadena valida terminada en '\0'.
 * El NUL final no se transmite.
 *
 * Esta funcion no agrega CR/LF, CRC ni los tres bytes finales Nextion.
 * Para las tramas USV, enviar la cadena ya construida por trama_usv.c.
 * Para instrucciones HMI, utilizar NEXTION_EnviarComando() u otra
 * funcion de nextion.c: ellas agregan 0xFF 0xFF 0xFF.
 */

HAL_StatusTypeDef UART_DMA_TransmitirTexto(
        UART_HandleTypeDef *huart,
        const char *texto);


/* Consulta si una transmisión DMA continúa activa.
 *
 * Puede utilizarse antes de enviar una trama nueva.
 *
 * Ejemplo:
 *
 * if (UART_DMA_TransmisionOcupada(&huart1) == 0U)
 * {
 *     UART_DMA_TransmitirTexto(&huart1, trama_comando);
 * }
 *
 * Retorna:
 *
 * 0 = UART disponible.
 * 1 = Existe una transmisión DMA en proceso.
 */
/*
 * Tambien reconoce un envio iniciado directamente por nextion.c.
 * Para una UART no administrada retorna 0; ese cero no valida
 * el manejador ni sustituye comprobar el retorno de Transmitir.
 */

uint8_t UART_DMA_TransmisionOcupada(
        UART_HandleTypeDef *huart);


/* Consulta si ocurrió un desbordamiento del búfer circular.
 *
 * Un desbordamiento sucede cuando llegan bytes más rápidamente
 * de lo que el programa principal puede procesarlos.
 *
 * Después de consultar esta bandera, la función la coloca
 * nuevamente en cero.
 *
 * Retorna:
 *
 * 0 = No ocurrió un desbordamiento nuevo.
 * 1 = Se descartaron uno o más bytes.
 *
 * Si ocurre un desbordamiento, la trama debe descartarse.
 * El CRC también permitirá detectar que la información
 * recibida está incompleta.
 */
/*
 * Tambien informa descartes por linea invalida o por error UART.
 * Consultar esta bandera no cancela el descarte interno pendiente.
 * El CRC-8 ayuda a detectar alteraciones, pero no garantiza detectar
 * toda corrupcion ni sustituye descartar mensajes discontinuos.
 */

uint8_t UART_DMA_HuboDesbordamiento(
        UART_HandleTypeDef *huart);


/* Retorna la cantidad acumulada de errores detectados
 * por una UART.
 *
 * El contador aumenta cuando HAL ejecuta:
 *
 * HAL_UART_ErrorCallback()
 *
 * Puede utilizarse para diagnosticar:
 *
 * - Ruido eléctrico.
 * - Pérdida de comunicación.
 * - Errores de paridad.
 * - Errores de framing.
 * - Overrun del periférico UART.
 *
 * El valor no se borra después de consultarlo.
 */
uint32_t UART_DMA_CantidadErrores(
        UART_HandleTypeDef *huart);


#endif /* UART_DMA_H_ */

