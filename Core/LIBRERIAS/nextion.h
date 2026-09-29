/*
 * nextion.h
 *
 * Librería para la comunicación entre el STM32H562RGT6
 * y una pantalla Nextion mediante UART y DMA.
 *
 * En el control de tierra se utilizará:
 *
 * UART5_TX -> envío de información hacia Nextion.
 * UART5_RX -> recepción de eventos desde Nextion.
 *
 * Configuración de comunicación:
 *
 * - Velocidad: 115200 bits por segundo.
 * - Longitud: 8 bits.
 * - Paridad: ninguna.
 * - Bits de parada: 1.
 *
 * Cada comando enviado a Nextion termina con:
 *
 * 0xFF 0xFF 0xFF
 */

#ifndef NEXTION_H_
#define NEXTION_H_


/*
 * Incluye:
 *
 * - UART_HandleTypeDef.
 * - HAL_StatusTypeDef.
 * - Definiciones HAL para el STM32H562.
 *
 * Se utiliza main.h en lugar de stm32f4xx_hal.h porque
 * este proyecto trabaja con un STM32H562RGT6.
 */
#include "main.h"


/*
 * Incluye los tipos:
 *
 * uint8_t
 * uint16_t
 * int32_t
 */
#include <stdint.h>


/*
 * Tamaño del buffer permanente utilizado para las
 * transmisiones de UART5_TX mediante DMA.
 *
 * Debe tener suficiente espacio para almacenar varios
 * comandos de la pantalla en una sola transferencia.
 */
#define NEXTION_BUFFER_TX_TAMANO       512U


/*
 * Tamaño máximo reservado para construir un comando
 * individual antes de copiarlo al buffer del DMA.
 *
 * Ejemplo de un comando individual:
 *
 * tParada.txt="EMERGENCIA"
 */
#define NEXTION_COMANDO_MAXIMO         128U


/*
 * Inicializa la librería de comunicación con Nextion.
 *
 * La inicialización física de UART5 y GPDMA1 se realiza
 * previamente mediante las funciones generadas por CubeMX:
 *
 * MX_GPDMA1_Init();
 * MX_UART5_Init();
 *
 * Parámetro:
 *
 * huart = manejador de la UART conectada a Nextion.
 *
 * En este proyecto se utilizará:
 *
 * &huart5
 *
 * Retorna:
 *
 * HAL_OK    = manejador válido.
 * HAL_ERROR = manejador inválido.
 */
HAL_StatusTypeDef NEXTION_Init(
    UART_HandleTypeDef *huart);


/*
 * Verifica si la parte transmisora de la UART se encuentra
 * disponible para iniciar una nueva transferencia DMA.
 *
 * Esta función revisa únicamente el estado de transmisión.
 * De esa manera UART5 puede permanecer recibiendo mediante
 * DMA mientras se envía información hacia la pantalla.
 *
 * Parámetro:
 *
 * huart = manejador de UART que será consultado.
 *
 * Retorna:
 *
 * 1 = transmisión disponible.
 * 0 = transmisión ocupada o manejador inválido.
 */
uint8_t NEXTION_TransmisionLista(
    UART_HandleTypeDef *huart);


/*
 * Envía un comando completo hacia Nextion utilizando DMA.
 *
 * La función agrega automáticamente los tres bytes finales:
 *
 * 0xFF 0xFF 0xFF
 *
 * Por lo tanto, estos bytes no deben incluirse manualmente
 * dentro del texto recibido en el parámetro comando.
 *
 * Ejemplo:
 *
 * NEXTION_EnviarComando(
 *     &huart5,
 *     "tParada.txt=\"NORMAL\"");
 *
 * Parámetros:
 *
 * huart   = manejador de la UART conectada a Nextion.
 * comando = instrucción que se enviará a la pantalla.
 *
 * Retorna:
 *
 * HAL_OK    = transmisión iniciada correctamente.
 * HAL_BUSY  = existe otra transmisión DMA en ejecución.
 * HAL_ERROR = parámetro inválido o comando demasiado largo.
 */
HAL_StatusTypeDef NEXTION_EnviarComando(
    UART_HandleTypeDef *huart,
    const char *comando);


/*
 * Envía una cadena de texto hacia un objeto de Nextion.
 *
 * El objeto utilizado debe ser un componente que posea
 * la propiedad txt dentro del proyecto HMI.
 *
 * Ejemplo:
 *
 * NEXTION_EnviarTexto(
 *     &huart5,
 *     "tParada",
 *     "NORMAL");
 *
 * La función genera:
 *
 * tParada.txt="NORMAL"
 *
 * Parámetros:
 *
 * huart  = manejador de UART.
 * objeto = nombre del componente dentro de Nextion.
 * texto  = contenido que se mostrará.
 *
 * Retorna:
 *
 * HAL_OK    = transmisión iniciada correctamente.
 * HAL_BUSY  = transmisor DMA ocupado.
 * HAL_ERROR = parámetro inválido o comando demasiado largo.
 */
HAL_StatusTypeDef NEXTION_EnviarTexto(
    UART_HandleTypeDef *huart,
    const char *objeto,
    const char *texto);


/*
 * Envía un número entero hacia la propiedad val
 * de un objeto de Nextion.
 *
 * Se puede utilizar con:
 *
 * - Indicadores.
 * - Barras de progreso.
 * - Gauges.
 * - Variables numéricas.
 *
 * Ejemplo:
 *
 * NEXTION_EnviarEntero(
 *     &huart5,
 *     "zAngulo",
 *     90);
 *
 * La función genera:
 *
 * zAngulo.val=90
 *
 * Parámetros:
 *
 * huart  = manejador de UART.
 * objeto = nombre del componente dentro de Nextion.
 * valor  = valor entero que será asignado.
 *
 * Retorna:
 *
 * HAL_OK    = transmisión iniciada correctamente.
 * HAL_BUSY  = transmisor DMA ocupado.
 * HAL_ERROR = parámetro inválido o comando demasiado largo.
 */
HAL_StatusTypeDef NEXTION_EnviarEntero(
    UART_HandleTypeDef *huart,
    const char *objeto,
    int32_t valor);


/*
 * Convierte un valor decimal a texto y lo envía
 * hacia un objeto de texto de Nextion.
 *
 * Ejemplo:
 *
 * NEXTION_EnviarFloat(
 *     &huart5,
 *     "tBateria",
 *     12.5f,
 *     1);
 *
 * La función genera:
 *
 * tBateria.txt="12.5"
 *
 * Los decimales se limitan internamente a un máximo de 3.
 *
 * Parámetros:
 *
 * huart     = manejador de UART.
 * objeto    = nombre del objeto de texto.
 * valor     = número decimal que se mostrará.
 * decimales = cantidad de cifras después del punto.
 *
 * Retorna:
 *
 * HAL_OK    = transmisión iniciada correctamente.
 * HAL_BUSY  = transmisor DMA ocupado.
 * HAL_ERROR = parámetro inválido o comando demasiado largo.
 */
HAL_StatusTypeDef NEXTION_EnviarFloat(
    UART_HandleTypeDef *huart,
    const char *objeto,
    float valor,
    uint8_t decimales);


/*
 * Envía en una sola transferencia DMA los datos utilizados
 * inicialmente por la interfaz de prueba del profesor.
 *
 * Objetos utilizados en el proyecto HMI:
 *
 * tTemperatura = valor de la temperatura.
 * tBateria     = voltaje de batería.
 * tNumAngulo   = ángulo numérico de la cámara.
 * zAngulo      = indicador gráfico del ángulo.
 * tParada      = estado de la parada de emergencia.
 *
 * El ángulo de cámara tiene el siguiente rango:
 *
 * -90.0 a +90.0 grados.
 *
 * Para el indicador gráfico se realiza la conversión:
 *
 * -90 grados -> 0
 *   0 grados -> 90
 * +90 grados -> 180
 *
 * El estado de parada se representa de esta manera:
 *
 * parada_emergencia = 0:
 *
 * - Texto: NORMAL.
 * - Fondo: verde.
 *
 * parada_emergencia = 1:
 *
 * - Texto: EMERGENCIA.
 * - Fondo: rojo.
 *
 * Parámetros:
 *
 * huart               = manejador de UART5.
 * temperatura         = temperatura que se mostrará.
 * bateria             = voltaje de batería.
 * angulo_camara       = ángulo entre -90 y +90 grados.
 * parada_emergencia   = estado digital del botón de parada.
 *
 * Retorna:
 *
 * HAL_OK    = transferencia DMA iniciada correctamente.
 * HAL_BUSY  = UART5_TX todavía está transmitiendo.
 * HAL_ERROR = parámetro inválido o buffer insuficiente.
 */
HAL_StatusTypeDef NEXTION_EnviarTrama(
    UART_HandleTypeDef *huart,
    float temperatura,
    float bateria,
    float angulo_camara,
    uint8_t parada_emergencia);


#endif /* NEXTION_H_ */