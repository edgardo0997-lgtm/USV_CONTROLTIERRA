/*
 * nextion.h
 *
 * Comunicacion entre el STM32H562RGT6 y la pantalla Nextion
 * mediante USART1 y DMA, a 115200 baudios, 8N1.
 */

#ifndef NEXTION_H_
#define NEXTION_H_

#include "main.h"
#include "trama_usv.h"

#include <stdint.h>

/* Buffer permanente usado por el DMA para agrupar una actualizacion completa. */
#define NEXTION_BUFFER_TX_TAMANO       1536U

/* Buffer temporal maximo para un comando individual. */
#define NEXTION_COMANDO_MAXIMO          128U

HAL_StatusTypeDef NEXTION_Init(
    UART_HandleTypeDef *huart);

uint8_t NEXTION_TransmisionLista(
    UART_HandleTypeDef *huart);

HAL_StatusTypeDef NEXTION_EnviarComando(
    UART_HandleTypeDef *huart,
    const char *comando);

HAL_StatusTypeDef NEXTION_EnviarTexto(
    UART_HandleTypeDef *huart,
    const char *objeto,
    const char *texto);

HAL_StatusTypeDef NEXTION_EnviarEntero(
    UART_HandleTypeDef *huart,
    const char *objeto,
    int32_t valor);

HAL_StatusTypeDef NEXTION_EnviarFloat(
    UART_HandleTypeDef *huart,
    const char *objeto,
    float valor,
    uint8_t decimales);

/*
 * Funcion conservada para compatibilidad con la interfaz de prueba anterior.
 */
HAL_StatusTypeDef NEXTION_EnviarTrama(
    UART_HandleTypeDef *huart,
    float temperatura,
    float bateria,
    float angulo_camara,
    uint8_t parada_emergencia);

/*
 * Actualiza en una sola transferencia DMA los objetos reales de page0 y
 * pageIMU. La telemetria procede de una trama $PUSVD ya validada.
 *
 * Si telemetria_valida vale cero, los datos remotos se muestran como no
 * disponibles. Los controles locales (motores, camara, bomba y parada)
 * continuan actualizandose.
 */
HAL_StatusTypeDef NEXTION_EnviarEstadoUSV(
    UART_HandleTypeDef *huart,
    const USV_Telemetria *telemetria,
    uint8_t telemetria_valida,
    const USV_Comando *comando,
    float voltaje_control);

#endif /* NEXTION_H_ */
