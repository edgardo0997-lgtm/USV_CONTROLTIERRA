/*
 * uart.c
 *
 *  Created on: 15/11/2021
 *      Author: Alcides Ramos
 */

#include "uart.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

/*
 * Estas funciones se conservan para pruebas puntuales con UART bloqueante.
 * Para comandos USV y recepcion continua utilizar uart_dma.c.
 * Llamarlas desde el programa principal, con las interrupciones habilitadas;
 * no utilizarlas desde callbacks ni como parte del ciclo periodico del control.
 *
 * Se conservan las firmas originales del profesor. Las funciones void
 * no informan al llamador si hubo un error, y uartx_read() retorna cero
 * tanto ante un error como ante la recepcion de un byte NUL.
 */

/* Limite de espera en milisegundos; evita utilizar HAL_MAX_DELAY. */
#define UARTX_TIEMPO_ESPERA_MS 100U


/* Envia un byte solamente si el transmisor esta disponible. */
void uartx_write(UART_HandleTypeDef *huart,uint8_t ch)
{
    if ((huart == NULL) || (huart->gState != HAL_UART_STATE_READY))
    {
        return;
    }

    /* La llamada sigue siendo bloqueante, pero con espera limitada. */
    (void)HAL_UART_Transmit(huart, &ch, 1U, UARTX_TIEMPO_ESPERA_MS);
}


/*
 * Envia la cadena en una sola llamada, sin transmitir su NUL final.
 * No agrega CRC, CR/LF ni los terminadores de la pantalla Nextion.
 * info debe apuntar a una cadena valida terminada en '\0'.
 */
void uartx_write_text(UART_HandleTypeDef *huart,char *info)
{
    size_t longitud;

    if ((huart == NULL) || (info == NULL) ||
        (huart->gState != HAL_UART_STATE_READY))
    {
        return;
    }

    longitud = strlen(info);

    /* HAL recibe una longitud de 16 bits. No se permite truncarla. */
    if ((longitud == 0U) || (longitud > UINT16_MAX))
    {
        return;
    }

    (void)HAL_UART_Transmit(
        huart, (uint8_t *)info, (uint16_t)longitud, UARTX_TIEMPO_ESPERA_MS);
}


/* Lee un byte con espera limitada, sin detener una recepcion DMA activa. */
char uartx_read(UART_HandleTypeDef *huart)
{
    uint8_t dato_rx_=0;

    if ((huart == NULL) || (huart->RxState != HAL_UART_STATE_READY))
    {
        return 0;
    }

    if (HAL_UART_Receive(huart, &dato_rx_, 1U,
                        UARTX_TIEMPO_ESPERA_MS) != HAL_OK)
    {
        return 0;
    }

    return (char)dato_rx_;
}


/*
 * cuanto es el tamano TOTAL del destino, incluyendo espacio para '\0'.
 * Se guardan como maximo cuanto - 1 caracteres.
 * El caracter final tambien se conserva si se recibe y queda espacio.
 * Si ocurre un error o expira la espera, el texto parcial queda terminado.
 *
 * Se mantiene el parametro char original por compatibilidad con uart.h.
 * Para que su capacidad sea portable entre compiladores, usar 1 a 127.
 * No pasar un valor mayor que el tamano real del arreglo info.
 */
void uartx_read_text(UART_HandleTypeDef *huart,char *info,const char final,char cuanto)
{
    char dato_rx=0;//  los datos leidos son de 8 bits
    uint16_t i=0;
    uint16_t capacidad;
    uint32_t tiempo_inicio;
    uint32_t tiempo_transcurrido;

    if ((info == NULL) || (cuanto <= 0))
    {
        return;
    }

    capacidad = (uint16_t)cuanto;
    memset(info,0,capacidad);//limpia el buffer

    /* Un destino de un byte solo puede contener el terminador NUL. */
    if ((huart == NULL) || (capacidad < 2U) ||
        (huart->RxState != HAL_UART_STATE_READY))
    {
        return;
    }

    /* El limite de tiempo se aplica a la lectura completa, no a cada byte. */
    tiempo_inicio = HAL_GetTick();

    while (1)// lee hasta que llegue el final
    {
        tiempo_transcurrido = (uint32_t)(HAL_GetTick() - tiempo_inicio);
        if (tiempo_transcurrido >= UARTX_TIEMPO_ESPERA_MS)
        {
            break;
        }

        /* Comprueba el estado antes de leer; no consume bytes del DMA RX. */
        if (huart->RxState != HAL_UART_STATE_READY)
        {
            break;
        }

        if (HAL_UART_Receive(
                huart, (uint8_t *)&dato_rx, 1U,
                UARTX_TIEMPO_ESPERA_MS - tiempo_transcurrido) != HAL_OK)
        {
            break;
        }

        *info=dato_rx;//  va almacenado en el buffer
        info++;
        i++;// incrementa contador

        /* Se conserva siempre un byte libre para terminar la cadena. */
        *info = '\0';

        if ((dato_rx == final) || (i >= (capacidad - 1U)))
        {
            break;// si llegan n elementosa sale
        }
    }
}

