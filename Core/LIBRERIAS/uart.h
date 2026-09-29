/*
 * uart.h
 *
 *  Created on: 15/11/2021
 *      Author: Alcides Ramos
 */

#ifndef LIBRERIAS_UART_H_
#define LIBRERIAS_UART_H_


#include "main.h"
#include "stdio.h"
#include "string.h"

/* Define explicitamente uint8_t, utilizado en uartx_write(). */
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif


/*
 * USO DE ESTA LIBRERIA EN EL PROYECTO
 *
 * Se conserva como referencia del profesor y para pruebas puntuales.
 * Sigue siendo bloqueante: no utilizarla en el ciclo periodico del
 * control de tierra, donde una espera retrasaria otras tareas.
 *
 * Las tramas del bote se construyen con trama_usv.c y se transportan
 * mediante uart_dma.c. Las instrucciones HMI utilizan nextion.c.
 * Estas funciones no construyen tramas ni verifican su CRC.
 *
 * Llamarlas desde el programa principal, con las interrupciones
 * habilitadas para que HAL_GetTick() pueda avanzar.
 * No llamarlas desde callbacks o interrupciones.
 */

/*
 * Envia un byte mediante UART bloqueante.
 *
 * huart = direccion del manejador HAL.
 * ch    = byte que se desea transmitir.
 *
 * El .c corregido utiliza una espera de 100 ms.
 * Si el manejador es NULL o el transmisor esta ocupado, no envia.
 * Por compatibilidad retorna void y no informa errores al llamador.
 */
void uartx_write(UART_HandleTypeDef *huart,uint8_t ch);

/*
 * Envia una cadena en una sola llamada a HAL_UART_Transmit().
 *
 * huart = direccion del manejador HAL.
 * info  = cadena valida terminada en '\0'.
 *
 * El terminador NUL no se transmite.
 * No envia cadenas vacias, punteros NULL ni textos de mas de 65535 bytes.
 * No agrega CRC, CR/LF ni los bytes finales de Nextion.
 * La espera HAL es de 100 ms para toda la transferencia.
 * Si expira, el receptor puede haber recibido solamente una parte.
 * Esta firma void no permite comprobar si el envio se completo.
 */
void uartx_write_text(UART_HandleTypeDef *huart, char *info);

/*
 * Recibe un byte mediante UART bloqueante.
 *
 * huart = direccion del manejador HAL.
 *
 * Retorna el byte recibido como char.
 * Retorna cero si el manejador es NULL, la recepcion esta ocupada
 * o HAL informa un error o tiempo agotado.
 *
 * El .c corregido limita la espera HAL a 100 ms.
 * No detiene ni consume la recepcion de una UART que tiene DMA RX activo.
 * Con esta firma no se puede distinguir un error de un byte NUL valido.
 * Para recepcion continua del proyecto utilizar UART_DMA_Leer()
 * o UART_DMA_LeerLinea(), segun el formato de los datos.
 */
char uartx_read(UART_HandleTypeDef *huart);

/*
 * Recibe texto hasta encontrar final, agotar espacio o terminar la espera.
 *
 * huart = direccion del manejador HAL.
 * info  = arreglo donde se guardara el texto.
 * final = caracter que termina la lectura; por ejemplo '\n'.
 * cuanto = capacidad TOTAL del arreglo, incluyendo el terminador '\0'.
 *
 * Guarda como maximo cuanto - 1 caracteres y termina la cadena con NUL.
 * Conserva el caracter final si se recibe y cabe en el arreglo.
 * El destino se limpia al comenzar, si info y cuanto son validos.
 * Con capacidad 1 solo puede entregar una cadena vacia.
 *
 * Los 100 ms corresponden a la lectura completa, no a cada caracter.
 * Si ocurre un error, un timeout o falta espacio, puede entregar
 * solamente un fragmento; void no permite saber el motivo de salida.
 * No leer con esta funcion mientras DMA RX esta activo.
 *
 * Se conserva char cuanto para coincidir con el .c y el modelo original.
 * Usar capacidades de 1 a 127 para portabilidad entre compiladores.
 * Nunca indicar una capacidad mayor que el tamano real de info.
 * Para tramas USV de hasta 256 bytes utilizar UART_DMA_LeerLinea().
 */
void uartx_read_text(UART_HandleTypeDef *huart,char  *info,const char final,char cuanto);



#ifdef __cplusplus
}
#endif


#endif /* LIBRERIAS_UART_H_ */

