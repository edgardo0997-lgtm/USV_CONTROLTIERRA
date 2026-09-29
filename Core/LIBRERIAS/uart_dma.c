/*
 * uart_dma.c
 *
 * Librería para la comunicación UART mediante DMA.
 *
 * Esta librería administra:
 *
 * - USART1 para la comunicación con el XBee.
 * - UART5 para la comunicación con la pantalla Nextion.
 * - Recepción continua mediante DMA circular.
 * - Detección de pausas mediante el evento IDLE.
 * - Almacenamiento de los bytes recibidos en un búfer circular.
 * - Transmisión mediante DMA sin bloquear el programa principal.
 * - Recuperación básica después de un error de comunicación.
 *
 * La librería fue diseñada para trabajar con la configuración
 * realizada previamente en STM32CubeMX.
 */


/* Archivo de cabecera correspondiente a esta librería. */
#include "uart_dma.h"

/* Contiene las variables huart1 y huart5 generadas por CubeMX. */
#include "usart.h"

/* Se utiliza para memcpy() y strlen(). */
#include <string.h>

/* Los indices y los tamanos entregados a HAL son de 16 bits. */
#if (UART_DMA_RX_BUFFER_SIZE < 2U) || (UART_DMA_RX_BUFFER_SIZE > UINT16_MAX)
#error "UART_DMA_RX_BUFFER_SIZE debe estar entre 2 y 65535 bytes."
#endif
#if (UART_DMA_FIFO_SIZE < 2U) || (UART_DMA_FIFO_SIZE > UINT16_MAX)
#error "UART_DMA_FIFO_SIZE debe estar entre 2 y 65535 bytes."
#endif
#if (UART_DMA_TX_BUFFER_SIZE < 1U) || (UART_DMA_TX_BUFFER_SIZE > UINT16_MAX)
#error "UART_DMA_TX_BUFFER_SIZE debe estar entre 1 y 65535 bytes."
#endif
#if (UART_DMA_LINE_BUFFER_SIZE < 2U) || (UART_DMA_LINE_BUFFER_SIZE > UINT16_MAX)
#error "UART_DMA_LINE_BUFFER_SIZE debe estar entre 2 y 65535 bytes."
#endif

/*
 * Las funciones publicas se utilizan desde el programa principal.
 * No llamar a iniciar, leer, procesar o transmitir desde una interrupcion.
 * Mantener las IRQ de cada UART y de su DMA RX con la misma prioridad
 * de preempcion, para que los callbacks de recepcion no se interrumpan
 * entre si. Mantener activos los eventos de media transferencia y fin.
 * No bloquear el programa tanto tiempo que DMA complete una vuelta
 * sin atender sus interrupciones: este buffer no puede recuperar
 * bytes que el hardware ya sobrescribio.
 */


/* ================================================================
 * ESTRUCTURA INTERNA DE CONTROL
 * ================================================================
 *
 * Cada UART necesita sus propios búferes, índices y banderas.
 * Esta estructura permite utilizar el mismo código para USART1
 * y UART5 sin duplicar todas las funciones.
 */
typedef struct
{
    /* Dirección del manejador HAL de la UART.
     *
     * Para el proyecto:
     * - &huart1 corresponde al XBee.
     * - &huart5 corresponde a la pantalla Nextion.
     */
    UART_HandleTypeDef *huart;


    /* Búfer donde el DMA deposita directamente los bytes.
     *
     * El DMA trabaja en modo circular. Cuando llega al final
     * del arreglo, vuelve automáticamente a la posición cero.
     */
    uint8_t rx_dma[UART_DMA_RX_BUFFER_SIZE];


    /* Última posición del búfer DMA que ya fue procesada.
     *
     * Permite identificar solamente los bytes nuevos cuando
     * ocurre un evento IDLE, media transferencia o transferencia
     * completa.
     */
    volatile uint16_t ultima_posicion_dma;


    /* Búfer circular utilizado por el programa principal.
     *
     * Los datos nuevos se copian desde rx_dma hacia este búfer.
     * De esta manera, el DMA puede seguir recibiendo mientras
     * el programa principal procesa una trama anterior.
     */
    uint8_t fifo_rx[UART_DMA_FIFO_SIZE];


    /* Posición donde se escribirá el siguiente byte recibido. */
    volatile uint16_t cabeza_fifo;


    /* Posición desde donde se leerá el siguiente byte. */
    volatile uint16_t cola_fifo;


    /* Búfer temporal utilizado para formar una línea completa.
     *
     * Las tramas del XBee terminan en "\r\n". Los bytes se
     * acumulan aquí hasta encontrar el carácter '\n'.
     */
    char linea[UART_DMA_LINE_BUFFER_SIZE];


    /* Cantidad de caracteres almacenados actualmente
     * en el búfer linea.
     */
    uint16_t longitud_linea;


    /* Indica que una línea fue demasiado larga.
     *
     * Cuando vale 1, se descartan los caracteres hasta encontrar
     * '\n'. Esto permite recuperar la sincronización sin aceptar
     * una trama incompleta.
     */
    uint8_t descartando_linea;


    /* Búfer interno para transmisión mediante DMA.
     *
     * HAL_UART_Transmit_DMA() no copia los datos recibidos.
     * Por eso la librería guarda primero la información aquí,
     * evitando que el búfer original sea modificado mientras
     * el DMA todavía está transmitiendo.
     */
    uint8_t tx_dma[UART_DMA_TX_BUFFER_SIZE];


    /* Indica que existe una transmisión DMA en proceso.
     *
     * 0 = UART disponible para transmitir.
     * 1 = Transmisión actualmente activa.
     */
    volatile uint8_t tx_ocupado;


    /* Solicita reiniciar la recepción después de un error.
     *
     * La interrupción solamente activa esta bandera.
     * El reinicio real se realiza posteriormente desde
     * UART_DMA_Procesar(), fuera de la interrupción.
     */
    volatile uint8_t reiniciar_recepcion;


    /* Indica que se llenó completamente el búfer circular.
     *
     * Cuando esto sucede, los bytes nuevos se descartan.
     * El CRC de la trama permitirá detectar que el mensaje
     * recibido está incompleto.
     */
    volatile uint8_t desbordamiento;

    /*
     * Solicita descartar los datos despues de una perdida de bytes.
     * Es independiente de la bandera publica de desbordamiento:
     * consultar esa bandera no debe ocultar una trama incompleta.
     */
    volatile uint8_t perdida_recepcion;


    /* Contador acumulado de errores detectados por la UART. */
    volatile uint32_t cantidad_errores;

} UART_DMA_Contexto;


/* ================================================================
 * CONTEXTOS DE LAS UART DEL PROYECTO
 * ================================================================
 */

/* Contexto correspondiente a USART1.
 *
 * USART1 se utilizará para:
 * - Enviar la trama de comandos al XBee.
 * - Recibir la telemetría proveniente del bote.
 */
static UART_DMA_Contexto contexto_usart1;


/* Contexto correspondiente a UART5.
 *
 * UART5 se utilizará para:
 * - Enviar datos a la pantalla Nextion.
 * - Recibir eventos táctiles o respuestas de la pantalla.
 */
static UART_DMA_Contexto contexto_uart5;

/* Impide borrar los buffers si la inicializacion se llama otra vez. */
static uint8_t inicializacion_realizada;


/* ================================================================
 * FUNCIONES INTERNAS
 * ================================================================
 */


/* Busca el contexto correspondiente a una UART.
 *
 * Parámetro:
 * huart = Dirección del manejador HAL de la UART.
 *
 * Retorna:
 * Dirección del contexto correspondiente.
 *
 * Retorna NULL si la UART no pertenece a USART1 ni UART5.
 */
static UART_DMA_Contexto *UART_DMA_ObtenerContexto(
        UART_HandleTypeDef *huart)
{
    /* Verificación de seguridad para evitar utilizar
     * un puntero inválido.
     */
    if (huart == NULL)
    {
        return NULL;
    }

    /* Compara la instancia del periférico recibido
     * con la instancia de USART1.
     */
    if ((huart == &huart1) &&
        (huart->Instance == USART1) &&
        (contexto_usart1.huart == huart))
    {
        return &contexto_usart1;
    }

    /* Compara la instancia recibida con UART5. */
    if ((huart == &huart5) &&
        (huart->Instance == UART5) &&
        (contexto_uart5.huart == huart))
    {
        return &contexto_uart5;
    }

    /* La UART recibida no está administrada por esta librería. */
    return NULL;
}


/* Reinicia todas las variables internas de un contexto.
 *
 * Esta función se utiliza antes de comenzar la recepción DMA.
 */
static void UART_DMA_LimpiarContexto(
        UART_DMA_Contexto *contexto,
        UART_HandleTypeDef *huart)
{
    /* No se realiza ninguna operación si el contexto
     * recibido es inválido.
     */
    if (contexto == NULL)
    {
        return;
    }

    /* Coloca en cero los búferes, índices y banderas. */
    memset(contexto, 0, sizeof(UART_DMA_Contexto));

    /* Asocia nuevamente el contexto con su UART. */
    contexto->huart = huart;
}


/*
 * Descarta una recepcion cuya continuidad ya no se puede asegurar.
 * La seccion protegida es corta y conserva el estado previo de IRQ.
 * Para lectura por lineas, se ignora el resto hasta el siguiente LF.
 * Para Nextion binario, el lector debe reiniciar su decodificador
 * cuando UART_DMA_HuboDesbordamiento() informe una perdida.
 */
static void UART_DMA_DescartarRecepcion(UART_DMA_Contexto *contexto)
{
    uint32_t estado_irq = __get_PRIMASK();

    __disable_irq();
    contexto->cola_fifo = contexto->cabeza_fifo;
    contexto->longitud_linea = 0U;
    contexto->descartando_linea = 1U;
    contexto->perdida_recepcion = 0U;
    __DMB();
    __set_PRIMASK(estado_irq);
}


/* Guarda un byte dentro del búfer circular de recepción.
 *
 * Esta función se ejecuta desde la interrupción UART.
 */
static void UART_DMA_GuardarByte(
        UART_DMA_Contexto *contexto,
        uint8_t dato)
{
    uint16_t siguiente_posicion;

    /* Verificación del puntero recibido. */
    if (contexto == NULL)
    {
        return;
    }

    /* No mezcla nuevos bytes con un bloque que ya perdio informacion. */
    if (contexto->perdida_recepcion != 0U)
    {
        return;
    }

    /* Calcula la siguiente posición del búfer circular.
     *
     * El operador módulo hace que después de la última
     * posición se regrese automáticamente a cero.
     */
    siguiente_posicion =
        (uint16_t)((contexto->cabeza_fifo + 1U) %
                   UART_DMA_FIFO_SIZE);

    /* Si la siguiente posición coincide con la cola,
     * significa que el búfer está completamente lleno.
     */
    if (siguiente_posicion == contexto->cola_fifo)
    {
        /* Se informa el desbordamiento y se descarta
         * el byte nuevo.
         */
        contexto->desbordamiento = 1U;
        contexto->perdida_recepcion = 1U;
        return;
    }

    /* Guarda el byte en la posición actual. */
    contexto->fifo_rx[contexto->cabeza_fifo] = dato;

    /* Actualiza la cabeza del búfer circular. */
    /* Publica el indice solamente despues de guardar el byte. */
    __DMB();
    contexto->cabeza_fifo = siguiente_posicion;
}


/* Copia varios bytes desde el búfer DMA hacia el FIFO.
 *
 * Esta función permite mover únicamente la parte nueva
 * recibida por el DMA.
 */
static void UART_DMA_GuardarBloque(
        UART_DMA_Contexto *contexto,
        const uint8_t *datos,
        uint16_t longitud)
{
    uint16_t indice;

    /* Verifica los punteros antes de acceder a memoria. */
    if ((contexto == NULL) || (datos == NULL))
    {
        return;
    }

    /* Copia los bytes individualmente hacia el FIFO. */
    for (indice = 0U; indice < longitud; indice++)
    {
        UART_DMA_GuardarByte(contexto, datos[indice]);
    }
}


/* Procesa una nueva posición reportada por el DMA.
 *
 * La posición puede encontrarse:
 *
 * - Después de la posición anterior.
 * - Al comienzo del búfer después de un retorno circular.
 *
 * La función procesa correctamente ambos casos.
 */
static void UART_DMA_ProcesarPosicion(
        UART_DMA_Contexto *contexto,
        uint16_t posicion_actual)
{
    uint16_t posicion_anterior;

    /* Verifica que el contexto sea válido. */
    if (contexto == NULL)
    {
        return;
    }

    /* Evita procesar una posición fuera del tamaño
     * real del búfer DMA.
     */
    if (posicion_actual > UART_DMA_RX_BUFFER_SIZE)
    {
        return;
    }

    posicion_anterior = contexto->ultima_posicion_dma;

    /* No llegaron datos nuevos desde el evento anterior. */
    if (posicion_actual == posicion_anterior)
    {
        return;
    }

    /* Caso 1:
     * La nueva posición está después de la anterior.
     *
     * Los datos nuevos forman un bloque continuo.
     */
    if (posicion_actual > posicion_anterior)
    {
        UART_DMA_GuardarBloque(
            contexto,
            &contexto->rx_dma[posicion_anterior],
            (uint16_t)(posicion_actual - posicion_anterior));
    }
    else
    {
        /* Caso 2:
         * El DMA llegó al final del arreglo y volvió al inicio.
         *
         * Primero se copia desde la posición anterior hasta
         * el final del arreglo.
         */
        UART_DMA_GuardarBloque(
            contexto,
            &contexto->rx_dma[posicion_anterior],
            (uint16_t)(UART_DMA_RX_BUFFER_SIZE -
                       posicion_anterior));

        /* Después se copia desde el inicio hasta la posición
         * actual reportada por el DMA.
         */
        if (posicion_actual > 0U)
        {
            UART_DMA_GuardarBloque(
                contexto,
                &contexto->rx_dma[0],
                posicion_actual);
        }
    }

    /* Cuando HAL reporta exactamente el tamaño completo del
     * búfer, la posición lógica siguiente debe ser cero.
     */
    /*
     * Se conserva aqui el valor completo que reporto HAL, incluido
     * UART_DMA_RX_BUFFER_SIZE. El comentario anterior describe la
     * normalizacion anterior a cero: ahora no se aplica, para evitar
     * duplicar el bloque si TC e IDLE reportan consecutivamente el fin.
     * La siguiente posicion menor detectara el retorno circular.
     */
    contexto->ultima_posicion_dma = posicion_actual;
}


/* Inicia la recepción mediante DMA con detección de IDLE.
 *
 * La función HAL_UARTEx_ReceiveToIdle_DMA() permite generar
 * un evento cuando:
 *
 * - Se llena la mitad del búfer.
 * - Se llena completamente el búfer.
 * - Se detecta una pausa en la recepción UART.
 *
 * Como el DMA fue configurado en modo circular desde CubeMX,
 * la recepción puede continuar indefinidamente.
 */
static HAL_StatusTypeDef UART_DMA_IniciarRecepcion(
        UART_DMA_Contexto *contexto)
{
    HAL_StatusTypeDef estado;

    /* Verifica que existan el contexto, la UART y el canal DMA. */
    if ((contexto == NULL) ||
        (contexto->huart == NULL) ||
        (contexto->huart->hdmarx == NULL))
    {
        return HAL_ERROR;
    }

    /* No modifica la posicion de una recepcion que ya esta activa. */
    if (contexto->huart->RxState != HAL_UART_STATE_READY)
    {
        return HAL_BUSY;
    }

    /* La nueva recepción comenzará desde el inicio
     * del búfer DMA.
     */
    contexto->ultima_posicion_dma = 0U;

    /* Un error que ocurra al arrancar debe poder activar la solicitud. */
    contexto->reiniciar_recepcion = 0U;

    /* Inicia la recepción continua con detección de IDLE. */
    estado = HAL_UARTEx_ReceiveToIdle_DMA(
        contexto->huart,
        contexto->rx_dma,
        UART_DMA_RX_BUFFER_SIZE);

    /* Si la recepción comenzó correctamente, se elimina
     * cualquier solicitud anterior de reinicio.
     */
    /* Se limpia antes de arrancar; no se borra un error nuevo del callback. */
    if (estado != HAL_OK)
    {
        contexto->reiniciar_recepcion = 1U;
    }

    return estado;
}


/* ================================================================
 * FUNCIONES PÚBLICAS
 * ================================================================
 */


/* Inicializa la recepción DMA de USART1 y UART5.
 *
 * Esta función debe llamarse una sola vez después de:
 *
 * - MX_GPDMA1_Init()
 * - MX_UART5_Init()
 * - MX_USART1_UART_Init()
 */
HAL_StatusTypeDef UART_DMA_Iniciar(void)
{
    HAL_StatusTypeDef estado_usart1;
    HAL_StatusTypeDef estado_uart5;

    /* No reinicializa memoria mientras DMA todavia puede utilizarla. */
    if (inicializacion_realizada != 0U)
    {
        return HAL_BUSY;
    }

    /* CubeMX debe enlazar ambos canales de recepcion antes de empezar. */
    if ((huart1.Instance != USART1) ||
        (huart5.Instance != UART5) ||
        (huart1.hdmarx == NULL) ||
        (huart5.hdmarx == NULL))
    {
        return HAL_ERROR;
    }

    if ((huart1.RxState != HAL_UART_STATE_READY) ||
        (huart5.RxState != HAL_UART_STATE_READY))
    {
        return HAL_BUSY;
    }

    /* Prepara el contexto utilizado por el XBee. */
    UART_DMA_LimpiarContexto(
        &contexto_usart1,
        &huart1);

    /* Prepara el contexto utilizado por Nextion. */
    UART_DMA_LimpiarContexto(
        &contexto_uart5,
        &huart5);

    /* Aunque una UART falle, Procesar podra reintentar su recepcion. */
    inicializacion_realizada = 1U;

    /* Inicia la recepción circular de USART1. */
    estado_usart1 =
        UART_DMA_IniciarRecepcion(&contexto_usart1);

    /* Inicia la recepción circular de UART5. */
    estado_uart5 =
        UART_DMA_IniciarRecepcion(&contexto_uart5);

    /* Se informa primero cualquier problema encontrado
     * en la comunicación principal con el XBee.
     */
    if (estado_usart1 != HAL_OK)
    {
        return estado_usart1;
    }

    /* Retorna el resultado obtenido al iniciar UART5. */
    return estado_uart5;
}


/* Atiende solicitudes de recuperación de las UART.
 *
 * Esta función debe ejecutarse periódicamente dentro del
 * while(1) del programa principal.
 *
 * No debe ejecutarse dentro de una interrupción.
 */
void UART_DMA_Procesar(void)
{
    UART_DMA_Contexto *contextos[2];
    uint8_t indice;

    /* Lista de las dos UART administradas. */
    contextos[0] = &contexto_usart1;
    contextos[1] = &contexto_uart5;

    /* Revisa individualmente cada UART. */
    for (indice = 0U; indice < 2U; indice++)
    {
        UART_DMA_Contexto *contexto = contextos[indice];

        /* Permite llamar a Procesar sin acceder a una UART no inicializada. */
        if (contexto->huart == NULL)
        {
            continue;
        }

        /*
         * Un error exclusivamente RX no libera un envio activo.
         * Si HAL ya devolvio el transmisor a READY, se puede liberar
         * una reserva que quedo pendiente despues de un error TX.
         */
        if ((contexto->tx_ocupado != 0U) &&
            (contexto->huart->gState == HAL_UART_STATE_READY))
        {
            contexto->tx_ocupado = 0U;
        }

        if (contexto->perdida_recepcion != 0U)
        {
            UART_DMA_DescartarRecepcion(contexto);
        }

        /* Solo se realiza el procedimiento si una interrupción
         * informó previamente un error.
         */
        if (contexto->reiniciar_recepcion != 0U)
        {
            /* Detiene únicamente la recepción de la UART.
             * La transmisión es independiente.
             */
            if (HAL_UART_AbortReceive(contexto->huart) == HAL_OK)
            {
                /* No une bytes anteriores al error con la nueva recepcion. */
                UART_DMA_DescartarRecepcion(contexto);

                /* Intenta comenzar nuevamente la recepción. */
                if (UART_DMA_IniciarRecepcion(contexto) != HAL_OK)
                {
                    /* Si todavía no fue posible, se conserva
                     * la solicitud para intentarlo nuevamente.
                     */
                    contexto->reiniciar_recepcion = 1U;
                }
            }
        }
    }
}


/* Retorna la cantidad de bytes disponibles en el FIFO
 * de una UART.
 */
size_t UART_DMA_DatosDisponibles(
        UART_HandleTypeDef *huart)
{
    UART_DMA_Contexto *contexto;
    uint16_t cabeza;
    uint16_t cola;

    contexto = UART_DMA_ObtenerContexto(huart);

    if (contexto == NULL)
    {
        return 0U;
    }

    /* No anuncia como validos los bytes que preceden a una perdida. */
    if ((contexto->perdida_recepcion != 0U) ||
        (contexto->reiniciar_recepcion != 0U))
    {
        return 0U;
    }

    /* Se toman copias locales de los índices porque la cabeza
     * puede actualizarse desde una interrupción.
     */
    cabeza = contexto->cabeza_fifo;
    cola = contexto->cola_fifo;

    if (cabeza >= cola)
    {
        return (size_t)(cabeza - cola);
    }

    return (size_t)(UART_DMA_FIFO_SIZE - cola + cabeza);
}


/* Lee bytes disponibles desde el FIFO.
 *
 * Parámetros:
 * huart     = UART desde donde se desea leer.
 * destino   = Arreglo donde se copiarán los bytes.
 * capacidad = Cantidad máxima de bytes que pueden copiarse.
 *
 * Retorna:
 * Cantidad real de bytes copiados.
 */
size_t UART_DMA_Leer(
        UART_HandleTypeDef *huart,
        uint8_t *destino,
        size_t capacidad)
{
    UART_DMA_Contexto *contexto;
    size_t cantidad = 0U;

    contexto = UART_DMA_ObtenerContexto(huart);

    if ((contexto == NULL) ||
        (destino == NULL) ||
        (capacidad == 0U))
    {
        return 0U;
    }

    /* Continúa mientras exista espacio en el destino
     * y todavía queden bytes disponibles.
     */
    while ((cantidad < capacidad) &&
           (contexto->cola_fifo != contexto->cabeza_fifo))
    {
        uint32_t estado_irq = __get_PRIMASK();

        /* Evita que un error de IRQ se mezcle con la lectura de un byte. */
        __disable_irq();
        if ((contexto->perdida_recepcion != 0U) ||
            (contexto->reiniciar_recepcion != 0U))
        {
            UART_DMA_DescartarRecepcion(contexto);
            __set_PRIMASK(estado_irq);
            return 0U;
        }

        /* Copia el siguiente byte disponible. */
        destino[cantidad] =
            contexto->fifo_rx[contexto->cola_fifo];

        cantidad++;

        /* Avanza la cola del búfer circular. */
        __DMB();
        contexto->cola_fifo =
            (uint16_t)((contexto->cola_fifo + 1U) %
                       UART_DMA_FIFO_SIZE);
        __set_PRIMASK(estado_irq);
    }

    return cantidad;
}


/* Busca y entrega una línea completa terminada en '\n'.
 *
 * Esta función es adecuada para las tramas de texto:
 *
 * $CMD,...*CRC\r\n
 * $TEL,...*CRC\r\n
 *
 * Retorna:
 * - Longitud de la línea cuando se encuentra una trama completa.
 * - Cero cuando todavía no se ha recibido '\n'.
 */
/*
 * Los identificadores actuales del proyecto son $PUSVU y $PUSVD.
 * Los ejemplos anteriores se conservan como comentarios originales.
 * IDLE no delimita una trama: se espera realmente el caracter LF.
 * No alternar Leer y LeerLinea sobre la misma UART mientras se
 * construye una linea, porque ambos consumen el mismo FIFO.
 */
size_t UART_DMA_LeerLinea(
        UART_HandleTypeDef *huart,
        char *destino,
        size_t capacidad)
{
    UART_DMA_Contexto *contexto;
    uint8_t dato;

    contexto = UART_DMA_ObtenerContexto(huart);

    if ((contexto == NULL) ||
        (destino == NULL) ||
        (capacidad < 2U))
    {
        return 0U;
    }

    /* Si no se entrega una linea, no queda una trama anterior en destino. */
    destino[0] = '\0';

    /* Procesa los bytes disponibles de uno en uno. */
    while (UART_DMA_Leer(huart, &dato, 1U) == 1U)
    {
        /* Si una línea anterior fue demasiado larga,
         * se descartan bytes hasta encontrar su final.
         */
        if (contexto->descartando_linea != 0U)
        {
            if (dato == (uint8_t)'\n')
            {
                contexto->descartando_linea = 0U;
                contexto->longitud_linea = 0U;
            }

            continue;
        }

        /* Comprueba que quede espacio para el carácter nuevo
         * y para el carácter nulo final.
         */
        if (contexto->longitud_linea >=
            (UART_DMA_LINE_BUFFER_SIZE - 1U))
        {
            contexto->longitud_linea = 0U;
            /* Si este byte ya es LF, la linea siguiente no se descarta. */
            contexto->descartando_linea =
                (dato == (uint8_t)'\n') ? 0U : 1U;
            contexto->desbordamiento = 1U;
            continue;
        }

        /* Una trama de texto no admite un NUL dentro de sus valores. */
        if (dato == 0U)
        {
            contexto->longitud_linea = 0U;
            contexto->descartando_linea = 1U;
            contexto->desbordamiento = 1U;
            continue;
        }

        /* Almacena el carácter recibido. */
        contexto->linea[contexto->longitud_linea] = (char)dato;
        contexto->longitud_linea++;

        /* Una línea queda completa cuando se recibe '\n'. */
        if (dato == (uint8_t)'\n')
        {
            size_t longitud =
                (size_t)contexto->longitud_linea;
            uint32_t estado_irq;

            /* Verifica que el búfer entregado por el usuario
             * sea suficientemente grande.
             */
            if (longitud >= capacidad)
            {
                contexto->longitud_linea = 0U;
                contexto->desbordamiento = 1U;
                return 0U;
            }

            /* No entrega una linea si se perdieron bytes antes de copiarla. */
            estado_irq = __get_PRIMASK();
            __disable_irq();
            if ((contexto->perdida_recepcion != 0U) ||
                (contexto->reiniciar_recepcion != 0U))
            {
                UART_DMA_DescartarRecepcion(contexto);
                __set_PRIMASK(estado_irq);
                return 0U;
            }

            /* Copia la línea completa al destino. */
            memcpy(destino, contexto->linea, longitud);

            /* Agrega el carácter nulo para convertirla
             * en una cadena válida de C.
             */
            destino[longitud] = '\0';

            /* Prepara el acumulador para la próxima trama. */
            contexto->longitud_linea = 0U;
            __set_PRIMASK(estado_irq);

            return longitud;
        }
    }

    /* Todavía no existe una línea completa. */
    return 0U;
}


/* Transmite un bloque de datos utilizando DMA.
 *
 * La función copia primero los datos al búfer interno.
 * De esta manera, el búfer original puede modificarse
 * después de que la función retorne.
 */
HAL_StatusTypeDef UART_DMA_Transmitir(
        UART_HandleTypeDef *huart,
        const uint8_t *datos,
        uint16_t longitud)
{
    UART_DMA_Contexto *contexto;
    HAL_StatusTypeDef estado;

    contexto = UART_DMA_ObtenerContexto(huart);

    /* Verifica los parámetros recibidos. */
    if ((contexto == NULL) ||
        (datos == NULL) ||
        (longitud == 0U) ||
        (longitud > UART_DMA_TX_BUFFER_SIZE))
    {
        return HAL_ERROR;
    }

    /* Un canal TX debe haber sido enlazado por CubeMX. */
    if (contexto->huart->hdmatx == NULL)
    {
        return HAL_ERROR;
    }

    /* No se inicia otra transmisión si el DMA todavía
     * está utilizando el búfer interno.
     */
    /* Incluye envios iniciados directamente por la libreria Nextion. */
    if ((contexto->tx_ocupado != 0U) ||
        (contexto->huart->gState != HAL_UART_STATE_READY))
    {
        return HAL_BUSY;
    }

    /* Copia los datos al búfer protegido de transmisión. */
    memcpy(contexto->tx_dma, datos, longitud);

    /* Reserva la UART antes de iniciar el DMA. */
    contexto->tx_ocupado = 1U;

    /* Inicia la transmisión sin bloquear el programa. */
    estado = HAL_UART_Transmit_DMA(
        contexto->huart,
        contexto->tx_dma,
        longitud);

    /* Si HAL no pudo iniciar la transmisión,
     * se libera nuevamente el búfer.
     */
    if (estado != HAL_OK)
    {
        contexto->tx_ocupado = 0U;
    }

    return estado;
}


/* Transmite una cadena de texto mediante DMA.
 *
 * Esta función es útil para:
 * - Tramas del XBee.
 * - Comandos de texto.
 * - Mensajes de diagnóstico.
 */
HAL_StatusTypeDef UART_DMA_TransmitirTexto(
        UART_HandleTypeDef *huart,
        const char *texto)
{
    size_t longitud;

    if (texto == NULL)
    {
        return HAL_ERROR;
    }

    /* Busca el final solo dentro del maximo que esta UART puede enviar. */
    for (longitud = 0U; longitud <= UART_DMA_TX_BUFFER_SIZE; longitud++)
    {
        if (texto[longitud] == '\0')
        {
            break;
        }
    }

    /* La cadena debe caber completamente en el búfer DMA. */
    if ((longitud == 0U) ||
        (longitud > UART_DMA_TX_BUFFER_SIZE))
    {
        return HAL_ERROR;
    }

    return UART_DMA_Transmitir(
        huart,
        (const uint8_t *)texto,
        (uint16_t)longitud);
}


/* Informa si una UART continúa transmitiendo.
 *
 * Retorna:
 * 0 = UART disponible.
 * 1 = Transmisión DMA en proceso.
 */
uint8_t UART_DMA_TransmisionOcupada(
        UART_HandleTypeDef *huart)
{
    UART_DMA_Contexto *contexto;

    contexto = UART_DMA_ObtenerContexto(huart);

    if (contexto == NULL)
    {
        return 0U;
    }

    /* Tambien detecta una transmision iniciada fuera de esta libreria. */
    return ((contexto->tx_ocupado != 0U) ||
            (contexto->huart->gState != HAL_UART_STATE_READY)) ? 1U : 0U;
}


/* Consulta si ocurrió un desbordamiento.
 *
 * Después de consultar la bandera, la función la limpia.
 * De esta manera se puede detectar nuevamente un evento futuro.
 */
uint8_t UART_DMA_HuboDesbordamiento(
        UART_HandleTypeDef *huart)
{
    UART_DMA_Contexto *contexto;
    uint8_t resultado;
    uint32_t estado_irq;

    contexto = UART_DMA_ObtenerContexto(huart);

    if (contexto == NULL)
    {
        return 0U;
    }

    /* La lectura y limpieza no deben borrar un evento nuevo de IRQ. */
    estado_irq = __get_PRIMASK();
    __disable_irq();
    resultado = contexto->desbordamiento;
    contexto->desbordamiento = 0U;
    __set_PRIMASK(estado_irq);

    return resultado;
}


/* Retorna la cantidad acumulada de errores registrados
 * por una UART desde el inicio del programa.
 */
uint32_t UART_DMA_CantidadErrores(
        UART_HandleTypeDef *huart)
{
    UART_DMA_Contexto *contexto;

    contexto = UART_DMA_ObtenerContexto(huart);

    if (contexto == NULL)
    {
        return 0U;
    }

    return contexto->cantidad_errores;
}


/* ================================================================
 * CALLBACKS DE LA LIBRERÍA HAL
 * ================================================================
 */


/* Esta función es llamada automáticamente por HAL cuando ocurre
 * un evento de recepción:
 *
 * - Línea IDLE.
 * - Media transferencia DMA.
 * - Transferencia DMA completa.
 */
void HAL_UARTEx_RxEventCallback(
        UART_HandleTypeDef *huart,
        uint16_t Size)
{
    UART_DMA_Contexto *contexto;

    contexto = UART_DMA_ObtenerContexto(huart);

    if (contexto == NULL)
    {
        return;
    }

    /* Durante una recuperacion no se aceptan fragmentos del DMA anterior. */
    if (contexto->reiniciar_recepcion != 0U)
    {
        return;
    }

    /* Procesa solamente los bytes nuevos recibidos por DMA. */
    UART_DMA_ProcesarPosicion(contexto, Size);
}


/* Esta función es llamada cuando finaliza una transmisión DMA. */
void HAL_UART_TxCpltCallback(
        UART_HandleTypeDef *huart)
{
    UART_DMA_Contexto *contexto;

    contexto = UART_DMA_ObtenerContexto(huart);

    if (contexto == NULL)
    {
        return;
    }

    /* Libera el búfer para permitir la siguiente transmisión. */
    contexto->tx_ocupado = 0U;
}


/* Esta función es llamada automáticamente cuando HAL detecta
 * un error en la UART.
 */
void HAL_UART_ErrorCallback(
        UART_HandleTypeDef *huart)
{
    UART_DMA_Contexto *contexto;

    contexto = UART_DMA_ObtenerContexto(huart);

    if (contexto == NULL)
    {
        return;
    }

    /* Incrementa el contador para diagnóstico. */
    contexto->cantidad_errores++;

    /* Libera una posible transmisión interrumpida. */
    /*
     * Se difiere esa liberacion a Procesar y solamente cuando HAL
     * indique gState READY. Un error RX puede ocurrir mientras el
     * DMA TX sigue leyendo el buffer, por lo que aqui no se libera.
     */

    /* Marca la discontinuidad para no unir fragmentos a traves del error. */
    contexto->perdida_recepcion = 1U;
    contexto->desbordamiento = 1U;

    /* Solicita que el programa principal recupere
     * posteriormente la recepción.
     */
    contexto->reiniciar_recepcion = 1U;
}
