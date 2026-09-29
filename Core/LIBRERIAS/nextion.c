/*
 * nextion.c
 *
 * Librería para enviar información desde el STM32H562RGT6
 * hacia la pantalla Nextion mediante UART y DMA.
 *
 * Esta versión conserva la forma de trabajo de la librería
 * suministrada por el profesor, pero la transmisión se realiza
 * mediante HAL_UART_Transmit_DMA().
 */

#include "nextion.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

/* La longitud que recibe HAL_UART_Transmit_DMA() es de 16 bits. */
#if (NEXTION_BUFFER_TX_TAMANO < 4U) || (NEXTION_BUFFER_TX_TAMANO > UINT16_MAX)
#error "NEXTION_BUFFER_TX_TAMANO debe estar entre 4 y 65535 bytes."
#endif


/*
 * Buffer permanente utilizado por el DMA de transmisión.
 *
 * El buffer debe permanecer en memoria mientras el DMA está
 * enviando los datos. Por ese motivo no puede ser una variable
 * local dentro de las funciones.
 */
static uint8_t nextion_buffer_tx[NEXTION_BUFFER_TX_TAMANO];

/*
 * Recuerda que UART esta utilizando el buffer compartido.
 * Aunque otra UART este libre, no puede modificar un envio pendiente.
 * Llamar estas funciones desde el lazo principal, no desde interrupciones.
 */
static UART_HandleTypeDef *nextion_uart_tx = NULL;


/*
 * Secuencia que Nextion exige al finalizar cada comando.
 *
 * Cada instrucción enviada a la pantalla debe terminar con:
 *
 * 0xFF 0xFF 0xFF
 */
static const uint8_t nextion_fin_comando[3] =
{
    0xFF,
    0xFF,
    0xFF
};


/*
 * Verifica si la parte de transmisión de la UART se encuentra libre.
 *
 * Se revisa únicamente gState porque UART5 puede estar recibiendo
 * continuamente mediante DMA al mismo tiempo que transmite.
 *
 * Retorna:
 *
 * 1 = transmisión disponible.
 * 0 = transmisión ocupada o puntero inválido.
 */
uint8_t NEXTION_TransmisionLista(UART_HandleTypeDef *huart)
{
    /* Verifica que el manejador recibido sea válido. */
    if ((huart == NULL) || (huart->hdmatx == NULL))
    {
        return 0U;
    }

    /* Protege el buffer incluso si se recibe un manejador UART diferente. */
    if ((nextion_uart_tx != NULL) &&
        (nextion_uart_tx->gState != HAL_UART_STATE_READY))
    {
        return 0U;
    }

    /*
     * gState representa el estado del transmisor.
     *
     * Si vale HAL_UART_STATE_READY, se puede iniciar
     * una nueva transferencia mediante DMA.
     */
    if (huart->gState == HAL_UART_STATE_READY)
    {
        return 1U;
    }

    return 0U;
}


/*
 * Inicializa la librería de la pantalla Nextion.
 *
 * La configuración física de UART5 ya fue realizada en CubeMX.
 * Por ahora esta función solamente verifica el manejador.
 *
 * Más adelante podrá utilizarse para enviar configuraciones
 * iniciales cuando la pantalla física esté disponible.
 */
HAL_StatusTypeDef NEXTION_Init(UART_HandleTypeDef *huart)
{
    /* No se puede inicializar la librería con un puntero nulo. */
    if (huart == NULL)
    {
        return HAL_ERROR;
    }

    /* CubeMX debe haber enlazado un canal DMA de transmision a la UART. */
    if (huart->hdmatx == NULL)
    {
        return HAL_ERROR;
    }

    /*
     * No es necesario reinicializar UART5 aquí porque
     * CubeMX ejecuta MX_UART5_Init() desde main.c.
     */
    return HAL_OK;
}


/*
 * Agrega un comando formateado al buffer general de Nextion.
 *
 * Esta función se utiliza para agrupar varios comandos en una
 * sola transferencia DMA.
 *
 * Parámetros:
 *
 * buffer        = memoria donde se construye la transmisión.
 * tamano_buffer = capacidad total del buffer.
 * utilizados    = cantidad de bytes ya almacenados.
 * formato       = formato que se escribirá en el buffer.
 *
 * Retorna:
 *
 * 1 = comando agregado correctamente.
 * 0 = error o espacio insuficiente.
 */
static uint8_t NEXTION_AgregarComando(
    uint8_t *buffer,
    size_t tamano_buffer,
    size_t *utilizados,
    const char *formato,
    ...)
{
    va_list argumentos;
    int escritos;
    size_t espacio_disponible;

    /* Verifica todos los punteros recibidos. */
    if ((buffer == NULL) ||
        (utilizados == NULL) ||
        (formato == NULL))
    {
        return 0U;
    }

    /* Verifica que todavía exista espacio en el buffer. */
    if (*utilizados >= tamano_buffer)
    {
        return 0U;
    }

    /* Calcula el espacio disponible desde la posición actual. */
    espacio_disponible = tamano_buffer - *utilizados;

    /*
     * Deben quedar al menos cuatro posiciones:
     *
     * - Una para el comando.
     * - Tres para 0xFF 0xFF 0xFF.
     */
    if (espacio_disponible <= sizeof(nextion_fin_comando))
    {
        return 0U;
    }

    /* Inicia la lectura de los argumentos variables. */
    va_start(argumentos, formato);

    /* Escribe el comando dentro del buffer de transmisión. */
    escritos = vsnprintf(
        (char *)&buffer[*utilizados],
        espacio_disponible,
        formato,
        argumentos);

    /* Finaliza la lectura de argumentos variables. */
    va_end(argumentos);

    /* Verifica que snprintf haya trabajado correctamente. */
    if (escritos < 0)
    {
        return 0U;
    }

    /*
     * Verifica que el comando y los tres bytes finales
     * entren completamente en el buffer.
     */
    /* El nulo temporal de snprintf se reemplaza por el primer byte 0xFF. */
    if (((size_t)escritos + sizeof(nextion_fin_comando)) >
        espacio_disponible)
    {
        return 0U;
    }

    /* Avanza hasta el final del comando escrito. */
    *utilizados += (size_t)escritos;

    /* Agrega los tres bytes de finalización exigidos por Nextion. */
    memcpy(
        &buffer[*utilizados],
        nextion_fin_comando,
        sizeof(nextion_fin_comando));

    /* Actualiza la cantidad total de bytes almacenados. */
    *utilizados += sizeof(nextion_fin_comando);

    return 1U;
}


/*
 * Envía un comando individual a la pantalla mediante DMA.
 *
 * La función copia el comando al buffer permanente y agrega
 * automáticamente los tres bytes 0xFF.
 */
HAL_StatusTypeDef NEXTION_EnviarComando(
    UART_HandleTypeDef *huart,
    const char *comando)
{
    size_t longitud_comando;
    size_t longitud_total;

    /* Verifica que los punteros sean válidos. */
    if ((huart == NULL) || (comando == NULL))
    {
        return HAL_ERROR;
    }

    /* Un DMA no configurado es un error, no una transmision ocupada. */
    if (huart->hdmatx == NULL)
    {
        return HAL_ERROR;
    }

    /*
     * No se debe modificar el buffer mientras el DMA
     * esté realizando una transmisión anterior.
     */
    if (NEXTION_TransmisionLista(huart) == 0U)
    {
        return HAL_BUSY;
    }

    /* Obtiene la cantidad de caracteres del comando. */
    longitud_comando = strlen(comando);

    /* Comprueba el limite antes de sumar los bytes de finalizacion. */
    if ((longitud_comando == 0U) ||
        (longitud_comando >
         (sizeof(nextion_buffer_tx) - sizeof(nextion_fin_comando))))
    {
        return HAL_ERROR;
    }

    /*
     * Calcula el tamaño completo incluyendo los tres
     * bytes de finalización.
     */
    longitud_total =
        longitud_comando + sizeof(nextion_fin_comando);

    /* Verifica que el comando entre en el buffer permanente. */
    if (longitud_total > sizeof(nextion_buffer_tx))
    {
        return HAL_ERROR;
    }

    /* Copia el comando al buffer que utilizará el DMA. */
    memcpy(
        nextion_buffer_tx,
        comando,
        longitud_comando);

    /* Agrega 0xFF 0xFF 0xFF después del comando. */
    memcpy(
        &nextion_buffer_tx[longitud_comando],
        nextion_fin_comando,
        sizeof(nextion_fin_comando));

    /*
     * Inicia la transferencia mediante el canal DMA
     * asignado a UART5_TX.
     */
    /* Registra el manejador antes de entregar el buffer al DMA. */
    nextion_uart_tx = huart;
    return HAL_UART_Transmit_DMA(
        huart,
        nextion_buffer_tx,
        (uint16_t)longitud_total);
}


/*
 * Envía texto hacia un objeto de texto de Nextion.
 *
 * Ejemplo:
 *
 * NEXTION_EnviarTexto(&huart5, "tParada", "NORMAL");
 *
 * Comando producido:
 *
 * tParada.txt="NORMAL"
 */
HAL_StatusTypeDef NEXTION_EnviarTexto(
    UART_HandleTypeDef *huart,
    const char *objeto,
    const char *texto)
{
    char comando[NEXTION_COMANDO_MAXIMO];
    int resultado;

    /* Verifica los punteros recibidos. */
    if ((huart == NULL) ||
        (objeto == NULL) ||
        (texto == NULL))
    {
        return HAL_ERROR;
    }

    /* Construye el comando reconocido por la pantalla Nextion. */
    resultado = snprintf(
        comando,
        sizeof(comando),
        "%s.txt=\"%s\"",
        objeto,
        texto);

    /* Verifica que el comando haya entrado en el buffer local. */
    if ((resultado < 0) ||
        ((size_t)resultado >= sizeof(comando)))
    {
        return HAL_ERROR;
    }

    /* Envía el comando completo mediante DMA. */
    return NEXTION_EnviarComando(huart, comando);
}


/*
 * Envía un número entero hacia la propiedad val de un objeto.
 *
 * Ejemplo:
 *
 * NEXTION_EnviarEntero(&huart5, "zAngulo", 90);
 *
 * Comando producido:
 *
 * zAngulo.val=90
 */
HAL_StatusTypeDef NEXTION_EnviarEntero(
    UART_HandleTypeDef *huart,
    const char *objeto,
    int32_t valor)
{
    char comando[NEXTION_COMANDO_MAXIMO];
    int resultado;

    /* Verifica los punteros recibidos. */
    if ((huart == NULL) || (objeto == NULL))
    {
        return HAL_ERROR;
    }

    /* Construye el comando de asignación numérica. */
    resultado = snprintf(
        comando,
        sizeof(comando),
        "%s.val=%ld",
        objeto,
        (long)valor);

    /* Verifica que el comando se haya construido correctamente. */
    if ((resultado < 0) ||
        ((size_t)resultado >= sizeof(comando)))
    {
        return HAL_ERROR;
    }

    /* Envía el comando hacia la pantalla mediante DMA. */
    return NEXTION_EnviarComando(huart, comando);
}


/*
 * Envía un número decimal convertido a texto.
 *
 * Ejemplo:
 *
 * NEXTION_EnviarFloat(&huart5, "tBateria", 12.5f, 1);
 *
 * Comando producido:
 *
 * tBateria.txt="12.5"
 */
HAL_StatusTypeDef NEXTION_EnviarFloat(
    UART_HandleTypeDef *huart,
    const char *objeto,
    float valor,
    uint8_t decimales)
{
    char comando[NEXTION_COMANDO_MAXIMO];
    int resultado;

    /* Verifica los punteros recibidos. */
    if ((huart == NULL) || (objeto == NULL))
    {
        return HAL_ERROR;
    }

    /* No envia NaN ni infinito como si fueran una medicion valida. */
    if (!isfinite(valor))
    {
        return HAL_ERROR;
    }

    /*
     * Limita los decimales para evitar generar comandos
     * innecesariamente largos.
     */
    if (decimales > 3U)
    {
        decimales = 3U;
    }

    /* Construye el texto decimal que será mostrado. */
    resultado = snprintf(
        comando,
        sizeof(comando),
        "%s.txt=\"%.*f\"",
        objeto,
        (int)decimales,
        (double)valor);

    /* Verifica que el resultado entre en el buffer. */
    if ((resultado < 0) ||
        ((size_t)resultado >= sizeof(comando)))
    {
        return HAL_ERROR;
    }

    /* Envía el comando mediante UART5 y DMA. */
    return NEXTION_EnviarComando(huart, comando);
}


/*
 * Envía en una sola transferencia DMA los datos utilizados
 * por la interfaz de prueba del profesor.
 *
 * Datos mostrados:
 *
 * - Temperatura.
 * - Voltaje de batería.
 * - Ángulo de la cámara.
 * - Indicador gráfico del ángulo.
 * - Estado de la parada de emergencia.
 *
 * Los nombres de los objetos deben existir en el proyecto HMI:
 *
 * tTemperatura
 * tBateria
 * tNumAngulo
 * zAngulo
 * tParada
 */
HAL_StatusTypeDef NEXTION_EnviarTrama(
    UART_HandleTypeDef *huart,
    float temperatura,
    float bateria,
    float angulo_camara,
    uint8_t parada_emergencia)
{
    size_t utilizados = 0U;
    int32_t angulo_aguja;
    const char *texto_parada;
    uint16_t color_parada;

    /* Verifica que el manejador UART sea válido. */
    if (huart == NULL)
    {
        return HAL_ERROR;
    }

    /* Verifica el DMA y evita convertir un angulo NaN a un numero entero. */
    if ((huart->hdmatx == NULL) ||
        !isfinite(temperatura) ||
        !isfinite(bateria) ||
        !isfinite(angulo_camara))
    {
        return HAL_ERROR;
    }

    /*
     * Impide modificar el buffer mientras exista
     * una transmisión DMA en ejecución.
     */
    if (NEXTION_TransmisionLista(huart) == 0U)
    {
        return HAL_BUSY;
    }

    /* Limita el ángulo de cámara entre -90 y +90 grados. */
    if (angulo_camara < -90.0f)
    {
        angulo_camara = -90.0f;
    }
    else if (angulo_camara > 90.0f)
    {
        angulo_camara = 90.0f;
    }

    /*
     * Nextion no acepta valores negativos en el indicador.
     *
     * Se convierte:
     *
     * -90 grados -> 0
     *   0 grados -> 90
     * +90 grados -> 180
     */
    angulo_aguja = (int32_t)(angulo_camara + 90.0f);

    /*
     * Selecciona el texto y el color correspondiente
     * al estado del botón de emergencia.
     */
    if (parada_emergencia != 0U)
    {
        texto_parada = "EMERGENCIA";

        /* Código RGB565 utilizado por Nextion para rojo. */
        color_parada = 63488U;
    }
    else
    {
        texto_parada = "NORMAL";

        /* Código RGB565 utilizado por Nextion para verde. */
        color_parada = 2016U;
    }

    /* Agrega la temperatura al buffer general. */
    if (NEXTION_AgregarComando(
            nextion_buffer_tx,
            sizeof(nextion_buffer_tx),
            &utilizados,
            "tTemperatura.txt=\"%.2f\"",
            (double)temperatura) == 0U)
    {
        return HAL_ERROR;
    }

    /* Agrega el voltaje de batería al buffer general. */
    if (NEXTION_AgregarComando(
            nextion_buffer_tx,
            sizeof(nextion_buffer_tx),
            &utilizados,
            "tBateria.txt=\"%.2f\"",
            (double)bateria) == 0U)
    {
        return HAL_ERROR;
    }

    /* Agrega el valor numérico del ángulo de la cámara. */
    if (NEXTION_AgregarComando(
            nextion_buffer_tx,
            sizeof(nextion_buffer_tx),
            &utilizados,
            "tNumAngulo.txt=\"%.1f\"",
            (double)angulo_camara) == 0U)
    {
        return HAL_ERROR;
    }

    /* Agrega la posición del indicador gráfico. */
    if (NEXTION_AgregarComando(
            nextion_buffer_tx,
            sizeof(nextion_buffer_tx),
            &utilizados,
            "zAngulo.val=%ld",
            (long)angulo_aguja) == 0U)
    {
        return HAL_ERROR;
    }

    /* Agrega el texto del estado de emergencia. */
    if (NEXTION_AgregarComando(
            nextion_buffer_tx,
            sizeof(nextion_buffer_tx),
            &utilizados,
            "tParada.txt=\"%s\"",
            texto_parada) == 0U)
    {
        return HAL_ERROR;
    }

    /* Agrega el color de fondo del indicador de emergencia. */
    if (NEXTION_AgregarComando(
            nextion_buffer_tx,
            sizeof(nextion_buffer_tx),
            &utilizados,
            "tParada.bco=%u",
            (unsigned int)color_parada) == 0U)
    {
        return HAL_ERROR;
    }

    /*
     * Envía todos los comandos anteriores utilizando
     * una sola transferencia por UART5_TX y DMA.
     */
    /* Mantiene protegido el buffer hasta que esta UART termine el envio. */
    nextion_uart_tx = huart;
    return HAL_UART_Transmit_DMA(
        huart,
        nextion_buffer_tx,
        (uint16_t)utilizados);
}
