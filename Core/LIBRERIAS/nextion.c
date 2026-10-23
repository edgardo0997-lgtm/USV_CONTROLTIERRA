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

/* Colores RGB565 utilizados por la interfaz HMI. */
#define NEXTION_COLOR_ROJO             63488U
#define NEXTION_COLOR_VERDE             2016U
#define NEXTION_COLOR_GRIS             33840U

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

    /*
     * No es necesario reinicializar USART1 aqui porque CubeMX ya ejecuto
     * MX_USART1_UART_Init(). La actualizacion completa usa interrupciones
     * y no depende de que exista un canal DMA para la pantalla.
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
 * Agrega texto a un objeto y reemplaza caracteres que podrian cerrar
 * prematuramente una cadena de comandos Nextion.
 */
static uint8_t NEXTION_AgregarTextoSeguro(
    uint8_t *buffer,
    size_t tamano_buffer,
    size_t *utilizados,
    const char *objeto,
    const char *texto)
{
    char texto_seguro[NEXTION_COMANDO_MAXIMO];
    size_t origen;
    size_t destino = 0U;

    if ((objeto == NULL) || (texto == NULL))
    {
        return 0U;
    }

    for (origen = 0U; texto[origen] != '\0'; origen++)
    {
        if ((destino + 1U) >= sizeof(texto_seguro))
        {
            return 0U;
        }

        if ((texto[origen] == '"') ||
            (texto[origen] == '\\') ||
            ((uint8_t)texto[origen] < 0x20U))
        {
            texto_seguro[destino] = '?';
        }
        else
        {
            texto_seguro[destino] = texto[origen];
        }

        destino++;
    }

    texto_seguro[destino] = '\0';

    return NEXTION_AgregarComando(
        buffer,
        tamano_buffer,
        utilizados,
        "%s.txt=\"%s\"",
        objeto,
        texto_seguro);
}


/* Agrega un valor x10 con una cifra decimal y su unidad. */
static uint8_t NEXTION_AgregarDecimalX10(
    uint8_t *buffer,
    size_t tamano_buffer,
    size_t *utilizados,
    const char *objeto,
    int32_t valor_x10,
    const char *unidad)
{
    char texto[48];
    uint32_t magnitud;
    const char *signo;
    int resultado;

    if ((objeto == NULL) || (unidad == NULL))
    {
        return 0U;
    }

    if (valor_x10 < 0)
    {
        signo = "-";
        /* Tambien funciona para INT32_MIN sin usar operaciones de 64 bits. */
        magnitud = (uint32_t)(-(valor_x10 + 1)) + 1U;
    }
    else
    {
        signo = "";
        magnitud = (uint32_t)valor_x10;
    }

    resultado = snprintf(
        texto,
        sizeof(texto),
        "%s%lu.%01lu%s",
        signo,
        (unsigned long)(magnitud / 10U),
        (unsigned long)(magnitud % 10U),
        unidad);

    if ((resultado < 0) || ((size_t)resultado >= sizeof(texto)))
    {
        return 0U;
    }

    return NEXTION_AgregarTextoSeguro(
        buffer,
        tamano_buffer,
        utilizados,
        objeto,
        texto);
}


/* Reproduce para la pantalla el signo que utiliza la trama $PUSVU. */
static int16_t NEXTION_PotenciaConSigno(
    uint16_t potencia,
    uint8_t avante,
    uint8_t atras,
    uint8_t parada,
    uint8_t falla)
{
    if ((parada != 0U) || (falla != 0U))
    {
        return 0;
    }

    if ((avante != 0U) && (atras == 0U))
    {
        return (int16_t)potencia;
    }

    if ((avante == 0U) && (atras != 0U))
    {
        return -(int16_t)potencia;
    }

    return 0;
}


/* Convierte hhmmss.ss a hh:mm:ss para el objeto tHora. */
static uint8_t NEXTION_FormatearUTC(
    const char *utc,
    char *destino,
    size_t capacidad)
{
    size_t indice;
    int resultado;

    if ((utc == NULL) || (destino == NULL) || (capacidad == 0U))
    {
        return 0U;
    }

    for (indice = 0U; indice < 6U; indice++)
    {
        if ((utc[indice] < '0') || (utc[indice] > '9'))
        {
            return 0U;
        }
    }

    resultado = snprintf(
        destino,
        capacidad,
        "%c%c:%c%c:%c%c",
        utc[0], utc[1],
        utc[2], utc[3],
        utc[4], utc[5]);

    return ((resultado == 8) && ((size_t)resultado < capacidad)) ? 1U : 0U;
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


/*
 * Actualiza los objetos reales del proyecto HMI recibido.
 *
 * page0:
 * - bateria, corriente, temperatura, inundacion, GPS, velocidad y rumbo;
 * - potencias de babor/estribor, camara, bomba y parada.
 *
 * pageIMU:
 * - yaw, pitch, roll y estado de inclinacion.
 *
 * Los comandos de ambas paginas se agrupan en una sola transferencia DMA.
 */
/*
 * Envia el estado completo repartido en cuatro bloques pequenos.
 * Cada llamada transmite un bloque diferente mediante DMA y luego avanza
 * al siguiente. La funcion solamente inicia el envio y regresa enseguida:
 * una pantalla desconectada, ocupada o con error no detiene el ciclo que
 * transmite la trama de control al bote por UART5.
 */
static HAL_StatusTypeDef NEXTION_EnviarEstadoUSVPorBloques(
    UART_HandleTypeDef *huart,
    const USV_Telemetria *telemetria,
    uint8_t telemetria_valida,
    const USV_Comando *comando,
    float voltaje_control)
{
    static uint8_t bloque = 0U;
    size_t utilizados = 0U;
    int16_t motor_babor;
    int16_t motor_estribor;
    uint16_t magnitud_babor;
    uint16_t magnitud_estribor;
    uint16_t color_parada;
    uint16_t color_bomba_on;
    uint16_t color_bomba_off;
    const char *texto_parada;
    char texto[64];
    int resultado;

    if ((huart == NULL) ||
        (telemetria == NULL) ||
        (comando == NULL) ||
        !isfinite(voltaje_control))
    {
        return HAL_ERROR;
    }

    /*
     * Si USART1 aun transmite el bloque anterior, retorna inmediatamente.
     * No espera, no aborta y no afecta la UART5 utilizada por el bote.
     */
    if (huart->gState != HAL_UART_STATE_READY)
    {
        return HAL_BUSY;
    }

    motor_babor = NEXTION_PotenciaConSigno(
        comando->potencia_babor,
        comando->babor_avante,
        comando->babor_atras,
        comando->parada,
        comando->falla_direccion);

    motor_estribor = NEXTION_PotenciaConSigno(
        comando->potencia_estribor,
        comando->estribor_avante,
        comando->estribor_atras,
        comando->parada,
        comando->falla_direccion);

    magnitud_babor =
        (uint16_t)((motor_babor < 0) ? -motor_babor : motor_babor);
    magnitud_estribor =
        (uint16_t)((motor_estribor < 0) ? -motor_estribor : motor_estribor);

    if (comando->parada != 0U)
    {
        texto_parada = "EMERGENCIA";
        color_parada = NEXTION_COLOR_ROJO;
    }
    else
    {
        texto_parada = "NORMAL";
        color_parada = NEXTION_COLOR_VERDE;
    }

    if (comando->bomba != 0U)
    {
        color_bomba_on = NEXTION_COLOR_VERDE;
        color_bomba_off = NEXTION_COLOR_GRIS;
    }
    else
    {
        color_bomba_on = NEXTION_COLOR_GRIS;
        color_bomba_off = NEXTION_COLOR_ROJO;
    }

    switch (bloque)
    {
        case 0U:
        {
            int32_t angulo_aguja = (int32_t)comando->camara + 90;

            if (angulo_aguja < 0)
            {
                angulo_aguja = 0;
            }
            else if (angulo_aguja > 180)
            {
                angulo_aguja = 180;
            }

            if ((NEXTION_AgregarComando(
                    nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                    "tNumAngulo.txt=\"%d\"", (int)comando->camara) == 0U) ||
                (NEXTION_AgregarComando(
                    nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                    "zAngulo.val=%ld", (long)angulo_aguja) == 0U) ||
                (NEXTION_AgregarTextoSeguro(
                    nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                    "tParada", texto_parada) == 0U) ||
                (NEXTION_AgregarComando(
                    nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                    "tParada.bco=%u", (unsigned int)color_parada) == 0U) ||
                (NEXTION_AgregarComando(
                    nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                    "tBabor.txt=\"%d %%\"", (int)motor_babor) == 0U) ||
                (NEXTION_AgregarComando(
                    nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                    "jBABOR.val=%u", (unsigned int)magnitud_babor) == 0U) ||
                (NEXTION_AgregarComando(
                    nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                    "tEstribor.txt=\"%d %%\"", (int)motor_estribor) == 0U) ||
                (NEXTION_AgregarComando(
                    nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                    "jEstribor.val=%u", (unsigned int)magnitud_estribor) == 0U) ||
                (NEXTION_AgregarComando(
                    nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                    "tBomba2.bco=%u", (unsigned int)color_bomba_on) == 0U) ||
                (NEXTION_AgregarComando(
                    nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                    "tBomba1.bco=%u", (unsigned int)color_bomba_off) == 0U))
            {
                return HAL_ERROR;
            }
            break;
        }

        case 1U:
        {
            int32_t bateria_x10;
            int32_t corriente_x10;
            int32_t temperatura_x10;
            uint32_t barra_bateria;
            uint32_t barra_corriente;
            uint32_t barra_temperatura;
            const char *estado_agua;
            uint32_t barra_agua;

            if (telemetria_valida != 0U)
            {
                bateria_x10 = (int32_t)telemetria->voltaje_x10;
                corriente_x10 = (int32_t)telemetria->corriente_x10;
                temperatura_x10 = (int32_t)telemetria->temperatura_x10;
                estado_agua =
                    (telemetria->inundacion != 0U) ? "AGUA" : "SECO";
                barra_agua =
                    (telemetria->inundacion != 0U) ? 100U : 0U;
            }
            else
            {
                if (voltaje_control < 0.0f)
                {
                    voltaje_control = 0.0f;
                }
                else if (voltaje_control > 14.0f)
                {
                    voltaje_control = 14.0f;
                }

                bateria_x10 =
                    (int32_t)((voltaje_control * 10.0f) + 0.5f);
                corriente_x10 = 0;
                temperatura_x10 = 0;
                estado_agua = "SIN DATOS";
                barra_agua = 0U;
            }

            barra_bateria =
                ((uint32_t)bateria_x10 * 100U + 70U) / 140U;
            barra_corriente =
                ((uint32_t)corriente_x10 * 100U + 500U) / 1000U;

            if (temperatura_x10 < -100)
            {
                barra_temperatura = 0U;
            }
            else
            {
                barra_temperatura =
                    ((uint32_t)(temperatura_x10 + 100) * 100U + 475U) /
                    950U;
            }

            if ((NEXTION_AgregarDecimalX10(
                    nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                    "tBateria", bateria_x10, " V") == 0U) ||
                (NEXTION_AgregarComando(
                    nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                    "jbat.val=%lu", (unsigned long)barra_bateria) == 0U) ||
                (NEXTION_AgregarDecimalX10(
                    nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                    "tCorr", corriente_x10, " A") == 0U) ||
                (NEXTION_AgregarComando(
                    nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                    "jCorr.val=%lu", (unsigned long)barra_corriente) == 0U) ||
                (NEXTION_AgregarDecimalX10(
                    nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                    "tTemperatura", temperatura_x10, " C") == 0U) ||
                (NEXTION_AgregarComando(
                    nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                    "barTemp.val=%lu", (unsigned long)barra_temperatura) == 0U) ||
                (NEXTION_AgregarTextoSeguro(
                    nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                    "tHumedad", estado_agua) == 0U) ||
                (NEXTION_AgregarComando(
                    nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                    "jHumedad.val=%lu", (unsigned long)barra_agua) == 0U))
            {
                return HAL_ERROR;
            }
            break;
        }

        case 2U:
        {
            if (telemetria_valida == 0U)
            {
                if ((NEXTION_AgregarTextoSeguro(
                        nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                        "gps_satelites", "SAT 0") == 0U) ||
                    (NEXTION_AgregarTextoSeguro(
                        nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                        "lblGPS", "SIN DATOS") == 0U) ||
                    (NEXTION_AgregarTextoSeguro(
                        nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                        "tLatitud", "---") == 0U) ||
                    (NEXTION_AgregarTextoSeguro(
                        nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                        "tLongitud", "---") == 0U) ||
                    (NEXTION_AgregarTextoSeguro(
                        nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                        "tvel", "--.- km/h") == 0U) ||
                    (NEXTION_AgregarComando(
                        nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                        "barVEL.val=0") == 0U) ||
                    (NEXTION_AgregarTextoSeguro(
                        nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                        "tRumbo", "---") == 0U) ||
                    (NEXTION_AgregarComando(
                        nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                        "zRumbo.val=0") == 0U) ||
                    (NEXTION_AgregarTextoSeguro(
                        nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                        "page0.tHora", "--:--:--") == 0U))
                {
                    return HAL_ERROR;
                }
            }
            else
            {
                uint32_t barra_velocidad;
                uint32_t velocidad_kmh_x10;
                uint32_t rumbo_entero;
                const char *estado_gps;

                barra_velocidad =
                    ((uint32_t)telemetria->velocidad_x10 * 100U + 150U) /
                    300U;
                velocidad_kmh_x10 =
                    ((uint32_t)telemetria->velocidad_x10 * 1852U + 500U) /
                    1000U;
                rumbo_entero = (uint32_t)telemetria->rumbo_x10 / 10U;
                estado_gps =
                    (telemetria->calidad_gps != 0U) ? "GPS OK" : "SIN FIX";

                resultado = snprintf(
                    texto, sizeof(texto), "SAT %u",
                    (unsigned int)telemetria->satelites);
                if ((resultado < 0) || ((size_t)resultado >= sizeof(texto)) ||
                    (NEXTION_AgregarTextoSeguro(
                        nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                        "gps_satelites", texto) == 0U) ||
                    (NEXTION_AgregarTextoSeguro(
                        nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                        "lblGPS", estado_gps) == 0U))
                {
                    return HAL_ERROR;
                }

                resultado = snprintf(
                    texto, sizeof(texto), "%s %c",
                    telemetria->latitud, telemetria->hemisferio_latitud);
                if ((resultado < 0) || ((size_t)resultado >= sizeof(texto)) ||
                    (NEXTION_AgregarTextoSeguro(
                        nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                        "tLatitud", texto) == 0U))
                {
                    return HAL_ERROR;
                }

                resultado = snprintf(
                    texto, sizeof(texto), "%s %c",
                    telemetria->longitud, telemetria->hemisferio_longitud);
                if ((resultado < 0) || ((size_t)resultado >= sizeof(texto)) ||
                    (NEXTION_AgregarTextoSeguro(
                        nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                        "tLongitud", texto) == 0U) ||
                    (NEXTION_AgregarDecimalX10(
                        nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                        "tvel", (int32_t)velocidad_kmh_x10, " km/h") == 0U) ||
                    (NEXTION_AgregarComando(
                        nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                        "barVEL.val=%lu", (unsigned long)barra_velocidad) == 0U))
                {
                    return HAL_ERROR;
                }

                resultado = snprintf(
                    texto, sizeof(texto), "%03lu", (unsigned long)rumbo_entero);
                if ((resultado < 0) || ((size_t)resultado >= sizeof(texto)) ||
                    (NEXTION_AgregarTextoSeguro(
                        nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                        "tRumbo", texto) == 0U) ||
                    (NEXTION_AgregarComando(
                        nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                        "zRumbo.val=%lu", (unsigned long)rumbo_entero) == 0U))
                {
                    return HAL_ERROR;
                }

                if (NEXTION_FormatearUTC(
                        telemetria->utc, texto, sizeof(texto)) == 0U)
                {
                    strcpy(texto, "--:--:--");
                }

                if (NEXTION_AgregarTextoSeguro(
                        nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                        "page0.tHora", texto) == 0U)
                {
                    return HAL_ERROR;
                }
            }
            break;
        }

        default:
        {
            if (telemetria_valida == 0U)
            {
                if ((NEXTION_AgregarTextoSeguro(
                        nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                        "tYaw", "--.-") == 0U) ||
                    (NEXTION_AgregarComando(
                        nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                        "zYaw.val=0") == 0U) ||
                    (NEXTION_AgregarTextoSeguro(
                        nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                        "tPitch", "--.-") == 0U) ||
                    (NEXTION_AgregarTextoSeguro(
                        nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                        "tRoll", "--.-") == 0U) ||
                    (NEXTION_AgregarTextoSeguro(
                        nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                        "tInclinacion", "SIN DATOS") == 0U))
                {
                    return HAL_ERROR;
                }
            }
            else
            {
                uint32_t yaw_entero = (uint32_t)telemetria->yaw_x10 / 10U;
                const char *estado_inclinacion;

                if (((telemetria->pitch_x10 >= -30) &&
                     (telemetria->pitch_x10 <= 30)) &&
                    ((telemetria->roll_x10 >= -30) &&
                     (telemetria->roll_x10 <= 30)))
                {
                    estado_inclinacion = "NIVEL";
                }
                else
                {
                    estado_inclinacion = "INCLINADO";
                }

                if ((NEXTION_AgregarDecimalX10(
                        nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                        "tYaw", telemetria->yaw_x10, "") == 0U) ||
                    (NEXTION_AgregarComando(
                        nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                        "zYaw.val=%lu", (unsigned long)yaw_entero) == 0U) ||
                    (NEXTION_AgregarDecimalX10(
                        nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                        "tPitch", telemetria->pitch_x10, "") == 0U) ||
                    (NEXTION_AgregarDecimalX10(
                        nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                        "tRoll", telemetria->roll_x10, "") == 0U) ||
                    (NEXTION_AgregarTextoSeguro(
                        nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                        "tInclinacion", estado_inclinacion) == 0U))
                {
                    return HAL_ERROR;
                }
            }
            break;
        }
    }

    /* Avanza aunque este bloque falle: un dato de pantalla nunca detiene USV. */
    bloque = (uint8_t)((bloque + 1U) % 4U);

    if ((utilizados == 0U) || (utilizados > UINT16_MAX))
    {
        return HAL_ERROR;
    }

    /* Arranca por interrupcion y retorna de inmediato al ciclo principal. */
    nextion_uart_tx = huart;

    return HAL_UART_Transmit_IT(
        huart,
        nextion_buffer_tx,
        (uint16_t)utilizados);
}


HAL_StatusTypeDef NEXTION_EnviarEstadoUSV(
    UART_HandleTypeDef *huart,
    const USV_Telemetria *telemetria,
    uint8_t telemetria_valida,
    const USV_Comando *comando,
    float voltaje_control)
{
#if 1
    return NEXTION_EnviarEstadoUSVPorBloques(
        huart,
        telemetria,
        telemetria_valida,
        comando,
        voltaje_control);
#else
    size_t utilizados = 0U;
    int16_t motor_babor;
    int16_t motor_estribor;
    uint16_t magnitud_babor;
    uint16_t magnitud_estribor;
    int32_t angulo_aguja;
    uint16_t color_parada;
    uint16_t color_bomba_on;
    uint16_t color_bomba_off;
    const char *texto_parada;
    char texto[64];
    int resultado;

    if ((huart == NULL) ||
        (telemetria == NULL) ||
        (comando == NULL) ||
        !isfinite(voltaje_control))
    {
        return HAL_ERROR;
    }

    /*
     * Esta funcion no comparte una transferencia DMA pendiente. Si USART1
     * esta ocupada, se omite esta actualizacion sin afectar UART5.
     */
    if (huart->gState != HAL_UART_STATE_READY)
    {
        return HAL_BUSY;
    }

    motor_babor = NEXTION_PotenciaConSigno(
        comando->potencia_babor,
        comando->babor_avante,
        comando->babor_atras,
        comando->parada,
        comando->falla_direccion);

    motor_estribor = NEXTION_PotenciaConSigno(
        comando->potencia_estribor,
        comando->estribor_avante,
        comando->estribor_atras,
        comando->parada,
        comando->falla_direccion);

    magnitud_babor = (uint16_t)((motor_babor < 0) ? -motor_babor : motor_babor);
    magnitud_estribor =
        (uint16_t)((motor_estribor < 0) ? -motor_estribor : motor_estribor);

    angulo_aguja = (int32_t)comando->camara + 90;
    if (angulo_aguja < 0)
    {
        angulo_aguja = 0;
    }
    else if (angulo_aguja > 180)
    {
        angulo_aguja = 180;
    }

    if (comando->parada != 0U)
    {
        texto_parada = "EMERGENCIA";
        color_parada = NEXTION_COLOR_ROJO;
    }
    else
    {
        texto_parada = "NORMAL";
        color_parada = NEXTION_COLOR_VERDE;
    }

    if (comando->bomba != 0U)
    {
        color_bomba_on = NEXTION_COLOR_VERDE;
        color_bomba_off = NEXTION_COLOR_GRIS;
    }
    else
    {
        color_bomba_on = NEXTION_COLOR_GRIS;
        color_bomba_off = NEXTION_COLOR_ROJO;
    }

    /* Controles locales: se muestran incluso sin telemetria del bote. */
    if ((NEXTION_AgregarComando(
            nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
            "tNumAngulo.txt=\"%d\"", (int)comando->camara) == 0U) ||
        (NEXTION_AgregarComando(
            nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
            "zAngulo.val=%ld", (long)angulo_aguja) == 0U) ||
        (NEXTION_AgregarTextoSeguro(
            nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
            "tParada", texto_parada) == 0U) ||
        (NEXTION_AgregarComando(
            nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
            "tParada.bco=%u", (unsigned int)color_parada) == 0U) ||
        (NEXTION_AgregarComando(
            nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
            "tBabor.txt=\"%d %%\"", (int)motor_babor) == 0U) ||
        (NEXTION_AgregarComando(
            nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
            "jBABOR.val=%u", (unsigned int)magnitud_babor) == 0U) ||
        (NEXTION_AgregarComando(
            nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
            "tEstribor.txt=\"%d %%\"", (int)motor_estribor) == 0U) ||
        (NEXTION_AgregarComando(
            nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
            "jEstribor.val=%u", (unsigned int)magnitud_estribor) == 0U) ||
        (NEXTION_AgregarComando(
            nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
            "tBomba2.bco=%u", (unsigned int)color_bomba_on) == 0U) ||
        (NEXTION_AgregarComando(
            nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
            "tBomba1.bco=%u", (unsigned int)color_bomba_off) == 0U))
    {
        return HAL_ERROR;
    }

    if (telemetria_valida == 0U)
    {
        int32_t voltaje_local_x10;
        uint32_t barra_bateria;

        if (voltaje_control < 0.0f)
        {
            voltaje_control = 0.0f;
        }
        else if (voltaje_control > 14.0f)
        {
            voltaje_control = 14.0f;
        }

        voltaje_local_x10 = (int32_t)((voltaje_control * 10.0f) + 0.5f);
        barra_bateria = ((uint32_t)voltaje_local_x10 * 100U + 70U) / 140U;

        if ((NEXTION_AgregarDecimalX10(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "tBateria", voltaje_local_x10, " V") == 0U) ||
            (NEXTION_AgregarComando(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "jbat.val=%lu", (unsigned long)barra_bateria) == 0U) ||
            (NEXTION_AgregarTextoSeguro(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "tCorr", "--.- A") == 0U) ||
            (NEXTION_AgregarComando(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "jCorr.val=0") == 0U) ||
            (NEXTION_AgregarTextoSeguro(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "tTemperatura", "--.- C") == 0U) ||
            (NEXTION_AgregarComando(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "barTemp.val=0") == 0U) ||
            (NEXTION_AgregarTextoSeguro(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "tHumedad", "SIN DATOS") == 0U) ||
            (NEXTION_AgregarComando(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "jHumedad.val=0") == 0U) ||
            (NEXTION_AgregarTextoSeguro(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "gps_satelites", "SAT 0") == 0U) ||
            (NEXTION_AgregarTextoSeguro(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "lblGPS", "SIN DATOS") == 0U) ||
            (NEXTION_AgregarTextoSeguro(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "tLatitud", "---") == 0U) ||
            (NEXTION_AgregarTextoSeguro(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "tLongitud", "---") == 0U) ||
            (NEXTION_AgregarTextoSeguro(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "tvel", "--.- km/h") == 0U) ||
            (NEXTION_AgregarComando(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "barVEL.val=0") == 0U) ||
            (NEXTION_AgregarTextoSeguro(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "tRumbo", "---") == 0U) ||
            (NEXTION_AgregarComando(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "zRumbo.val=0") == 0U) ||
            (NEXTION_AgregarTextoSeguro(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "tYaw", "--.-") == 0U) ||
            (NEXTION_AgregarComando(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "zYaw.val=0") == 0U) ||
            (NEXTION_AgregarTextoSeguro(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "tPitch", "--.-") == 0U) ||
            (NEXTION_AgregarTextoSeguro(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "tRoll", "--.-") == 0U) ||
            (NEXTION_AgregarTextoSeguro(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "tInclinacion", "SIN DATOS") == 0U) ||
            (NEXTION_AgregarTextoSeguro(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "page0.tHora", "--:--:--") == 0U))
        {
            return HAL_ERROR;
        }
    }
    else
    {
        uint32_t barra_bateria;
        uint32_t barra_corriente;
        uint32_t barra_temperatura;
        uint32_t barra_velocidad;
        uint32_t velocidad_kmh_x10;
        uint32_t rumbo_entero;
        uint32_t yaw_entero;
        const char *estado_gps;
        const char *estado_inclinacion;

        barra_bateria =
            ((uint32_t)telemetria->voltaje_x10 * 100U + 70U) / 140U;
        barra_corriente =
            ((uint32_t)telemetria->corriente_x10 * 100U + 500U) / 1000U;
        barra_temperatura =
            ((uint32_t)((int32_t)telemetria->temperatura_x10 + 100) *
             100U + 475U) / 950U;
        barra_velocidad =
            ((uint32_t)telemetria->velocidad_x10 * 100U + 150U) / 300U;

        /* La telemetria llega en nudos; la interfaz principal muestra km/h. */
        velocidad_kmh_x10 =
            ((uint32_t)telemetria->velocidad_x10 * 1852U + 500U) / 1000U;

        rumbo_entero = (uint32_t)telemetria->rumbo_x10 / 10U;
        yaw_entero = (uint32_t)telemetria->yaw_x10 / 10U;
        estado_gps =
            (telemetria->calidad_gps != 0U) ? "GPS OK" : "SIN FIX";

        if (((telemetria->pitch_x10 >= -30) &&
             (telemetria->pitch_x10 <= 30)) &&
            ((telemetria->roll_x10 >= -30) &&
             (telemetria->roll_x10 <= 30)))
        {
            estado_inclinacion = "NIVEL";
        }
        else
        {
            estado_inclinacion = "INCLINADO";
        }

        if ((NEXTION_AgregarDecimalX10(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "tBateria", telemetria->voltaje_x10, " V") == 0U) ||
            (NEXTION_AgregarComando(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "jbat.val=%lu", (unsigned long)barra_bateria) == 0U) ||
            (NEXTION_AgregarDecimalX10(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "tCorr", telemetria->corriente_x10, " A") == 0U) ||
            (NEXTION_AgregarComando(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "jCorr.val=%lu", (unsigned long)barra_corriente) == 0U) ||
            (NEXTION_AgregarDecimalX10(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "tTemperatura", telemetria->temperatura_x10, " C") == 0U) ||
            (NEXTION_AgregarComando(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "barTemp.val=%lu", (unsigned long)barra_temperatura) == 0U) ||
            (NEXTION_AgregarTextoSeguro(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "tHumedad", (telemetria->inundacion != 0U) ? "AGUA" : "SECO") == 0U) ||
            (NEXTION_AgregarComando(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "jHumedad.val=%u",
                (telemetria->inundacion != 0U) ? 100U : 0U) == 0U))
        {
            return HAL_ERROR;
        }

        resultado = snprintf(
            texto, sizeof(texto), "SAT %u",
            (unsigned int)telemetria->satelites);
        if ((resultado < 0) || ((size_t)resultado >= sizeof(texto)) ||
            (NEXTION_AgregarTextoSeguro(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "gps_satelites", texto) == 0U) ||
            (NEXTION_AgregarTextoSeguro(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "lblGPS", estado_gps) == 0U))
        {
            return HAL_ERROR;
        }

        resultado = snprintf(
            texto, sizeof(texto), "%s %c",
            telemetria->latitud, telemetria->hemisferio_latitud);
        if ((resultado < 0) || ((size_t)resultado >= sizeof(texto)) ||
            (NEXTION_AgregarTextoSeguro(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "tLatitud", texto) == 0U))
        {
            return HAL_ERROR;
        }

        resultado = snprintf(
            texto, sizeof(texto), "%s %c",
            telemetria->longitud, telemetria->hemisferio_longitud);
        if ((resultado < 0) || ((size_t)resultado >= sizeof(texto)) ||
            (NEXTION_AgregarTextoSeguro(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "tLongitud", texto) == 0U) ||
            (NEXTION_AgregarDecimalX10(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "tvel", (int32_t)velocidad_kmh_x10, " km/h") == 0U) ||
            (NEXTION_AgregarComando(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "barVEL.val=%lu", (unsigned long)barra_velocidad) == 0U))
        {
            return HAL_ERROR;
        }

        resultado = snprintf(
            texto, sizeof(texto), "%03lu", (unsigned long)rumbo_entero);
        if ((resultado < 0) || ((size_t)resultado >= sizeof(texto)) ||
            (NEXTION_AgregarTextoSeguro(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "tRumbo", texto) == 0U) ||
            (NEXTION_AgregarComando(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "zRumbo.val=%lu", (unsigned long)rumbo_entero) == 0U) ||
            (NEXTION_AgregarDecimalX10(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "tYaw", telemetria->yaw_x10, "") == 0U) ||
            (NEXTION_AgregarComando(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "zYaw.val=%lu", (unsigned long)yaw_entero) == 0U) ||
            (NEXTION_AgregarDecimalX10(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "tPitch", telemetria->pitch_x10, "") == 0U) ||
            (NEXTION_AgregarDecimalX10(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "tRoll", telemetria->roll_x10, "") == 0U) ||
            (NEXTION_AgregarTextoSeguro(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "tInclinacion", estado_inclinacion) == 0U))
        {
            return HAL_ERROR;
        }

        if (NEXTION_FormatearUTC(
                telemetria->utc, texto, sizeof(texto)) == 0U)
        {
            strcpy(texto, "--:--:--");
        }

        if (NEXTION_AgregarTextoSeguro(
                nextion_buffer_tx, sizeof(nextion_buffer_tx), &utilizados,
                "page0.tHora", texto) == 0U)
        {
            return HAL_ERROR;
        }
    }

    /*
     * La actualizacion completa se envia directamente por USART1 con tiempo
     * limitado. Asi no depende del callback DMA TX de Nextion. Un error o
     * timeout solo descarta esta actualizacion; UART5 sigue independiente.
     */
    return HAL_UART_Transmit_DMA(
        huart,
        nextion_buffer_tx,
        (uint16_t)utilizados);
#endif
}
