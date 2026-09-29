/*
 * trama_usv.c
 *
 * Librería encargada de construir, verificar y decodificar
 * las tramas de comunicación del proyecto USV.
 *
 * Se utilizan dos identificadores:
 *
 * $PUSVU = Comandos enviados desde el control de tierra al bote.
 * $PUSVD = Telemetría enviada desde el bote al control de tierra.
 *
 * Los valores decimales se manejan como enteros multiplicados
 * por 10 para evitar errores al transmitir números float.
 *
 * Ejemplos:
 *
 * 75.5 %  se almacena como 755.
 * -45.0°  se almacena como -450.
 * 12.3 V  se almacena como 123.
 */

/*
 * ACTUALIZACION VIGENTE DEL COMANDO $PUSVU:
 *
 * La escala x10 se conserva solamente en los campos decimales de
 * la telemetria $PUSVD. El comando utiliza enteros sin escala x10.
 *
 * La trama enviada desde tierra contiene cinco valores:
 *
 * $PUSVU,babor,estribor,camara,bomba,parada\r\n
 *
 * - Babor y estribor: -100 a +100. Negativo significa atras.
 * - Camara: -90 a +90 grados.
 * - Bomba y parada: 0 o 1.
 *
 * Las luces y la reconexion quedan reservadas internamente para una
 * version futura. La falla de direccion se atiende dentro del control
 * de tierra llevando ambos motores a cero, pero no se transmite.
 * La trama de comandos no incluye secuencia ni CRC.
 *
 * La telemetria $PUSVD conserva su formato, escala x10 y CRC-8.
 */

#include "trama_usv.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>


/* ================================================================
 * FUNCIONES INTERNAS
 * ================================================================
 */


/* Convierte un carácter hexadecimal en su valor numérico.
 *
 * Retorna -1 si el carácter no pertenece a 0-9, A-F o a-f.
 */
static int USV_ValorHexadecimal(char caracter)
{
    if ((caracter >= '0') && (caracter <= '9'))
    {
        return caracter - '0';
    }

    if ((caracter >= 'A') && (caracter <= 'F'))
    {
        return caracter - 'A' + 10;
    }

    if ((caracter >= 'a') && (caracter <= 'f'))
    {
        return caracter - 'a' + 10;
    }

    return -1;
}


/* Comprueba que una cadena pueda incluirse de forma segura
 * dentro de una trama.
 *
 * No permite caracteres utilizados para separar o finalizar
 * los campos.
 */
static uint8_t USV_CadenaSegura(const char *cadena)
{
    size_t indice;

    if ((cadena == NULL) || (cadena[0] == '\0'))
    {
        return 0U;
    }

    for (indice = 0U; cadena[indice] != '\0'; indice++)
    {
        if ((cadena[indice] == ',') ||
            (cadena[indice] == '*') ||
            (cadena[indice] == '\r') ||
            (cadena[indice] == '\n'))
        {
            return 0U;
        }
    }

    return 1U;
}


/* Convierte un valor multiplicado por 10 en texto.
 *
 * Ejemplos:
 *
 * 755  produce "75.5".
 * -450 produce "-45.0".
 * 123  produce "12.3".
 */
/* Esta conversion se conserva para la telemetria, no para el comando. */
static uint8_t USV_FormatearDecimalX10(
        int32_t valor_x10,
        char *destino,
        size_t capacidad)
{
    int64_t magnitud;
    const char *signo;
    int resultado;

    if ((destino == NULL) || (capacidad == 0U))
    {
        return 0U;
    }

    if (valor_x10 < 0)
    {
        signo = "-";
        magnitud = -(int64_t)valor_x10;
    }
    else
    {
        signo = "";
        magnitud = (int64_t)valor_x10;
    }

    resultado = snprintf(
        destino,
        capacidad,
        "%s%lld.%01lld",
        signo,
        (long long)(magnitud / 10),
        (long long)(magnitud % 10));

    if ((resultado < 0) ||
        ((size_t)resultado >= capacidad))
    {
        return 0U;
    }

    return 1U;
}


/* Convierte un texto decimal en un entero multiplicado por 10.
 *
 * Esta función no utiliza atof() ni variables float.
 *
 * Ejemplos:
 *
 * "75.5"  produce 755.
 * "-45.0" produce -450.
 * "12"    produce 120.
 */
/* Esta conversion se conserva para recibir la telemetria. */
static uint8_t USV_LeerDecimalX10(
        const char *texto,
        int32_t *resultado)
{
    uint8_t negativo = 0U;
    uint8_t existen_digitos = 0U;
    uint8_t parte_decimal = 0U;
    uint64_t parte_entera = 0U;
    int64_t valor_final;
    size_t posicion = 0U;

    if ((texto == NULL) ||
        (resultado == NULL) ||
        (texto[0] == '\0'))
    {
        return 0U;
    }

    /* Lee el signo si está presente. */
    if (texto[posicion] == '-')
    {
        negativo = 1U;
        posicion++;
    }
    else if (texto[posicion] == '+')
    {
        posicion++;
    }

    /* Lee la parte entera. */
    while ((texto[posicion] >= '0') &&
           (texto[posicion] <= '9'))
    {
        existen_digitos = 1U;

        parte_entera =
            (parte_entera * 10U) +
            (uint64_t)(texto[posicion] - '0');

        /* Evita un desbordamiento al multiplicar por 10. */
        if (parte_entera > 214748364U)
        {
            return 0U;
        }

        posicion++;
    }

    if (existen_digitos == 0U)
    {
        return 0U;
    }

    /* Lee la parte decimal si existe. */
    if (texto[posicion] == '.')
    {
        posicion++;

        if ((texto[posicion] < '0') ||
            (texto[posicion] > '9'))
        {
            return 0U;
        }

        /* Se conserva el primer decimal. */
        parte_decimal =
            (uint8_t)(texto[posicion] - '0');

        posicion++;

        /* Se aceptan decimales adicionales, pero solamente
         * se conserva el primero porque trabajamos en x10.
         */
        while ((texto[posicion] >= '0') &&
               (texto[posicion] <= '9'))
        {
            posicion++;
        }
    }

    /* No debe existir ningún otro carácter. */
    if (texto[posicion] != '\0')
    {
        return 0U;
    }

    valor_final =
        (int64_t)(parte_entera * 10U) +
        parte_decimal;

    if (negativo != 0U)
    {
        valor_final = -valor_final;
    }

    if ((valor_final > INT32_MAX) ||
        (valor_final < INT32_MIN))
    {
        return 0U;
    }

    *resultado = (int32_t)valor_final;

    return 1U;
}


/* Convierte un campo de texto en un entero sin signo. */
static uint8_t USV_LeerEnteroSinSigno(
        const char *texto,
        uint32_t *resultado)
{
    uint32_t valor = 0U;
    uint32_t digito;
    size_t posicion;

    if ((texto == NULL) ||
        (resultado == NULL) ||
        (texto[0] == '\0') ||
        (texto[0] == '-') ||
        (texto[0] == '+'))
    {
        return 0U;
    }

    /* Lee solamente digitos: no acepta espacios, etiquetas ni decimales. */
    for (posicion = 0U; texto[posicion] != '\0'; posicion++)
    {
        if ((texto[posicion] < '0') || (texto[posicion] > '9'))
        {
            return 0U;
        }

        digito = (uint32_t)(texto[posicion] - '0');

        /* Comprueba el limite antes de multiplicar; funciona tambien en STM32. */
        if (valor > ((UINT32_MAX - digito) / 10U))
        {
            return 0U;
        }

        valor = (valor * 10U) + digito;
    }

    *resultado = valor;

    return 1U;
}


/* Lee un entero con signo para la direccion del bote y el angulo de camara. */
static uint8_t USV_LeerEnteroConSigno(
        const char *texto,
        int32_t *resultado)
{
    uint32_t magnitud;
    uint8_t negativo = 0U;

    if ((texto == NULL) || (resultado == NULL))
    {
        return 0U;
    }

    /* El signo es opcional; los siguientes caracteres deben ser digitos. */
    if (*texto == '-')
    {
        negativo = 1U;
        texto++;
    }
    else if (*texto == '+')
    {
        texto++;
    }

    if (USV_LeerEnteroSinSigno(texto, &magnitud) == 0U)
    {
        return 0U;
    }

    if (negativo != 0U)
    {
        if (magnitud > ((uint32_t)INT32_MAX + 1U))
        {
            return 0U;
        }

        /* INT32_MIN se trata aparte para no desbordar al cambiar el signo. */
        *resultado = (magnitud == ((uint32_t)INT32_MAX + 1U))
                   ? INT32_MIN : -(int32_t)magnitud;
    }
    else
    {
        if (magnitud > (uint32_t)INT32_MAX)
        {
            return 0U;
        }

        *resultado = (int32_t)magnitud;
    }

    return 1U;
}


/* Separa el cuerpo de una trama utilizando las comas.
 *
 * Cada coma se sustituye por '\0' y se guardan las
 * direcciones donde comienza cada campo.
 */
static size_t USV_SepararCampos(
        char *texto,
        char **campos,
        size_t capacidad)
{
    char *posicion;
    size_t cantidad = 0U;

    if ((texto == NULL) ||
        (campos == NULL) ||
        (capacidad == 0U))
    {
        return 0U;
    }

    campos[cantidad] = texto;
    cantidad++;

    for (posicion = texto;
         *posicion != '\0';
         posicion++)
    {
        if (*posicion == ',')
        {
            *posicion = '\0';

            if (cantidad >= capacidad)
            {
                return 0U;
            }

            campos[cantidad] = posicion + 1;
            cantidad++;
        }
    }

    return cantidad;
}


/* Agrega el CRC y la terminación a una trama.
 *
 * La terminación queda:
 *
 * *HH<CR><LF>
 */
static size_t USV_CerrarTrama(
        char *destino,
        size_t capacidad)
{
    size_t longitud;
    uint8_t crc;
    int resultado;

    if ((destino == NULL) ||
        (capacidad == 0U) ||
        (destino[0] != '$'))
    {
        return 0U;
    }

    longitud = strlen(destino);

    /* Se necesitan cinco caracteres:
     *
     * *
     * Dos caracteres del CRC.
     * \r
     * \n
     *
     * Además del carácter nulo final.
     */
    if ((longitud + 6U) > capacidad)
    {
        /* No deja una trama parcial disponible para un envio accidental. */
        destino[0] = '\0';
        return 0U;
    }

    /* El carácter '$' no se incluye en el cálculo. */
    crc = USV_CRC8(
        (const uint8_t *)&destino[1],
        longitud - 1U);

    resultado = snprintf(
        &destino[longitud],
        capacidad - longitud,
        "*%02X\r\n",
        (unsigned int)crc);

    if (resultado != 5)
    {
        destino[0] = '\0';
        return 0U;
    }

    return longitud + 5U;
}


/* Copia la parte de la trama anterior al asterisco.
 *
 * Primero comprueba el CRC para evitar procesar datos dañados.
 */
static uint8_t USV_CopiarCuerpo(
        const char *trama,
        char *destino,
        size_t capacidad)
{
    const char *asterisco;
    size_t longitud;

    if ((trama == NULL) ||
        (destino == NULL) ||
        (capacidad == 0U))
    {
        return 0U;
    }

    if (USV_VerificarTrama(trama) == 0U)
    {
        return 0U;
    }

    asterisco = strchr(trama, '*');

    if (asterisco == NULL)
    {
        return 0U;
    }

    longitud = (size_t)(asterisco - trama);

    if ((longitud + 1U) > capacidad)
    {
        return 0U;
    }

    memcpy(destino, trama, longitud);
    destino[longitud] = '\0';

    return 1U;
}


/* ================================================================
 * CRC DE LAS TRAMAS
 * ================================================================
 */


/* Calcula un CRC-8 utilizando el polinomio 0x07.
 *
 * El control de tierra y el microcontrolador del bote
 * deben utilizar exactamente este mismo algoritmo.
 */
uint8_t USV_CRC8(
        const uint8_t *datos,
        size_t longitud)
{
    uint8_t crc = 0U;
    uint8_t bit;
    size_t indice;

    if ((datos == NULL) && (longitud > 0U))
    {
        return 0U;
    }

    for (indice = 0U; indice < longitud; indice++)
    {
        crc ^= datos[indice];

        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x80U) != 0U)
            {
                crc =
                    (uint8_t)((crc << 1U) ^ 0x07U);
            }
            else
            {
                crc <<= 1U;
            }
        }
    }

    return crc;
}


/* Verifica el formato general y el CRC de una trama.
 *
 * Retorna:
 *
 * 1 = Trama correcta.
 * 0 = Trama inválida o dañada.
 */
uint8_t USV_VerificarTrama(
        const char *trama)
{
    const char *asterisco;
    const char *final;
    int hexadecimal_alto;
    int hexadecimal_bajo;
    uint8_t crc_recibido;
    uint8_t crc_calculado;
    size_t longitud_crc;

    if ((trama == NULL) || (trama[0] != '$'))
    {
        return 0U;
    }

    asterisco = strchr(trama, '*');

    if (asterisco == NULL)
    {
        return 0U;
    }

    /* Después del asterisco deben existir dos
     * caracteres hexadecimales.
     */
    if ((asterisco[1] == '\0') ||
        (asterisco[2] == '\0'))
    {
        return 0U;
    }

    hexadecimal_alto =
        USV_ValorHexadecimal(asterisco[1]);

    hexadecimal_bajo =
        USV_ValorHexadecimal(asterisco[2]);

    if ((hexadecimal_alto < 0) ||
        (hexadecimal_bajo < 0))
    {
        return 0U;
    }

    /* Comprueba la terminación después del CRC. */
    final = &asterisco[3];

    if (*final == '\r')
    {
        final++;
    }

    if (*final == '\n')
    {
        final++;
    }

    if (*final != '\0')
    {
        return 0U;
    }

    crc_recibido =
        (uint8_t)(
            (hexadecimal_alto << 4) |
            hexadecimal_bajo);

    longitud_crc =
        (size_t)(asterisco - &trama[1]);

    crc_calculado = USV_CRC8(
        (const uint8_t *)&trama[1],
        longitud_crc);

    return (crc_recibido == crc_calculado) ?
           1U : 0U;
}


/* ================================================================
 * TRAMA UPLINK: CONTROL DE TIERRA HACIA EL BOTE
 * ================================================================
 */


/* Convierte una magnitud de potencia y sus dos ordenes fisicas en
 * una sola potencia con signo para la trama compacta.
 */
static int16_t USV_PotenciaConSigno(
        uint16_t potencia,
        uint8_t avante,
        uint8_t atras)
{
    if ((avante != 0U) && (atras == 0U))
    {
        return (int16_t)potencia;
    }

    if ((avante == 0U) && (atras != 0U))
    {
        return -(int16_t)potencia;
    }

    /* Sin direccion o con una combinacion contradictoria: motor detenido. */
    return 0;
}


/* Construye la trama compacta de comandos $PUSVU sin CRC. */
size_t USV_ConstruirComando(
        char *destino,
        size_t capacidad,
        const USV_Comando *comando)
{
    int resultado;
    int16_t motor_babor;
    int16_t motor_estribor;

    if ((destino == NULL) ||
        (comando == NULL) ||
        (capacidad == 0U))
    {
        return 0U;
    }

    /* Una entrada invalida no debe dejar una orden anterior en este destino. */
    destino[0] = '\0';

    /* Verifica las unidades internas vigentes, sin escala x10. */
    if ((comando->potencia_global > 100U) ||
        (comando->direccion < -100) ||
        (comando->direccion > 100) ||
        (comando->potencia_babor > 100U) ||
        (comando->potencia_estribor > 100U) ||
        (comando->camara < -90) ||
        (comando->camara > 90))
    {
        return 0U;
    }

    /* Verifica los estados digitales. */
    if ((comando->babor_avante > 1U) ||
        (comando->babor_atras > 1U) ||
        (comando->estribor_avante > 1U) ||
        (comando->estribor_atras > 1U) ||
        (comando->luces > 1U) ||
        (comando->bomba > 1U) ||
        (comando->reconexion > 1U) ||
        (comando->modo_joystick > 1U) ||
        (comando->parada > 1U) ||
        (comando->falla_direccion > 1U))
    {
        return 0U;
    }

    /*
     * La parada y la falla tienen prioridad. El main.c ya lleva las
     * potencias a cero, pero esta segunda barrera evita que una estructura
     * inconsistente transmita movimiento durante una condicion insegura.
     */
    if ((comando->parada != 0U) ||
        (comando->falla_direccion != 0U))
    {
        motor_babor = 0;
        motor_estribor = 0;
    }
    else
    {
        motor_babor = USV_PotenciaConSigno(
            comando->potencia_babor,
            comando->babor_avante,
            comando->babor_atras);

        motor_estribor = USV_PotenciaConSigno(
            comando->potencia_estribor,
            comando->estribor_avante,
            comando->estribor_atras);
    }

    /*
     * Las luces y la reconexion siguen disponibles internamente, pero se
     * dejan fuera de la trama para una version futura. La falla de direccion
     * tambien es interna: ya llevo ambos motores a cero en el bloque anterior.
     * Cualquier incorporacion futura debe actualizar tambien el receptor.
     */
    resultado = snprintf(
        destino,
        capacidad,
        "$PUSVU,%d,%d,%d,%u,%u\r\n",
        (int)motor_babor,
        (int)motor_estribor,
        (int)comando->camara,
        (unsigned int)comando->bomba,
        (unsigned int)comando->parada);

    if ((resultado < 0) ||
        ((size_t)resultado >= capacidad))
    {
        destino[0] = '\0';
        return 0U;
    }

    return (size_t)resultado;
}


/* Decodifica una trama de comandos $PUSVU. */
uint8_t USV_LeerComando(
        const char *trama,
        USV_Comando *comando)
{
    char cuerpo[USV_TRAMA_MAXIMA];
    char *campos[USV_MAXIMO_CAMPOS];
    size_t longitud;
    size_t longitud_cuerpo;
    int32_t motor_babor;
    int32_t motor_estribor;
    int32_t camara;
    uint32_t bomba;
    uint32_t parada;
    size_t campos_leidos;

    if ((trama == NULL) || (comando == NULL))
    {
        return 0U;
    }

    /* El comando vigente debe terminar en CR/LF y no contiene CRC. */
    longitud = strlen(trama);

    if ((longitud < 2U) ||
        (trama[longitud - 2U] != '\r') ||
        (trama[longitud - 1U] != '\n') ||
        (strchr(trama, '*') != NULL))
    {
        return 0U;
    }

    longitud_cuerpo = longitud - 2U;

    if ((longitud_cuerpo + 1U) > sizeof(cuerpo))
    {
        return 0U;
    }

    memcpy(cuerpo, trama, longitud_cuerpo);
    cuerpo[longitud_cuerpo] = '\0';

    /* Separa las posiciones sin buscar etiquetas y sin utilizar sscanf(). */
    campos_leidos = USV_SepararCampos(
        cuerpo,
        campos,
        USV_MAXIMO_CAMPOS);

    /* Debe contener el identificador y exactamente cinco valores. */
    if ((campos_leidos != 6U) ||
        (strcmp(campos[0], "$PUSVU") != 0))
    {
        return 0U;
    }

    if ((USV_LeerEnteroConSigno(campos[1], &motor_babor) == 0U) ||
        (USV_LeerEnteroConSigno(campos[2], &motor_estribor) == 0U) ||
        (USV_LeerEnteroConSigno(campos[3], &camara) == 0U) ||
        (USV_LeerEnteroSinSigno(campos[4], &bomba) == 0U) ||
        (USV_LeerEnteroSinSigno(campos[5], &parada) == 0U))
    {
        return 0U;
    }

    if ((motor_babor < -100) ||
        (motor_babor > 100) ||
        (motor_estribor < -100) ||
        (motor_estribor > 100) ||
        (camara < -90) ||
        (camara > 90) ||
        (bomba > 1U) ||
        (parada > 1U))
    {
        return 0U;
    }

    /* Parte de una estructura limpia: los campos omitidos quedan en cero. */
    memset(comando, 0, sizeof(*comando));

    comando->potencia_babor =
        (uint16_t)((motor_babor < 0) ? -motor_babor : motor_babor);
    comando->potencia_estribor =
        (uint16_t)((motor_estribor < 0) ? -motor_estribor : motor_estribor);
    comando->camara = (int16_t)camara;
    comando->bomba = (uint8_t)bomba;
    comando->parada = (uint8_t)parada;

    if (motor_babor > 0)
    {
        comando->babor_avante = 1U;
    }
    else if (motor_babor < 0)
    {
        comando->babor_atras = 1U;
    }

    if (motor_estribor > 0)
    {
        comando->estribor_avante = 1U;
    }
    else if (motor_estribor < 0)
    {
        comando->estribor_atras = 1U;
    }

    /* El receptor futuro debe aplicar siempre la parada como prioridad. */
    if (comando->parada != 0U)
    {
        comando->potencia_babor = 0U;
        comando->potencia_estribor = 0U;
        comando->babor_avante = 0U;
        comando->babor_atras = 0U;
        comando->estribor_avante = 0U;
        comando->estribor_atras = 0U;
    }

    return 1U;
}


/* ================================================================
 * TRAMA DOWNLINK: BOTE HACIA EL CONTROL DE TIERRA
 * ================================================================
 */


/* Construye la trama de telemetría $PUSVD siguiendo
 * el orden de la Trama Versión 3.
 */
size_t USV_ConstruirTelemetria(
        char *destino,
        size_t capacidad,
        const USV_Telemetria *telemetria)
{
    char hdop[16];
    char altitud[16];
    char velocidad[16];
    char rumbo[16];
    char yaw[16];
    char pitch[16];
    char roll[16];
    char temperatura[16];
    char voltaje[16];
    char corriente[16];

    int resultado;

    if ((destino == NULL) ||
        (telemetria == NULL) ||
        (capacidad == 0U))
    {
        return 0U;
    }

    /* Verifica las cadenas GPS. */
    if ((USV_CadenaSegura(telemetria->utc) == 0U) ||
        (USV_CadenaSegura(telemetria->latitud) == 0U) ||
        (USV_CadenaSegura(telemetria->longitud) == 0U) ||
        (strlen(telemetria->utc) >= USV_TAMANO_UTC) ||
        (strlen(telemetria->latitud) >=
         USV_TAMANO_COORDENADA) ||
        (strlen(telemetria->longitud) >=
         USV_TAMANO_COORDENADA))
    {
        return 0U;
    }

    /* Verifica los hemisferios GPS. */
    if (((telemetria->hemisferio_latitud != 'N') &&
         (telemetria->hemisferio_latitud != 'S')) ||
        ((telemetria->hemisferio_longitud != 'E') &&
         (telemetria->hemisferio_longitud != 'W')))
    {
        return 0U;
    }

    /* Verifica los rangos de la Trama Versión 3. */
    if ((telemetria->calidad_gps > 8U) ||
        (telemetria->satelites > 24U) ||
        (telemetria->hdop_x10 < 5U) ||
        (telemetria->hdop_x10 > 500U) ||
        (telemetria->altitud_x10 < -100) ||
        (telemetria->altitud_x10 > 99990) ||
        (telemetria->velocidad_x10 > 300U) ||
        (telemetria->rumbo_x10 > 3599U) ||
        (telemetria->yaw_x10 > 3599U) ||
        (telemetria->pitch_x10 < -900) ||
        (telemetria->pitch_x10 > 900) ||
        (telemetria->roll_x10 < -900) ||
        (telemetria->roll_x10 > 900) ||
        (telemetria->temperatura_x10 < -100) ||
        (telemetria->temperatura_x10 > 850) ||
        (telemetria->inundacion > 1U) ||
        (telemetria->voltaje_x10 > 140U) ||
        (telemetria->corriente_x10 > 1000U) ||
        (telemetria->luces > 1U) ||
        (telemetria->modo_solicitado > 2U))
    {
        return 0U;
    }

    /* Convierte los valores x10 en texto decimal. */
    if ((USV_FormatearDecimalX10(
            telemetria->hdop_x10,
            hdop,
            sizeof(hdop)) == 0U) ||
        (USV_FormatearDecimalX10(
            telemetria->altitud_x10,
            altitud,
            sizeof(altitud)) == 0U) ||
        (USV_FormatearDecimalX10(
            telemetria->velocidad_x10,
            velocidad,
            sizeof(velocidad)) == 0U) ||
        (USV_FormatearDecimalX10(
            telemetria->rumbo_x10,
            rumbo,
            sizeof(rumbo)) == 0U) ||
        (USV_FormatearDecimalX10(
            telemetria->yaw_x10,
            yaw,
            sizeof(yaw)) == 0U) ||
        (USV_FormatearDecimalX10(
            telemetria->pitch_x10,
            pitch,
            sizeof(pitch)) == 0U) ||
        (USV_FormatearDecimalX10(
            telemetria->roll_x10,
            roll,
            sizeof(roll)) == 0U) ||
        (USV_FormatearDecimalX10(
            telemetria->temperatura_x10,
            temperatura,
            sizeof(temperatura)) == 0U) ||
        (USV_FormatearDecimalX10(
            telemetria->voltaje_x10,
            voltaje,
            sizeof(voltaje)) == 0U) ||
        (USV_FormatearDecimalX10(
            telemetria->corriente_x10,
            corriente,
            sizeof(corriente)) == 0U))
    {
        return 0U;
    }

    /* Construye la trama siguiendo el orden oficial. */
    resultado = snprintf(
        destino,
        capacidad,
        "$PUSVD,"
        "%s,%s,%c,%s,%c,"
        "%u,%u,%s,%s,M,"
        "%s,N,%s,T,"
        "%s,T,%s,P,%s,R,"
        "%s,C,%u,%s,V,%s,C,"
        "%u,STATE,%u",
        telemetria->utc,
        telemetria->latitud,
        telemetria->hemisferio_latitud,
        telemetria->longitud,
        telemetria->hemisferio_longitud,
        (unsigned int)telemetria->calidad_gps,
        (unsigned int)telemetria->satelites,
        hdop,
        altitud,
        velocidad,
        rumbo,
        yaw,
        pitch,
        roll,
        temperatura,
        (unsigned int)telemetria->inundacion,
        voltaje,
        corriente,
        (unsigned int)telemetria->luces,
        (unsigned int)telemetria->modo_solicitado);

    if ((resultado < 0) ||
        ((size_t)resultado >= capacidad))
    {
        return 0U;
    }

    return USV_CerrarTrama(destino, capacidad);
}


/* Decodifica una trama de telemetría $PUSVD. */
uint8_t USV_LeerTelemetria(
        const char *trama,
        USV_Telemetria *telemetria)
{
    char cuerpo[USV_TRAMA_MAXIMA];
    char *campos[USV_MAXIMO_CAMPOS];

    size_t cantidad_campos;
    uint32_t entero;
    int32_t decimal;

    if ((trama == NULL) || (telemetria == NULL))
    {
        return 0U;
    }

    if (USV_CopiarCuerpo(
            trama,
            cuerpo,
            sizeof(cuerpo)) == 0U)
    {
        return 0U;
    }

    cantidad_campos = USV_SepararCampos(
        cuerpo,
        campos,
        USV_MAXIMO_CAMPOS);

    /* $PUSVD más treinta campos de información. */
    if (cantidad_campos != 31U)
    {
        return 0U;
    }

    if (strcmp(campos[0], "$PUSVD") != 0)
    {
        return 0U;
    }

    /* Verifica cadenas y tamaños antes de copiarlas. */
    if ((USV_CadenaSegura(campos[1]) == 0U) ||
        (USV_CadenaSegura(campos[2]) == 0U) ||
        (USV_CadenaSegura(campos[4]) == 0U) ||
        (strlen(campos[1]) >= USV_TAMANO_UTC) ||
        (strlen(campos[2]) >=
         USV_TAMANO_COORDENADA) ||
        (strlen(campos[4]) >=
         USV_TAMANO_COORDENADA))
    {
        return 0U;
    }

    /* Verifica hemisferios y unidades. */
    if ((strlen(campos[3]) != 1U) ||
        ((campos[3][0] != 'N') &&
         (campos[3][0] != 'S')) ||
        (strlen(campos[5]) != 1U) ||
        ((campos[5][0] != 'E') &&
         (campos[5][0] != 'W')) ||
        (strcmp(campos[10], "M") != 0) ||
        (strcmp(campos[12], "N") != 0) ||
        (strcmp(campos[14], "T") != 0) ||
        (strcmp(campos[16], "T") != 0) ||
        (strcmp(campos[18], "P") != 0) ||
        (strcmp(campos[20], "R") != 0) ||
        (strcmp(campos[22], "C") != 0) ||
        (strcmp(campos[25], "V") != 0) ||
        (strcmp(campos[27], "C") != 0) ||
        (strcmp(campos[29], "STATE") != 0))
    {
        return 0U;
    }

    /* Calidad GPS. */
    if ((USV_LeerEnteroSinSigno(
            campos[6], &entero) == 0U) ||
        (entero > 8U))
    {
        return 0U;
    }

    telemetria->calidad_gps = (uint8_t)entero;

    /* Cantidad de satélites. */
    if ((USV_LeerEnteroSinSigno(
            campos[7], &entero) == 0U) ||
        (entero > 24U))
    {
        return 0U;
    }

    telemetria->satelites = (uint8_t)entero;

    /* HDOP. */
    if ((USV_LeerDecimalX10(
            campos[8], &decimal) == 0U) ||
        (decimal < 5) ||
        (decimal > 500))
    {
        return 0U;
    }

    telemetria->hdop_x10 = (uint16_t)decimal;

    /* Altitud. */
    if ((USV_LeerDecimalX10(
            campos[9], &decimal) == 0U) ||
        (decimal < -100) ||
        (decimal > 99990))
    {
        return 0U;
    }

    telemetria->altitud_x10 = decimal;

    /* Velocidad SOG. */
    if ((USV_LeerDecimalX10(
            campos[11], &decimal) == 0U) ||
        (decimal < 0) ||
        (decimal > 300))
    {
        return 0U;
    }

    telemetria->velocidad_x10 =
        (uint16_t)decimal;

    /* Rumbo COG. */
    if ((USV_LeerDecimalX10(
            campos[13], &decimal) == 0U) ||
        (decimal < 0) ||
        (decimal > 3599))
    {
        return 0U;
    }

    telemetria->rumbo_x10 =
        (uint16_t)decimal;

    /* Yaw o azimut. */
    if ((USV_LeerDecimalX10(
            campos[15], &decimal) == 0U) ||
        (decimal < 0) ||
        (decimal > 3599))
    {
        return 0U;
    }

    telemetria->yaw_x10 =
        (uint16_t)decimal;

    /* Pitch. */
    if ((USV_LeerDecimalX10(
            campos[17], &decimal) == 0U) ||
        (decimal < -900) ||
        (decimal > 900))
    {
        return 0U;
    }

    telemetria->pitch_x10 =
        (int16_t)decimal;

    /* Roll. */
    if ((USV_LeerDecimalX10(
            campos[19], &decimal) == 0U) ||
        (decimal < -900) ||
        (decimal > 900))
    {
        return 0U;
    }

    telemetria->roll_x10 =
        (int16_t)decimal;

    /* Temperatura. */
    if ((USV_LeerDecimalX10(
            campos[21], &decimal) == 0U) ||
        (decimal < -100) ||
        (decimal > 850))
    {
        return 0U;
    }

    telemetria->temperatura_x10 =
        (int16_t)decimal;

    /* Sensor de inundación. */
    if ((USV_LeerEnteroSinSigno(
            campos[23], &entero) == 0U) ||
        (entero > 1U))
    {
        return 0U;
    }

    telemetria->inundacion =
        (uint8_t)entero;

    /* Voltaje de batería. */
    if ((USV_LeerDecimalX10(
            campos[24], &decimal) == 0U) ||
        (decimal < 0) ||
        (decimal > 140))
    {
        return 0U;
    }

    telemetria->voltaje_x10 =
        (uint16_t)decimal;

    /* Corriente. */
    if ((USV_LeerDecimalX10(
            campos[26], &decimal) == 0U) ||
        (decimal < 0) ||
        (decimal > 1000))
    {
        return 0U;
    }

    telemetria->corriente_x10 =
        (uint16_t)decimal;

    /* Estado de luces. */
    if ((USV_LeerEnteroSinSigno(
            campos[28], &entero) == 0U) ||
        (entero > 1U))
    {
        return 0U;
    }

    telemetria->luces =
        (uint8_t)entero;

    /* Modo solicitado por el USV. */
    if ((USV_LeerEnteroSinSigno(
            campos[30], &entero) == 0U) ||
        (entero > 2U))
    {
        return 0U;
    }

    telemetria->modo_solicitado =
        (uint8_t)entero;

    /* Copia las cadenas GPS verificadas. */
    strcpy(telemetria->utc, campos[1]);
    strcpy(telemetria->latitud, campos[2]);
    strcpy(telemetria->longitud, campos[4]);

    telemetria->hemisferio_latitud =
        campos[3][0];

    telemetria->hemisferio_longitud =
        campos[5][0];

    return 1U;
}
