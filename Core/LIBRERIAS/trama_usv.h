/*
 * trama_usv.h
 *
 * Librería para construir, verificar y decodificar
 * las tramas de comunicación entre:
 *
 * - Control de tierra STM32H562RGT6
 * - Radio XBee
 * - Microcontrolador instalado en el bote
 *
 * La trama de comandos enviada desde tierra comienza con:
 *
 * $PUSVU
 *
 * La trama de telemetría recibida desde el bote comienza con:
 *
 * $PUSVD
 *
 * La trama de comandos $PUSVU se transmite sin CRC y termina con \r\n.
 * La telemetria $PUSVD conserva por ahora su formato anterior con CRC-8.
 */

#ifndef TRAMA_USV_H_
#define TRAMA_USV_H_

/* Incluye los tipos y definiciones generales del proyecto STM32. */
#include "main.h"

/* Define el tipo size_t utilizado para indicar tamaños de buffers. */
#include <stddef.h>

/* Define los tipos uint8_t, uint16_t, int16_t y uint32_t. */
#include <stdint.h>


/*
 * Tamaño máximo reservado para una trama completa.
 *
 * Incluye:
 *
 * - Identificador de la trama.
 * - Todos los campos.
 * - Separadores.
 * - Terminacion y, cuando corresponda, CRC.
 * - Retorno de carro y salto de línea.
 * - Carácter nulo final.
 */
#define USV_TRAMA_MAXIMA             256U


/*
 * Cantidad máxima de campos que puede separar el decodificador.
 *
 * La trama de telemetría $PUSVD utiliza actualmente 31 campos,
 * incluyendo el identificador de la trama.
 */
#define USV_MAXIMO_CAMPOS             32U


/*
 * Tamaño reservado para almacenar la hora UTC.
 *
 * Formato esperado:
 *
 * hhmmss.ss
 *
 * Ejemplo:
 *
 * 153025.50
 */
#define USV_TAMANO_UTC                16U


/*
 * Tamaño reservado para almacenar una coordenada GPS.
 *
 * La coordenada se conserva como texto para evitar perder
 * precisión durante la comunicación.
 */
#define USV_TAMANO_COORDENADA         20U


/*
 * FORMATO VIGENTE DE LA TRAMA DE COMANDOS $PUSVU
 *
 * Despues del identificador se transmiten cinco valores:
 *
 * Posicion 1: motor_babor,    de -100 a +100.
 * Posicion 2: motor_estribor, de -100 a +100.
 * Posicion 3: camara,         de  -90 a  +90 grados.
 * Posicion 4: bomba,          0 o 1.
 * Posicion 5: parada,         0 o 1.
 *
 * En los motores, un valor positivo significa avante, uno negativo
 * significa atras y cero significa motor detenido. El signo positivo
 * no se escribe para ahorrar un caracter.
 *
 * Ejemplo:
 *
 * $PUSVU,60,-30,15,1,0\r\n
 *
 * Esta trama no incluye secuencia, potencia general, direccion del
 * joystick, modo de control, ordenes separadas de avante/atras, luces,
 * reconexion, falla ni CRC. La reconexion queda reservada en el codigo
 * para una version futura. La falla sigue actuando internamente sobre la
 * seguridad, pero no ocupa espacio en el enlace hacia el bote.
 *
 * Al enviar, \r\n representa los bytes 0x0D y 0x0A. El caracter nulo
 * final solamente termina la cadena en memoria y no se transmite.
 */


/*
 * Estructura que contiene todas las órdenes enviadas
 * desde el control de tierra hacia el bote.
 *
 * Esta informacion se envia actualmente por UART5 mediante el radio.
 */
typedef struct
{
    /* Datos internos usados para calcular las potencias finales. */
    uint16_t potencia_global;     /* 0 a 100 %, no se transmite. */
    int16_t direccion;            /* -100 a 100, no se transmite. */
    uint16_t potencia_babor;      /* Magnitud final de 0 a 100 %. */
    uint16_t potencia_estribor;   /* Magnitud final de 0 a 100 %. */
    int16_t camara;               /* Angulo entero de -90 a +90 grados. */

    /*
     * Orden digital para mover el motor de babor hacia avante.
     *
     * 0 = desactivado.
     * 1 = activado.
     */
    uint8_t babor_avante;

    /*
     * Orden digital para mover el motor de babor hacia atrás.
     *
     * 0 = desactivado.
     * 1 = activado.
     */
    uint8_t babor_atras;

    /*
     * Orden digital para mover el motor de estribor hacia avante.
     *
     * 0 = desactivado.
     * 1 = activado.
     */
    uint8_t estribor_avante;

    /*
     * Orden digital para mover el motor de estribor hacia atrás.
     *
     * 0 = desactivado.
     * 1 = activado.
     */
    uint8_t estribor_atras;

    /* Se lee fisicamente, pero queda reservado para una version futura. */
    uint8_t luces;

    /*
     * Estado solicitado para la bomba de achique.
     *
     * 0 = bomba apagada.
     * 1 = bomba encendida.
     */
    uint8_t bomba;

    /*
     * Reservado para una version futura; actualmente no se transmite.
     * 0 = operacion normal; 1 = solicitud detectada internamente.
     */
    uint8_t reconexion;

    /* 0 = potenciometros independientes; 1 = joystick y general. */
    uint8_t modo_joystick;

    /*
     * Estado del botón de parada de emergencia.
     *
     * 0 = operación normal.
     * 1 = parada de emergencia activa.
     *
     * Cuando este campo vale 1, el bote debe llevar
     * las órdenes de movimiento a una condición segura.
     */
    uint8_t parada;

    /*
     * Indica que se detectó una combinación contradictoria
     * en las entradas de dirección.
     *
     * Ejemplos:
     *
     * - Babor avante y babor atrás activos simultáneamente.
     * - Estribor avante y estribor atrás activos simultáneamente.
     *
     * 0 = combinación válida.
     * 1 = falla de dirección detectada.
     *
     * Este estado detiene internamente la propulsion, pero no se transmite.
     */
    uint8_t falla_direccion;

} USV_Comando;


/*
 * Estructura que contiene la telemetría enviada
 * desde el bote hacia el control de tierra.
 *
 * Su organización sigue la Trama Versión 3.
 */
typedef struct
{
    /*
     * Hora UTC recibida del sistema GPS.
     *
     * Formato:
     *
     * hhmmss.ss
     */
    char utc[USV_TAMANO_UTC];

    /*
     * Latitud recibida del sistema GPS.
     *
     * Se conserva como texto para mantener la precisión.
     */
    char latitud[USV_TAMANO_COORDENADA];

    /*
     * Hemisferio correspondiente a la latitud.
     *
     * Valores esperados:
     *
     * N = Norte.
     * S = Sur.
     */
    char hemisferio_latitud;

    /*
     * Longitud recibida del sistema GPS.
     *
     * Se conserva como texto para mantener la precisión.
     */
    char longitud[USV_TAMANO_COORDENADA];

    /*
     * Hemisferio correspondiente a la longitud.
     *
     * Valores esperados:
     *
     * E = Este.
     * W = Oeste.
     */
    char hemisferio_longitud;

    /*
     * Calidad de la solución GPS.
     *
     * Rango admitido:
     *
     * 0 a 8.
     */
    uint8_t calidad_gps;

    /*
     * Cantidad de satélites utilizados por el GPS.
     *
     * Rango admitido:
     *
     * 0 a 24 satélites.
     */
    uint8_t satelites;

    /*
     * Dilución horizontal de precisión del GPS.
     *
     * Se almacena multiplicada por 10.
     *
     * Ejemplos:
     *
     * 5   = 0.5
     * 12  = 1.2
     * 500 = 50.0
     */
    uint16_t hdop_x10;

    /*
     * Altitud del bote en metros.
     *
     * Se almacena multiplicada por 10.
     *
     * Ejemplos:
     *
     * -100  = -10.0 metros.
     *  125  = 12.5 metros.
     */
    int32_t altitud_x10;

    /*
     * Velocidad sobre el terreno del bote.
     *
     * Se almacena multiplicada por 10.
     *
     * Rango definido por la Trama Versión 3:
     *
     * 0 a 30.0 nudos.
     */
    uint16_t velocidad_x10;

    /*
     * Rumbo sobre el terreno entregado por el GPS.
     *
     * Se almacena multiplicado por 10.
     *
     * Rango:
     *
     * 0 a 3599 = 0.0 a 359.9 grados.
     */
    uint16_t rumbo_x10;

    /*
     * Ángulo yaw o azimut entregado por la IMU.
     *
     * Se almacena multiplicado por 10.
     *
     * Rango:
     *
     * 0 a 3599 = 0.0 a 359.9 grados.
     */
    uint16_t yaw_x10;

    /*
     * Ángulo pitch entregado por la IMU.
     *
     * Se almacena multiplicado por 10.
     *
     * Rango:
     *
     * -900 a +900 = -90.0 a +90.0 grados.
     */
    int16_t pitch_x10;

    /*
     * Ángulo roll entregado por la IMU.
     *
     * Se almacena multiplicado por 10.
     *
     * Rango:
     *
     * -900 a +900 = -90.0 a +90.0 grados.
     */
    int16_t roll_x10;

    /*
     * Temperatura medida en el bote.
     *
     * Se almacena multiplicada por 10.
     *
     * Rango:
     *
     * -100 a 850 = -10.0 a 85.0 grados Celsius.
     */
    int16_t temperatura_x10;

    /*
     * Estado del sensor de inundación.
     *
     * 0 = compartimiento seco.
     * 1 = presencia de agua.
     */
    uint8_t inundacion;

    /*
     * Voltaje medido en la alimentación del bote.
     *
     * Se almacena multiplicado por 10.
     *
     * Rango:
     *
     * 0 a 140 = 0.0 a 14.0 voltios.
     */
    uint16_t voltaje_x10;

    /*
     * Corriente consumida por el sistema del bote.
     *
     * Se almacena multiplicada por 10.
     *
     * Rango:
     *
     * 0 a 1000 = 0.0 a 100.0 amperios.
     */
    uint16_t corriente_x10;

    /*
     * Estado real de las luces de navegación del bote.
     *
     * 0 = apagadas.
     * 1 = encendidas.
     */
    uint8_t luces;

    /*
     * Modo solicitado o reportado por el sistema.
     *
     * Valores definidos por la Trama Versión 3:
     *
     * 0 = reposo.
     * 1 = remoto.
     * 2 = reconectar.
     */
    uint8_t modo_solicitado;

} USV_Telemetria;


/*
 * Calcula el CRC-8 de un bloque de información.
 *
 * Se utiliza el polinomio:
 *
 * 0x07
 *
 * El cálculo se realiza sobre los caracteres ubicados
 * entre '$' y '*', sin incluir estos dos caracteres.
 *
 * Parámetros:
 *
 * datos    = dirección del bloque de datos.
 * longitud = cantidad de bytes que serán procesados.
 *
 * Retorna:
 *
 * Valor CRC-8 calculado.
 */
uint8_t USV_CRC8(
    const uint8_t *datos,
    size_t longitud);


/*
 * Verifica la estructura y el CRC de una trama recibida.
 *
 * Parámetro:
 *
 * trama = cadena de texto recibida por UART.
 *
 * Retorna:
 *
 * 1 = trama válida.
 * 0 = trama inválida.
 */
uint8_t USV_VerificarTrama(
    const char *trama);


/*
 * Construye la trama de comandos $PUSVU que será enviada
 * desde el control de tierra hacia el bote.
 *
 * Parámetros:
 *
 * destino         = buffer donde se guardará la trama.
 * tamano_destino  = tamaño total del buffer.
 * comando         = estructura con las órdenes del control.
 *
 * Retorna:
 *
 * Cantidad de caracteres escritos en el buffer.
 *
 * Retorna 0 si:
 *
 * - Algún puntero es inválido.
 * - El buffer es demasiado pequeño.
 * - Alguno de los valores está fuera de rango.
 */
size_t USV_ConstruirComando(
    char *destino,
    size_t tamano_destino,
    const USV_Comando *comando);


/*
 * Verifica y decodifica una trama de comandos $PUSVU.
 *
 * Esta función se utilizará principalmente en el
 * microcontrolador instalado en el bote.
 *
 * Parámetros:
 *
 * trama   = cadena recibida mediante el XBee.
 * comando = estructura donde se guardarán las órdenes.
 *
 * Retorna:
 *
 * 1 = trama recibida y decodificada correctamente.
 * 0 = formato invalido o valores fuera de rango.
 */
uint8_t USV_LeerComando(
    const char *trama,
    USV_Comando *comando);


/*
 * Construye la trama de telemetría $PUSVD de acuerdo
 * con el formato definido en la Trama Versión 3.
 *
 * Esta función se utilizará principalmente en el
 * microcontrolador instalado en el bote.
 *
 * Parámetros:
 *
 * destino         = buffer donde se guardará la trama.
 * tamano_destino  = tamaño total del buffer.
 * telemetria      = estructura con los datos del bote.
 *
 * Retorna:
 *
 * Cantidad de caracteres escritos en el buffer.
 *
 * Retorna 0 si:
 *
 * - Algún puntero es inválido.
 * - El buffer es demasiado pequeño.
 * - Alguno de los datos está fuera de rango.
 */
size_t USV_ConstruirTelemetria(
    char *destino,
    size_t tamano_destino,
    const USV_Telemetria *telemetria);


/*
 * Verifica y decodifica una trama de telemetría $PUSVD.
 *
 * Esta función será utilizada por el control de tierra
 * para recuperar la información enviada desde el bote.
 *
 * Parámetros:
 *
 * trama      = cadena recibida mediante el XBee.
 * telemetria = estructura donde se guardarán los datos.
 *
 * Retorna:
 *
 * 1 = trama recibida y decodificada correctamente.
 * 0 = trama inválida, CRC incorrecto o valores fuera de rango.
 */
uint8_t USV_LeerTelemetria(
    const char *trama,
    USV_Telemetria *telemetria);


#endif /* TRAMA_USV_H_ */
