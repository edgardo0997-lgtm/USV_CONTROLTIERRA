/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "gpdma.h"
#include "icache.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* Procedimientos de control integrados en main.c, sin control_usv.c ni control_usv.h. */
/* Librería suministrada para leer ADC mediante DMA. */
#include "adc_x.h"

/* Librería de mapeo utilizada para convertir los valores ADC. */
#include "MAP_.h"

/* Librería para construir y decodificar las tramas del USV. */
#include "trama_usv.h"

/* Librería de comunicación con la pantalla Nextion. */
#include "nextion.h"

/* memcpy() para retirar de forma segura una trama de la cola de recepcion. */
#include <string.h>

/* snprintf() para construir el reporte de diagnostico de Teleplot. */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * ACTUALIZACION DEL PINOUT CONFIRMADA EN usart.c Y main.h:
 *
 * - Trama del bote: UART5, PB13 TX y PB12 RX, manejador huart5.
 * - Pantalla Nextion: USART1, PA9 TX y PA10 RX, manejador huart1.
 *
 * Estos dos nombres centralizan el uso de las UART en la aplicacion.
 * No renombrar huart1 ni huart5: cada uno conserva su instancia y DMA.
 * La trama de control vigente contiene cinco valores y no utiliza CRC.
 *
 * Se conservan literalmente los comentarios originales. Las menciones
 * originales de USART1 para la trama y UART5 para Nextion describen el
 * montaje anterior; la asignacion vigente es la indicada arriba.
 */
#define USV_UART_TRAMA                 (&huart5)
#define USV_UART_NEXTION               (&huart1)

/*
 * ACTUALIZACION VIGENTE:
 *
 * Se conservan los comentarios originales de las versiones anteriores.
 * El comando ahora utiliza directamente las unidades transmitidas:
 *
 * - Potencias: porcentajes enteros de 0 a 100.
 * - Direccion: demanda de giro entera de -100 a +100.
 * - Camara: grados enteros de -90 a +90.
 *
 * No se multiplican ni se dividen los comandos por 10.
 * Los miembros x10 de la telemetria se mantienen sin cambios.
 *
 * Selector MODO_MAN_JOY:
 *
 * 0 = modo manual con potenciometros independientes.
 * 1 = modo joystick con potenciometro general.
 *
 * La mezcla de motores se calcula en este archivo.
 * El receptor aplica las potencias finales sin repetir la mezcla.
 *
 * No se modifica el pinout, el reloj, la MPU ni la telemetria recibida.
 */

/*
 * ACTUALIZACION VIGENTE DE LAS ENTRADAS DIGITALES:
 *
 * Los interruptores y botones conectan sus entradas a GND
 * cuando se activan. Las entradas utilizan GPIO_PULLUP.
 *
 * Contacto abierto:
 *
 * - El pull-up mantiene el pin en nivel alto.
 * - USV_PinActivo() devuelve 0.
 *
 * Contacto cerrado a GND:
 *
 * - El pin queda en nivel bajo.
 * - USV_PinActivo() devuelve 1.
 *
 * PB15:
 *
 * - Abierto, nivel alto: modo manual independiente.
 * - Conectado a GND, nivel bajo: modo joystick.
 *
 * PA15, emergencia:
 *
 * - Abierto, nivel alto: boton inactivo.
 * - Conectado a GND, nivel bajo: parada activa.
 *
 * PC7, reconexion:
 *
 * - Abierto, nivel alto: boton inactivo.
 * - Conectado a GND, nivel bajo: solicitud activa.
 *
 * PA15 y PC7 deben configurarse en CubeMX como:
 *
 * - External Interrupt Mode with Falling edge trigger detection.
 * - Pull-up.
 * - Interrupciones EXTI15 y EXTI7 habilitadas en NVIC.
 *
 * Se conserva el comportamiento anterior de liberacion de emergencia:
 * despues de incluir la parada en una transmision UART completada,
 * puede liberarse si el boton ya esta inactivo.
 * No se agrega un procedimiento de rearme.
 *
 * Los comentarios antiguos sobre pull-down, nivel alto activo
 * y flanco ascendente describen la version anterior.
 */

#define USV_POTENCIA_MAXIMA            100U
#define USV_DIRECCION_MAXIMA           100


/*
 * Índices correspondientes al orden de conversión
 * configurado en ADC1 dentro de CubeMX.
 */
#define ADC_INDICE_POT_GLOBAL          0U
#define ADC_INDICE_JOY_DIRECCION       1U
#define ADC_INDICE_POT_BABOR           2U
#define ADC_INDICE_POT_ESTRIBOR        3U
#define ADC_INDICE_JOY_CAMARA          4U
#define ADC_INDICE_VOLTAJE             5U

/* Valor máximo de una conversión ADC de 12 bits. */
#define ADC_CODIGO_MAXIMO              4095U

/* Voltaje de referencia utilizado por el ADC. */
#define ADC_VOLTAJE_REFERENCIA         3.3f

/*
 * Factor correspondiente al divisor resistivo de VOLBAT.
 *
 * Está temporalmente en 1.0 porque primero debemos conocer
 * los valores definitivos de las resistencias utilizadas.
 *
 * Ejemplo:
 *
 * Si posteriormente el divisor reduce 12 V hasta 3 V,
 * este factor deberá modificarse según el divisor real.
 */
#define VOLBAT_FACTOR_DIVISOR          1.0f

/*
 * Zona muerta del joystick de dirección.
 *
 * El valor está multiplicado por 10:
 *
 * 50 = 5.0 % alrededor de la posición central.
 */
/*
 * Vigente: 5 unidades de demanda de giro, sin escala x10.
 * Mantiene una zona central de aproximadamente +/-5 %.
 */
#define DIRECCION_ZONA_MUERTA          5

/*
 * Zona muerta del joystick de cámara.
 *
 * El valor está multiplicado por 10:
 *
 * 30 = 3.0 grados alrededor de la posición central.
 */
/* Vigente: 3 grados enteros, sin escala x10. */
#define CAMARA_ZONA_MUERTA             3

/*
 * Periodo de transmision hacia el bote.
 *
 * 400 ms equivalen a una frecuencia de 2.5 Hz.
 */
#define PERIODO_ENVIO_USV_MS           400U

/*
 * Periodo de lectura de la memoria actualizada por el DMA del ADC.
 * El DMA permanece circular, pero no genera una interrupcion por cada
 * secuencia de seis canales; asi no satura la CPU ni detiene UART5.
 */
#define PERIODO_LECTURA_ADC_MS           10U

/*
 * Espera maxima y acotada para la corta trama UART5 a 115200 baudios.
 * Una trama normal tarda pocos milisegundos. Si UART5 falla, la llamada
 * regresa y el ciclo vuelve a intentarlo; nunca espera indefinidamente.
 */
#define USV_UART_TX_TIMEOUT_MS           10U

/* Dos tramas completas pueden quedar pendientes sin detener el ciclo. */
#define USV_RX_COLA_TAMANO                2U

/*
 * Indicador de actividad de envio en PB2.
 * Se supone un LED activo en alto; si la tarjeta lo enciende en bajo,
 * intercambiar GPIO_PIN_SET y GPIO_PIN_RESET en los dos niveles siguientes.
 * El LED refleja envios completados por UART5, no recepcion confirmada del bote.
 */
#define USV_LED_TX_GPIO_PORT           GPIOB
#define USV_LED_TX_PIN                 GPIO_PIN_2
#define USV_LED_TX_ENCENDIDO           GPIO_PIN_SET
#define USV_LED_TX_APAGADO             GPIO_PIN_RESET
#define USV_LED_TX_PULSO_MS              100U

/* La aplicacion necesita exactamente los seis canales acordados. */
#if adc1_canales != 6
#error "Configurar adc1_canales y ADC1 con los seis canales del proyecto."
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/*
 * Copia estable de los seis valores obtenidos por ADC1.
 *
 * El arreglo original adc1_codigo[] es actualizado
 * continuamente por el DMA.
 */
static uint16_t adc_copia[adc1_canales] = {0U};

/*
 * Lecturas de los controles antes de seleccionar el modo.
 *
 * Se guardan separadas de las ordenes finales para que:
 *
 * - Una emergencia no borre las lecturas disponibles.
 * - Un cambio de modo utilice las ultimas muestras recibidas.
 * - Los controles de un modo no interfieran con el otro.
 *
 * Las potencias se almacenan directamente de 0 a 100.
 * La direccion se almacena directamente de -100 a +100.
 */
static uint16_t potencia_general_leida = 0U;
static uint16_t potencia_babor_leida = 0U;
static uint16_t potencia_estribor_leida = 0U;
static int16_t direccion_leida = 0;

/*
 * Estructura con todas las órdenes que serán enviadas
 * desde el control de tierra hacia el bote.
 */
static USV_Comando comando_tierra = {0};

/*
 * Estructura donde se almacenará la telemetría
 * recibida desde el bote.
 */
static USV_Telemetria telemetria_bote = {0};

/* Buffer utilizado para construir la trama $PUSVU. */
static char trama_tx[USV_TRAMA_MAXIMA] = {0};

/* Buffer utilizado para recibir una trama $PUSVD. */
static char trama_rx[USV_TRAMA_MAXIMA] = {0};

/*
 * Recepcion no bloqueante de UART5.
 *
 * La interrupcion recibe un byte, construye una linea y la deposita en una
 * cola de dos posiciones. El ciclo principal decodifica despues. Ningun CRC,
 * campo incorrecto, desbordamiento o ausencia de telemetria puede detener
 * la construccion y el envio periodico de la trama $PUSVU.
 */
static uint8_t uart5_rx_byte = 0U;
static char uart5_rx_construccion[USV_TRAMA_MAXIMA] = {0};
static volatile uint16_t uart5_rx_longitud = 0U;
static volatile uint8_t uart5_rx_recibiendo_trama = 0U;

static char uart5_rx_cola[USV_RX_COLA_TAMANO][USV_TRAMA_MAXIMA] = {{0}};
static volatile uint16_t uart5_rx_cola_longitud[USV_RX_COLA_TAMANO] = {0U};
static volatile uint8_t uart5_rx_cola_entrada = 0U;
static volatile uint8_t uart5_rx_cola_salida = 0U;
static volatile uint8_t uart5_rx_cola_cantidad = 0U;

/*
 * Voltaje calculado a partir del canal VOLBAT.
 *
 * Mientras VOLBAT_FACTOR_DIVISOR sea 1.0, representa
 * solamente el voltaje presente directamente en PC5.
 */
static float voltaje_control = 0.0f;

/* Tiempo de la última transmisión hacia el bote. */
static uint32_t tiempo_ultimo_envio = 0U;

/* Tiempos y estado del LED; solo los utiliza el ciclo principal. */
static uint32_t tiempo_encendido_led = 0U;
static uint8_t led_tx_activo = 0U;

/*
 * Solicita actualizar Nextion despues de apagar el pulso de PB2.
 * Separa el trabajo de USART1 del instante reservado para UART5.
 */
static uint8_t nextion_actualizacion_pendiente = 0U;

/*
 * Indica que existe información válida del bote.
 *
 * 0 = no hay telemetría válida.
 * 1 = telemetría recibida correctamente.
 */
static uint8_t telemetria_valida = 0U;

/*
 * Bandera activada por la interrupción de RECONEXION.
 *
 * Permite conservar una pulsación hasta que se envíe
 * al menos una trama con la solicitud.
 */
static volatile uint8_t interrupcion_reconexion = 0U;

/*
 * Bandera activada por la interrupción de EMERGENCIA.
 *
 * Permite que la parada sea atendida inmediatamente y
 * conservada hasta que se transmita al bote.
 */
static volatile uint8_t interrupcion_emergencia = 0U;

/* Contadores utilizados para diagnóstico de comunicación. */
static volatile uint32_t errores_uart5 = 0U;
static volatile uint32_t errores_construccion_trama = 0U;
static volatile uint32_t desbordamientos_uart5 = 0U;

/* Diagnostico de la recepcion del bote y del envio hacia Nextion. */
static volatile uint32_t diag_rx5_bytes = 0U;
static volatile uint32_t diag_rx5_lineas = 0U;
static uint32_t diag_pusvd_validas = 0U;
static uint32_t diag_pusvd_crc_o_formato = 0U;
static uint32_t diag_pusvd_campos_err = 0U;
static uint16_t diag_rx5_ultima_longitud = 0U;
static uint32_t diag_ultimo_pusvd_ok_ms = 0U;
static uint32_t diag_ultimo_reporte_ms = 0U;
static uint32_t diag_nextion_tx_ok = 0U;
static uint32_t diag_nextion_tx_error = 0U;


/*
 * Los contadores distinguen un flanco nuevo de uno ya incluido en una
 * trama. Asi una IRQ nueva no se pierde al liberar una bandera anterior.
 */
static volatile uint32_t generacion_emergencia;
static volatile uint32_t generacion_reconexion;

/* Impide arrancar la aplicacion dos veces sobre los mismos DMA activos. */
static uint8_t control_iniciado;

/* No solicita propulsion hasta recibir la primera transferencia ADC. */
static uint8_t adc_muestras_recibidas;

/* Indica que el DMA del ADC arranco y puede leerse periodicamente. */
static uint8_t adc_dma_activo;

/* Tiempo de la ultima copia de los seis canales ADC. */
static uint32_t tiempo_ultima_lectura_adc;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* Configura PB2 y mantiene el indicador ligado a envios aceptados. */
static void USV_IniciarLedTX(void);
static void USV_RegistrarEnvioLedTX(void);
static void USV_ActualizarLedTX(void);

/* Lee una entrada digital configurada como activa en nivel alto. */
/* Vigente: lee una entrada con pull-up, activa en nivel bajo. */
static uint8_t USV_PinActivo(
    GPIO_TypeDef *puerto,
    uint16_t pin);

/* Redondea un valor float al entero más cercano. */
static int32_t USV_RedondearFloat(float valor);

/* Convierte un código ADC hacia el rango solicitado. */
static int32_t USV_MapearADC(
    uint16_t codigo_adc,
    float salida_minima,
    float salida_maxima);

/* Actualiza los valores analógicos utilizados por el comando. */
static void USV_ActualizarAnalogicos(void);

/* Selecciona los controles y calcula la propulsion segun el modo. */
static void USV_AplicarModoPropulsion(void);

/* Actualiza los interruptores y aplica la seguridad del sistema. */
static void USV_ActualizarDigitales(void);

/* Procesa las tramas recibidas desde el bote. */
static void USV_ProcesarRecepcionXBee(void);

/* Rearma la recepcion por interrupcion si UART5 quedo disponible. */
static void USV_AsegurarRecepcionBote(void);

/* Construye y transmite la trama de control hacia el bote. */
static uint8_t USV_EnviarTramaControl(void);

/* Actualiza los elementos principales de la pantalla Nextion. */
static void USV_ActualizarPantalla(void);

/* Inicia la aplicacion y comprueba los procedimientos DMA. */
static HAL_StatusTypeDef USV_IniciarControl(void);

/* Ejecuta los procedimientos del control desde el ciclo principal. */
static void USV_ProcesarControl(void);

/* Envia una vez por segundo los contadores hacia Teleplot por USART2/PA2. */
static void USV_EnviarDiagnosticoTeleplot(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * Diagnostico independiente por USART2 TX, pin PA2, a 115200 baudios.
 * No utiliza UART5 (XBee) ni USART1 (Nextion). Si Teleplot o el adaptador
 * no estan conectados, la trama $PUSVU continua enviandose normalmente.
 */
static void USV_EnviarDiagnosticoTeleplot(void)
{
    uint32_t ahora = HAL_GetTick();
    uint32_t edad_ms;
    static char salida[420];
    int longitud;

    if ((uint32_t)(ahora - diag_ultimo_reporte_ms) < 1000U)
    {
        return;
    }

    diag_ultimo_reporte_ms = ahora;
    edad_ms = (diag_pusvd_validas != 0U) ?
        (uint32_t)(ahora - diag_ultimo_pusvd_ok_ms) : 0U;

    longitud = snprintf(
        salida,
        sizeof(salida),
        ">rx5_bytes:%lu\r\n"
        ">rx5_lineas:%lu\r\n"
        ">pusvd_validas:%lu\r\n"
        ">pusvd_crc_o_formato:%lu\r\n"
        ">pusvd_campos_err:%lu\r\n"
        ">rx5_uart_err:%lu\r\n"
        ">rx5_hal_error:%lu\r\n"
        ">rx5_desbordamientos:%lu\r\n"
        ">rx5_ultima_longitud:%u\r\n"
        ">pusvd_edad_ms:%lu\r\n"
        ">nextion_telemetria_valida:%u\r\n"
        ">nextion_tx_ok:%lu\r\n"
        ">nextion_tx_error:%lu\r\n",
        (unsigned long)diag_rx5_bytes,
        (unsigned long)diag_rx5_lineas,
        (unsigned long)diag_pusvd_validas,
        (unsigned long)diag_pusvd_crc_o_formato,
        (unsigned long)diag_pusvd_campos_err,
        (unsigned long)errores_uart5,
        (unsigned long)huart5.ErrorCode,
        (unsigned long)desbordamientos_uart5,
        (unsigned int)diag_rx5_ultima_longitud,
        (unsigned long)edad_ms,
        (unsigned int)telemetria_valida,
        (unsigned long)diag_nextion_tx_ok,
        (unsigned long)diag_nextion_tx_error);

    if ((longitud > 0) && ((size_t)longitud < sizeof(salida)))
    {
        /* Espera acotada: un fallo de Teleplot no puede bloquear el control. */
        (void)HAL_UART_Transmit(
            &huart2,
            (uint8_t *)salida,
            (uint16_t)longitud,
            35U);
    }
}

/* Inicializa solamente PB2; conserva las demas entradas y perifericos. */
static void USV_IniciarLedTX(void)
{
    GPIO_InitTypeDef gpio_led = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(
        USV_LED_TX_GPIO_PORT, USV_LED_TX_PIN, USV_LED_TX_APAGADO);

    gpio_led.Pin = USV_LED_TX_PIN;
    gpio_led.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_led.Pull = GPIO_NOPULL;
    gpio_led.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(USV_LED_TX_GPIO_PORT, &gpio_led);

    tiempo_encendido_led = 0U;
    led_tx_activo = 0U;
}

/*
 * Enciende PB2 durante un pulso visible por cada transmision aceptada.
 * Se llama exclusivamente cuando HAL_UART_Transmit() devuelve HAL_OK.
 */
static void USV_RegistrarEnvioLedTX(void)
{
    HAL_GPIO_WritePin(
        USV_LED_TX_GPIO_PORT, USV_LED_TX_PIN, USV_LED_TX_ENCENDIDO);

    tiempo_encendido_led = HAL_GetTick();
    led_tx_activo = 1U;
}

/*
 * Apaga PB2 cuando finaliza el pulso, sin detener el ciclo con HAL_Delay().
 * Con una trama cada 400 ms, el LED permanece 100 ms encendido y
 * aproximadamente 300 ms apagado entre transmisiones aceptadas.
 * Las restas uint32_t toleran el desbordamiento normal de HAL_GetTick().
 */
static void USV_ActualizarLedTX(void)
{
    uint32_t ahora;

    if (led_tx_activo == 0U)
    {
        return;
    }

    ahora = HAL_GetTick();

    if ((uint32_t)(ahora - tiempo_encendido_led) >=
        USV_LED_TX_PULSO_MS)
    {
        HAL_GPIO_WritePin(
            USV_LED_TX_GPIO_PORT, USV_LED_TX_PIN, USV_LED_TX_APAGADO);
        led_tx_activo = 0U;
    }
}

/*
 * Lee una entrada GPIO activa en nivel alto.
 *
 * Todas las entradas del control fueron configuradas
 * con resistencia pull-down:
 *
 * 0 V   = entrada inactiva.
 * 3.3 V = entrada activa.
 */

/*
 * ACTUALIZACION VIGENTE:
 *
 * El comentario anterior corresponde al montaje antiguo.
 * Ahora todas las entradas de interruptores y botones
 * utilizan pull-up y se activan al conectarse a GND.
 *
 * GPIO_PIN_SET:
 *
 * - Contacto abierto.
 * - Entrada inactiva.
 * - Devuelve 0.
 *
 * GPIO_PIN_RESET:
 *
 * - Contacto cerrado a GND.
 * - Entrada activa.
 * - Devuelve 1.
 *
 * Esta conversion es comun para motores, luces, bomba,
 * selector de modo, reconexion y emergencia.
 */
static uint8_t USV_PinActivo(
    GPIO_TypeDef *puerto,
    uint16_t pin)
{
    if (HAL_GPIO_ReadPin(puerto, pin) == GPIO_PIN_RESET)
    {
        return 1U;
    }

    return 0U;
}


/*
 * Redondea un número decimal al entero más cercano.
 *
 * Se utiliza después de map() para conservar correctamente
 * los valores positivos y negativos.
 */
static int32_t USV_RedondearFloat(float valor)
{
    if (valor >= 0.0f)
    {
        return (int32_t)(valor + 0.5f);
    }

    return (int32_t)(valor - 0.5f);
}


/*
 * Convierte un código ADC de 0 a 4095 hacia un rango
 * seleccionado mediante la función map() del profesor.
 */
static int32_t USV_MapearADC(
    uint16_t codigo_adc,
    float salida_minima,
    float salida_maxima)
{
    float resultado;

    /* Protege el cálculo ante un valor superior a 12 bits. */
    if (codigo_adc > ADC_CODIGO_MAXIMO)
    {
        codigo_adc = ADC_CODIGO_MAXIMO;
    }

    /* Realiza la conversión lineal entre los dos rangos. */
    resultado = map(
        (float)codigo_adc,
        0.0f,
        (float)ADC_CODIGO_MAXIMO,
        salida_minima,
        salida_maxima);

    /* Entrega el valor convertido y redondeado. */
    return USV_RedondearFloat(resultado);
}


/*
 * Copia los seis canales ADC y los convierte a las unidades
 * utilizadas por la trama de comunicación.
 */
static void USV_ActualizarAnalogicos(void)
{
    uint8_t indice;
    int32_t direccion;
    int32_t camara;

    /*
     * Copia el arreglo actualizado por el DMA.
     *
     * Cada elemento uint16_t puede leerse de manera atómica
     * en el STM32H562.
     */
    for (indice = 0U; indice < adc1_canales; indice++)
    {
        adc_copia[indice] = adc1_codigo[indice];
    }

    /*
     * La copia permite trabajar con valores locales, pero el DMA sigue
     * activo: seis lecturas atomicas no garantizan un unico barrido ADC.
     * No se presenta esta copia como una captura simultanea de canales.
     */
    adc_muestras_recibidas = 1U;

    /*
     * ACTUALIZACION DE UNIDADES:
     *
     * Los comentarios originales de escala se mantienen debajo.
     * Las conversiones vigentes son:
     *
     * Potenciometros -> 0...100 enteros.
     * Direccion      -> -100...+100 enteros.
     * Camara         -> -90...+90 grados enteros.
     *
     * Las lecturas de propulsion no se copian directamente al comando.
     * USV_AplicarModoPropulsion() selecciona las que corresponden.
     */

    /*
     * Potenciómetro general:
     *
     * ADC 0...4095 -> 0...1000
     * Trama       -> 0.0...100.0 %
     */
    /* Vigente: porcentaje entero 0...100, utilizado en modo joystick. */
    potencia_general_leida =
        (uint16_t)USV_MapearADC(
            adc_copia[ADC_INDICE_POT_GLOBAL],
            0.0f,
            (float)USV_POTENCIA_MAXIMA);

    /*
     * Joystick de dirección:
     *
     * ADC 0...4095 -> -1000...+1000
     * Trama       -> -100.0...+100.0 %
     */
    /* Vigente: demanda de giro entera -100...+100, sin escala x10. */
    direccion = USV_MapearADC(
        adc_copia[ADC_INDICE_JOY_DIRECCION],
        -(float)USV_DIRECCION_MAXIMA,
        (float)USV_DIRECCION_MAXIMA);

    /*
     * Elimina pequeñas variaciones eléctricas alrededor
     * de la posición central del joystick.
     */
    if ((direccion >= -DIRECCION_ZONA_MUERTA) &&
        (direccion <= DIRECCION_ZONA_MUERTA))
    {
        direccion = 0;
    }

    /* Se guarda la lectura; solamente gobierna en modo joystick. */
    direccion_leida = (int16_t)direccion;

    /*
     * Potenciómetro de babor:
     *
     * ADC 0...4095 -> 0.0...100.0 %
     */
    /* Porcentaje entero utilizado en modo manual independiente. */
    potencia_babor_leida =
        (uint16_t)USV_MapearADC(
            adc_copia[ADC_INDICE_POT_BABOR],
            0.0f,
            (float)USV_POTENCIA_MAXIMA);

    /*
     * Potenciómetro de estribor:
     *
     * ADC 0...4095 -> 0.0...100.0 %
     */
    /* Porcentaje entero utilizado en modo manual independiente. */
    potencia_estribor_leida =
        (uint16_t)USV_MapearADC(
            adc_copia[ADC_INDICE_POT_ESTRIBOR],
            0.0f,
            (float)USV_POTENCIA_MAXIMA);

    /*
     * Joystick de cámara:
     *
     * ADC 0...4095 -> -900...+900
     * Trama       -> -90.0...+90.0 grados
     */
    /*
     * Vigente: grados enteros -90...+90.
     * La camara funciona en ambos modos y no necesita escala x10.
     */
    camara = USV_MapearADC(
        adc_copia[ADC_INDICE_JOY_CAMARA],
        -90.0f,
        90.0f);

    /* Aplica una pequeña zona muerta en el centro. */
    if ((camara >= -CAMARA_ZONA_MUERTA) &&
        (camara <= CAMARA_ZONA_MUERTA))
    {
        camara = 0;
    }

    comando_tierra.camara = (int16_t)camara;

    /*
     * Convierte el canal VOLBAT a voltios.
     *
     * Cuando se definan las resistencias del divisor,
     * VOLBAT_FACTOR_DIVISOR deberá actualizarse.
     */
    voltaje_control =
        ((float)adc_copia[ADC_INDICE_VOLTAJE] *
         ADC_VOLTAJE_REFERENCIA *
         VOLBAT_FACTOR_DIVISOR) /
        (float)ADC_CODIGO_MAXIMO;
}


/*
 * Selecciona los controles segun el modo y calcula las potencias finales.
 *
 * modo_joystick = 0:
 *
 * - El potenciometro de babor gobierna el motor de babor.
 * - El potenciometro de estribor gobierna el motor de estribor.
 * - El potenciometro general no interviene.
 * - El joystick de direccion no interviene.
 * - General y direccion se envian en cero.
 *
 * modo_joystick = 1:
 *
 * - El general establece el limite de potencia para CADA motor.
 * - El joystick determina la diferencia entre los motores.
 * - Los potenciometros independientes no intervienen.
 *
 * Giro positivo, hacia la derecha:
 *
 * - Babor conserva la potencia general.
 * - Estribor disminuye proporcionalmente.
 *
 * Giro negativo, hacia la izquierda:
 *
 * - Estribor conserva la potencia general.
 * - Babor disminuye proporcionalmente.
 *
 * Ejemplo con general en 50:
 *
 * Direccion    Babor    Estribor
 *     0          50        50
 *    50          50        25
 *   100          50         0
 *   -50          25        50
 *  -100           0        50
 *
 * No se aumenta ningun motor por encima de la potencia general.
 * Los resultados se redondean al porcentaje entero mas cercano.
 *
 * Las multiplicaciones siguientes forman parte del calculo proporcional;
 * no son una escala x10 ni una conversion del formato de la trama.
 *
 * Las ordenes digitales de avante/atras conservan su funcionamiento.
 * Este procedimiento no agrega avance automatico.
 * La seguridad se aplica despues de calcular estas potencias.
 */
static void USV_AplicarModoPropulsion(void)
{
    uint32_t potencia;
    int32_t direccion;

    if (comando_tierra.modo_joystick == 0U)
    {
        /* En manual se ignoran general y joystick de direccion. */
        comando_tierra.potencia_global = 0U;
        comando_tierra.direccion = 0;

        /* Cada motor recibe su potenciometro independiente. */
        comando_tierra.potencia_babor = potencia_babor_leida;
        comando_tierra.potencia_estribor = potencia_estribor_leida;

        return;
    }

    /* En joystick solo intervienen el general y la direccion. */
    potencia = (uint32_t)potencia_general_leida;
    direccion = (int32_t)direccion_leida;

    /* Guarda las unidades directamente, sin escala x10. */
    comando_tierra.potencia_global = (uint16_t)potencia;
    comando_tierra.direccion = (int16_t)direccion;

    /* Con el joystick centrado ambos motores reciben el general. */
    comando_tierra.potencia_babor = (uint16_t)potencia;
    comando_tierra.potencia_estribor = (uint16_t)potencia;

    if (direccion > 0)
    {
        /*
         * Derecha: disminuye estribor y mantiene babor.
         * Suma medio divisor para redondear el resultado entero.
         */
        comando_tierra.potencia_estribor =
            (uint16_t)(
                ((potencia *
                  (uint32_t)(USV_DIRECCION_MAXIMA - direccion)) +
                 (uint32_t)(USV_DIRECCION_MAXIMA / 2)) /
                (uint32_t)USV_DIRECCION_MAXIMA);
    }
    else if (direccion < 0)
    {
        /* Izquierda: disminuye babor y mantiene estribor. */
        comando_tierra.potencia_babor =
            (uint16_t)(
                ((potencia *
                  (uint32_t)(USV_DIRECCION_MAXIMA + direccion)) +
                 (uint32_t)(USV_DIRECCION_MAXIMA / 2)) /
                (uint32_t)USV_DIRECCION_MAXIMA);
    }
}


/*
 * Lee todas las entradas digitales y aplica las condiciones
 * de seguridad antes de construir la trama.
 */
static void USV_ActualizarDigitales(void)
{
    uint8_t reconexion_pin;
    uint8_t emergencia_pin;

    /* Lee las órdenes independientes del motor de babor. */
    comando_tierra.babor_avante =
        USV_PinActivo(
            BABOR_AVANTE_GPIO_Port,
            BABOR_AVANTE_Pin);

    comando_tierra.babor_atras =
        USV_PinActivo(
            BABOR_ATRAS_GPIO_Port,
            BABOR_ATRAS_Pin);

    /* Lee las órdenes independientes del motor de estribor. */
    comando_tierra.estribor_avante =
        USV_PinActivo(
            ESTRIBOR_AVANTE_GPIO_Port,
            ESTRIBOR_AVANTE_Pin);

    comando_tierra.estribor_atras =
        USV_PinActivo(
            ESTRIBOR_ATRAS_GPIO_Port,
            ESTRIBOR_ATRAS_Pin);

    /* Lee el interruptor de las luces de navegación. */
    comando_tierra.luces =
        USV_PinActivo(
            LUCES_NAV_GPIO_Port,
            LUCES_NAV_Pin);

    /* Lee el interruptor de la bomba de achique. */
    comando_tierra.bomba =
        USV_PinActivo(
            BOMBA_ACHIQUE_GPIO_Port,
            BOMBA_ACHIQUE_Pin);

    /* Lee el selector de modo manual por joystick. */
    /*
     * Nombre vigente del campo: modo_joystick.
     *
     * Pin bajo = manual independiente.
     * Pin alto = joystick con potenciometro general.
     */
    /*
     * ACTUALIZACION VIGENTE CON PULL-UP:
     *
     * El comentario anterior describe la polaridad antigua.
     *
     * PB15 abierto, nivel alto:
     *
     * - USV_PinActivo() devuelve 0.
     * - modo_joystick = 0.
     * - Modo manual con potenciometros independientes.
     *
     * PB15 conectado a GND, nivel bajo:
     *
     * - USV_PinActivo() devuelve 1.
     * - modo_joystick = 1.
     * - Modo joystick con potenciometro general.
     */
    comando_tierra.modo_joystick =
        USV_PinActivo(
            MODO_MAN_JOY_GPIO_Port,
            MODO_MAN_JOY_Pin);

    /*
     * Recalcula desde las lecturas guardadas despues de leer el selector.
     * Esto tambien permite cambiar de modo entre dos notificaciones ADC.
     * La emergencia y las fallas tienen prioridad en el bloque posterior.
     */
    USV_AplicarModoPropulsion();

    /* Lee físicamente la entrada de reconexión. */
    reconexion_pin =
        USV_PinActivo(
            RECONEXION_GPIO_Port,
            RECONEXION_Pin);

    /*
     * La solicitud queda activa si el pin está en alto
     * o si anteriormente fue detectada por la interrupción.
     */
    /*
     * Vigente: reconexion_pin contiene el estado logico, no el nivel
     * electrico. Vale 1 cuando PC7 esta conectado a GND.
     */
    if ((reconexion_pin != 0U) ||
        (interrupcion_reconexion != 0U))
    {
        comando_tierra.reconexion = 1U;
    }
    else
    {
        comando_tierra.reconexion = 0U;
    }

    /* Lee físicamente la parada de emergencia. */
    emergencia_pin =
        USV_PinActivo(
            EMERGENCIA_GPIO_Port,
            EMERGENCIA_Pin);

    /*
     * Conserva la emergencia detectada hasta que sea enviada.
     *
     * Cuando el botón se encuentra en su posición normal
     * el pin debe permanecer en nivel bajo.
     */
    /*
     * ACTUALIZACION VIGENTE:
     *
     * Boton inactivo, contacto abierto: PA15 en nivel alto.
     * Boton activado, contacto cerrado a GND: PA15 en nivel bajo.
     *
     * emergencia_pin vale 1 cuando la parada esta activada.
     * Tambien se conserva una pulsacion breve detectada por EXTI.
     */
    if (emergencia_pin != 0U)
    {
        interrupcion_emergencia = 1U;
    }

    comando_tierra.parada = interrupcion_emergencia;

    /*
     * Detecta órdenes contradictorias.
     *
     * Un motor no puede recibir simultáneamente una orden
     * para avanzar y otra para retroceder.
     */
    if (((comando_tierra.babor_avante != 0U) &&
         (comando_tierra.babor_atras != 0U)) ||
        ((comando_tierra.estribor_avante != 0U) &&
         (comando_tierra.estribor_atras != 0U)))
    {
        comando_tierra.falla_direccion = 1U;
    }
    else
    {
        comando_tierra.falla_direccion = 0U;
    }

    /*
     * Ante una parada de emergencia o una combinación
     * contradictoria, se anulan todas las órdenes de propulsión.
     *
     * Las luces y la bomba no se apagan automáticamente,
     * ya que pueden ser necesarias durante la emergencia.
     */
    /* Tambien mantiene propulsion a cero antes de la primera lectura ADC. */
    if ((comando_tierra.parada != 0U) ||
        (comando_tierra.falla_direccion != 0U) ||
        (adc_muestras_recibidas == 0U))
    {
        comando_tierra.potencia_global = 0U;
        comando_tierra.direccion = 0;
        comando_tierra.potencia_babor = 0U;
        comando_tierra.potencia_estribor = 0U;

        comando_tierra.babor_avante = 0U;
        comando_tierra.babor_atras = 0U;
        comando_tierra.estribor_avante = 0U;
        comando_tierra.estribor_atras = 0U;
    }
}


/*
 * Procesa las lineas que la interrupcion de UART5 dejo en la cola.
 *
 * La decodificacion se realiza fuera de la interrupcion. Una trama erronea
 * se descarta y no modifica telemetria_bote ni telemetria_valida. De esta
 * manera Nextion conserva el ultimo valor correcto hasta recibir otro.
 */
static void USV_ProcesarRecepcionXBee(void)
{
    uint16_t longitud;
    uint8_t indice;
    uint32_t estado_irq;

    while (1)
    {
        estado_irq = __get_PRIMASK();
        __disable_irq();

        if (uart5_rx_cola_cantidad == 0U)
        {
            __set_PRIMASK(estado_irq);
            break;
        }

        indice = uart5_rx_cola_salida;
        longitud = uart5_rx_cola_longitud[indice];

        if ((longitud > 0U) && (longitud < sizeof(trama_rx)))
        {
            memcpy(trama_rx, uart5_rx_cola[indice], longitud + 1U);
        }
        else
        {
            trama_rx[0] = '\0';
            longitud = 0U;
        }

        uart5_rx_cola_salida =
            (uint8_t)((uart5_rx_cola_salida + 1U) % USV_RX_COLA_TAMANO);
        uart5_rx_cola_cantidad--;

        __set_PRIMASK(estado_irq);

        if (longitud > 0U)
        {
            diag_rx5_ultima_longitud = longitud;

            if (USV_VerificarTrama(trama_rx) == 0U)
            {
                /* CRC, terminacion o estructura exterior incorrecta. */
                diag_pusvd_crc_o_formato++;
            }
            else if (USV_LeerTelemetria(
                         trama_rx,
                         &telemetria_bote) != 0U)
            {
                diag_pusvd_validas++;
                diag_ultimo_pusvd_ok_ms = HAL_GetTick();
                telemetria_valida = 1U;
            }
            else
            {
                /* CRC correcto, pero identificador, campos o rangos invalidos. */
                diag_pusvd_campos_err++;

                /*
                 * Diagnostico temporal: muestra por Teleplot una trama
                 * rechazada cada tres segundos para revisar sus 31 campos.
                 * Esta salida utiliza USART2/PA2 y no modifica UART5 ni
                 * detiene el envio periodico de la trama $PUSVU al bote.
                 */
                {
                    static uint32_t ultimo_dump_ms = 0U;
                    uint32_t ahora = HAL_GetTick();

                    if ((uint32_t)(ahora - ultimo_dump_ms) >= 3000U)
                    {
                        static const char etiqueta[] = "TRAMA_RX: ";

                        ultimo_dump_ms = ahora;

                        (void)HAL_UART_Transmit(
                            &huart2,
                            (uint8_t *)etiqueta,
                            (uint16_t)(sizeof(etiqueta) - 1U),
                            50U);

                        (void)HAL_UART_Transmit(
                            &huart2,
                            (uint8_t *)trama_rx,
                            (uint16_t)strlen(trama_rx),
                            100U);
                    }
                }
            }
        }
        /* Si es invalida, se ignora sin detener ni borrar datos anteriores. */
    }
}


/*
 * Si un error de UART dejo detenida la recepcion, intenta rearmarla.
 * La llamada es no bloqueante. Su resultado no afecta el envio por PB13.
 */
static void USV_AsegurarRecepcionBote(void)
{
    if (USV_UART_TRAMA->RxState == HAL_UART_STATE_READY)
    {
        if (HAL_UART_Receive_IT(
                USV_UART_TRAMA,
                &uart5_rx_byte,
                1U) != HAL_OK)
        {
            errores_uart5++;
        }
    }
}


/*
 * Construye y transmite la trama completa de comandos por UART5.
 * La trama es corta y se envia cada 400 ms, por lo que una transmision
 * directa de aproximadamente 3 ms evita depender del callback del DMA TX.
 * La recepcion de UART5 y la comunicacion Nextion conservan sus DMA.
 */
static uint8_t USV_EnviarTramaControl(void)
{
    size_t longitud;
    HAL_StatusTypeDef estado;
    uint32_t estado_irq;
    uint32_t emergencia_en_trama;
    uint32_t generacion_reconexion_observada;

    /*
     * Relee las entradas justo antes de construir la trama y captura
     * los flancos asociados a ella en una seccion corta protegida.
     * No se transmite ni se espera a HAL con las IRQ bloqueadas.
     */
    estado_irq = __get_PRIMASK();
    __disable_irq();
    USV_ActualizarDigitales();
    emergencia_en_trama = generacion_emergencia;
    generacion_reconexion_observada = generacion_reconexion;
    __set_PRIMASK(estado_irq);

    /*
     * Construye la trama compacta $PUSVU sin CRC.
     * La libreria combina la magnitud y las ordenes fisicas para generar
     * cada motor entre -100 (atras) y +100 (avante).
     */
    longitud = USV_ConstruirComando(
        trama_tx,
        sizeof(trama_tx),
        &comando_tierra);

    /* Una longitud de cero indica un error de construcción. */
    if (longitud == 0U)
    {
        errores_construccion_trama++;
        return 0U;
    }

    /*
     * Transmite exactamente los bytes producidos por la trama.
     * UART no necesita que exista un receptor conectado para finalizar.
     */
    estado = HAL_UART_Transmit(
        USV_UART_TRAMA,
        (uint8_t *)trama_tx,
        (uint16_t)longitud,
        USV_UART_TX_TIMEOUT_MS);

    if (estado == HAL_OK)
    {
        /* No actualiza el LED ante HAL_BUSY, HAL_ERROR ni trama invalida. */
        USV_RegistrarEnvioLedTX();

        /*
         * RECONEXION queda reservada internamente para una version futura
         * y no forma parte de la trama actual. Se libera la pulsacion ya
         * observada despues de un ciclo aceptado para evitar dejarla fija.
         * Si llega otro flanco durante el proceso, se conserva.
         */
        estado_irq = __get_PRIMASK();
        __disable_irq();

        if ((comando_tierra.reconexion != 0U) &&
            (generacion_reconexion == generacion_reconexion_observada))
        {
            interrupcion_reconexion = 0U;
        }

        /*
         * La emergencia solamente se libera después de enviar
         * la orden y comprobar que el botón volvió a nivel bajo.
         */
        /*
         * ACTUALIZACION VIGENTE CON PULL-UP:
         *
         * El boton inactivo deja PA15 en nivel alto.
         * USV_PinActivo() devuelve 0 en esa condicion.
         *
         * Mientras PA15 permanezca conectado a GND,
         * USV_PinActivo() devuelve 1 y la emergencia no se libera.
         *
         * Se conserva el comportamiento anterior:
         * HAL_OK confirma que UART5 termino de sacar los bytes,
         * pero no confirma que el bote recibio la orden.
         */
        if ((comando_tierra.parada != 0U) &&
            (generacion_emergencia == emergencia_en_trama) &&
            (USV_PinActivo(
                EMERGENCIA_GPIO_Port,
                EMERGENCIA_Pin) == 0U))
        {
            interrupcion_emergencia = 0U;
        }

        __set_PRIMASK(estado_irq);

        return 1U;
    }

    return 0U;
}


/*
 * Entrega a Nextion toda la telemetria $PUSVD ya validada, junto con los
 * controles locales. USART1 permanece independiente de UART5: si la
 * pantalla esta ocupada o falla, se descarta solamente esa actualizacion.
 */
static void USV_ActualizarPantalla(void)
{
    HAL_StatusTypeDef estado_nextion;

    estado_nextion = NEXTION_EnviarEstadoUSV(
        USV_UART_NEXTION,
        &telemetria_bote,
        telemetria_valida,
        &comando_tierra,
        voltaje_control);

    if (estado_nextion == HAL_OK)
    {
        diag_nextion_tx_ok++;
    }
    else
    {
        /* Incluye HAL_BUSY y HAL_ERROR; no afecta el envio hacia el bote. */
        diag_nextion_tx_error++;
    }
}


/*
 * Inicializa la aplicacion despues de los MX_* de CubeMX.
 * Solo arrancan aqui los procedimientos DMA y las referencias de tiempo;
 * no se modifica el reloj, la MPU ni el pinout.
 */
static HAL_StatusTypeDef USV_IniciarControl(void)
{
    HAL_StatusTypeDef estado;

    if (control_iniciado != 0U)
    {
        return HAL_BUSY;
    }

    /*
     * Verifica que el manejador de UART5 utilizado para
     * la pantalla Nextion sea válido.
     */
    /* Verifica huart1, ahora conectado a la pantalla Nextion. */
    /*
     * Una falla de la pantalla no debe impedir el envio hacia el bote.
     * Las funciones de Nextion rechazaran sus propias transferencias
     * mientras la interfaz no se encuentre disponible.
     */
    /* Una falla de Nextion no impide iniciar ni transmitir hacia el bote. */
    (void)NEXTION_Init(USV_UART_NEXTION);

    /*
     * UART5 recibe la telemetria byte a byte mediante interrupcion.
     * No se inicia la capa UART DMA y tampoco se recibe desde Nextion:
     * USART1 se utiliza solamente para entregar datos a la pantalla.
     * Si esta primera solicitud falla, el ciclo principal vuelve a intentarla.
     */
    (void)HAL_UART_Receive_IT(
        USV_UART_TRAMA,
        &uart5_rx_byte,
        1U);

    /*
     * Inicia las seis conversiones de ADC1 mediante DMA.
     *
     * El DMA actualiza continuamente adc1_codigo[] siguiendo
     * el orden de los seis Ranks configurados en CubeMX.
     */
    /*
     * El DMA circular actualiza adc1_codigo[] por hardware. No necesitamos
     * una interrupcion por cada secuencia de seis conversiones. Con el ADC
     * continuo esa interrupcion ocurriria decenas de miles de veces por
     * segundo y podria impedir que el ciclo principal atienda UART5.
     */
    HAL_NVIC_DisableIRQ(GPDMA1_Channel0_IRQn);
    HAL_NVIC_ClearPendingIRQ(GPDMA1_Channel0_IRQn);

    /* Comprueba el resultado y conserva volatile en el arreglo del DMA. */
    estado = ADC_Read_DMA(
        &hadc1,
        adc1_canales,
        adc1_codigo);
    if (estado != HAL_OK)
    {
        /*
         * Sin ADC se conservan las potencias en cero por seguridad,
         * pero UART5 continua enviando la trama periodicamente.
         */
        adc_dma_activo = 0U;
        adc_muestras_recibidas = 0U;
    }
    else
    {
        /* Da tiempo a que el DMA complete varias secuencias iniciales. */
        HAL_Delay(1U);
        USV_ActualizarAnalogicos();
        adc_dma_activo = 1U;
    }

    /* Lee el estado inicial de todos los interruptores. */
    USV_ActualizarDigitales();

    /* Guarda las referencias iniciales de tiempo. */
    /* Permite realizar el primer envio inmediatamente al entrar al ciclo. */
    tiempo_ultimo_envio = HAL_GetTick() - PERIODO_ENVIO_USV_MS;
    tiempo_ultima_lectura_adc = HAL_GetTick();
    control_iniciado = 1U;
    return HAL_OK;
}

/*
 * Ejecuta una vuelta del control. No contiene esperas UART bloqueantes.
 * El retardo de 1 ms del main original se conserva al final.
 */
static void USV_ProcesarControl(void)
{
    uint32_t tiempo_actual;

    if (control_iniciado == 0U)
    {
        return;
    }

    /*
     * La recepcion es independiente del transmisor. Si una interrupcion o
     * un error la dejo libre, se rearma sin esperar y sin afectar UART5 TX.
     */
    USV_AsegurarRecepcionBote();
    USV_ProcesarRecepcionXBee();

    /* Obtiene el tiempo actual del sistema. */
    tiempo_actual = HAL_GetTick();

    /*
     * El DMA actualiza el arreglo continuamente sin interrumpir la CPU.
     * El ciclo copia y convierte los seis canales cada 10 ms.
     */
    if ((adc_dma_activo != 0U) &&
        ((uint32_t)(tiempo_actual - tiempo_ultima_lectura_adc) >=
         PERIODO_LECTURA_ADC_MS))
    {
        tiempo_ultima_lectura_adc = tiempo_actual;
        USV_ActualizarAnalogicos();
    }

    /*
     * Las entradas digitales se revisan continuamente
     * para atender rápidamente una emergencia.
     */
    USV_ActualizarDigitales();

    /*
     * No se invalida ni se borra la ultima telemetria correcta por tiempo.
     * Una trama ausente o incorrecta simplemente no refresca la pantalla.
     */

    /*
     * Envia la trama de control cada 400 ms.
     *
     * UART5 transmite directamente y no depende del estado de Nextion.
     */
    /* Envio por UART5 cada 400 ms, sin bloquear el ciclo. */
    if ((uint32_t)(tiempo_actual -
                   tiempo_ultimo_envio) >=
        PERIODO_ENVIO_USV_MS)
    {
        /*
         * La referencia de tiempo solo avanza cuando UART5 completa el envio.
         * Si estaba ocupada o presento un error, se reintenta desde el
         * ciclo principal sin esperar otros 400 ms.
         */
        if (USV_EnviarTramaControl() != 0U)
        {
            tiempo_ultimo_envio = tiempo_actual;
            nextion_actualizacion_pendiente = 1U;
        }
    }

    /* Actualiza la duracion del pulso generado por un envio completado. */
    USV_ActualizarLedTX();

    /*
     * Espera a que PB2 complete su pulso visible antes de atender Nextion.
     * La trama al bote ya salio y la pantalla conserva todos sus campos.
     */
    if ((nextion_actualizacion_pendiente != 0U) &&
        (led_tx_activo == 0U))
    {
        nextion_actualizacion_pendiente = 0U;
        USV_ActualizarPantalla();
    }

    /*
     * Retardo pequeño para evitar ejecutar innecesariamente
     * el ciclo principal a máxima velocidad.
     */
    USV_EnviarDiagnosticoTeleplot();
    HAL_Delay(1U);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_GPDMA1_Init();
  MX_ADC1_Init();
  MX_ICACHE_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* PB2 queda apagado hasta que UART5 acepte el primer envio del control. */
  USV_IniciarLedTX();

  /*
   * Inicia los procedimientos del control despues de los MX_* de CubeMX.
   * La trama utiliza UART5; la pantalla Nextion utiliza USART1.
   * La aplicacion permanece en Error_Handler si algun DMA no puede arrancar.
   */
  if (USV_IniciarControl() != HAL_OK)
  {
      Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /*
       * Ejecuta lectura de controles, seguridad, trama, telemetria y pantalla.
       * Los procedimientos estan definidos en este mismo archivo.
       */
      USV_ProcesarControl();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV2;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_0);
}

/* USER CODE BEGIN 4 */

/*
 * Recibe un byte de telemetria desde el bote por UART5.
 *
 * Solo acepta lineas que comiencen con '$' y terminen con '\n'. La trama se
 * copia a una cola corta para que el CRC y los campos se revisen en el ciclo
 * principal. Nunca se decodifica ni se transmite a Nextion dentro de la IRQ.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t longitud;
    uint8_t indice;

    if ((huart == NULL) || (huart->Instance != UART5))
    {
        return;
    }

    diag_rx5_bytes++;

    if (uart5_rx_byte == (uint8_t)'$')
    {
        uart5_rx_longitud = 0U;
        uart5_rx_recibiendo_trama = 1U;
    }

    if (uart5_rx_recibiendo_trama != 0U)
    {
        longitud = uart5_rx_longitud;

        if (longitud < (USV_TRAMA_MAXIMA - 1U))
        {
            uart5_rx_construccion[longitud] = (char)uart5_rx_byte;
            longitud++;
            uart5_rx_longitud = longitud;

            if (uart5_rx_byte == (uint8_t)'\n')
            {
                diag_rx5_lineas++;
                uart5_rx_construccion[longitud] = '\0';

                if (uart5_rx_cola_cantidad < USV_RX_COLA_TAMANO)
                {
                    indice = uart5_rx_cola_entrada;
                    memcpy(
                        uart5_rx_cola[indice],
                        uart5_rx_construccion,
                        longitud + 1U);

                    uart5_rx_cola_longitud[indice] = longitud;
                    uart5_rx_cola_entrada =
                        (uint8_t)((indice + 1U) % USV_RX_COLA_TAMANO);
                    uart5_rx_cola_cantidad++;
                }
                else
                {
                    /* Cola llena: descarta esta linea y conserva las anteriores. */
                    desbordamientos_uart5++;
                }

                uart5_rx_longitud = 0U;
                uart5_rx_recibiendo_trama = 0U;
            }
        }
        else
        {
            /* Linea demasiado larga: se descarta hasta encontrar otro '$'. */
            uart5_rx_longitud = 0U;
            uart5_rx_recibiendo_trama = 0U;
            desbordamientos_uart5++;
        }
    }

    /* Rearme no bloqueante para el siguiente byte. */
    if (HAL_UART_Receive_IT(USV_UART_TRAMA, &uart5_rx_byte, 1U) != HAL_OK)
    {
        errores_uart5++;
    }
}

/*
 * Callback ejecutado cuando una entrada EXTI detecta
 * el flanco ascendente configurado en CubeMX.
 */
/*
 * ACTUALIZACION VIGENTE:
 *
 * El comentario anterior corresponde a la configuracion antigua.
 * Los botones ahora conectan a GND al activarse.
 *
 * PC7 y PA15 utilizan interrupciones por flanco descendente:
 *
 * - Reposo: nivel alto por el pull-up.
 * - Activacion: transicion desde nivel alto hacia nivel bajo.
 *
 * Por eso se utiliza HAL_GPIO_EXTI_Falling_Callback().
 */
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    /*
     * Conserva la solicitud de reconexión hasta que
     * sea incluida en una trama transmitida.
     */
    if (GPIO_Pin == RECONEXION_Pin)
    {
        generacion_reconexion++;
        interrupcion_reconexion = 1U;
    }

    /*
     * Activa inmediatamente la parada de emergencia.
     *
     * La orden de propulsión será llevada a cero antes
     * de construir la siguiente trama.
     */
    if (GPIO_Pin == EMERGENCIA_Pin)
    {
        generacion_emergencia++;
        interrupcion_emergencia = 1U;
    }
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};
  MPU_Attributes_InitTypeDef MPU_AttributesInit = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region 0 and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x08FFF000;
  MPU_InitStruct.LimitAddress = 0x08FFFFFF;
  MPU_InitStruct.AttributesIndex = MPU_ATTRIBUTES_NUMBER0;
  MPU_InitStruct.AccessPermission = MPU_REGION_ALL_RO;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Attribute 0 and the memory to be protected
  */
  MPU_AttributesInit.Number = MPU_ATTRIBUTES_NUMBER0;
  MPU_AttributesInit.Attributes = INNER_OUTER(MPU_NOT_CACHEABLE);

  HAL_MPU_ConfigMemoryAttributes(&MPU_AttributesInit);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
