/*
 * adc.c
 *
 *  Created on: 01/08/2026
 *      Author: Alcides Ramos
 */

#include "adc_x.h"
#include <stdint.h>
#include <stddef.h>

//defina los adc a usar
volatile uint16_t adc1_codigo[adc1_canales];
volatile uint8_t adc1_flag = 0; //flag de conversion del adc

uint16_t ADC_Read(ADC_HandleTypeDef  *adc_n)
{
	uint16_t valor_adc = 0;

	//Verifica que se haya recibido un manejador ADC valido.
	if (adc_n == NULL)
	{
		return 0;
	}

	//Esta lectura individual no debe usarse mientras el mismo ADC trabaja por DMA.
	if (HAL_ADC_Start(adc_n) == HAL_OK)
	{
		if (HAL_ADC_PollForConversion(adc_n,1000) == HAL_OK)
		{
			//Obtiene el resultado antes de detener la conversion.
			valor_adc = (uint16_t)HAL_ADC_GetValue(adc_n);
		}

		//Detiene el ADC tambien cuando la espera de conversion falla.
		HAL_ADC_Stop(adc_n);
	}

	//El valor cero tambien puede ser una lectura valida; no identifica por si solo un error.
	return valor_adc;
}

HAL_StatusTypeDef ADC_Read_DMA (ADC_HandleTypeDef  *adc_n,uint8_t adc_num,volatile uint16_t *adc_codigo)
{

	//Comprueba el ADC, el arreglo de destino y la cantidad de muestras.
	if ((adc_n == NULL) || (adc_codigo == NULL) || (adc_num == 0))
	{
		return HAL_ERROR;
	}

	//CubeMX debe haber enlazado el manejador DMA al ADC.
	if (adc_n->DMA_Handle == NULL)
	{
		return HAL_ERROR;
	}

	//Evita solicitar mas muestras que las disponibles en el arreglo de esta libreria.
	if ((adc_codigo == adc1_codigo) && (adc_num > adc1_canales))
	{
		return HAL_ERROR;
	}

	//El destino uint16_t requiere una direccion alineada a dos bytes.
	if (((uintptr_t)adc_codigo % sizeof(uint16_t)) != 0U)
	{
		return HAL_ERROR;
	}

	//La bandera pertenece a ADC1; no se modifica al iniciar otro ADC.
	if (adc_n->Instance == ADC1)
	{
		adc1_flag = 0;
	}

	//Llamar una sola vez al iniciar el control, con el DMA circular configurado.
	//El arreglo debe permanecer disponible mientras el DMA esta funcionando.
	//Devuelve HAL_OK, HAL_BUSY o HAL_ERROR para que el control detecte un fallo de arranque.
	return HAL_ADC_Start_DMA(adc_n, (uint32_t *)(uintptr_t)adc_codigo, adc_num);
 // HAL_Delay(10);//retardo de conversion si necesita
}



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	//Si es el adc1
    if ((hadc != NULL) && (hadc->Instance == ADC1))
    {
        //Indica que termino una transferencia; el DMA circular continua funcionando.
        adc1_flag = 1;
    }
//si es otro adce colocar la bandera

}