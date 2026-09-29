/*
 * MAP.C
 *
 *  Created on: Jun 1, 2025
 *      Author: ALCIDES_RAMOS
 */
#include "MAP_.h"

float map(float x_in, float x1_, float x2_, float y1_, float y2_)
{
  float pendiente;

  //Evita dividir entre cero cuando los limites de entrada son iguales.
  if (x2_ == x1_)
  {
    //No existe un mapeo valido en este caso; devuelve el primer limite de salida.
    return y1_;
  }

  //Calcula la pendiente entre el rango de entrada y el rango de salida.
  pendiente=(y2_ - y1_) / (x2_ - x1_);

  //Calcula el desplazamiento de salida correspondiente al valor de entrada.
  pendiente=pendiente*(x_in- x1_);

  //Suma el primer limite de salida para completar el mapeo lineal.
  pendiente=pendiente+y1_;

  //Devuelve el resultado; los limites y la zona muerta se aplican en el control.
  return( pendiente);
}