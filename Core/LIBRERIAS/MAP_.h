/*
 * MAP.H
 *
 *  Created on: Jun 1, 2025
 *      Author: ALCIDES_RAMOS
 */

//Evita declarar el contenido repetidamente cuando se incluye desde varios archivos.
#ifndef LIBRERIAS_MAP_H_
#define LIBRERIAS_MAP_H_

//Convierte un valor de un rango de entrada a un rango de salida.
//x_in: valor que se desea convertir.
//x1_, x2_: limites del rango de entrada.
//y1_, y2_: limites del rango de salida.
//Si x1_ y x2_ son iguales, devuelve y1_ para evitar dividir entre cero.
//No limita el resultado ni aplica una zona muerta.
float map(float x_in, float x1_, float x2_, float y1_, float y2_);

#endif /* LIBRERIAS_MAP_H_ */