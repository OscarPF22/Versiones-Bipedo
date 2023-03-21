-Capaz de realizar un paso

*Se han arreglado los fallos por limitación de corriente 
 incormporando las dos funciones para mover los servos de forma lineal.

- Problema por arreglar: Pierna izquierda el servo del robillo hay que 
  modificarle el valor porque topa con el límite al girar más de 10º a la izq.

- las funciones de movimiento de servo se les pasa los valores originales como referencia, para que se actualicen en
la propia funcion.

- la forma de trabajar con estas funciones ahora es de la siguiente forma:
	*Si voy a aumentar, paso el valor original más el incremento como valor final (val_actua + inc)
	*Si voy a poner en posición inicial lo que hago es pasarle como valor final el #define.