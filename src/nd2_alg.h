
/*****************************************************************************/
/*                                                                           */
/*  Fichero:     nd2.h                                                       */
/*  Autor:       Javier Minguez                                              */
/*  Creado:      28/05/2003                                                  */
/*  Modificado:  21/06/2003                                                  */
/*                                                                           */
/*****************************************************************************/

#ifndef nd2_h
#define nd2_h

#include "geometria.h"

// ----------------------------------------------------------------------------
// CONSTANTES.
// ----------------------------------------------------------------------------

// N�mero de sectores: m�ltiplo de 4.
#define SECTORES 180

#define VERDADERO 1
#define FALSO 0
#define NO_SIGNIFICATIVO -1

// ----------------------------------------------------------------------------
// TIPOS.
// ----------------------------------------------------------------------------

// Informaci�n acerca del robot.

// Dimensiones del robot.
//   Consideramos el robot definido por un rect�ngulo. Numeramos sus
//   dimensiones, medidas a partir de su centro en las direcciones principales,
//   siguiendo la misma convenci�n que para los sectores:
//     Dimension[0]: distancia desde el centro a la trasera del robot.
//     Dimension[1]: distancia desde el centro a la izquierda del robot.
//     Dimension[2]: distancia desde el centro al frontal del robot.
//     Dimension[3]: distancia desde el centro a la derecha del robot.
typedef float TDimensiones[4];

typedef float TMatriz2x2[2][2];

typedef struct {

  TDimensiones Dimensiones;
  float enlarge;

  short int geometriaRect; // Si es cuadrado o no

  float R; // radio del robot por si es circular

  short int holonomo;

  float E[SECTORES];   // Distancia desde el origen de SR2 al per�metro del robot.
  float ds[SECTORES];  // Distancia de seguridad: desde el per�metro del robot al
  // per�metro de seguridad.

  float velocidad_lineal_maxima;
  float velocidad_angular_maxima;

  float aceleracion_lineal_maxima;
  float aceleracion_angular_maxima;

  float discontinuidad; // Espacio m�nimo por el que cabe el robot.

  float T; // Per�odo.

  TMatriz2x2 H; // Generador de movimientos: "Inercia" del robot.
  TMatriz2x2 G; // Generador de movimientos: "Fuerza" aplicada sobre el robot.

} TInfoRobot;

// Informaci�n acerca del objetivo.

typedef struct {
  TCoordenadas c0;
  TCoordenadas c1;
  TCoordenadasPolares p1;
  int s; // Sector.
} TObjetivo;

// Informaci�n acerca de la regi�n escogida.

#define DIRECCION_OBJETIVO                0
#define DIRECCION_DISCONTINUIDAD_INICIAL  1
#define DIRECCION_DISCONTINUIDAD_FINAL    2

typedef struct {
  int principio;
  int final;

  int principio_ascendente;
  int final_ascendente;

  int descartada;

  int direccion_tipo;
  int direccion_sector;
  float direccion_angulo;
} TRegion;

typedef struct {
  int longitud;
  TRegion vector[SECTORES];
} TVRegiones;

// Informaci�n interna del meto-do de navegaci�n.

typedef struct {

  TObjetivo objetivo;

  TSR SR1;                         // Estado actual del robot: posici�n y orientaci�n
  TVelocities velocidades;         // Estado actual del robot: velocidades 
                                   // lineal y angular.
  TCoordenadasPolares d[SECTORES]; // Distancia desde el centro del robot al
                                   // obst�culo m�s pr�ximo en cada sector
  float dr[SECTORES];              // Distancia desde el per�metro del robot al
                                   // obst�culo m�s pr�ximo en cada sector.
  TVRegiones regiones;             // S�lo como informaci�n de cara al exterior:
                                   // Lista de todas las regiones encontradas en el
                                   // proceso de selecci�n.
  int region;                      // Como almacenamos m�s de una regi�n debemos
                                   // indicar cu�l es la escogida.

  int obstaculo_izquierda, obstaculo_derecha;

  float angulosin;                 // S�lo como informaci�n de cara al exterior:
                                   // �ngulo antes de tener en cuenta los obst�culos
                                   // m�s pr�ximos.
  float angulocon;                 // S�lo como informaci�n de cara al exterior:
                                   // �ngulo despu�s de tener en cuenta los
                                   // obst�culos m�s pr�ximos.
  char situacion[20];              // S�lo como informaci�n de cara al exterior:
                                   // Situaci�n en la que se encuentra el robot.
  char cutting[20];                // S�lo como informaci�n de cara al exterior:
                                   // Cutting aplicado al movimiento del robot.

                                   // angle from from navigation algorithm, used by
                                   // movement generator ?!
  float angulo;                    // Salida del algoritmo de navegaci�n y entrada
                                   // al generador de movimientos: direcci�n de
                                   // movimiento deseada.
  float velocidad;                 // Salida del algoritmo de navegaci�n y entrada
                                   // al generador de movimientos: velocidad lineal
                                   // deseada

} TInfoND;

// ----------------------------------------------------------------------------
// VARIABLES
// ----------------------------------------------------------------------------

extern TInfoRobot robot;

// ----------------------------------------------------------------------------
// FUNCIONES.
// ----------------------------------------------------------------------------

extern float sector2angulo( int sector );

extern int angulo2sector( float angulo );


#endif
