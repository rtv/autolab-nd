
/*****************************************************************************/
/*                                                                           */
/*  Fichero:     nd.c                                                        */
/*  Autor:       Javier Minguez  -- Javier Osuna                             */
/*  Creado:      17/10/2002                                                  */
/*  Modificado:  24/06/2003                                                  */
/*                                                                           */
/*  $Id: nd_alg.cpp,v 1.4 2009-04-08 22:40:41 jwawerla Exp $                 */
/*                                                                           */
/*****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "nd_alg.h"
#include "nd2_alg.h"
#include "printerror.h"
#include <assert.h>
//#include <stdlib.h>

// ----------------------------------------------------------------------------
// CONSTANTES
// ----------------------------------------------------------------------------
#define DISTANCIA_INFINITO 1e6F

// ----------------------------------------------------------------------------
// VARIABLES
// ----------------------------------------------------------------------------
FILE *depuracion;
int iteracion = 0;

// Esta variable NO debe declararse como est�tica.
TInfoRobot robot;

static TVelocities velocidades; // Resultado de IterarND().

// ----------------------------------------------------------------------------
// FUNCIONES
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Operaciones con sectores
// ----------------------------------------------------------------------------

#define INCREMENTAR_SECTOR(s) (((s)+1)%SECTORES)
#define DECREMENTAR_SECTOR(s) (((s)+(SECTORES-1))%SECTORES)

// Esta funci�n NO debe declararse como est�tica.
float sector2angle ( int sector )
{
  // Sector debe estar entre 0 y SECTORES-1.

#define FACTOR (-(2.0F*PI)/SECTORES)
#define SUMANDO (-SECTORES/2)

  return FACTOR* ( sector + SUMANDO );

#undef SUMANDO
#undef FACTOR
}
//---------------------------------------------------------------------------
int angle2sector ( float angle )
{
  // Angle debe estar normalizado.

#define FACTOR (-SECTORES/(2.0F*PI))
#define SUMANDO ((SECTORES+1.0F)/2.0F)

  return ( ( int ) ( FACTOR*angle + SUMANDO ) ) % SECTORES;

#undef SUMANDO
#undef FACTOR
}
//---------------------------------------------------------------------------
static int ObtenerSectorP ( TCoordenadasPolares p )
{

#define FACTOR (-SECTORES/(2.0F*PI))
#define SUMANDO ((SECTORES+1.0F)/2.0F)

  return ( ( int ) ( FACTOR*p.a + SUMANDO ) ) % SECTORES;

#undef SUMANDO
#undef FACTOR
}
//---------------------------------------------------------------------------
static int DistanciaSectorialOrientada ( int s1, int s2 )  // Distancia de s1 a s2.
{
  return ( s1 <= s2 ) ? s2 - s1 : ( ( s2 + SECTORES ) - s1 ) % SECTORES;
}

// ----------------------------------------------------------------------------
// InicializarND y sus funciones auxiliares.
// ----------------------------------------------------------------------------

static void InicializarE ( void )
{
  // Calcula la distancia desde el origen (punto de coordenadas 0.0F,0.0F)
  // hasta el per�metro (que contiene el origen) en la direcci�n de la bisectriz
  // de cada sector.

  TCoordenadasPolares limite;
  int li, ld, i;

  limite.a = ARCOTANGENTE ( robot.Dimensiones[0], robot.Dimensiones[1] );
  li = angle2sector ( limite.a );

  if ( sector2angle ( li ) > limite.a )
    li++;

  ConstruirCoordenadasPxy ( &limite, robot.Dimensiones[2], robot.Dimensiones[1] );

  ld = angle2sector ( limite.a );

  if ( sector2angle ( ld ) > limite.a )
    ld++;

  robot.E[0] = -robot.Dimensiones[0];

  for ( i = 1; i < li; i++ )
    robot.E[i] = robot.Dimensiones[0] / ( float ) cos ( sector2angle ( i ) );

  for ( i = li; i < ld; i++ )
    robot.E[i] = robot.Dimensiones[1] / ( float ) sin ( sector2angle ( i ) );

  for ( i = ld; i <= SECTORES / 2; i++ )
    robot.E[i] = limite.r;

  for ( i = SECTORES / 2 + 1; i < SECTORES; i++ )
    robot.E[i] = robot.E[SECTORES-i]; // Por simetria respecto del eje X.
}
//---------------------------------------------------------------------------
static void InicializarERedondo ( void )
{
  // Calcula la distancia desde el origen (punto de coordenadas 0.0F,0.0F)
  // hasta el per�metro (que contiene el origen) en la direcci�n de la bisectriz
  // de cada sector.
  int i;

  for ( i = 0; i < SECTORES; i++ )
    robot.E[i] = robot.R;
}
//---------------------------------------------------------------------------
static void InicializarDSRedondo ( float dmax )
{
  // Calcula la distancia desde el origen (punto de coordenadas 0.0F,0.0F)
  // hasta el per�metro (que contiene el origen) en la direcci�n de la bisectriz
  // de cada sector.
  int i;

  for ( i = 0; i < SECTORES; i++ )
    robot.ds[i] = dmax;
}
//---------------------------------------------------------------------------
static void InicializarDS ( float dsmax, float dsmin )
{
  TCoordenadas p1, p2;
  TCoordenadas q1, q2, q3;
  TCoordenadasPolares q4;
  float limite1, limite2, limite3, limite4;
  float coseno, seno;
  float a, b, c, m, n;
  float angle, distancia;
  int i;

  ConstruirCoordenadasCxy ( &p1, robot.Dimensiones[0], robot.Dimensiones[1] );
  ConstruirCoordenadasCxy ( &p2, robot.Dimensiones[2], robot.Dimensiones[1] );

  b = dsmax - dsmin;
  c = p2.x - p1.x;
  a = ( float ) sqrt ( CUADRADO ( c ) - CUADRADO ( b ) );
  coseno = a / c;
  seno = b / c;

  SumarCoordenadasCxyC ( p1, -dsmin, 0.0F, &q1 );
  SumarCoordenadasCxyC ( p1, -dsmin*seno, dsmin*coseno, &q2 );
  SumarCoordenadasCxyC ( p2, -dsmax*seno, dsmax*coseno, &q3 );
  ConstruirCoordenadasPC ( &q4, p2 );
  q4.r += dsmax;

  limite1 = ARCOTANGENTE ( q1.x, q1.y );
  limite2 = ARCOTANGENTE ( q2.x, q2.y );
  limite3 = ARCOTANGENTE ( q3.x, q3.y );
  limite4 = q4.a;

  robot.ds[0] = -q1.x - robot.E[0]; // = q1.x/(float)cos(PI) - ...;

  m = CUADRADO ( p1.x ) + CUADRADO ( p1.y ) - CUADRADO ( dsmin );
  n = CUADRADO ( p2.x ) + CUADRADO ( p2.y ) - CUADRADO ( dsmax );

  b = q3.x - q2.x;
  c = q3.y - q2.y;
  a = b * q2.y - c * q2.x;

  for ( i = 1; i < SECTORES / 2; i++ ) {
    angle = sector2angle ( i );

    // C�lculo de la distancia de seguridad correspondiente a la bisectriz
    // del sector i.

    if ( angle >= limite1 )
      // r1
      distancia = q1.x / ( float ) cos ( angle );

    else
      if ( angle >= limite2 ) {
        // r2
        distancia = p1.x * ( float ) cos ( angle ) + p1.y * ( float ) sin ( angle );
        distancia = distancia + ( float ) sqrt ( CUADRADO ( distancia ) - m );

      } else
        if ( angle >= limite3 )
          // r3
          distancia = a / ( b * ( float ) sin ( angle ) - c * ( float ) cos ( angle ) );

        else
          if ( angle >= limite4 ) {

            // r4
            distancia = p2.x * ( float ) cos ( angle ) + p2.y * ( float ) sin ( angle );
            distancia = distancia + ( float ) sqrt ( CUADRADO ( distancia ) - n );

          } else

            // r5
            distancia = q4.r;

    // Fin del c�lculo de la distancia de seguridad correspondiente a la bisectriz
    // del sector i.

    robot.ds[i] = distancia - robot.E[i];

    robot.ds[SECTORES-i] = robot.ds[i]; // El robot es sim�trico respecto del eje X.
  }

  //  robot.ds[SECTORES/2]=q4.x-robot.E[SECTORES/2]; // = q4.x/(float)cos(0.0F) - ...;
  robot.ds[SECTORES/2] = q4.r - robot.E[SECTORES/2]; // = q4.x/(float)cos(0.0F) - ...;
}
//---------------------------------------------------------------------------
void InicializarND ( TParametersND *parametros )
{
  /* printf("geom %d\n",parametros->geometriaRect); */
  robot.geometriaRect = parametros->geometryRect;
  robot.holonomo = parametros->holonomic;

  if ( parametros->geometryRect == 1 ) {
    // Cuadrado
    robot.Dimensiones[0] = -parametros->back;
    robot.Dimensiones[1] = parametros->left;
    robot.Dimensiones[2] = parametros->front;
    robot.Dimensiones[3] = -robot.Dimensiones[1];

    robot.enlarge = parametros->enlarge;

    InicializarE();
    InicializarDS ( parametros->dsmax, parametros->dsmin );

  } else {
    // Redondo
    robot.R = parametros->R;
    InicializarERedondo();
    InicializarDSRedondo ( parametros->dsmax );
  }

  robot.velocidad_lineal_maxima = parametros->vlmax;
  robot.velocidad_angular_maxima = parametros->vamax;
  robot.aceleracion_lineal_maxima = parametros->almax;
  robot.aceleracion_angular_maxima = parametros->aamax;
  robot.discontinuidad = parametros->discontinuity;

  robot.T = parametros->T;

  if ( !robot.holonomo ) {
    robot.H[0][0] = ( float ) exp ( -parametros->almax * parametros->T /
                                    parametros->vlmax );
    robot.H[0][1] = 0.0F; // Se tiene en cuenta m�s adelante y no se incluye en
    // las ecuaciones
    robot.H[1][0] = 0.0F; // Se tiene en cuenta m�s adelante y no se incluye en
    // las ecuaciones
    robot.H[1][1] = ( float ) exp ( -parametros->aamax * parametros->T /
                                    parametros->vamax );

    robot.G[0][0] = ( 1.0F - ( float ) exp ( -parametros->almax * parametros->T /
                      parametros->vlmax ) ) *
                    ( parametros->vlmax / parametros->almax );
    robot.G[0][1] = 0.0F; // Se tiene en cuenta m�s adelante y no se incluye en
    // las ecuaciones
    robot.G[1][0] = 0.0F; // Se tiene en cuenta m�s adelante y no se incluye en
    // las ecuaciones
    robot.G[1][1] = ( 1.0F - ( float ) exp ( -parametros->aamax * parametros->T /
                      parametros->vamax ) ) *
                    ( parametros->vamax / parametros->almax );
  }
}

// ----------------------------------------------------------------------------
// IterarND y sus funciones auxiliares.
// ----------------------------------------------------------------------------
// IterarND / SectorizarMapa

static void SectorizarMapa ( TInfoEntorno *mapa, TInfoND *nd )
{
  TCoordenadas p;
  TCoordenadasPolares pp; // M�dulos al cuadrado para evitar ra�ces innecesarias.
  int i, j;

  // clear all sector bins from obstacles
  for ( i = 0; i < SECTORES; i++ )
    nd->d[i].r = -1.0F;

  for ( i = 0; i < mapa->longitud; i++ ) {
    p = mapa->punto[i];
    // transform obstacle point into local coordinate system
    TRANSFORMACION01 ( & ( nd->SR1 ), &p )
    // transform obstacle point into polar coordinates
    ConstruirCoordenadasPcC ( &pp, p );

    // find sector index
    j = ObtenerSectorP ( pp );

    // fill polar histogram with closest obstacle distance
    if ( ( nd->d[j].r < 0.0F ) || ( pp.r < nd->d[j].r ) )
      nd->d[j] = pp;
  }

  for ( i = 0; i < SECTORES; i++ )
    if ( nd->d[i].r >= 0.0F ) {
      // RAIZ: sqrt(x)
      nd->d[i].r = RAIZ ( nd->d[i].r );

      if ( ( i != SECTORES / 2 ) && ( nd->d[i].r < robot.E[i] + 0.01F ) )
        nd->d[i].r = robot.E[i] + 0.01F;
    }
}

//----------------------------------------------------------------------------

// IterarND / ParadaEmergencia

static int ParadaEmergencia ( TInfoND *nd )
{
  // Devuelve 1 si hay peligro de colisi�n y hay que hacer una parada de emergencia;
  // devuelve 0 en caso contrario.
  // En la detecci�n de colisi�n se tiene en cuenta que el robot es sim�trico
  // respecto del eje X.

  TCoordenadas p;
  TCoordenadasPolares pp;
  int i;

  // Detecta si obstaculo en la parte delantera
  ConstruirCoordenadasCxy ( &p, robot.Dimensiones[2], robot.Dimensiones[1] );
  ConstruirCoordenadasPC ( &pp, p );

  for ( i = angle2sector ( pp.a ); i <= angle2sector ( -pp.a ); i++ )
    if ( ( nd->d[i].r >= 0.0F ) &&
         ( nd->d[i].r <= pp.r ) && ( ( float ) fabs ( nd->d[i].a ) <= pp.a ) )
      return 1;

  return 0;
}

// ----------------------------------------------------------------------------
// IterarND / SeleccionarRegiones / SiguienteDiscontinuidad

static void SiguienteDiscontinuidad ( TInfoND *nd, int principio, int left,
                                      int *discontinuidad, int *ascendente )
{
  // Se busca desde "principio" en la direcci�n indicada por "left".

  int i, j;
  float distancia_i, distancia_j;
  int no_obstaculo_i, no_obstaculo_j;

  j = principio;
  distancia_j = nd->d[j].r;
  no_obstaculo_j = ( distancia_j < 0.0F );

  do {
    i = j;
    distancia_i = distancia_j;
    no_obstaculo_i = no_obstaculo_j;

    j = ( left ? DECREMENTAR_SECTOR ( i ) : INCREMENTAR_SECTOR ( i ) );
    distancia_j = nd->d[j].r;
    no_obstaculo_j = ( distancia_j <= 0.0F );

    if ( no_obstaculo_i && no_obstaculo_j )
      continue;

    if ( no_obstaculo_i || no_obstaculo_j ) {
      *discontinuidad = i;
      *ascendente = no_obstaculo_i;
      return;
    }

    if ( ( float ) fabs ( distancia_i -distancia_j ) >= robot.discontinuidad ) {
      *discontinuidad = i;
      *ascendente = ( distancia_i > distancia_j );
      return;
    }

  } while ( j != principio );

  *discontinuidad = -1;
}

//---------------------------------------------------------------------------
// IterarND / SeleccionarRegiones / ObjetivoAlcanzable

static int ObjetivoAlcanzable ( TInfoND *nd, TRegion *region, int direction_tipo )
{
  // "direction_tipo" puede tomar los siguientes valores declarados en 'nd2.h':
  // - DIRECTION_OBJETIVO
  // - DIRECTION_DISCONTINUIDAD_INICIAL
  // - DIRECTION_DISCONTINUIDAD_FINAL

  TCoordenadas FL[SECTORES], FR[SECTORES];
  int nl, nr;

  TCoordenadasPolares objetivo_intermedio_polares; // Respecto de SR1.
  TCoordenadas objetivo_intermedio;                // Respecto de un SR con origen en el origen de SR1 y
  // girado hasta que el semieje positivo de abscisas
  // coincide con la direccin al objetivo intermedio.

  int sector_auxiliar;
  float limite;

  TCoordenadas p1, p2, p;
  int i, j;

  region->direction_tipo = direction_tipo;

  if ( region->direction_tipo == DIRECTION_OBJETIVO ) {

    region->direction_sector = nd->objetivo.s;
    objetivo_intermedio_polares = nd->objetivo.p1;

  } else {

    if ( region->direction_tipo == DIRECTION_DISCONTINUIDAD_INICIAL ) {
      region->direction_sector = region->principio;
      sector_auxiliar = DECREMENTAR_SECTOR ( region->direction_sector );
    } else {
      region->direction_sector = region->final;
      sector_auxiliar = INCREMENTAR_SECTOR ( region->direction_sector );
    }

    if ( nd->d[region->direction_sector].r < 0.0F )
      ConstruirCoordenadasPra ( &objetivo_intermedio_polares, nd->d[sector_auxiliar].r +
                                DISTANCIA_INFINITO,
                                BisectrizAngleNoOrientado ( sector2angle (
                                                               region->direction_sector ),
                                                             nd->d[sector_auxiliar].a ) );
    else {
      ConstruirCoordenadasCP ( &p1, nd->d[region->direction_sector] );
      ConstruirCoordenadasCP ( &p2, nd->d[sector_auxiliar] );
      ConstruirCoordenadasPxy ( &objetivo_intermedio_polares, ( p1.x + p2.x ) / 2.0F,
                                ( p1.y + p2.y ) / 2.0F );
    }

  }

  region->direction_angle = objetivo_intermedio_polares.a;

  ConstruirCoordenadasCxy ( &objetivo_intermedio, objetivo_intermedio_polares.r, 0.0F );

  // Determinaci�n de si el objetivo est� dentro de un C-Obst�culo y
  // construcci�n de las listas de puntos FL y FR.

  // Para no hacer ra�ces cuadradas dentro del bucle.
  limite = CUADRADO ( robot.discontinuidad / 2.0F );
  nl = 0;
  nr = 0;

  for ( i = 0; i < SECTORES; i++ ) {
    // Si no existe un obst�culo en el sector actual, pasamos al siguiente sector.
    if ( nd->d[i].r < 0.0F )
      continue;

    ConstruirCoordenadasCra ( &p, nd->d[i].r, nd->d[i].a - region->direction_angle );

    if ( ( p.x < 0.0F ) || ( p.x >= objetivo_intermedio.x ) ||
         ( ( float ) fabs ( p.y ) > robot.discontinuidad ) ) // Si el obst�culo no est�
      // en el rect�ngulo que consideramos, pasamos al siguiente sector.
      continue;

    if ( DISTANCIA_CUADRADO2 ( p, objetivo_intermedio ) < limite )
      // Si el objetivo intermedio
      // est� en colisi�n con el obst�culo, es inalcanzable.
      return 0; // Objetivo intermedio inalcanzable.

    if ( p.y > 0.0F )
      FL[nl++] = p;
    else
      FR[nr++] = p;
  }

  // Determinaci�n de si los obst�culos nos impiden alcanzar el objetivo intermedio.

  limite = CUADRADO ( robot.discontinuidad ); // Para no hacer ra�ces cuadradas dentro de los bucles.

  for ( i = 0; i < nl; i++ )
    for ( j = 0; j < nr; j++ )
      if ( DISTANCIA_CUADRADO2 ( FL[i], FR[j] ) < limite )
        return 0; // Objetivo intermedio inalcanzable.

  return 1; // Objetivo intermedio alcanzable.
}
//---------------------------------------------------------------------------
// IterarND / SeleccionarRegion
static void SeleccionarRegion ( TInfoND *nd )
{

#define LEFT TRUE
#define RIGHT FALSE

  int objetivo_a_la_vista;
  TRegion *region, *region_left, *region_right, *region_auxiliar;
  int indice, indice_left, indice_right, indice_auxiliar;
  int distancia_left, distancia_right;


  objetivo_a_la_vista = ( nd->d[nd->objetivo.s].r < 0.0F ) ||
                        ( nd->objetivo.p1.r <= nd->d[nd->objetivo.s].r );

  //printf ( " %f %f %f \n", nd->d[nd->objetivo.s].r, nd->objetivo.p1.r, nd->d[nd->objetivo.s].r );
  // Inicializamos el vector de regiones.
  nd->regiones.longitud = 0;

  indice = nd->regiones.longitud++;
  region = & ( nd->regiones.vector[indice] );
  region->descartada = FALSE;

  nd->region = -1;

  // Buscamos la primera discontinuidad.

  SiguienteDiscontinuidad ( nd, nd->objetivo.s, LEFT, & ( region->principio ),
                            & ( region->principio_ascendente ) );

  if ( region->principio == -1 ) {

    // No hay discontinuidades.

    region->principio = 0;
    region->final = SECTORES - 1;

    // do we have direct line of sight ?
    if ( objetivo_a_la_vista ) {
      region->direction_tipo = DIRECTION_OBJETIVO;
      region->direction_sector = nd->objetivo.s;
      region->direction_angle = nd->objetivo.p1.a;
      nd->region = indice;
      return;
    }

    // Objetivo inalcanzable.
    region->descartada = TRUE;

    return;
  }

  // Existe al menos una discontinuidad.

  SiguienteDiscontinuidad ( nd, nd->objetivo.s, RIGHT, & ( region->final ),
                            & ( region->final_ascendente ) );

  if ( region->final == DECREMENTAR_SECTOR ( region->principio ) ) {

    // Hay una sola discontinuidad.

    if ( objetivo_a_la_vista ) {
      region->direction_tipo = DIRECTION_OBJETIVO;
      region->direction_sector = nd->objetivo.s;
      region->direction_angle = nd->objetivo.p1.a;
      nd->region = indice;
      return;
    }

    if ( ObjetivoAlcanzable ( nd, region, & ( region->principio_ascendente ) ?
                              DIRECTION_DISCONTINUIDAD_INICIAL : DIRECTION_DISCONTINUIDAD_FINAL ) ) {
      nd->region = indice;
      return;
    }

    // Objetivo inalcanzable.
    region->descartada = TRUE;

    return;
  }

  // Hay dos o m�s discontinuidades.

  if ( objetivo_a_la_vista ) {

    // Regi�n del objetivo.

    if ( !region->principio_ascendente && !region->final_ascendente ) {

      // Regi�n artificial.

      indice_auxiliar = nd->regiones.longitud; // No incrementamos la longitud del
      // vector: Nuestra regi�n auxiliar
      // es ilegal.
      region_auxiliar = & ( nd->regiones.vector[indice_auxiliar] );

      *region_auxiliar = *region; // Utilizamos la regi�n auxiliar para almacenar
      // el contenido de la regi�n que vamos a modificar.

      region->principio = nd->objetivo.s;
      region->final = nd->objetivo.s;

      if ( ObjetivoAlcanzable ( nd, region, DIRECTION_OBJETIVO ) ) {
        nd->region = indice;
        return;
      }

      nd->regiones.longitud++; // Regularizamos la situaci�n de nuestra regi�n
      // auxiliar y

      indice = indice_auxiliar;  // la escogemos como regi�n a examinar.
      region = region_auxiliar;

    } else
      if ( ObjetivoAlcanzable ( nd, region, DIRECTION_OBJETIVO ) ) {

        // Regi�n "natural".

        nd->region = indice;
        return;
      }

  }

  indice_left = indice;

  region_left = region;

  indice_right = indice;
  region_right = region;

  do {

    distancia_left =
      DistanciaSectorialOrientada ( region_left->principio, nd->objetivo.s );
    distancia_right =
      DistanciaSectorialOrientada ( nd->objetivo.s, region_right->final );

    if ( distancia_left <= distancia_right ) {

      // Probamos por la regi�n left.
      if ( region_left->principio_ascendente ) {

        if ( ObjetivoAlcanzable ( nd, region_left,
                                  DIRECTION_DISCONTINUIDAD_INICIAL ) ) {
          if ( region_right->principio_ascendente ||
               region_right->final_ascendente ) {
            nd->region = indice_left;
            return;
          }

          if ( indice_right > indice_left ) {
            nd->regiones.longitud--;
            nd->region = indice_left;
            return;
          }

          *region_right = *region_left;

          nd->regiones.longitud--;
          nd->region = indice_right;
          return;
        }

        if ( indice_left != indice_right )
          region_left->descartada = TRUE;

        region_auxiliar = region_left;

        indice_left = nd->regiones.longitud++;

        region_left = & ( nd->regiones.vector[indice_left] );

        region_left->descartada = FALSE;

        region_left->final = DECREMENTAR_SECTOR ( region_auxiliar->principio );

        region_left->final_ascendente = !region_auxiliar->principio_ascendente;

        SiguienteDiscontinuidad ( nd, region_left->final, LEFT,
                                  & ( region_left->principio ),
                                  & ( region_left->principio_ascendente ) );

      } else { // Principio descendente: Ser� un final ascendente en la
        // siguiente regi�n left.

        if ( indice_left != indice_right ) {

          region_left->final = DECREMENTAR_SECTOR ( region_left->principio );
          region_left->final_ascendente =
            !region_left->principio_ascendente;

        } else {

          region_auxiliar = region_left;

          indice_left = nd->regiones.longitud++;
          region_left = & ( nd->regiones.vector[indice_left] );
          region_left->descartada = FALSE;

          region_left->final = DECREMENTAR_SECTOR ( region_auxiliar->principio );
          region_left->final_ascendente = !region_auxiliar->principio_ascendente;

        }

        SiguienteDiscontinuidad ( nd, region_left->final, LEFT,
                                  & ( region_left->principio ),
                                  & ( region_left->principio_ascendente ) );

        if ( ObjetivoAlcanzable ( nd, region_left,
                                  DIRECTION_DISCONTINUIDAD_FINAL ) ) {
          if ( region_right->principio_ascendente ||
               region_right->final_ascendente ) {
            nd->region = indice_left;
            return;
          }

          if ( indice_right > indice_left ) {
            nd->regiones.longitud--;
            nd->region = indice_left;
            return;
          }

          *region_right = *region_left;

          nd->regiones.longitud--;
          nd->region = indice_right;
          return;
        }

      }

    } else {

      // Probamos por la regi�n right.

      if ( region_right->final_ascendente ) {

        if ( ObjetivoAlcanzable ( nd, region_right,
                                  DIRECTION_DISCONTINUIDAD_FINAL ) ) {
          if ( region_left->principio_ascendente ||
               region_left->final_ascendente ) {
            nd->region = indice_right;
            return;
          }

          if ( indice_left > indice_right ) {
            nd->regiones.longitud--;
            nd->region = indice_right;
            return;
          }

          *region_left = *region_right;

          nd->regiones.longitud--;
          nd->region = indice_left;
          return;
        }

        if ( indice_right != indice_left )
          region_right->descartada = TRUE;

        region_auxiliar = region_right;

        indice_right = nd->regiones.longitud++;

        region_right = & ( nd->regiones.vector[indice_right] );

        region_right->descartada = FALSE;

        region_right->principio = INCREMENTAR_SECTOR ( region_auxiliar->final );

        region_right->principio_ascendente = !region_auxiliar->final_ascendente;

        SiguienteDiscontinuidad ( nd, region_right->principio, RIGHT,
                                  & ( region_right->final ),
                                  & ( region_right->final_ascendente ) );

      } else { // Final descendente: Ser� un principio ascendente en
        // la siguiente regi�n right

        if ( indice_right != indice_left ) {

          region_right->principio = INCREMENTAR_SECTOR ( region_right->final );
          region_right->principio_ascendente = !region_right->final_ascendente;

        } else {

          region_auxiliar = region_right;

          indice_right = nd->regiones.longitud++;
          region_right = & ( nd->regiones.vector[indice_right] );
          region_right->descartada = FALSE;

          region_right->principio = INCREMENTAR_SECTOR ( region_auxiliar->final );
          region_right->principio_ascendente = !region_auxiliar->final_ascendente;

        }

        SiguienteDiscontinuidad ( nd, region_right->principio, RIGHT,
                                  & ( region_right->final ),
                                  & ( region_right->final_ascendente ) );

        if ( ObjetivoAlcanzable ( nd, region_right,
                                  DIRECTION_DISCONTINUIDAD_INICIAL ) ) {
          if ( region_left->principio_ascendente ||
               region_left->final_ascendente ) {
            nd->region = indice_right;
            return;
          }

          if ( indice_left > indice_right ) {
            nd->regiones.longitud--;
            nd->region = indice_right;
            return;
          }

          *region_left = *region_right;

          nd->regiones.longitud--;
          nd->region = indice_left;
          return;
        }

      }

    }

  } while ( ( distancia_left < SECTORES / 2 ) ||
            ( distancia_right < SECTORES / 2 ) );

  // *region_left == *region_right (al menos los campos que determinan
  // la region) y son las dos �ltimas del vector.
  nd->regiones.longitud--;

  nd->regiones.vector[nd->regiones.longitud-1].descartada = TRUE;

#undef LEFT
#undef RIGHT
}

//----------------------------------------------------------------------------
// IterarND / ConstruirDR
static void ConstruirDR ( TInfoND *nd )
{
  int i;

  for ( i = 0; i < SECTORES; i++ ) {
    nd->dr[i] = ( nd->d[i].r < 0.0F ) ? -1.0F : nd->d[i].r - robot.E[i];
  }
}

// ----------------------------------------------------------------------------
// IterarND / control_angle / ObtenerObstaculos / ActualizarMinimo
static void ActualizarMinimo ( int *sector_minimo, float *valor_minimo, int sector, float valor )
{
  if ( ( *sector_minimo == -1 ) || ( valor < *valor_minimo ) ) {
    *sector_minimo = sector;
    *valor_minimo = valor;
  }
}

//---------------------------------------------------------------------------
// IterarND / control_angle / ObtenerObstaculos
static void ObtenerObstaculos ( TInfoND *nd, float beta )
{
  // Buscamos todos los obst�culos que est�n dentro de la distancia de
  // seguridad y nos quedamos
  // con el m�s cercano por la left y el m�s cercano por la right.
  // El obst�culo m�s cercano es el de menor dr/ds.
  // Un obst�culo es por la left si nos limita el giro a la left
  // (nos obliga a
  // rectificar el �ngulo de partida hacia la right para esquivarlo).

#define ACTUALIZAR_OBSTACULO_LEFT ActualizarMinimo(&(nd->obstaculo_left),&min_izq,i,nd->dr[i]/robot.ds[i]);
#define ACTUALIZAR_OBSTACULO_RIGHT ActualizarMinimo(&(nd->obstaculo_right),&min_der,i,nd->dr[i]/robot.ds[i]);

  TCoordenadas p;
  TCoordenadasPolares pp;
  float alfa, angle, min_izq, min_der;
  int i;

  ConstruirCoordenadasCxy ( &p, robot.Dimensiones[0], robot.Dimensiones[1] );
  ConstruirCoordenadasPcC ( &pp, p );
  alfa = pp.a;

  nd->obstaculo_left = -1;
  nd->obstaculo_right = -1;

  for ( i = 0; i < SECTORES; i++ ) {
    if ( ( nd->dr[i] >= 0.0F ) && ( nd->dr[i] <= robot.ds[i] ) ) {
      angle = nd->d[i].a;
      /*       if (AngleNormalizado(angle-beta)>=0) */

      if ( angle >= beta )
        ACTUALIZAR_OBSTACULO_LEFT
        else
          ACTUALIZAR_OBSTACULO_RIGHT

        }
  }
#undef ACTUALIZAR_OBSTACULO_LEFT
#undef ACTUALIZAR_OBSTACULO_RIGHT
}
//---------------------------------------------------------------------------
// IterarND / control_angle / solHSGR
static float solHSGR ( TInfoND *nd )
{
  return nd->regiones.vector[nd->region].direction_angle;
}

//---------------------------------------------------------------------------
// IterarND / control_angle / solHSNR
static float solHSNR ( TInfoND *nd )
{
  TRegion *region = & ( nd->regiones.vector[nd->region] );
  int final = region->final;

  if ( region->principio > region->final )
    final += SECTORES;

  return sector2angle ( ( ( region->principio + final ) / 2 ) % SECTORES );
}

//---------------------------------------------------------------------------
// IterarND / control_angle / solHSWR
static float solHSWR ( TInfoND *nd )
{
  TRegion *region = & ( nd->regiones.vector[nd->region] );

  if ( region->direction_tipo == DIRECTION_DISCONTINUIDAD_INICIAL ) {
    return ( nd->d[DECREMENTAR_SECTOR ( region->principio ) ].a
             - ( float ) atan2 ( ( robot.discontinuidad / 2.0F + robot.ds[SECTORES/2] ),
                                 nd->d[DECREMENTAR_SECTOR ( region->principio ) ].r ) );
  } else {
    return ( nd->d[INCREMENTAR_SECTOR ( region->final ) ].a
             + ( float ) atan2 ( ( robot.discontinuidad / 2.0F + robot.ds[SECTORES/2] ),
                                 nd->d[INCREMENTAR_SECTOR ( region->final ) ].r ) );
  }
}
//-----------------------------------------------------------------------------
// IterarND / control_angle / solLS1
static float solLS1 ( TInfoND *nd )
{
  TRegion *region = & ( nd->regiones.vector[nd->region] );
  //float angle_objetivo=nd->regiones.vector[nd->region].direction_angle;
  float anglePrueba;

  float angle_parcial, dist_obs_dsegur, angle_cota;
  int final = region->final;

  if ( region->principio > region->final )
    final += SECTORES;

  if ( final - nd->regiones.vector[nd->region].principio > SECTORES / 4 ) {
    if ( region->direction_tipo == DIRECTION_DISCONTINUIDAD_INICIAL )
      angle_parcial = nd->d[DECREMENTAR_SECTOR ( region->principio ) ].a
                       - ( float ) atan2 ( ( robot.discontinuidad / 2 +
                                             robot.ds[SECTORES/2] ),
                                           nd->d[DECREMENTAR_SECTOR (
                                                   region->principio ) ].r );
    else
      angle_parcial = nd->d[INCREMENTAR_SECTOR ( region->final ) ].a
                       + ( float ) atan2 ( ( robot.discontinuidad / 2 +
                                             robot.ds[SECTORES/2] ),
                                           nd->d[INCREMENTAR_SECTOR (
                                                   region->final ) ].r );
  } else
    angle_parcial = sector2angle ( ( ( region->principio + final ) / 2 ) % SECTORES );

  if ( nd->obstaculo_left != -1 ) {
    angle_cota = AngleNormalizado ( nd->d[nd->obstaculo_left].a + PI -
                                      angle_parcial ) + angle_parcial;
    dist_obs_dsegur = nd->dr[nd->obstaculo_left] /
                      robot.ds[nd->obstaculo_left];
  } else {
    angle_cota = AngleNormalizado ( nd->d[nd->obstaculo_right].a + PI -
                                      angle_parcial ) + angle_parcial;
    dist_obs_dsegur = nd->dr[nd->obstaculo_right] / robot.ds[nd->obstaculo_right];
  }

  // Codigo Osuna
  //return AngleNormalizado(angle_parcial * dist_obs_dsegur  + angle_cota * (1-dist_obs_dsegur));

  // Codigo Minguez
  anglePrueba = angle_parcial * dist_obs_dsegur  + angle_cota * ( 1 - dist_obs_dsegur );


  if ( anglePrueba > M_PI )
    anglePrueba = ( float ) ( M_PI - 0.01 );
  else
    if ( anglePrueba < -M_PI )
      anglePrueba = - ( float ) ( M_PI + 0.01 );

  return AngleNormalizado ( anglePrueba );

}
//-----------------------------------------------------------------------------
// IterarND / control_angle / solLSG

static float solLSG ( TInfoND *nd )
{

  float angle_parcial, dist_obs_dsegur, angle_cota, anglePrueba;

  angle_parcial = nd->regiones.vector[nd->region].direction_angle;

  if ( nd->obstaculo_left != -1 ) {
    // something about left
    angle_cota = AngleNormalizado ( nd->d[nd->obstaculo_left].a + PI - angle_parcial ) + angle_parcial;
    dist_obs_dsegur = nd->dr[nd->obstaculo_left] / robot.ds[nd->obstaculo_left];
  } else {
    // something about right
    angle_cota = AngleNormalizado ( nd->d[nd->obstaculo_right].a + PI - angle_parcial ) + angle_parcial;
    dist_obs_dsegur = nd->dr[nd->obstaculo_right] / robot.ds[nd->obstaculo_right];
  }

  // Codigo Osuna
  // return AngleNormalizado(angle_parcial * dist_obs_dsegur  + angle_cota * (1-dist_obs_dsegur));

  anglePrueba = angle_parcial * dist_obs_dsegur  + angle_cota * ( 1.0f - dist_obs_dsegur );



  // Codigo Minguez
  if ( anglePrueba > M_PI )
    anglePrueba = ( float ) ( M_PI - 0.01 );
  else
    if ( anglePrueba < -M_PI )
      anglePrueba = - ( float ) ( M_PI + 0.01 );

  return AngleNormalizado ( anglePrueba );
}
//-----------------------------------------------------------------------------
// IterarND / control_angle / solLS2
static float solLS2 ( TInfoND *nd )
{
  float ci = nd->dr[nd->obstaculo_left] / robot.ds[nd->obstaculo_left];
  float cd = nd->dr[nd->obstaculo_right] / robot.ds[nd->obstaculo_right];
  float ad, ai; // �ngulos cota izquierdo y derecho.
  float ang_par = nd->regiones.vector[nd->region].direction_angle;

  /*
    ad = AngleNormalizado(nd->d[nd->obstaculo_right].a+PI-ang_par)+ang_par;

    ai = AngleNormalizado(nd->d[nd->obstaculo_left].a-PI-ang_par)+ang_par;

    return AngleNormalizado((ad+ai)/2.0F+(ci-cd)/(ci+cd)*(ad-ai)/2.0F);
  */
  ad = M_PI / 2.0F;
  ai = -M_PI / 2.0F;

  if ( ci <= cd )
    return AngleNormalizado ( ang_par + ( ci - cd ) / ( ci + cd ) * ( ang_par - ai ) );
  else
    return AngleNormalizado ( ang_par + ( ci - cd ) / ( ci + cd ) * ( ad - ang_par ) );
}

//-----------------------------------------------------------------------------
// IterarND / control_angle
static void control_angle ( TInfoND *nd )
{
  // C�lculo del �ngulo de movimiento en funci�n de la regi�n escogida para el
  // movimiento del robot, la situaci�n del objetivo y,
  // en su caso, la distancia a los obst�culos m�s pr�ximos.

  TRegion *region = & ( nd->regiones.vector[nd->region] );
  int final = region->final;

  if ( region->principio > region->final )
    final += SECTORES;

  ObtenerObstaculos ( nd, nd->regiones.vector[nd->region].direction_angle );

  if ( ( nd->obstaculo_left == -1 ) && ( nd->obstaculo_right == -1 ) ) {
    if ( region->direction_tipo == DIRECTION_OBJETIVO ) {
      sprintf ( nd->situacion, "HSGR" ); // High Safty Goal in Region
      nd->anglesin = solHSGR ( nd );
      nd->angle = nd->anglesin;
    } else
      if ( final - nd->regiones.vector[nd->region].principio > SECTORES / 4 ) {
        sprintf ( nd->situacion, "HSWR" ); // High Safty Wide Region
        nd->anglesin = solHSWR ( nd );
        nd->angle = nd->anglesin;
      } else {
        sprintf ( nd->situacion, "HSNR" );
        nd->anglesin = solHSNR ( nd ); // High Safty Narrow Region
        nd->angle = nd->anglesin;
      }
  } else {
    if ( ( nd->obstaculo_left != -1 ) && ( nd->obstaculo_right != -1 ) ) {
      sprintf ( nd->situacion, "LS2" ); // Low Safety 2
      nd->angle = solLS2 ( nd );
      nd->anglesin = nd->angle;
    } else
      if ( region->direction_tipo == DIRECTION_OBJETIVO ) {
        sprintf ( nd->situacion, "LSG" );
        nd->angle = solLSG ( nd );
        nd->anglesin = nd->angle;
      } else {
        sprintf ( nd->situacion, "LS1" );
        nd->angle = solLS1 ( nd ); // Low Safety 1
        nd->anglesin = nd->angle;
      }
  }
  // limit angle
  AplicarCotas ( & ( nd->angle ), -PI , PI );
  //AplicarCotas ( & ( nd->angle ), -PI / 2.0F, PI / 2.0F );
  //nd->angle = NORMALIZE_TO_2PI ( nd->angle );
}

//----------------------------------------------------------------------------
// IterarND / control_velocidad
static void control_velocidad ( TInfoND *nd )
{
  // Velocidad lineal del robot.

  // Coeficiente de distancia por la left.
  float ci = ( nd->obstaculo_left != -1 ) ? nd->dr[nd->obstaculo_left] /
             robot.ds[nd->obstaculo_left] : 1.0F;
  // Coeficiente de distancia por la right.
  float cd = ( nd->obstaculo_right != -1 ) ? nd->dr[nd->obstaculo_right] /
             robot.ds[nd->obstaculo_right] : 1.0F;

  nd->velocidad = robot.velocidad_lineal_maxima * MINIMO ( ci, cd );
}

//----------------------------------------------------------------------------
// Cutting / GenerarMovimientoFicticio
static void GenerarMovimientoFicticio ( TInfoND *nd, float angle, TVelocities *velocidades )
{

  // shutup compiler
  angle = angle;

  float ci = ( nd->obstaculo_left != -1 ) ? nd->dr[nd->obstaculo_left] /
             robot.ds[nd->obstaculo_left] : 1.0F;
  // Coeficiente de distancia por la left.
  float cd = ( nd->obstaculo_right != -1 ) ? nd->dr[nd->obstaculo_right] /
             robot.ds[nd->obstaculo_right] : 1.0F;
  // Coeficiente de distancia por la right.
  float cvmax = MAXIMO ( 0.2F, MINIMO ( ci, cd ) );

  velocidades->v = robot.velocidad_lineal_maxima * cvmax * ( float ) cos ( nd->angle );
  // Calculada en SR2C.
  velocidades->w = robot.velocidad_angular_maxima * cvmax * ( float ) sin ( nd->angle );
  // Calculada en SR2C.
//  fprintf(depuracion,"%d: <a,ci,cd,cvmax,v,w>=<%f,%f,%f,%f,%f,%f>\n",++iteracion,nd->angle,ci,cd,cvmax,velocidades->v,velocidades->w);

  //printf ( "ND: %s angle %f cvmax %f region %d \n",nd->situacion, R2D ( nd->angle ), cvmax, nd->region );
  // limit velocities
  AplicarCotas ( & ( velocidades->v ), 0.0F, robot.velocidad_lineal_maxima );
  AplicarCotas ( & ( velocidades->w ), -robot.velocidad_angular_maxima,
                 robot.velocidad_angular_maxima );

  /*

    #define FMAX robot.aceleracion_lineal_maxima

    TCoordenadas F;

    ConstruirCoordenadasCra(&F,FMAX*nd->velocidad/robot.velocidad_lineal_maxima,angle);

    velocidades->v=robot.H[0][0]*nd->velocidades.v+robot.G[0][0]*F.x;
    velocidades->w=robot.H[1][1]*nd->velocidades.w+robot.G[1][1]*F.y;

    #undef FMAX
  */
}
//---------------------------------------------------------------------------
// GiroBrusco
static void GiroBrusco ( TInfoND *nd, TVelocities *velocidades )
{
  TCoordenadasPolares esquina;
  int right, left;

  ConstruirCoordenadasPxy ( &esquina, robot.Dimensiones[2], robot.Dimensiones[1] );
  right = ( nd->obstaculo_right != -1 ) && ( ( float )
            fabs ( nd->d[nd->obstaculo_right].a ) <= esquina.a ) &&
            ( nd->d[nd->obstaculo_right].r <= esquina.r + robot.enlarge );
  left = ( nd->obstaculo_left != -1 ) && ( ( float )
              fabs ( nd->d[nd->obstaculo_left].a ) <= esquina.a ) &&
              ( nd->d[nd->obstaculo_left].r <= esquina.r + robot.enlarge );

  if ( right && left ) {
    velocidades->w = 0.0F;
//    printf("giro brusco central\n");
    return;
  }

  if ( right || left ) {
    velocidades->v = 0.0F;
//    printf("giro brusco lateral\n");
  }
}

//---------------------------------------------------------------------------
// Cutting / ObtenerSituacionCutting

#define CUTTING_NINGUNO   0
#define CUTTING_LEFT 1
#define CUTTING_RIGHT   2
#define CUTTING_AMBOS     3

static int ObtenerSituacionCutting ( TInfoND *nd, float w )
{
  TCoordenadas p;
  int resultado = CUTTING_NINGUNO;
  int obstaculo_left = 0;
  int obstaculo_right = 0;
  int i;

  resultado = CUTTING_NINGUNO;

  i = 0;

  // shutup compiler
  w = w;

  while ( i < SECTORES ) {
    if ( ( ( nd->d[i].a < -PI / 2.0F ) || ( nd->d[i].a > PI / 2.0F ) )
         && ( nd->d[i].r >= 0.0F ) && ( nd->dr[i] <= robot.enlarge / 2.0F ) ) {

      ConstruirCoordenadasCP ( &p, nd->d[i] );

      if ( p.y >= robot.Dimensiones[1] ) { // Obst�culo a la left.

        if ( obstaculo_right )
          return CUTTING_AMBOS;

        obstaculo_left = 1;

        resultado = CUTTING_LEFT;

      } else
        if ( p.y <= robot.Dimensiones[3] ) { // Obst�culo a la right.

          if ( obstaculo_left )
            return CUTTING_AMBOS;

          obstaculo_right = 1;

          resultado = CUTTING_RIGHT;

        } else
          if ( p.x <= robot.Dimensiones[0] ) // Obst�culo detr�s.
            return CUTTING_AMBOS;
    }

    i++;

    if ( i == SECTORES / 4 + 1 )
      i = 3 * SECTORES / 4;
  }

  return resultado;
}
//---------------------------------------------------------------------------
// Cutting / AngleSinRotacion
static float AngleSinRotacion ( TInfoND *nd, TVelocities *velocidades )
{
  TCoordenadas F;
  float angle;

  if ( robot.aceleracion_angular_maxima*robot.T < fabs ( nd->velocidades.w ) ) {
    velocidades->w = ( nd->velocidades.w > 0 ) ? nd->velocidades.w -
                     robot.aceleracion_angular_maxima * robot.T : nd->velocidades.w +
                     robot.aceleracion_angular_maxima * robot.T;

    if ( robot.aceleracion_lineal_maxima*robot.T < nd->velocidades.v )
      velocidades->v = nd->velocidades.v - robot.aceleracion_lineal_maxima * robot.T;
    else
      velocidades->v = 0.0f;
  } else {
    velocidades->v = nd->velocidad;
    velocidades->w = 0.0F;
  }

  F.x = ( velocidades->v - robot.H[0][0] * nd->velocidades.v ) / robot.G[0][0];

  F.y = ( velocidades->w - robot.H[1][1] * nd->velocidades.w ) / robot.G[1][1];
  angle = ARCOTANGENTE ( F.x, F.y );


  return angle;
}

//---------------------------------------------------------------------------
// Cutting

static void Cutting ( TInfoND *nd, TVelocities *velocidades )
{
  switch ( ObtenerSituacionCutting ( nd, velocidades->w ) ) {

    case CUTTING_NINGUNO:
      sprintf ( nd->cutting, "NINGUNO" );
      return;

    case CUTTING_LEFT:
      sprintf ( nd->cutting, "LEFT" );

      if ( velocidades->w >= 0.0F )
        return;

      break;

    case CUTTING_RIGHT:
      sprintf ( nd->cutting, "RIGHT" );

      if ( velocidades->w <= 0.0F )
        return;

      break;

    case CUTTING_AMBOS:
      sprintf ( nd->cutting, "AMBOS" );
  }

  nd->angle = AngleSinRotacion ( nd, velocidades );
}

#undef CUTTING_NINGUNO
#undef CUTTING_LEFT
#undef CUTTING_RIGHT
#undef CUTTING_AMBOS

// ----------------------------------------------------------------------------
// IterarND / GenerarMovimiento

/* Unused
static void GenerarMovimiento(TInfoND *nd,TVelocities *velocidades) {
  #define FMAX robot.aceleracion_lineal_maxima

  TCoordenadas F;

  ConstruirCoordenadasCra(&F,FMAX*nd->velocidad/robot.velocidad_lineal_maxima,nd->angle);

  velocidades->v=robot.H[0][0]*nd->velocidades.v+robot.G[0][0]*F.x;
  velocidades->w=robot.H[1][1]*nd->velocidades.w+robot.G[1][1]*F.y;

//   printf("v= %f \n w= %f \n",velocidades->v,velocidades->w);


  AplicarCotas(&(velocidades->v),0.0F,robot.velocidad_lineal_maxima);
  AplicarCotas(&(velocidades->w),-robot.velocidad_angular_maxima,robot.velocidad_angular_maxima);

  #undef FMAX
}
*/

// ----------------------------------------------------------------------------

// IterarND

TVelocities *IterarND ( TCoordenadas objetivo,
                        float goal_tol,
                        TInfoMovimiento *movimiento,
                        TInfoEntorno *mapa, void* info )
{

  // Devuelve NULL si se requiere una parada de emergencia o si no encuentra
  // una regi�n por la que hacer avanzar el robot.
  // Devuelve un puntero a (0.0F,0.0F) si se ha alcanzado el objetivo.
  TInfoND* nd = (TInfoND*)info;

  // Valgrind says that some of the values in this nd structure are
  // uninitialized when it's accessed in ObtenerSituacionCutting(), so I'm
  // zeroing it here.  - BPG
  memset ( nd, 0, sizeof ( TInfoND ) );

  // depuracion=fopen("depuracion.txt","at");
  // Tratamiento de los par�metros "objetivo" y "movimiento".

  nd->objetivo.c0 = objetivo; // goal location
  nd->SR1 = movimiento->SR1; // robot position
  nd->velocidades = movimiento->velocidades;

  nd->objetivo.c1 = nd->objetivo.c0; // goal location

  // Transform goal to robot local coordinate system, translate and rotate
  TRANSFORMACION01 ( & ( nd->SR1 ), & ( nd->objetivo.c1 ) )

  // convert goal to polar coordinates
  ConstruirCoordenadasPC ( & ( nd->objetivo.p1 ), nd->objetivo.c1 );

  // calculate sector from goal polar coordinates
  nd->objetivo.s = ObtenerSectorP ( nd->objetivo.p1 );
  // Sectorizaci�n del mapa
  SectorizarMapa ( mapa, nd );

  // Evaluaci�n de la necesidad de una parada de emergencia
  // Solo en el caso de robot rectangular
  if ( robot.geometriaRect == 1 )
    if ( ParadaEmergencia ( nd ) ) {
      PRT_MSG0 ( 6, "ND: Emergency Shutdown" );
      return 0;
    }

  // Selecci�n de la regi�n por la cual avanzar� el robot
  SeleccionarRegion ( nd );

  if ( nd->region < 0 ) {
    PRT_ERR0 ( "ND: Cannot find region" );
    return 0;
  }

  // Construcci�n de la distancia desde el per�metro del robot al obst�culo m�s
  // cercano en cada sector
  ConstruirDR ( nd );

  // Deteccion de fin de trayecto. -- Despu�s de considerar la necesidad de una
  // parada de emergencia
  // Caso geometria rectangular
  if ( robot.geometriaRect == 1 ) {
    // Replaced this check with the user-specified goal tolerance - BPG
    /*
    // Cuadrado
    if ((nd->objetivo.c1.x>=robot.Dimensiones[0]) &&
       (nd->objetivo.c1.x<=robot.Dimensiones[2]) &&
       (nd->objetivo.c1.y>=robot.Dimensiones[3]) &&
       (nd->objetivo.c1.y<=robot.Dimensiones[1])) {
        */
    if ( hypot ( objetivo.x - movimiento->SR1.posicion.x,
                 objetivo.y - movimiento->SR1.posicion.y ) < goal_tol ) {
      // Ya hemos llegado.
      velocidades.v = 0.0F;
      velocidades.w = 0.0F;
      return &velocidades;
    }
  } else
    if ( ( CUADRADO ( nd->objetivo.c1.x ) + CUADRADO ( nd->objetivo.c1.y ) )
         < CUADRADO ( robot.R ) ) {
      // Redondo
      velocidades.v = 0.0F;
      velocidades.w = 0.0F;
      return &velocidades;
    }

  // C�lculo del movimiento del robot
  control_angle ( nd );   // Obtenci�n de la direcci�n de movimiento.

  control_velocidad ( nd ); // Obtenci�n de la velocidad de movimiento.

//printf("speed %f %f \n",nd->velocidades.v, nd->velocidades.w);
  //  if (nd->velocidad<0.05F)
  //    nd->velocidad=0.05F;
  nd->velocidad = robot.velocidad_lineal_maxima;

  // Hasta aqui es el ND standart


  // En funcion del tipo de robot.
  if ( robot.holonomo ) { // HOLONOMIC ROBOT
    // ya se han aplicado cotas al angle
    // printf("Movimiento Holonomo\n");
    // velocidades.v=
    //   nd->velocidad*fabs(DistanciaAngular(fabs(nd->angle),M_PI/2))/(M_PI/2);
    velocidades.v = nd->velocidad * ( float ) fabs ( AmplitudAngleNoOrientado ( ( float )
                    fabs ( nd->angle ), M_PI / 2 ) ) / ( M_PI / 2 );
    /*     velocidades.v= nd->velocidad; */
    /*     velocidades.w=0.4; */
    //    velocidades.w=
    //  (M_PI/2-DistanciaAngular(fabs(nd->angle),M_PI/2))/(M_PI/2)*
    //  robot.velocidad_angular_maxima;
    velocidades.w = nd->angle / ( M_PI / 2 ) * robot.velocidad_angular_maxima;
    velocidades.v_theta = nd->angle;
    /*    if (nd->angle<0) {
          velocidades.w=-velocidades.w;
       printf("vdsd�f\n");
     }
    */

  } else { // NON-HOLONOMIC ROBOT
    velocidades.v_theta = 0.0F;
    // printf("Movimiento No Holonomo\n");
    // Calculo del movimiento Generador de movimientos
    GenerarMovimientoFicticio ( nd, nd->angle, &velocidades );
    // Aplicar correcciones al movimiento calculado.

    if ( robot.geometriaRect == 1 ) {
      // Cuadrado
      GiroBrusco ( nd, &velocidades );
      /*       printf("Entra en cutting %d\n",robot.geometriaRect); */
      Cutting ( nd, &velocidades ); // Evitar colisi�n en zona posterior.
    }
  }

  // limit speed commands
  AplicarCotas ( &velocidades.v,
                 0.0F,
                 robot.velocidad_lineal_maxima );
  AplicarCotas ( &velocidades.w,
                 -robot.velocidad_angular_maxima,
                 robot.velocidad_angular_maxima );

  /*     printf("w = %f\n",velocidades.w);   */

  return &velocidades;
}
