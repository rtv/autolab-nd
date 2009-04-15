/***************************************************************************
 * Project: RAPI                                                           *
 * Author:  Jens Wawerla (jwawerla@sfu.ca)                                 *
 * $Id: utilities.cpp,v 1.3 2009-04-03 15:10:02 jwawerla Exp $
 ***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************
 * $Log: utilities.cpp,v $
 * Revision 1.3  2009-04-03 15:10:02  jwawerla
 * *** empty log message ***
 *
 * Revision 1.2  2009-03-20 03:05:23  jwawerla
 * dynamic obstacles added to wavefront
 *
 * Revision 1.1.1.1  2009-03-15 03:52:02  jwawerla
 * First commit
 *
 * Revision 1.3  2008/12/13 00:29:06  jwawerla
 * Fixed some random number issues
 *
 * Revision 1.2  2008/12/04 01:41:19  jwawerla
 * Snapshot
 *
 * Revision 1.1.1.1  2008/04/10 21:26:30  jwawerla
 * rapi3
 *
 * Revision 1.1.1.1  2008/02/02 22:19:54  jwawerla
 * new to cvs
 *
 * Revision 1.2  2008/01/11 02:05:17  jwawerla
 * Added local coordinate system
 *
 * Revision 1.1.1.1  2008/01/10 19:44:01  jwawerla
 * first time in cvs
 *
 *
 ***************************************************************************/

#include "utilities.h"
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>

//---------------------------------------------------------------------------
float normalDistribution ( float a, float b )
{
  return 1 / ( sqrt ( TWOPI*b*b ) ) * exp ( -0.5* a*a / ( b*b ) );
}
//---------------------------------------------------------------------------
float triangularDistribution ( float a, float b )
{
  return MAX ( 0, 1 / ( sqrt ( 6 ) *b ) - fabs ( a ) / ( 6*b*b ) );
}
//---------------------------------------------------------------------------
void initRandomNumberGenerator()
{
  srand ( time ( NULL ) + getpid() );
  srand48 ( time ( NULL ) + getpid() );
}
//------------------------------------------------------------------------
int randNo ( int minimum, int maximum )
{
  return ( int ) LIMIT ( minimum + drand48() * ( maximum - minimum ),
                         minimum, maximum );
}
//-----------------------------------------------------------------------------
double randNo ( double minimum, double maximum )
{
  double r;

  // RAND_MAX = 2147483647
  r = drand48();  // r = [0,1]

  return minimum + r * ( maximum - minimum );
}
//-----------------------------------------------------------------------------
double randOne()
{
  return ( double ) rand() / ( double ) ( RAND_MAX );
}
//-----------------------------------------------------------------------------
void printBottomLine ( char *str )
{
  printf ( "\033[25;1H%s", str );
  fflush ( stdout );
}
//-----------------------------------------------------------------------------
double normalRand ( double mu, double sigma )
{
  // this code is from http://www.dreamincode.net/code/snippet1446.htm

  double dist;
  double angle;
  // choose a pair of uniformly distributed deviates, one for the
  // distance and one for the angle, and perform transformations
  dist = sqrt ( -2.0 * log ( drand48() ) );
  angle = 2.0 * PI * ( drand48() );

  // calcaulate return second deviate
  return dist * sin ( angle ) * sigma + mu;
}
//-----------------------------------------------------------------------------

