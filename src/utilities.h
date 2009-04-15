/***************************************************************************
 * Project: RAPI                                                           *
 * Author:  Jens Wawerla (jwawerla@sfu.ca)                                 *
 * $Id: utilities.h,v 1.7 2009-04-08 22:40:41 jwawerla Exp $
 ***************************************************************************
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
 * $Log: utilities.h,v $
 * Revision 1.7  2009-04-08 22:40:41  jwawerla
 * Hopefully ND interface issue solved
 *
 * Revision 1.6  2009-04-03 15:10:02  jwawerla
 * *** empty log message ***
 *
 * Revision 1.5  2009-03-31 23:52:59  jwawerla
 * Moved cell index from float to int math
 *
 * Revision 1.4  2009-03-29 00:54:27  jwawerla
 * Replan ctrl seems to work now
 *
 * Revision 1.3  2009-03-26 04:09:45  jwawerla
 * Minor bug fixing
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
 * Revision 1.3  2008/01/17 23:41:11  jwawerla
 * fixed some bugs
 *
 * Revision 1.2  2008/01/11 02:05:17  jwawerla
 * Added local coordinate system
 *
 * Revision 1.1.1.1  2008/01/10 19:44:00  jwawerla
 * first time in cvs
 *
 *
 ***************************************************************************/

#ifndef _UTILITIES_H_
#define _UTILITIES_H_

#include <sys/time.h>
#include <math.h>
#include <limits>

#ifndef INFINITY
  #define INFINITY infinity()
#endif

#ifndef SEC_PER_HOUR
  #define SEC_PER_HOUR 3600.0
#endif

/** Figure out the sign of a number */
#ifndef SIGN
  #define SIGN(x) (((x) == 0) ? 0 : (((x) > 0) ? 1 : -1))
#endif

/** Well... ?! */
#ifndef PI
  #define PI 3.14159265358979323846
#endif

/** pi/2 */
#ifndef HALFPI
  #define HALFPI PI/2.0
#endif

/** 2 times pi = 6.28... */
#ifndef TWOPI
  #define TWOPI 6.28318530717958
#endif

/** Converts from joules to watt hours */
#ifndef JOULE_TO_WATTHOURS
  #define JOULE_TO_WATTHOURS(x) x / 3600.0f
#endif

/**
 * Conversts x from radians to degree
 * @param x value [rad]
 * @return [deg]
 */
#ifndef R2D
  #define R2D(x) x*57.2957795
#endif

/**
 * Calculates euclidian distance
 * @return distance
 */
#ifndef EUCLIDIAN
  #define EUCLIDIAN(x, y, a, b) sqrt( (x-a)*(x-a) + (y-b)*(y-b))
#endif

/**
 * Conversts x from degree to radians
 * @param x value [deg]
 * @return [rad]
 */
#ifndef D2R
  #define D2R(x) x*0.01745329
#endif

/**
 * Maximum value of a and b
 * @param a
 * @param b
 * @return max of a and b
 */
#ifndef MAX
  #define MAX(a,b) ((a > b) ? (a) : (b))
#endif

/**
 * Minimum value of a and b
 * @param a
 * @param b
 * @return min of a and b
 */
#ifndef MIN
  #define MIN(a,b) ((a < b) ? (a) : (b))
#endif

/**
 * Normalizes the value z to be in the interval [-pi, pi]
 * @param z to be normalized
 * @return normalized value
 */
#ifndef NORMALIZE_ANGLE
  #define NORMALIZE_ANGLE(z) atan2(sin(z), cos(z))
#endif

/**
 * Limits the value x to be in the interval [a, b]
 * @param a lower limit
 * @param b upper limit
 * @return limited value
 */
#ifndef LIMIT
  #define LIMIT(x,a,b) MIN(MAX(x,a), b)
#endif

/**
 * Rounds a number
 */
#define ROUND(x) floor(x+0.5)

/**
 * Calculates the square of x
 * @param x
 * @return x^2
 */
template<typename T>
inline T pow2(T x) { return x*x; };

/**
 * Checks if a x is infinity or not
 * @return true is x is infinity, false otherwise
 */
template<typename T>
inline bool isNan ( T value )
{
  return value != value;
}

/**
 * Checks if a x is nan or not
 * @return true is x is nan, false otherwise
 */
template<typename T>
inline bool isInf ( T value )
{
  return std::numeric_limits<T>::has_infinity &&
         value == std::numeric_limits<T>::infinity();
}

/**
 * Checks if x and y are about equal, that is within epsilon
 * @param x
 * @param y
 * @param epslion
 * @return true if about equal, false otherwise
 */
template<typename A, typename B>
inline bool epsilonEqual(A x, A y, B epsilon)
{
  if ( fabs ( x - y ) < epsilon )
    return true;

  return false;
}
/**
 * fmod(40, 10) = 10, which is not correct, it should be 0
 * This function fixes this problem
 * @param x
 * @param y
 * @return fmod(x,y)
 */
template<typename T>
inline T fmodCorrect(T x, T y)
{
  T fm;

  fm = fmod ( x, y );
  if ( epsilonEqual ( fm, y, 0.00001 ) )
    return 0.0;

  return fm;
}
/**
 * Generates a time stamp
 * @return [s]
 */
inline double timeStamp()
{
  struct timeval tv;
  gettimeofday ( &tv, 0 );
  return tv.tv_sec + tv.tv_usec * 1e-6;
}
/**
 * ARM doesn't have round, so here it is
 * @param x value to be rounded
 * @return rounded x
 */
inline double roundArm (double x)
{
  int x_int = ( int ) ( x + 0.5 );
  return ( x_int );
}
/**
 * Calculates the value for a of a zero centered normal distribution
 * @param a value to calculate
 * @param b standard deviation
 */
float normalDistribution(float a, float b);


/**
 * Calculates the value for a of a zero centered triangular distribution
 * @param a value to calculate
 * @param b standard deviation
 */
float triangularDistribution(float a, float b);

/**
 * \fn int randNo(int min, int max)
 * This function generates a random integer number in [min, max]
 * @param min lower bound of number generated
 * @param max upper bound of number generated
 * @return random number [min, max]
 */
int randNo(int min, int max);

/**
 * \fn int randNo(int min, int max)
 * This function generates a random floating point number in [min, max]
 * @param min lower bound of number generated
 * @param max upper bound of number generated
 * @return random number [min, max]
 */
double randNo(double min, double max);

/**
 * \fn double randOne()
 * Generates a uniform random number between 0.0 and 1.0
 */
double randOne();

/**
 * \fn void initRandomNumberGenerator()
 * Initializes the random number generator
 */
void initRandomNumberGenerator();
/**
 * \fn void printBottomLine()
 * Prints str at the bottom line of a xterm
 */
void printBottomLine(char *str);
/**
 * \fn double normalRand(double mu, double sigma)
 * Generates a random number for the normal distribution (mu, sigma)
 * @return normal distributed random number
 */
double normalRand(double mu, double sigma);

#endif

