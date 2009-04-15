
/***************************************************************************
 * Desc: Error handling macros
 * Author: Andrew Howard
 * Date: 13 May 2002
 * CVS: $Id: error.c,v 1.1.1.1 2009-03-15 03:52:02 jwawerla Exp $
 **************************************************************************/

#include <stdarg.h>
#include "error.h"

#ifdef WIN32
#include <conio.h>
#endif

// User-selected msg level: 0 for the most important messages (always
// printed); 9 for the least important.
static int msgLevel;

// File for logging messages
static FILE *msgFile = NULL;


/**
 * Initialize error logging
 * @param _msgLevel 0-10 
 * @param fgWriteLogFile 1 - write logging to file else no file is written 
 */
void ErrorInit( int _msgLevel, int fgWriteLogFile )
{
  msgLevel = _msgLevel;
  if (fgWriteLogFile == 1) {
    msgFile = fopen( "./rapi.log", "a+" );
  }
  else {
    msgFile = NULL;
  }
}


// Function for printing and logging errors.
void ErrorPrint( int msgType, int level, const char *file, int line,
                 const char *fmt, ... )
{
  va_list ap;

#ifdef WIN32
  if ( level <= msgLevel ) {
    va_start( ap, fmt );
    vprintf( fmt, ap );
    printf( "    %s:%d \n", file, line );
    va_end( ap );
  }
#else
  if ( level <= msgLevel ) {
    va_start( ap, fmt );
    vfprintf( stderr, fmt, ap );
    fprintf( stderr, "    %s:%d \n", file, line );
    va_end( ap );
  }
#endif
  if ( msgFile ) {
    fprintf( msgFile, "%d %s:%d ", msgType, file, line );
    va_start( ap, fmt );
    vfprintf( msgFile, fmt, ap );
    va_end( ap );
  }

  return;
}
