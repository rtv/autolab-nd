
/***************************************************************************
 * Desc: Error handling macros
 * Author: Andrew Howard
 * Date: 13 May 2002
 * CVS: $Id: error.h,v 1.1.1.1 2009-03-15 03:52:02 jwawerla Exp $
 **************************************************************************/

#ifndef _ERROR_H_
#define _ERROR_H_

#include <stdio.h>
#include <errno.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize error logging
 * @param _msgLevel 0-10 
 * @param fgWriteLogFile 1 - write logging to file else no file is written 
 */
void ErrorInit(int _msgLevel, int fgWriteLogFile);

/// @internal Function for print and logging errors.  Do not call this
/// function directly; use the macros below.
void ErrorPrint(int msgType, int level, const char *file, int line, const char *fmt, ...);

#ifdef __cplusplus
}
#endif


/// @internal Message types (internal use only; code should use the macros)
#define PRT_ERR_ERR 0
#define PRT_ERR_WARN 1
#define PRT_ERR_MSG 2
#define PRT_ERR_DBG 2


#ifndef WIN32
//FOR UNIX      
/// Error message macros
#define PRT_ERR0(msg)              ErrorPrint(PRT_ERR_ERR, 0, __FILE__, __LINE__, "\033[1;41;32mERROR\033[0m:" msg "\n")
#define PRT_ERR1(msg, a)           ErrorPrint(PRT_ERR_ERR, 0, __FILE__, __LINE__, "\033[1;41;32mERROR\033[0m:" msg "\n", a)
#define PRT_ERR2(msg, a, b)        ErrorPrint(PRT_ERR_ERR, 0, __FILE__, __LINE__, "\033[1;41;32mERROR\033[0m:" msg "\n", a, b)
#define PRT_ERR3(msg, a, b, c)     ErrorPrint(PRT_ERR_ERR, 0, __FILE__, __LINE__,  "\033[1;41;32mERROR\033[0m:" msg "\n", a, b, c)
#define PRT_ERR4(msg, a, b, c, d)   ErrorPrint(PRT_ERR_ERR, 0, __FILE__, __LINE__,  "\033[1;41;32mERROR\033[0m:" msg "\n", a, b, c, d)
#define PRT_ERR5(msg, a, b, c, d, e)   ErrorPrint(PRT_ERR_ERR, 0, __FILE__, __LINE__,  "\033[1;41;32mERROR\033[0m:" msg "\n", a, b, c, d, e)

/// Warning message macros
#define PRT_WARN0(msg)             ErrorPrint(PRT_ERR_WARN, 0, __FILE__, __LINE__, "\033[1;37;44mWARNING\033[0m: " msg "\n")
#define PRT_WARN1(msg, a)          ErrorPrint(PRT_ERR_WARN, 0, __FILE__, __LINE__, "\033[1;37;44mWARNING\033[0m: " msg "\n", a)
#define PRT_WARN2(msg, a, b)       ErrorPrint(PRT_ERR_WARN, 0, __FILE__, __LINE__, "\033[1;37;44mWARNING\033[0m: " msg "\n", a, b)
#define PRT_WARN3(msg, a, b, c)    ErrorPrint(PRT_ERR_WARN, 0, __FILE__, __LINE__, "\033[1;37;44mWARNING\033[0m: " msg "\n", a, b, c)
#define PRT_WARN4(msg, a, b, c, d) ErrorPrint(PRT_ERR_WARN, 0, __FILE__, __LINE__, "\033[1;37;44mWARNING\033[0m: " msg "\n", a, b, c, d)

/// General messages.  Use level to indicate the message importance
///  - 0 : important
///  - 1 : informative
///  - 2+ : diagnostic
/// All messages are recorded in the log file, but only the more important
/// messages are printed on the console.  Use the command line option to
/// dictate which messages will be printed.
#define PRT_MSG0(level, msg)             ErrorPrint(PRT_ERR_MSG, level, __FILE__, __LINE__, "\033[1;37;42mINFO\033[0m: " msg "\n") 
#define PRT_MSG1(level, msg, a)          ErrorPrint(PRT_ERR_MSG, level, __FILE__, __LINE__, "\033[1;37;42mINFO\033[0m: " msg "\n", a) 
#define PRT_MSG2(level, msg, a, b)       ErrorPrint(PRT_ERR_MSG, level, __FILE__, __LINE__, "\033[1;37;42mINFO\033[0m: " msg "\n", a, b)
#define PRT_MSG3(level, msg, a, b, c)    ErrorPrint(PRT_ERR_MSG, level, __FILE__, __LINE__, "\033[1;37;42mINFO\033[0m: " msg "\n", a, b, c)
#define PRT_MSG4(level, msg, a, b, c, d) ErrorPrint(PRT_ERR_MSG, level, __FILE__, __LINE__, "\033[1;37;42mINFO\033[0m: " msg "\n", a, b, c, d)

#else
// FOR WIN32
/// Error message macros
#define PRT_ERR0(msg)              ErrorPrint(PRT_ERR_ERR, 0, __FILE__, __LINE__, "ERROR:" msg "\n")
#define PRT_ERR1(msg, a)           ErrorPrint(PRT_ERR_ERR, 0, __FILE__, __LINE__, "ERROR:" msg "\n", a)
#define PRT_ERR2(msg, a, b)        ErrorPrint(PRT_ERR_ERR, 0, __FILE__, __LINE__, "ERROR:" msg "\n", a, b)
#define PRT_ERR3(msg, a, b, c)     ErrorPrint(PRT_ERR_ERR, 0, __FILE__, __LINE__,  "ERROR:" msg "\n", a, b, c)
#define PRT_ERR4(msg, a, b, c,d)   ErrorPrint(PRT_ERR_ERR, 0, __FILE__, __LINE__,  "ERROR:" msg "\n", a, b, c, d)

/// Warning message macros
#define PRT_WARN0(msg)             ErrorPrint(PRT_ERR_WARN, 0, __FILE__, __LINE__, "WARNING: " msg "\n")
#define PRT_WARN1(msg, a)          ErrorPrint(PRT_ERR_WARN, 0, __FILE__, __LINE__, "WARNING: " msg "\n", a)
#define PRT_WARN2(msg, a, b)       ErrorPrint(PRT_ERR_WARN, 0, __FILE__, __LINE__, "WARNING: " msg "\n", a, b)
#define PRT_WARN3(msg, a, b, c)    ErrorPrint(PRT_ERR_WARN, 0, __FILE__, __LINE__, "WARNING: " msg "\n", a, b, c)
#define PRT_WARN4(msg, a, b, c, d) ErrorPrint(PRT_ERR_WARN, 0, __FILE__, __LINE__, "WARNING: " msg "\n", a, b, c, d)

/// General messages.  Use level to indicate the message importance
///  - 0 : important
///  - 1 : informative
///  - 2+ : diagnostic
/// All messages are recorded in the log file, but only the more important
/// messages are printed on the console.  Use the command line option to
/// dictate which messages will be printed.
#define PRT_MSG0(level, msg)             ErrorPrint(PRT_ERR_MSG, level, __FILE__, __LINE__, "INFO: " msg "\n") 
#define PRT_MSG1(level, msg, a)          ErrorPrint(PRT_ERR_MSG, level, __FILE__, __LINE__, "INFO: " msg "\n", a) 
#define PRT_MSG2(level, msg, a, b)       ErrorPrint(PRT_ERR_MSG, level, __FILE__, __LINE__, "INFO: " msg "\n", a, b)
#define PRT_MSG3(level, msg, a, b, c)    ErrorPrint(PRT_ERR_MSG, level, __FILE__, __LINE__, "INFO: " msg "\n", a, b, c)
#define PRT_MSG4(level, msg, a, b, c, d) ErrorPrint(PRT_ERR_MSG, level, __FILE__, __LINE__, "INFO: " msg "\n", a, b, c, d)

#endif


#endif
