cmake_minimum_required(VERSION 2.6)

PROJECT(autolab-nd)
SET( V_MAJOR 0 )
SET( V_MINOR 1 )
SET( V_BUGFIX 0 )

SET( VERSION ${V_MAJOR}.${V_MINOR}.${V_BUGFIX} )
SET( APIVERSION ${V_MAJOR}.${V_MINOR} )

ADD_DEFINITIONS(-g -Wall -O2)

SET(CMAKE_VERBOSE_MAKEFILE OFF)

INCLUDE(FindPkgConfig)
PKG_SEARCH_MODULE( RAPI REQUIRED autolab-rapi  )
IF( RAPI_FOUND )
  MESSAGE( STATUS ${INDENT} "Rapi version ${RAPI_VERSION} detected at ${RAPI_PREFIX}" )
  #MESSAGE( STATUS "    RAPI_CFLAGS  = ${RAPI_CFLAGS}" )
  #MESSAGE( STATUS "    RAPI_LDFLAGS = ${RAPI_LDFLAGS}" )
  #MESSAGE( STATUS "    RAPI_INCLUDE = ${RAPI_INCLUDE_DIRS}" )
  #MESSAGE( STATUS "    RAPI_LIBS    = ${RAPI_LIBRARIES}" )
ELSE( RAPI_FOUND )
  MESSAGE( ${INDENT} "Rapi not detected" )
ENDIF( RAPI_FOUND )

# paths common to all subdirectories
INCLUDE_DIRECTORIES( ${RAPI_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${RAPI_LIBRARY_DIRS} )


# optional Stage stuff
PKG_SEARCH_MODULE( STAGE stage  )
IF( STAGE_FOUND )
  MESSAGE( "Found Stage" )
  FIND_PACKAGE( OpenGL REQUIRED )
  ADD_SUBDIRECTORY(examples) # right now the example targets Stage
ENDIF( STAGE_FOUND )

# check for Stage before descending
ADD_SUBDIRECTORY(src)


# Create and install the pkgconfig file from the .in template
SET( PKGFILE ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.pc )
CONFIGURE_FILE( ${PKGFILE}.in ${PKGFILE} @ONLY)
INSTALL( FILES ${PKGFILE} DESTINATION lib/pkgconfig/)
