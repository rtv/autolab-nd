cmake_minimum_required(VERSION 2.6)

PROJECT(autolab-nd)

INCLUDE(FindPkgConfig)

#add definitions, compiler switches, etc.
ADD_DEFINITIONS(-g -Wall -O2)

# pkg_search_module( GLIB REQUIRED gthread-2.0 )

# pkg_search_module( STAGE REQUIRED stage )
#MESSAGE(STAGE "${STAGE_LIBRARY_DIRS}" )
#MESSAGE(STAGE "${STAGE_LIBRARIES}" )
#MESSAGE(STAGEDIR "${STAGE_INCLUDE_DIRS}" )

ADD_SUBDIRECTORY(src)
