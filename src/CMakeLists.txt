

SET( HEADERS
  nd.h
  nd_alg.h
  nd2_alg.h
  geometria.h
  ndplus.h
)

SET( SOURCES
  geometria.cpp
  nd.cpp
  nd_alg.cpp
  ndplus.cpp
)

SET( MYLIBS
  autolab-nd
  ${RAPI_LIBRARIES}
)

# if Stage is present, add code for visualizing ND in Stage
if( STAGE_FOUND )
  MESSAGE( ${INDENT} "Building Stage visualization" )
  SET( HEADERS ${HEADERS} ndvis.h stagend.h )
  SET( SOURCES ${SOURCES} ndvis.cpp stagend.cpp )
  SET( MYLIBS ${MYLIBS} ${STAGE_LIBRARIES} )
  INCLUDE_DIRECTORIES( ${STAGE_INCLUDE_DIRS} )
  LINK_DIRECTORIES( ${STAGE_LIBRARY_DIRS} )

  STRING(REGEX REPLACE ";" " " STAGE_LDFLAGS_MORE "${STAGE_LDFLAGS_OTHER}")
  MESSAGE(STAGE_LDFLAGS_MORE " ${STAGE_LDFLAGS_MORE}" )

endif ( STAGE_FOUND )

# build a shared library
ADD_LIBRARY( autolab-nd SHARED ${SOURCES} )

# give the shared library a version number
SET_TARGET_PROPERTIES( autolab-nd PROPERTIES
							  VERSION ${VERSION}
								  LINK_FLAGS "${STAGE_LDFLAGS_MORE}"							  
)

TARGET_LINK_LIBRARIES(  ${MYLIBS} )

INSTALL(TARGETS autolab-nd
  LIBRARY DESTINATION lib )

INSTALL(FILES ${HEADERS}
  DESTINATION include/${PROJECT_NAME}-${APIVERSION}
)
