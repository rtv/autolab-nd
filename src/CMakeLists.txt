

SET( HEADERS
  nd.h
  nd_alg.h
  nd2_alg.h
  geometria.h
)

SET( SOURCES 
  geometria.cpp
  nd.cpp
  nd_alg.cpp
)

SET( MYLIBS
  autolab-nd
  ${RAPI_LIBRARIES}
)

# if Stage is present, add code for visualizing ND in Stage
if( STAGE_FOUND )
  MESSAGE( ${INDENT} "Building Stage visualization" )
  SET( HEADERS ${HEADERS} ndvis.h )
  SET( SOURCES ${SOURCES} ndvis.cpp )	 
  SET( MYLIBS ${MYLIBS} ${STAGE_LIBRARIES} ${OPENGL_LIBRARY} )
  INCLUDE_DIRECTORIES( ${STAGE_INCLUDE_DIRS} )
  LINK_DIRECTORIES( ${STAGE_LIBRARY_DIRS} )
endif ( STAGE_FOUND )

# build a shared library
ADD_LIBRARY( autolab-nd SHARED ${SOURCES} )

# give the shared library a version number
SET_TARGET_PROPERTIES( autolab-nd PROPERTIES
  VERSION ${VERSION}
)

TARGET_LINK_LIBRARIES(  ${MYLIBS} )

INSTALL(TARGETS autolab-nd
  LIBRARY DESTINATION lib )

INSTALL(FILES ${HEADERS}
  DESTINATION include/${PROJECT_NAME}-${APIVERSION}
)
