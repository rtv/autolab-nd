

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

SET( LIBRARIES
  ${RAPI_LIBRARIES}
)

# if Stage is present, add code for visualizing ND in Stage
if( STAGE_FOUND )
  SET( HEADERS ${HEADERS} ndvis.h )
  SET( SOURCES ${SOURCES} ndvis.cpp )	 
  SET( LIBRARIES ${LIBRARIES} ${STAGE_LIBRARIES} ${OPENGL_LIBRARIES} )
endif ( STAGE_FOUND )


# build a shared library
ADD_LIBRARY( autolab-nd SHARED ${SOURCES} )

# give the shared library a version number
SET_TARGET_PROPERTIES( autolab-nd PROPERTIES
  VERSION ${VERSION}
)

TARGET_LINK_LIBRARIES( autolab-nd ${LIBRARIES} )

INSTALL(TARGETS autolab-nd
  LIBRARY DESTINATION lib )

INSTALL(FILES ${HEADERS}
  DESTINATION include/${PROJECT_NAME}-${APIVERSION}
)
