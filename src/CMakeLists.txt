

SET( HEADERS
  nd.h
  nd_alg.h
)

SET( SOURCES 
	  geometria.cpp
	  nd.cpp
	  nd_alg.cpp 
)


# build a shared library
ADD_LIBRARY( autolab-nd SHARED ${SOURCES} )

# give the shared library a version number
SET_TARGET_PROPERTIES( autolab-nd PROPERTIES
  VERSION ${VERSION}
)

TARGET_LINK_LIBRARIES( autolab-nd
  ${RAPI_LIBRARIES}
  ${OPENGL}
)

INSTALL(TARGETS autolab-nd
  LIBRARY DESTINATION lib )

INSTALL(FILES ${HEADERS}
  DESTINATION include/${PROJECT_NAME}-${APIVERSION}
)
