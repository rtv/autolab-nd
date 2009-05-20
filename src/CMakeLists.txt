

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
ADD_LIBRARY( nd SHARED ${SOURCES} )

# give the shared library a version number
SET_TARGET_PROPERTIES( nd PROPERTIES
  VERSION ${VERSION}
)

TARGET_LINK_LIBRARIES( nd
  ${RAPI_LIBRARIES}
  ${OPENGL}
)

INSTALL(TARGETS nd
  LIBRARY DESTINATION lib )

INSTALL(FILES ${HEADERS}
  DESTINATION include/${PROJECT_NAME}-${APIVERSION}
)
