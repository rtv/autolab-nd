

SET( TARGET ${PROJECT_NAME} )

SET( HEADERS  
	  nd.h
)

SET( SOURCES 
	  error.c
	  geometria.cpp
	  utilities.cpp
	  nd.cpp
	  nd_alg.cpp
	  common.cpp
	  rangefinder.cpp 
)

# build a shared library
ADD_LIBRARY( ${TARGET} SHARED ${SOURCES} )

# give the shared library a version number
SET_TARGET_PROPERTIES( ${TARGET} PROPERTIES
		       VERSION ${VERSION}
)

INSTALL(TARGETS ${TARGET}
		  LIBRARY DESTINATION lib )

INSTALL(FILES ${HEADERS}
        DESTINATION include/${PROJECT_NAME}-${APIVERSION}
)
