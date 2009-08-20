INCLUDE(FindPkgConfig)
PKG_SEARCH_MODULE( RAPISTAGE REQUIRED autolab-rapi-stage  )

INCLUDE(FindPkgConfig)
PKG_SEARCH_MODULE( RAPISTAGE REQUIRED autolab-rapi-stage  )

INCLUDE_DIRECTORIES(
  ../src
  ${RAPISTAGE_INCLUDE_DIRS}
)
LINK_DIRECTORIES( 
  ${STAGE_LIBRARY_DIRS}
  ${RAPISTAGE_LIBRARY_DIRS}
)

# must come AFTER the LINK_DIRECTORIES() command. sigh...
ADD_LIBRARY( wander_nd MODULE wander_nd.cpp )

ADD_LIBRARY( wander_sonar_nd MODULE wander_sonar_nd.cpp )

# message( "*&^&^%" rapi stage lib dirs: ${RAPISTAGE_LIBRARY_DIRS} )

TARGET_LINK_LIBRARIES( wander_nd
  autolab-nd
  ${RAPISTAGE_LIBRARIES}
  ${OPENGL_LIBRARY}
)

TARGET_LINK_LIBRARIES( wander_sonar_nd
  autolab-nd
  ${RAPISTAGE_LIBRARIES}
  ${OPENGL_LIBRARY}
)


INSTALL(TARGETS wander_nd
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
)

STRING(REGEX REPLACE ";" " " STAGE_LDFLAGS_FULL "${STAGE_LDFLAGS}")
MESSAGE(STAGE_LDFLAGS_FULL " ${STAGE_LDFLAGS_FULL}" )

# delete the "lib" prefix from the plugin libraries
SET_TARGET_PROPERTIES( wander_nd PROPERTIES 
							  PREFIX "" 
							  LINK_FLAGS "${STAGE_LD_FLAGS_FULL}"
)

SET_TARGET_PROPERTIES( wander_sonar_nd PROPERTIES 
							  PREFIX "" 
							  LINK_FLAGS "${STAGE_LD_FLAGS_FULL}"
)
