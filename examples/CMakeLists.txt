

ADD_LIBRARY( wander_nd MODULE wander_nd.cpp )

INCLUDE(FindPkgConfig)
PKG_SEARCH_MODULE( RAPISTAGE REQUIRED autolab-rapi-stage  )

INCLUDE_DIRECTORIES( 
  ${RAPISTAGE_INCLUDE_DIRS}
)

LINK_DIRECTORIES( 
  ${RAPISTAGE_LIBRARY_DIRS}						
)

TARGET_LINK_LIBRARIES( wander_nd
    autolab-nd
    ${STAGE_LIBRARIES}
    ${RAPISTAGE_LIBRARIES}						
    ${OPENGL_LIBRARIES}
)

INSTALL(TARGETS wander_nd
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
)

# delete the "lib" prefix from the plugin libraries
SET_TARGET_PROPERTIES( wander_nd PROPERTIES PREFIX "" )

