
		  
# build a shared library
ADD_LIBRARY( autolab-nd SHARED
            error.c
				geometria.cpp
            utilities.cpp
            nd.cpp
            nd_alg.cpp
            common.cpp
            rangefinder.cpp
            )


#TARGET_LINK_LIBRARIES( fasrrobot
#  ${OPENGL_LIBRARIES}
#)

INSTALL(TARGETS autolab-nd
	LIBRARY DESTINATION lib
)
