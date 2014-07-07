# - Try to find OpenCL
# This module tries to find an OpenCL implementation on your system. It supports
# AMD / ATI, Apple and NVIDIA implementations, but shoudl work, too.
#
# Once done this will define
#  CMINPACK_FOUND        - system has CMinpack
#  CMINPACK_INCLUDE_DIRS  - the CMinpack include directory
#  CMINPACK_LIBRARIES    - link these to use CMinpack
#
# WIN32 should work, but is untested

FIND_PACKAGE( PackageHandleStandardArgs )

IF (APPLE)

  FIND_LIBRARY(CMINPACK_LIBRARIES CMINPACK DOC "CMINPACK lib for OSX")
  FIND_PATH(CMINPACK_INCLUDE_DIRS CMINPACK.h DOC "Include for CMINPACK on OSX")

ELSE (APPLE)

	IF (WIN32)
	
	    FIND_PATH(CMINPACK_INCLUDE_DIRS minpack.h)
	    FIND_LIBRARY(CMINPACK_LIBRARIES cminpack.lib )
	
	ELSE (WIN32)

            # Unix style platforms
            FIND_LIBRARY(CMINPACK_LIBRARIES cminpack
              ENV LD_LIBRARY_PATH
            )

            FIND_PATH(CMINPACK_INCLUDE_DIRS minpack.h PATHS)

	ENDIF (WIN32)
ENDIF (APPLE)


