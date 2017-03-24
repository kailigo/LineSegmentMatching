# set project's name
PROJECT( LJL)

#ADD_DEFINITIONS(-DUNICODE)
ADD_DEFINITIONS(-D_CRT_SECURE_NO_WARNINGS)

###############################################################################
# CMake settings
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
SET(CMAKE_COLOR_MAKEFILE ON)
SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

# Allow advanced users to generate Makefiles printing detailed commands
MARK_AS_ADVANCED(CMAKE_VERBOSE_MAKEFILE)

# Path to additional CMake modules
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules ${CMAKE_MODULE_PATH})

SET(CMAKE_DEBUG_POSTFIX "d")
	
#large address aware option setting
IF(${MINGW})
	SET(LARGEADDRESSAWARE "--large-address-aware")
ELSE(${MINGW})
	IF (WIN32)
		SET(LARGEADDRESSAWARE "/LARGEADDRESSAWARE")
	ELSE (WIN32)
		SET(LARGEADDRESSAWARE " ")
	ENDIF (WIN32)
ENDIF (${MINGW})

IF (WIN32)
	SET(APP_OPT "WIN32")
ELSE (WIN32)
	IF (APPLE)
		#SET(APP_OPT "MACOSX_BUNDLE")
		SET(APP_OPT "EXCLUDE_FROM_ALL")
	ELSE (APPLE)
		SET(APP_OPT " ")
	ENDIF (APPLE)
ENDIF (WIN32)

# this command finds OpenCV libraries and sets all required variables
FIND_PACKAGE(OpenCV REQUIRED)
#SET(${OPENCV_INCLUDE_DIR} "C:\opencv\opencv\include\opencv; /usr/include/opencv2")
# Set the include directories
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/GeneralLibs)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/GeneralLibs/CmdLine)
# INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/GeneralLibs/LSD_1.6)
# INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/GeneralLibs/libTIFF)
# INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src/Common)
	
INCLUDE_DIRECTORIES(${OPENCV_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

LINK_DIRECTORIES(${CMAKE_BINARY_DIR})
LINK_DIRECTORIES(${CMAKE_BINARY_DIR}/bin)
# Setup output directories
SET(LIBRARY_OUTPUT_PATH 
	${PROJECT_BINARY_DIR}/bin
	CACHE
	PATH
	"Single directory for all libraries")
	
SET(EXECUTABLE_OUTPUT_PATH 
	${PROJECT_BINARY_DIR}/bin
	CACHE
	PATH
	"Single directory for all executables")

# enable warnings
ADD_DEFINITIONS( -Wall )

# this command finds Qt4 libraries and sets all required variables
# note that it's Qt4, not QT4 or qt4
# FIND_PACKAGE( Qt4 REQUIRED)

# SET(QT_USE_QTMAIN TRUE)
# SET(QT_USE_QTOPENGL TRUE)
# SET(QT_USE_QTXML TRUE)

# add some useful macros and variables
# (QT_USE_FILE is a variable defined by FIND_PACKAGE( Qt4 ) that contains a path to CMake script)
# INCLUDE( ${QT_USE_FILE} )

# we need this to be able to include headers produced by uic in our code
# (CMAKE_BINARY_DIR holds a path to the build directory, while INCLUDE_DIRECTORIES() works just like INCLUDEPATH from qmake)
INCLUDE_DIRECTORIES( ${CMAKE_BINARY_DIR} )

# set (FILES_TO_TRANSLATE )

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(GeneralLibs)
# file (GLOB TRANSLATIONS_FILES translations/*.ts)

# option (UPDATE_TRANSLATIONS "Update source translation translations/*.ts
# files (WARNING: make clean will delete the source .ts files! Danger!)")
# if (UPDATE_TRANSLATIONS)
  # qt4_create_translation(QM_FILES ${FILES_TO_TRANSLATE} ${TRANSLATIONS_FILES})
# else (UPDATE_TRANSLATIONS)
  # qt4_add_translation(QM_FILES ${TRANSLATIONS_FILES})
# endif (UPDATE_TRANSLATIONS) 

# add_custom_target (translations_target DEPENDS ${QM_FILES})


#install(FILES ${QM_FILES} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# install(FILES ${QM_FILES} DESTINATION ${CMAKE_INSTALL_PREFIX}/translations)

# ADD_CUSTOM_COMMAND(
	# TARGET translations_target
	# POST_BUILD
	# COMMAND ${CMAKE_COMMAND} ARGS -E copy ${CMAKE_CURRENT_BINARY_DIR}/omc_es.qm ${CMAKE_CURRENT_BINARY_DIR}/src
	# )
