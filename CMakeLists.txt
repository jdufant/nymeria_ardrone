cmake_minimum_required(VERSION 2.8.3)
project(nymeria_ardrone)

find_package(catkin REQUIRED COMPONENTS
	roscpp REQUIRED
	ardrone_autonomy
	geometry_msgs
	std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES nymeria_ardrone
	CATKIN_DEPENDS roscpp geometry_msgs std_msgs ardrone_autonomy
	DEPENDS system_lib
)

###########
## Build ##
###########

## Your package locations should be listed before other locations
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
set(LIBRARY_SOURCES
	src/Nymeria.cpp
	src/NymeriaMutex.cpp
	src/NymeriaMutexCommand.cpp
	src/NymeriaMutexObstacle.cpp
	src/NymeriaMutexSecurityDistance.cpp
	src/NymeriaCheckObstacle.cpp
	src/UDPWrapper.cpp
	src/exception/NymeriaExceptions.cpp
	src/exception/NymeriaParamExc.cpp
	src/exception/NymeriaInvalidSecurityDistance.cpp
)

add_library(
  	${PROJECT_NAME}
  		${LIBRARY_SOURCES}
)

# because library depends on libraries provided by other catkin packages
target_link_libraries(
	${PROJECT_NAME}
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
)

## Declare a cpp executable
add_executable(nymeria_command		src/Command.cpp)
add_executable(nymeria_sensor_interface	src/SensorInterface.cpp)
add_executable(nymeria_controller	src/Controller.cpp)
add_executable(nymeria_test_functionnal	test/functionnal_tests.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries (nymeria_command nymeria_ardrone ${catkin_LIBRARIES})
target_link_libraries (nymeria_sensor_interface nymeria_ardrone ${catkin_LIBRARIES})
target_link_libraries (nymeria_controller nymeria_ardrone ${catkin_LIBRARIES} )
target_link_libraries (nymeria_test_functionnal nymeria_ardrone ${catkin_LIBRARIES} )

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS nymeria_ardrone
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".git" EXCLUDE
)
