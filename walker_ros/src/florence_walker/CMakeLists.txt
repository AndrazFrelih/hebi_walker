cmake_minimum_required(VERSION 3.0.2)

project(florence_walker)

set(CMAKE_CXX_FLAGS "-std=c++11 -fPIC ${CMAKE_CXX_FLAGS}")
add_compile_definitions(EIGEN_STACK_ALLOCATION_LIMIT=0)

#######################################################################
## Find catkin macros and libraries ##
#######################################################################
find_package(catkin REQUIRED COMPONENTS
  control_msgs
  control_toolbox
  controller_manager
  controller_manager_msgs
  controller_interface
  realtime_tools
  roscpp
  rospy
  std_msgs
  roslib
  tf
  pluginlib					#export classes as plugins	
  #libclass_loader 			#dynamically load exported classes (plugins)
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(PythonLibs 3.6)

## speeds up Eigen (allows parallellization)
find_package(OpenMP)
if (OPENMP_FOUND)
	message(STATUS "Openmp was found")
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

#######################################################################
## PROJECT_DIR IS THE VARIABLE THAT HAS TO BE SET
#######################################################################
SET(PROJECT_DIR /home/hebi/Documents/andraz-f)
#######################################################################
SET(LIBRARY_DIR ${PROJECT_DIR}/libraries)
#######################################################################

#######################################################################
## HEBI VARIABLES ##
#######################################################################
SET(HEBI_DIR ${LIBRARY_DIR}/hebi-cpp-3.3.0)
SET(HEBI_INCLUDE_DIR ${HEBI_DIR}/hebi-cpp/hebi/include ${HEBI_DIR}/hebi-cpp/src ${HEBI_DIR}/util)
SET(HEBI_LIB_DIR ${HEBI_DIR}/build/hebi-cpp-debug)
SET(HEBI_LIB ${HEBI_LIB_DIR}/libhebic++.so)

#######################################################################
## RBDL VARIABLES ##
#######################################################################
SET(CUSTOM_RBDL_PATH ${LIBRARY_DIR}/rbdl-lib/rbdl)
SET(RBDL_INCLUDE_DIR ${CUSTOM_RBDL_PATH}/include)
SET(RBDL_INCLUDE_DIR_CFG ${CUSTOM_RBDL_PATH}/build/include)
SET(RBDL_LIBRARY_DIR ${CUSTOM_RBDL_PATH}/build)
SET(RBDL_LIB ${RBDL_LIBRARY_DIR}/librbdl.so)

SET(RBDL_URDFREADER_INCLUDE_DIR ${CUSTOM_RBDL_PATH}/addons/urdfreader)
SET(RBDL_URDFREADER_LIBRARY_DIR ${CUSTOM_RBDL_PATH}/build/addons/urdfreader)
SET(RBDL_URDFREADER_LIB ${RBDL_URDFREADER_LIBRARY_DIR}/librbdl_urdfreader.so)

#######################################################################
## QPOASES VARIABLES ##
#######################################################################
SET(QPOASES_DIR ${LIBRARY_DIR}/qpOASES-3.2.1)
SET(QPOASES_INCLUDE_DIR ${QPOASES_DIR}/include)
SET(QPOASES_SRC_DIR ${QPOASES_DIR}/src)
SET(QPOASES_LIBRARY_DIR ${QPOASES_DIR}/bin)
SET(QPOASES_LIB ${QPOASES_LIBRARY_DIR}/libqpOASES.so)
SET(QPOASES_BLAS_LAPACK_REPL ${QPOASES_SRC_DIR}/LAPACKReplacement.cpp ${QPOASES_SRC_DIR}/BLASReplacement.cpp)


#######################################################################
## catkin specific configuration ##
#######################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS control_msgs control_toolbox realtime_tools roscpp rospy std_msgs roslib controller_manager controller_interface tf pluginlib
)

#######################################################################
## BUILD ##
#######################################################################

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
	${PYTHON_INCLUDE_DIRS}
	${HEBI_INCLUDE_DIR}
	${RBDL_INCLUDE_DIR} 
	${RBDL_INCLUDE_DIR_CFG} 
	${RBDL_URDFREADER_INCLUDE_DIR}
	${QPOASES_INCLUDE_DIR}
)

#######################################################################
## Declare a C++ library - all plugin required files are included ##
#######################################################################
add_library(${PROJECT_NAME}
	## controller classes
		src/ctrl/RobotBaseController.cpp
		src/ctrl/FlorenceVisNode.cpp
		src/ctrl/controllers/FlorenceJointController.cpp
		src/ctrl/controllers/FlorenceComShiftController.cpp
		src/ctrl/controllers/FlorenceFullController.cpp
	## additional classes needed to build single controllers
		# Floating base estimation
			src/est/FloatingBaseEst.cpp
			src/est/magdwick.cpp
		# Balancer
			src/bal/Balancer.cpp
		# Full body IKine
			src/ikine/FullBodyIKine.cpp
		# Trajectory generator
			src/tgen/TrajectoryGenerator.cpp
		# Pattern generator
			src/pgen/PatternGenerator.cpp
)

FILE(GLOB SRC ${QPOASES_SRC_DIR}/*.cpp)
add_library(alg_repl STATIC ${SRC})

#######################################################################
## MAIN NODES -EXEC DEFINITION ##
#######################################################################
add_executable(florence_controller 
	src/florence_controller.cpp 
	src/hw/MotorData.cpp
	src/hw/RobotInterface.cpp
)

add_executable(florence_fsm 
	src/florence_fsm.cpp 
)

add_executable(fsen_calib_node 
	src/fsen_calib.cpp 
)

add_executable(joint_offs_comp_node 
	src/joint_offset_comp.cpp 
)


#######################################################################
## TEST NODES -EXEC DEFINITION ##
#######################################################################
add_executable(lib_test_node 
	src/lib_test.cpp 
)

add_executable(fb_speed_test_node 
	src/fb_speed_test.cpp 
)

add_executable(PG_test_node 
	src/pgen/PG_test.cpp
	src/pgen/PatternGenerator.cpp
)

add_executable(TG_test_node 
	src/tgen/TG_test.cpp
	src/pgen/PatternGenerator.cpp
	src/tgen/TrajectoryGenerator.cpp
)

#######################################################################
## REQUIRED LIBRARIES ##
#######################################################################

set(REQ_LIBS 
	${catkin_LIBRARIES}
	${PYTHON_LIBRARIES}
	${HEBI_LIB}
	${RBDL_LIB}
	${RBDL_URDFREADER_LIB}
	alg_repl
	${QPOASES_LIB}
	#${LAPACK_LIBS}
)

#######################################################################
## MAIN NODES -LIBS LINKING ##
#######################################################################
target_link_libraries(florence_controller ${REQ_LIBS})
target_link_libraries(florence_fsm ${REQ_LIBS})
target_link_libraries(fsen_calib_node ${REQ_LIBS})
target_link_libraries(joint_offs_comp_node ${REQ_LIBS})

#######################################################################
## TEST NODES -LIBS LINKING ##
#######################################################################
target_link_libraries(lib_test_node	${REQ_LIBS})
target_link_libraries(fb_speed_test_node ${REQ_LIBS})
target_link_libraries(PG_test_node ${REQ_LIBS})
target_link_libraries(TG_test_node ${REQ_LIBS})



#library that needs to be installed
target_link_libraries(	${PROJECT_NAME}
						${roscpp_LIBRARIES}
						${REQ_LIBS}
)

#######################################################################
## INSTALLATION - REQUIRED AS PLUGGINS ARE GENERATED ##
#######################################################################

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES florence_ctrl_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

