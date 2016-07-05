 # This file is a part of CoSimA (CogIMon) project
 #
 # Copyright (C) 2016 by Dennis Leroy Wigand <dwigand at cor-lab dot uni-bielefeld dot de>
 #
 # This file may be licensed under the terms of the
 # GNU Lesser General Public License Version 3 (the ``LGPL''),
 # or (at your option) any later version.
 #
 # Software distributed under the License is distributed
 # on an ``AS IS'' basis, WITHOUT WARRANTY OF ANY KIND, either
 # express or implied. See the LGPL for the specific language
 # governing rights and limitations.
 #
 # You should have received a copy of the LGPL along with this
 # program. If not, go to http://www.gnu.org/licenses/lgpl.html
 # or write to the Free Software Foundation, Inc.,
 # 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 #
 # The development of this software was supported by:
 #   European Community’s Horizon 2020 robotics program ICT-23-2014
 #     under grant agreement 644727 - CogIMon
 #   CoR-Lab, Research Institute for Cognition and Robotics
 #     Bielefeld University
 #

cmake_minimum_required(VERSION 2.8.3)
project(rtt-core-extensions)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

string(TOUPPER "${CMAKE_PROJECT_NAME}" PROJECT_NAME_UPPER)

set(${PROJECT_NAME_UPPER}_VERSION_MAJOR 0)
set(${PROJECT_NAME_UPPER}_VERSION_MINOR 1)
set(${PROJECT_NAME_UPPER}_VERSION "${${PROJECT_NAME_UPPER}_VERSION_MAJOR}.${${PROJECT_NAME_UPPER}_VERSION_MINOR}")

set(INSTALL_SUFFIX "${${PROJECT_NAME_UPPER}_VERSION}")

set(LIBRARY_NAME "${CMAKE_PROJECT_NAME}${INSTALL_SUFFIX}")

set(COMPONENTS_LIB_NAME "${CMAKE_PROJECT_NAME}-components")

# dependencies

find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-typekit)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos-RTT. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

find_package(Eigen3)
add_definitions(${EIGEN3_DEFINITIONS})
include_directories(${EIGEN3_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

find_package(RST-RT REQUIRED)
message(STATUS "RST-RT version: ${RST-RT_VERSION}")
include_directories(BEFORE SYSTEM ${RST-RT_INCLUDE_DIRS})
add_definitions(${RST-RT_CFLAGS})
link_directories(${RST-RT_LIBRARY_DIR} ${RST-RT_LIBRARY_DIRS})

include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${USE_OROCOS_INCLUDE_DIRS}
    ${RST-RT_INCLUDE_DIRS}
)

# Sub-directories

add_subdirectory(src)

# Configuration installation

configure_file("${CMAKE_PROJECT_NAME}-config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}-config.cmake"
    @ONLY)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}-config.cmake"
    DESTINATION "share/${CMAKE_PROJECT_NAME}-${INSTALL_SUFFIX}")

# Build orocos components

orocos_component(${COMPONENTS_LIB_NAME} src/rtt-kinematic-chain-ja.cpp src/rtt-kinematic-chain-jt.cpp)
set_property(TARGET ${COMPONENTS_LIB_NAME} APPEND PROPERTY COMPILE_DEFINITIONS RTT_COMPONENT)

target_link_libraries(${COMPONENTS_LIB_NAME}
    ${USE_OROCOS_LIBRARIES}
    ${OROCOS-RTT_LIBRARIES}
    ${Boost_LIBRARIES}
    ${RST-RT_LIBRARIES}
    ${LIBRARY_NAME}
)
orocos_generate_package(INCLUDE_DIRS include)
