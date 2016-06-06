 # This file is a part of RST-RT (CogIMon) project
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

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

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

orocos_component(${PROJECT_NAME} src/rtt-kinematic-chain-ja.cpp src/rtt-kinematic-chain-jt.cpp src/rtt-jointaware-taskcontext.cpp)
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS RTT_COMPONENT)

target_link_libraries(${PROJECT_NAME}
   ${USE_OROCOS_LIBRARIES}
   ${OROCOS-RTT_LIBRARIES}
   ${Boost_LIBRARIES}
   ${RST-RT_LIBRARIES}
)
orocos_generate_package(INCLUDE_DIRS include)
