/* ============================================================
 *
 * This file is a part of RST-RT (CogIMon) project
 *
 * Copyright (C) 2017 by Dennis Leroy Wigand <dwigand at cor-lab dot uni-bielefeld dot de>
 *
 * This file may be licensed under the terms of the
 * GNU Lesser General Public License Version 3 (the ``LGPL''),
 * or (at your option) any later version.
 *
 * Software distributed under the License is distributed
 * on an ``AS IS'' basis, WITHOUT WARRANTY OF ANY KIND, either
 * express or implied. See the LGPL for the specific language
 * governing rights and limitations.
 *
 * You should have received a copy of the LGPL along with this
 * program. If not, go to http://www.gnu.org/licenses/lgpl.html
 * or write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The development of this software was supported by:
 *   European Community’s Horizon 2020 robotics program ICT-23-2014
 *     under grant agreement 644727 - CogIMon
 *   CoR-Lab, Research Institute for Cognition and Robotics
 *     Bielefeld University
 *
 * ============================================================ */
#include "rtt-introspection-base-test.hpp"
#include <rtt/Operation.hpp>
#include <string>
#include <fstream>
#include <streambuf>

#include <rtt/os/TimeService.hpp>

using namespace cogimon;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

RTTIntrospectionBaseTest::RTTIntrospectionBaseTest(const std::string &name) : RTTIntrospectionBase(name) {
	time_service = RTT::os::TimeService::Instance();

	out_data = 1.337;
	out_data_port.setName("out_data_port");
    out_data_port.doc("Output port for testing");
    out_data_port.setDataSample(out_data);
    this->addPort(out_data_port);
}

bool RTTIntrospectionBaseTest::configureHookInternal() {
	return true;
}

void RTTIntrospectionBaseTest::updateHookInternal() {
	RTT::os::TimeService::nsecs start = RTT::os::TimeService::ticks2nsecs(time_service->getTicks());

	// while ((RTT::os::TimeService::ticks2nsecs(time_service->getTicks()) - start) < 1E+6) {
	out_data += out_data * 1 / start;
	this->writePort(out_data_port, out_data);
	// }
}

bool RTTIntrospectionBaseTest::startHookInternal() {
	return true;
}

void RTTIntrospectionBaseTest::stopHookInternal() {
	
}

void RTTIntrospectionBaseTest::cleanupHookInternal() {

}

//ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE(cogimon::RTTIntrospectionBaseTest)
