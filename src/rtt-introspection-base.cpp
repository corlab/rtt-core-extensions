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
#include "rtt-introspection-base.hpp"
#include <rtt/Operation.hpp>
#include <string>
#include <fstream>
#include <streambuf>

using namespace cogimon;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

RTTIntrospectionBase::RTTIntrospectionBase(const std::string &name) : TaskContext(name),
																	  useCallTraceIntrospection(true) {
	this->provides("introspection")->addProperty("useCallTraceIntrospection", useCallTraceIntrospection);
	time_service = RTT::os::TimeService::Instance();
}

bool RTTIntrospectionBase::configureHook() {
	if (this->provides("introspection")->getPort("out_call_trace_sample_port")) {
		this->provides("introspection")->removePort("out_call_trace_sample_port");
	}
	//prepare introspection output variables
    cts_start = rstrt::monitoring::CallTraceSample("startHook()", this->getName(), 0.1, rstrt::monitoring::CallTraceSample::CALL_UNIVERSAL);
	cts_configure = rstrt::monitoring::CallTraceSample("configureHook()", this->getName(), 0.1, rstrt::monitoring::CallTraceSample::CALL_UNIVERSAL);
	cts_update = rstrt::monitoring::CallTraceSample("updateHook()", this->getName(), 0.1, rstrt::monitoring::CallTraceSample::CALL_UNIVERSAL);
	cts_stop = rstrt::monitoring::CallTraceSample("stopHook()", this->getName(), 0.1, rstrt::monitoring::CallTraceSample::CALL_UNIVERSAL);
	cts_cleanup = rstrt::monitoring::CallTraceSample("cleanupHook()", this->getName(), 0.1, rstrt::monitoring::CallTraceSample::CALL_UNIVERSAL);
	//prepare introspection output ports
    out_call_trace_sample_port.setName("out_call_trace_sample_port");
    out_call_trace_sample_port.doc("Output port for call trace samples");
    out_call_trace_sample_port.setDataSample(cts_update);
    this->provides("introspection")->addPort(out_call_trace_sample_port);
	
	return true;
}

void RTTIntrospectionBase::updateHook() {
	if (useCallTraceIntrospection) {
		// start intro
		cts_update.call_type = rstrt::monitoring::CallTraceSample::CALL_START;
		cts_update.call_time = RTT::os::TimeService::ticks2nsecs(time_service->getTicks());
		out_call_trace_sample_port.write(cts_update);

		// launch internal updateHook
		// TODO
		
		// end intro
		
		cts_update.call_type = rstrt::monitoring::CallTraceSample::CALL_END;
		cts_update.call_time = RTT::os::TimeService::ticks2nsecs(time_service->getTicks());
		out_call_trace_sample_port.write(cts_update);
	} else {
		// launch internal updateHook
		// TODO
	}
}

bool RTTIntrospectionBase::startHook() {
	if (useCallTraceIntrospection) {
		// start intro
		cts_start.call_type = rstrt::monitoring::CallTraceSample::CALL_START;
		cts_start.call_time = RTT::os::TimeService::ticks2nsecs(time_service->getTicks());
		out_call_trace_sample_port.write(cts_start);

		// launch internal startHook TODO

		// end intro
		cts_start.call_type = rstrt::monitoring::CallTraceSample::CALL_END;
		cts_start.call_time = RTT::os::TimeService::ticks2nsecs(time_service->getTicks());
		out_call_trace_sample_port.write(cts_start);
		
		// TODO return
		return true;
	} else {
		// launch internal startHook
		// TODO return
		return true;
	}
}

void RTTIntrospectionBase::stopHook() {
	if (useCallTraceIntrospection) {
		// start intro
		cts_stop.call_type = rstrt::monitoring::CallTraceSample::CALL_START;
		cts_stop.call_time = RTT::os::TimeService::ticks2nsecs(time_service->getTicks());
		out_call_trace_sample_port.write(cts_stop);

		// launch internal stopHook
		// TODO

		// end intro
		cts_stop.call_type = rstrt::monitoring::CallTraceSample::CALL_END;
		cts_stop.call_time = RTT::os::TimeService::ticks2nsecs(time_service->getTicks());
		out_call_trace_sample_port.write(cts_stop);
	} else {
		// launch internal stopHook
		// TODO
	}
}

void RTTIntrospectionBase::cleanupHook() {
	// launch internal cleanupHook
	// TODO
}


//ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE(cogimon::RTTIntrospectionBase)
