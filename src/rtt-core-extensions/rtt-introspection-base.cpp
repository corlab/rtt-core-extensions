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
#include <limits>

using namespace cogimon;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

RTTIntrospectionBase::RTTIntrospectionBase(const std::string &name) : TaskContext(name),
																	  useCallTraceIntrospection(true),
																	  call_trace_storage_size(3200),
																	  cts_send_latest_after(std::numeric_limits<uint_least64_t>::infinity()),
																	  cts_last_send(0) {
	this->provides("introspection")->addProperty("useCallTraceIntrospection", useCallTraceIntrospection).doc("Enable/Disable the introspection output.");
	this->provides("introspection")->addProperty("cts_send_latest_after", cts_send_latest_after).doc("Amount of time that can maximally pass before sending the samples.");
	this->provides("introspection")->addOperation("setCallTraceStorageSize", &RTTIntrospectionBase::setCallTraceStorageSize, this).doc("Set the size of the introspection output storage.");
	time_service = RTT::os::TimeService::Instance();
}

bool RTTIntrospectionBase::configureHook() {
	if (this->provides("introspection")->getPort("out_call_trace_sample_port")) {
		this->provides("introspection")->removePort("out_call_trace_sample_port");
	}
	if (this->provides("introspection")->getPort("out_call_trace_sample_vec_port")) {
		this->provides("introspection")->removePort("out_call_trace_sample_vec_port");
	}
	//prepare introspection output variables
    cts_start = rstrt::monitoring::CallTraceSample("startHook()", this->getName(), 0.0, rstrt::monitoring::CallTraceSample::CALL_UNIVERSAL);
	cts_configure = rstrt::monitoring::CallTraceSample("configureHook()", this->getName(), 0.0, rstrt::monitoring::CallTraceSample::CALL_UNIVERSAL);
	cts_update = rstrt::monitoring::CallTraceSample("updateHook()", this->getName(), 0.0, rstrt::monitoring::CallTraceSample::CALL_UNIVERSAL);
	cts_stop = rstrt::monitoring::CallTraceSample("stopHook()", this->getName(), 0.0, rstrt::monitoring::CallTraceSample::CALL_UNIVERSAL);
	cts_cleanup = rstrt::monitoring::CallTraceSample("cleanupHook()", this->getName(), 0.0, rstrt::monitoring::CallTraceSample::CALL_UNIVERSAL);

	cts_port = rstrt::monitoring::CallTraceSample("port_access", this->getName(), 0.0, rstrt::monitoring::CallTraceSample::CALL_UNIVERSAL);
	//prepare introspection output ports
    out_call_trace_sample_port.setName("out_call_trace_sample_port");
    out_call_trace_sample_port.doc("Output port for call trace samples");
    out_call_trace_sample_port.setDataSample(cts_update);
    this->provides("introspection")->addPort(out_call_trace_sample_port);

	call_trace_storage.resize(call_trace_storage_size);
	call_trace_storage.reserve(call_trace_storage_size);
	out_call_trace_sample_vec_port.setName("out_call_trace_sample_vec_port");
    out_call_trace_sample_vec_port.doc("Output port for call trace samples vector");
    out_call_trace_sample_vec_port.setDataSample(call_trace_storage);
    this->provides("introspection")->addPort(out_call_trace_sample_vec_port);
	// empty but capacity is unchanged!
	call_trace_storage.clear();

	cts_last_send = 0;

	return configureHookInternal();
}

void RTTIntrospectionBase::updateHook() {
	if (useCallTraceIntrospection) {
		// start intro
		cts_update.call_time = RTT::os::TimeService::ticks2nsecs(time_service->getTicks());
		cts_update.call_type = rstrt::monitoring::CallTraceSample::CALL_START_WITH_DURATION;
		// out_call_trace_sample_port.write(cts_update);

		// launch internal updateHook
		updateHookInternal();
		
		// end intro
		// cts_update.call_type = rstrt::monitoring::CallTraceSample::CALL_END;
		cts_update.call_duration = RTT::os::TimeService::ticks2nsecs(time_service->getTicks());
		// out_call_trace_sample_port.write(cts_update);


		RTT::os::TimeService::nsecs tmp_send_time = RTT::os::TimeService::ticks2nsecs(time_service->getTicks());
		if (((tmp_send_time - cts_last_send) > cts_send_latest_after) || (call_trace_storage.size() >= call_trace_storage_size)) {
			// publish if the time limit has been passed or if the storage is full.
			out_call_trace_sample_vec_port.write(call_trace_storage);
			cts_last_send = tmp_send_time;
			call_trace_storage.clear();
		}
		call_trace_storage.push_back(cts_update);
	} else {
		updateHookInternal();
	}
}

bool RTTIntrospectionBase::startHook() {
	bool startRet = false;
	if (useCallTraceIntrospection) {
		// start intro
		cts_start.call_time = RTT::os::TimeService::ticks2nsecs(time_service->getTicks());
		cts_start.call_type = rstrt::monitoring::CallTraceSample::CALL_START_WITH_DURATION;
		// out_call_trace_sample_port.write(cts_start);

		// launch internal startHook
		startRet = startHookInternal();

		// end intro
		// cts_start.call_type = rstrt::monitoring::CallTraceSample::CALL_END;
		cts_start.call_duration = RTT::os::TimeService::ticks2nsecs(time_service->getTicks());
		// out_call_trace_sample_port.write(cts_start);

		RTT::os::TimeService::nsecs tmp_send_time = RTT::os::TimeService::ticks2nsecs(time_service->getTicks());
		if (((tmp_send_time - cts_last_send) > cts_send_latest_after) || (call_trace_storage.size() >= call_trace_storage_size)) {
			// publish if the time limit has been passed or if the storage is full.
			out_call_trace_sample_vec_port.write(call_trace_storage);
			cts_last_send = tmp_send_time;
			call_trace_storage.clear();
		}
		call_trace_storage.push_back(cts_start);
		return startRet;
	} else {
		return startHookInternal();
	}
}

void RTTIntrospectionBase::stopHook() {
	if (useCallTraceIntrospection) {
		// start intro
		cts_stop.call_time = RTT::os::TimeService::ticks2nsecs(time_service->getTicks());
		cts_stop.call_type = rstrt::monitoring::CallTraceSample::CALL_START_WITH_DURATION;
		// out_call_trace_sample_port.write(cts_stop);

		// launch internal stopHook
		stopHookInternal();

		// end intro
		// cts_stop.call_type = rstrt::monitoring::CallTraceSample::CALL_END;
		cts_stop.call_duration = RTT::os::TimeService::ticks2nsecs(time_service->getTicks());
		// out_call_trace_sample_port.write(cts_stop);

		RTT::os::TimeService::nsecs tmp_send_time = RTT::os::TimeService::ticks2nsecs(time_service->getTicks());
		if (((tmp_send_time - cts_last_send) > cts_send_latest_after) || (call_trace_storage.size() >= call_trace_storage_size)) {
			// publish if the time limit has been passed or if the storage is full.
			out_call_trace_sample_vec_port.write(call_trace_storage);
			cts_last_send = tmp_send_time;
			call_trace_storage.clear();
		}
		call_trace_storage.push_back(cts_stop);
	} else {
		stopHookInternal();
	}
}

void RTTIntrospectionBase::cleanupHook() {
	cleanupHookInternal();
}

void RTTIntrospectionBase::setCallTraceStorageSize(const int size) {
	call_trace_storage_size = size;
	call_trace_storage.clear();
	call_trace_storage.reserve(call_trace_storage_size);
}

//ORO_CREATE_COMPONENT_LIBRARY()
// ORO_LIST_COMPONENT_TYPE(cogimon::RTTIntrospectionBase)
