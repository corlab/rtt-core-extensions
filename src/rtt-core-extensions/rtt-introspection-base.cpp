/* ============================================================
 *
 * This file is a part of CoSiMA (CogIMon) project
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
																	  useCallTraceIntrospection(false),
																	  usePortTraceIntrospection(false),
																	  call_trace_storage_size(200),
																	  cts_send_latest_after(UINT_LEAST64_MAX),
																	  cts_last_send(0),
																	  cts_send_pro_hook(true),
																	  wmect(0) {
	this->provides("introspection")->addProperty("useCallTraceIntrospection", useCallTraceIntrospection).doc("Enable/Disable the introspection output.");
	this->provides("introspection")->addProperty("usePortTraceIntrospection", usePortTraceIntrospection).doc("Enable/Disable the port introspection output.");
	// this->provides("introspection")->addProperty("cts_send_latest_after", cts_send_latest_after).doc("Amount of time that can maximally pass before sending the samples.");
	this->provides("introspection")->addProperty("call_trace_storage_size", call_trace_storage_size).doc("Storage capacity.");
	this->provides("introspection")->addOperation("setCallTraceStorageSize", &RTTIntrospectionBase::setCallTraceStorageSize, this).doc("Set the size of the introspection output storage.");
	this->provides("introspection")->addOperation("enableAllIntrospection", &RTTIntrospectionBase::enableAllIntrospection, this).doc("Enables or Disables all introspection capabilities.");
	
	time_service = RTT::os::TimeService::Instance();
	wmectI = 0;
}

void RTTIntrospectionBase::enableAllIntrospection(const bool enable) {
	useCallTraceIntrospection = enable;
	usePortTraceIntrospection = enable;
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

	cts_port = rstrt::monitoring::CallTraceSample("port_access######################################",
													this->getName(), 0.0, rstrt::monitoring::CallTraceSample::CALL_UNIVERSAL);
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
// uint_least64_t overhead_start = time_service->getNSecs();
	if (useCallTraceIntrospection) {
		cts_update.call_time = time_service->getNSecs();
		cts_update.call_type = rstrt::monitoring::CallTraceSample::CALL_START_WITH_DURATION;

		// launch internal updateHook
		updateHookInternal();

		cts_update.call_duration = time_service->getNSecs();
		uint_least64_t wmect_tmp = cts_update.call_duration - cts_update.call_time;
		if (wmect_tmp > wmect) {
			wmect = wmect_tmp;
    		// RTT::log(RTT::Error) << "1[" << this->getName() << "] wmect: " << wmect << "ns, " << wmect * 1E-6 << "ms" << RTT::endlog();
		}

		// if (((cts_update.call_duration - cts_last_send) > cts_send_latest_after && !call_trace_storage.empty()) || (call_trace_storage.size() >= call_trace_storage_size)) {
		// uint_least64_t ss = time_service->getNSecs();

		// bool done = false;
		if (call_trace_storage.size() >= call_trace_storage_size) {
			// done = true;
			// publish if the storage is full.
			out_call_trace_sample_vec_port.write(call_trace_storage);
			// cts_last_send = cts_update.call_duration;
			call_trace_storage.clear();
		}
		call_trace_storage.push_back(cts_update);
		
		// uint_least64_t ee = time_service->getNSecs();
		// uint_least64_t diff = ee - ss;
		// if (diff > wmectI) {
		// 	wmectI = diff;
		// 	RTT::log(RTT::Error) << "2[" << this->getName() << "] wmect: " << wmectI << "ns, " << wmectI * 1E-6 << "ms: done " << done << RTT::endlog();
		// }
// uint_least64_t overhead_end = time_service->getNSecs();
// RTT::log(RTT::Fatal) << " [" << this->getName() << "] " << (overhead_end - overhead_start) << " " << wmect_tmp << RTT::endlog();
	} else {
		updateHookInternal();
	}
}

bool RTTIntrospectionBase::startHook() {
	bool startRet = false;
	if (useCallTraceIntrospection) {
		// start intro
		cts_start.call_time = time_service->getNSecs();
		cts_start.call_type = rstrt::monitoring::CallTraceSample::CALL_START_WITH_DURATION;
		// out_call_trace_sample_port.write(cts_start);

		// launch internal startHook
		startRet = startHookInternal();

		// end intro
		// cts_start.call_type = rstrt::monitoring::CallTraceSample::CALL_END;
		cts_start.call_duration = time_service->getNSecs();
		// out_call_trace_sample_port.write(cts_start);
		// if (((cts_start.call_duration - cts_last_send) > cts_send_latest_after && !call_trace_storage.empty()) || (call_trace_storage.size() >= call_trace_storage_size)) {
		if (call_trace_storage.size() >= call_trace_storage_size) {
			// publish if the time limit has been passed or if the storage is full.
			out_call_trace_sample_vec_port.write(call_trace_storage);
			// cts_last_send = cts_start.call_duration;
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
		// // start intro
		// cts_stop.call_time = time_service->getNSecs();
		// cts_stop.call_type = rstrt::monitoring::CallTraceSample::CALL_START_WITH_DURATION;
		// // out_call_trace_sample_port.write(cts_stop);

		// launch internal stopHook
		stopHookInternal();

		// // end intro
		// // cts_stop.call_type = rstrt::monitoring::CallTraceSample::CALL_END;
		// cts_stop.call_duration = time_service->getNSecs();
		
		// // out_call_trace_sample_port.write(cts_stop);
		// if (call_trace_storage.size() >= call_trace_storage_size) {
		// 	out_call_trace_sample_vec_port.write(call_trace_storage);
		// 	call_trace_storage.clear();
		// 	call_trace_storage.push_back(cts_stop);
		// 	// might be too fast!!!
		// 	out_call_trace_sample_vec_port.write(call_trace_storage);
		// } else {
		// 	call_trace_storage.push_back(cts_stop);
		// 	out_call_trace_sample_vec_port.write(call_trace_storage);
		// }
		// cts_last_send = cts_stop.call_duration;
		// call_trace_storage.clear();

		RTT::log(RTT::Error) << "END [" << this->getName() << "] WMECT: " << wmect  << "ns (" << wmect * 1E-6 << "ms)" << RTT::endlog();
	} else {
		stopHookInternal();
	}
}

void RTTIntrospectionBase::cleanupHook() {
	cts_send_latest_after = UINT_LEAST64_MAX;
	cleanupHookInternal();
}

void RTTIntrospectionBase::setCallTraceStorageSize(const int size) {
	call_trace_storage_size = size;
	call_trace_storage.clear();
	call_trace_storage.reserve(call_trace_storage_size);
}

uint_least64_t RTTIntrospectionBase::getWMECT() {
	return wmect;
}

void RTTIntrospectionBase::setWMECT(const uint_least64_t wmect) {
	this->wmect = wmect;
}

void RTTIntrospectionBase::processCTS(rstrt::monitoring::CallTraceSample& cts) {
	if (call_trace_storage.size() >= call_trace_storage_size) {
		// done = true;
		// publish if the storage is full.
		out_call_trace_sample_vec_port.write(call_trace_storage);
		// cts_last_send = cts_update.call_duration;
		call_trace_storage.clear();
	}
	call_trace_storage.push_back(cts);
}

//ORO_CREATE_COMPONENT_LIBRARY()
// ORO_LIST_COMPONENT_TYPE(cogimon::RTTIntrospectionBase)
