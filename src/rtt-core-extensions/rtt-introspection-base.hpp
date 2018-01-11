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
#ifndef RTT_INTROSPECTION_BASE_HPP
#define RTT_INTROSPECTION_BASE_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <rtt/os/Semaphore.hpp>

#include <Eigen/Dense>

#include <vector>

#include <Eigen/Core>
#include <time.h>
#include <rtt/os/TimeService.hpp>
#include <sstream>
#include <rtt/Logger.hpp>

#include <boost/shared_ptr.hpp>

#include <thread>
#include <memory>

// RST-RT includes
#include <rst-rt/monitoring/CallTraceSample.hpp>

#include <rtt/os/TimeService.hpp>

namespace cogimon {

class RTTIntrospectionBase : public RTT::TaskContext {
public:
	RTTIntrospectionBase(std::string const& name);
	// virtual ~RTTIntrospectionBase() {};

	bool configureHook();
	bool startHook();
	void updateHook();
	void stopHook();
	void cleanupHook();

	void setCallTraceStorageSize(const int size);

	template<class T>
	RTT::FlowStatus readPort(RTT::InputPort<T>& input_port, RTT::base::DataSourceBase::shared_ptr source, bool copy_old_data = true) {
		RTT::FlowStatus f = input_port.read(source, copy_old_data);
		if (useCallTraceIntrospection && usePortTraceIntrospection) {
			cts_port.call_time = time_service->getNSecs();
			cts_port.call_name = input_port.getName();
			if (f == RTT::NoData) {
				cts_port.call_type = rstrt::monitoring::CallTraceSample::CALL_PORT_READ_NODATA;
			} else if (f == RTT::OldData) {
				cts_port.call_type = rstrt::monitoring::CallTraceSample::CALL_PORT_READ_OLDDATA;
			} else if (f == RTT::NewData) {
				cts_port.call_type = rstrt::monitoring::CallTraceSample::CALL_PORT_READ_NEWDATA;
			}
			// out_call_trace_sample_port.write(cts_port);
			// if (((cts_port.call_time - cts_last_send) > cts_send_latest_after && !call_trace_storage.empty()) || (call_trace_storage.size() >= call_trace_storage_size)) {
			if (call_trace_storage.size() >= call_trace_storage_size) {
				// publish if the time limit has been passed or if the storage is full.
				out_call_trace_sample_vec_port.write(call_trace_storage);
				// cts_last_send = cts_port.call_time;
				call_trace_storage.clear();
			}
			call_trace_storage.push_back(cts_port);
		}
		return f;
	}

	template<class T>
	RTT::FlowStatus readPort(RTT::InputPort<T>& input_port, typename RTT::base::ChannelElement<T>::reference_t sample, bool copy_old_data = true) {
		RTT::FlowStatus f = input_port.read(sample, copy_old_data);
		if (useCallTraceIntrospection && usePortTraceIntrospection) {
			cts_port.call_time = time_service->getNSecs();
			cts_port.call_name = input_port.getName();
			if (f == RTT::NoData) {
				cts_port.call_type = rstrt::monitoring::CallTraceSample::CALL_PORT_READ_NODATA;
			} else if (f == RTT::OldData) {
				cts_port.call_type = rstrt::monitoring::CallTraceSample::CALL_PORT_READ_OLDDATA;
			} else if (f == RTT::NewData) {
				cts_port.call_type = rstrt::monitoring::CallTraceSample::CALL_PORT_READ_NEWDATA;
			}
			// out_call_trace_sample_port.write(cts_port);
			// if (((cts_port.call_time - cts_last_send) > cts_send_latest_after && !call_trace_storage.empty()) || (call_trace_storage.size() >= call_trace_storage_size)) {
			if (call_trace_storage.size() >= call_trace_storage_size) {
				// publish if the time limit has been passed or if the storage is full.
				out_call_trace_sample_vec_port.write(call_trace_storage);
				// cts_last_send = cts_port.call_time;
				call_trace_storage.clear();
			}
			call_trace_storage.push_back(cts_port);
		}
		return f;
	}

	// with pointer
	template<class T>
	RTT::FlowStatus readPort(boost::shared_ptr<RTT::InputPort<T> > input_port, RTT::base::DataSourceBase::shared_ptr source, bool copy_old_data = true) {
		RTT::FlowStatus f = input_port->read(source, copy_old_data);
		if (useCallTraceIntrospection && usePortTraceIntrospection) {
			cts_port.call_time = time_service->getNSecs();
			cts_port.call_name = input_port->getName();
			if (f == RTT::NoData) {
				cts_port.call_type = rstrt::monitoring::CallTraceSample::CALL_PORT_READ_NODATA;
			} else if (f == RTT::OldData) {
				cts_port.call_type = rstrt::monitoring::CallTraceSample::CALL_PORT_READ_OLDDATA;
			} else if (f == RTT::NewData) {
				cts_port.call_type = rstrt::monitoring::CallTraceSample::CALL_PORT_READ_NEWDATA;
			}
			// out_call_trace_sample_port.write(cts_port);
			// if (((cts_port.call_time - cts_last_send) > cts_send_latest_after && !call_trace_storage.empty()) || (call_trace_storage.size() >= call_trace_storage_size)) {
			if (call_trace_storage.size() >= call_trace_storage_size) {
				// publish if the time limit has been passed or if the storage is full.
				out_call_trace_sample_vec_port.write(call_trace_storage);
				// cts_last_send = cts_port.call_time;
				call_trace_storage.clear();
			}
			call_trace_storage.push_back(cts_port);
		}
		return f;
	}

	template<class T>
	RTT::FlowStatus readPort(boost::shared_ptr<RTT::InputPort<T> > input_port, typename RTT::base::ChannelElement<T>::reference_t sample, bool copy_old_data = true) {
		RTT::FlowStatus f = input_port->read(sample, copy_old_data);
		if (useCallTraceIntrospection && usePortTraceIntrospection) {
			cts_port.call_time = time_service->getNSecs();
			cts_port.call_name = input_port->getName();
			if (f == RTT::NoData) {
				cts_port.call_type = rstrt::monitoring::CallTraceSample::CALL_PORT_READ_NODATA;
			} else if (f == RTT::OldData) {
				cts_port.call_type = rstrt::monitoring::CallTraceSample::CALL_PORT_READ_OLDDATA;
			} else if (f == RTT::NewData) {
				cts_port.call_type = rstrt::monitoring::CallTraceSample::CALL_PORT_READ_NEWDATA;
			}
			// out_call_trace_sample_port.write(cts_port);
			// if (((cts_port.call_time - cts_last_send) > cts_send_latest_after && !call_trace_storage.empty()) || (call_trace_storage.size() >= call_trace_storage_size)) {
			if (call_trace_storage.size() >= call_trace_storage_size) {
				// publish if the time limit has been passed or if the storage is full.
				out_call_trace_sample_vec_port.write(call_trace_storage);
				// cts_last_send = cts_port.call_time;
				call_trace_storage.clear();
			}
			call_trace_storage.push_back(cts_port);
		}
		return f;
	}

	template<class T>
	void writePort(RTT::OutputPort<T>& output_port, const T& sample) {
		output_port.write(sample);
		if (useCallTraceIntrospection && usePortTraceIntrospection) {
			cts_port.call_time = time_service->getNSecs();
			cts_port.call_name = output_port.getName();
			cts_port.call_type = rstrt::monitoring::CallTraceSample::CALL_PORT_WRITE;
			// out_call_trace_sample_port.write(cts_port);
			// if (((cts_port.call_time - cts_last_send) > cts_send_latest_after && !call_trace_storage.empty()) || (call_trace_storage.size() >= call_trace_storage_size)) {
			if (call_trace_storage.size() >= call_trace_storage_size) {
				// publish if the time limit has been passed or if the storage is full.
				out_call_trace_sample_vec_port.write(call_trace_storage);
				// cts_last_send = cts_port.call_time;
				call_trace_storage.clear();
			}
			call_trace_storage.push_back(cts_port);
		}
	}

	template<class T>
	void writePort(boost::shared_ptr<RTT::OutputPort<T> > output_port, const T& sample) {
		output_port->write(sample);
		if (useCallTraceIntrospection && usePortTraceIntrospection) {
			cts_port.call_time = time_service->getNSecs();
			cts_port.call_name = output_port->getName();
			cts_port.call_type = rstrt::monitoring::CallTraceSample::CALL_PORT_WRITE;
			// out_call_trace_sample_port.write(cts_port);
			// if (((cts_port.call_time - cts_last_send) > cts_send_latest_after && !call_trace_storage.empty()) || (call_trace_storage.size() >= call_trace_storage_size)) {
			if (call_trace_storage.size() >= call_trace_storage_size) {
				// publish if the time limit has been passed or if the storage is full.
				out_call_trace_sample_vec_port.write(call_trace_storage);
				// cts_last_send = cts_port.call_time;
				call_trace_storage.clear();
			}
			call_trace_storage.push_back(cts_port);
		}
	}

	uint_least64_t getWMECT();
	void setWMECT(const uint_least64_t wmect);

	RTT::os::TimeService* time_service;

	void processCTS(rstrt::monitoring::CallTraceSample& cts);

//protected:
	bool useCallTraceIntrospection;
	bool usePortTraceIntrospection;

private:
	RTT::OutputPort<rstrt::monitoring::CallTraceSample> out_call_trace_sample_port;

	RTT::OutputPort<std::vector<rstrt::monitoring::CallTraceSample> > out_call_trace_sample_vec_port;

	rstrt::monitoring::CallTraceSample cts_start;
	rstrt::monitoring::CallTraceSample cts_configure;
	rstrt::monitoring::CallTraceSample cts_update;
	rstrt::monitoring::CallTraceSample cts_stop;
	rstrt::monitoring::CallTraceSample cts_cleanup;

	rstrt::monitoring::CallTraceSample cts_port;

	uint_least64_t cts_send_latest_after;
	uint_least64_t cts_last_send;

	virtual bool configureHookInternal() = 0;
	virtual bool startHookInternal() = 0;
	virtual void updateHookInternal() = 0;
	virtual void stopHookInternal() = 0;
	virtual void cleanupHookInternal() = 0;

	std::vector<rstrt::monitoring::CallTraceSample> call_trace_storage;
	std::size_t call_trace_storage_size;

	bool cts_send_pro_hook;
	// debug information
	uint_least64_t wmect;
	uint_least64_t wmectI;
};

}
#endif
