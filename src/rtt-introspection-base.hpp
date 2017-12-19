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
	// virtual ~RTTIntrospectionBase() {}

	bool configureHook();
	bool startHook();
	void updateHook();
	void stopHook();
	void cleanupHook();

	bool configureHookInternal() {return true;};
	bool startHookInternal() {return true;};
	void updateHookInternal() {};
	void stopHookInternal() {};
	void cleanupHookInternal() {};


protected:
	bool useCallTraceIntrospection;

	template<class T>
	RTT::FlowStatus readPort(RTT::InputPort<T>& input_port, RTT::base::DataSourceBase::shared_ptr source, bool copy_old_data = true);

	template<class T>
	RTT::FlowStatus readPort(RTT::InputPort<T>& input_port, typename RTT::base::ChannelElement<T>::reference_t sample, bool copy_old_data = true);

	template<class T>
	bool writePort(RTT::OutputPort<T>& output_port, const T& sample);

private:
	RTT::OutputPort<rstrt::monitoring::CallTraceSample> out_call_trace_sample_port;
	rstrt::monitoring::CallTraceSample cts_start;
	rstrt::monitoring::CallTraceSample cts_configure;
	rstrt::monitoring::CallTraceSample cts_update;
	rstrt::monitoring::CallTraceSample cts_stop;
	rstrt::monitoring::CallTraceSample cts_cleanup;

	rstrt::monitoring::CallTraceSample cts_port;

	RTT::os::TimeService* time_service;
};

}
#endif
