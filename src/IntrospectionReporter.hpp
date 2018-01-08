/* ============================================================
 *
 * This file is a part of CoSiMA (CogIMon) project
 *
 * Copyright (C) 2018 by Dennis Leroy Wigand <dwigand at cor-lab dot uni-bielefeld dot de>
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
#ifndef COSIMA_INTROSPECTION_REPORTER_HPP
#define COSIMA_INTROSPECTION_REPORTER_HPP


#include <boost/tuple/tuple.hpp>

#include <rtt/Property.hpp>
#include <rtt/PropertyBag.hpp>
#include <rtt/marsh/MarshallInterface.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/TaskContext.hpp>

#include <rtt/RTT.hpp>

#include <ocl/OCL.hpp>

// RST-RT includes
#include <rst-rt/monitoring/CallTraceSample.hpp>

namespace cosima
{

class IntrospectionReporter : public RTT::TaskContext {

public:

    IntrospectionReporter( std::string name = "IntrospectionReporter" );
    // virtual ~IntrospectionReporter();

    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

private:

    std::vector<std::shared_ptr<RTT::InputPort<std::vector<rstrt::monitoring::CallTraceSample> > > > in_ctsamples_ports;
    std::vector<std::vector<rstrt::monitoring::CallTraceSample> > in_ctsamples_vars;
    // std::vector<RTT::FlowStatus> in_ctsamples_flows;

    std::vector<rstrt::monitoring::CallTraceSample> in_current_var;
    RTT::FlowStatus in_current_flow;

    std::vector<rstrt::monitoring::CallTraceSample> ctsamples_storage;
    uint storage_size;

    RTT::ConnPolicy report_policy;
};

}

#endif
