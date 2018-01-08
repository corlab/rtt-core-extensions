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
#include "IntrospectionReporter.hpp"
#include <rtt/Logger.hpp>

#include <rtt/marsh/PropertyDemarshaller.hpp>
#include <rtt/marsh/PropertyMarshaller.hpp>
#include <iostream>
#include <fstream>
#include <exception>
#include <boost/algorithm/string.hpp>

#include "ocl/Component.hpp"
#include <rtt/types/PropertyDecomposition.hpp>
#include <boost/lexical_cast.hpp>

#include <iostream>
#include <fstream>

ORO_LIST_COMPONENT_TYPE(cosima::IntrospectionReporter)

namespace cosima
{
    using namespace std;
    using namespace RTT;
    using namespace RTT::detail;

    IntrospectionReporter::IntrospectionReporter(std::string name) : TaskContext(name),
                                                                    in_current_flow(RTT::NoData),
                                                                    storage_size(500000),
                                                                    report_policy(ConnPolicy::data(ConnPolicy::LOCK_FREE,true,false)) {
        ctsamples_storage.resize(storage_size);
    }

    bool IntrospectionReporter::configureHook() {
        for (std::string peerName : this->getPeerList()) {
            TaskContext* peer = getPeer(peerName);
            if (!peer) {
                // Couldn't add peer!
                continue;
            }
            // find introspection if
            Service::shared_ptr intro_srv = peer->provides()->getService("introspection");
            if (!intro_srv) {
                // Couldn't find IF!
                continue;
            }
            RTT::base::PortInterface* pi = intro_srv->getPort("out_call_trace_sample_vec_port");
            if (!pi) {
                // Couldn't find port!
                continue;
            }

            RTT::base::OutputPortInterface* portO =  dynamic_cast<RTT::base::OutputPortInterface*>(pi);
            if (!portO) {
                log(Error) << "Can not report OutputPort "<< pi->getName() <<" of Component " << peerName << endlog();
                return false;
            }

            std::shared_ptr<RTT::InputPort<std::vector<rstrt::monitoring::CallTraceSample> > > ipi(new RTT::InputPort<std::vector<rstrt::monitoring::CallTraceSample> >("in_" + peerName + "_port"));

            if (report_policy.type == ConnPolicy::DATA) {
                log(Info) << "Not buffering of data flow connections. You may miss samples." << endlog();
            } else {
                log(Info) << "Buffering ports with size "<< report_policy.size << ", as set in ReportPolicy property." << endlog();
            }

            this->ports()->addEventPort(*ipi.get());
            
            if (portO->connectTo(ipi.get(), report_policy) == false) {
                log(Error) << "Could not connect to OutputPort " << pi->getName() << endlog();
                this->ports()->removePort(ipi->getName());
                continue;
            }
            in_ctsamples_ports.push_back(ipi);
        }

        RTT::log(RTT::Error) << "PORTS: " << in_ctsamples_ports.size() << RTT::endlog();
        return true;
    }

    bool IntrospectionReporter::startHook() {
        return true;
    }

    void IntrospectionReporter::updateHook() {
        if (ctsamples_storage.size() == ctsamples_storage.capacity()) {
            return;
        }

        for (std::shared_ptr<RTT::InputPort<std::vector<rstrt::monitoring::CallTraceSample> > > port : in_ctsamples_ports) {
            in_current_flow = port->read(in_current_var);
            RTT::log(RTT::Error) << "READ PORT " << port->getName() << RTT::endlog();

            if (in_current_flow == RTT::NewData) {
                RTT::log(RTT::Error) << "New Data!" << RTT::endlog();
                if ((ctsamples_storage.size()+in_current_var.size()) <= ctsamples_storage.capacity()) {
                    ctsamples_storage.insert(ctsamples_storage.end(), in_current_var.begin(), in_current_var.end());
                } else {
                    // testing TODO
                    uint elements_left = ctsamples_storage.capacity() - ctsamples_storage.size();
                    for (unsigned i = 0; i < elements_left; i++) {
                        ctsamples_storage.push_back(in_current_var[i]);
                    } 
                }
            }
        }
    }
    
    void IntrospectionReporter::stopHook() {
        RTT::log(RTT::Error) << "Logged Samples " << ctsamples_storage.size() << RTT::endlog();

        ofstream myfile;
        myfile.open ("rtReport.dat");
        myfile << "{\"root\":[\n";

        bool first = true;
        for (rstrt::monitoring::CallTraceSample cts : ctsamples_storage) {
            if (first) {
                first = false;
                myfile << cts;
            } else {
                myfile << ",\n" << cts;
            }
        }
        myfile << "\n]}\n";
        myfile.close();
        RTT::log(RTT::Error) << "Finished writing to rtReport.dat" << RTT::endlog();
    }

    void IntrospectionReporter::cleanupHook() {

    }

}
