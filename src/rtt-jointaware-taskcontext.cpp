/* ============================================================
 *
 * This file is a part of RST-RT (CogIMon) project
 *
 * Copyright (C) 2016 by Dennis Leroy Wigand <dwigand at cor-lab dot uni-bielefeld dot de>
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
#include "rtt-jointaware-taskcontext.hpp"
#include <rtt/Operation.hpp>

using namespace cogimon;
using namespace RTT;

#define joint_names_mapping_lookup(memberDict, remoteDict, jointName) memberDict.jointName = remoteDict["jointName"]

RTTJointAwareTaskContext::RTTJointAwareTaskContext(const std::string &name) :
		TaskContext(name), is_joint_mapping_loaded(false) {
	this->addOperation("retrieveJointMappings",
			&RTTJointAwareTaskContext::retrieveJointMappings, this,
			ClientThread).doc(
			"Retrieves the joint mappings from the output ports. This operation needs to be called before start() can be executed.");
}

bool RTTJointAwareTaskContext::start() {
	if (!is_joint_mapping_loaded) {
		RTT::log(RTT::Warning)
				<< "this.retrieveJointMappings() needs to be called before this component can be started!"
				<< RTT::endlog();
		return false;
	}
	return true;
}

bool RTTJointAwareTaskContext::retrieveJointMappings() {
	std::vector<base::PortInterface*> ports = this->ports()->getPorts();
	std::vector<base::PortInterface*>::iterator ports_iter;
	for (ports_iter = ports.begin(); ports_iter != ports.end(); ++ports_iter) {
		std::map<std::string, int> mapping;
		if (getJointNameMappingFromPort(*ports_iter, mapping)) {
			// convert the mapping the way you want, using the following macro: joint_names_mapping_lookup
			retrieveJointMappingsHook((*ports_iter)->getName(), mapping);
		}
	}
	is_joint_mapping_loaded = true;
	return true;
}

bool RTTJointAwareTaskContext::isJointMappingLoaded() {
	return is_joint_mapping_loaded;
}

bool RTTJointAwareTaskContext::getJointNameMappingFromPort(
		RTT::base::PortInterface* port, std::map<std::string, int>& mapping) {
	// get remote task context via the output port
	// TODO validate the current channel, there could be more connections available!
	if (!port->connected()) {
		RTT::log(RTT::Warning) << "Port: " << port->getName()
				<< " needs to be connected, in order to retrieve the joint mapping!"
				<< RTT::endlog();
		return false;
	}

	TaskContext* remoteTaskContext =
			port->getManager()->getCurrentChannel()->getOutputEndPoint()->getPort()->getInterface()->getOwner();

	// get name of the associated input port
	std::string remoteInputPortname =
			port->getManager()->getCurrentChannel()->getOutputEndPoint()->getPort()->getName();

	// check if operation is available
	Service::shared_ptr remoteServicePtr = remoteTaskContext->provides();
	if (remoteServicePtr) {
		if (remoteServicePtr->hasService("joint_info")) {
			if (remoteServicePtr->hasOperation("getJointMappingForPort")) {

				OperationCaller<std::map<std::string, int>(std::string)> opCaller =
						remoteServicePtr->getOperation(
								"getJointMappingForPort");
				if (opCaller.ready()) {
					mapping = opCaller(remoteInputPortname);
					return true;
				} else {
					// throw exception TODO
				}

			}
		}
	}
	return false;
}
