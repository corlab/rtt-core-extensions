/* ============================================================
 *
 * This file is a part of CoSimA (CogIMon) project
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
#ifndef RTT_JOINTAWARE_TASKCONTEXT_HPP
#define RTT_JOINTAWARE_TASKCONTEXT_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#define joint_names_mapping_lookup(memberDict, remoteDict, jointName) memberDict.jointName = remoteDict["jointName"]

namespace cogimon {

class RTTJointAwareTaskContext: public RTT::TaskContext {
public:
	RTTJointAwareTaskContext(std::string const& name);

	bool startHook();

	/**
	 * Implement this function in your own class and
	 * make use of this macro: joint_names_mapping_lookup.
	 *
	 * port_name	name of the associated output port (of this component)
	 * mapping		mapping containing the joint names and associated indexes
	 */
	virtual void retrieveJointMappingsHook(std::string const& port_name,
			std::map<std::string, int> const& mapping) = 0;

	/**
	 * Sometimes further processing is needed after the mappings are received.
	 */
	virtual void processJointMappingsHook() = 0;

protected:
	/**
	 * Tries to retrieve joint mappings for all ports.
	 * This doesn't always makes sense... If you exactly know which ports can be used
	 * to retrieve the joint name mapping, then use retrieveJointMappings(RTT::base::PortInterface* port).
	 *
	 * To be called from the outside or at the end of the configureHook,
	 * whenever the ports are connected to retrieve the joint mapping from.
	 */
	bool retrieveJointMappings();

	/**
	 * Use this to retrieve the joint mapping in a more selective way.
	 */
	bool retrieveJointMappingsSelectively(const std::string& portName);

	/**
	 * Actually retrieves the joint mapping over the port connection to another component.
	 */
	bool getJointNameMappingFromPort(RTT::base::PortInterface* port,
			std::map<std::string, int>& mapping);

	/**
	 * Use this method to check if the joint mapping is fully loaded by now.
	 */
	bool isJointMappingLoaded();

	/**
	 * Gets the remote TaskContext through a connected port.
	 */
	TaskContext* getTaskContextFromPort(RTT::base::PortInterface* port);

	bool is_joint_mapping_loaded;
};

}
#endif
