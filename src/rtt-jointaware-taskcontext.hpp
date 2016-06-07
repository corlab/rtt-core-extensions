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
#ifndef RTT_JOINTAWARE_TASKCONTEXT_HPP
#define RTT_JOINTAWARE_TASKCONTEXT_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

namespace cogimon {

class RTTJointAwareTaskContext: public RTT::TaskContext {
public:
	RTTJointAwareTaskContext(std::string const& name);

	bool start();

	/**
	 * Implement this function in your own class and
	 * make use of this macro: joint_names_mapping_lookup.
	 *
	 * port_name	name of the associated output port (of this component)
	 * mapping		mapping containing the joint names and associated indexes
	 */
	virtual void retrieveJointMappingsHook(std::string const& port_name,
			std::map<std::string, int> const& mapping) = 0;


protected:
	bool retrieveJointMappings();

	bool getJointNameMappingFromPort(RTT::base::PortInterface* port,
			std::map<std::string, int>& mapping);

	/**
	 * Use this method to check if the joint mapping is fully loaded by now.
	 */
	bool isJointMappingLoaded();

	bool is_joint_mapping_loaded;
};

}
#endif
