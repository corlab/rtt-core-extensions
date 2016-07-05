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
#ifndef RTT_JOINT_MAPPING_SERVICE_HPP
#define RTT_JOINT_MAPPING_SERVICE_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/ServiceRequester.hpp>
#include <rtt/OperationCaller.hpp>

namespace cosima {

typedef std::vector<std::string> JointNameVector;
typedef std::map<std::string, JointNameVector> KinematicChainMap;

class JointMappingIF {
public:
	virtual KinematicChainMap getKinematicChainsAndJoints() = 0;
};

class JointMappingServiceReq: public RTT::ServiceRequester {

public:
	JointMappingServiceReq(RTT::TaskContext* owner);
	RTT::OperationCaller<KinematicChainMap()> getKinematicChainsAndJoints;
};

class JointMappingServiceProv: public RTT::Service {

public:
	JointMappingServiceProv(RTT::TaskContext* owner, JointMappingIF* service);
};

}
#endif
