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
#include "JointMappingService.hpp"

using namespace cosima;
using namespace RTT;

JointMappingServiceReq::JointMappingServiceReq(TaskContext* owner) :
		ServiceRequester("JointMapping", owner), getKinematicChainsAndJoints(
				"getKinematicChainsAndJoints") {
	addOperationCaller(getKinematicChainsAndJoints);
}

JointMappingServiceProv::JointMappingServiceProv(TaskContext* owner, JointMappingIF* service) :
		Service("JointMapping", owner) {
	this->doc(
			"The JointMapping service provides an interface to retrieve all kinematic chains including their associated joints, which are ordered in a particular way.");
	this->addOperation("getKinematicChainsAndJoints",
			&JointMappingIF::getKinematicChainsAndJoints, service).doc(
			"Returns a map containing all available kinematic chains with their associated joints");
}
