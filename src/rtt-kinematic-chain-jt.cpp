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
#include "rtt-kinematic-chain-jt.hpp"
#include <rtt/Operation.hpp>
#include <string>
#include <fstream>
#include <streambuf>

using namespace cogimon;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

RTTKinematicChainJt::RTTKinematicChainJt(const std::string &name) :
		TaskContext(name), _feedback_dims(-1), _command_dims(-1), executeContinuously(
				false) {

	this->properties()->addProperty("executeContinuously", executeContinuously);
	this->addOperation("configureUserside", &RTTKinematicChainJt::configureUserside, this, ClientThread);
	this->addOperation("addPortRobotside", &RTTKinematicChainJt::addPortRobotside, this, ClientThread);
}

bool RTTKinematicChainJt::configureUserside(int dimFB, int dimCmdInput) {
	this->_feedback_dims = dimFB;
	this->_command_dims = dimCmdInput;
	return true;
}

bool RTTKinematicChainJt::addPortRobotside(std::string portName, int dim) {
	if ((_feedback_dims == -1) || (_command_dims == -1)) {
		RTT::log(RTT::Warning) << "call configureUserside first"
				<< RTT::endlog();
		return false;
	}
	// check already existing dimensions
	int dimAmount = 0;
	for (int i = 0; i < _robot_chain_ports.size(); i++) {
		dimAmount += _robot_chain_ports[i]->data.torques.rows();
	}
	if ((dimAmount + dim) > _command_dims) {
		RTT::log(RTT::Warning) << "Adding this port would in total("
				<< (dimAmount + dim)
				<< ") exceed the dimensions of the command input( "
				<< _command_dims << ")! Hence, skipping this port!"
				<< RTT::endlog();
		return false;
	}

	rstrt::dynamics::JointTorques tmpJa(dim);
	tmpJa.torques.setZero();

	boost::shared_ptr<OutputPortContainer<rstrt::dynamics::JointTorques> > tmpCont(
			new OutputPortContainer<rstrt::dynamics::JointTorques>());
	tmpCont->port.setName(portName);
	tmpCont->port.setDataSample(tmpJa);
	tmpCont->data = tmpJa;

	// add port to context!
	this->ports()->addPort(tmpCont->port);

	_robot_chain_ports.push_back(tmpCont);
	return true;
}

bool RTTKinematicChainJt::configureHook() {
	if ((_robot_chain_ports.size() > 0) && (_feedback_dims > -1)
			&& (_command_dims > -1)) {
		// create dummy data
		rstrt::dynamics::JointTorques tmpFb(_feedback_dims);
		tmpFb.torques.setZero();
		rstrt::dynamics::JointTorques tmpCmd(_command_dims);
		tmpCmd.torques.setZero();

		_feedback_port.data.torques = tmpFb.torques;
		_feedback_port.port.setName("feedback_out");
		// TODO add doc
		_feedback_port.port.setDataSample(tmpFb);

		_command_port.data.torques = tmpCmd.torques;
		_command_port.port.setName("command_in");

		_robot_feedback_port.data.torques = tmpFb.torques;
		_robot_feedback_port.port.setName("robot_fb_in");

		// add ports to context!
		this->ports()->addPort(_feedback_port.port);
		this->ports()->addPort(_command_port.port);
		this->ports()->addPort(_robot_feedback_port.port);

		return true;
	} else {
		RTT::log(RTT::Warning) << "You have to configure the ports first!"
				<< RTT::endlog();
		return false;
	}
}

void RTTKinematicChainJt::updateHook() {
	// read complete input-vector
	if (_command_port.port.connected()) {
		_command_port.flowstatus = _command_port.port.readNewest(
				_command_port.data);

		if ((executeContinuously && _command_port.flowstatus != RTT::NoData)
				|| (!executeContinuously
						&& _command_port.flowstatus == RTT::NewData)) {

			// iterate through output ports
			int floatingIndex = 0;
			for (int i = 0; i < _robot_chain_ports.size(); i++) {
				for (int j = 0; j < _robot_chain_ports[i]->data.torques.rows();
						j++) {
					_robot_chain_ports[i]->data.torques(j) =
							_command_port.data.torques(j + floatingIndex);

					RTT::log(RTT::Debug) << "_robot_chain_ports[" << i
							<< "]->data.torques(" << j
							<< ") = (_command_port.data.torques("
							<< j + floatingIndex << ")): "
							<< (double) _command_port.data.torques(
									j + floatingIndex) << RTT::endlog();
				}
				floatingIndex += _robot_chain_ports[i]->data.torques.rows();
			}
			for (int i = 0; i < _robot_chain_ports.size(); i++) {
				if (_robot_chain_ports[i]->port.connected()) {
					_robot_chain_ports[i]->port.write(
							_robot_chain_ports[i]->data);
				}
			}
		}
	}
	// write feedback to user-side
	if (_robot_feedback_port.port.connected()) {
		_robot_feedback_port.flowstatus = _robot_feedback_port.port.readNewest(
				_robot_feedback_port.data);
		if (_robot_feedback_port.flowstatus == RTT::NewData) {
			if (_feedback_port.port.connected()) {
				_feedback_port.port.write(_robot_feedback_port.data);
			}
		}
	}
}

//ORO_CREATE_COMPONENT(cogimon::RTTKinematicChainJt)
ORO_LIST_COMPONENT_TYPE(cogimon::RTTKinematicChainJt)
