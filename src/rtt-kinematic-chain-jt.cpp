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
		RTTJointAwareTaskContext(name), _feedback_dims(-1), _command_dims(-1), executeContinuously(
				false) {

	this->properties()->addProperty("executeContinuously", executeContinuously);

	this->addOperation("addPortRobotside",
			&RTTKinematicChainJt::addPortRobotside, this, ClientThread);

	this->addOperation("addPortRobotFBside",
			&RTTKinematicChainJt::addPortRobotFBside, this, ClientThread);

	this->addOperation("setChainandCtrlName",
			&RTTKinematicChainJt::setChainandCtrlName, this, ClientThread);

	this->provides("joint_info")->addOperation("getJointMappingForPort",
			&RTTKinematicChainJt::getJointMappingForPort, this,
			RTT::ClientThread);
}

std::map<std::string, int> RTTKinematicChainJt::getJointMappingForPort(
		std::string portName) {
	std::map<std::string, int> result;
	if (is_joint_mapping_loaded) {
		if (_command_port.port.getName() == portName) {
			result = _command_port.joint_name_mapping;
		}
	} else {
		RTT::log(RTT::Error)
				<< "getJointMappingForPort is called before this component had a chance to get the mapping itself."
				<< RTT::endlog();
	}
	return result;
}

void RTTKinematicChainJt::retrieveJointMappingsHook(
		std::string const& port_name,
		std::map<std::string, int> const& mapping) {
	for (unsigned int i = 0; i < _robot_chain_ports.size(); i++) {
		if (_robot_chain_ports[i]->port.getName() == port_name) {
			_robot_chain_ports[i]->joint_name_mapping = mapping;
			break;
		}
	}
}

void RTTKinematicChainJt::processJointMappingsHook() {
	// further processing of mappings is needed
	unsigned floatingIndex = 0;
	std::map<std::string, int>::iterator iter;
	for (unsigned int i = 0; i < _robot_chain_ports.size(); i++) {
		for (iter = _robot_chain_ports[i]->joint_name_mapping.begin();
				iter != _robot_chain_ports[i]->joint_name_mapping.end();
				++iter) {
			_command_port.joint_name_mapping[iter->first] = floatingIndex;
			floatingIndex++;
		}
	}
	if (!connectFunctionCallHandler()) {
		RTT::log(RTT::Error)
				<< "Could not connect to setControlModes IF. Hence I won't be able to to change the ctrl mode automatically."
				<< RTT::endlog();
	}
}

bool RTTKinematicChainJt::addPortRobotFBside(std::string portName, int dim) {
	rstrt::robot::JointState tmpJa(dim);
	tmpJa.torques.fill(0);
	tmpJa.velocities.fill(0);
	tmpJa.torques.fill(0);

	boost::shared_ptr<InputPortContainer<rstrt::robot::JointState> > tmpCont(
			new InputPortContainer<rstrt::robot::JointState>());
	tmpCont->port.setName(portName);
	tmpCont->data = tmpJa;

	// add port to context!
	this->ports()->addPort(tmpCont->port);

	_robot_feedback_ports.push_back(tmpCont);

	_feedback_dims = 0;
	for (int i = 0; i < _robot_feedback_ports.size(); i++) {
		_feedback_dims += _robot_feedback_ports[i]->data.torques.rows();
	}

	return true;
}

void RTTKinematicChainJt::setChainandCtrlName(std::string chainName,
		std::string ctrlName) {
	_chain.push_back(chainName);
	_ctrlmode.push_back(ctrlName);
}

bool RTTKinematicChainJt::addPortRobotside(std::string portName, int dim) {
	rstrt::dynamics::JointTorques tmpJa(dim);
	tmpJa.torques.fill(0);

	boost::shared_ptr<OutputPortContainer<rstrt::dynamics::JointTorques> > tmpCont(
			new OutputPortContainer<rstrt::dynamics::JointTorques>());
	tmpCont->port.setName(portName);
	tmpCont->port.setDataSample(tmpJa);
	tmpCont->data = tmpJa;

	// add port to context!
	this->ports()->addPort(tmpCont->port);

	_robot_chain_ports.push_back(tmpCont);

	_command_dims = 0;
	for (int i = 0; i < _robot_chain_ports.size(); i++) {
		_command_dims += _robot_chain_ports[i]->data.torques.rows();
	}

	return true;
}

bool RTTKinematicChainJt::configureHook() {
	// TODO perhaps needed to call super.configureHook() ?

	if ((_robot_chain_ports.size() > 0) && (_feedback_dims > -1)
			&& (_command_dims > -1)) {
		// create dummy data
		rstrt::robot::JointState tmpFb(_feedback_dims);
		tmpFb.angles.fill(0);
		tmpFb.velocities.fill(0);
		tmpFb.torques.fill(0);

		rstrt::dynamics::JointTorques tmpCmd(_command_dims);
		tmpCmd.torques.fill(0);

		_feedback_port.data = tmpFb;
		_feedback_port.port.setName("feedback");
		// TODO add doc
		_feedback_port.port.setDataSample(tmpFb);

		_command_port.data.torques = tmpCmd.torques;
		_command_port.port.setName("command");

		// add ports to context!
		this->ports()->addPort(_feedback_port.port);
		this->ports()->addPort(_command_port.port);

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
			unsigned int floatingIndex = 0;
			for (unsigned int i = 0; i < _robot_chain_ports.size(); i++) {
				for (unsigned int j = 0;
						j < _robot_chain_ports[i]->data.torques.rows(); j++) {
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
			for (unsigned int i = 0; i < _robot_chain_ports.size(); i++) {
				if (_robot_chain_ports[i]->port.connected()) {
					_robot_chain_ports[i]->port.write(
							_robot_chain_ports[i]->data);
				}
			}
		}
	}

	if (_feedback_port.port.connected()) {
		for (unsigned int i = 0; i < _robot_feedback_ports.size(); i++) {
			if (_robot_feedback_ports[i]->port.connected()) {
				_robot_feedback_ports[i]->flowstatus =
						_robot_feedback_ports[i]->port.read(
								_robot_feedback_ports[i]->data);
				if (_robot_feedback_ports[i]->flowstatus != RTT::NoData) {
					unsigned int floatingIndex = 0;
					for (unsigned int j = 0;
							j < _robot_feedback_ports[i]->data.torques.rows();
							j++) {
						_feedback_port.data.angles(j + floatingIndex) =
								_robot_feedback_ports[i]->data.angles(j);

						_feedback_port.data.velocities(j + floatingIndex) =
								_robot_feedback_ports[i]->data.velocities(j);

						_feedback_port.data.torques(j + floatingIndex) =
								_robot_feedback_ports[i]->data.torques(j);
					}
					floatingIndex +=
							_robot_feedback_ports[i]->data.torques.rows();
				}
			}
		}
		_feedback_port.port.write(_feedback_port.data);
	}
}

bool RTTKinematicChainJt::connectFunctionCallHandler() {
	std::vector<TaskContext*> taskContexts;
	for (unsigned int i = 0; i < _robot_chain_ports.size(); i++) {
		TaskContext* tmp = this->getTaskContextFromPort(
				this->getPort(_robot_chain_ports[i]->port.getName()));
		if ((tmp) && (tmp->getName() != this->getName())) {
			taskContexts.push_back(tmp);
		}
	}
	for (unsigned int i = 0; i < taskContexts.size(); i++) {
		if (taskContexts[i]->provides()->hasOperation("setControlMode")) {
			callers.push_back(taskContexts[i]->getOperation("setControlMode"));
		} else {
			RTT::log(RTT::Info) << "Component " << taskContexts[i]->getName()
					<< " does not implement mandatory operation: setControlMode!"
					<< RTT::endlog();
		}
	}
	if (callers.size() > 0) {
		return true;
	}
	return false;
}

bool RTTKinematicChainJt::startHook() {
	if (!is_joint_mapping_loaded) {
		RTT::log(RTT::Warning)
				<< "this.retrieveJointMappings() needs to be called before this component can be started!"
				<< RTT::endlog();
		return false;
	}

	if ((_chain.size() == 0) || (_ctrlmode.size() == 0)) {
		RTT::log(RTT::Warning)
				<< "this.setChainandCtrlName() needs to be called before this component can be started!"
				<< RTT::endlog();
		return false;
	}

	for (unsigned int i = 0; i < callers.size(); i++) {
		callers[i](_chain[i], _ctrlmode[i]);
		// handle exception and return of false... TODO
	}
	return true;
}

//ORO_CREATE_COMPONENT(cogimon::RTTKinematicChainJt)
ORO_LIST_COMPONENT_TYPE(cogimon::RTTKinematicChainJt)
