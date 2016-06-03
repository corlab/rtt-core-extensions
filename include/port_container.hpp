#ifndef _PORT_CONTAINER_H_
#define _PORT_CONTAINER_H_

#include <string>
#include <rtt/Port.hpp>

namespace cogimon {
    template <class T> class InputPortContainer {
    public:
    	InputPortContainer() : flowstatus(RTT::NoData) {

        }

        ~InputPortContainer(){

        }

        RTT::InputPort<T> port;
        RTT::FlowStatus flowstatus;
        T data;
    };

    template <class T> class OutputPortContainer {
    public:
    	OutputPortContainer() {

        }

        ~OutputPortContainer(){

        }

        RTT::OutputPort<T> port;
        T data;
    };
}
#endif
