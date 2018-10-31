/* main.cpp
 * MTE 380 Design Project
 * Original Author(s): Lyle Cheatham / Eric Murphy-Zaremba / Conner Currie
 * Creation Date: Oct 20 /2018
 */


#include "Arduino.h"
#include "packet.h"
#include "sensor_packet.h"
#include "constants.h"
#include "DemoInterface.hpp"


// Hacky stuff, don't remove
extern "C" {
int _getpid() { return -1; }
int _kill(int pid, int sig) { return -1; }
int _write() { return -1; }
}

DemoInterface* demo;

void setup(){
	/*
	std::unique_ptr<Packet_Data> data(new Sensor_Packet());
	Packet test (node_id::rasp_pi, packet_id::sensor_packet, std::move(data));
    auto result = test.serialize();
    if(result.first == p_err_none){
        Serial.write(result.second.data(), result.second.size());
    }*/
		
	demo = new DemoInterface();
	Serial.println("Starting...");
}

void loop(){
	int8 rcv_char = DemoInterface::get_char();
	demo->run_command(rcv_char);
}
