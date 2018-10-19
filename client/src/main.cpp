#include "Arduino.h"
#include "packet.h"
#include "sensor_packet.h"

// Hacky stuff, don't remove
extern "C" {
int _getpid() { return -1; }
int _kill(int pid, int sig) { return -1; }
int _write() { return -1; }
}

void setup(){
    std::unique_ptr<Packet_Data> data(new Sensor_Packet());
	Packet test (node_id::rasp_pi, packet_id::sensor_packet, std::move(data));
    auto result = test.serialize();
    if(result.first == p_err_none){
        Serial.write(result.second.data(), result.second.size());
    }

}

void loop(){

}
