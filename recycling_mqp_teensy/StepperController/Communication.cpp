#include "Communication.h"

/** See Communication.h
*/
uint8_t send_packet(packet_send_t* packet) {
	Serial.print("<status>");
	int count = Serial.write((uint8_t*) packet, sizeof(packet_send_t));    // todo: remind me why they had to write in binary vs print?
	return count;
}
