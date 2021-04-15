#ifndef MQP_COMMS_H_
#define MQP_COMMS_H_

#include <Arduino.h>
#include <inttypes.h>

// The packets the Teensy receives from the Pi TODO
typedef struct packet_recv {
	uint8_t enable;

} packet_recv_t;

// The update packets the Teensy sends to the Pi
typedef struct packet_send {
	uint8_t control_state;
	uint8_t enabled;
	uint8_t limit;
	int32_t stepper_positions[3];
	int8_t stepper_directions[3];
} packet_send_t;

/** Send the status update packet to master through the pi Serial connection
    @param packet the packet to send
    @return TODO
*/
uint8_t send_packet(packet_send_t* packet);

#endif
