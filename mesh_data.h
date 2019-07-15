#ifndef MESH_DATA_H
#define MESH_DATA_H

#define ALARM_ON                   0x03
#define ALARM_OFF                  0x00

#define MAX_UNICAST_ADDRESS        63

#define DEFAULT_ARRAY_SIZE          8

typedef struct {
	uint8 alarm_signal;
	uint16 unicast_address;
	uint8 heart_beat;
	uint8 battery_percent;
	uint8 time_out;
}mesh_lpn_data_str;
typedef struct{
	mesh_lpn_data_str* mesh_lpn_data;
	uint16 num_lpn;
	uint16 current_lpn_node;
}mesh_lpn_data_array_t;

uint16 data2message(mesh_lpn_data_str mesh_data) {
	uint16 data = 0x0000;
	data = data | (mesh_data.alarm_signal & 0x01);
	data = data | ((mesh_data.unicast_address & 0x7f) << 1);
	data = data | ((mesh_data.heart_beat & 0x01) << 8);
	data = data | ((mesh_data.battery_percent & 0x7f) << 9);
	return data;
}

mesh_lpn_data_str message2data(uint16 data) {
	mesh_lpn_data_str mesh_data;
	mesh_data.alarm_signal = data & 0x01;
	mesh_data.unicast_address = (data >> 1) & 0x7f;
	mesh_data.heart_beat = (data >> 8) & 0x01;
	mesh_data.battery_percent = (data >> 9) & 0x7f;
	mesh_data.time_out = 0;
	return mesh_data;
}

uint16 get_unicast_address(uint16 message) {
	return (message >> 1) & 0x007f;
}
uint8 get_alarm_signal(uint8 message){
	return message & 0x01 ;
}

#endif
