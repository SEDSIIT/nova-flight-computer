#include "storage_driver.h"
#include "w25qxx.h"

#if (_STORAGE_DEBUG == 1)
#include <stdio.h>
#endif


void pack_data_storage(data_storage_t *unpacked_data, uint8_t *flash_buffer)
{
	// time
	flash_buffer[0] = unpacked_data->time.uint8[0];
	flash_buffer[1] = unpacked_data->time.uint8[1];
	flash_buffer[2] = unpacked_data->time.uint8[2];
	flash_buffer[3] = unpacked_data->time.uint8[3];

	// event flags
	flash_buffer[4] = unpacked_data->event_flags;

	// system flags
	flash_buffer[5] = unpacked_data->system_flags;

	// temperature
	flash_buffer[6] = unpacked_data->temperature;

	// pressure
	flash_buffer[7] = unpacked_data->pressure.uint8[0];
	flash_buffer[8] = unpacked_data->pressure.uint8[1];
	flash_buffer[9] = unpacked_data->pressure.uint8[2];
	flash_buffer[10] = unpacked_data->pressure.uint8[3];

	// altitude
	flash_buffer[11] = unpacked_data->altitude.uint8[0];
	flash_buffer[12] = unpacked_data->altitude.uint8[1];
	flash_buffer[13] = unpacked_data->altitude.uint8[2];
	flash_buffer[14] = unpacked_data->altitude.uint8[3];

	// speed
	flash_buffer[15] = unpacked_data->speed.uint8[0];
	flash_buffer[16] = unpacked_data->speed.uint8[1];

	// accel_x
	flash_buffer[17] = unpacked_data->accel_x.uint8[0];
	flash_buffer[18] = unpacked_data->accel_x.uint8[1];
	flash_buffer[19] = unpacked_data->accel_x.uint8[2];
	flash_buffer[20] = unpacked_data->accel_x.uint8[3];

	// accel_y
	flash_buffer[21] = unpacked_data->accel_y.uint8[0];
	flash_buffer[22] = unpacked_data->accel_y.uint8[1];
	flash_buffer[23] = unpacked_data->accel_y.uint8[2];
	flash_buffer[24] = unpacked_data->accel_y.uint8[3];

	// accel_z
	flash_buffer[25] = unpacked_data->accel_z.uint8[0];
	flash_buffer[26] = unpacked_data->accel_z.uint8[1];
	flash_buffer[27] = unpacked_data->accel_z.uint8[2];
	flash_buffer[28] = unpacked_data->accel_z.uint8[3];

	// gyro_x
	flash_buffer[29] = unpacked_data->gyro_x.uint8[0];
	flash_buffer[30] = unpacked_data->gyro_x.uint8[1];
	flash_buffer[31] = unpacked_data->gyro_x.uint8[2];
	flash_buffer[32] = unpacked_data->gyro_x.uint8[3];

	// gyro_y
	flash_buffer[33] = unpacked_data->gyro_y.uint8[0];
	flash_buffer[34] = unpacked_data->gyro_y.uint8[1];
	flash_buffer[35] = unpacked_data->gyro_y.uint8[2];
	flash_buffer[36] = unpacked_data->gyro_y.uint8[3];

	// gyro_z
	flash_buffer[37] = unpacked_data->gyro_z.uint8[0];
	flash_buffer[38] = unpacked_data->gyro_z.uint8[1];
	flash_buffer[39] = unpacked_data->gyro_z.uint8[2];
	flash_buffer[40] = unpacked_data->gyro_z.uint8[3];

	// mag_x
	flash_buffer[41] = unpacked_data->mag_x.uint8[0];
	flash_buffer[42] = unpacked_data->mag_x.uint8[1];
	flash_buffer[43] = unpacked_data->mag_x.uint8[2];
	flash_buffer[44] = unpacked_data->mag_x.uint8[3];

	// mag_y
	flash_buffer[45] = unpacked_data->mag_y.uint8[0];
	flash_buffer[46] = unpacked_data->mag_y.uint8[1];
	flash_buffer[47] = unpacked_data->mag_y.uint8[2];
	flash_buffer[48] = unpacked_data->mag_y.uint8[3];

	// mag_z
	flash_buffer[49] = unpacked_data->mag_z.uint8[0];
	flash_buffer[50] = unpacked_data->mag_z.uint8[1];
	flash_buffer[51] = unpacked_data->mag_z.uint8[2];
	flash_buffer[52] = unpacked_data->mag_z.uint8[3];

	// Latitude
	flash_buffer[53] = unpacked_data->latitude.uint8[0];
	flash_buffer[54] = unpacked_data->latitude.uint8[1];
	flash_buffer[55] = unpacked_data->latitude.uint8[2];
	flash_buffer[56] = unpacked_data->latitude.uint8[3];

	// Longitude
	flash_buffer[57] = unpacked_data->longitude.uint8[0];
	flash_buffer[58] = unpacked_data->longitude.uint8[1];
	flash_buffer[59] = unpacked_data->longitude.uint8[2];
	flash_buffer[60] = unpacked_data->longitude.uint8[3];

	// GPS altitude
	flash_buffer[61] = unpacked_data->gps_altitude.uint8[0];
	flash_buffer[62] = unpacked_data->gps_altitude.uint8[1];

	// Number of Satellites;
	flash_buffer[63] = unpacked_data->satellites;
}

void pack_data_transmit(data_transmit_t *unpacked_data, uint8_t *transmit_buffer)
{

	// time
	transmit_buffer[0] = unpacked_data->time.uint8[0];
	transmit_buffer[1] = unpacked_data->time.uint8[1];
	transmit_buffer[2] = unpacked_data->time.uint8[2];
	transmit_buffer[3] = unpacked_data->time.uint8[3];

	// event flags
	transmit_buffer[4] = unpacked_data->event_flags;

	// system flags
	transmit_buffer[5] = unpacked_data->system_flags;

	// system voltage
	transmit_buffer[6] = unpacked_data->system_voltage;

	// pressure
	transmit_buffer[7] = unpacked_data->pressure.uint8[0];
	transmit_buffer[8] = unpacked_data->pressure.uint8[1];
	transmit_buffer[9] = unpacked_data->pressure.uint8[2];
	transmit_buffer[10] = unpacked_data->pressure.uint8[3];

	// altitude
	transmit_buffer[11] = unpacked_data->altitude.uint8[0];
	transmit_buffer[12] = unpacked_data->altitude.uint8[1];
	transmit_buffer[13] = unpacked_data->altitude.uint8[2];
	transmit_buffer[14] = unpacked_data->altitude.uint8[3];

	// speed
	transmit_buffer[15] = unpacked_data->speed.uint8[0];
	transmit_buffer[16] = unpacked_data->speed.uint8[1];
	transmit_buffer[17] = unpacked_data->speed.uint8[2];
	transmit_buffer[18] = unpacked_data->speed.uint8[3];

	// Latitude
	transmit_buffer[19] = unpacked_data->latitude.uint8[0];
	transmit_buffer[20] = unpacked_data->latitude.uint8[1];
	transmit_buffer[21] = unpacked_data->latitude.uint8[2];
	transmit_buffer[22] = unpacked_data->latitude.uint8[3];

	// Longitude
	transmit_buffer[23] = unpacked_data->longitude.uint8[0];
	transmit_buffer[24] = unpacked_data->longitude.uint8[1];
	transmit_buffer[25] = unpacked_data->longitude.uint8[2];
	transmit_buffer[26] = unpacked_data->longitude.uint8[3];

	// GPS altitude
	transmit_buffer[27] = unpacked_data->gps_altitude.uint8[0];
	transmit_buffer[28] = unpacked_data->gps_altitude.uint8[1];
	transmit_buffer[29] = unpacked_data->gps_altitude.uint8[2];
	transmit_buffer[30] = unpacked_data->gps_altitude.uint8[3];

	// Number of Satellites;
	transmit_buffer[31] = unpacked_data->satellites;
}

void unpack_data_storage(data_storage_t *packed_data, uint8_t *flash_buffer)
{

}

void unpack_data_telemetry(data_telemetry_t *packed_data, uint8_t *transmit_buffer)
{

}
