#ifndef _STORAGE_DRIVER
#define _STORAGE_DRIVER

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include "main.h"

#define _STORAGE_DEBUG 0

typedef union data_4byte
{
	uint8_t uint8[4]; // 1 Byte form ([3]<<24|[2]<<16|...)
	uint32_t uint32;
	int32_t int32;
	float f;
} data_4byte;

typedef union data_2byte
{
	uint8_t uint8[2]; // 1 Byte form
	uint16_t uint16;
	int16_t int16;
} data_2byte;

typedef struct
	{
		// System (7 Bytes)
		union data_4byte time; // Milliseconds (Overflow in ~50 days)
		uint8_t event_flags; // launch, coast_1, sustainer_iginition, coast_2, apogee, drogue_deploy, main_deploy, land
		uint8_t system_flags; //
		uint8_t temperature; // Celsius (specially formatted)

		// Barometer (10 Bytes)
		union data_4byte pressure; // Pascal
		union data_4byte altitude; // Meters
		union data_2byte speed; // meters/second

		// Accelerometers (12 Bytes)
		union data_4byte accel_x; // Acceleration in direction of flight (m/s^2) (will swap accelerometers when sensor is saturated)
		union data_4byte accel_y;
		union data_4byte accel_z;

		// Gyroscope (12 Bytes)
		union data_4byte gyro_x; // degrees/ second
		union data_4byte gyro_y;
		union data_4byte gyro_z;

		// Magnetometer (12 Bytes)
		union data_4byte mag_x; // Gauss
		union data_4byte mag_y;
		union data_4byte mag_z;

		// GPS (11 bytes)
		union data_4byte latitude; // latitude in degrees
		union data_4byte longitude; // longitude in degrees
		union data_2byte gps_altitude; // meters (from GPS)
		uint8_t satellites; // number of satellites for GPS lock

	} data_storage_t; // 64 bytes

typedef struct
	{
	// system (7 bytes)
	union data_4byte time; // Milliseconds
	uint8_t event_flags; // Launch phase
	uint8_t system_flags; // Status of system
	uint8_t system_voltage; // encoded voltage

	// performance metrics (12 bytes)
	union data_4byte pressure; // Pascal
	union data_4byte altitude; // Meters
	union data_4byte speed; // meters/second

	// GPS (13 bytes)
	union data_4byte latitude; // latitude in degrees
	union data_4byte longitude; // longitude in degrees
	union data_4byte gps_altitude; // meters (from GPS)
	uint8_t satellites; // number of satellites for GPS lock

	} data_telemetry_t; // 32 byte message


void pack_data_storage(data_storage_t *unpacked_data, uint8_t *flash_buffer);
void pack_data_telemetry(data_telemetry_t *unpacked_data, uint8_t *transmit_buffer);

void unpack_data_storage(data_storage_t *packed_data, uint8_t *flash_buffer);
void unpack_data_telemetry(data_telemetry_t *packed_data, uint8_t *transmit_buffer);

uint32_t get_next_storage_address(void);

#endif
