#ifndef __MODBUS_H
#define __MODBUS_H

#include <stdint.h>

// Register ID
#define MBREG_TEST_1				0
#define MBREG_TEST_2				1

// Coil ID
#define MBCOIL_TEST_1				0
#define MBCOIL_TEST_2				1


void modbus_init(void);
int modbus_coil_get(int id);
void modbus_coil_set(int id);
void modbus_coil_reset(int id);
void modbus_coil_toggle(int id);
uint16_t modbus_register_get(int id);
void modbus_register_set(int id, uint16_t val);
void modbus_update(void);


#endif
