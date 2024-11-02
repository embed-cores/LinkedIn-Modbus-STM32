#include <string.h>
#include "main.h"
#include "modbus.h"

#define SLAVE_ADDR			1
#define MAX_REGISTERS		33
#define MAX_COILS			44
#define MODBUS_BUF_SIZE		256

// Exceptions
#define ILLEGAL_FUNCTION		0x01
#define ILLEGAL_DATA_ADDRESS	0x02
#define ILLEGAL_DATA_VALUE		0x03
#define SLAVE_DEVICE_FAILURE	0x04
#define ACKNOWLEDGE				0x05
#define SLAVE_DEVICE_BUSY		0x06


static uint16_t registers_mem[MAX_REGISTERS];
static uint8_t coils_mem[8];

static uint8_t tx_buf[256];
static uint8_t rx_buf[256];
static int tx_len;		// minus CRC
static int rx_len;
static int ev_new_data;
static int ev_send_finished;
static uint32_t last_poll_tm;
static int master_timeout;	// When master has not been polled for 10 seconds

static const int8_t read_only_regs[MAX_REGISTERS] =
{ 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 1, 1, 0, 0, 0 };

static const int8_t read_only_coils[MAX_COILS] =
{ 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0 };

static const uint8_t table_crc_hi[] =
{ 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00,
		0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00,
		0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00,
		0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00,
		0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00,
		0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00,
		0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01,
		0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00,
		0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00,
		0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00,
		0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00,
		0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00,
		0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00,
		0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
		0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00,
		0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00,
		0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00,
		0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00,
		0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00,
		0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00,
		0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00,
		0xC1, 0x81, 0x40 };

static const uint8_t table_crc_lo[] =
{ 0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05,
		0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A,
		0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B,
		0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14,
		0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11,
		0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36,
		0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF,
		0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28,
		0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D,
		0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22,
		0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63,
		0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C,
		0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69,
		0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE,
		0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5, 0x77,
		0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50,
		0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55,
		0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A,
		0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B,
		0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 0x44,
		0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41,
		0x81, 0x80, 0x40 };

// ------------------------------------------------------------------

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
	if(huart == &huart1)
	{
		ev_new_data = 1;
		rx_len = size;
	}
}

// ------------------------------------------------------------------

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
		ev_send_finished = 1;
}

// ------------------------------- P R I V A T E ------------------------------- //

static int crc16_check(void)
{
	uint8_t crc_hi = 0xFF;
	uint8_t crc_lo = 0xFF;
	uint8_t idx;

	for (int i = 0; i < rx_len - 2; i++)
	{
		idx = crc_lo ^ rx_buf[i];
		crc_lo = crc_hi ^ table_crc_hi[idx];
		crc_hi = table_crc_lo[idx];
	}

	if (crc_lo != rx_buf[rx_len - 2])
		return 0;

	if (crc_hi != rx_buf[rx_len - 1])
		return 0;

	return 1;
}

// ------------------------------------------------------------------

static void crc16_add(uint16_t len)
{
	uint8_t crc_hi = 0xFF;
	uint8_t crc_lo = 0xFF;
	uint8_t idx;

	for (int i = 0; i < len; i++)
	{
		idx = crc_lo ^ tx_buf[i];
		crc_lo = crc_hi ^ table_crc_hi[idx];
		crc_hi = table_crc_lo[idx];
	}

	tx_buf[len] = crc_lo;
	tx_buf[len + 1] = crc_hi;
}

// ------------------------------------------------------------------

static void SendResponse()
{
	tx_buf[0] = SLAVE_ADDR;
	crc16_add(tx_len);
	tx_len += 2;
	HAL_Delay(3);
	HAL_GPIO_WritePin(MDB_DIR_GPIO_Port, MDB_DIR_Pin, GPIO_PIN_SET);
	for (int i = 0; i < 100; i++);
	HAL_UART_Transmit_IT(&huart1, tx_buf, tx_len);
}

// ------------------------------------------------------------------

static void Exception(uint8_t ex_code)
{
	tx_buf[1] = 0x80 | rx_buf[1];
	tx_buf[2] = ex_code;
	tx_len = 3;
	SendResponse();
}

// ------------------------------------------------------------------

static void read_registers(void)
{
	uint16_t start_addr;
	uint16_t regs_num;

	start_addr = rx_buf[2];
	start_addr <<= 8;
	start_addr |= rx_buf[3];

	regs_num = rx_buf[4];
	regs_num <<= 8;
	regs_num |= rx_buf[5];

	if ((regs_num < 1) || (regs_num > MAX_REGISTERS))
	{
		Exception(ILLEGAL_DATA_VALUE);
		return;
	}

	uint16_t end_addr = start_addr + regs_num - 1;
	if (end_addr >= MAX_REGISTERS)
	{
		Exception(ILLEGAL_DATA_ADDRESS);
		return;
	}

	tx_buf[1] = rx_buf[1];
	tx_buf[2] = regs_num * 2;
	tx_len = 3;

	for (int i = 0; i < regs_num; i++)
	{
		tx_buf[tx_len++] = registers_mem[start_addr] >> 8;
		tx_buf[tx_len++] = registers_mem[start_addr];
		start_addr++;
	}

	SendResponse();
}

// ------------------------------------------------------------------

static void read_coils(void)
{
	uint16_t start_addr;
	uint16_t coils_num;

	start_addr = rx_buf[2];
	start_addr <<= 8;
	start_addr |= rx_buf[3];

	coils_num = rx_buf[4];
	coils_num <<= 8;
	coils_num |= rx_buf[5];

	if ((coils_num < 1) || (coils_num > MAX_COILS))
	{
		Exception(ILLEGAL_DATA_VALUE);
		return;
	}

	uint16_t end_addr = start_addr + coils_num - 1;
	if (end_addr >= MAX_COILS)
	{
		Exception(ILLEGAL_DATA_VALUE);
		return;
	}

	tx_buf[1] = rx_buf[1];
	tx_buf[2] = (coils_num / 8) + ((coils_num % 8) > 0 ? 1 : 0);
	tx_len = 3;

	int start_byte = start_addr / 8;
	int src_bit_pos = start_addr % 8;
	int dest_bit_pos = 0;

	tx_buf[tx_len] = 0;

	for (int i = 0; i < coils_num; i++)
	{
		tx_buf[tx_len] |= ((coils_mem[start_byte] >> src_bit_pos) & 0x01)
				<< dest_bit_pos;

		src_bit_pos++;
		dest_bit_pos++;

		if (src_bit_pos > 7)
		{
			src_bit_pos = 0;
			start_byte++;
		}

		if (dest_bit_pos > 7)
		{
			dest_bit_pos = 0;
			tx_len++;
			tx_buf[tx_len] = 0;
		}
	}

	if ((coils_num % 8) != 0)
		tx_len++;

	SendResponse();
}

// ------------------------------------------------------------------

static void preset_single_register(void)
{
	uint16_t start_addr;

	start_addr = rx_buf[2];
	start_addr <<= 8;
	start_addr |= rx_buf[3];

	if (start_addr >= MAX_REGISTERS)
	{
		Exception(ILLEGAL_DATA_ADDRESS);
		return;
	}

	if (read_only_regs[start_addr])
	{
		Exception(ILLEGAL_DATA_ADDRESS);
		return;
	}

	registers_mem[start_addr] = rx_buf[4];
	registers_mem[start_addr] <<= 8;
	registers_mem[start_addr] |= rx_buf[5];

	// Prepare Response
	tx_buf[1] = rx_buf[1];
	tx_buf[2] = rx_buf[2];
	tx_buf[3] = rx_buf[3];
	tx_buf[4] = rx_buf[4];
	tx_buf[5] = rx_buf[5];
	tx_len = 6;

	if(start_addr == MBREG_FLOW_SENSOR)
		process_reset_flow_counter();

	SendResponse();
}

// ------------------------------------------------------------------

static void preset_multiple_registers(void)
{
	uint16_t start_addr;
	uint16_t regs_num;

	start_addr = rx_buf[2];
	start_addr <<= 8;
	start_addr |= rx_buf[3];

	regs_num = rx_buf[4];
	regs_num <<= 8;
	regs_num |= rx_buf[5];

	if ((regs_num < 1) || (regs_num > MAX_REGISTERS))
	{
		Exception(ILLEGAL_DATA_VALUE);
		return;
	}

	uint16_t end_addr = start_addr + regs_num - 1;
	if (end_addr >= MAX_REGISTERS)
	{
		Exception(ILLEGAL_DATA_ADDRESS);
		return;
	}

	int idx = 7;
	for (int i = 0; i < regs_num; i++)
	{
		if (read_only_regs[start_addr])
		{
			Exception(ILLEGAL_DATA_ADDRESS);
			return;
		}

		registers_mem[start_addr] = rx_buf[idx++];
		registers_mem[start_addr] <<= 8;
		registers_mem[start_addr] |= rx_buf[idx++];
		start_addr++;
	}

	// Prepare Response
	tx_buf[1] = rx_buf[1];
	tx_buf[2] = rx_buf[2];
	tx_buf[3] = rx_buf[3];
	tx_buf[4] = rx_buf[4];
	tx_buf[5] = rx_buf[5];
	tx_len = 6;

	SendResponse();
}

// ------------------------------------------------------------------

static void force_single_coil(void)
{
	uint16_t start_addr;

	start_addr = rx_buf[2];
	start_addr <<= 8;
	start_addr |= rx_buf[3];

	if (start_addr >= MAX_COILS)
	{
		Exception(ILLEGAL_DATA_ADDRESS);
		return;
	}

	if (read_only_coils[start_addr])
	{
		Exception(ILLEGAL_DATA_ADDRESS);
		return;
	}

	int start_byte = start_addr / 8;
	uint8_t mask = 0x01 << (start_addr % 8);

	if ((rx_buf[4] == 0xFF) && (rx_buf[5] == 0x00))
		coils_mem[start_byte] |= mask;
	else if ((rx_buf[4] == 0x00) && (rx_buf[5] == 0x00))
		coils_mem[start_byte] &= ~mask;

	tx_buf[1] = rx_buf[1];
	tx_buf[2] = rx_buf[2];
	tx_buf[3] = rx_buf[3];
	tx_buf[4] = rx_buf[4];
	tx_buf[5] = rx_buf[5];
	tx_len = 6;

	SendResponse();
}

// ------------------------------------------------------------------

static void force_multiple_coils(void)
{
	uint16_t start_addr;
	uint16_t coils_num;

	start_addr = rx_buf[2];
	start_addr <<= 8;
	start_addr |= rx_buf[3];

	coils_num = rx_buf[4];
	coils_num <<= 8;
	coils_num |= rx_buf[5];

	if ((coils_num < 1) || (coils_num > MAX_COILS))
	{
		Exception(ILLEGAL_DATA_VALUE);
		return;
	}

	uint16_t end_addr = start_addr + coils_num - 1;
	if (end_addr >= MAX_COILS)
	{
		Exception(ILLEGAL_DATA_ADDRESS);
		return;
	}

	int start_byte = start_addr / 8;
	int dest_bit_pos = start_addr % 8;
	int src_bit_pos = 0;
	int idx = 7;

	for (int i = 0; i < coils_num; i++)
	{
		if (read_only_coils[start_addr + i])
		{
			Exception(ILLEGAL_DATA_ADDRESS);
			return;
		}

		if ((rx_buf[idx] >> src_bit_pos) & 0x01)
			coils_mem[start_byte] |= (0x01 << dest_bit_pos);
		else
			coils_mem[start_byte] &= ~(0x01 << dest_bit_pos);

		src_bit_pos++;
		dest_bit_pos++;

		if (src_bit_pos > 7)
		{
			src_bit_pos = 0;
			idx++;
		}

		if (dest_bit_pos > 7)
		{
			dest_bit_pos = 0;
			start_byte++;
		}
	}

	tx_buf[1] = rx_buf[1];
	tx_buf[2] = rx_buf[2];
	tx_buf[3] = rx_buf[3];
	tx_buf[4] = rx_buf[4];
	tx_buf[5] = rx_buf[5];
	tx_len = 6;

	SendResponse();
}
// -------------------------------- P U B L I C -------------------------------- //

void modbus_init(void)
{
	memset(registers_mem, 0, 30 * sizeof(uint16_t));
	memset(coils_mem, 0, 8);

	ev_new_data = 0;
	ev_send_finished = 0;
	last_poll_tm = HAL_GetTick();
	master_timeout = 0;

	HAL_UART_Transmit_IT(&huart1, (uint8_t*) &rx_len, 4);	// Start Hang Fix
	HAL_GPIO_WritePin(MDB_DIR_GPIO_Port, MDB_DIR_Pin, GPIO_PIN_RESET);
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_buf, MODBUS_BUF_SIZE);
}

// ------------------------------------------------------------------

int modbus_coil_get(int id)
{
	uint8_t reg, mask;

	if (id >= MAX_COILS)
		Error_Handler();

	reg = coils_mem[id / 8];
	mask = 0x01 << (id % 8);
	return (reg & mask) ? 1 : 0;
}

// ------------------------------------------------------------------

void modbus_coil_set(int id)
{
	uint8_t mask;

	if (id >= MAX_COILS)
		Error_Handler();

	mask = 0x01 << (id % 8);
	coils_mem[id / 8] |= mask;
}

// ------------------------------------------------------------------

void modbus_coil_reset(int id)
{
	uint8_t mask;

	if (id >= MAX_COILS)
		Error_Handler();

	mask = 0x01 << (id % 8);
	coils_mem[id / 8] &= ~mask;
}

// ------------------------------------------------------------------

void modbus_coil_toggle(int id)
{
	uint8_t mask;

	if (id >= MAX_COILS)
		Error_Handler();

	mask = 0x01 << (id % 8);
	if (coils_mem[id / 8] & mask)
		coils_mem[id / 8] &= ~mask;
	else
		coils_mem[id / 8] |= mask;
}

// ------------------------------------------------------------------

uint16_t modbus_register_get(int id)
{
	if (id >= MAX_REGISTERS)
		Error_Handler();

	return registers_mem[id];
}

// ------------------------------------------------------------------

void modbus_register_set(int id, uint16_t val)
{
	if (id >= MAX_REGISTERS)
		Error_Handler();

	registers_mem[id] = val;
}

// ------------------------------------------------------------------

void modbus_update(void)
{
	master_timeout = (HAL_GetTick() - last_poll_tm) > 10000;

	if (ev_new_data)
	{
		ev_new_data = 0;

		if ((rx_len == 0) || (rx_len > 256))
		{
			HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_buf, MODBUS_BUF_SIZE);
			return;
		}

		// Slave Address Check
		if (rx_buf[0] != SLAVE_ADDR)
		{
			HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_buf, MODBUS_BUF_SIZE);
			return;
		}

		// CRC Check (No Response when failed)
		if (!crc16_check())
		{
			HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_buf, MODBUS_BUF_SIZE);
			return;
		}

		last_poll_tm = HAL_GetTick();
		switch (rx_buf[1])
		{
		// Read Coils
		// Read Inputs
		case 0x01:
		case 0x02:
			read_coils();
			break;

			// Read Holding Registers
			// Read Input Registers
		case 0x03:
		case 0x04:
			read_registers();
			break;

			// Preset Single Register
		case 0x06:
			preset_single_register();
			break;

			// Preset Multiple Registers
		case 0x10:
			preset_multiple_registers();
			break;

			// Force Single Coil
		case 0x05:
			force_single_coil();
			break;

			// Force Multiple Coils
		case 0x0F:
			force_multiple_coils();
			break;

		default:
			Exception(ILLEGAL_FUNCTION);
			break;
		}
	}

	if (ev_send_finished)
	{
		ev_send_finished = 0;
		HAL_GPIO_WritePin(MDB_DIR_GPIO_Port, MDB_DIR_Pin, GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_buf, MODBUS_BUF_SIZE);
	}
}

// ------------------------------------------------------------------

