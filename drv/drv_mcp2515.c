#include "drv_mcp2515.h"
#include "drv_spi.h"
#include "drv_delay.h"

/* https://github.com/eziya/STM32_SPI_MCP2515 */

#define			 MAX_SPI_TX_BUF_LEN      16
static uint8_t   g_spi_tx_buf[MAX_SPI_TX_BUF_LEN] = {0};

void mcp2515_init(void)
{
    spi_init();
}

bool mcp2515_set_config_mode(void)
{
    /* CANCTRL Register Configuration 모드 설정 */  
//	mcp2515_bit_modify(MCP2515_CANCTRL, CANCTRL_REQOP, 0x80);
	mcp2515_write_byte(MCP2515_CANCTRL, 0x80);

    uint32_t loop = 2;

    do {    
    /* 모드전환 확인 */    
    if((mcp2515_read_byte(MCP2515_CANSTAT) & 0xE0) == 0x80)
        return true;

    loop--;
    } while(loop > 0); 

    return false;
}

bool mcp2515_set_normal_mode(void)
{
    /* CANCTRL Register Normal 모드 설정 */  
//	mcp2515_bit_modify(MCP2515_CANCTRL, CANCTRL_REQOP, 0x00);
	mcp2515_write_byte(MCP2515_CANCTRL, 0x00);

    uint8_t loop = 2;

    do {    
    /* 모드전환 확인 */    
    if((mcp2515_read_byte(MCP2515_CANSTAT) & 0xE0) == 0x00)
        return true;

    loop--;
    } while(loop > 0);

    return false;
}

bool mcp2515_set_loopback_mode(void)
{
    /* CANCTRL Register Normal 모드 설정 */
//    mcp2515_bit_modify(MCP2515_CANCTRL, CANCTRL_REQOP, 0x40);
	mcp2515_write_byte(MCP2515_CANCTRL, 0x40);

    uint8_t loop = 2;

    do {
    /* 모드전환 확인 */
    if((mcp2515_read_byte(MCP2515_CANSTAT) & 0xE0) == 0x00)
        return true;

    loop--;
    } while(loop > 0);

    return false;
}

bool mcp2515_set_sleep_mode(void)
{
    /* CANCTRL Register Sleep 모드 설정 */  
    mcp2515_write_byte(MCP2515_CANCTRL, 0x20);

    uint8_t loop = 10;

    do {    
    /* 모드전환 확인 */    
    if((mcp2515_read_byte(MCP2515_CANSTAT) & 0xE0) == 0x20)
        return true;

    loop--;
    } while(loop > 0);

    return false;
}

uint8_t mcp2515_read_byte(uint8_t address)
{
    return spi_can_read(MCP2515_READ, address);
}

void mcp2515_write_byte(uint8_t address, uint8_t data)
{    
    spi_can_write(MCP2515_WRITE, address, data);
}

void mcp2515_request_to_send(uint8_t instruction)
{    
    spi_can_rst(instruction);
}

uint8_t mcp2515_read_status(void)
{
    return spi_can_status(MCP2515_READ_STATUS);
}

uint8_t mcp2515_get_rx_status(void)
{
    return spi_can_status(MCP2515_RX_STATUS);
}

void mcp2515_bit_modify(uint8_t address, uint8_t mask, uint8_t data)
{    
    spi_can_bit_modify(MCP2515_BIT_MOD, address, mask, data);
}

void mcp2515_reset(void)
{
    spi_can_rst(MCP2515_RESET);
}


void mcp2515_write_byte_sequence(uint8_t startAddress, uint8_t endAddress, uint8_t *data)
{
    uint8_t total_len = MIN(endAddress - startAddress + 2, MAX_SPI_TX_BUF_LEN);
//    memset(g_spi_tx_buf, 0x00, MAX_SPI_TX_BUF_LEN);

    g_spi_tx_buf[0] = startAddress;
    memcpy(&g_spi_tx_buf[1], data, total_len);
    spi_can_load_buffer(MCP2515_WRITE, g_spi_tx_buf, total_len);
}

void mcp2515_read_rx_sequence(uint8_t instruction, uint8_t *data, uint8_t length)
{
//	memset(g_spi_tx_buf, 0x00, MAX_SPI_TX_BUF_LEN);
	spi_can_read_rx_buffer(instruction, data, length);
}

void mcp2515_load_tx_sequence(uint8_t instruction, uint8_t *idReg, uint8_t dlc, uint8_t *data)
{
    uint8_t total_len = MIN(4 + 1 + dlc, MAX_SPI_TX_BUF_LEN);
    memset(g_spi_tx_buf, 0x00, MAX_SPI_TX_BUF_LEN);

    memcpy(g_spi_tx_buf, idReg, 4);
    g_spi_tx_buf[4] = dlc;
    memcpy(&g_spi_tx_buf[5], data, dlc);

    spi_can_load_buffer(instruction, g_spi_tx_buf, total_len);
}


