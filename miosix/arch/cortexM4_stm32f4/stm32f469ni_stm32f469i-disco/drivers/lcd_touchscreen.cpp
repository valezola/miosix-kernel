#ifdef LCD_TOUCHSCREEN 

#include "drivers/stm32f2_f4_i2c.h"
#include "lcd_touchscreen.h"

#define LCD_I2C_ADDR ((unsigned char) 0x54) 

using namespace miosix;


static I2C1Driver& i2c = I2C1Driver::instance();

TouchscreenDriver& TouchscreenDriver::instance()
{
    static TouchscreenDriver singleton;
    return singleton;
    
}

void TouchscreenDriver::init()
{
    i2c.init();

    /*
     * Set all writable registers to their default values
     */
    //uint8_t device_mode[1] = {0x00}; // Set the CTPM to Working Mode
    //uint8_t ctrl[1] = {0x01}; // Set the CTPM to switch to monitor mode with no touching
    //uint8_t g_mode[1] = {0x01}; // Set the CTPM to Interrupt Trigger mode, as default

    uint8_t device_mode = 0x00; // Set the CTPM to Working Mode
    uint8_t ctrl = 0x01; // Set the CTPM to switch to monitor mode with no touching
    uint8_t g_mode = 0x01; // Set the CTPM to Interrupt Trigger mode, as default

    /* 
     * Send all initialization values to the CTPM over I2C
     */
    write_reg(0x00, &device_mode, 1);
    write_reg(0x86, &ctrl, 1);
    write_reg(0xA4, &g_mode, 1);


}

void TouchscreenDriver::read_gesture(struct gesture_data_t& gesture_data)
{
    /*
     * To read from the FT6206 Touch Screen Module, we have to send a write command with
     * the starting address of the registers to be read. It appears to be possible to
     * read multiple registers in a single read command.
     * 
     */

    uint8_t data_array[15];
    read_reg(0x00, data_array, 15);

    //TODO: fill structure 

}

void TouchscreenDriver::write_reg(unsigned char reg_addr, unsigned char *data, int len)
//void TouchscreenDriver::write_reg(uint8_t reg_addr, void* data, int len)
{
    /*
     * To write a register, we send the register address followed by the data to write
     */

    int i=0;
    uint8_t packet[32];

    packet[0] = reg_addr;
    for (i=0; i<len; i++) {
        packet[i+1] = data[i];
    }

    i2c.send(LCD_I2C_ADDR, packet, len+1);
}

void TouchscreenDriver::read_reg(unsigned char reg_addr, unsigned char *data, int len)
{
    /*
     * To read from a register, we send a write packet containing only the address of the
     * register to be read, followed by a stop bit. Then we send a read command.
     */

    i2c.send(LCD_I2C_ADDR, &reg_addr, 1);
    i2c.recv(LCD_I2C_ADDR, data, len);
}

#endif //LCD_TOUCHSCREEN
