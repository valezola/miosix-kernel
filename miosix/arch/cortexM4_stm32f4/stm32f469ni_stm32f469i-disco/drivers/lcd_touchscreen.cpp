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
     * Read all the touch data as a byte array
     */
    uint8_t data_array[15];
    read_reg(0x00, data_array, 15);

    /*
     * Fill gesture ID and number of touches
     */
    switch( data_array[0x01] )
    {
        case 0x10: gesture_data.gestureID = MOVE_UP; break;
        case 0x14: gesture_data.gestureID = MOVE_RIGHT; break;
        case 0x18: gesture_data.gestureID = MOVE_DOWN; break;
        case 0x1C: gesture_data.gestureID = MOVE_LEFT; break;
        case 0x48: gesture_data.gestureID = ZOOM_IN; break;
        case 0x49: gesture_data.gestureID = ZOOM_OUT; break;
        case 0x00: gesture_data.gestureID = NO_GESTURE; break;
        default: gesture_data.gestureID = NO_GESTURE; break;
    }

    gesture_data.num_touches = data_array[0x02] & 0x0F;

    /*
     * Fill first touch structure
     */
    switch(data_array[0x03] & 0xC0) 
    {
        case 0b00000000: gesture_data.touches[0].event = PRESS_DOWN; break;
        case 0b01000000: gesture_data.touches[0].event = LIFT_UP; break;
        case 0b10000000: gesture_data.touches[0].event = CONTACT; break;
        case 0b11000000: gesture_data.touches[0].event = NO_EVENT; break;
        default: gesture_data.touches[0].event = NO_EVENT; break;
    }
    gesture_data.touches[0].touchID = (data_array[0x05] & 0xF0) >> 4;
    gesture_data.touches[0].area = (data_array[0x08] & 0xF0) >> 4;
    gesture_data.touches[0].weight = data_array[0x07];
    gesture_data.touches[0].x = ((data_array[0x03] & 0x0F) << 8 ) | data_array[0x04];
    gesture_data.touches[0].y = ((data_array[0x05] & 0x0F) << 8 ) | data_array[0x06];

    /*
     * Fill second touch structure
     */
    switch(data_array[0x09] & 0xC0) 
    {
        case 0b00000000: gesture_data.touches[1].event = PRESS_DOWN; break;
        case 0b01000000: gesture_data.touches[1].event = LIFT_UP; break;
        case 0b10000000: gesture_data.touches[1].event = CONTACT; break;
        case 0b11000000: gesture_data.touches[1].event = NO_EVENT; break;
        default: gesture_data.touches[1].event = NO_EVENT; break;
    }
    gesture_data.touches[1].touchID = (data_array[0x0B] & 0xF0) >> 4;
    gesture_data.touches[1].area = (data_array[0x0E] & 0xF0) >> 4;
    gesture_data.touches[1].weight = data_array[0x0D];
    gesture_data.touches[1].x = ((data_array[0x09] & 0x0F) << 8 ) | data_array[0x0A];
    gesture_data.touches[1].y = ((data_array[0x0B] & 0x0F) << 8 ) | data_array[0x0C];

}

void TouchscreenDriver::write_reg(unsigned char reg_addr, unsigned char *data, int len)
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
     * To read from the FT6206 Touch Screen Module, we have to send a write message with
     * the address of the first register to be read. It appears to be possible to
     * read multiple registers in a single read command.
     */
    i2c.send(LCD_I2C_ADDR, &reg_addr, 1);
    i2c.recv(LCD_I2C_ADDR, data, len);
}

#endif //LCD_TOUCHSCREEN
