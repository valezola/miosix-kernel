#ifdef LCD_TOUCHSCREEN 

#include "drivers/stm32f2_f4_i2c.h"
#include "lcd_touchscreen.h"

#define LCD_I2C_ADDR ((unsigned char) 0x54) 

using namespace miosix;

TouchscreenDriver& TouchscreenDriver::instance()
{
    static TouchscreenDriver singleton;
    return singleton;
    
}

void TouchscreenDriver::init()
{
    static I2C1Driver& i2c = I2C1Driver::instance();
    i2c.init();

    unsigned char device_mode[2];
    device_mode[0] = 0x00; // Address of the Device Mode register in the CTPM
    device_mode[1] = 0x00; // Set the CTPM in Working Mode

    /*
     * Set all writable registers to their default values
     */
    unsigned char ctrl[2];
    ctrl[0] = 0x86; // Address of CTRL register in the CTPM
    ctrl[1] = 0x01; // Set the CTPM to switch to monitor mode with no touching

    /*
     * Set the interrupt working mode of the device to Trigger mode
     * */
    unsigned char g_mode[2];
    g_mode[0] = 0xA4; // Address of the G_MODE register in the CTPM
    g_mode[1] = 0x01; // Set the CTPM in Interrupt Trigger mode, as default

    /* 
     * Send all initialization values to the CTPM over I2C
     */
    i2c.send(LCD_I2C_ADDR, device_mode, 2);
    i2c.send(LCD_I2C_ADDR, ctrl, 2);
    i2c.send(LCD_I2C_ADDR, g_mode, 2);

}

void TouchscreenDriver::read_gesture(struct gesture_data_t& gesture_data)
{
}

#endif //LCD_TOUCHSCREEN
