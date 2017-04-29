#ifdef LCD_TOUCHSCREEN

#include "drivers/stm32f2_f4_i2c.h"
#include "lcd_touchscreen.h"

using namespace miosix;

TouchscreenDriver& TouchscreenDriver::instance()
{
    static TouchscreenDriver singleton;
    return singleton;
    
}

void TouchscreenDriver::init()
{
}

void TouchscreenDriver::read_gesture(struct gesture_data_t& gesture_data)
{
    //wait for interrupt
    //read from i2c registers 
    //check that slave address is correct: 0x54
    //data packet:
    //
    //A new interrupt is issued by the TouchScreen controller every time a packet is sent
    //This means that, upon receiving an interrupt, only one packet needs to be read
    //(interrupt polling mode)
    //Actually, packet bursts are also possible (interrupt trigger mode)
    //
}

#endif //LCD_TOUCHSCREEN
