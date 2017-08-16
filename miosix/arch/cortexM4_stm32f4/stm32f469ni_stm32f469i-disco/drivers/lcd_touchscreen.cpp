#ifdef LCD_TOUCHSCREEN 

#include "drivers/stm32f2_f4_i2c.h"
#include "lcd_touchscreen.h"
#include <kernel/scheduler/scheduler.h>

#define LCD_I2C_ADDR ((unsigned char) 0x54) 
#define BUFLEN (0xbd)

using namespace miosix;

typedef Gpio<GPIOJ_BASE,5> INT;
typedef Gpio<GPIOH_BASE,7> RST;

static I2C1Driver& i2c = I2C1Driver::instance();
Thread *reading = 0; // Thread waiting to read
static Mutex mutex;

TouchscreenDriver& TouchscreenDriver::instance()
{
    static TouchscreenDriver singleton;
    return singleton;
    
}


void __attribute__((naked)) EXTI9_5_IRQHandler() {

    saveContext();
    asm volatile("bl _Z8exti9irqv");
    restoreContext();
}

void exti9irq() {
    if (reading) {
        reading->IRQwakeup();
        if(reading->IRQgetPriority()> Thread::IRQgetCurrentThread()->IRQgetPriority()) {
            Scheduler::IRQfindNextThread();
        }
	reading = 0;
    }

}


void reset() {

    /*
     * Send a low pulse on the reset line for more than 1ms
     */
    RST::low();
    delayUs(1500);
    RST::high();
}

void interrupt_wake_up() {

    /*
     * The CTPM can wake up from hibernation by receiving a low pulse on the
     * interrupt line. The pulse must last between 0.5 ms and 1 ms.
     *
     * After sending the pulse, INT needs to be set to input mode again to
     * work as interrupt line.
     */
    INT::mode(Mode::OUTPUT);
    INT::low();
    delayUs(800);
    INT::high();

    INT::mode(Mode::INPUT);

}

void TouchscreenDriver::init() {

    /*
     * Interrupt registers configuration
     */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; /* Enable SYSCFG for EXTI-GPIO mapping */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOJEN; /* Enable GPIO J for interrupt */
    RCC_SYNC();

    INT::mode(Mode::INPUT);
    EXTI->IMR |= EXTI_IMR_MR5; /* Set interrupt mask bit */
    EXTI->FTSR |= EXTI_FTSR_TR5; /* Configure interrupt for falling edge */
    EXTI->PR |= EXTI_PR_PR5; /* Clear pending bit */
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI5_PJ; /*Configure EXTI5 for pin PJ5*/

    /*
     * Inizialize NVIC interrupt line corresponding to EXTI5
     */
    NVIC_SetPriority(EXTI9_5_IRQn,10); //Same priority as i2c bus
    NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
    NVIC_EnableIRQ(EXTI9_5_IRQn);

    RST::mode(Mode::OUTPUT); /* Initialize reset line */

    i2c.init();


    /*
     * Set all writable registers to their default values
     */

    uint8_t device_mode = 0x00; // Set the CTPM to Working Mode
    //uint8_t thresh
    //filter <--- default value for these not specified on datasheet
    uint8_t ctrl = 0x01; // Set the CTPM to switch to monitor mode with no touching
    uint8_t time_enter_monitor = 0x0A; // Timeout to enter monitor mode when no touch
    // report rate in active mode
    uint8_t period_monitor = 0x28; // Report rate in monitor mode
    uint8_t radian = 0x0A; // Minimum angle for rotation
    uint8_t offset_lr = 0x19; // Max offset for left and right gesture
    uint8_t offset_ud = 0x19; // Max offset for up and down gesture
    uint8_t distance_rl = 0x19; // Max distance for left and right gesture
    uint8_t distance_ud = 0x19; // Max distance for up and down gesture
    uint8_t distance_zoom = 0x32; // Max distance for up and down gesture

    //uint8_t g_mode = 0x01; // Set the CTPM to Interrupt Trigger mode
    uint8_t g_mode = 0x00; // Set the CTPM to Interrupt Polling mode
    uint8_t power_mode = 0x00; 
    uint8_t state = 0x01; // Operating mode

    /* 
     * Send all initialization values to the CTPM over I2C
     */
    write_reg(0x00, &device_mode, 1);
    write_reg(0x86, &ctrl, 1);
    write_reg(0x87, &time_enter_monitor, 1);
    write_reg(0x89, &period_monitor, 1);
    write_reg(0x91, &radian, 1);
    write_reg(0x92, &offset_lr, 1);
    write_reg(0x93, &offset_ud, 1);
    write_reg(0x94, &distance_rl, 1);
    write_reg(0x95, &distance_ud, 1);
    write_reg(0x96, &distance_zoom, 1);

    write_reg(0xA4, &g_mode, 1);
    write_reg(0xA5, &power_mode, 1);
    write_reg(0xBC, &state, 1);

}

void TouchscreenDriver::read_gesture(struct gesture_data_t& gesture_data)
{
    /* Clear packet buffer first */
    unsigned char data_array[BUFLEN];
    int i;
    for (i=0; i<BUFLEN; i++) {
        data_array[i] = 0x00;
    }

    {
        FastInterruptDisableLock dLock;
        reading = Thread::getCurrentThread();
        Thread::IRQwait();
        {
            FastInterruptEnableLock eLock(dLock);
            Thread::yield();
        }
    }

    read_reg(0x00, data_array, BUFLEN);

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

#undef LCD_I2C_ADDR 
#undef BUFLEN 
#endif //LCD_TOUCHSCREEN
