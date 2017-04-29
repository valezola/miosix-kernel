#ifdef LCD_TOUCHSCREEN

/**
 * Driver for the TFT LCD Touchscreen of the STM32F469ni board in Miosix
 */
namespace miosix
{

enum gesture_t {
    MOVE_UP, MOVE_RIGHT, MOVE_DOWN, MOVE_LEFT,
    ZOOM_IN, ZOOM_OUT, NO_GESTURE
};

enum event_t {
    PRESS_DOWN,
    LIFT_UP,
    CONTACT,
    NO_EVENT
};

struct touch_data_t {
    enum event_t event;
    unsigned int touchID : 4; //TODO: is bitfield a good idea here?
    unsigned int area : 4; 
    uint8_t weight;
    uint16_t x; 
    uint16_t y;

};

struct gesture_data_t {
    enum gesture_t gestureID;
    unsigned int num_touches : 4; //TODO: is 4 bits, 2 bits or more better?
    struct touch_data_t touches[2];
};

class TouchscreenDriver {
public:
    static TouchscreenDriver& instance();
    void init();

    void read_gesture(struct gesture_data_t& gesture_data);

private:
//    TouchscreenDriver(const TouchscreenDriver&);
//    TouchscreenDriver& operator=(const TouchscreenDriver&);

    TouchscreenDriver() { init(); }
};

} //namespace miosix

#endif //LCD_TOUCHSCREEN
