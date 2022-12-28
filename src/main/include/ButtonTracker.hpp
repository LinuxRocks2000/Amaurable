#include <frc/GenericHID.h>

typedef uint16_t var; // This is gonna screw me over later. HA!


class ButtonTracker{
public: // Andrew, this is just for you
    frc::GenericHID* stick;
    unsigned int last;
    bool* buttons;
    bool* events;

    ButtonTracker(frc::GenericHID* maura, var fVal, var lVal){
        stick = maura;
        size = fVal;
        last = lVal + 1; // 0 indexing an' crap.

        buttons = (bool*)malloc((last - size) * sizeof(bool));
        memset(buttons, 0, (last - size) * sizeof(bool));

        events = (bool*)malloc((last - size) * sizeof(bool));
        memset(events, 0, (last - size) * sizeof(bool));
    }

    void test(){
        for (var b = 0; b < last - size; b ++) {
            if (stick -> GetRawButton(b + size)){
                buttons[b] = true;
            }
            else {
                if (buttons[b]){
                    events[b] = true;
                    std::cout << "You pressed button #" << b + size << std::endl;
                }
                buttons[b] = false;
            }
        }
    }

    bool GetReleased(var which){
        bool ret = events[which - size];
        events[which - size] = false;
        return ret;
    }

    bool get(var which){
        bool ret = buttons[which - size];
        buttons[which - size] = false;
        return ret;
    }
};
