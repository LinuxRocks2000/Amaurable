#include <Motor.hpp>

#define INDEX_ROTS  125
#define INDEX_SPEED -0.75


class Indexer {
public: // These, Andrew, are your valentines card this year.
    Motor* indexMotor;
    char indexState = 0;
    long indexTicks = 0;
    // 0 = nothing
    // 1 = ball just hit the thing
    // 2 = running a couple encoder tix

    void index(){
        indexState = 1;
        indexMotor -> ZeroEncoder();
    }

    void run(){
        if (indexState == 0){
            indexMotor -> Set(0);
            indexTicks = 0;
        }
        else if (indexState == 1){
            indexTicks ++;
            indexMotor -> Set(INDEX_SPEED);
            if (indexTicks > INDEX_ROTS){
                indexState = 0;
            }
        }
        else if (indexState == 2){
            indexMotor -> Set(-INDEX_SPEED);
            indexState = 0;
        }
        else if (indexState == 3){
            indexMotor -> Set(INDEX_SPEED);
            indexState = 0;
        }
    }

    void barf(){
        indexState = 2;
    }

    void up(){
        indexState = 3;
    }
};
