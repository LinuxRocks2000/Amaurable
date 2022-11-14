class Drivetrain{
public: // Just for youse, Andrew
    Motor* backLeft;
    Motor* backRight;
    Motor* frontLeft;
    Motor* frontRight;
    double _left = 0;
    double _right = 0;

    void right(double amount){
        _right += amount;
    }

    void left(double amount){
        _left += amount;
    }

    void turn(double amount){
        _right += amount;
        _left -= amount;
    }

    void forward(double amount){
        _right += amount;
        _left += amount;
    }

    void limit(double by){
        _right *= by;
        _left *= by;
    }

    void apply(){
        backLeft -> Set(_left);
        frontLeft -> Set(_left);
        backRight -> Set(_right);
        frontRight -> Set(_right);
        _left = 0;
        _right = 0;
    }
};
