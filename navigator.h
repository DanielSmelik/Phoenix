#ifndef NAVIGATOR_H
#define NAVIGATOR_H

class navigator{
    public:
    navigator(int trig_1, int echo_1, int trig_2, int echo_2, int trig_3, int echo_3, int baz_pin);
    void begin();
    float gyro_value();
    float ultra_check(int trigPin, int echoPin);
    void run();
    void gyroturn(int sp, int times);
    void reset_gyro();
    void steer(int ang);
    int check_us();
    void go_to_dir();
    private:
    int _trig_1, _trig_2, _trig_3;
    int _echo_1, _echo_2, _echo_3;
    int _baz_pin;
    int us_front;
    int us_left;
    int us_right;

};
#endif 
