#ifndef NAVIGATOR_H
#define NAVIGATOR_H

class navigator{
    public:
  navigator(int test_param);
    void begin();
    float gyro_value();
    float ultra_check(int trigPin, int echoPin);
    void run();
    void gyroturn(int sp, int times);
    private:


};
#endif 
