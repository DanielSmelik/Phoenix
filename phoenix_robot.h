#ifndef PHOENIX_ROBOT_H
#define PHOENIX_ROBOT_H

class Phoenix{
  private:
    const int button_pin;

  public:
    Phoenix(int _test_arg);  
    void begin(); 
    void motgo(int speedl, int speedr);
    void motbrake();
    void run();
    void readcli();
    //void startFlameSensors();
    //void readFlameSensors(int values[]); 
    //bool detectFlame(int values[]); 
    void blinkRing();
    int check_us();
    char get_dir();
    float get_anglez();
    void steer(int ang, float kp, float ki, float kd, int defspeed);
    void gyroturn(int sp, int times, const float kp, const float ki, const float kd);
    
};
#endif 
