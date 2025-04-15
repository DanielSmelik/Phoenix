#ifndef PHOENIX_ROBOT_H
#define PHOENIX_ROBOT_H

class Phoenix{
  private:
    int test_arg;

  public:
    Phoenix(int _test_arg);  
    void begin(); 
    void motgo(int speedl, int speedr);
    void run();
    void readcli();
    void startFlameSensors();
    void readFlameSensors(int values[]); 
    bool detectFlame(int values[]); 
};
#endif 
