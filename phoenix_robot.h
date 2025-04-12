#ifndef PHOENIX_ROBOT_H
#define PHOENIX_ROBOT_H

class Phoenix{
  private:

    int* flameSensorPins;
    const int flameThreshold;
    const int numFlameSensors; 

  public:
    Phoenix(int _flameSensorPins[], const int _numFlameSensors, const int _flameThreshold);    void begin(); 
    void motgo(int speedl, int speedr);
    void run();
    void readcli();
    void startFlameSensors();
    void readFlameSensors(int values[]); 
    bool detectFlame(int values[]); 
};
#endif 
