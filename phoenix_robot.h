#ifndef PHOENIX_ROBOT_H
#define PHOENIX_ROBOT_H

class Phoenix{
  private:
    int  _test_param;

  public:
    Phoenix(int test_param);
    void begin();
    void motgo(int speed);
    void run();
    void test(int i); //was used to test serial. irrelevant rn. 
    void readcli();

};
#endif 
