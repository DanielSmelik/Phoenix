#ifndef CLICLI_H
#define CLICLI_H
#include "phoenix_robot.h"

 class clicli {

  public:
   clicli(Phoenix &phoenix) ;
   void begin();   //must be called from  void setup()
   void run();   //must be called from  void loop()

  private:
   Phoenix &robot;
   int number;

 };
#endif 