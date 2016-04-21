#include <drone_process.h>


class claseA : public DroneProcess
{
private:
  ros::NodeHandle n;

public:
  claseA(){}
  ~claseA(){}

protected:
  void ownOpen(){
    ownRun();
  }
  void ownRun(){
    ros::Rate r(10);
    while(ros::ok())
    {
      ros::spinOnce();
      std::cout << "bucle2" << std::endl;
      r.sleep();
    }
  }
  void ownRecover(){}

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "mainA");
  claseA ca;
  ca.open();
  ca.start();
  return 0;
}