#include <drone_process.h>
#include <std_msgs/Empty.h>

class claseA : public DroneProcess
{
private:
  ros::NodeHandle n;

  ros::Subscriber top;

public:
  claseA(){}
  ~claseA(){}

protected:

  bool readerCall(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
  {
    std::cout << "Leo" << std::endl;
    return true;
  }

  void topCall(const std_msgs::Empty &msg)
  {
    std::cout << "top" << std::endl;
  }

  void ownOpen()
  {
    ownRun();
  }

  void ownRun(){
    std::string t = "top";

    top = n.subscribe(t, 1, &claseA::topCall,this);

    ros::ServiceServer reader=n.advertiseService("reader",&claseA::readerCall,this);
    ros::Rate r(10);
    std::cout << "1" << std::endl;
    while(true)
    {
      DroneProcess::spin();
    }
    std::cout << "2" << std::endl;
    //while(ros::ok())
    //{
      //ros::spinOnce();
      //std::cout << "bucle2" << std::endl;
      //r.sleep();
    //}
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