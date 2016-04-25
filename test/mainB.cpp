#include <drone_process.h>
#include <std_msgs/Empty.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mainB");
  ros::NodeHandle n;
  std::string t = "top";
  ros::Publisher pub = n.advertise<std_msgs::Empty>(t, 10);
  ros::ServiceClient serv = n.serviceClient<std_srvs::Empty>("reader");
  std::cout << serv.isPersistent() << std::endl;
  std::cout << true << std::endl;
  ros::Rate loop_rate(10);
  while(ros::ok())
  {
    std::cout << "pub" << std::endl;
    std_msgs::Empty msg;
    pub.publish(msg);
    if(serv.waitForExistence(ros::Duration(2)))
    {
      std::cout << "serv" << std::endl;
      std_srvs::Empty srv;
      serv.call(srv);
      std::cout << "ends" << std::endl;
    }
    loop_rate.sleep();
  }
  return 0;
}