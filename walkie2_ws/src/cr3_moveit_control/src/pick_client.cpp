#include "ros/ros.h"
#include "cr3_moveit_control/cr3_pick.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pick_client_cpp");
  ROS_INFO("INPUT");
  if (argc != 7)
    {
      ROS_INFO("usage: add_six_ints_client X Y Z roll pitch yawn");
    }
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<cr3_moveit_control::cr3_pick>("pick_success");
  cr3_moveit_control::cr3_pick srv;
  double posx, posy, posz, orix, oriy, oriz, oriw;
  // std::cin >> posx;
  // std::cin >> posy;
  // std::cin >> posz;
  // std::cin >> orix;
  // std::cin >> oriy;
  // std::cin >> oriz;
  // std::cin >> oriw;
  posx = -0.3; posy = 0.0; posz = 0.4; orix =0; oriy = 0; oriz = 0; oriw =1;
  srv.request.geo_req.position.x = posx;
  srv.request.geo_req.position.y = posy;
  srv.request.geo_req.position.z = posz;
  srv.request.geo_req.orientation.x = orix;
  srv.request.geo_req.orientation.y = oriy;
  srv.request.geo_req.orientation.z = oriz;
  srv.request.geo_req.orientation.w = oriw;
  if (client.call(srv))
    {
      ROS_INFO("VALUE: %B", (bool)srv.response.success_grasp);
    }
  else
    {
      ROS_ERROR("ERROR");
      return 1;
    }
  return 0;

}
