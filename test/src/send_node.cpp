#include"test/send_node.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "can_send");
    ros::NodeHandle nh("~");
    delphi_can del_can(nh);
    while(ros::ok())
    {
        del_can.send_canmsg();
    }
    ros::spin();
    return 0;
}
