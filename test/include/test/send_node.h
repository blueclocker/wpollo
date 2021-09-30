#include<canlib.h>
#include<stdio.h>
#include<ros/ros.h>

class delphi_can
{
private:
    canHandle h;
public:
    delphi_can(ros::NodeHandle &n);
    ~delphi_can();
    void send_canmsg();
};
