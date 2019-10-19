#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <boost/thread/thread.hpp>
// ros/ros.h是一个实用的头文件，它引用了ROS系统中大部分常用的头文件
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57


class SmartCarKeyboardTeleopNode {

private:

    double walk_vel_;
    double run_vel_;
    double yaw_rate_;
    double yaw_rate_run_;


    geometry_msgs::Twist cmdvel_;
    //为这个进程的节点创建一个句柄,第一个创建的NodeHandle会为节点进行初始化,
    //最后一个销毁的 NodeHandle 则会释放该节点所占用的所有资源
    ros::NodeHandle n_;

    //初始化发布节点
    ros::Publisher pub_;


public:

    SmartCarKeyboardTeleopNode() {
        //告诉master我们将要在cmd_vel(话题名)上发布geometry_msgs/Twist消息类型的消息
        //这样master就会告诉所有订阅了cmd_vel话题的节点,将要有数据发布．
        //第二个参数是发布序列的大小,如果我们发布消息的频率太高，
        //缓冲区中的消息在大于1000个的时候就会开始丢弃先前发布的消息
        //NodeHandle::advertise() 返回一个 ros::Publisher 对象,它有两个作用：
        //1) 它有一个 publish() 成员函数可以让你在topic上发布消息；
        //2) 如果消息类型不对,它会拒绝发布。

        pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);


        ros::NodeHandle n_private("~");

        n_private.param("walk_vel", walk_vel_, 0.5);
        n_private.param("run_vel", run_vel_, 1.0);
        n_private.param("yaw_rate", yaw_rate_, 1.0);
        n_private.param("yaw_rate_run", yaw_rate_run_, 1.5);

    }


    ~SmartCarKeyboardTeleopNode() {}

    void keyboardLoop();


    void stopRobot() {

        cmdvel_.linear.x = 0.0;

        cmdvel_.angular.z = 0.0;

        pub_.publish(cmdvel_);

    }

};


SmartCarKeyboardTeleopNode *tbk;

int kfd = 0;

struct termios cooked, raw;

bool done;


int main(int argc, char **argv) {
    //初始化ROS.它允许允许ROS通过命令进行名称重映射．
    //我们可以指定节点的名称，但是这里的名称必须是basename,名称内不能包含/等符号
    ros::init(argc, argv, "tbk", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);

    SmartCarKeyboardTeleopNode tbk;

    //重新定义一个线程
    boost::thread t = boost::thread(boost::bind(&SmartCarKeyboardTeleopNode::keyboardLoop, &tbk));


    //ros::spin()在调用后不会再返回，也就是你的主程序到这儿就不往下执行了，
    //而ros::spinOnce()后者在调用后还可以继续执行之后的程序。
    ros::spin();

    //中断线程
    t.interrupt();
    //由于线程中断，所以立即返回
    t.join();
    //速度置零
    tbk.stopRobot();

    tcsetattr(kfd, TCSANOW, &cooked);


    return (0);

}


void SmartCarKeyboardTeleopNode::keyboardLoop() {

    char c;
    double max_tv = walk_vel_;
    double max_rv = yaw_rate_;
    bool dirty = false;
    int speed = 0;
    int turn = 0;

    // get the console in raw mode  
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("Use WASD keys to control the robot");
    puts("Press Shift to move faster");

    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;

    for (;;) {
        boost::this_thread::interruption_point();

        // get the next event from the keyboard  
        int num;

        if ((num = poll(&ufd, 1, 250)) < 0) {
            perror("poll():");
            return;
        } else if (num > 0) {
            if (read(kfd, &c, 1) < 0) {
                perror("read():");
                return;
            }
        } else {
            if (dirty == true) {
                stopRobot();
                dirty = false;
            }
            continue;
        }

        switch (c) {
            case KEYCODE_W:
                max_tv = walk_vel_;
                speed = 1;
                turn = 0;
                dirty = true;
                break;
            case KEYCODE_S:
                max_tv = walk_vel_;
                speed = -1;
                turn = 0;
                dirty = true;
                break;
            case KEYCODE_A:
                max_rv = yaw_rate_;
                speed = 0;
                turn = 1;
                dirty = true;
                break;
            case KEYCODE_D:
                max_rv = yaw_rate_;
                speed = 0;
                turn = -1;
                dirty = true;
                break;
            case KEYCODE_W_CAP:
                max_tv = run_vel_;
                speed = 1;
                turn = 0;
                dirty = true;
                break;
            case KEYCODE_S_CAP:
                max_tv = run_vel_;
                speed = -1;
                turn = 0;
                dirty = true;
                break;
            case KEYCODE_A_CAP:
                max_rv = yaw_rate_run_;
                speed = 0;
                turn = 1;
                dirty = true;
                break;
            case KEYCODE_D_CAP:
                max_rv = yaw_rate_run_;
                speed = 0;
                turn = -1;
                dirty = true;
                break;
            default:
                max_rv = yaw_rate_;
                speed = 0;
                turn = 0;
                dirty = false;
        }
        cmdvel_.linear.x = speed * max_tv;
        cmdvel_.angular.z = turn * max_rv;
        pub_.publish(cmdvel_);
    }
}