/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_UP    0x41
#define KEYCODE_DOWN  0x42
#define KEYCODE_LEFT  0x44
#define KEYCODE_RIGHT 0x43
#define KEYCODE_SPACE 0x20

int mode = 1; // pulsed mode or continuous mode

/**
 * Class to send force commands from arrow key input
 */
class teleForceCmd
{
public:
    teleForceCmd();
    void keyLoop();
    void pubForce(double x, double y, double z);
private:
    double Fx, Fy, Fz;
    ros::NodeHandle n;
    ros::Publisher force_pub;
    geometry_msgs::Wrench Force;
};

/**
 * Default constructor to initialize the force values and the publisher
 */
teleForceCmd::teleForceCmd()
{
    Fx = 0;
    Fy = 0;
    Fz = 0;
    force_pub = n.advertise<geometry_msgs::Wrench>("/apply_force/trunk", 20);   // Initialize the publisher
    sleep(1);
    pubForce(Fx, Fy, Fz);                                                       // Publish the initialized force values
}

int kfd = 0; // TODO make const
struct termios cooked, raw;

/**
 * Signal handler to shutdown the terminal and the ROS node properly
 * @param sig
 */
void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);   // Unclear; probably quitting the terminal input
    ros::shutdown();                    // Stopping the ROS node
    exit(0);                            // Exiting the program
}

/**
 * Main function to start the tele operation with the arrow keys
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "external_force");    // Initialize ROS-node with name external_force
    teleForceCmd remote;                        // Object of teleForceCmd to control the direction via arrow keys
    signal(SIGINT,quit);                        // Set event handler for SIGINT to the quit() function
    remote.keyLoop();                           // Run the keyboard input loop of the remote object
    return(0);
}

/**
 * Method to publish a wrench message with the three given force values
 * @param x Force in x-Direction
 * @param y Force in y-Direction
 * @param z Force in z-Direction
 */
void teleForceCmd::pubForce(double x, double y, double z)
{
    Force.force.x = Fx;         // populate message values
    Force.force.y = Fy;
    Force.force.z = Fz;
    force_pub.publish(Force);   // publish message
    ros::spinOnce();
}

/**
 * Method to read the arrow keys and send corresponding force values as wrench messages
 */
void teleForceCmd::keyLoop()
{
    char c;             // Input character
    bool dirty=false;   // Flag to determine if the current state was published or not
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use 'Space' to change mode, default is Pulsed mode:");
    puts("Use 'Up/Down/Left/Right' to change direction");
    for(;;){
        // get the next event from the keyboard
        if(read(kfd, &c, 1) < 0){
            perror("read():");
            exit(-1);
        }
        ROS_DEBUG("value: 0x%02X\n", c);
        switch(c){
        case KEYCODE_UP:
            if(mode > 0) {
                Fx = 60;
            } else {
                Fx += 16;
                if(Fx > 220) Fx = 220;
                if(Fx < -220) Fx = -220;
            }
            ROS_INFO("Fx:%3d   Fy:%3d   Fz:%3d", (int)Fx, (int)Fy, (int)Fz);
            dirty = true;
            break;
        case KEYCODE_DOWN:
            if(mode > 0) {
                Fx = -60;
            } else {
                Fx -= 16;
                if(Fx > 220) Fx = 220;
                if(Fx < -220) Fx = -220;
            }
            ROS_INFO("Fx:%3d   Fy:%3d   Fz:%3d", (int)Fx, (int)Fy, (int)Fz);
            dirty = true;
            break;
        case KEYCODE_LEFT:
            if(mode > 0) {
                Fy = 30;
            } else {
                Fy += 8;
                if(Fy > 220) Fy = 220;
                if(Fy < -220) Fy = -220;
            }
            ROS_INFO("Fx:%3d   Fy:%3d   Fz:%3d", (int)Fx, (int)Fy, (int)Fz);
            dirty = true;
            break;
        case KEYCODE_RIGHT:
            if(mode > 0) {
                Fy = -30;
            } else {
                Fy -= 8;
                if(Fy > 220) Fy = 220;
                if(Fy < -220) Fy = -220;
            }
            ROS_INFO("Fx:%3d   Fy:%3d   Fz:%3d", (int)Fx, (int)Fy, (int)Fz);
            dirty = true;
            break;
        case KEYCODE_SPACE:
            mode = mode*(-1);
            if(mode > 0){
                ROS_INFO("Change to Pulsed mode.");
            } else {
                ROS_INFO("Change to Continuous mode.");
            }
            Fx = 0;
            Fy = 0;
            Fz = 0;            
            ROS_INFO("Fx:%3d   Fy:%3d   Fz:%3d", (int)Fx, (int)Fy, (int)Fz);
            dirty = true;
            break;
        }
        if(dirty == true){                  // TODO remove comparasion
            pubForce(Fx, Fy, Fz);           // publish the given forces
            if(mode > 0){                   // check for pulsed mode
                usleep(100000); // 100 ms   // TODO make time a defined constant
                Fx = 0;                     // reset the force values
                Fy = 0;
                Fz = 0;
                pubForce(Fx, Fy, Fz);       // publish the default forces
            }
            dirty=false;                    // set flag to indicate the changes where published
        }
    }
    return;
}
