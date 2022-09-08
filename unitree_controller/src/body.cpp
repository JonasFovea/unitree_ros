/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "body.h"

namespace unitree_model {

ros::Publisher servo_pub[12];
unitree_legged_msgs::LowCmd lowCmd; //LowCmd message provided by the unitree_ros_to_real package
unitree_legged_msgs::LowState lowState; //LowState message provided by the unitree_ros_to_real package


/**
 * Function to initialize the MotorCmd messages in the LowCmd message.
 * Three consecutive motor commands correspond to the three joints of a leg.
 */
// These parameters are only for reference.
// Actual patameters need to be debugged if you want to run on real robot.
void paramInit()
{
    // The LowCmd contains an array of 20 MotorCmd messages
    // Loop over 4 groups of 3 MotorCmd messages
    for(int i=0; i<4; i++){
        // CMD 0
        lowCmd.motorCmd[i*3+0].mode = 0x0A; // motor target mode
        lowCmd.motorCmd[i*3+0].Kp = 70;     // motor spring stiffness coefficient
        lowCmd.motorCmd[i*3+0].dq = 0;      // motor target velocity
        lowCmd.motorCmd[i*3+0].Kd = 3;      // motor damper coefficient
        lowCmd.motorCmd[i*3+0].tau = 0;     // motor target torque
        // CMD 1
        lowCmd.motorCmd[i*3+1].mode = 0x0A;
        lowCmd.motorCmd[i*3+1].Kp = 180;
        lowCmd.motorCmd[i*3+1].dq = 0;
        lowCmd.motorCmd[i*3+1].Kd = 8;
        lowCmd.motorCmd[i*3+1].tau = 0;
        // CMD 2
        lowCmd.motorCmd[i*3+2].mode = 0x0A;
        lowCmd.motorCmd[i*3+2].Kp = 300;
        lowCmd.motorCmd[i*3+2].dq = 0;
        lowCmd.motorCmd[i*3+2].Kd = 15;
        lowCmd.motorCmd[i*3+2].tau = 0;
    }
    //Loop over all MotorCmd messages to set the q value to the corresponding q value of the LowState
    for(int i=0; i<12; i++){
        lowCmd.motorCmd[i].q = lowState.motorState[i].q; // motor target position
    }
}

/**
 * Function to move all joints into standing position
 */
void stand()
{   
    double pos[12] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 
                      0.0, 0.67, -1.3, -0.0, 0.67, -1.3}; // TODO make const
    moveAllPosition(pos, 2*1000); // why this duration? TODO maybe extract to defined constant
}

void motion_init()
{
    paramInit();
    stand();
}

void sendServoCmd()
{
    for(int m=0; m<12; m++){
        servo_pub[m].publish(lowCmd.motorCmd[m]);
    }
    ros::spinOnce();
    usleep(1000);
}

/**
 * Function to move all motors to the given target position
 * @param targetPos Array of 12 positional values for the leg joints
 * @param duration Number of steps to split the movement into
 */
void moveAllPosition(double* targetPos, double duration)
{
    double pos[12] ,lastPos[12], percent;
    for(int j=0; j<12; j++) lastPos[j] = lowState.motorState[j].q;                  // Get the current position from the last state
    for(int i=1; i<=duration; i++){                                                 // Loop for duration times
        if(!ros::ok()) break;                                                       // Check if ROS is still running
        percent = (double)i/duration;                                               // Calculate the progress percentage
        for(int j=0; j<12; j++){                                                    // Loop over all joints in the LowCmd message
            lowCmd.motorCmd[j].q = lastPos[j]*(1-percent) + targetPos[j]*percent;   // Set the next target position part
        }
        sendServoCmd();                                                             // Send the command to the servos
    }
}


}
