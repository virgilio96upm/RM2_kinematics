# RM2_Kinematics

Here, you will find the RM2 leg and soft robotic arm kinematics. They are written in Matlab 

## Soft_robot_arm kinematics
The forward kinematics are solved in an analytical fashion and the inverse kinematics through neural networks. The parameters of the neural networks are provided both in .mat and excel files.
The inverse kinematics needs to be tuned to compensate the error based on the prediction of the network.

## Leg_ Kinematics
The forward and inverse kinematics are solved in an analytical fashion. Both needs validation of singular positions and update the workspace of the leg based on the range of movement of the real robot and the translation from model results to the robot.
