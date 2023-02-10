#include <string>
#include <string.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <math.h>

#define PI 3.1415926
float val = PI / 180.0;
float joint_angle_1=0.0,joint_angle_2=0.0,joint_angle_3=0.0;

void publish_state(ros::Publisher& pub, float joint_angle_1, float joint_angle_2, float joint_angle_3){
	sensor_msgs::JointState joint_state;
	joint_state.header.stamp = ros::Time::now();
	joint_state.name.resize(3);
	joint_state.position.resize(3);
	joint_state.name[0] = "A_upper_joint";
	joint_state.position[0] = tfRadians(joint_angle_1);
    joint_state.name[1] = "B_upper_joint";
	joint_state.position[1] = tfRadians(joint_angle_2);
	joint_state.name[2] = "C_upper_joint";
	joint_state.position[2] = tfRadians(joint_angle_3);
	pub.publish(joint_state);
	ros::Duration(2).sleep();
}

class IK
{
public:
	float R=355, M=100, u=294, rh=25, k=50/sin(60*val), K=177.5*tan(PI/6.0), A, B;
	float theta_a, theta_b, theta_c;
	float top_center[3] = {0,0,0};

	float position_A[3] = {top_center[0], top_center[1]-K, rh};
	float position_B[3] = {top_center[0]+K*cos(150*val), top_center[1]+K*sin(150*val), top_center[2]+rh};
	float position_C[3] = {top_center[0]+K*cos(30*val), top_center[1]+K*sin(30*val), top_center[2]+rh};

	void inverse_kinematic(float G[3], float *theta_a, float *theta_b, float *theta_c);
};

void IK::inverse_kinematic(float G[3], float *theta_a, float *theta_b, float *theta_c)
{
	float motorposition_a[3] = {G[0], G[1]-k, G[2]+rh};
	float motorposition_b[3] = {G[0]+k*cos(150*val), G[1]+k*sin(150*val), G[2]+rh};
	float motorposition_c[3] = {G[0]+k*cos(30*val), G[1]+k*sin(30*val), G[2]+rh};

	float temp_A[3] = {position_A[0]-motorposition_a[0],position_A[1]-motorposition_a[1],position_A[2]-motorposition_a[2]};
	float temp_B[3] = {position_B[0]-motorposition_b[0],position_B[1]-motorposition_b[1],position_B[2]-motorposition_b[2]};
	float temp_C[3] = {position_C[0]-motorposition_c[0],position_C[1]-motorposition_c[1],position_C[2]-motorposition_c[2]};
	
	*theta_a = asin((-pow(u,2) + (pow(temp_A[0],2) + pow(temp_A[1],2) + pow(temp_A[2],2)) + pow(M,2))/ \
					(2* M* pow((pow(temp_A[1],2) + pow(temp_A[2],2)),0.5))) - atan(temp_A[1]/temp_A[2]);
	
	A = -2 * temp_B[2];
    B = -(pow(3, 0.5)) * temp_B[0] + temp_B[1];
    *theta_b = asin((-pow(u,2) + pow(M,2) + pow(temp_B[0],2) + pow(temp_B[1],2) + pow(temp_B[2],2))/ (M* sqrt(pow(A,2) +pow(B,2)))) - atan(B/A);

    A = -2 * temp_C[2];
    B = (pow(3, 0.5)) * temp_C[0] + temp_C[1];
    *theta_c = asin((-pow(u,2) + pow(M,2) + pow(temp_C[0],2) + pow(temp_C[1],2) + pow(temp_C[2],2))/ (M* sqrt(pow(A,2) +pow(B,2)))) - atan(B/A);

	*theta_a = *theta_a / PI *180 - 50.2161;
	*theta_b = *theta_b / PI *180 - 50.2161;
	*theta_c = *theta_c / PI *180 - 50.2161;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "state_publisher");
	ros::NodeHandle n;
	ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states",1);
	
	tf::TransformBroadcaster broadcaster;
	ros::Rate loop_rate(30);

	const double degree = M_PI/180;

	double tilt = 0, tinc = degree, swivel = 0, angle = 0, height = 0, hinc = 0.005;

	geometry_msgs::TransformStamped odom_trans;
	sensor_msgs::JointState joint_state;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "axis";

	while(ros::ok())
	{
		float goal_pos[3];
		IK ik;
		ros::Duration(2).sleep();
		
		goal_pos[0]=0, goal_pos[1]=90, goal_pos[2]=-350;
		ik.inverse_kinematic(goal_pos, &ik.theta_a, &ik.theta_b, &ik.theta_c);
		std::cout << "a:" << ik.theta_a << " b:" << ik.theta_b << " c:" << ik.theta_c << std::endl;
		ROS_INFO("P1");
		publish_state(joint_pub, ik.theta_a, ik.theta_b, ik.theta_c);

		goal_pos[0]=10, goal_pos[1]=120, goal_pos[2]=-350;
		ik.inverse_kinematic(goal_pos, &ik.theta_a, &ik.theta_b, &ik.theta_c);
		std::cout << "a:" << ik.theta_a << " b:" << ik.theta_b << " c:" << ik.theta_c << std::endl;
		ROS_INFO("P2");
		publish_state(joint_pub, ik.theta_a, ik.theta_b, ik.theta_c);

		goal_pos[0]=0, goal_pos[1]=0, goal_pos[2]=-350;
		ik.inverse_kinematic(goal_pos, &ik.theta_a, &ik.theta_b, &ik.theta_c);
		std::cout << "a:" << ik.theta_a << " b:" << ik.theta_b << " c:" << ik.theta_c << std::endl;
		ROS_INFO("P3");
		publish_state(joint_pub, ik.theta_a, ik.theta_b, ik.theta_c);

		goal_pos[0]=120, goal_pos[1]=150, goal_pos[2]=-350;
		ik.inverse_kinematic(goal_pos, &ik.theta_a, &ik.theta_b, &ik.theta_c);
		std::cout << "a:" << ik.theta_a << " b:" << ik.theta_b << " c:" << ik.theta_c << std::endl;
		ROS_INFO("P4");
		publish_state(joint_pub, ik.theta_a, ik.theta_b, ik.theta_c);

        
		loop_rate.sleep();
        ros::spinOnce();



	}
}
