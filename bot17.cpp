#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"
#include "bot17.h"
#include <math.h>
#include <vector>
#include <fstream>
#include <string>
#include <cstdlib>

#define PI 3.14159265358979323846 

//Initialize Bot parameters
BOT::BOT(int index, int num, const std::string& pub_topic, const std::string& sub_topic)
{
	
	id = index;
	num_bots = num;
	publisher = n.advertise<geometry_msgs::Twist>(pub_topic, 100);
	sub = n.subscribe<nav_msgs::Odometry>(sub_topic, 100, &BOT::pos_reader, this);
	ros::Rate loop(20.0);
	int i=0;
	state = -1;
	lines_x = (float*)malloc(sizeof(float) * num_bots);
	lines_y = (float*)malloc(sizeof(float) * num_bots);
	lines_angles = (float*)malloc(sizeof(float) * num_bots);
	BOT::init_shape();
	while(ros::ok())
	{
		std::cout<<pub_topic<<std::endl;
		bot_pub_lin_x = 0.0;
		bot_pub_lin_y = 0.0;
		bot_pub_lin_z = 0.0;
		bot_pub_ang_x = 0.0;
		bot_pub_ang_y = 0.0;
		bot_pub_ang_z = 0.0;
		rotating = 0;
		moving = 0;
		pref_vel = 0.1;
		num_lines = 0;
		radii = 0.161;
		vel_x = 0.0;
		vel_y = 0.0;
		ros::spinOnce();
		loop.sleep();
		if(i==5)
		{	
			break;
		}
		i++;
	}
	
}

//Read bot odometry
void BOT::pos_reader(const nav_msgs::Odometry::ConstPtr& msg)
{
	bot = *msg;
	bot_pos_x = bot.pose.pose.position.x;
	bot_pos_y = bot.pose.pose.position.y;
	bot_pos_z = bot.pose.pose.position.z;
	ori.x = bot.pose.pose.orientation.x;
	ori.y = bot.pose.pose.orientation.y;
	ori.z = bot.pose.pose.orientation.z;
	ori.w = bot.pose.pose.orientation.w;
	bot_lin_x = bot.twist.twist.linear.x;
	bot_lin_y = bot.twist.twist.linear.y;
	bot_lin_z = bot.twist.twist.linear.z;
	bot_ang_x = bot.twist.twist.angular.x;
	bot_ang_y = bot.twist.twist.angular.y;
	bot_ang_z = bot.twist.twist.angular.z;
	angles = toEuler(ori);
	vel_x = (bot_lin_x) * cos(angles.z * PI/180.0);
	vel_y = (bot_lin_x) * sin(angles.z * PI/180.0);
	//BOT::display();
}

//Display bot odometry
void BOT::display()
{

	printf("\tPosition %d:\n", id);
	printf("\t\tx: %f\n", bot_pos_x);
	printf("\t\ty: %f\n", bot_pos_y);
	printf("\t\tz: %f\n", bot_pos_z);
	printf("\tOrientation: %d\n", id);
	printf("\t\tx: %f\n", ori.x);
	printf("\t\ty: %f\n", ori.y);
	printf("\t\tz: %f\n", ori.z);
	printf("\t\tz: %f\n", ori.w);
	printf("\tLinear: %d\n", id);
	printf("\t\tx: %f\n", bot_lin_x);
	printf("\t\ty: %f\n", bot_lin_y);
	//printf("\t\tz: %f\n", bot_lin_z);
	printf("\tAngular: %d\n", id);
	//printf("\t\tx: %f\n", bot_ang_x);
	//printf("\t\ty: %f\n", bot_ang_y);
	printf("\t\tz: %f\n", bot_ang_z);
	printf("\tEuler Angles: %d\n", id);
	//printf("\t\tRoll: %f\n", angles.x);
	//printf("\t\tPitch: %f\n", angles.y);
	printf("\t\tYaw: %f\n", angles.z);
	printf("\tVelocity: %d\n", id);
	printf("\t\tx: %f\n", vel_x);
	printf("\t\ty: %f\n", vel_y);
}

geometry_msgs::Vector3 BOT::toEuler(geometry_msgs::Quaternion msg)
{
	tf::Quaternion quat;
	tf::quaternionMsgToTF(msg, quat);
	// the tf::Quaternion has a method to acess roll pitch and yaw
	double roll, pitch, yaw;
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

	// the found angles are written in a geometry_msgs::Vector3
	geometry_msgs::Vector3 rpy;
	rpy.x = roll*180/PI;
	rpy.y = pitch*180/PI;
	rpy.z = yaw*180/PI;
	if(rpy.x < 0.0)
	{
		rpy.x = 360.0 + rpy.x;
	}
	if(rpy.y < 0.0)
	{
		rpy.y = 360.0 + rpy.y;
	}
	if(rpy.z < 0.0)
	{
		rpy.z = 360.0 + rpy.z;
	} 
	return rpy;
}

float BOT::getDist(float source_x, float source_y, float neighbor_x, float neighbor_y)
{
	return sqrt(pow(source_x - neighbor_x, 2) + pow(source_y - neighbor_y, 2));
}

//Get angle between two points in range 0:360
float BOT::getAngle(float source_x, float source_y, float neighbor_x, float neighbor_y)
{
	if((source_x - neighbor_x)==0.0 && (source_y - neighbor_y)!=0.0)
	{
		if((neighbor_y - source_y) > 0.0)
		{
			return 90.0;
		}
		else
		{
			return 270.0;
		}
	}
	float angle = atan((neighbor_y - source_y)/(neighbor_x - source_x)) * 180.0/PI;
	if(neighbor_x < source_x)
	{
		angle = 180.0 + angle;
	}
	else if((neighbor_y < source_y) & (neighbor_x > source_x))
	{
		angle = 360.0 + angle;
	}
	return angle;
}

//Find all visible robots within a given range of a robot
void BOT::update_bot_info()
{

	int i;
	//dist.clear();
	//angle.clear();
	std::vector<float> dist_vec;
	std::vector<float> angle_vec; 
	std::vector<int> visibles;
	for(i=0; i < num_bots; i++)
	{
		if(i == id)
		{
				continue;
		}
		
		float rel_dist = BOT::getDist(bot_pos_x, bot_pos_y, bots[i].bot_pos_x, bots[i].bot_pos_y);
		float rel_ang = BOT::getAngle(bot_pos_x, bot_pos_y, bots[i].bot_pos_x, bots[i].bot_pos_y);
		int j;
		int visible = 1;
		for(j = 0; j < num_bots; j++)
		{
			if(j != i && j != id)
			{
				float ang_diff = (asin((radii)/BOT::getDist(bot_pos_x, bot_pos_y, bots[j].bot_pos_x, bots[j].bot_pos_y)) * 180/PI);
				float vo_max = ang_diff + BOT::getAngle(bot_pos_x, bot_pos_y, bots[j].bot_pos_x, bots[j].bot_pos_y);
				float vo_min = BOT::getAngle(bot_pos_x, bot_pos_y, bots[j].bot_pos_x, bots[j].bot_pos_y) - ang_diff;
				if(((rel_ang >= vo_min && rel_ang <= vo_max) || (rel_ang >= (vo_min + 360.0) && rel_ang <= (vo_max + 360.0)) || ((rel_ang + 360.0) >= vo_min && (rel_ang + 360.0) <= vo_max)) && rel_dist > BOT::getDist(bot_pos_x, bot_pos_y, bots[j].bot_pos_x, bots[j].bot_pos_y))
				{
					visible = 0;
					break;
				}
			}
		}		
		if(visible == 1)
		{
			//printf("%d IS VISIBLE TO %d DIST %f\n", i, id, rel_dist);
			dist_vec.push_back(rel_dist);
			angle_vec.push_back(rel_ang);
			visibles.push_back(i);
			if(rel_dist <= 20.0 && state != -1 && bots[i].state != 1 && bots[i].state != 2 && bots[i].state != 3 && bots[i].state != 4)
			{
				bots[i].state = 0;
			}		
		}		
	}
	dist = dist_vec;
	angle = angle_vec;	
	visible_bots = visibles;
}

//Rotate bot by given angle, requiring minimum rotation 
void BOT::rotate( float deg = 45.0)
{
	float target_rotate = angles.z;
	float ang_vel;
	if(deg > 0)
	{
		ang_vel = 0.2;
	}
	else if(deg < 0)
	{
		ang_vel = -0.2;
	}

	if((angles.z >= 0.0) && (angles.z <= 360))
	{
		ros::Rate rate(20.0);
		while((fabs(angles.z - target_rotate) < abs(deg)))
		{
			BOT::update_bot_info();
			bot_pub_ang_x = 0.0;
			bot_pub_ang_y = 0.0;
			bot_pub_ang_z = ang_vel;
			bot_pub_lin_x = 0.0;
			bot_pub_lin_y = 0.0;
			bot_pub_lin_z = 0.0;
			BOT::pub();
			ros::spinOnce();
			rate.sleep();
		}
		
		bot_pub_ang_x = 0.0;
		bot_pub_ang_y = 0.0;		
		bot_pub_ang_z = 0.0;
		bot_pub_lin_x = 0.0;
		bot_pub_lin_y = 0.0;
		bot_pub_lin_z = 0.0;
		BOT::pub(); 
						
	}
}

//Find new velocity of bot using clear path algorithm given HRVO collision cones
float BOT::clear_path()
{
	int i;
	std::vector<float> intersections_x;
	std::vector<float> intersections_y;
	float vel_a_x = bot_pos_x + pref_vel_x;
	float vel_a_y = bot_pos_y + pref_vel_y;
	int flag = 1;
	//Find projections of current velocity vector on collision cones
	for(i = 0; i < num_lines; i++)
	{
		float m1 = tan(lines_angles[i] * PI/180.0);
		float m = -1.0 / m1;
		float x = ((m * vel_a_x) - (m1 * lines_x[i]) - vel_a_y + lines_y[i]) / (m - m1);
		float y = (m * (x - vel_a_x)) + vel_a_y;
		if(i % 2 == 0)
		{
			float ang = BOT::getAngle(lines_x[i], lines_y[i], vel_a_x, vel_a_y);
			if(ang < lines_angles[i] && ang > lines_angles[i+1])
			{
				flag = 0;
			}
		}

		intersections_x.push_back(x); 
		intersections_y.push_back(y);
	}
	//Check if velocity vector inside a collision cone
	if(flag == 1)
	{
		
		target_vel_x = vel_a_x;
		target_vel_y = vel_a_y;
		return sqrt(pow(target_vel_x - bot_pos_x, 2) + pow(target_vel_y - bot_pos_y, 2));
	}
	
	printf("Number of lines= %d\n", num_lines);
	std::cout <<"Number of black marks= " <<intersections_x.size() <<std::endl;
	//Find intersection points of collision cones
	for(i = 0; i < (num_lines - 1); i++)
	{
		int j;
		float m = tan(lines_angles[i] * PI/180.0);
		for(j = i+1; j < num_lines; j++)
		{
			float m1 = tan(lines_angles[j] * PI/180.0);
			float x = ((m * lines_x[i]) - (m1 * lines_x[j]) - lines_y[i] + lines_y[j]) / (m - m1);
			float y = (m * (x - lines_x[i])) + lines_y[i];
			
			intersections_x.push_back(x);
			intersections_y.push_back(y);
		}
	}
	std::cout <<"Total Number of marks= " <<intersections_x.size() <<"\n";
	float new_vel_x = 100000.0;
	float new_vel_y = 100000.0;
	float min_dist = BOT::getDist(vel_a_x, vel_a_y, new_vel_x, new_vel_y);
	for(i = 0; i < intersections_x.size(); i++)
	{
		//printf("Intersection Point X= %f Y= %f\n", intersections_x[i], intersections_y[i]);
		if( min_dist < BOT::getDist(vel_a_x, vel_a_y, intersections_x[i], intersections_y[i]))
		{
			continue;
		}
		else
		{
			int j;
			int outside = 1;
			for(j = 0; j < (num_lines - 1); j=j+2)
			{
				if(intersections_x[i] == lines_x[j] && intersections_y[i] == lines_y[j])
				{
					continue;
				}
				float angle_wrt_apex = BOT::getAngle(lines_x[j], lines_y[j], intersections_x[i], intersections_y[i]);
				/*if(angle_wrt_apex > 270.0)
				{
					angle_wrt_apex -= 360.0;
				}*/
				float limit_max = lines_angles[j];
				float limit_min = lines_angles[j+1];
				if(limit_min < 0.0)
				{
					limit_min += 360.0;
					limit_max += 360.0;
				}
				if(limit_max < limit_min)
				{
					limit_max += 360.0;
				}
				printf("Angle wrt Apex %f %f= %f\n", limit_max, limit_min, angle_wrt_apex);
				if((((limit_max - angle_wrt_apex) > 0.01) && ((angle_wrt_apex - limit_min) >  0.01)) || (((limit_max - angle_wrt_apex - 360.0) > 0.01) && ((angle_wrt_apex + 360.0 - limit_min) >  0.01)))
				{
					printf("Point outside\n");
					outside = 0;
					break;
				}
			}
			if(outside == 1)
			{
				printf("Point inside\n");
				new_vel_x = intersections_x[i];
				new_vel_y = intersections_y[i];
				//printf("Points outside %f %f \n", new_vel_x, new_vel_y);
				min_dist = BOT::getDist(vel_a_x, vel_a_y, intersections_x[i], intersections_y[i]);
			}
		}
	}
	
	if(new_vel_x == 100000.0)
	{
		printf("NO AVAILABLE POINTS!!\n");
		return 0.0;
	}
	else
	{
		target_vel_x = new_vel_x;
		target_vel_y = new_vel_y;
		printf("Target velocity X = %f  Y = %f\n", target_vel_x, target_vel_y);
		return sqrt(pow(new_vel_x - bot_pos_x, 2) + pow(new_vel_y - bot_pos_y, 2));
	}
}


//Form collision cone induced between two robots
float BOT::HRVO_2(int i)
{
	float margin = 0.03;
	float vo_radii = bots[i].radii + radii + margin;
	float vel_a_x = bot_pos_x + vel_x;
	float vel_a_y = bot_pos_y + vel_y;
	float rvo_x = (bots[i].bot_pos_x + bots[i].vel_x + bots[i].bot_pos_x + vel_x) / 2.0;
	float rvo_y = (bots[i].bot_pos_y + bots[i].vel_y + bots[i].bot_pos_y + vel_y) / 2.0;
	float vo_x = bots[i].bot_pos_x + bots[i].vel_x;
	float vo_y = bots[i].bot_pos_y + bots[i].vel_y;
	//Apex of RVO cone
	float apex_rvo_x = (bot_pos_x + bots[i].vel_x + vel_a_x) / 2.0;
	float apex_rvo_y = (bot_pos_y + bots[i].vel_y + vel_a_y) / 2.0;
	//Apex of VO cone	
	float apex_vo_x = bot_pos_x + bots[i].vel_x;
	float apex_vo_y = bot_pos_y + bots[i].vel_y;
	float vel_ang_wrt_b = BOT::getAngle(apex_rvo_x, apex_rvo_y, vel_a_x, vel_a_y);
	float vel_ang_wrt_a = BOT::getAngle(bot_pos_x, bot_pos_y, vel_a_x, vel_a_y);
	float vo_cent_ang = BOT::getAngle(apex_rvo_x, apex_rvo_y, rvo_x, rvo_y);
	float vo_cent_dist = BOT::getDist(apex_rvo_x, apex_rvo_y, rvo_x, rvo_y);
	printf("Dist to Cent %f %f \n", vo_cent_dist, vo_radii + 0.02);
	if(vo_radii + 0.02 > vo_cent_dist)
	{
		if((bots[i].state == 10 || bots[i].state == 2 || bots[i].state == 3) && (shape_num == bots[i].shape_num))
		//if(bots[i].state == 0 && BOT::getDist(bot_pos_x, bot_pos_y, target_pose_x ,target_pose_y) <= 0.5)
		{
			printf("TOO CLOSE TOO CLOSE TOO CLOSE TOO CLOSE TOO CLOSE TOO CLOSE TOO CLOSE TOO CLOSE\n");
			return 0.0;
		}
		if(bots[i].state >= 1 && (shape_num == bots[i].shape_num))
		{
			int iter;
			int closest_stationary_bot = -1;
			float dist_to_bot = 1000000.0;
			for(iter = 0; iter < dist.size(); iter++)
			{
				if(dist[iter] < 5 * radii && bots[visible_bots[iter]].state >= 2)
				{
					if(dist[iter] < dist_to_bot)
					{
						dist_to_bot = dist[iter];
						closest_stationary_bot = visible_bots[iter];
					}
				}
			}
			if(closest_stationary_bot != -1)
			{
				target_pose_x = bots[closest_stationary_bot].bot_pos_x;
				target_pose_y = bots[closest_stationary_bot].bot_pos_y;
			}
		}
	}
	//Angle of collision cone
	float vo_max_ang = (asin(vo_radii/std::max(vo_cent_dist, vo_radii)) * 180/PI) + vo_cent_ang + 10.0;
	float vo_min_ang = vo_cent_ang - (asin(vo_radii/std::max(vo_cent_dist, vo_radii)) * 180/PI) - 10.0;
	float apex_x;
	float apex_y;
	//Apex of HRVO cone
	if( vel_ang_wrt_b < vo_cent_ang)
	{
		printf("RIGHT SIDE\n");
		float m = tan((vo_min_ang) * PI/180.0);
		float m1 = tan((vo_max_ang) * PI/180.0);
		apex_x = ((m * apex_rvo_x) - (m1 * apex_vo_x) - apex_rvo_y + apex_vo_y) / (m - m1);
		apex_y = (m * (apex_x - apex_rvo_x)) + apex_rvo_y;
	}
	else
	{
		printf("LEFT SIDE\n");
		float m = tan((vo_max_ang) * PI/180.0);
		float m1 = tan((vo_min_ang) * PI/180.0);
		apex_x = ((m * apex_rvo_x) - (m1 * apex_vo_x) - apex_rvo_y + apex_vo_y) / (m - m1);
		apex_y = (m * (apex_x - apex_rvo_x)) + apex_rvo_y;
	}
	
	printf("Obstacle Velocity %d X = %f Y = %f Vel = %f\n", i, bots[i].vel_x, bots[i].vel_y, sqrt(pow(bots[i].vel_x, 2) + pow(bots[i].vel_y, 2)));
	printf("VO_max_ang btw %d & %d= %f\n", id, i, vo_max_ang);
	printf("VO_min_ang btw %d & %d= %f\n", id, i, vo_min_ang);
	printf("Vel angle wrt b= %f\n", vel_ang_wrt_b);
	printf("Apex X= %f\n", apex_x);
	printf("Apex Y= %f\n", apex_y);

	lines_x[num_lines] = apex_x;
	lines_y[num_lines] = apex_y;
	lines_angles[num_lines] = vo_max_ang;//+5.0;
	num_lines++;
	lines_x[num_lines] = apex_x;
	lines_y[num_lines] = apex_y;
	lines_angles[num_lines] = vo_min_ang;//-5.0;
	num_lines++;

	if(((vel_ang_wrt_b > vo_max_ang) || (vel_ang_wrt_b < vo_min_ang)))
	{
		return 100.0;
	}
	else
	{
		return 1.0;
	}
}

//Find obstacles using HRVO and find new velocity using clear path
float BOT::obstacle()
{
	int i;
	float new_vel = 100.0;

	for(i=0; i<dist.size(); i++)
	{
		printf("No_seg %d %d\n", dist.size(), i);
		if(dist[i] <= 0.5)
		{
			
			float obs = BOT::HRVO_2(visible_bots[i]);
			printf("VO %d: %f\n", id, obs);	
			
			if(obs != 100.0)
			{
				if(obs == 0.0)
				{
					return 0.0;
				}
				new_vel = obs;
				
			}
		}  
	}
	float dist_2_dest = BOT::getDist(bot_pos_x,  bot_pos_y, target_pose_x, target_pose_y);
	if(dist_2_dest == 0.0)
	{
		pref_vel_x = 0.0;
		pref_vel_y = 0.0;
	}
	else
	{
		pref_vel_x = -pref_vel * (bot_pos_x - target_pose_x) / dist_2_dest;
		pref_vel_y = -pref_vel * (bot_pos_y - target_pose_y) / dist_2_dest;
	}
	printf("Preferred Velocity X= %f Y= %f\n", pref_vel_x, pref_vel_y);
	printf("Preferred Angle= %f\n", BOT::getAngle(bot_pos_x, bot_pos_y, target_pose_x, target_pose_y));
	
	float result = clear_path();
	num_lines = 0;
	printf("NEW VELOCITY %d = %f\n", id, result);
	return result;

}

//Modulate bot velocity until it reaches the target position
void BOT::motion_1D( geometry_msgs::PoseStamped target_pose)
{
	float lin_vel = pref_vel;
	float ang_vel = 0.0;
	/*std::stringstream ss;
	ss << "log/" << id << "_12.csv";
	std::ofstream myfile;
      	myfile.open(ss.str().c_str());
	myfile << "Current_X,Current_Y,Cur_angle,Vel_x,Vel_y,Cur_vel,Target_X,Target_Y,Target_angle,Target_vel_x,Target_vel_y,Target_vel,Angular_speed,Pref_vel_x,Pref_vel_y,Pref_vel,Time\n";*/
	//target_pose_x = 0.0;
	//target_pose_y = 0.0;
	float target_ang = BOT::getAngle(bot_pos_x, bot_pos_y, target_pose_x, target_pose_y);
	target_vel_x = bot_pos_x + (pref_vel * cos(target_ang * PI/180.0));
	target_vel_y = bot_pos_y + (pref_vel * sin(target_ang * PI/180.0));
	printf("%f %f ", fabs(target_pose.pose.position.x - bot_pos_x), fabs(target_pose.pose.position.y - bot_pos_y));
	ros::Rate rate(20.0);
	float prev_error = 0.0;
	int at_target = 0;
	int inside_shape = BOT::shape_checker(bot_pos_x, bot_pos_y);
	while(((fabs(target_pose_x - bot_pos_x) > radii) || (fabs(target_pose_y - bot_pos_y) > radii)) || (inside_shape == 0))
	{
		printf("Updating\n");	
		BOT::update_bot_info();
		float obs = BOT::obstacle();
		target_ang = BOT::getAngle(bot_pos_x, bot_pos_y, target_vel_x, target_vel_y);
		
		printf("Current Angle= %f\n", angles.z);
		printf("Target Angle= %f\n", target_ang);
		printf("Target Velocity X= %f Y= %f\n", target_vel_x, target_vel_y);
		printf("Target Position X= %f Y= %f\n", target_pose_x, target_pose_y);
		if(target_ang - angles.z >= 180.0)
		{
			target_ang = target_ang - 360.0;
		}
		else if(angles.z - target_ang >= 180.0)
		{
			target_ang = 360.0 + target_ang;	
		}
		
		if((angles.z <= (target_ang + 2.0)) && (angles.z >= (target_ang - 2.0)))
		{
			ang_vel = 0.0;
		}
		else
		{
			ang_vel = (0.07*(angles.z - target_ang) + 0.05*((angles.z - target_ang)-prev_error)/0.1);
			if(ang_vel <= -1.0)
			{
				ang_vel = -1.0;
			}
			else if(ang_vel >= 1.0)
			{
				ang_vel = 1.0;
			}
			prev_error = (angles.z - target_ang);
				
			printf("CHANGING ANGLE %f\n", ang_vel);
		}
		
		printf("****DONE CHECKING OBSTACLES FOR %d ******\n", id);
		if(obs != 100.0)
		{
			if(obs == 0.0)
			{
				at_target = 1;
				break;
			}
			lin_vel = obs;
		}
		//myfile << bot_pos_x << "," << bot_pos_y << "," << angles.z << "," << vel_x << "," << vel_y << "," << bot_lin_x << "," << target_pose_x << "," << target_pose_y << "," << target_ang << "," << target_vel_x << "," << target_vel_y << "," << lin_vel << "," << ang_vel << "," << pref_vel_x << "," << pref_vel_y << "," << pref_vel << "," << ros::Time::now().toNSec() << "\n";

		bot_pub_lin_x = lin_vel / 2.0;
		bot_pub_lin_y = 0.0;
		bot_pub_lin_z = 0.0;
		bot_pub_ang_x = 0.0;
		bot_pub_ang_y = 0.0;
		bot_pub_ang_z = ang_vel;
		printf("\tPosition: %d\n", id);
		printf("\t\tx: %f\n", bot_pos_x);
		printf("\t\ty: %f\n", bot_pos_y);
		printf("\t\tz: %f\n", bot_pos_z);
		printf("\tPublish Linear: %d\n", id);
		printf("\t\tx: %f\n", bot_pub_lin_x);
		printf("\t\ty: %f\n", bot_pub_lin_y);
		printf("\t\tz: %f\n", bot_pub_lin_z);
		BOT::pub();
		ros::spinOnce();
		rate.sleep();
		inside_shape = BOT::shape_checker(bot_pos_x, bot_pos_y);
		//printf("Motion\n");
	}
	state = 1;
	if(at_target == 0)
	{
		state = 2;
	}
	bot_pub_ang_x = 0.0;
	bot_pub_ang_y = 0.0;		
	bot_pub_ang_z = 0.0;
	bot_pub_lin_x = 0.0;
	bot_pub_lin_y = 0.0;
	bot_pub_lin_z = 0.0;
	printf("Finished motion %d\n", state); 
	printf("\tPublish Linear: %d\n", id);
	printf("\t\tx: %f\n", bot_pub_lin_x);
	printf("\t\ty: %f\n", bot_pub_lin_y);
	printf("\t\tz: %f\n", bot_pub_lin_z);
	BOT::pub();
	BOT::edge_follow();
}

//Check if Bot is inside the boundary of any specified triangle
int BOT::shape_checker(float x, float y)
{
	float u, v, v0_x, v0_y, v1_x, v1_y, v2_x, v2_y;
	int i;
	for(i = 0; i < triangles.size(); i++)
	{
		v0_x = triangles[i].vertex_3_x - triangles[i].vertex_1_x;
		v0_y = triangles[i].vertex_3_y - triangles[i].vertex_1_y;
		v1_x = triangles[i].vertex_2_x - triangles[i].vertex_1_x;
		v1_y = triangles[i].vertex_2_y - triangles[i].vertex_1_y;
		v2_x = x - triangles[i].vertex_1_x;
		v2_y = y - triangles[i].vertex_1_y;

		u = ((((v1_x * v1_x) + (v1_y * v1_y)) * ((v2_x * v0_x) + (v2_y * v0_y))) - (((v1_x * v0_x) + (v1_y * v0_y)) * ((v2_x * v1_x) + (v2_y * v1_y)))) / ((((v0_x * v0_x) + (v0_y * v0_y)) * ((v1_x * v1_x) + (v1_y * v1_y))) - (((v0_x * v1_x) + (v0_y * v1_y)) * ((v1_x * v0_x) + (v1_y * v0_y))));

		v = ((((v0_x * v0_x) + (v0_y * v0_y)) * ((v2_x * v1_x) + (v2_y * v1_y))) - (((v0_x * v1_x) + (v0_y * v1_y)) * ((v2_x * v0_x) + (v2_y * v0_y)))) / ((((v0_x * v0_x) + (v0_y * v0_y)) * ((v1_x * v1_x) + (v1_y * v1_y))) - (((v0_x * v1_x) + (v0_y * v1_y)) * ((v1_x * v0_x) + (v1_y * v0_y))));

		if((u >= 0.0) && (v >= 0.0) && (u + v < 1))
		{
			if(x == bot_pos_x && y == bot_pos_y && state > 0)
			{
				target_pose_x = triangles[i].target_vertex_x;
				target_pose_y = triangles[i].target_vertex_y;
				stop_ang = triangles[i].stop_ang;
				shape_num = triangles[i].shape_id;
			}
			return 1;
		}
	}
	
   	return 0; 
}


//Check which formation point is the closest to the BOT
void BOT::closest_median()
{
	float closest_point = 1000000.0;
	int index = -1;
	int i;
	for(i = 0; i< triangles.size(); i++)
	{
		if(std::find(covered_shapes.begin(), covered_shapes.end(), triangles[i].shape_id) == covered_shapes.end())
		{
			float dist_to_vertex = BOT::getDist(bot_pos_x, bot_pos_y, triangles[i].ref_vertex_x , triangles[i].ref_vertex_y);
			if(dist_to_vertex < closest_point)
			{
				closest_point = dist_to_vertex;
				index = i;
			}
		}
	}
	
	target_pose_x = triangles[index].ref_vertex_x;
	target_pose_y = triangles[index].ref_vertex_y;
	closest_tri = index;
	shape_num = triangles[index].shape_id;
}

//Find the vertex from which the BOT can jump from one shape boundary to another
void BOT::exit_vertex(float x, float y, int shape_id)
{
	float closest_vertex_x = 1000000.0;
	float closest_vertex_y = 1000000.0;
	float closest_dist = 1000000.0;
	int i;
	for(i = 0; i< triangles.size(); i++)
	{
		if(triangles[i].shape_id == shape_id)
		{
			float dist_to_vertex = BOT::getDist(x, y, triangles[i].vertex_1_x , triangles[i].vertex_1_y);
			if(dist_to_vertex < closest_dist)
			{
				closest_dist = dist_to_vertex;
				closest_vertex_x = triangles[i].vertex_1_x;
				closest_vertex_y = triangles[i].vertex_1_y;
			}
			dist_to_vertex = BOT::getDist(x, y, triangles[i].vertex_2_x , triangles[i].vertex_2_y);
			if(dist_to_vertex < closest_dist)
			{
				closest_dist = dist_to_vertex;
				closest_vertex_x = triangles[i].vertex_2_x;
				closest_vertex_y = triangles[i].vertex_2_y;
			}
			dist_to_vertex = BOT::getDist(x, y, triangles[i].vertex_3_x , triangles[i].vertex_3_y);
			if(dist_to_vertex < closest_dist)
			{
				closest_dist = dist_to_vertex;
				closest_vertex_x = triangles[i].vertex_3_x;
				closest_vertex_y = triangles[i].vertex_3_y;
			}
		}
	}
	exit_x = closest_vertex_x;
	exit_y = closest_vertex_y;
}

//Specify shape to be formed
void BOT::init_shape()
{
	struct tri triangle;
	shape_num = 2;

	triangle.shape_id = 0;
	triangle.ref_vertex_x = 0.0;
	triangle.ref_vertex_y = -1.5;
	triangle.target_vertex_x = 0.0;
	triangle.target_vertex_y = -1.5;
	triangle.vertex_1_x = 0.0;
	triangle.vertex_1_y = 0.0;
	triangle.vertex_2_x = 3.0;
	triangle.vertex_2_y = -1.5;
	triangle.vertex_3_x = 0.0;
	triangle.vertex_3_y = -3.0;
	triangle.stop_ang = 270.0;//90.0;	

	triangles.push_back(triangle);

	
}

/*

*-finding stationary obstacle in front inside shape
!-Alternate clockwise and anti-clockwise
!-Check if next robot in shape

-Shape Travel : If closest point in current triangle, dont leave triangle
-If closest point outside current triangle, leave triangle
-If outside all triangles, mve to triangle
-If inside a triangle and trying to get into the neighboring triangle, make sure to stay inside a neighboring triangle. Just make sure when leaving current triangle, moving into another triangle.

*-Look for closest while ouside triangle
-Dont look for new closest while in a triangle
-When leaing a triangle, see if entering new triangle or exiting
-If entering new triangle and can enter, Enter it and then change closest point
-If entering new triangle but cannot enter, stop
-If exiting shape all together, stop

-if inside shape and blocking robot outside shape, stop
-check if inside shape of any neighbor bots

-getting stuck in depression: If surrounded and some part inside the shape, stop
*/
void BOT::edge_follow()
{
	float start_x = bot_pos_x;
	float start_y = bot_pos_y;
	int revolve = 0;
	int exit = 0;
	float step = 0.05;
	float comm_range = 5 * radii ;
	int current_shape = shape_num;
	float margin = 0.03;
	int crossed = 0;
	int inside_shape = BOT::shape_checker(bot_pos_x, bot_pos_y);
	while(/*((inside_shape == 0) || (state != 3)) &&*/ (state != 2))
	{
		float dist_from_start = BOT::getDist(bot_pos_x, bot_pos_y, start_x , start_y);
		if((dist_from_start < 2 * radii) && revolve == 2 && inside_shape == 0)
		{
			
			exit = 1;
			covered_shapes.push_back(shape_num);
			BOT::closest_median();
			BOT::exit_vertex(target_pose_x, target_pose_y, covered_shapes[covered_shapes.size() - 1]);
			revolve = 3;
		}
		else if((dist_from_start > 5 * radii) && (revolve == 1) && inside_shape == 0)
		{
			revolve = 2;
		}
		if(exit == 1)
		{
			printf("Exiting at %f %f\n", exit_x, exit_y);
			if(BOT::getDist(bot_pos_x, bot_pos_y, exit_x , exit_y) < 2*radii)
			{
				geometry_msgs::PoseStamped posi;
				posi.pose.position.x = target_pose_x;
				posi.pose.position.y = target_pose_y;
				posi.pose.position.z = 0.0;
				state = 0;
				//pref_vel = 0.3;
				BOT::motion_1D(posi);
				break;	
			}
		}
		if(inside_shape == 0)
		{
			/*if(state > 1)
			{
				start_x = bot_pos_x;
				start_y = bot_pos_y;
			}*/
			state = 1;
		}
		std::vector<float> obs_ang;
		std::vector<float> obs_dist;
		std::vector<int> states;
		std::vector<int> ids;
		printf("Finding new angle for %d\n", id);
		BOT::update_bot_info();
		
		float dir;
		float new_ang;
		int i;
		int closest = -1;
		float min_dist = 10000.0;
		for(i=0; i<dist.size(); i++)
		{
			if((dist[i] < min_dist) && (dist[i] <= comm_range) && (bots[visible_bots[i]].state >= 2))
			{
				min_dist = dist[i];
				closest = visible_bots[i];
			}
		}

		int turn = 1;
		if(closest == -1)
		{
			turn = 0;
			float new_ang = BOT::getAngle(bot_pos_x, bot_pos_y, target_pose_x, target_pose_y);
			dir = new_ang;
			float vo_radii = 2 * radii + margin;
			float vo_max_ang;
			float vo_min_ang;
			int obstacle = 0;
			float limit_min = new_ang - 90.0;
			float limit_max = new_ang;
			for(i = 0; i < dist.size(); i++)
			{				
				vo_max_ang = (asin(vo_radii/std::max(dist[i], vo_radii)) * 180/PI) + angle[i];
				vo_min_ang = angle[i] - (asin(vo_radii/std::max(dist[i], vo_radii)) * 180/PI);
				

				if(((new_ang < vo_max_ang && new_ang > vo_min_ang) || (new_ang < vo_max_ang+360.0 && new_ang > vo_min_ang+360.0) || (new_ang+360.0 < vo_max_ang && new_ang+360.0 > vo_min_ang)) && (dist[i] <= ((vo_radii) + step + 0.02)))
				{
					
					if((inside_shape == 0) && (BOT::shape_checker(bot_pos_x + dist[i] * cos(angle[i] * PI/180.0) , bot_pos_y + dist[i] * sin(angle[i] * PI/180.0)) == 1))				
					{
						turn = 0;
					}
					else if((inside_shape == 1) && (BOT::shape_checker(bot_pos_x + dist[i] * cos(angle[i] * PI/180.0) , bot_pos_y + dist[i] * sin(angle[i] * PI/180.0)) == 0))
					{
						state = 3;
						turn = 0;
					}
					else if((limit_min < vo_min_ang && limit_max > vo_min_ang) || (limit_min + 360.0 < vo_min_ang && limit_max + 360.0 > vo_min_ang) || (limit_min < vo_min_ang + 360.0 && limit_max> vo_min_ang + 360.0))
					{
						
						int future_point = BOT::shape_checker((bot_pos_x + (step + radii) * cos((vo_min_ang - 10.0) * PI/180.0)) , (bot_pos_y + (step + radii) * sin((vo_min_ang - 10.0) * PI/180.0)));
						if((inside_shape == 1) && future_point == 0)
						{
							turn = 0;							
						}
						else
						{
							new_ang = vo_min_ang - 10.0;							
							if(new_ang < 0.0)
							{
								if(new_ang <= -360.0)
								{
									new_ang += 360.0;
								}
								new_ang += 360.0;
							}	
						}
						
					}
					else
					{
						obstacle = 1;
						break;
					}
				}
			}
			if(obstacle == 0)
			{
				float ang_diff = angles.z - new_ang;
				if(ang_diff < -180.0)
				{
					ang_diff = 360.0 + ang_diff;
				}
				else if(ang_diff > 180.0)
				{
					ang_diff = ang_diff - 360;
				} 
				BOT::rotate(ang_diff);
				float ang_vel = 0.0;
				float lin_vel = 0.05;	
				float initial_x = bot_pos_x;
				float initial_y = bot_pos_y; 	
			
				while(BOT::getDist(initial_x, initial_y, bot_pos_x, bot_pos_y) < lin_vel)
				{
					//printf("%f\n", bot_lin_x);
					bot_pub_lin_x = lin_vel;
					bot_pub_lin_y = 0.0;
					bot_pub_lin_z = 0.0;
					bot_pub_ang_x = 0.0;
					bot_pub_ang_y = 0.0;
					bot_pub_ang_z = ang_vel;
					BOT::pub();
					ros::spinOnce();
				}
				printf("Finished rotating bot %d\n", id);
				bot_pub_lin_x = 0.0;
				bot_pub_lin_y = 0.0;
				bot_pub_lin_z = 0.0;
				bot_pub_ang_x = 0.0;
				bot_pub_ang_y = 0.0;
				bot_pub_ang_z = 0.0;
				BOT::pub();
				ros::spinOnce();
				inside_shape = BOT::shape_checker(bot_pos_x, bot_pos_y);
				if(inside_shape == 0)
				{
					BOT::closest_median();	
				}
			}
		}
		else
		{	
			int at_boundary = 0;
			int k;
			for(k=0; k<dist.size(); k++)
			{
				if(closest == visible_bots[k])
				{
					break;
				}
			}
			if(inside_shape == 0)
			{
				dir = BOT::getAngle(bot_pos_x, bot_pos_y, bots[closest].bot_pos_x, bots[closest].bot_pos_y) - (asin((2 * radii)/dist[k]) * 180/PI) - 10.0;
			}
			else
			{
				dir = BOT::getAngle(bot_pos_x, bot_pos_y, target_pose_x, target_pose_y);
			}
		
			for(i=0; i<dist.size(); i++)
			{
				if(dist[i] <= comm_range)
				{
					float corrected_ang;
					if(angle[i] > dir)
					{	
						corrected_ang = angle[i] - 360.0;
					}	
					else
					{
						corrected_ang = angle[i];
					}				
					obs_ang.push_back(corrected_ang);
					obs_dist.push_back(dist[i]);
					states.push_back(bots[visible_bots[i]].state);
					ids.push_back(visible_bots[i]);
					//printf("%d\n",i);
					int j = 0;
					while(corrected_ang <= obs_ang[j] && j < obs_ang.size())
					{
						j++;
					} 
					if(j != obs_ang.size())
					{
						int k = obs_ang.size() - 1;
						while(k > j)
						{
							obs_ang[k] = obs_ang[k-1];
							obs_dist[k] = obs_dist[k-1];
							states[k] = states[k-1];
							ids[k] = ids[k-1];
							k--;	
						}
						obs_ang[j] = corrected_ang;
						obs_dist[j] = dist[i];
						states[j] = bots[visible_bots[i]].state;
						ids[j] = visible_bots[i];
					}
				}  
			}

			new_ang = dir;
			float vo_radii = 2 * radii + margin;
			float vo_max_ang;
			float vo_min_ang;
			printf("Max ang = %d\n", obs_ang.size()-1);
			printf("Inside shape for %d = %d %f %f\n", id, inside_shape, bot_pos_x, bot_pos_y);
			if(obs_ang.size() != 0)
			{
				vo_max_ang = (asin(vo_radii/std::max(obs_dist[obs_ang.size()-1], vo_radii)) * 180/PI) + obs_ang[obs_ang.size()-1];
		
				vo_min_ang = obs_ang[obs_ang.size()-1] - (asin(vo_radii/std::max(obs_dist[obs_ang.size()-1], vo_radii)) * 180/PI);
				//printf("No Seg 3\n");
				
				if(obs_ang[obs_ang.size()-1] < 0.0)
				{
					vo_max_ang += 360.0;
					vo_min_ang += 360.0;
				}

				printf(" Outside Max = %f Min = %f Center = %f Current = %f new_ang = %f obs = %d state = %d id = %d dist = %f\n", vo_max_ang, vo_min_ang, obs_ang[obs_ang.size()-1], dir, new_ang, obs_ang.size(),states[obs_ang.size()-1], ids[obs_ang.size()-1], obs_dist[obs_ang.size()-1]);

				if(((new_ang < vo_max_ang && new_ang > vo_min_ang) || (new_ang < vo_max_ang+360.0 && new_ang > vo_min_ang+360.0) || (new_ang+360.0 < vo_max_ang && new_ang+360.0 > vo_min_ang)) && (obs_dist[obs_ang.size()-1] <= ((vo_radii) + step + 0.02)))
				{	
					
					if(states[obs_ang.size()-1] >= 2)
					{
						new_ang = vo_min_ang - 10.0;
						if(inside_shape == 0)
						{
							at_boundary = 1;
						}
						if(new_ang < 0.0)
						{
							if(new_ang <= -360.0)
							{
								new_ang += 360.0;
							}
							new_ang += 360.0;
						}
						if(revolve == 0 && inside_shape == 0)
						{
							
							revolve = 1;
						}
					}
					else
					{
						float limit_min = new_ang - 90.0;
						float limit_max = new_ang; 
						if((inside_shape == 0) && (BOT::shape_checker(bot_pos_x + obs_dist[obs_ang.size()-1] * cos(obs_ang[obs_ang.size()-1] * PI/180.0) , bot_pos_y + obs_dist[obs_ang.size()-1] * sin(obs_ang[obs_ang.size()-1] * PI/180.0)) == 1))				
						{
							
							turn = 0;
						}
						else if(((limit_min < vo_min_ang && limit_max > vo_min_ang) || (limit_min + 360.0 < vo_min_ang && limit_max + 360.0 > vo_min_ang) || (limit_min < vo_min_ang + 360.0 && limit_max> vo_min_ang + 360.0)) && (at_boundary == 0))
						{

							int future_point = BOT::shape_checker((bot_pos_x + (step + radii) * cos((vo_min_ang - 10.0) * PI/180.0)) , (bot_pos_y + (step + radii) * sin((vo_min_ang - 10.0) * PI/180.0)));
							if((inside_shape == 1) && future_point == 0)
							{
								turn = 0;							
							}
							else if(inside_shape == 1)
							{
								new_ang = vo_min_ang - 10.0;							
								if(new_ang < 0.0)
								{
									if(new_ang <= -360.0)
									{
										new_ang += 360.0;
									}
									new_ang += 360.0;
								}	
							}
							printf("New_ang = %f\n", new_ang);
							
						}
						else
						{
							
							turn = 0;
						}
					}
				
				}
			}
			printf("Turn = %d", turn);
			for(i = 0; i < obs_ang.size(); i++)
			{
				vo_max_ang = (asin(vo_radii/std::max(obs_dist[i], vo_radii)) * 180/PI) + obs_ang[i];
				vo_min_ang = obs_ang[i] - (asin(vo_radii/std::max(obs_dist[i], vo_radii)) * 180/PI);

				if(obs_ang[i] < 0.0)
				{
					vo_max_ang += 360.0;
					vo_min_ang += 360.0;
				}

				printf(" Inside Max = %f Min = %f Center = %f Current = %f New_ang = %f Dist = %f State = %d id = %d\n", vo_max_ang, vo_min_ang, obs_ang[i], dir, new_ang, obs_dist[i], states[i], ids[i]);
				if(((new_ang < vo_max_ang && new_ang > vo_min_ang) || (new_ang < vo_max_ang+360.0 && new_ang > vo_min_ang+360.0) || (new_ang+360.0 < vo_max_ang && new_ang+360.0 > vo_min_ang)) && (obs_dist[i] <= ((vo_radii) + step + 0.02)))
				{	
					if(states[i] >= 2)
					{
						new_ang = vo_min_ang - 10.0;
						if(inside_shape == 0)
						{
							at_boundary = 1;
						}
						if(new_ang < 0.0)
						{
							if(new_ang <= -360.0)
							{
								new_ang += 360.0;
							}
							new_ang += 360.0;
						}
						if(revolve == 0 && inside_shape == 0)
						{
							//start_x = bot_pos_x;
							//start_y = bot_pos_y;
							revolve = 1;
						}
						turn = 1;
						
					}
					else
					{
						float limit_min = dir - 90.0;
						float limit_max = dir;
						
						if((inside_shape == 0) && (BOT::shape_checker(bot_pos_x + obs_dist[i] * cos(obs_ang[i] * PI/180.0) , bot_pos_y + obs_dist[i] * sin(obs_ang[i] * PI/180.0)) == 1))				
						{
							
							turn = 0;
						}
						else if((inside_shape == 1) && (BOT::shape_checker(bot_pos_x + obs_dist[i] * cos(obs_ang[i] * PI/180.0) , bot_pos_y + obs_dist[i] * sin(obs_ang[i] * PI/180.0)) == 0))
						{
							
							state = 3;
							turn = 0;
							//}
						}				
						else if(((limit_min < vo_min_ang && limit_max > vo_min_ang) || (limit_min + 360.0 < vo_min_ang && limit_max + 360.0 > vo_min_ang) || (limit_min < vo_min_ang + 360.0 && limit_max> vo_min_ang + 360.0)) && (at_boundary == 0))
						{
							
							int future_point = BOT::shape_checker((bot_pos_x + (step + radii) * cos((vo_min_ang - 10.0) * PI/180.0)) , (bot_pos_y + (step + radii) * sin((vo_min_ang - 10.0) * PI/180.0)));
							if((inside_shape == 1) && future_point == 0)
							{
								turn = 0;							
							}
							else if(inside_shape == 1)
							{
								new_ang = vo_min_ang - 10.0;							
								if(new_ang < 0.0)
								{
									if(new_ang <= -360.0)
									{
										new_ang += 360.0;
									}
									new_ang += 360.0;
								}	
							}
							
						}
						else
						{
							/*if(inside_shape == 1)
							{
								state = 3;
							}*/
							turn = 0;
							break;
						}
					}
				}
				else
				{
					//break;
				}
				printf("Turn = %d", turn);
			}
			printf("Turn = %d", turn);
			if(obs_ang.size() == 0)
			{
				dir = BOT::getAngle(bot_pos_x, bot_pos_y, target_pose_x, target_pose_y);
			}
			
		}
		
		if(new_ang < 0.0)
		{
			new_ang += 360.0;
		}

		float max = dir + 10.0;
		float min = dir - 140.0;
		
		if((inside_shape == 1) && !((new_ang < max && new_ang > min) || (new_ang < max+360.0 && new_ang > min+360.0) || (new_ang+360.0 < max && new_ang+360.0 > min)))
		{
			
			printf("\n\n\n****************OUT OF RANGE %d *****************\n\n\n", id);
			state = 3;
			turn = 0;
		}
		

		if(((fabs(target_pose_x - bot_pos_x) < 0.03) && (fabs(target_pose_y - bot_pos_y) < 0.03)))
		{
			state = 2;
			turn = 0;
		}

		int future_point = BOT::shape_checker((bot_pos_x + (step + radii) * cos(new_ang * PI/180.0)) , (bot_pos_y + (step + radii) * sin(new_ang * PI/180.0)));
		if((inside_shape == 1) && future_point == 0)
		{
			printf("\n\n\n****************LEAVING SHAPE %f %f *****************\n\n\n", bot_pos_x + (step + radii) * cos(new_ang * PI/180.0) , bot_pos_y + (step + radii) * sin(new_ang * PI/180.0));
			state = 3;
			turn = 0;
		}
		

		if(turn)
		{
			printf("###########  EDGE FOLLOWING  ##################\n");
			printf("Bot %d Current X = %f , Y = %f Angle = %f Turn = %d States = %d\n", id, bot_pos_x, bot_pos_y, angles.z, turn, state);
			printf("Bot %d Target X = %f , Y = %f Angle = %f\n", id, target_pose_x, target_pose_y, new_ang);
			
			float ang_diff = angles.z - new_ang;
			if(ang_diff < -180.0)
			{
				ang_diff = 360.0 + ang_diff;
			}
			else if(ang_diff > 180.0)
			{
				ang_diff = ang_diff - 360;
			} 
			BOT::rotate(ang_diff);
			BOT::update_bot_info();
			float vo_radii = 2 * radii + margin;
			float vo_max_ang;
			float vo_min_ang;
			int obstacle = 0;
			for(i = 0; i < dist.size(); i++)
			{				
				vo_max_ang = (asin(vo_radii/std::max(dist[i], vo_radii)) * 180/PI) + angle[i];
				vo_min_ang = angle[i] - (asin(vo_radii/std::max(dist[i], vo_radii)) * 180/PI);

				if(((new_ang < vo_max_ang && new_ang > vo_min_ang) || (new_ang < vo_max_ang+360.0 && new_ang > vo_min_ang+360.0) || (new_ang+360.0 < vo_max_ang && new_ang+360.0 > vo_min_ang)) && (dist[i] <= ((vo_radii) + step + 0.02)))
				{
					printf("Id = %d   Obs = %d\n", id, i);
					printf("%f %f %f\n", vo_max_ang, new_ang, vo_min_ang);
					obstacle = 1;
					break;
				}
			}
			printf("###########  OBSTACLE %d ##################\n", obstacle);	
			float ang_vel = 0.0;
			float lin_vel = 0.05;	
			float initial_x = bot_pos_x;
			float initial_y = bot_pos_y; 	
			
			while((BOT::getDist(initial_x, initial_y, bot_pos_x, bot_pos_y) < lin_vel) && obstacle == 0)
			{
				//printf("%f\n", bot_lin_x);
				bot_pub_lin_x = lin_vel;
				bot_pub_lin_y = 0.0;
				bot_pub_lin_z = 0.0;
				bot_pub_ang_x = 0.0;
				bot_pub_ang_y = 0.0;
				bot_pub_ang_z = ang_vel;
				BOT::pub();
				ros::spinOnce();
			}
			printf("Finished rotating bot %d\n", id);
		}
		bot_pub_lin_x = 0.0;
		bot_pub_lin_y = 0.0;
		bot_pub_lin_z = 0.0;
		bot_pub_ang_x = 0.0;
		bot_pub_ang_y = 0.0;
		bot_pub_ang_z = 0.0;
		BOT::pub();
		ros::spinOnce();
		if(inside_shape == 0)
		{
			BOT::closest_median();

		}
		inside_shape = BOT::shape_checker(bot_pos_x, bot_pos_y);

		
	}
	
	bot_pub_ang_x = 0.0;
	bot_pub_ang_y = 0.0;		
	bot_pub_ang_z = 0.0;
	bot_pub_lin_x = 0.0;
	bot_pub_lin_y = 0.0;
	bot_pub_lin_z = 0.0;
	printf("\tPublish Linear: %d\n", id);
	printf("\t\tx: %f\n", bot_pub_lin_x);
	printf("\t\ty: %f\n", bot_pub_lin_y);
	printf("\t\tz: %f\n", bot_pub_lin_z);
	BOT::pub();
}

void BOT::rand_motion()
{
	float lin_vel = 0.05;
	float ang_vel = 0.0;
	int connnected = 0;
	ros::Rate rate(20.0);
	ros::Time start = ros::Time::now();
	float prev_error = 0.0;
	float target_ang;
	if(id == 0)
	{
		state = 0;
	}
	while(state == -1)
	{
		int n = 0;
		int m = 0;
		BOT::update_bot_info();
		int i;
		std::vector<float> rand_obs_ang;
		for(i = 0; i < dist.size(); i++)
		{
			if(dist[i] < 4 * radii)
			{
				if(dist[i] < 3 * radii)
				{
					rand_obs_ang.push_back(angle[i]);
				}
				if(angle[i] < 180.0)
				{
					//n++;
				}
				else
				{
					//m++;
				}
			}
	
		}
		target_ang = (rand() + ((180 * m)/(m + 1))) % (360 - ((180 * n)/(n + 1)));
		if(rand_obs_ang.size() > 0)
		{
			int ang_sum = 0;
			for(i = 0; i< rand_obs_ang.size(); i++)
			{
				ang_sum +=rand_obs_ang[i]; 
			}
			target_ang = ((ang_sum / rand_obs_ang.size()) + 180)%360;
			
		}
		printf("Current Angle= %f\n", angles.z);
		printf("Target Angle= %f\n", target_ang);
		printf("Range %d %d\n", ((180 * m)/(m + 1)), (360 - ((180 * n)/(n + 1))));
		if(fabs(target_ang - angles.z >= 180.0))
		{
			if(target_ang < angles.z)
			{
				target_ang = 360.0 + target_ang;
			}
			else
			{
				target_ang = 360.0 - target_ang;	
			}
		}

		if((angles.z <= (target_ang + 2.0)) && (angles.z >= (target_ang - 2.0)))
		{
			ang_vel = 0.0;
		}
		else
		{
			if(angles.z > target_ang)
			{
				
				ang_vel = (0.07*(angles.z - target_ang) + 0.05*((angles.z - target_ang)-prev_error)/0.1);
				if(ang_vel <= -1.0)
				{
					ang_vel = -1.0;
				}
				else if(ang_vel >= 1.0)
				{
					ang_vel = 1.0;
				}
				prev_error = (angles.z - target_ang);
			}
			else
			{
				
				ang_vel = (0.04*(angles.z - target_ang) + 0.05*((angles.z - target_ang)-prev_error)/0.1);
				if(ang_vel <= -1.0)
				{
					ang_vel = -1.0;
				}
				else if(ang_vel >= 1.0)
				{
					ang_vel = 1.0;
				}
				prev_error = (angles.z - target_ang);;
			}	
			printf("CHANGING ANGLE %f\n", ang_vel);
		}
		
		printf("****DONE CHECKING OBSTACLES FOR %d ******\n", id);
		bot_pub_lin_x = lin_vel;
		bot_pub_lin_y = 0.0;
		bot_pub_lin_z = 0.0;
		bot_pub_ang_x = 0.0;
		bot_pub_ang_y = 0.0;
		bot_pub_ang_z = ang_vel;
		printf("\tPosition: %d\n", id);
		printf("\t\tx: %f\n", bot_pos_x);
		printf("\t\ty: %f\n", bot_pos_y);
		printf("\t\tz: %f\n", bot_pos_z);
		printf("\tPublish Linear: %d\n", id);
		printf("\t\tx: %f\n", bot_pub_lin_x);
		printf("\t\ty: %f\n", bot_pub_lin_y);
		printf("\t\tz: %f\n", bot_pub_lin_z);
		BOT::pub();
		ros::spinOnce();
		//rate.sleep();
		
	}
	moving = 0;
	bot_pub_ang_x = 0.0;
	bot_pub_ang_y = 0.0;		
	bot_pub_ang_z = 0.0;
	bot_pub_lin_x = 0.0;
	bot_pub_lin_y = 0.0;
	bot_pub_lin_z = 0.0;
	printf("\tPublish Linear: %d\n", id);
	printf("\t\tx: %f\n", bot_pub_lin_x);
	printf("\t\ty: %f\n", bot_pub_lin_y);
	printf("\t\tz: %f\n", bot_pub_lin_z);
	BOT::pub();
	BOT::closest_median();
	BOT::rotate(angles.z - BOT::getAngle(bot_pos_x, bot_pos_y, target_pose_x, target_pose_y));
	geometry_msgs::PoseStamped posi;
	posi.pose.position.x = target_pose_x;
	posi.pose.position.y = target_pose_y;
	posi.pose.position.z = 0.0;
	BOT::motion_1D(posi);
	//BOT::edge_follow();
	while(1)
	{	
		bot_pub_ang_x = 0.0;
		bot_pub_ang_y = 0.0;		
		bot_pub_ang_z = 0.0;
		bot_pub_lin_x = 0.0;
		bot_pub_lin_y = 0.0;
		bot_pub_lin_z = 0.0;
		BOT::pub();
		BOT::update_bot_info();
	}
}

void BOT::pub()
{
	geometry_msgs::Twist publ;
	publ.angular.x = 0.0;
	publ.angular.y = 0.0;
	publ.angular.z = bot_pub_ang_z;
	publ.linear.x = bot_pub_lin_x;
	publ.linear.y = bot_pub_lin_y;
	publ.linear.z = bot_pub_lin_z;
	//printf("Publishing %f\n", bot_pub_ang_z);
	publisher.publish(publ);
	//printf("Publishing-2 %f\n", bot_pub_ang_z);
}

