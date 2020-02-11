#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "bot17.h"
#include <pthread.h>

#define PI 3.14159265358979323846
#define NUM_BOTS 45
/*
Get angle to point
rotate to face point
keep calculating which bots are within given radius
move till you reach point or obstacle
edge follow towards lower gradient
stop when cant go closer.
work on local and global minimum
*/

struct node{
	BOT *bot;
	int id;
};


float getAngle(BOT bot, geometry_msgs::PoseStamped target_pose)
{
	float angle = atan((target_pose.pose.position.y - bot.bot_pos_y)/(target_pose.pose.position.x - bot.bot_pos_x)) * 180/PI;
	if(target_pose.pose.position.x < bot.bot_pos_x)
	{
		angle = 180.0 + angle;
	}
	else if((target_pose.pose.position.y < bot.bot_pos_y) & (target_pose.pose.position.x > bot.bot_pos_x))
	{
		angle = 360.0 + angle;
	}
	return angle;
}


void *controller(void *param)
{
	
	struct node *data = (struct node*)param;
	geometry_msgs::PoseStamped posi;
	posi.pose.position.x = 0.0;
	posi.pose.position.y = 0.0;
	posi.pose.position.z = 0.0;
	printf("Bot ID: %d\n", data->id);

	data->bot[data->id].rand_motion();
	while(1){
	}
	pthread_exit(NULL);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Nami");
	//ros::NodeHandle nh;
	const std::string& pub_topic_0 = "/create_clone_0/cmd_vel";
	const std::string& pub_topic_1 = "/create_clone_1/cmd_vel";
	const std::string& pub_topic_2 = "/create_clone_2/cmd_vel";
	const std::string& pub_topic_3 = "/create_clone_3/cmd_vel";
	const std::string& pub_topic_4 = "/create_clone_4/cmd_vel";
	const std::string& pub_topic_5 = "/create_clone_5/cmd_vel";
	const std::string& pub_topic_6 = "/create_clone_6/cmd_vel";
	const std::string& pub_topic_7 = "/create_clone_7/cmd_vel";
	const std::string& pub_topic_8 = "/create_clone/cmd_vel";
	const std::string& pub_topic_9 = "/create/cmd_vel";
	const std::string& pub_topic_10 = "/create_0/cmd_vel";
	const std::string& pub_topic_11 = "/create_1/cmd_vel";
	const std::string& pub_topic_12 = "/create_2/cmd_vel";
	const std::string& pub_topic_13 = "/create_3/cmd_vel";
	const std::string& pub_topic_14 = "/create_4/cmd_vel";
	const std::string& pub_topic_15 = "/create_5/cmd_vel";
	const std::string& pub_topic_16 = "/create_6/cmd_vel";
	const std::string& pub_topic_17 = "/create_7/cmd_vel";
	const std::string& pub_topic_18 = "/create_8/cmd_vel";
	const std::string& pub_topic_19 = "/create_9/cmd_vel";
	const std::string& pub_topic_20 = "/create_10/cmd_vel";
	const std::string& pub_topic_21 = "/create_11/cmd_vel";
	const std::string& pub_topic_22 = "/create_12/cmd_vel";
	const std::string& pub_topic_23 = "/create_13/cmd_vel";
	const std::string& pub_topic_24 = "/create_14/cmd_vel";
	const std::string& pub_topic_25 = "/create_15/cmd_vel";
	const std::string& pub_topic_26 = "/create_16/cmd_vel";
	const std::string& pub_topic_27 = "/create_17/cmd_vel";
	const std::string& pub_topic_28 = "/create_18/cmd_vel";
	const std::string& pub_topic_29 = "/create_19/cmd_vel";
	const std::string& pub_topic_30 = "/create_20/cmd_vel";
	const std::string& pub_topic_31 = "/create_21/cmd_vel";
	const std::string& pub_topic_32 = "/create_22/cmd_vel";
	const std::string& pub_topic_33 = "/create_23/cmd_vel";
	const std::string& pub_topic_34 = "/create_24/cmd_vel";
	const std::string& pub_topic_35 = "/create_25/cmd_vel";
	const std::string& pub_topic_36 = "/create_26/cmd_vel";
	const std::string& pub_topic_37 = "/create_27/cmd_vel";
	const std::string& pub_topic_38 = "/create_28/cmd_vel";
	const std::string& pub_topic_39 = "/create_29/cmd_vel";
	const std::string& pub_topic_40 = "/create_30/cmd_vel";
	const std::string& pub_topic_41 = "/create_31/cmd_vel";
	const std::string& pub_topic_42 = "/create_32/cmd_vel";
	const std::string& pub_topic_43 = "/create_33/cmd_vel";
	const std::string& pub_topic_44 = "/create_34/cmd_vel";

	const std::string& sub_topic_0 = "/create_clone_0/odom";
	const std::string& sub_topic_1 = "/create_clone_1/odom";
	const std::string& sub_topic_2 = "/create_clone_2/odom";
	const std::string& sub_topic_3 = "/create_clone_3/odom";
	const std::string& sub_topic_4 = "/create_clone_4/odom";
	const std::string& sub_topic_5 = "/create_clone_5/odom";
	const std::string& sub_topic_6 = "/create_clone_6/odom";
	const std::string& sub_topic_7 = "/create_clone_7/odom";
	const std::string& sub_topic_8 = "/create_clone/odom";
	const std::string& sub_topic_9 = "/create/odom";
	const std::string& sub_topic_10 = "/create_0/odom";
	const std::string& sub_topic_11 = "/create_1/odom";
	const std::string& sub_topic_12 = "/create_2/odom";
	const std::string& sub_topic_13 = "/create_3/odom";
	const std::string& sub_topic_14 = "/create_4/odom";
	const std::string& sub_topic_15 = "/create_5/odom";
	const std::string& sub_topic_16 = "/create_6/odom";
	const std::string& sub_topic_17 = "/create_7/odom";
	const std::string& sub_topic_18 = "/create_8/odom";
	const std::string& sub_topic_19 = "/create_9/odom";
	const std::string& sub_topic_20 = "/create_10/odom";
	const std::string& sub_topic_21 = "/create_11/odom";
	const std::string& sub_topic_22 = "/create_12/odom";
	const std::string& sub_topic_23 = "/create_13/odom";
	const std::string& sub_topic_24 = "/create_14/odom";
	const std::string& sub_topic_25 = "/create_15/odom";
	const std::string& sub_topic_26 = "/create_16/odom";
	const std::string& sub_topic_27 = "/create_17/odom";
	const std::string& sub_topic_28 = "/create_18/odom";
	const std::string& sub_topic_29 = "/create_19/odom";
	const std::string& sub_topic_30 = "/create_20/odom";
	const std::string& sub_topic_31 = "/create_21/odom";
	const std::string& sub_topic_32 = "/create_22/odom";
	const std::string& sub_topic_33 = "/create_23/odom";
	const std::string& sub_topic_34 = "/create_24/odom";
	const std::string& sub_topic_35 = "/create_25/odom";
	const std::string& sub_topic_36 = "/create_26/odom";
	const std::string& sub_topic_37 = "/create_27/odom";
	const std::string& sub_topic_38 = "/create_28/odom";
	const std::string& sub_topic_39 = "/create_29/odom";
	const std::string& sub_topic_40 = "/create_30/odom";
	const std::string& sub_topic_41 = "/create_31/odom";
	const std::string& sub_topic_42 = "/create_32/odom";
	const std::string& sub_topic_43 = "/create_33/odom";
	const std::string& sub_topic_44 = "/create_34/odom";


	BOT bot[45]= {BOT(0, NUM_BOTS, pub_topic_0, sub_topic_0), BOT(1, NUM_BOTS, pub_topic_1, sub_topic_1), BOT(2, NUM_BOTS, pub_topic_2, sub_topic_2), BOT(3, NUM_BOTS, pub_topic_3, sub_topic_3), BOT(4, NUM_BOTS, pub_topic_4, sub_topic_4), BOT(5, NUM_BOTS, pub_topic_5, sub_topic_5), BOT(6, NUM_BOTS, pub_topic_6, sub_topic_6), BOT(7, NUM_BOTS, pub_topic_7, sub_topic_7), BOT(8, NUM_BOTS, pub_topic_8, sub_topic_8), BOT(9, NUM_BOTS, pub_topic_9, sub_topic_9), BOT(10, NUM_BOTS, pub_topic_10, sub_topic_10), BOT(11, NUM_BOTS, pub_topic_11, sub_topic_11), BOT(12, NUM_BOTS, pub_topic_12, sub_topic_12), BOT(13, NUM_BOTS, pub_topic_13, sub_topic_13), BOT(14, NUM_BOTS, pub_topic_14, sub_topic_14), BOT(15, NUM_BOTS, pub_topic_15, sub_topic_15), BOT(16, NUM_BOTS, pub_topic_16, sub_topic_16), BOT(17, NUM_BOTS, pub_topic_17, sub_topic_17), BOT(18, NUM_BOTS, pub_topic_18, sub_topic_18), BOT(19, NUM_BOTS, pub_topic_19, sub_topic_19), BOT(20, NUM_BOTS, pub_topic_20, sub_topic_20), BOT(21, NUM_BOTS, pub_topic_21, sub_topic_21), BOT(22, NUM_BOTS, pub_topic_22, sub_topic_22), BOT(23, NUM_BOTS, pub_topic_23, sub_topic_23), BOT(24, NUM_BOTS, pub_topic_24, sub_topic_24), BOT(25, NUM_BOTS, pub_topic_25, sub_topic_25), BOT(26, NUM_BOTS, pub_topic_26, sub_topic_26), BOT(27, NUM_BOTS, pub_topic_27, sub_topic_27), BOT(28, NUM_BOTS, pub_topic_28, sub_topic_28), BOT(29, NUM_BOTS, pub_topic_29, sub_topic_29), BOT(30, NUM_BOTS, pub_topic_30, sub_topic_30), BOT(31, NUM_BOTS, pub_topic_31, sub_topic_31), BOT(32, NUM_BOTS, pub_topic_32, sub_topic_32), BOT(33, NUM_BOTS, pub_topic_33, sub_topic_33), BOT(34, NUM_BOTS, pub_topic_34, sub_topic_34), BOT(35, NUM_BOTS, pub_topic_35, sub_topic_35), BOT(36, NUM_BOTS, pub_topic_36, sub_topic_36), BOT(37, NUM_BOTS, pub_topic_37, sub_topic_37), BOT(38, NUM_BOTS, pub_topic_38, sub_topic_38), BOT(39, NUM_BOTS, pub_topic_39, sub_topic_39), BOT(40, NUM_BOTS, pub_topic_40, sub_topic_40), BOT(41, NUM_BOTS, pub_topic_41, sub_topic_41), BOT(42, NUM_BOTS, pub_topic_42, sub_topic_42), BOT(43, NUM_BOTS, pub_topic_43, sub_topic_43), BOT(44, NUM_BOTS, pub_topic_44, sub_topic_44)};
	
	int k;
	for(k=0; k < NUM_BOTS; k++)
	{
		bot[k].bots = bot;
	}

	geometry_msgs::PoseStamped pos;
	pos.pose.position.x = 0.0;
	pos.pose.position.y = 0.0;
	pos.pose.position.z = 0.0;
	struct node *params = (struct node*)malloc(sizeof(struct node) * NUM_BOTS);
	pthread_t threads[NUM_BOTS];
	int i =0;
	while(i < 1000 )
	{
		i++;
	}
	printf("Entered\n");
	int j;
	for(j=0; j<NUM_BOTS; j++)
	{
		printf("1\n");
		params[j].bot = bot;
		printf("2\n");
		params[j].id = j;
		printf("Creating thread %d\n", j);
		int rc = pthread_create(&threads[j], NULL, controller,(void *)&params[j]);
		if(rc)
		{
			printf("Failed to create thread %d\n", j);
			exit(-1);
		}
		if(j==(NUM_BOTS -1))
			pthread_join(threads[j], NULL);
				
	}
	printf("Exiting thread \n");
	pthread_exit(NULL);
	      	
		
}
