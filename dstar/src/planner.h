#include <cstdio>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <string>
//#include <stderr.h>
/** include ros libraries**********************/
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
/** ********************************************/ 

#include <boost/foreach.hpp>
//#define forEach BOOST_FOREACH

/** for global path planner interface */
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>

//#include <pcl_conversions/pcl_conversions.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include <set>
using std::string;
#include"Dstar.h"

#ifndef ROSPLANNER_H
#define ROSPLANNER_H

namespace plannerD
{
	class ROSplanner:public nav_core::BaseGlobalPlanner
	{
		public:
			ros::NodeHandle ROSNodeHandle;
			ROSplanner();
			ROSplanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
			void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
			bool makePlan(const geometry_msgs::PoseStamped& start,const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan);
			~ROSplanner();
		private:
			void publishPath(const std::vector<geometry_msgs::PoseStamped> grid_plan);
			int plan(std::vector<geometry_msgs::PoseStamped>& grid_plan,
			                      const geometry_msgs::PoseStamped& start,const geometry_msgs::PoseStamped& goal);		
			Dstar *ds;
			float originX;
			float originY;
			float resolution;
			costmap_2d::Costmap2DROS* costmap_ros_;
			costmap_2d::Costmap2D* costmap_;
			bool initialized_;
			int width;
			int height;
			int mapSize;
			int value;
			int cnt_no_plan_;
			int cnt_make_plan_;
			std::string costmap_frame_;  ///<  @brief costmap frame
    			std::string planner_frame_; ///<  @brief planner frame
			base_local_planner::CostmapModel* world_model_; ///<  @brief World Model associated to the costmap
    			std::vector<geometry_msgs::Point> footprint_spec_; ///< @brief FootPrint list of points of the robot

			ros::Publisher pub_path_;
			ros::NodeHandle nh; 
			//ros::Publisher pub_path_dedicated_;

		
		
		
	};
};

#endif
