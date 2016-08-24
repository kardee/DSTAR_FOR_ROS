#include "planner.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(plannerD::ROSplanner, nav_core::BaseGlobalPlanner)
	
namespace plannerD
{
	ROSplanner::ROSplanner()
	{
		cout << "Entering into the no parametrized constructor" << endl;
	}
	
	ROSplanner::ROSplanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
	{
		cout << "Entering into the two parametrized ROSplanner constructor" << endl;
		initialize(name,costmap_ros);
	}
	
	void ROSplanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
	{
		if (!initialized_)
  		{
    			costmap_ros_ = costmap_ros;
    			costmap_ = costmap_ros_->getCostmap();
			costmap_frame_ = "map";
    			ros::NodeHandle private_nh("~/" + name);
	
    			originX = costmap_->getOriginX();
    			originY = costmap_->getOriginY();
			width = costmap_->getSizeInCellsX();
			height = costmap_->getSizeInCellsY();
			resolution = costmap_->getResolution();
			mapSize = width*height;
			//tBreak = 1+1/(mapSize); 
			value =0;
			
			try
			{
				footprint_spec_ = costmap_ros->getRobotFootprint();
				if((int)(footprint_spec_.size())>0)
					ROS_INFO("footprint_spec_ loaded with %d elements", (int)footprint_spec_.size());
				//world_model_ = new CostmapModel(*costmap_);
				ds = new Dstar();
				ds->init(0,0,10,10); 
			}
			catch(exception &e)
			{
				ROS_ERROR("Exception occured");
			}
			
			pub_path_ = nh.advertise<nav_msgs::Path>("dstar_planner_path",1000);
			//pub_path_dedicated_ = nh_.advertise<visualization_msgs::Marker>("dstar_planner",1000);

			int size_footprint = (int) footprint_spec_.size();
			//initialized_ = true;
		

			/*
			for (unsigned int iy = 0; iy < costmap_->getSizeInCellsY(); iy++)
    			{
      				for (unsigned int ix = 0; ix < costmap_->getSizeInCellsX(); ix++)
      				{
        				unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));
        				//cout<<cost;
        				if (cost == 0)
        	  				ds->updateCell(ix,iy,1);
        				else
        	  				ds->updateCell(ix,iy,-1);
      				}
						
			}
			*/
			ROS_INFO("Dstar planner initialized successfully");
    			initialized_ = true;
	
		}
		else
			ROS_WARN("This planner has already been initialized... doing nothing");
	}

	int ROSplanner::plan(std::vector<geometry_msgs::PoseStamped>& grid_plan,
			      const geometry_msgs::PoseStamped& start,const geometry_msgs::PoseStamped& goal)
	{
		unsigned int start_mx;
		unsigned int start_my;
		double start_x = start.pose.position.x;
		double start_y = start.pose.position.y;

		costmap_->worldToMap(start_x,start_y,start_mx,start_my);
		ROS_DEBUG("update start point");
		ds->updateStart(start_mx,start_my);		
		
		unsigned int goal_mx;
		unsigned int goal_my;
		double goal_x = goal.pose.position.x;
		double goal_y = goal.pose.position.y;

		costmap_->worldToMap(goal_x,goal_y,goal_mx,goal_my);
		ROS_DEBUG("update start point");
		ds->updateGoal(goal_mx,goal_my);
		
		int nx_cell,ny_cell;
		nx_cell = costmap_->getSizeInCellsX();
		ny_cell = costmap_->getSizeInCellsY();

		unsigned char *grid = costmap_->getCharMap();
		for(int x=0; x<(int)costmap_->getSizeInCellsX(); x++)
		{
         		for(int y=0; y<(int)costmap_->getSizeInCellsY(); y++)
			{
            			int index = costmap_->getIndex(x,y);
        
            			double c = (double)grid[index];
            			if( c >= 128)
               				ds->updateCell(x, y, -1);
            			else if (c == costmap_2d::FREE_SPACE)
                			ds->updateCell(x, y, 1);
            			else
                			ds->updateCell(x, y, c);
    			}
  		}

		ds->replan();
		
		list<state> path_ = ds->getPath();
		grid_plan.clear();
		grid_plan.push_back(start);	
		
		double costmap_resolution = costmap_->getResolution();
		double origin_costmap_x = costmap_->getOriginX();
		double origin_costmap_y = costmap_->getOriginY();
		
		std::list<state>::const_iterator iterator;
		for(iterator = path_.begin();iterator!= path_.end();++iterator)
		{	
			state node = *iterator;
			geometry_msgs::PoseStamped next_node;
			next_node.header.seq = cnt_make_plan_;
			next_node.header.stamp = ros::Time::now();
			next_node.header.frame_id = costmap_frame_;
		
			next_node.pose.position.x = (node.x + 0.5) * costmap_resolution + origin_costmap_x;
			next_node.pose.position.y = (node.y + 0.5) * costmap_resolution + origin_costmap_y;

			grid_plan.push_back(next_node);
		} 
		
		if(path_.size() > 0)
		{
			publishPath(grid_plan);
			return true;	
		}
		else
		{
			return false;	
		}
	}	


	void ROSplanner::publishPath(const std::vector<geometry_msgs::PoseStamped> plan)
	{
		if(!initialized_)
		{
			ROS_ERROR("Planner not initialized");
		}
		
		nav_msgs::Path gui_path;
		gui_path.poses.resize(plan.size());
		
		if(!plan.empty())
		{
			gui_path.header.frame_id = plan[0].header.frame_id;
			gui_path.header.stamp = plan[0].header.stamp;
		}

		for(unsigned int i = 0; i < plan.size();++i)
		{
			gui_path.poses[i] = plan[i];
		}
		pub_path_.publish(gui_path);
	}

	bool ROSplanner::makePlan(const geometry_msgs::PoseStamped& start,
				  const geometry_msgs::PoseStamped& goal,
		                  std::vector<geometry_msgs::PoseStamped>& plan)
	{
		if (!initialized_)
  		{
    			ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
  			return false;
  		}
		ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y,
        		    goal.pose.position.x, goal.pose.position.y);
		plan.clear();
		
		std::vector<geometry_msgs::PoseStamped> grid_plan;
		geometry_msgs::PoseStamped s;
		//s = transformPose(start);
		
		ROS_DEBUG("Start to plan");
		if(this->plan(grid_plan,start,goal))
		{
			cnt_no_plan_ = 0;
			cnt_make_plan_++;
			
			for(size_t i = 0;i < grid_plan.size(); i++ )
			{
				geometry_msgs::PoseStamped posei;
				posei.header.seq = cnt_make_plan_;
				posei.header.stamp = ros::Time::now();
				posei.header.frame_id = costmap_frame_;
				posei.pose.position.x = grid_plan[i].pose.position.x;
				posei.pose.position.y = grid_plan[i].pose.position.y;
				posei.pose.position.z = 0;
				posei.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,grid_plan[i].pose.position.z);
				plan.push_back(posei);
			}
			ROS_DEBUG("Dstar path found");
			return true;

		}

		
		/*
	        if (goal.header.frame_id != costmap_ros_->getGlobalFrameID())
  		{
    			ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
              			costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
    			return false;
  		}
	
  		tf::Stamped < tf::Pose > goal_tf;
  		tf::Stamped < tf::Pose > start_tf;

  		poseStampedMsgToTF(goal, goal_tf);
  		poseStampedMsgToTF(start, start_tf);
		
		int startX = start.pose.position.x;
  		int startY = start.pose.position.y;
	
  		int goalX = goal.pose.position.x;
  		int goalY = goal.pose.position.y;
	
		ds->init(startX,startY,goalX,goalY);
		
		ds->replan();
		
		mypath = ds->getPath();
		if(mypath.size() > 0)
		{
			i = mypath.begin();
			while(i != mypath.end())
			{
				geometry_msgs::PoseStamped pose = goal;
				pose.pose.position.x = i->x;
				pose.pose.position.y = i->y;
				pose.pose.position.z = 0.0;
	
        			pose.pose.orientation.x = 0.0;
        			pose.pose.orientation.y = 0.0;
        			pose.pose.orientation.z = 0.0;
        			pose.pose.orientation.w = 1.0;
	
        			plan.push_back(pose);
				i++;
			}	
		
			float path_length = 0.0;
	
			std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin();
	
			geometry_msgs::PoseStamped last_pose;
			last_pose = *it;
			it++;
			for (; it!=plan.end(); ++it) {
	   			path_length += hypot(  (*it).pose.position.x - last_pose.pose.position.x, 
		                 (*it).pose.position.y - last_pose.pose.position.y );
	   			last_pose = *it;
			}
			cout <<"The global path length: "<< path_length<< " meters"<<endl;
      			//publish the plan
	
      			return true;
		}	
		*/
		else
		{
			cnt_no_plan_++;
			ROS_WARN("The planner failed to find a path, choose other goal position");
      			return false;
		}
	}	
};
