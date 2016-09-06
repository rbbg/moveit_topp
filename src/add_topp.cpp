#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <class_loader/class_loader.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <moveit_topp/moveit_topp.h>

namespace topp_planning_adapter
{

class AddTOPP : public planning_request_adapter::PlanningRequestAdapter
{
public:

  AddTOPP() : planning_request_adapter::PlanningRequestAdapter(), nh_("~")
  {
  }

  ~AddTOPP(){ };

  bool computeTimeStampsTOPP(robot_trajectory::RobotTrajectory& rob_trajectory,
						  const double max_velocity_scaling_factor = 1.0,
						  const double max_acceleration_scaling_factor = 1.0) const {
	  
	  const robot_model::JointModelGroup *group = rob_trajectory.getGroup();


	  // const std::vector<std::string> &vars = group->getVariableNames();
	  // const robot_model::RobotModel &rmodel = group->getParentModel();
	  
	  // ROS_INFO_STREAM("waypoints: " << rob_trajectory.getWayPointCount());

	  // double velocity_scaling_factor = 1.0;
	  // if (max_velocity_scaling_factor > 0.0 && max_velocity_scaling_factor <= 1.0)
	  //   velocity_scaling_factor = max_velocity_scaling_factor;
	  // else
	  //   if (max_velocity_scaling_factor == 0.0)
	  //     logDebug("A max_velocity_scaling_factor of 0.0 was specified, defaulting to %f instead.", velocity_scaling_factor);
	  //   else
	  //     logWarn("Invalid max_velocity_scaling_factor %f specified, defaulting to %f instead.", max_velocity_scaling_factor, velocity_scaling_factor);

	  // double acceleration_scaling_factor = 1.0;
	  // if (max_acceleration_scaling_factor > 0.0 && max_acceleration_scaling_factor <= 1.0)
	  //   acceleration_scaling_factor = max_acceleration_scaling_factor;
	  // else
	  //   if (max_acceleration_scaling_factor == 0.0)
	  //     logDebug("A max_acceleration_scaling_factor of 0.0 was specified, defaulting to %f instead.", acceleration_scaling_factor);
	  //   else
	  //     logWarn("Invalid max_acceleration_scaling_factor %f specified, defaulting to %f instead.", max_acceleration_scaling_factor, acceleration_scaling_factor);


	  // std::vector<double> vel_limits, acc_limits;
	  // for (std::size_t j = 0 ; j < vars.size() ; ++j)
	  // {
	  //   double v_max = 1.0;
	  //   double a_max = 1.0;
	  //   const robot_model::VariableBounds &b = rmodel.getVariableBounds(vars[j]);
	  //   if (b.velocity_bounded_)
	  // 	  v_max = std::min(fabs(b.max_velocity_* velocity_scaling_factor), fabs(b.min_velocity_* velocity_scaling_factor));
	  //      if (b.acceleration_bounded_)
	  //        a_max = std::min(fabs(b.max_acceleration_* acceleration_scaling_factor), fabs(b.min_acceleration_* acceleration_scaling_factor));
	  //      vel_limits.push_back(v_max);
	  //      acc_limits.push_back(a_max);
	  // }
	  // moveit_topp::MoveItTopp mtop(vel_limits, acc_limits);

	  moveit_topp::MoveItTopp mtop(nh_, group);

	  ROS_INFO("moveitTOPP was made!");
	  mtop.computeTimeStamps(rob_trajectory);

	  return true;
  }

  virtual std::string getDescription() const {return "Add TOPP";}

  virtual bool adaptAndPlan(const PlannerFn &planner,
                            const planning_scene::PlanningSceneConstPtr& planning_scene,
                            const planning_interface::MotionPlanRequest &req,
                            planning_interface::MotionPlanResponse &res,
                            std::vector<std::size_t> &added_path_index) const
  {
  	ROS_INFO("adaptAndPlan of TOPP plugin");
    bool result = planner(planning_scene, req, res);
    if (result && res.trajectory_)
    {
      ROS_DEBUG("Running '%s'", getDescription().c_str());
      if (!computeTimeStampsTOPP(*res.trajectory_, req.max_velocity_scaling_factor, req.max_acceleration_scaling_factor))
        ROS_WARN("TOPP for the solution path failed.");
    }
    ROS_INFO("returning result!");
    return result;
  }
private:
	ros::NodeHandle nh_;

};

}

CLASS_LOADER_REGISTER_CLASS(topp_planning_adapter::AddTOPP,
                            planning_request_adapter::PlanningRequestAdapter);
