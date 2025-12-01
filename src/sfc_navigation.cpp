// sfc_naviigation is a whole task class and ros node for navigation using SFC-based planner
# include <ros/ros.h>

#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"

#include "utils/utils.h"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <Eigen/Dense>
#include <thread>

// Configuration
struct Config
    {
        // mapping parameters
        std::string mapTopic;
        std::string targetTopic;
        std::string mavros_stateTopic;
        std::string odomTopic;
        double voxelWidth;
        std::vector<double> mapBound;
        double dilateRadius;
        // vehicle parameters
        double vehicleMass;
        double gravAcc;
        double horizDrag;
        double vertDrag;
        double parasDrag;
        double speedEps;
        
        double DesHeight;

        Config(const ros::NodeHandle &nh_priv)
        {
            // load parameters from rosparam server
            nh_priv.param<std::string>("map_topic", mapTopic, "/voxel_map");
            nh_priv.param<std::string>("target_topic", targetTopic, "/move_base_simple/goal");
            nh_priv.param<std::string>("mavros_state_topic", mavros_stateTopic, "/mavros/setpoint_position/local");
            nh_priv.param<std::string>("odom_topic", odomTopic, "/mavros/local_position/odom");
            nh_priv.param<double>("voxel_width", voxelWidth, 0.5);
            nh_priv.param<std::vector<double>>("map_bound", mapBound, std::vector<double>({-10.0, 10.0, -10.0, 10.0, 0.0, 5.0}));
            nh_priv.param<double>("dilate_radius", dilateRadius, 1.0);
            nh_priv.param<double>("vehicle_mass", vehicleMass, 1.5);
            nh_priv.param<double>("grav_acc", gravAcc, 9.81);
            nh_priv.param<double>("horiz_drag", horizDrag, 0.1);
            nh_priv.param<double>("vert_drag", vertDrag, 0.2);
            nh_priv.param<double>("paras_drag", parasDrag, 0.01);
            nh_priv.param<double>("speed_eps", speedEps, 0.1);
            nh_priv.param<double>("des_height", DesHeight, 1.0);
        }
    };

class SFCNavigation
{
    private:
    Config config_;
    ros::NodeHandle nh_;
    ros::Subscriber mapSub_;
    ros::Subscriber targetSub_;
    ros::Subscriber mavros_stateSub_;
    ros::Subcriber odomSub_;
    ros::ServiceClient armClient_;
	ros::ServiceClient setModeClient_;
    ros::Publisher posePub_;
    ros::Publisher statePub_;

    ros::Timer stateUpdateTimer_;

    std::thread targetPubWorker_;

    voxel_map::VoxelMap voxelMap_;
    Eigen::Vector3d Goal_; // target position
    Eigen::Vector3d currPos_;

    geometry_msgs::PoseStamped poseTgt_;   // position control target

    nav_msgs::Odometry odom_;
    mavros_msgs::State mavrosState_;

    Eigen::Vector3d currVel_;
    Eigen::Vector3d currAcc_;
    ros::Time prevStateTime_;



    bool mapInitialized_;
    bool odomReceived_;
    bool mavrosStateReceived_;
    bool stateUpdateFirstTime_;

    // Subcriber callback functions
    inline void mapCB(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        // Initialize voxel map from point cloud
        if (!mapInitialized)
        {
            size_t cur = 0;
            const size_t total = msg->data.size() / msg->point_step;
            float *fdata = (float *)(&msg->data[0]);
            for (size_t i = 0; i < total; i++)
            {
                cur = msg->point_step / sizeof(float) * i;
                if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) ||
                    std::isnan(fdata[cur + 1]) || std::isinf(fdata[cur + 1]) ||
                    std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2]))
                {
                    continue;
                }
                voxelMap_.setOccupied(Eigen::Vector3d(fdata[cur + 0],fdata[cur + 1],fdata[cur + 2]));
            }
            voxelMap_.dilate(std::ceil(config.dilateRadius / voxelMap.getScale()));
            mapInitialized = true;
            ROS__INFO("[sfc_Nav]:Voxel map initialized.");
        }
    }
    
    inline void targetCB(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (mapInitialized)
        {
            if (Goal.empty())
            {
                const double zGoal = config_.DesHeight; // desired height
                Eigen::Vector3d target(msg->pose.position.x, msg->pose.position.y, zGoal);
                if (voxelMap_.query(target) == 0)
                {
                    Goal_ = target;
                    ROS_INFO("[sfc_Nav]:New target received: (%.2f, %.2f, %.2f).", Goal(0), Goal(1), Goal(2));
                    plan(); // TODO: implement plan function
                }
                else
                {
                    ROS_WARN("[sfc_Nav]:Target in occupied voxel, please choose another target.");
                }
            }
            else
            {
                ROS_WARN("[sfc_Nav]:Previous target not reached yet.");
            }
            
        }
        return;
    }

    inline void mavros_stateCB(const mavros_msgs::State::ConstPtr &msg)
    {
        mavrosState_ = *msg;
		if (not mavrosStateReceived_){
			mavrosStateReceived_ = true;
		}
    }

    inline void odomCB(const nav_msgs::Odometry::ConstPtr &msg)
    {
        odom_ = *odom;
		currPos_(0) = this->odom_.pose.pose.position.x;
		currPos_(1) = this->odom_.pose.pose.position.y;
		currPos_(2) = this->odom_.pose.pose.position.z;
		if (not odomReceived_){
			odomReceived_ = true;
		}
    }

    inline void stateUpdateCB(const ros::TimerEvent &e)
    {
        Eigen::Vector3d currVelBody (this->odom_.twist.twist.linear.x, this->odom_.twist.twist.linear.y, this->odom_.twist.twist.linear.z);
		Eigen::Vector4d orientationQuat (this->odom_.pose.pose.orientation.w, this->odom_.pose.pose.orientation.x, this->odom_.pose.pose.orientation.y, this->odom_.pose.pose.orientation.z);
        Eigen::Matrix3d orientationRot = AutoFlight::quat2RotMatrix(orientationQuat);
        this->currVel_ = orientationRot * currVelBody; // convert to world frame
        ros::Time currTime = ros::Time::now();	
		if (this->stateUpdateFirstTime_){
			this->currAcc_ = Eigen::Vector3d (0.0, 0.0, 0.0);
			this->prevStateTime_ = currTime;
			this->stateUpdateFirstTime_ = false;
		}
        else
        {
            double dt = (currTime - this->prevStateTime_).toSec();
			this->currAcc_ = (this->currVel_ - this->prevVel_)/dt; 
			this->prevVel_ = this->currVel_; 
			this->prevStateTime_ = currTime;
        }
    }

    inline void publishTarget()
    {
        /*
        isolated thread to publish target state
        */
        ros::Rate r200 (200);
		// warmup
		for(int i = 100; ros::ok() && i > 0; --i){
            poseTgt_.header.stamp = ros::Time::now();
            // poseTgt_.pose.position.x = currPos_(0);
            // poseTgt_.pose.position.y = currPos_(1);
            // poseTgt_.pose.position.z = currPos_(2);
            posePub_.publish(poseTgt_); // position control
        }
        // vehicle mode and arm commands
        mavros_msgs::SetMode offboardMode;
		offboardMode.request.custom_mode = "OFFBOARD";
		mavros_msgs::CommandBool armCmd;
        armCmd.request.value = true;
        ros::Time lastRequest = ros::Time::now();
        while (ros::ok()){
            //
			if (this->mavrosState_.mode != "OFFBOARD" && (ros::Time::now() - lastRequest > ros::Duration(5.0))){
                if (this->setModeClient_.call(offboardMode) && offboardMode.response.mode_sent)
                {
                    ROS_INFO("[sfc_Nav]: Offboard enabled.");
                }
                lastRequest = ros::Time::now();
            } else {
                if (!this->mavrosState_.armed && (ros::Time::now() - lastRequest > ros::Duration(5.0)))
                {
                    if (this->armClient_.call(armCmd) && armCmd.response.success){
                        ROS_INFO("[sfc_Nav]: Vehicle armed success.");
                    }
                    lastRequest = ros::Time::now();
                }
            }
            // state control
			this->statePub_.publish(this->stateTgt_);
			r200.sleep();
		}	
    }

    inline void updateTarget(const geometry_msgs::PoseStamped& position)
    {
        this->poseTgt_ = position;
        this->poseTgt_.header.frame_id = "map";
    }

    void flightBase::updateTargetWithState(const tracking_controller::Target& state){
		this->stateTgt_ = state;
	}

    public:
    // constructor
    SFCNavigation(const Config &conf, ros::NodeHandle &nh): config_(conf), nh_(nh)
    {
    // Initialize voxel map
    const Eigen::Vector3i xyz((config_.mapBound[1] - config_.mapBound[0]) / config_.voxelWidth,
                            (config_.mapBound[3] - config_.mapBound[2]) / config_.voxelWidth,
                            (config_.mapBound[5] - config_.mapBound[4]) / config_.voxelWidth);
    const Eigen::Vector3d offset(config_.mapBound[0], config_.mapBound[2], config_.mapBound[4]);
    voxelMap = voxel_map::VoxelMap(xyz, offset, config_.voxelWidth);

    // Subcribers
    mapSub_ = nh_.subscribe(config_.mapTopic, 1, &SFCNavigation::mapCB, this, ros::TransportHints().tcpNoDelay());
    targetSub_ = nh_.subscribe(config_.targetTopic, 1, &SFCNavigation::targetCB, this, ros::TransportHints().tcpNoDelay());
    mavros_stateSub_ = nh_.subscribe<mavros_msgs::State>(config_.mavros_stateTopic, 1000, &SFCNavigation::mavros_stateCB, this);
    odomSub_ = nh_.subscribe<nav_msgs::Odometry>(config_.odomTopic, 1000, &SFCNavigation::odomCB, this);
    // Service client
    armClient_ = this->nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    setModeClient_ = this->nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");	
    // Publishers
    posePub_ = this->nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1000);
	statePub_ = this->nh_.advertise<tracking_controller::Target>("/autonomous_flight/target_state", 1000);
    
    // Wait for odometry and mavros to be ready
    this->odomReceived_ = false;
    this->mavrosStateReceived_ = false;
    ROS_INFO("[sfc_Nav]:Waiting for odometry and mavros state...");
    ros::Rate r10(10);
    while (ros::ok() and not (this->odomReceived_ and this->mavrosStateReceived_)){
        ros::spinOnce();
        r10.sleep();
    }
    ROS_INFO("[sfc_Nav]:Odom and mavros state received.");
    // Tareget publish thread
    targetPubWorker_ = std::thread(&SFCNavigation::publishTarget, this);
    targetPubWorker_.detach();
    // Vehicle state update timer
    stateUpdateTimer_ = this->nh_.createTimer(ros::Duration(0.033), &SFCNavigation::stateUpdateCB, this);
    }

    inline ~SFCNavigation()
    {}

    inline void takeoff(){
        geometry_msgs::PoseStamped ps;
		ps.header.frame_id = "map";
		ps.header.stamp = ros::Time::now();
		ps.pose.position.x = odom_.pose.pose.position.x;
		ps.pose.position.y = odom_.pose.pose.position.y;
        ps.pose.position.z = config_.DesHeight;
        ps.pose.orientation = odom_.pose.pose.orientation;
        updateTarget(ps);

        tracking_controller::Target psT;
		psT.type_mask = psT.IGNORE_ACC_VEL;
		psT.header.frame_id = "map";
		psT.header.stamp = ros::Time::now();
		psT.position.x = this->odom_.pose.pose.position.x;
		psT.position.y = this->odom_.pose.pose.position.y;
		psT.position.z = this->takeoffHgt_;
		psT.yaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		updateTargetWithState(psT);

        ROS_INFO("[sfc_Nav]:Start taking off to height %.2f m.", config_.DesHeight);
        ros::Rate r30 (30);
		while (ros::ok() and std::abs(this->odom_.pose.pose.position.z - this->takeoffHgt_) >= 0.1){
			ros::spinOnce();
			r30.sleep();
		}
        ros::Time startTime = ros::Time::now();
        // hold for 3 seconds
		while (ros::ok()){
			ros::Time currTime = ros::Time::now();
			if ((currTime - startTime).toSec() >= 3){
				break;
			}
			ros::spinOnce();
			r30.sleep();
		}
        ROS_INFO("[sfc_Nav]:Takeoff success.");
    }


}

int main(int argc, char **argv)
{
    // ROS node initialization
    ros::init(argc, argv, "sfc_navigation_node");
    ros::NodeHandle nh_;
    // private node handle
    ros::NodeHandle nh_priv("~");
    // Instantiate SFCNavigation class
    SFCNavigation sfc_navigation(Config(ros::NodeHandle("~")), nh_);
    sfc_navigation.takeoff();
    ros::Rate lr(1000);
    while (ros::ok())
    {
        ros::spinOnce();
        lr.sleep();
    }
    return 0;
}