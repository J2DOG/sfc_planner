// sfc_naviigation is a whole task class and ros node for navigation using SFC-based planner
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <Eigen/Dense>
#include <thread>

#include <tracking_controller/Target.h>

#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"

#include "sfc_planner/mps_geo_utils.hpp"
#include "sfc_planner/mps.hpp"
#include "sfc_planner/mps_sfc_gen.hpp"

#include "utils/utils.h"

#include "visualizer/visualizer.hpp"

// Configuration
struct Config
    {
        // mapping parameters
        std::string mapTopic;
        std::string targetTopic;
        std::string mavros_stateTopic;
        std::string odomTopic;
        std::string obstaclePath;

        double voxelWidth;
        std::vector<double> mapBound;
        double dilateRadius;
        double desHeight;

        double timeoutRRT;

        // planning parameters uesd by gcopter
        // vehicle parameters
        double vehicleMass;
        double gravAcc;
        double horizDrag;
        double vertDrag;
        double parasDrag;
        double speedEps;

        double maxVelMag; // max velocity
        double maxBdrMag; // max body rate
        double maxTiltAngle; // max tilt angle
        double minThrust; // min thrust
        double maxThrust; // max thrust

        std::vector<double> chiVec;
        int integralIntervs;

        double weightT;
        double smoothingEps;
        double relCostTol;
        
        
        

        Config(const ros::NodeHandle &nh_priv)
        {
            // load parameters from rosparam server
            nh_priv.param<std::string>("map_topic", mapTopic, "/voxel_map");
            nh_priv.param<std::string>("target_topic", targetTopic, "/move_base_simple/goal");
            nh_priv.param<std::string>("mavros_state_topic", mavros_stateTopic, "/mavros/setpoint_position/local");
            nh_priv.param<std::string>("odom_topic", odomTopic, "/mavros/local_position/odom");
            nh_priv.param<std::string>("obstacle_path", obstaclePath, "/home/jtodog/catkin_ws/src/CERLAB-UAV-Autonomy/polys_mapgen/cfg/random_obstacles.yaml");

            nh_priv.param<double>("voxel_width", voxelWidth, 0.5);
            nh_priv.param<std::vector<double>>("map_bound", mapBound, std::vector<double>({-10.0, 10.0, -10.0, 10.0, 0.0, 5.0}));
            nh_priv.param<double>("dilate_radius", dilateRadius, 1.0);
            nh_priv.param<double>("des_height", desHeight, 1.0);

            nh_priv.param<double>("timeout_rrt", timeoutRRT, 0.1);

            nh_priv.param<double>("vehicle_mass", vehicleMass, 1.5);
            nh_priv.param<double>("grav_acc", gravAcc, 9.81);
            nh_priv.param<double>("horiz_drag", horizDrag, 0.1);
            nh_priv.param<double>("vert_drag", vertDrag, 0.2);
            nh_priv.param<double>("paras_drag", parasDrag, 0.01);
            nh_priv.param<double>("speed_eps", speedEps, 0.1);

            nh_priv.param<double>("max_vel_mag", maxVelMag, 5.0);
            nh_priv.param<double>("max_bdr_mag", maxBdrMag, 3.0);
            nh_priv.param<double>("max_tilt_angle", maxTiltAngle, 0.5);
            nh_priv.param<double>("min_thrust", minThrust, 2.0);
            nh_priv.param<double>("max_thrust", maxThrust, 15.0);  
            nh_priv.param<std::vector<double>>("chi_vec", chiVec, std::vector<double>({1.0, 1.0, 1.0, 1.0, 1.0}));
            nh_priv.param<int>("integral_intervs", integralIntervs, 10);
            nh_priv.param<double>("weight_t", weightT, 1.0);
            nh_priv.param<double>("smoothing_eps", smoothingEps, 1e-4);
            nh_priv.param<double>("rel_cost_tol", relCostTol, 1e-3);
            
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
    ros::Subscriber odomSub_;
    ros::ServiceClient armClient_;
	ros::ServiceClient setModeClient_;
    ros::Publisher posePub_;
    ros::Publisher statePub_;
    ros::Publisher history_pathPub_;

    ros::Timer stateUpdateTimer_;

    std::thread targetPubWorker_;

    voxel_map::VoxelMap voxelMap_;
    std::vector<Eigen::MatrixX4d> obstacles_;
    Eigen::Vector3d Goal_; // target position
    Eigen::Vector3d currPos_;
    const size_t max_history_size_ = 100;
    std::vector<geometry_msgs::PoseStamped> trajectory_history_;
    nav_msgs::Path historyPath_;

    tracking_controller::Target stateTgt_;
    geometry_msgs::PoseStamped poseTgt_;   // position control target

    nav_msgs::Odometry odom_;
    mavros_msgs::State mavrosState_;

    Eigen::Vector3d currVel_;
    Eigen::Vector3d prevVel_;
    Eigen::Vector3d currAcc_;
    ros::Time prevStateTime_;

    Trajectory<5> traj_;
    double trajStamp_;
    double DesYaw_;

    Visualizer visualizer_;



    bool mapInitialized_ = false;
    bool odomReceived_ = false;
    bool mavrosStateReceived_ = false;
    bool stateUpdateFirstTime_ = true;
    bool waitForGoal_ = false;
    bool holdingPosition_ = false;
    bool stateControl_ = false;
    bool trajReady_ = false;

    // Subcriber callback functions
    void mapCB(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        // Initialize voxel map from point cloud
        if (!mapInitialized_)
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
            voxelMap_.dilate(std::ceil(config_.dilateRadius / voxelMap_.getScale()));
            mapInitialized_ = true;
            ROS_INFO("[sfc_Nav]:Voxel map initialized.");
        }
    }
    
    void targetCB(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (mapInitialized_)
        {
            if (waitForGoal_)
            {
                const double zGoal = config_.desHeight; // desired height
                Eigen::Vector3d target(msg->pose.position.x, msg->pose.position.y, zGoal);
                if (voxelMap_.query(target) == 0)
                {
                    Goal_ = target;
                    ROS_INFO("[sfc_Nav]:New target accepted: (%.2f, %.2f, %.2f).", Goal_(0), Goal_(1), Goal_(2));
                    waitForGoal_ = false;
                    plan(); 
                }
                else
                {
                    ROS_WARN("[sfc_Nav]:Target is occupied, please choose another target.");
                }
            }
            else
            {
                ROS_WARN("[sfc_Nav]:Previous target not reached yet.");
            }
            
        }
        else{
            ROS_WARN("[sfc_Nav]:Map has not been initialized yet.");
        }
        return;
    }

    void mavros_stateCB(const mavros_msgs::State::ConstPtr &msg)
    {
        mavrosState_ = *msg;
		if (not mavrosStateReceived_){
			mavrosStateReceived_ = true;
		}
    }

    void odomCB(const nav_msgs::Odometry::ConstPtr &odom)
    {
        odom_ = *odom;
		currPos_(0) = odom_.pose.pose.position.x;
		currPos_(1) = odom_.pose.pose.position.y;
		currPos_(2) = odom_.pose.pose.position.z;
		if (not odomReceived_){
			odomReceived_ = true;
		}
    }

    void stateUpdateCB(const ros::TimerEvent &e)
    {
        Eigen::Vector3d currVelBody (odom_.twist.twist.linear.x, odom_.twist.twist.linear.y, odom_.twist.twist.linear.z);
		Eigen::Vector4d orientationQuat (odom_.pose.pose.orientation.w, odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y, odom_.pose.pose.orientation.z);
        Eigen::Matrix3d orientationRot = AutoFlight::quat2RotMatrix(orientationQuat);
        currVel_ = orientationRot * currVelBody; // convert to world frame
        ros::Time currTime = ros::Time::now();	
		if (stateUpdateFirstTime_){
			currAcc_ = Eigen::Vector3d (0.0, 0.0, 0.0);
			prevStateTime_ = currTime;
            prevVel_ = currVel_;
			stateUpdateFirstTime_ = false;
		}
        else
        {
            double dt = (currTime - prevStateTime_).toSec();
			currAcc_ = (currVel_ - prevVel_)/dt; 
			prevVel_ = currVel_; 
			prevStateTime_ = currTime;
        }

        // update the history traj
        geometry_msgs::PoseStamped pose_stamped;
        
        pose_stamped.header = odom_.header;
        pose_stamped.pose = odom_.pose.pose;
        trajectory_history_.push_back(pose_stamped);
        if (trajectory_history_.size() > max_history_size_) 
        {
            trajectory_history_.erase(trajectory_history_.begin());
        }
        // visual history path
        PublishHistoryPath();
    }

    void publishTarget()
    {
        /*
        isolated thread to publish target. 
        */
        ros::Rate r200 (200);
		// warmup
		for(int i = 100; ros::ok() && i > 0; --i){
            poseTgt_.header.stamp = ros::Time::now();
            poseTgt_.pose.position.x = currPos_(0);
            poseTgt_.pose.position.y = currPos_(1);
            poseTgt_.pose.position.z = currPos_(2);
            posePub_.publish(poseTgt_); // position control to warm up
        }

        // vehicle mode and arm commands
        mavros_msgs::SetMode offboardMode;
		offboardMode.request.custom_mode = "OFFBOARD";
		mavros_msgs::CommandBool armCmd;
        armCmd.request.value = true;
        ros::Time lastRequest = ros::Time::now();
        while (ros::ok()){
            //
			if (mavrosState_.mode != "OFFBOARD" && (ros::Time::now() - lastRequest > ros::Duration(5.0))){
                if (setModeClient_.call(offboardMode) && offboardMode.response.mode_sent)
                {
                    ROS_INFO("[sfc_Nav]: Offboard enabled.");
                }
                lastRequest = ros::Time::now();
            } else {
                if (!mavrosState_.armed && (ros::Time::now() - lastRequest > ros::Duration(5.0)))
                {
                    if (armClient_.call(armCmd) && armCmd.response.success){
                        ROS_INFO("[sfc_Nav]: Vehicle armed success.");
                    }
                    lastRequest = ros::Time::now();
                }
            }
            // state control
            if (stateControl_)
            {
                statePub_.publish(stateTgt_);
            } 
            else
            {
                posePub_.publish(poseTgt_);
            }
			r200.sleep();
		}	
    }

    void updateTarget(const geometry_msgs::PoseStamped& position)
    {
        poseTgt_ = position;
        poseTgt_.header.frame_id = "map";
    }

    void updateTargetWithState(const tracking_controller::Target& state)
    {
		stateTgt_ = state;
	}

    void PublishHistoryPath() {
        if (trajectory_history_.empty()) return;
        
        historyPath_.header.frame_id = trajectory_history_.front().header.frame_id;
        historyPath_.header.stamp = ros::Time::now();
        historyPath_.poses = trajectory_history_; 
        history_pathPub_.publish(historyPath_);
    } 

    void plan()
    {
        
        std::vector<Eigen::Vector3d> route;
        Eigen::Vector3d start = currPos_;
        Eigen::Vector3d goal = Goal_;
        double timeout = 0.01;
        double cost;
        // plan path
        cost = mps_sfc_gen::planPath<voxel_map::VoxelMap>(start,
                                                goal,
                                                voxelMap_.getOrigin(),
                                                voxelMap_.getCorner(),
                                                &voxelMap_, timeout,
                                                route);
        if (cost == INFINITY)
        {
            ROS_WARN("[sfc_Nav]:Failed to plan path.");
            waitForGoal_ = true;
            return;
        }
        else{
            ROS_INFO("[sfc_Nav]: RRT Plan length: %ld.",route.size());
        }
        std::vector<Eigen::MatrixX4d> hPolys; //sfc

        // std::vector<Eigen::Vector3d> pc;
        // // get surface point cloud
        // voxelMap_.getSurf(pc);
        // sfc_gen::convexCover(route,
        //                         pc,
        //                         voxelMap_.getOrigin(),
        //                         voxelMap_.getCorner(),
        //                         5.0, // progress
        //                         3.0, // range
        //                         hPolys);
        // sfc_gen::shortCut(hPolys);
        Eigen::Matrix<double, 6, 4> meta_poly = Eigen::Matrix<double, 6, 4>::Zero();
        meta_poly(0, 0) = 1.0;
        meta_poly(1, 0) = -1.0;
        meta_poly(2, 1) = 1.0;
        meta_poly(3, 1) = -1.0;
        meta_poly(4, 2) = 1.0;
        meta_poly(5, 2) = -1.0;

        meta_poly(0, 3) = -2.0;
        meta_poly(1, 3) = -2.0;
        meta_poly(2, 3) = -2.0;
        meta_poly(3, 3) = -2.0;
        meta_poly(4, 3) = -2.0;
        meta_poly(5, 3) = -2.0;
        const double d_min = 0.1;
        ROS_INFO("[sfc_gen]: convexCovering...");
        mps_sfc_gen::convexCover(route,
                                obstacles_,
                                voxelMap_.getOrigin(),
                                voxelMap_.getCorner(),
                                meta_poly,
                                d_min,
                                hPolys);
        ROS_INFO("[sfc_gen]: hPolys length: %ld", hPolys.size());
        if (route.size() > 1)
        {
            visualizer_.visualizePolytope(hPolys);
            Eigen::Matrix3d iniState;
            Eigen::Matrix3d finState;
            iniState << route.front(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(); // position, velocity, acceleration
            finState << route.back(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
            gcopter::GCOPTER_PolytopeSFC gcopter;

            Eigen::VectorXd magnitudeBounds(5);
            Eigen::VectorXd penaltyWeights(5);
            Eigen::VectorXd physicalParams(6);
            // set magnitude bounds
            magnitudeBounds(0) = config_.maxVelMag; // max velocity
            magnitudeBounds(1) = config_.maxBdrMag; // max body rate
            magnitudeBounds(2) = config_.maxTiltAngle; // max tilt angle
            magnitudeBounds(3) = config_.minThrust; // min thrust
            magnitudeBounds(4) = config_.maxThrust; // max thrust
            // set penalty weights
            penaltyWeights(0) = (config_.chiVec)[0]; 
            penaltyWeights(1) = (config_.chiVec)[1];
            penaltyWeights(2) = (config_.chiVec)[2];
            penaltyWeights(3) = (config_.chiVec)[3];
            penaltyWeights(4) = (config_.chiVec)[4];
            // set physical parameters
            physicalParams(0) = config_.vehicleMass; // vehicle mass
            physicalParams(1) = config_.gravAcc; // gravitational acceleration
            physicalParams(2) = config_.horizDrag; // horizontal drag
            physicalParams(3) = config_.vertDrag; // vertical drag
            physicalParams(4) = config_.parasDrag; // parasitic drag
            physicalParams(5) = config_.speedEps; // speed epsilon
            const int quadratureRes = config_.integralIntervs;

            traj_.clear();

            if (!gcopter.setup(config_.weightT,
                iniState, finState,
                hPolys, INFINITY,
                config_.smoothingEps,
                quadratureRes,
                magnitudeBounds,
                penaltyWeights,
                physicalParams))
            {
                ROS_WARN("[sfc_Nav]: GCOPTER setup failed!");
                return;
            }
            ROS_INFO("[sfc_Nav]: GCOPTER optimizing!");
            if (std::isinf(gcopter.optimize(traj_, config_.relCostTol)))
            {
                return;
            }
            if (traj_.getPieceNum() > 0)
            {
                trajStamp_ = ros::Time::now().toSec();
                visualizer_.visualize(traj_, route);
                trajReady_ = true;
                stateControl_ = true;
                ROS_INFO("[sfc_Nav]: traj_ generate success!");
            }
        }
    }
    public:

    // constructor
    SFCNavigation(const Config &conf, ros::NodeHandle &nh): 
    config_(conf), nh_(nh), visualizer_(nh_)
    {
    // Initialize voxel map
    const Eigen::Vector3i xyz((config_.mapBound[1] - config_.mapBound[0]) / config_.voxelWidth,
                            (config_.mapBound[3] - config_.mapBound[2]) / config_.voxelWidth,
                            (config_.mapBound[5] - config_.mapBound[4]) / config_.voxelWidth);
    const Eigen::Vector3d offset(config_.mapBound[0], config_.mapBound[2], config_.mapBound[4]);
    voxelMap_ = voxel_map::VoxelMap(xyz, offset, config_.voxelWidth);
    // load  obstacles in Hrep
    obstacles_ = mps_geo_utils::loadObstaclesFromYAML(config_.obstaclePath);
    // Subcribers
    mapSub_ = nh_.subscribe(config_.mapTopic, 1, &SFCNavigation::mapCB, this, ros::TransportHints().tcpNoDelay());
    targetSub_ = nh_.subscribe(config_.targetTopic, 1, &SFCNavigation::targetCB, this, ros::TransportHints().tcpNoDelay());
    mavros_stateSub_ = nh_.subscribe<mavros_msgs::State>(config_.mavros_stateTopic, 1000, &SFCNavigation::mavros_stateCB, this);
    odomSub_ = nh_.subscribe<nav_msgs::Odometry>(config_.odomTopic, 1000, &SFCNavigation::odomCB, this);
    // Service client
    armClient_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    setModeClient_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");	
    // Publishers
    posePub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1000);
	statePub_ = nh_.advertise<tracking_controller::Target>("/autonomous_flight/target_state", 1000);
    history_pathPub_ = nh_.advertise<nav_msgs::Path>("/visualizer/history_path", 10);
    // Wait for odometry and mavros to be ready
    odomReceived_ = false;
    mavrosStateReceived_ = false;
    ROS_INFO("[sfc_Nav]:Waiting for odometry and mavros state...");
    ros::Rate r10(10);
    while (ros::ok() and not (odomReceived_ and mavrosStateReceived_)){
        ros::spinOnce();
        r10.sleep();
    }
    DesYaw_ = AutoFlight::rpy_from_quaternion(odom_.pose.pose.orientation);
    ROS_INFO("[sfc_Nav]:Odom and mavros state received.");
    // Tareget publish thread
    targetPubWorker_ = std::thread(&SFCNavigation::publishTarget, this);
    targetPubWorker_.detach();
    // Vehicle state update timer
    stateUpdateTimer_ = nh_.createTimer(ros::Duration(0.033), &SFCNavigation::stateUpdateCB, this);
    }

    // takeoff function
    void takeoff()
    {
        // wait for map initialization
        ROS_INFO("[sfc_Nav]:takeoff is waiting for map initialization...");
        ros::Rate r5 (5);
        while (ros::ok() and not mapInitialized_){
            ros::spinOnce();
            r5.sleep();
        }
        geometry_msgs::PoseStamped ps;
		ps.header.frame_id = "map";
		ps.header.stamp = ros::Time::now();
		ps.pose.position.x = odom_.pose.pose.position.x;
		ps.pose.position.y = odom_.pose.pose.position.y;
        ps.pose.position.z = config_.desHeight;
        ps.pose.orientation = odom_.pose.pose.orientation;
        updateTarget(ps); // position control target update

        // tracking_controller::Target psT;
		// psT.type_mask = tracking_controller::Target::IGNORE_ACC_VEL;
		// psT.header.frame_id = "map";
		// psT.header.stamp = ros::Time::now();
		// psT.position.x = odom_.pose.pose.position.x;
		// psT.position.y = odom_.pose.pose.position.y;
		// psT.position.z = config_.desHeight;
		// psT.yaw = DesYaw_;
		// updateTargetWithState(psT);

        ROS_INFO("[sfc_Nav]:Start taking off to height %.2f m.", config_.desHeight);
        stateControl_ = false; // position control to take off
        ros::Rate r30 (30);
		while (ros::ok() && std::abs(odom_.pose.pose.position.z - config_.desHeight) >= 0.05){
            std::cout << "\r[sfc_Nav]: Current Height: " 
            << std::fixed << std::setprecision(2) 
            << odom_.pose.pose.position.z << " (m)"
            << std::flush;
			ros::spinOnce();
			r30.sleep();
		}
        std::cout << std::endl;
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
        ROS_INFO("[sfc_Nav]:Takeoff success, waiting for goal...");
        waitForGoal_ = true;
    }
    
    void process()
    {
        Eigen::VectorXd physicalParams(6);
        physicalParams(0) = config_.vehicleMass;
        physicalParams(1) = config_.gravAcc;
        physicalParams(2) = config_.horizDrag;
        physicalParams(3) = config_.vertDrag;
        physicalParams(4) = config_.parasDrag;
        physicalParams(5) = config_.speedEps;
        flatness::FlatnessMap flatmap;
        flatmap.reset(physicalParams(0), physicalParams(1), physicalParams(2),
                    physicalParams(3), physicalParams(4), physicalParams(5));
        if (traj_.getPieceNum() > 0 && trajReady_)
        {
            const double delta = ros::Time::now().toSec() - trajStamp_; 
            if (delta > 0.0 && delta < traj_.getTotalDuration())
            {
                // flatmap.forward(traj_.getVel(delta),
                //                 traj_.getAcc(delta),
                //                 traj_.getJer(delta),
                //                 0.0, 0.0,
                //                 thr, quat, omg);
                Eigen::Vector3d pos = traj_.getPos(delta);
                Eigen::Vector3d vel = traj_.getVel(delta);
                Eigen::Vector3d acc = traj_.getAcc(delta);
                tracking_controller::Target stateTarget;
                stateTarget.header.frame_id = "map";
                stateTarget.header.stamp = ros::Time::now();
                stateTarget.position.x = pos(0);
                stateTarget.position.y = pos(1);
                stateTarget.position.z = pos(2);
                stateTarget.velocity.x = vel(0);
                stateTarget.velocity.y = vel(1);
                stateTarget.velocity.z = vel(2);
                stateTarget.acceleration.x = acc(0);
                stateTarget.acceleration.y = acc(1);
                stateTarget.acceleration.z = acc(2);
                stateTarget.yaw = DesYaw_; // Extract heading angle from quaternion representation of attitude
                updateTargetWithState(stateTarget);
                // Terminal visual
                Eigen::Vector3d posErr;
                posErr(0) = odom_.pose.pose.position.x - pos(0);
                posErr(1) = odom_.pose.pose.position.y - pos(1);
                posErr(2) = odom_.pose.pose.position.z - pos(2);
                std::cout << "\r[sfc_Nav]: traj_time: " << std::fixed << std::setprecision(3) << delta << " s, "
                << "Position error: " << posErr.norm() << " m" << std::flush;
            }
            else if (delta >= traj_.getTotalDuration())
            {
                std::cout << std::endl;
                waitForGoal_ = true;
                trajReady_ = false;
                ROS_INFO("[sfc_Nav]:Traj finish.");
            }
        }
    }
    
};



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
        sfc_navigation.process();
        ros::spinOnce();
        lr.sleep();
    }
    return 0;
}