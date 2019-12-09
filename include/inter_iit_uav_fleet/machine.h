
#include <inter_iit_uav_fleet/callbacks.h>
#include <future>
#include <chrono>

#include <std_msgs/String.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/WaypointPull.h>

#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/back/mpl_graph_fsm_check.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/euml/common.hpp>
#include <boost/msm/front/euml/operator.hpp>

#define echo(X) std::cout << X << std::endl
#define sq(X) (X)*(X)

namespace state_machine
{
    namespace msm = boost::msm;
    namespace mpl = boost::mpl;

    // state machine commands
    struct CmdTakeOff{CmdTakeOff(){}};
    struct CmdExploring{CmdExploring(){}};
    struct CmdGotoLZ{CmdGotoLZ(){}};
    struct CmdLand{CmdLand(){}};
    struct CmdHover{CmdHover(){}};
    
    // state variables
    bool ContMission = true;
    bool AtLZ = false;

    bool terminator(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        ContMission = req.data;
        res.success = (ContMission == req.data);
        return true;
    }

    // state machine object
    struct fsm :  public msm::front::state_machine_def <fsm>
    {
        typedef msm::active_state_switch_before_transition active_state_switch_policy;

        template <class Event, class FSM> void on_entry(Event const&, FSM& ){if(verbose)   echo("Entered state machine");}
        template <class Event, class FSM> void on_exit(Event const &, FSM &){if(verbose)   echo("Exited state machine");}

        // state definitions

        struct Rest : public msm::front::state <>{
            template <class Event, class FSM> void on_entry(Event const &, FSM &){if(verbose)  echo("Entered Rest state");}
            template <class Event, class FSM> void on_exit(Event const &, FSM &){if(verbose)   echo("Exited Rest state");}
        };
        struct Hover : public msm::front::state<>{
            template <class Event, class FSM> void on_entry(Event const &, FSM &){if(verbose)  echo("Entered Hover mode");}
            template <class Event, class FSM> void on_exit(Event const &, FSM &){if(verbose)   echo("Exited Hover mode");}
        };
        struct Exploring : public msm::front::state<>
        {
            template <class Event, class FSM> void on_entry(Event const &, FSM &){if(verbose)  echo("Entered Exploring state");}
            template <class Event, class FSM> void on_exit(Event const &, FSM &){if(verbose)   echo("Exited Exploring state");}
        };
        struct ReachLZ : public msm::front::state<>{
            template <class Event, class FSM> void on_entry(Event const &, FSM &){if(verbose)  echo("Entered ReachingLZ state");}
            template <class Event, class FSM> void on_exit(Event const &, FSM &){if(verbose)   echo("Exited ReachingLZ state");}
        };

        // inital state declaration
        typedef Rest initial_state;

        // global node handle
        ros::NodeHandle nh;

        // publishers
        ros::Publisher command_pub_ = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
        ros::Publisher pose_pub_ = nh.advertise<inter_iit_uav_fleet::Poses>("objects", 1);

        // subscribers
        // ros::Subscriber utm_pose_sub_ = nh.subscribe("utm_pose", 1, utm_pose_cb_);
        ros::Subscriber mav_pose_sub_ = nh.subscribe("mavros/local_position/odom", 10, mav_pose_cb_);
        ros::Subscriber mission_wp_sub = nh.subscribe("mavros/mission/reached", 10, wp_reached_cb_);
        ros::Subscriber state_sub_ = nh.subscribe("mavros/state",1, state_cb_);

        // service clients
        ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
        ros::ServiceClient mission_client = nh.serviceClient<mavros_msgs::WaypointPull>("mavros/mission/pull");
        ros::ServiceClient detector_client = nh.serviceClient<inter_iit_uav_fleet::signal>("detector/terminate");

        ros::ServiceServer terminate_server = nh.advertiseService("planner/stop", terminator);

        // state transition functions

        void TakeOff(CmdTakeOff const& cmd)
        { 
            if(verbose)   echo(" Starting execution");

            ros::Rate loopRate(10);

            // utm_pose_.pose.position.z = -DBL_MAX;
            // if(verbose)   echo("  Waiting for UTM position");
            // while(utm_pose_.pose.position.z == -DBL_MAX){
            //     ros::spinOnce();
            //     loopRate.sleep();
            // }
            // if(verbose)   echo("  Received UTM position");

            if(verbose)   echo("  Waiting for odometry");
            mav_pose_.pose.pose.position.z = -DBL_MAX;
            while(mav_pose_.pose.pose.position.z == -DBL_MAX){
                ros::spinOnce();
                loopRate.sleep();
            }
            if(verbose)   echo("  Received odometry");
            home_pose_ = mav_pose_;

            return;
        }

        void Hovering(CmdHover const& cmd)
        {
            if(verbose)   echo(" Hovering");
            ros::Rate hoverRate(1.0/hover_time);
            hoverRate.sleep();  
        }

        void Explore(CmdExploring const& cmd)
        {
            if(verbose)   echo(" Exploring");
            ros::Rate loopRate(10);

            geometry_msgs::PoseStamped mission_msg;

            inter_iit_uav_fleet::signal start;
            start.request.signal = 1;
            if(detector_client.call(start) && start.response.success){
                if(verbose)   echo("  Started detector node");
            }

            mavros_msgs::WaypointPull req;
            int num_wp=0;
            while(num_wp == 0)
            {
                if(mission_client.call(req) && req.response.success){
                    num_wp = req.response.wp_received - 1;
                }
            }
            if(verbose)     echo("  Received waypoints");

            if(verbose)   echo("  Waiting for odometry");
            mav_pose_.pose.pose.position.z = -DBL_MAX;
            while(mav_pose_.pose.pose.position.z == -DBL_MAX){
                ros::spinOnce();
                loopRate.sleep();
            }
            if(verbose)   echo("  Received odometry");

            mavros_msgs::SetMode mission_set_mode, offb_set_mode;
            offb_set_mode.request.custom_mode = "OFFBOARD";
            mission_set_mode.request.custom_mode = "AUTO.MISSION";
            bool mode_set_= false;

            if(verbose)   echo("  Changing mode to Mission");
            while (!mode_set_){
                ros::spinOnce();
                if (set_mode_client.call(mission_set_mode) && mission_set_mode.response.mode_sent){
                    if(verbose)   echo("   Mission enabled");
                    mode_set_=true;
                }
                loopRate.sleep();
            }
            if(verbose)   echo("  Changed mode to Mission");

            while(ContMission)
            {
                ros::spinOnce();
                loopRate.sleep();
            }
            ContMission = false;

            while (mode_set_){
                ros::spinOnce();

                mission_msg.header.stamp = ros::Time::now();
                mission_msg.pose.position = mav_pose_.pose.pose.position;
                
                command_pub_.publish(mission_msg);
                
                while(mav_mode_.mode == "AUTO.MISSION"){
                    if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) mode_set_ = false;
                    ros::spinOnce();
                }
                if(verbose)   echo("   Offboard enabled");
                return_pose_ = mav_pose_;

                loopRate.sleep();
            }
            if(verbose)   echo("  Switched to Offboard");

            return;
        }

        void GotoLZ(CmdGotoLZ const& cmd)
        {
            if(verbose)   echo(" Going to LZ");
            ros::Rate loopRate(10);

            geometry_msgs::PoseStamped mission_msg;

            inter_iit_uav_fleet::signal stop;
            stop.request.signal = 0;
            if(detector_client.call(stop) && stop.response.success){
                if(verbose)   echo("  detector node stopped");
            }

            // utm_pose_.pose.position.z = -DBL_MAX;
            // if(verbose)   echo("  Waiting for UTM position");
            // while(utm_pose_.pose.position.z == -DBL_MAX){
            //     ros::spinOnce();
            //     loopRate.sleep();
            // }
            // if(verbose)   echo("  Received UTM position");

            mav_pose_.pose.pose.position.z = -DBL_MAX;
            if(verbose)   echo("  Waiting for odometry");
            while(mav_pose_.pose.pose.position.z == -DBL_MAX){
                ros::spinOnce();
                loopRate.sleep();
            }

            mission_msg.header.stamp = ros::Time::now();
            mission_msg.pose.position.x = home_pose_.pose.pose.position.x;
            mission_msg.pose.position.y = home_pose_.pose.pose.position.y;
            mission_msg.pose.position.z = hover_height;

            if(verbose)   echo("  Home location: x = " << mission_msg.pose.position.x << ", y = " << mission_msg.pose.position.y);
            home_msg_ = mission_msg;

            command_pub_.publish(mission_msg);
            return;
        }

        void Landing(CmdLand const& cmd)
        {
            if(verbose)   echo(" Landing"); 
            ros::Rate loopRate(10);

            mavros_msgs::SetMode land_set_mode;
            bool mode_set_ = false, LandingDone = false;
            land_set_mode.request.custom_mode = "AUTO.LAND";
            geometry_msgs::PoseStamped mission_msg;

            if(verbose)   echo("  Waiting for odometry");
            mav_pose_.pose.pose.position.z = -DBL_MAX;
            while(mav_pose_.pose.pose.position.z == -DBL_MAX){
                ros::spinOnce();
                loopRate.sleep();
            }
            if(verbose)   echo("  Received odometry");

            if(verbose)   echo("  Landing initiated");
            int i = 0;
            while(!LandingDone){
                mission_msg.header.stamp = ros::Time::now();
                mission_msg.pose.position.x = mav_pose_.pose.pose.position.x;
                mission_msg.pose.position.y = mav_pose_.pose.pose.position.y;
                mission_msg.pose.position.z = mav_pose_.pose.pose.position.z - descent_step;
                command_pub_.publish(mission_msg);
                LandingDone = (mav_pose_.pose.pose.position.z > land_height) ? false : true;
                ros::spinOnce();
                loopRate.sleep();
            }
            
            while (!mode_set_)
            {
                ros::spinOnce();
                if (set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent)
                {
                    if (verbose)    echo("   Land enabled");
                    mode_set_ = true;
                }
                loopRate.sleep();
            }

            if(verbose) echo("  Changed mode to Land");
            if(verbose) echo("  Landing done");

            return;
        }

        // location check functions

        bool isAtLZ(ros::NodeHandle nh){
            double dist = 0;
            bool AtLoc = false;
            ros::Rate loopRate(10);

            if(verbose)   echo("  Waiting for odometry");
            mav_pose_.pose.pose.position.z = -DBL_MAX;
            while(mav_pose_.pose.pose.position.z == -DBL_MAX){
                ros::spinOnce();
                loopRate.sleep();
            }
            if(verbose)   echo("  Received odometry");

            if(verbose)   echo("  Enroute to LZ, please wait");
            while(!AtLoc){
                ros::spinOnce();
                dist = sq(mav_pose_.pose.pose.position.x - home_msg_.pose.position.x) + sq(mav_pose_.pose.pose.position.y - home_msg_.pose.position.y);
                AtLoc = (dist > sq(loc_error)) ? false : true;
                loopRate.sleep();
            }

            AtLZ = AtLoc;
            return AtLoc;
        }

        // transition guard functions

        bool ReachedLZ(CmdHover const & cmd){
            if(isAtLZ(nh)){if(verbose)   echo(" Reached LZ");}
            return AtLZ;
        }

        bool ExecMission(CmdExploring const &){
            if(ContMission){
                if(verbose)   echo(" Executing Mission");
                return true;
            }
            else{
                if(verbose)   echo(" Not executing Mission");
                return false;
            }
        }      
        
        template<class Event> bool StopMission(Event const &){
            if(!ContMission){
                if(verbose)   echo(" Not executing Mission");
                return true;
            }
            else{
                if(verbose)   echo(" Executing Mission");
                return false;
            }
        }

        // transition table

        struct transition_table : mpl::vector<
        
        //      Type        Start            Event            Next              Action				    Guard
        // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
                a_row<    Rest          ,  CmdTakeOff   ,  Hover         ,  &fsm::TakeOff                               >,
        // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
                  row<    Hover         ,  CmdLand      ,  Rest          ,  &fsm::Landing     ,  &fsm::StopMission      >,
                  row<    Hover         ,  CmdExploring ,  Exploring     ,  &fsm::Explore     ,  &fsm::ExecMission      >,
                  row<    Hover         ,  CmdGotoLZ    ,  ReachLZ       ,  &fsm::GotoLZ      ,  &fsm::StopMission      >,
        // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
                a_row<    Exploring     ,  CmdHover     ,  Hover         ,  &fsm::Hovering                              >,
        // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
                  row<    ReachLZ       ,  CmdHover     ,  Hover         ,  &fsm::Hovering    ,  &fsm::ReachedLZ        >
        // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
        
        >{};
    };

    // state machine back-end declaration
    typedef msm::back::state_machine<fsm> fsm_;

    // state list
    static char const *const state_names[] = {"Rest", "Hover", "Exploring", "ReachLZ"};

    // helper function -- output current state
    void echo_state(fsm_ const& msg){ if(verbose) echo("Current state -- " << state_names[msg.current_state()[0]]); }
    
    // state publisher
    void statePublish(ros::NodeHandle nh, fsm_ *fsm)
    {
        ros::Publisher statePub = nh.advertise<std_msgs::String>("curr_state", 10);
        ros::Rate loopRate(10);

        std_msgs::String msg;
        while(ros::ok()){
            msg.data = state_names[fsm->current_state()[0]];
            statePub.publish(msg);
            loopRate.sleep();
        }
    }
}