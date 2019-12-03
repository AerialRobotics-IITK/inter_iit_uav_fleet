#include <inter_iit_uav_fleet/machine.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh, ph("~");

    // get params
    nh.getParam("verbose", verbose);
    nh.getParam("error/local", loc_error);

    nh.getParam("height/hover", hover_height);
    nh.getParam("height/land", land_height);
    nh.getParam("height/step", descent_step);

    nh.getParam("delay/hover", hover_time);
    nh.getParam("delay/transition", transition_time);

    nh.getParam("numObjects", totalObjects);

    ros::Rate transitRate(1.0/transition_time);
    ros::Rate loopRate(10);
    
    state_machine::fsm_ machine;
    machine.start();
    auto state = std::async(std::launch::async, state_machine::statePublish, ph, &machine);

    machine.process_event(state_machine::CmdTakeOff());       
    if(verbose)   state_machine::echo_state(machine);
    
    // execution loop
    while(state_machine::ContMission)
    {
        transitRate.sleep();       
        machine.process_event(state_machine::CmdExploring());     
        if(verbose)    state_machine::echo_state(machine);
        
        transitRate.sleep();       
        machine.process_event(state_machine::CmdHover());         
        if(verbose)    state_machine::echo_state(machine);

        transitRate.sleep();       
        machine.process_event(state_machine::CmdGotoLZ());        
        if(verbose)   state_machine::echo_state(machine);
    
        transitRate.sleep();       
        machine.process_event(state_machine::CmdHover());         
        if(verbose)    state_machine::echo_state(machine);
    
        transitRate.sleep();       
        machine.process_event(state_machine::CmdLand());          
        if(verbose)    state_machine::echo_state(machine);
    }

    return 0;
}
