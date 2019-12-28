#include <inter_iit_uav_fleet/machine.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planner");
  ros::NodeHandle nh, ph("~");

  // get params
  nh.getParam("is_verbose", is_verbose);
  nh.getParam("error/local", loc_error);

  nh.getParam("height/hover", hover_height);
  nh.getParam("height/land", land_height);
  nh.getParam("height/step", descent_step);

  nh.getParam("delay/hover", hover_time);
  nh.getParam("delay/transition", transition_time);

  nh.getParam("numObjects", totalObjects);

  ros::Rate transitRate(1.0 / transition_time);
  ros::Rate loopRate(10);

  // initialize dynamic reconfigure server
  dynamic_reconfigure::Server<inter_iit_uav_fleet::reconfigConfig> cfg_server;
  dynamic_reconfigure::Server<inter_iit_uav_fleet::reconfigConfig>::CallbackType call_f =
      boost::bind(&cfgCallback, _1, _2);
  cfg_server.setCallback(call_f);

  // declare and launch the FSM
  state_machine::fsm_ machine;
  machine.start();

  // helper thread that publishes current FSM state
  auto state = std::async(std::launch::async, state_machine::statePublish, ph, &machine);

  machine.process_event(state_machine::CmdTakeOff());
  if (is_verbose)
    state_machine::echo_state(machine);

  // execution loop
  while (state_machine::ContMission)
  {
    transitRate.sleep();
    machine.process_event(state_machine::CmdExploring());
    if (is_verbose)
      state_machine::echo_state(machine);
  }

  transitRate.sleep();
  machine.process_event(state_machine::CmdHover());
  if (is_verbose)
    state_machine::echo_state(machine);

  transitRate.sleep();
  machine.process_event(state_machine::CmdLand());
  if (is_verbose)
    state_machine::echo_state(machine);

  return 0;
}
