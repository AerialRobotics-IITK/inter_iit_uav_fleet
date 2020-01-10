tmux new -s inter_iit -d
tmux send-keys -t inter_iit 'roslaunch fkie_master_discovery master_discovery.launch' C-m
sleep 3

tmux split-window -v -t inter_iit
tmux send-keys -t inter_iit 'roslaunch fkie_master_sync master_sync.launch' C-m
sleep 3

tmux split-window -h -t inter_iit
tmux send-keys -t inter_iit 'main' C-m
sleep 3

tmux select-pane -t inter_iit:0.0
tmux split-window -h -t inter_iit
tmux send-keys -t inter_iit 'planner'

tmux attach -t inter_iit


 
