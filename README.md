# MR_project

+) Implement the rendez-vous of differential drive robots

  version 1 (after accordingly modifying turtles.launch and main.launch as specified in the respective files):
  
    `roslaunch mr_project main.launch`
    
    `rosrun mr_project rendezvous.py`
    
  version 2 (after accordingly modifying turtles.launch and main.launch as specified in the respective files): 
  
    `roslaunch mr_project main.launch` 

+) Implement formation travelling w/ obstacle avoidance

  run the simulation world:
  
    `roslaunch mr_project main.launch`
    
  in two new shells run respectively the server and the client needed to implement the ROS action:
  
    `rosrun mr_project group_action_server.py`
    
    `rosrun mr_project group_action_client.py`

+) Implement leader-follower dynamic: the leader is responsible for the path planning while the others follow 
  
  Open three terminal and run in each one respectively the following commands:
    
    `roslaunch mr_project multi_spawn.launch`
  
    `roslaunch mr_project multi_navigation.launch`
  
    `rosrun mr_project follower.py`
