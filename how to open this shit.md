roslaunch apollo_gazebo apollo_empty.launch            #3shan nsha8al el gazebo m3 el world
roslaunch apollo_navigation move_base.launch		#3shan nsha8al el navegation 
rosrun tf2_ros static_transform_plisher 0 0 0 0 0 0 map odom  #3shan lw mfesh amcl m7tot ya5od mn el odom 3alatol	
roslaunch husky_navigation amcl.launch                       #ysha8al el amcl bta3 el husky (lw sha8alt da elle fo2 la)
rviz								#rviz
rqt								#for dynamic comfigration
rosrun apollo_navigation clear_on_global.py  #3shan nsha8al elmankash bta3 eno y3mel clear l opestical layer fe el global bd ma ywsal llgoal
