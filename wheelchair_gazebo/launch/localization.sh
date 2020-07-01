echo "Launch rosnode: "
echo "- qr_pose_sim "
rosrun test_ag qr_pose_sim
echo "- ukf_sim "
rosrun test_ag ukf_sim 
echo "- ukf_qr_sim "
rosrun test_ag ukf_qr_sim
