<!-- 


### steps to collect data 

turn the robot on (long press wait for green light and let go)

after the gripper is setting up its ready to go 

set up the wire on the robot gripper using the remote
    - R1 lock robot 
    - L1 release lock 
    this R1 + L1 sequence can regain control of the robot if moveit is on 

    - R/L2 open and close gripper  

open 4 terminals 


run these bash commands by order 

1. launch_kinova 

after rviz is opened check for the planning group **manipulator** pllaner id should be **LIN** in the **pilz_industrial_motion_planner**

set the goal to Ready and press plan + execute  if it fails somtimes chosing a colse by pose could help 

2. launch_data_collection 
check that all is started and 4 refs are published 

set up the wire in the fixed gripper by pressing setup botton in the robotiq panel on rviz (it will open and close slowly possition the wire rufly on the midlle)

3. rqt 
check the 4 DIGIT image topics under /digit

set up 5 befor you run 4 and run it right after 

4. 
cd ~/ws_moveit
setupr2
ros2 run 
ros2 run moveit_cpp_demo plan_to_goal


5. ros2 launch data_collection_bringup record.launch.py name:=<suffix> duration:=<seconds>

 -->



# Data Collection Procedure

## 1. Power On the Robot

* Turn on the robot by long-pressing the power button.
* Wait for the green light, then release.
* After initialization the gripper sets up automatically; once finished, the robot is ready.

## 2. Prepare the Wire on the Robot Gripper (using the remote)

* R1: lock robot
* L1: release lock
  (R1 + L1 together can regain control if MoveIt is active)
* R2 / L2: open and close the Robotiq gripper
  Use these controls to place the wire inside the robot gripper.
* The sques botton in the center of the remote changesthe teleoperation mode. Teleoperation is done using the joysticks  

## 3. Open Four Terminals

You will use four separate terminals for the following steps.

## 4. Terminal 1 — Launch Kinova

Command:
launch_kinova

After RViz opens:

* Make sure motion planning display checkmark is true
* In the Planning Group, choose: manipulator
* Check "Use Cartesian Path" if not checked
* In motion planning the context tab planner should be: LIN (Pilz Industrial Motion Planner)
* Set the goal to “Ready” and click Plan + Execute
  (if it fails, choose a nearby pose and try again)

## 5. NatNet Activation

* Open Anydesk and connect to the last computer (leftmost) in the recent connections (1 711 845 694)
Or physically walk 5 steps to the computer.
* Type "Motive" in the Windows search bar and activate.

## 6. Terminal 2 — Launch Data Collection

Command:
launch_data_collection

Check:

* All nodes launched correctly
* Four reference images/frames are published

Then in RViz:

* Open the Robotiq panel
* Press the Setup button (the gripper will open/close slowly)
* Place the wire roughly in the middle of the fixed gripper

## 7. Terminal 3 — RQT Image Check

Command:

r2

rqt

Verify:

* Under the `/digit` namespace, confirm all 4 live DIGIT image topics appear correctly:
  `/digit/D21118/image_raw`, `/digit/D21122/image_raw`,
  `/digit/D21123/image_raw`, `/digit/D21124/image_raw`
* Check that the matching reference images are also available on `/digit/*/ref_image`


## 8. Terminal 4 — MoveIt CPP Demo

Commands:

cd ~/ws_moveit 

setupr2

ros2 launch moveit_cpp_demo plan_to_goal

## 9. Record a Bag File

Command:
`ros2 launch data_collection_bringup record.launch.py name:=<suffix> duration:=<seconds>`

Where:

* `name`: Descriptive suffix (file will be `<timestamp>_<suffix>`)
* `duration`: Recording length in seconds (default: 600)
* `rate_hz`: (Optional) Throttling frequency (default: 10)

This launch file records throttled versions of the active publisher topics:

* `/digit/D21118/image_raw_throttled`
* `/digit/D21122/image_raw_throttled`
* `/digit/D21123/image_raw_throttled`
* `/digit/D21124/image_raw_throttled`
* `/joint_states_throttled`
* `/natnet/unlabeled_marker_data_throttled`
* `/natnet/fixed_ee_pose_throttled`
* `/end_effector_pose_throttled`

---

If you want, I can refine formatting, add troubleshooting notes, or generate a more compact version.




## 10. check the bag file existe under


`/home/rotem/data_collection/recordings`
