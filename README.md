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
check the 4 images under full_data topic

set up 5 befor you run 4 and run it right after 

4. 
cd ~/ws_moveit
setupr2
ros2 run 
ros2 launch moveit_cpp_demo plan_to_gaol.launch.py --show-args
Arguments (pass arguments as '<name>:=<value>'):

    'setup_gripper':
        Whether to setup the gripper.
        (default: 'true')

    'test_random_moves':
        Whether to test random moves.
        (default: 'false')

    'go_to_ready':
        Whether to go to ready position on start.
        (default: 'true')

ros2 launch moveit_cpp_demo plan_to_gaol.launch.py setup_gripper:=false test_random_moves:=true go_to_ready:=false



5. ros2 launch data_collection_bringup record.launch.py name:=<H_M_d_m> duration:=<duration_sec>
Arguments (pass arguments as '<name>:=<value>'):

    'name':
        Name of the rosbag to record to
        (default: '')

    'duration':
        Duration to record the rosbag in seconds
        (default: '600.0')
    

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

## 7. Terminal 3 — RQT Topic Check

Command:

r2

rqt

Verify:

* Under `/digit/<serial>/image_raw`, confirm all 4 DIGIT camera images appear correctly
* Under `/digit/<serial>/ref_image`, confirm all 4 reference images are available
* Confirm `/natnet/fixed_ee_pose` and `/end_effector_pose` are updating
* `full_data_pub` is currently not part of the standard bringup flow


## 8. Terminal 4 — MoveIt CPP Demo

Commands:

cd ~/ws_moveit 

setupr2

ros2 launch moveit_cpp_demo plan_to_goal

## 9. Record a Bag File

Command:

`ros2 launch data_collection_bringup record.launch.py name:=<H_M_d_m> duration:=<duration_sec> [directory:=<output_dir>] [rate_hz:=<hz>]`

Example:

`ros2 launch data_collection_bringup record.launch.py name:=deg_0_0 duration:=300  rate_hz:=20.0`

Arguments:

* `name`: suffix for the bag name
  Default: `""`
* `duration`: recording duration in seconds
  Default: `600.0`
* `directory`: output directory for the bag
  Default: `/home/rotem/data_collection/recordings/`
* `rate_hz`: throttle rate applied before recording
  Default: `10.0`

Where:

* `<angle_setup>` = how the wire is setup on the gripper
* `<duration_sec>` = number of seconds to record

---

If you want, I can refine formatting, add troubleshooting notes, or generate a more compact version.




## 10. Check the Recorded Bag Location


By default, `run_bag` writes the MCAP bag under:

`/home/rotem/data_collection/recordings/`
