source install/rarm/share/rarm/bring_up/gp7.sh    - -  to open in gazebo world

ros2 run rarm_mvoeit arm_gripper_loop_controller.py   -- to run the motion

Here when reload=True the inverse kinematic will be calculated to given time step. If the time step is lower, it might take more time to run so i saved the data it to a file called data.npy to be reused in the next run. If that file exists it won't calculate the ik and run from the file. Therefore, if you want to make new motion only change reload=True (new motion will be created in to file data.npy) then build (using colcon build command) again change reload=False so that it will keep running from file without taking much time.

For the current configuration


Motion 1
<img width="1000" height="600" alt="Figure_1" src="https://github.com/user-attachments/assets/7e23be8b-eb28-4774-b2f8-3f24ecd5a593" />
<img width="1000" height="400" alt="Figure_2" src="https://github.com/user-attachments/assets/b9e7a18b-214a-473a-90c4-aeb7ab34ebb6" />


Motion 2
<img width="1000" height="600" alt="Figure_1 m2" src="https://github.com/user-attachments/assets/6631ca6c-a0d1-4601-bfd0-3e661334be7d" />
<img width="1000" height="400" alt="Figure_2 m2" src="https://github.com/user-attachments/assets/313dec14-5a7e-4c7a-af55-2e15b31ea558" />

Motion 3
<img width="1000" height="600" alt="Figure_1 m3" src="https://github.com/user-attachments/assets/429878b9-7769-411a-8b0f-8030cd1a7ba0" />
<img width="1000" height="400" alt="Figure_2 m3" src="https://github.com/user-attachments/assets/a3903020-d0b0-426a-9fad-4a4f8ecd4b14" />


Motion 4
<img width="1000" height="600" alt="Figure_1 m4" src="https://github.com/user-attachments/assets/b27e0e17-4537-4d91-a4eb-5af12a1d9cf0" />
<img width="1000" height="400" alt="Figure_2 m4" src="https://github.com/user-attachments/assets/af21a8e3-38ae-460a-9af0-de38618d93cd" />


Motion 5
<img width="1000" height="600" alt="Figure_1 m5" src="https://github.com/user-attachments/assets/66530c01-c1ed-498e-8dd7-6ff937ef1362" />
<img width="1000" height="400" alt="Figure_2 m5" src="https://github.com/user-attachments/assets/fff4b724-f107-4e92-bd69-479903706f27" />

