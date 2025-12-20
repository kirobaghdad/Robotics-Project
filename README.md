## How to Use ##

1. Clone the git repo into the catkin workspace `src` folder.

2. Build the workspace:

    ```bash
    cd ~/catkin_ws
    catkin_make
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    ```

3. In one terminal, run:

    ```bash
    roslaunch limo_gazebo_sim limo_four_diff.launch
    ```

4. In another terminal, run:

    ```bash
    roslaunch limo_bringup limo_gmapping.launch
    ```

5. In VS Code, go to the debug section on the left and click on **Start Debugging**. This will start the navigation.

## Documenting the trials ##

1. First approach:

At the beginning, we tried to make everything from scratch.
<br>
By everything I mean navigation, mapping, etc...
It was really hard.

2. Second approach

We used move_base, explore_lite and gmapping.
<br>
The first for moving the robot, the second for exploring the world, and the last for mapping.