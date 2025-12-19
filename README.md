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