# deeporange14_control Package
# Deep Orange DBW Controls Interface Package for Autonomous Driving

### Functionality:
- DeepOrangeDbwCan.cpp: handles the CAN-ROS interface, leveraging SocketCAN, to send commands to Raptor and receive command from Raptor
- DeepOrangeStateSupervisor.cpp: ROS state machine in sync with the Raptor controller state machine
- DeepOrangeInterfaceNode.cpp - instantiates the ROS node for the controller

### Cloning and building:
- Use the following to clone this repository and properly set up submodules:
```
git clone https://github.com/DO13-Autonomy/deeporange14_control.git
cd deeporange14_control
git submodule init
git submodule update
```
  - These commands clone the repository, then initialize and clone the submodules
  - To verify the command has worked, check that `src/pugixml`, `src/raptor-dbw-ros`, and `src/ros_canopen` are not empty
  - See below for more information on these dependencies
- If ROS has been installed and sourced (either in `~/.bashrc` or manually), run the following from the top-level of the repository to install dependencies:
```
rosdep install --from-paths src --ignore-src -r -y 
```
- Build the code by calling `catkin build` in the top-level of the repo

### Viewing or editing files in VS Code:
- It is recommended to open the top-level folder (e.g., `deeporange14_control`) in VS Code, as the file `.vscode/c_cpp_properties.json` include paths are relative to this workspace directory
  - If the `src` subfolder is opened in VS Code, IntelliSense may indicate false-positive errors about missing include paths
  - This file is written to be agnostic to clone location and should not require editing
- VS Code will detect that this is a Git repository and can be used for making commits in the GUI, rather than relying on using the CLI
  - It is recommended to take advantage of this and commit changes in incremental, related chuncks, rather than one big commit at the end
  - When looking at diffs, it is also possible to stage partial file changes by selecting the lines in the RHS pane (the new file), right-clicking, and selecting "Stage Selected Revisions" -- this can be helpful when trying to create modular commits from larger-scale changes
  - The commandline may still be used for pushing and pulling, or use the option in the bottom left of the VS Code window to simultaneous pull from, then push to, the remote repository
- Recommended extensions to consider installing:
  - CMakeTools
  - Pylance
  - Python
  - Python Debugger
  - Python Environments
  - C/C++
  - C/C++ DevTools
  - C/C++ Extension pack
  - C/C++ Themes
  - Robot Developer Environment
  - Robot Developer Extensions for URDF

### How to Use:
- To add/update CAN msg IDs- update these in the dbc in the `dbc` folder and `DeepOrangeCanDispatch.h`
- State machine enumerations are given in `DeepOrangeStateEnums.h`

### External Dependencies:
- `can_dbc_parser`: Developed by New Eagle as part of metapackage - https://github.com/NewEagleRaptor/raptor-dbw-ros
- `socketcan_bridge` : Package for interfacing CAN to ROS on Linux - https://github.com/ros-industrial/ros_canopen
- `can_msgs` : Package to support CAN msg format on ROS topics. - https://github.com/ros-industrial/ros_canopen
- `pugixml`: Package for processing XML files. - https://github.com/joselusl/pugixml

### Testing:
Automatic linting capability was added to the deeporange14_control package through `roslint` and `roslaunch-check`.  After building the pacakge, the tests can be launched using
```
catkin test deeporange14_control
```
Results will be printed to the console and in associated XML files in `build/deeporange14_control/test_results/deeporange14_control`.

The third-party pacakges also contain some unit tests which can be launched one-at-a-time using the above command (substituting the package name).  All tests in the workspace can be launched with `catkin test`.

Additional linting can be done with the [catkin_lint](https://fkie.github.io/catkin_lint/) tool, which can be installed following the instructions at the link and launched from the root of the catkin workspace using
```
source devel/setup.bash
catkin_lint --pkg deeporange14_control
```

### Contributors:
 - Pavan Kumar Athinarapu (athinak@clemson.edu)
 - Anirudh Kyatham (akyata@clemson.edu)
 - Mrunali Mahadeo Sawant (msawant@clemson.edu)
 - Vasanth Seethapathi (vasants@clemson.edu)
 - Sanskruti Jadhav (sanskrj@clemson.edu)
 - Shubham Gupta (gupta9@clemson.edu)
 - Shubhankar Kulkarni (sskulka@clemson.edu)
 - Priyanshu Rawat (prawat@clemson.edu)
 - Vasudev Purohit (vpurohi@clemson.edu)

 - Program Manager/Maintainer: Chris Paredis (paredis@clemson.edu)
