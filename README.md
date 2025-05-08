# deeporange14_control Package
# Deep Orange DBW Controls Interface Package for Autonomous Driving

### Functionality:

- `DeepOrangeDbwCan.cpp`: Enable passing the ROS topics to the Raptor DBW controller via SocketCAN, which executes the commands for the platform to move. Additionally monitors rtk fix, logging status
- `DeepOrangeVelocityController.cpp`: Use the vehicle dynamics model to convert vehicle velocity to left and right track velocity/torques.
- `DeepOrangeStateSupervisor.cpp`: A ROS state machine in sync with the Raptor controller state machine.
- `DataLogger.cpp`: logging CAN and ROS data while monitoring logging status active or Fail
- `DeepOrangeInterfaceNode.cpp`: creates the controller node, wrapping each of the above functions

### Cloning and building:
- Use the following to clone this repository and properly set up submodules:
```
git clone https://github.com/DO13-Autonomy/deeporange14_control.git -b ros2
cd deeporange14_control
git submodule init
git submodule update
```
  - These commands clone the `ros2` branch of the repository, then initialize and clone the submodules
  - To verify the command has worked, check that `src/raptor-dbw-ros` is not empty
  - See below for more information on dependencies
- If ROS has been installed and sourced (either in `~/.bashrc` or manually), run the following from the top-level of the repository to install dependencies:
```
rosdep install --from-paths src --ignore-src -r -y 

```
- Build the code by calling `colcon build` in the top-level of the repo

### How to Use:
- To add/update CAN msg IDs- update these in the dbc in the `dbc` folder as well as `dispatch_can_msgs.h` include file
- Vehicle and subsystem states are enumerated in `DeeporangeStateEnums.h`

### Source Code Dependencies:
- `can_dbc_parser`: Developed by New Eagle as part of metapackage - https://github.com/NewEagleRaptor/raptor-dbw-ros2 (branch: foxy)
- `deeporange14_control`: Package implementing the interface with the Raptor software and performing control actions
- `deeporange14_msgs`: Package for custom messages for DO14 DBW operation

All other dependencies will be installed with `rosdep` using the information in `package.xml`.

### Testing:
Automatic linting capability was added to the deeporange14_control package through `ament-lint` and `roslaunch-check`.  After building the pacakge, the tests can be launched for the `deeporange14_control` package
```
colcon test --event-handlers console_direct+ --packages-select deeporange14_control
```
Results can be found in multiple locations:
  - Printed to the console (when the `--event-handlers console_direct+` options is used)
  - Log files in `log/latest_test/test_results/deeporange14_control` or (for older tests) `log/test_YYYY-MM-DD_HH-MM-DD/deeporange14_control`
  - Log file `build/deeporange14_control/Testing/Temporary/LastTest_YYYYMMDD-XXXX.log` (with other summaries in the other files in this `Testing` directory)
  - XML files in `build/deeporange14_control/test_results` (only latest tests results)

The third-party pacakges also contain some unit tests which can be launched one-at-a-time using the above command (substituting the package name).  All tests in the workspace can be launched with `colcon test --event-handlers console_direct+`.  To exclude console output, just run `colcon test` without the `--event-handler` setting.

To check an XML launch file (or any other XML-formatted file), use `ament_xmllint` from the commandline:
```
ament_lint src/deeporange14_control/launch/control.lauch
```
(Run without an argument, this will produce results for every file with the XML extension found, recursively, in the working directory.  It does not detect XML-formatted files that do not have the XML extension, such as launch files.)

Note that the following failures are expected, and will be addressed once the code progresses to a more complete state:
- `ament_cpplint`: performs static lint on the source code
  - Expected to fail in for 9 files (9 total failures)
    - 4 failures in `deeporange14_control` header files due to `build/include_order` wanting the header file to be included in itself (not possible to pass this check as it is being run)
    - 4 failures in `DeepOrangeInterfaceNode.cpp` due to `build/include_order` wanting a header file with this same file name to be included (not possible to pass this check as there is no `DeepOrangeInterfaceNode.h` file in the package)
    - 1 failure in `DataLogger.cpp` due to exceeding the recommended line length.  The line in question is a system command that contains a list of topics to record.  This is not addressed/fixed because of the uncertain future of this particular component within the node (and the potential to rework this part of the code if the logger is retained).
- `ament_uncrustify`: applies a configuration file, based on the [ROS2 coding standards](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html#id1), to ensure consistent formatting
  - Expected to fail for 8 files
    - 6 failures in `deeporange14_control` header files due to a check that wants the contents of a namespace block to be indented, but `public`/`private` keywords within a(n indented) class to align all the way to the left.  This is a discrepancy with both the Google Style Guide and the changes to this made by the ROS2 style guide, and so was left as-is.
    - 1 failure in `DataLogger.cpp` due to the same problem identified by `ament_cpplint`
    - 1 failure in `DeepOrangeDbwCan.cpp` due to the code not compiling after following the advice provided by `ament_uncrustify`

An additional format checker, `ament_clang_format`, is also available to run as a test, but it was found that many of those setting actively work against `ament_uncrustify`, so that it is not possible to pass both format checkers simultaneously.  It was opted to retain use of `ament_uncrustify` since this is the format checked packaged in the "automatic testing" provided by ament_cmake.  (Conversations on issues raised in ament_cmake_lint related to this discrepancy also point to Uncrustify being more flexible/easier to configure to check for the formatting requirements, partially driving the decision to keep that check in place for this code, for now.)

### Contributors:

#### DO15
 - Pavan Kumar Athinarapu (athinak@clemson.edu)
 - Anirudh Kyatham (akyata@clemson.edu)
 - Mrunali Mahadeo Sawant (msawant@clemson.edu)
 - Vasanth Seethapathi (vasants@clemson.edu)

#### DO14
 - Sanskruti Jadhav (sanskrj@clemson.edu)
 - Shubham Gupta (gupta9@clemson.edu)
 - Shubhankar Kulkarni (sskulka@clemson.edu)
 - Priyanshu Rawat (prawat@clemson.edu)
 - Vasudev Purohit (vpurohi@clemson.edu)

 DO14/15 Program Manager/Maintainer: Chris Paredis (paredis@clemson.edu)
 ROS2 Maintainer: Alicia Zinnecker (azinnec@clemson.edu)
