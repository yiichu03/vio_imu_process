To debug or develop the codebase, we recommend to install and use linter and QtCreator.

## Install static code analysis tool linter
The below instructions installs linter which requres python2 following the
guide at [here](https://github.com/ethz-asl/linter/tree/master).

```
sudo pip2 install yapf requests pylint
sudo apt install clang-format-6.0
cd $SLAM_TOOL_PATH
git clone https://github.com/JzHuai0108/linter.git
cd linter
git fetch origin
git checkout -b feature/onespacecppcomment

echo ". $(realpath setup_linter.sh)" >> ~/.bashrc
bash

cd swift_vio_ws/src/swift_vio
init_linter_git_hooks
```

## Develop with QtCreator

Follow the below steps

### 1. Build swift_vio as described earlier.

### 2. Open swift_vio with QtCreator

Open QtCreator by

```
source /opt/ros/melodic/setup.bash # or .zsh
/opt/Qt/Tools/QtCreator/bin/qtcreator
```

Then, open swift_vio_ws/src/swift_vio/CMakeLists.txt in QtCreator. 

If a dialog warning "the CMakeCache.txt file or the project configuration has changed",
comes up with two buttons, "Overwrite Changes in CMake" and 
"Apply Changes to Project" with the former being the default one. 
To avoid adding CMAKE_PREFIX_PATH in future builds with catkin in a terminal,
the "Apply Changes to Project" option is recommended.

To enable building test targets inside QtCreator, you may need to turn on 
"CATKIN_ENABLE_TESTING" in the CMake section of Building Settings and 
select the *_test target in a newly added Build Step from the Build Steps section.
The default target "all" may not emcompass building some test targets.

To solve the error about loading shared libraries libmetis.so, 
add the lib path of gtsam to LD_LIBRARY_PATH as below inside the terminal or 
the Run Environment in QtCreator (Projects > Build and Run > Run > Run Environment).

```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
```

## Debug repeatable exceptions in QtCreator
Make QtCreator break on exceptions, referring to [here](https://stackoverflow.com/questions/16735413/how-can-i-make-qtcreator-break-on-exceptions).

## Debug erratic exceptions
Use debug_estimator_exception app

## Dynamic code analysis
Build with option -DUSE_SANITIZER=Address.

## catkin build versus QtCreator
After opening and building the project with QtCreator,
the following warning and associated errors may come up when the project is built again by catkin in a terminal.
a. "WARNING: Your workspace is configured to explicitly extend a workspace which
yields a CMAKE_PREFIX_PATH which is different from the cached CMAKE_PREFIX_PATH
used last time this workspace was built."
b. ImportError: No module named genmsg

Of course, you can resolve these issues by "catkin clean", but it is too expensive.
To resolve these errors, CMAKE_PREFIX_PATH needs to be specified when invoking catkin build, e.g.,
```
catkin build sliding_window_estimator -DCMAKE_PREFIX_PATH=/opt/ros/$ROS_DISTRO
```

