VSS-SampleCpp
=============
[![License: GPL v3](https://img.shields.io/badge/License-GPL%20v3-blue.svg)][gpl3]
[![Build Status](https://travis-ci.org/SIRLab/VSS-SampleCpp.svg?branch=master)][travis]

Mais informações podem ser encontradas em [VSS-SDK][vss-sdk].

## Lendo um pacote state

```c_cpp
global_state.balls(0).pose().x();       // Pos X
global_state.balls(0).pose().y();       // Pos Y

global_state.robots_yellow(i).pose().x();       // Pos X
global_state.robots_yellow(i).pose().y();       // Pos Y
global_state.robots_yellow(i).pose().yaw();     // Rot Z

global_state.robots_blue(i).pose().x();         // Pos X
global_state.robots_blue(i).pose().y();         // Pos Y
global_state.robots_blue(i).pose().yaw();       // Rot Z
```

## Montando um pacote commands

```c_cpp
global_commands = vss_command::Global_Commands();

global_commands.set_situation(NONE);
global_commands.set_name("Nome_do_time");
global_commands.set_is_team_yellow(true);

for(int i = 0 ; i < 3 ; i++){
    vss_command::Robot_Command *robot = global_commands.add_robot_commands();
    robot->set_id(i);
    robot->set_left_vel(5.0);
    robot->set_right_vel(-5.0);
}
```

## Motando um pacote debug

```c_cpp
global_debug = vss_debug::Global_Debug();
	
for(int i = 0 ; i < 3 ; i++){
    vss_debug::Pose *steps = global_debug.add_step_poses();
    steps->set_id(i);
    steps->set_x(5.0);
    steps->set_y(33.0);
    steps->set_yaw(1.0);
}

for(int i = 0 ; i < 3 ; i++){
    vss_debug::Pose *finals = global_debug.add_final_poses();
    finals->set_id(i);
    finals->set_x(6.0);
    finals->set_y(22.0);
    finals->set_yaw(2.0);
}
```

Installation
------------
[INSTALL][install]

Cloning
-------
```
$ git clone --recursive https://github.com/SIRLab/VSS-SampleCpp.git
```

Development and Use
-------------------

```
$ make proto
$ make 
$ ./VSS-SampleCpp -c [yellow || blue]
```

License
-------

This code is licensed under the [GNU GENERAL PUBLIC LICENSE Version 3][gpl3], of which a textual copy is available at [LICENSE.txt](LICENSE.txt).

You are allowed and encouraged to use this software on the IEEE Very Small Size Soccer competitions.  If you do, please let us know.

[gpl3]: http://www.gnu.org/licenses/gpl-3.0/
[travis]: https://travis-ci.org/SIRLab/VSS-SampleRust
[install]: https://github.com/SIRLab/VSS-SampleCppy/blob/master/INSTALL.md
[vss-sdk]: http://sirlab.github.io/VSS-SDK