### Here is some information I got from a thread post that I think is very usefull. ###

LaunchConfiguration is local to the launch file and scoped.

DeclareLaunchArgument allows you to expose the argument outside of your launch file. Allowing them to be listed, set, or marked as required when a user launches it from the command line (using ros2 launch) or when including it from another launch file (using IncludeLaunchDescription).

A LaunchConfiguration cannot be required to be set when launching or including and it is not possible to set it when launching from the command line. You can set a LaunchConfiguration before including another launch file, but an argument is better if you want it to be reused.




```python 
import launch


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('msg', default_value='hello world'),
        launch.actions.DeclareLaunchArgument('other'),
        launch.actions.LogInfo(msg=launch.substitutions.LaunchConfiguration('msg')),
        launch.actions.LogInfo(msg=launch.substitutions.LaunchConfiguration('other')),
    ])

```


If I try to launch it without arguments (ros2 launch ./test.launch.py) I get:

```python
[INFO] [launch]: Default logging verbosity is set to INFO
[ERROR] [launch]: Caught exception in launch (see debug for traceback): Included launch description missing required argument 'other' (description: 'no description given'), given: []
```

That's because the other option has no default value and therefore is required.

I actually found out that the --show-arguments option was broken while testing this, so I'm not sure if that's what you ran into, but I open a pr to fix:

https://github.com/ros2/launch_ros/pu...

With that fix I can check the arguments (ros2 launch -s ./test.launch.py):

```python
    Arguments (pass arguments as '<name>:=<value>'):

        'msg':
            no description given
            (default: 'hello world')

        'other':
            no description given

```

I can run it if I specify other (ros2 launch ./test.launch.py other:='lorem ipsum'):

Output: 
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.actions.log_info]: hello world
[INFO] [launch.actions.log_info]: lorem ipsum