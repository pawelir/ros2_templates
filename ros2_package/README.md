# ros2_package

[ A short description of the package. ]

## Building from source

[![Build and test code](https://github.com/pawelir/ros2_templates/actions/workflows/test-ros2-package.yaml/badge.svg)](https://github.com/pawelir/ros2_templates/actions/workflows/test-ros2-package.yaml)

_**Note:** Before proceeding with the below steps, make sure to change the directory to your workspace where the `src` folder is placed._

Resolve dependencies with the [rosdep](https://wiki.ros.org/rosdep):

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Build package and source local workspace

```bash
colcon build --symlink-install --packages-select ros2_package
source install/setup.sh
```

## Usage

[ Description of the quickest way to launch a core functionality of the package. ]

```bash
ros2 launch ros2_package ros2_package.launch.py
```

### Config files

- **ros2_package.yaml** - [ Short description of the content of this config file ]

### Launch files

- **ros2_package.launch.py** - [ Short description of the launched entities ]

  - **`argument_1`** _(Default: `default_value`)_ - [ Short description ]

## Nodes

### ros2_package_node

[ Description of the node functionality. ]

#### Subscribed Topics

- **`/namespace/topic_name`** _package_msgs/MsgType_ - [Short description]

#### Published Topics

- **`/namespace/ros2_package/topic_name`** _package_msgs/MsgType_ - [Short description]

#### Service servers

- **`/namespace/ros2_package/service_name`** _package_srvs/Type_ - [Short description]

#### Service clients

- **`/namespace/service_name`** _package_srvs/Type_ - [Short description]

#### Action servers

- **`/namespace/ros2_package/action_name`** _package_action/Type_ - [Short description]

#### Action clients

- **`/namespace/action_name`** _package_action/Type_ - [Short description]

#### ROS Parameters

| Name             | Type             | Default value   | Limit values (optional) | Description             |
| ---------------- | ---------------- | --------------- | ----------------------- | ----------------------- |
| `parameter_name` | [parameter_type] | [default_value] | min: [low], max: [high] | [Parameter Description] |
