# Bosch UWB

## Use

```bash
cd /home/edison/bosch_UWB
make uwb.up      # build + start
make uwb.shell   # open a terminal in the container
make uwb.down    # stop
```

## Shared volume (host ⇄ container)

This repo is mounted into the container as a **shared volume**:

- **Edit/create on the host**: changes appear immediately inside the container.
- **Edit/create inside the container**: changes are written back to your host filesystem (same workspace directory).

## First-time build (inside the container)

After the **first** time you run `make uwb.shell`, you need to compile the ROS 2 workspace once:

```bash
colcon build
source install/setup.bash
```

After that, when you open a **new terminal** with `make uwb.shell`, you should **not** need to manually run `source install/setup.bash` again (it is sourced from `.bashrc`).

## Running the nodes

- **Run all nodes at the same time (recommended)**:

```bash
ros2 launch sensors sensors.launch.py
```

This launch file starts `unix_timestamp`, `uwb_node`, and (optionally) `uwb_test_sender`.

- **Run individually**:

```bash
ros2 run sensors uwb_node
ros2 run sensors uwb_test_sender
ros2 run sensors unix_timestamp
```



