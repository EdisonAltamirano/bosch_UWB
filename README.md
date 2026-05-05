# Bosch UWB

## Use

```bash
cd bosch_UWB
make uwb.up      # build image + start container
make uwb.shell   # open a terminal in the container
make uwb.down    # stop container (keeps it alive — no data lost)
```

> `make uwb.down` stops the container without removing it, so installed packages and any in-container state are preserved. To fully remove the container run `docker compose down` manually.

## Shared volume (host ⇄ container)

This repo is mounted into the container as a **shared volume** at `/home/ws/src`:

- **Edit/create on the host**: changes appear immediately inside the container at `/home/ws/src`.
- **Edit/create inside the container**: changes are written back to the host filesystem in this same directory.

The container maps to your host UID/GID automatically (via `LOCAL_USER_ID` / `LOCAL_GROUP_ID`), so files created inside the container are owned by your host user and remain editable from the host without permission issues.

## First-time build (inside the container)

After the **first** time you run `make uwb.shell`, you need to compile the ROS 2 workspace once:

```bash
cd /ws
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



