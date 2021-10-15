# Cake Sim Node

This project aims to provide a containerized environment to run the simulations of Cake projects.

A Cake Sim Node:

- is a single docker container
- provides a single simulation world
- uses Gazebo as simulation engine
- provides a Web UI for viewing the simulation world (based on gzweb)
- can cmmunicate with an external node running a Cake Robotics application DDS on UDP (in fact, any ROS 2 application works as long as DOMAIN_ID is 0)

**Security Note:** Cake Sim Node uses projects like Gazebo and gzweb which are complex and under heavy development. These projects are not designed to have security measures against user input. Therefore, if you're providing Cake Sim Node as a user service, you should assume that any arbitary user code can be executed on the node. This means:

- Nodes should not have elevated privileges
- Nodes should be isolated from each other and the host. If you trust user input, kernel namespace isolation should be enough, as it's unlikely to accidentaly break through namespace isolation. For untrusted user input, you should use more isolation.
- Nodes outgoing traffic should be firewalled
- No sensitive data should be stored in the nodes (either in the file system or as environment variables).

## Usage

```
git clone https://github.com/cakerobotics/sim
cd sim
docker build . --tag=sim:local
docker run --rm -t -p 8080:8080 sim:local
```

Now you can see the simulation on http://localhost:8080. To communicate with a Cake Robotics project, just run the other container with user code on the same network that Sim Node is running on. By default, all docker containers run on the same network called *bridge*, so just run the application container normally and you're good. Don't forget to edit the props file and change `sim` value to `true`.

Note 1: If you mount a `props.yaml` or `props.json` in root path (`/`) of the container, that file will be read and used to generate a robot before simulation starts. Otherwise, a default diff_drive wheeled robot will be used. (TODO: More info)

Note 2: Environment variable `WORLD` can be used to select simulation world. When empty, a default world will be used. When points to a file, that file will be used (e.g. `/usr/share/gazebo-11/worlds/cafe.world`). To use with an external `.world` file, just mount it somewhere in the node file system and use the environment value to point to the path.
