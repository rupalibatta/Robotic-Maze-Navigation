# Autonomous Maze Navigation

The project for **Caltech ME 129**, built with a two-wheel, Raspberry Pi-controlled robot that follows lines, maps intersections, and navigates an initially unknown network of paths. The repository tracks nine stages of development, categorized into goals, from motor control to autonomous exploration, shortest-path navigation, and NFC-assisted prize localization.

## Project outline

1. Control two DC motors with GPIO PWM and calibrate the robot's turns.
2. Follow lines using three infrared sensors and detect intersections, dead ends, and turns.
3. Build an intersection map while exploring unknown streets and detecting blocked paths.
4. Navigate to specified coordinates using Dijkstra's algorithm, or explore toward goals outside the known map.
5. Integrate ultrasonic obstacle sensing, ROS 2 commands, and NFC readings to search for a target prize.

## Mapping and navigation

The robot represents the maze as a graph of intersections at integer coordinates, with eight possible headings (45° increments). Each outgoing street is tracked as unknown, unexplored, connected, nonexistent, or a dead end, with a separate flag for temporary blockages. Line-sensor readings are smoothed to detect lateral deviations and intersections, while a calibrated angle sensor helps with turns.

When exploring, the robot checks unknown directions, follows unexplored streets, and routes toward reachable intersections with unfinished paths. For known destinations, `Map.setgoal()` runs Dijkstra's algorithm over connected, unblocked streets, assigning each intersection a cost and direction toward the goal. If the target is outside the mapped area, the controller selects unexplored routes in roughly the target direction.

## Sensors and task control

The later versions use three ultrasonic sensors for nearby obstacles, a magnetometer-based heading estimate, and a PN532 NFC reader for intersection identification. The `brain.py` state machine supports manual control, exploration, goal-directed driving, and a `FETCH` mode. In fetch mode, the robot uses reported distances to a prize at tagged intersections to guide its search and identify candidate locations. This is prize **localization**, not a demonstrated physical pickup mechanism.

`goals9/multithreaded_robot.py` separates robot control, terminal commands, and ROS 2 communication into threads. The ROS node publishes the estimated pose and accepts goal, exploration, and fetch commands. Map plots and saved map files support inspection of the explored network.

## Repository contents

| Folder | Contents |
| --- | --- |
| [`goals1/`](goals1/) | DC motor control and basic driving tests |
| [`goals2/`](goals2/) | Infrared line sensing, line following, and turning |
| [`goals3/`](goals3/) | Intersection detection and initial robot behaviors |
| [`goals4/`](goals4/) | Heading sensing, intersection maps, and graph-based navigation |
| [`goals5/`](goals5/) | Autonomous mapping, route planning, and saved test maps |
| [`goals6/`](goals6/) | Proximity sensing, wall following, and obstacle handling |
| [`goals7/`](goals7/) | Updated navigation and mapping with multithreaded control |
| [`goals8/`](goals8/) | ROS 2 integration, robot control, and map visualizations |
| [`goals9/`](goals9/) | NFC sensing, prize search, and integrated navigation controller |

## Authors

**Rupali Batta and Alexis Herfurth**  
California Institute of Technology, **ME 129**
