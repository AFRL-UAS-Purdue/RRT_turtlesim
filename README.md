1. Git clone that repo from the base dir
2. cd RRT_turtlesim
3. docker compose build
4. docker compose up
5. split the terminator window that appears with shift+ctrl+e
6. colcon build
7. ros2 run turtlesim turtlesim_node
8. split the terminator window again
9. ./gen_obst.sh
This will place bunch of turtles where I've hard coded the obstacles. Eventually these hard coded obstacles will be replaced with octomap.
10. ros2 run rrt_python rrt
At this point the main turtle should be listening to that ros topic for goal positions.
11. Split terminator window one more time
12. rqt
13. Use the rqt topic publisher to manually publish the desired goal location to /move_base_simple/goal
A good range to publish goal locations is between [1,10] for both x and y. Turtle will get stuck if it tries to move beyond the walls.
