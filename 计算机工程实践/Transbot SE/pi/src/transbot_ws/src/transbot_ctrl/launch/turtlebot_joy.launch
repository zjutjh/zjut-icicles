<launch>
    <param name="use_sim_time" value="false"/>
    <node name="joy_node" pkg="joy" type="joy_node" output="screen" respawn="false"/>
    <node name="turtlebot_joy" pkg="transbot_ctrl" type="turtlebot_joy.py" output="screen">
        <param name="linear_speed_limit" type="double" value="2.0"/>
        <param name="angular_speed_limit" type="double" value="2.0"/>
    </node>
</launch>
