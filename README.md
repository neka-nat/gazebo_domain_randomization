# Gazebo Domain Randomization

https://arxiv.org/abs/1703.06907

![result](gazebo_domain_randomizer/images/result.gif)

## Run

```
roslaunch gazebo_domain_randomizer demo.launch
```

If you want to randomize with external trigger, execute with the following option.

```
roslaunch gazebo_domain_randomizer demo.launch event_mode:='trigger'
```

And please execute the following command on another console.

```
rostopic pub /randomizers/randomizer/trigger std_msgs/Empty "{}"  -r 1.0
```

## Randomizers

|Node name |Randomized parameter|
| -------- | -------------------- | 
| **light_randomizer** | - light color </br> - attenuation |
| **shape_randomizer** | - shape(sphere, box, cylinder) </br> - shape color </br> - position(x, y, z) |
| **sky_randomizer** | - time of day |
| **physics properties randomizer** | - gravity(x, y, z) |
| **link_properties_randomizer** | - link mass |
| **joint_properties_randomizer** | - joint damping |
| **link_visual_properties_randomizer** | - link color |
| **surface_params_randomizer** | - link mu1, mu2, mu_torsion </br> - link poisson ratio |
