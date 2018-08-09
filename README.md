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

### Light randomizer
This randomizer randomly changes the properties of the light source given by its name.

### Shape randomizer
This randomizer randomly generates objects of various shapes in the world space.
The generated objects are not collision objects.

### Link properties randomizer
This randomizer randomly changes mass of the link given by the model name.