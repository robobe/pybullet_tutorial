# Lesson 2
- basic urdf
- load custom urdf
- run pybullet simulation loop

## basic urdf

**URDF** = Unified Robot Description Format

It is an XML format that describes:

- Links (rigid bodies)
- Joints (connections)
- Visual shape
- Collision shape
- Inertial properties


!!! tip "Robot = Links + Joints"


### link

A `<link>` represents a **rigid body**.
It has three independent aspects:

1. visual: what you see
2. collision: what physics **touches**
3. inertial: how physics **move** it

#### Demo: simple box
```xml
<?xml version="1.0"?>
<robot name="box_robot">

  <link name="base_link">

    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <origin xyz="0 0 0"/>
    </visual>

    <collision>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="1.0"/>
      <inertia 
        ixx="0.01" ixy="0.0" ixz="0.0"
        iyy="0.01" iyz="0.0"
        izz="0.01"/>
    </inertial>

  </link>

</robot>

```

!!! info "coordinate frame"
    when we create link, we create new **coordinate frame**.
    

#### Visual
What we see

- Geometry
- optional mesh
- Material/color
- local origin

##### Geometry type
- box
- cylider
- sphere
- mesh


#### Visual origin

```xml
<origin xyz="0.2 0 0"/>
```


---

!!! info "Exersise_1"
    Create simple urdf that diffend red box size 1,1,1 and load it using pybullet
    