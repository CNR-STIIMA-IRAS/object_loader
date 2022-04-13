# object_loader
TBD

## Object format
TBD

## Services
TBD


- AddObjects  

- RemoveObjects

- AttachObject

- DetachObject

- ListObjects

- ChangeColor

- AddObjectsGroup

- RemoveObjectsGroup





## Load object groups
Add/remove group of objects.

add group service is defined [here](object_loader_msgs/srv/AddObjectsGroup.srv)

remove group service is defined [here](object_loader_msgs/srv/RemoveObjectsGroup.srv)

Yaml format
```yaml
A1650:
  - {type: "MountingArea_A1650",  frame: "table",   position: [0.3, 0.5, 0.0], quaternion: [0.0, 0.0, 0.0, 1.0]}
  - {type: "MountingArea_A1650",  frame: "table",   position: [0.3, 1.5, 0.0], quaternion: [0.0, 0.0, 0.0, 1.0]}

A1652:
  - {type: "MountingArea_A1652",  frame: "table",   position: [0.3, 0.5, 0.0], quaternion: [0.0, 0.0, 0.0, 1.0]}
  - {type: "MountingArea_A1652",  frame: "table",   position: [0.3, 1.5, 0.0], quaternion: [0.0, 0.0, 0.0, 1.0]}
```

Services:
```shell
rosservice call /add_objects_group "objects_group: 'A1650'"
rosservice call /remove_objects_group "objects_group: 'A1650'"
```
