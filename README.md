# sdf2urdf

Converter gazebo's sdf to urdf.

## Dependencies
```(sh)
sudo pip3 install xacro
```

## Usage

```(sh)
  export PYTHON_PATH=$(pwd)/GazeboMaterial
  ./sdf2urdf <input.sdf> [ <output.urdf> ]
```

### Examples

See `./examples/`
```(sh)
./sdf2urdf.py ./examples/example001.sdf
```
