from GazeboMaterial.GazeboMaterialFile import *
from GazeboMaterial.GazeboMaterialItem import *


# Example content
def _getTestContent():
	return """
import * from "grid.material"

vertex_program Gazebo/DepthMapVS glsl
{
  source depth_map.vert

  default_params
  {
    param_named_auto texelOffsets texel_offsets
  }
}
fragment_program Gazebo/DepthMapFS glsl
{
  source depth_map.frag

  default_params
  {
    param_named_auto pNear near_clip_distance
    param_named_auto pFar far_clip_distance
  }
}

material Gazebo/DepthMap
{
  technique
  {
    pass
    {
      vertex_program_ref Gazebo/DepthMapVS { }
      fragment_program_ref Gazebo/DepthMapFS { }
    }
  }
}

vertex_program Gazebo/XYZPointsVS glsl
{
  source depth_points_map.vert
}

fragment_program Gazebo/XYZPointsFS glsl
{
  source depth_points_map.frag

  default_params
  {
    param_named_auto width viewport_width
    param_named_auto height viewport_height
  }
}

material Gazebo/XYZPoints
{
  technique
  {
    pass pcd_tex
    {
      separate_scene_blend one zero one zero

      vertex_program_ref Gazebo/XYZPointsVS { }
      fragment_program_ref Gazebo/XYZPointsFS { }
    }
  }
}

material Gazebo/Grey
{
  technique
  {
    pass main
    {
      ambient .3 .3 .3  1.0
      diffuse .7 .7 .7  1.0
      specular 0.01 0.01 0.01 1.000000 1.500000
    }
  }
}
material Gazebo/Gray : Gazebo/Grey
{
}

"""


def load_gazebo_setup():
	from dotenv import load_dotenv
	from pathlib import Path
	from os.path import join as joinp, dirname, abspath

	dotenv_path = Path('/usr/share/gazebo/setup.sh')
	load_dotenv(dotenv_path)

def gazebo_material_test():
	gazebo_resource_path = os.getenv('GAZEBO_RESOURCE_PATH').split(':')

	for gzpath in gazebo_resource_path:
		gazebo_material_path = joinp(str(gazebo_resource_path), "media", "materials", "scripts", "gazebo.material")
		if not os.path.exists(gazebo_material_path):
			continue
		gzMaterial = GazeboMaterialFile(gazebo_material_path);
		gzMaterial._parse(_getTestContent())
		GAZEBO_GREY='Gazebo/Grey'
		GAZEBO_GRAY='Gazebo/Gray'

		print("Test: ",
			gzMaterial.find('material[name={0}].technique.pass.ambient'.format(GAZEBO_GREY)),
		)
		grey = gzMaterial.getColor(GAZEBO_GREY)
		gray = gzMaterial.getColor(GAZEBO_GRAY)
		assert grey
		assert gray
		c1, c2 = gray[0], grey[0]
		print(str(" ".join(c1.args())))

def gazebo_tree_test():
	pass

def parse_args():
	import argparse

	parser = argparse.ArgumentParser()
	parser.add_argument('--test-inheritance', action='store_true')
	parser.add_argument('--test-dumptree', action='store_true')
	return parser.parse_args()

if __name__ == "__main__":
	args = parse_args()
	if args.test_inheritance:
		gazebo_material_test()
	elif args.test_dumptree:
		pass

__all__ = ['GazeboMaterialItem', 'GazeboMaterialFile', 'load_gazebo_setup']

# vim: ts=4 sw=4 noet
