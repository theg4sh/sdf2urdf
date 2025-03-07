#!/usr/bin/env python3

import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

# GazeboMaterial
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from GazeboMaterial import *

# xml operations
from xacro import parse
from xacro.xmlutils import *


sdf_frames = {}
link_attach_joint = {}

rospack = None

class Item:
	"""
['ATTRIBUTE_NODE', 'CDATA_SECTION_NODE', 'COMMENT_NODE', 'DOCUMENT_FRAGMENT_NODE', 'DOCUMENT_NODE', 'DOCUMENT_TYPE_NODE', 'ELEMENT_NODE', 'ENTITY_NODE', 'ENTITY_REFERENCE_NODE', 'NOTATION_NODE', 'PROCESSING_INSTRUCTION_NODE', 'TEXT_NODE', '__doc__', '__init__', '__module__', '__nonzero__', '__repr__', '_attrs', '_attrsNS', '_call_user_data_handler', '_child_node_types', '_get_attributes', '_get_childNodes', '_get_firstChild', '_get_lastChild', '_get_localName', '_get_nodeName', '_magic_id_nodes', 'appendChild', 'attributes', 'childNodes', 'cloneNode', 'firstChild', 'getAttribute', 'getAttributeNS', 'getAttributeNode', 'getAttributeNodeNS', 'getElementsByTagName', 'getElementsByTagNameNS', 'getInterface', 'getUserData', 'hasAttribute', 'hasAttributeNS', 'hasAttributes', 'hasChildNodes', 'insertBefore', 'isSameNode', 'isSupported', 'lastChild', 'localName', 'namespaceURI', 'nextSibling', 'nodeName', 'nodeType', 'nodeValue', 'normalize', 'ownerDocument', 'parentNode', 'prefix', 'previousSibling', 'removeAttribute', 'removeAttributeNS', 'removeAttributeNode', 'removeAttributeNodeNS', 'removeChild', 'replaceChild', 'schemaType', 'setAttribute', 'setAttributeNS', 'setAttributeNode', 'setAttributeNodeNS', 'setIdAttribute', 'setIdAttributeNS', 'setIdAttributeNode', 'setUserData', 'nodeName', 'toprettyxml', 'toxml', 'unlink', 'writexml']
	"""
	def __init__(self, xmlnode, parent=None, document=None, level=0, root=None):
		self._document = xmlnode if xmlnode.nodeName == '#document' else document
		self._root = root if root is not None else self
		self._node = xmlnode
		self._parent = parent
		self._level = level
		self._children = []
		self._textvalue = None

		if (len(self._node.childNodes) == 1) and (self._node.childNodes[0].nodeName == '#text'):
			self._textvalue = Item(self._node.childNodes[0], document=self._document, parent=self, level=level+1, root=self.getRootNode())
			return
		for n in self._node.childNodes[:]: 
			if level == 0:
				self._root =  self
			# Skip pretty tabs
			if n.nodeName == '#text' and n.toxml().strip() == '':
				self._node.removeChild(n)
				continue
			self.appendChild(Item(n, parent=self, document=self._document, level=level+1, root=self.getRootNode()))

	def __repr__(self):
		return f'Item<level={self._level},name={self.nodeName}>';

	def appendChild(self, item):
		if item.__class__.__name__ == 'Item':
			if item._node.parentNode is not None and item._node.parentNode != self._node:
				self._node.appendChild(item._node)
			if item in item._parent._children:
				del item._parent._children[item._parent._children.index(item)]
			if item not in self._children:
				self._children.append(item)
		else:
			item = Item(item, parent=self, document=self._document, level=self._level+1, root=self.getRootNode())
			self._children.append(item)

	def setAttribute(self, name, value):
		self._node.setAttribute(name, value)

	def getAttribute(self, name):
		return self._node.getAttribute(name)

	def removeAttribute(self, name):
		self._node.removeAttribute(name)

	def text(self):
		return self._node.childNodes[0].toxml()

	@property
	def nodeName(self):
		return self._node.nodeName

	def removeChild(self, item):
		pn = item._node.parentNode
		node = item._node
		while pn is not None and pn != self._node.parentNode:#不知道这个循环的意义在哪里，其实就是让父节点删掉这个节点
			pn.removeChild(node)
			node, pn = pn, pn.parentNode

		#self._node.removeChild(item._node)
		if item in self._children:
			del self._children[self._children.index(item)]

	def replaceChild(self, replace, which):
		if which in self._children:
			self._children[self._children.index(which)] = replace
		self._node.replaceChild(replace._node, which._node)

	def cloneNode(self):
		return Item(self._node.cloneNode(True), parent=None, document=self._document, level=self._level, root=self.getRootNode())

	def createElement(self, *args, **kwargs):
		try:
			return self._document.createElement(*args, **kwargs)
		except Exception as e:
			sys.stderr.write("{0}\n".format(self._node))
			raise e

	def isPlugin(self):
		return self

	def isSDF(self):
		return self.nodeName == 'sdf' and self._level == 1

	def uriToPath(self, uri):
		def try_encode(uri, wrap):
			try:
				return wrap(uri.encode("utf-8"))
			except:
				return wrap(uri)

		parts = try_encode(uri, lambda x: x.split('/'))
		uritype, _, parts = parts[0], None, parts[2:]

		searchpath = []
		typemap = {
			"model:": ["GAZEBO_MODEL_PATH"],
			"file:":  ["GAZEBO_RESOURCE_PATH"],
		}.get(uritype)

		if typemap is None:
			return None

		for envvar in typemap:
			value = os.getenv(envvar)
			if value is not None:
				searchpath += try_encode(value, lambda x: x.strip(":").split(':'))

		for gmp in searchpath:
			path = os.path.join(gmp, *parts)
			if os.path.exists(path):
				return path
		return None

	_gazeboMaterialFiles = {}
	def _getGazeboMaterial(self, node):
		name, gzMaterial = None, None

		for opt in node._children:
			if opt.nodeName == 'name':
				name = opt.text()
				if not name.startswith('Gazebo/'):
					raise Exception("Unsupported color NS: {0}".format(name))
			elif opt.nodeName == 'uri':
				materialpath = self.uriToPath(opt.text())
				if materialpath is None:
					raise Exception(f"Cannot find material reference in GAZEBO_MODELS_PATH and GAZEBO_RESOURCE_PATH: {opt.text()}")
				if self.__class__._gazeboMaterialFiles.get(materialpath) is None:
					self.__class__._gazeboMaterialFiles[materialpath] = GazeboMaterialFile(materialpath)
				gzMaterial = self.__class__._gazeboMaterialFiles[materialpath]
			else:
				raise Exception("Unknown material/script tag: {0}".format(opt.nodeName))
		return name, gzMaterial

	def getRootNode(self):
		if self._root is None:
			return self
		return self._root

	_materialNodes = {}
	def _getMaterialNode(self, name):
		exists = name in self.__class__._materialNodes
		if not exists:
			root = self.getRootNode()
			item = Item(self.createElement('material'), parent=root, document=self._document, level=root._level+1, root=self.getRootNode())
			item.setAttribute('name', name)
			root.appendChild( item )
			self.__class__._materialNodes[name] = item
		return self.__class__._materialNodes[name], exists

	def convert(self):
		if self._level == 0:
			for c in self._children[:]:
				# Main node replacement
				if c.isSDF():
					"""
						<sdf version="1.5">
							<model name="typhoon_h480_roscam">
								<...>
								<...>
							</model>
						</sdf>
					---
						<robot name="typhoon_h480_roscam">
							<...>
							<...>
						</robot>
					"""
					self.frame = np.eye(4)
					node = Item(self.createElement('robot'), parent=self, document=self._document, level=self._level+1) # Root is None because it's a new root node
					copy = c.cloneNode()
					for ch in self._children:
						self.removeChild(ch)
					for ch in self._node.childNodes:
						self._node.removeChild(ch)

					for name, attr in copy._node.childNodes[0].attributes.items():
						node._node.setAttribute(name, attr)

					for ch in copy._node.childNodes[0].childNodes[:]:
						node._node.appendChild(ch)
						item = Item(ch, parent=self, document=self._document, level=self._level+1, root=node.getRootNode())
						node.appendChild(item)
					#self.replaceChild(node, copy)
					self.appendChild(node)
					break
				else:
					raise Exception("Unsupported file type: {0}".format(c.nodeName))

		_properties = {}
		for c in self._children[:]:
			if c.nodeName not in _properties:
				_properties[c.nodeName] = c
			elif type(_properties[c.nodeName]) == list:
				_properties[c.nodeName].append(c)
			else:#if there has existed a node with the same name, then convert it to a list
				_properties[c.nodeName] = [_properties[c.nodeName], c]
		global sdf_frames

		if self.nodeName in ['link','joint']:# if the node is link, then get the pose of the link
			link_name = self._node.getAttribute('name')
			if link_name not in sdf_frames: #the base link frame is Identity matrix
				sdf_frames[link_name] = np.eye(4)

			if 'pose' in _properties:
				#get joint frame
				relative_to_frame = np.eye(4)
				relative_to_frame_name = _properties['pose']._node.getAttribute('relative_to')
				if relative_to_frame_name == '':
					print(f"{link_name} do not have relative_to attribute")
				else:
					relative_to_frame = sdf_frames[relative_to_frame_name]
				#get relative pose to joint frame
				text = _properties['pose'].text().strip().split(' ')  # ['0', '0', '0', '0', '0', '0']
				Rotation_Matrix = R.from_euler('xyz', [float(x) for x in text[3:]]).as_matrix()
				Translation_Matrix = np.eye(4)
				Translation_Matrix[:3, :3] = Rotation_Matrix
				Translation_Matrix[:3, 3] = [float(x) for x in text[:3]]
				link_pose = relative_to_frame @ Translation_Matrix
				sdf_frames[link_name] = link_pose
				xyz = link_pose[:3,3]
				rpy = R.from_matrix(link_pose[:3,:3]).as_euler('xyz')
				_properties['pose']._node.childNodes[0].replaceWholeText(f"{xyz[0]} {xyz[1]} {xyz[2]} {rpy[0]} {rpy[1]} {rpy[2]}")

		global link_attach_joint
		if self.nodeName == 'joint':
			link_attach_joint[_properties['child'].text()] = self._node.getAttribute('name')
			base_frame = sdf_frames[_properties['parent'].text()]
			self_frame = sdf_frames[self._node.getAttribute('name')]
			relative_to_frame = np.linalg.inv(base_frame) @ self_frame
			xyz = relative_to_frame[:3, 3]
			rpy = R.from_matrix(relative_to_frame[:3, :3]).as_euler('xyz')
			_properties['pose']._node.childNodes[0].replaceWholeText(
				f"{xyz[0]} {xyz[1]} {xyz[2]} {rpy[0]} {rpy[1]} {rpy[2]}")

		if self.nodeName == 'link':
			# if the node is link, then get the pose of the link and its attach joint
			link_name = self._node.getAttribute('name')
			if link_name in link_attach_joint:
				link_frame = sdf_frames[link_name]
				joint_frame = sdf_frames[link_attach_joint[link_name]]
				link_relative_to_joint = np.linalg.inv(joint_frame) @ link_frame

				for c in self._children[:]:
					if c.nodeName in ['visual', 'collision', 'inertia']:
						for d in c._children[:]:
							if d.nodeName == 'pose':
								# compute Pose relatie to the joint frame
								text = d.text().strip().split(' ')  # ['0', '0', '0', '0', '0', '0']
								Rotation_Matrix = R.from_euler('xyz', [float(x) for x in text[3:]]).as_matrix()
								Translation_Matrix = np.eye(4)
								Translation_Matrix[:3, :3] = Rotation_Matrix
								Translation_Matrix[:3, 3] = [float(x) for x in text[:3]]
								pose_relative_to_joint = link_relative_to_joint @ Translation_Matrix
								xyz = pose_relative_to_joint[:3, 3]
								rpy = R.from_matrix(pose_relative_to_joint[:3, :3]).as_euler('xyz')
								d._node.childNodes[0].replaceWholeText(f"{xyz[0]} {xyz[1]} {xyz[2]} {rpy[0]} {rpy[1]} {rpy[2]}")
			self.removeChild(_properties['pose'])

		for c in self._children[:]:

			# Unused elements
			if c.nodeName in ['plugin', 'physics', 'gravity', 'velocity_decay', 'self_collide', 'surface', 'static', 'dissipation', 'stiffness', 'sensor', '#comment']:
				self.removeChild(c)

			# Inner tags into attributes, nothing else
			elif c.nodeName in ['inertia', 'cylinder', 'sphere', 'box', 'limit', 'dynamics'] :

				if c.nodeName == 'limit':
					c.convert()
				for mc in c._children[:]:
					c.setAttribute(mc.nodeName, mc.text().strip())
					c.removeChild(mc)

			# elif c.nodeName in ['visual', 'collision']:
			# 	if c.nodeName == 'collision' and c.nodeName in _properties:
			# 		self.removeChild(c)
			# 	else:
			# 		for name,attr in c._node.attributes.items():
			# 			c.removeAttribute(name)

			elif c.nodeName == 'material':
				"""
					<material>
					  <script>
						<name>Gazebo/DarkGrey</name>
						<uri>file://media/materials/scripts/gazebo.material</uri>
					  </script>
					</material>
				---
					<material name="DarkGrey">
					  <color rgba="0.3 0.3 0.3 1.0"/>
					</material>	
				"""
				unsupported = []
				for child in c._children:
					if child.nodeName == 'script' and c._level>1:
						name, gzMaterial = self._getGazeboMaterial(child)
						c.removeChild(c._children[0])
						if name:
							c.setAttribute("name", name)

						materials = gzMaterial.getColor(name)
						material, exists = self._getMaterialNode(name)
						if materials:
							if not exists:
								color = Item(self.createElement('color'), parent=c, document=self._document, level=c._level+1, root=self.getRootNode())
								rgba = materials[0].args + ['1.0', '1.0', '1.0', '1.0']
								color.setAttribute("rgba", " ".join(rgba[:4])) # rgba should contains 4 element, some of gazebo's material could contains 3 elements
								material.appendChild(color)
						else:
							query, fn = f'material[name={name}].technique.pass.ambient', gzMaterial.getFilename()
							raise Exception(f"Material not found ({query}) at {fn}")
					else:
						unsupported.append(child)

				if not c._children:
					sys.stderr.write(f"WARN: empty material: {c}\n")
				if unsupported:
					sys.stderr.write(f"WARN: Unsupported one of material subtag: {unsupported}\n")
					self.removeChild(c)
			elif c.nodeName == 'pose':
				"""
					<pose frame=''>0 0 0 0 0 -1.0471975512</pose>
				---
					<origin rpy="0 0 -1.0471975512" xyz="0 0 0"/>
				"""
				if self.nodeName == 'robot':
					self.removeChild(c)
				else:
					# Pose replaced with Origin
					origin = Item(self.createElement("origin"), parent=self, document=self._document, level=self._level+1, root=self.getRootNode())
					try:
						text = c.text().strip().split(' ')
					except:
						text = ['0', '0', '0', '0', '0', '0']
					origin.setAttribute("xyz", " ".join(text[:3]))
					origin.setAttribute("rpy", " ".join(text[3:]))
					self.replaceChild(origin, c)

			elif c.nodeName in ['parent', 'child', 'mass']:
				"""
					<nodeName>@text</nodeName>
				---
					<nodeName attr="@text"/>
				"""
				attr = dict(zip(
					('parent', 'child', 'mass'),
					('link',   'link',  'value'))).get(c.nodeName)
				if len(c._node.childNodes):
					c.setAttribute(attr, c.text().strip())
					cnodes = c._node.childNodes[:]
					for mc in cnodes:
						c._node.removeChild(mc)
				for mc in c._children:
					c.removeChild(mc)

			elif c.nodeName == 'mesh':
				mesh = c._node
				for mc in c._children[:]:
					if mc.nodeName == "scale":
						mesh.setAttribute("scale", mc.text())
					elif mc.nodeName == "uri":
						model = mc.text().strip()
						mesh.setAttribute("filename", model)
					else:
						sys.stderr.write("Ignored tagname {0} at level {1}: {2}".format(c.nodeName, self._level, mc.nodeName))
					c.removeChild(mc)

			elif c.nodeName == 'axis':
				"""
					<axis>
					  <xyz>0 -1 0</xyz>
					  <limit>
						<lower>0</lower>
						<upper>0</upper>
						<effort>100</effort>
						<velocity>-1</velocity>
					  </limit>
					  <dynamics>
						<damping>0.1</damping>
					  </dynamics>
					  <use_parent_model_frame>1</use_parent_model_frame>
					</axis>
				---
					<axis xyz="0 -1 0"/>
					<limit lower="0" upper="0" effort="100" velocity="-1"/>
				"""
				c.convert()
				for mc in c._children[:]:
					if mc.nodeName == 'xyz':
						c.setAttribute(mc.nodeName, mc._node.childNodes[0].toxml().strip())
					c.removeChild(mc)
					if mc.nodeName == 'limit':
						self.appendChild(mc)
					if mc.nodeName == 'dynamics':
						if mc.getAttribute('damping') or mc.getAttribute('friction'):
							self.appendChild(mc)
						else:
							c.removeChild(mc)
				_props = self._node.childNodes[:]
				for prop in _props:
					if prop.nodeName == 'origin':
						break
				else:
					origin = Item(self.createElement("origin"), parent=self, document=self._document, level=self._level+1, root=self.getRootNode())
					for name,value in c._node.attributes.items():
						origin.setAttribute(name, value)
					self.appendChild(origin)
					#self.removeChild(c)

		for c in self._children:
			# Fix broken tree
			if c._node.parentNode != self._node:
				self._node.appendChild(c._node)
				c._parent = self
			if c.nodeName == 'limit':
				c.convert()
			c.convert()

	def toxml(self):
		return self._node.toprettyxml(indent="  ")

	def dumptree(self):
		sys.stderr.write("{0}{1}: {2}({3})\n".format("  "*self._level, self._level, self.nodeName, len(self._children)))
		for c in self._children:
			c.dumptree()


def main():
	load_gazebo_setup()
	if len(sys.argv) == 1 or len(sys.argv) > 3 or 'help' in sys.argv or '--help' in sys.argv or '-h' in sys.argv:
		sys.stdout.write("""usage: {0} <input.sdf> [<output.urdf>]\n\n  By default content is printed to stdout.\n\n""".format(sys.argv[0]))
		exit(1)
	if len(sys.argv) == 3 and os.path.realpath(sys.argv[1]) == os.path.realpath(sys.argv[2]):
		raise Exception("Input and output filenames is the same file")
	xml = parse(None, sys.argv[1])
	output = sys.stdout
	if len(sys.argv) == 3:
		output = open(sys.argv[2], "w")

	doc = Item(xml)
	doc.convert()

	output.write(doc.toxml())

if __name__ == '__main__':
	main()

# vim: ts=4 sw=4 noet
