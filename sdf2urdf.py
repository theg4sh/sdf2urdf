#!/usr/bin/python2

import os
import sys
# GazeboMaterial
sys.path.append(os.path.dirname(os.path.realpath(sys.argv[0])))
# xml operations
from xacro import parse
from xacro.xmlutils import *

from GazeboMaterial import GazeboMaterialFile

rospack = None

class Item:
	"""
['ATTRIBUTE_NODE', 'CDATA_SECTION_NODE', 'COMMENT_NODE', 'DOCUMENT_FRAGMENT_NODE', 'DOCUMENT_NODE', 'DOCUMENT_TYPE_NODE', 'ELEMENT_NODE', 'ENTITY_NODE', 'ENTITY_REFERENCE_NODE', 'NOTATION_NODE', 'PROCESSING_INSTRUCTION_NODE', 'TEXT_NODE', '__doc__', '__init__', '__module__', '__nonzero__', '__repr__', '_attrs', '_attrsNS', '_call_user_data_handler', '_child_node_types', '_get_attributes', '_get_childNodes', '_get_firstChild', '_get_lastChild', '_get_localName', '_get_nodeName', '_magic_id_nodes', 'appendChild', 'attributes', 'childNodes', 'cloneNode', 'firstChild', 'getAttribute', 'getAttributeNS', 'getAttributeNode', 'getAttributeNodeNS', 'getElementsByTagName', 'getElementsByTagNameNS', 'getInterface', 'getUserData', 'hasAttribute', 'hasAttributeNS', 'hasAttributes', 'hasChildNodes', 'insertBefore', 'isSameNode', 'isSupported', 'lastChild', 'localName', 'namespaceURI', 'nextSibling', 'nodeName', 'nodeType', 'nodeValue', 'normalize', 'ownerDocument', 'parentNode', 'prefix', 'previousSibling', 'removeAttribute', 'removeAttributeNS', 'removeAttributeNode', 'removeAttributeNodeNS', 'removeChild', 'replaceChild', 'schemaType', 'setAttribute', 'setAttributeNS', 'setAttributeNode', 'setAttributeNodeNS', 'setIdAttribute', 'setIdAttributeNS', 'setIdAttributeNode', 'setUserData', 'nodeName', 'toprettyxml', 'toxml', 'unlink', 'writexml']
	"""
	def __init__(self, xmlnode, parent=None, document=None, level=0):
		self._document = xmlnode if xmlnode.nodeName == '#document' else document
		self._node = xmlnode
		self._parent = parent
		self._level = level
		self._children = []
		self._textvalue = None

		if (len(self._node.childNodes) == 1) and (self._node.childNodes[0].nodeName == '#text'):
			self._textvalue = Item(self._node.childNodes[0], document=self._document, parent=self, level=level+1)
			return
		for n in self._node.childNodes[:]: 
			# Skip pretty tabs
			if n.nodeName == '#text' and n.toxml().strip() == '':
				self._node.removeChild(n)
				continue
			self.appendChild(Item(n, parent=self, document=self._document, level=level+1))

	def appendChild(self, item):
		if item.__class__.__name__ == 'Item':
			if item._node.parentNode is not None and item._node.parentNode != self._node:
				self._node.appendChild(item._node)
			if item in item._parent._children:
				del item._parent._children[item._parent._children.index(item)]
			if item not in self._children:
				self._children.append(item)
		else:
			item = Item(item, parent=self, document=self._document, level=self._level+1)
			self._children.append(item)

	def setAttribute(self, name, value):
		self._node.setAttribute(name, value)

	def getAttribute(self, name):
		return self._node.getAttribute(name)

	def text(self):
		return self._node.childNodes[0].toxml()

	def nodeName(self):
		return self._node.nodeName

	def removeChild(self, item):
		pn = item._node.parentNode
		node = item._node
		while pn is not None and pn != self._node.parentNode:
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
		return Item(self._node.cloneNode(True), parent=None, document=self._document, level=self._level)

	def createElement(self, *args, **kwargs):
		try:
			return self._document.createElement(*args, **kwargs)
		except Exception as e:
			sys.stderr.write("{0}\n".format(self._node))
			raise e

	def isPlugin(self):
		return self

	def isSDF(self):
		return self.nodeName() == 'sdf' and self._level == 1

	def uriToPath(self, uri):
		parts = uri.encode("utf-8").split('/')
		uritype, _, parts = parts[0], None, parts[2:]
		
		searchpath = []
		typemap = { "model:": ["GAZEBO_MODEL_PATH"], 
			    "file:":  ["GAZEBO_RESOURCE_PATH"] }.get(uritype)
		if typemap is None:
			return None

		for env in typemap:
			if os.environ.get(env) is not None:
				searchpath += os.environ.get(env).encode("utf-8").strip(":").split(':')

		for gmp in searchpath:
			path = os.path.join(gmp, *parts)
			if os.path.exists(path):
				return path
		return None

	_gazeboMaterialFiles = {}
	def _getGazeboMaterial(self, node):
		name, gzMaterial = None, None
		
		for opt in node._children:
			if opt.nodeName() == 'name':
				name = opt.text()
				if not name.startswith('Gazebo/'):
					raise Exception("Unsupported color NS: {0}".format(name))
			elif opt.nodeName() == 'uri':
				materialpath = self.uriToPath(opt.text())
				if materialpath is None:
					raise Exception("Cannot find material reference in GAZEBO_MODELS_PATH and GAZEBO_RESOURCE_PATH: {0}".format(opt.text()))
				if self.__class__._gazeboMaterialFiles.get(materialpath) is None:
					self.__class__._gazeboMaterialFiles[materialpath] = GazeboMaterialFile(materialpath)
				gzMaterial = self.__class__._gazeboMaterialFiles[materialpath]
			else:
				raise Exception("Unknown material/script tag: {0}".format(opt.nodeName()))
		return name, gzMaterial
		

	def convert(self):
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
				node = Item(self.createElement('robot'), parent=self, document=self._document, level=self._level+1)
				copy = c.cloneNode()
				for ch in self._children:
					self.removeChild(ch)
				for ch in self._node.childNodes:
					self._node.removeChild(ch)
				
				for name, attr in copy._node.childNodes[0].attributes.items():
					node._node.setAttribute(name, attr)

				for ch in copy._node.childNodes[0].childNodes[:]:
					node._node.appendChild(ch)
					item = Item(ch, parent=self, document=self._document, level=self._level+1)
					node.appendChild(item)
				#self.replaceChild(node, copy)
				self.appendChild(node)
				
			# Inner tags into attributes, nothing else
			elif c.nodeName() in ['inertia', 'cylinder', 'sphere', 'box', 'limit'] :
				for mc in c._children[:]:
					c.setAttribute(mc.nodeName(), mc.text().strip())
					c.removeChild(mc)

			# Unused elements
			elif c.nodeName() in ['plugin', 'physics', 'gravity', 'velocity_decay', 'self_collide', 'surface', 'static', '#comment']:
				self.removeChild(c)

			elif c.nodeName() == 'material':
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
				if len(c._children) == 1 and c._children[0].nodeName() == 'script': 
					name, gzMaterial = self._getGazeboMaterial(c._children[0])
					c.removeChild(c._children[0])
					if name:
						c.setAttribute("name", name)
					materials = gzMaterial.getColor(name)
					if materials:
						color = Item(self.createElement('color'), parent=c, document=self._document, level=c._level+1)
						color.setAttribute("rgba", " ".join(materials[0].args()))
						c.appendChild(color)
					else:
						raise Exception("Material not found ({1}) at {2}".format('material[name='+name+'].technique.pass.ambient', gzMaterial.getFilename()))
				else:
					raise Exception("Unsupported material subtag: {0}".format(c._children[0].nodeName() if len(c._children) else "<None>"))
			elif c.nodeName() == 'pose':
				"""
        				<pose frame=''>0 0 0 0 0 -1.0471975512</pose>
				---
					<origin rpy="0 0 -1.0471975512" xyz="0 0 0"/>
				"""
				# Pose replaced with Origin
				origin = Item(self.createElement("origin"), parent=self, document=self._document, level=self._level+1)
				text = c.text().strip().split(' ')
				origin.setAttribute("xyz", " ".join(text[:3]))
				origin.setAttribute("rcy", " ".join(text[3:]))
				#origin.appendChild(
				self.replaceChild(origin, c)

			elif c.nodeName() in ['parent', 'child', 'mass']:
				"""
					<nodeName>@text</nodeName>
				---
					<nodeName attr="@text"/>
				"""
				attr = dict(zip(
					('parent', 'child', 'mass'),
					('link',   'link',  'value'))).get(c.nodeName())
				if len(c._node.childNodes):
					c.setAttribute(attr, c.text().strip())
				for mc in c._children:
					c.removeChild(mc)

			elif c.nodeName() == 'mesh':
				mesh = c._node
				for mc in c._children[:]:
					if mc.nodeName() == "scale":
						mesh.setAttribute("scale", mc.text())
					elif mc.nodeName() == "uri":
						model = mc.text().strip()
						mesh.setAttribute("filename", "file://"+self.uriToPath(model))
					else:
						sys.stderr.write("Ignored tagname {0} at level {1}: {2}".format(c.nodeName(), self._level, mc.nodeName()))
					c.removeChild(mc)

			elif c.nodeName() == 'axis':
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
					<axis xyx="0 -1 0"/>
					<limit lower="0" upper="0" effort="100" velocity="-1"/>
				"""
				c.convert() # Fix limit internals tag2attr
				for mc in c._children[:]:
					if mc.nodeName() == 'xyz':
						c.setAttribute(mc.nodeName(), mc._node.childNodes[0].toxml().strip())
					c.removeChild(mc)
					if mc.nodeName() == 'limit':
						self.appendChild(mc);
				
						

		for c in self._children:
			# Fix broken tree
			if c._node.parentNode != self._node:
				self._node.appendChild(c._node)
				c._parent = self
			if c.nodeName() == 'limit':
				c.convert()
			c.convert()

	def toxml(self):
		return self._node.toprettyxml(indent="  ")
	def dumptree(self):
		sys.stderr.write("{0}{1}: {2}({3})\n".format("  "*self._level, self._level, self.nodeName(), len(self._children)))
		for c in self._children:
			c.dumptree()


def main():
	if len(sys.argv) == 1 or len(sys.argv) > 3 or 'help' in sys.argv or '--help' in sys.argv or '-h' in sys.argv:
		sys.stdout.write("""usage: {0} <input.sdf> [<output.urdf>]""".format(sys.argv[0]))
		exit(1)
	xml = parse(None, sys.argv[1])
	output = sys.stdout
	if len(sys.argv) == 3:
		output = open(sys.argv[2], "w")

	doc = Item(xml)
	doc.convert()

	output.write(doc.toxml())

if __name__ == '__main__':
	main()
