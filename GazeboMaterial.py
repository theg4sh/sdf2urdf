import os
import sys
# gazebo's material syntax parsing
import pyparsing as gz

class GazeboItem(object):
	def __init__(self, typeName):
		self._type = typeName
		self._name = None
		self._level = -1
		self._parent = None
		self._arguments = []
		self._children = []
		self.__has_inheritance = False
		self._inherits_link = []
		self._inheritance = []

	def level(self):
		return self._level

	def type(self):
		return self._type

	def parent(self):
		return self._parent

	def arg(self, idx):
		if idx < len(self._arguments):
			return self._arguments[idx]

	def args(self):
		return self._arguments

	def name(self):
		return self._name

	def _setName(self, name):
		self._name = name

	def __fixChildLevels(self, child):
		if child._parent != self or child._level == self._level+1:
			return
		child._level = self._level+1
		for c2 in child._children:
			child.__fixChildLevels(c2)

	def _addChild(self, item):
		self._children.append(item)
		self.__fixChildLevels(item)
		
	def addChild(self, item):
		#if item not in self._children:
		item._parent = self
		self._addChild(item)
		return item

	def addArgument(self, arg):
		self._arguments.append(arg)

	def addInheritance(self, classname):
		self.__has_inheritance = True
		self._inheritance.append(classname)

	def findAll(self, typeName):
		opts = {}
		if '[' in typeName:
			typeName, vargs = typeName[:typeName.index('[')], typeName[typeName.index('[')+1:typeName.rindex(']')]
			buf = {}
			for o in vargs.split(','): # multiple opts
				varg = o.split('=',1)
				if len(varg)==1:
					opts[varg[0]] = True
				elif len(varg)==2:
					opts[varg[0]] = varg[1]
		def checkOpts(child):
			for k, v in opts.items():
				if not hasattr(child, k):
					return False
				attr = getattr(child, k)
				if v is True:
					if attr is None or (callable(attr) and attr() is None):
						return False
				else:
					if attr is None or (not callable(attr) and attr != v) or (callable(attr) and attr() != v):
						return False
			return True
		out = []
		for child in self._children[:]:
			if child.type() == typeName and checkOpts(child):
				out.append(child)
		return out

	def __str__(self):
		srep = ""
		if self._level>=0:
			prefix = "  "*(self._level)
			srep = "{0}{1}".format(prefix, self._type)
			if self._arguments:
				srep += " ".join(['']+self._arguments)
			if self._inheritance:
				if type(self._inheritance[0]) == str:
					srep += self._inheritance[0]
				else:
					srep += self._inheritance[0].name()
		if self._children:
			if self._level >= 0:
				srep += '\n%s{\n'%prefix
			for child in self._children:
				srep += child.__str__()
			if self._level >= 0:
				srep += '%s}'%prefix
			else:
				srep += '\n'
		else:
			if self._level == 0 and self.type() != 'import':
				srep += ' { }'
		return srep+'\n'

	def dumptree(self, filename=None, fwrite=None):
		if fwrite is None:
			fwrite = sys.stdout
			if filename is not None:
				fwrite = open(fp, "w")
			

		lvl = self._level+1
		namedef = '@root' if lvl == 0 else ("<"+self._type+"[{0}]>".format(len(self._arguments)))
		name = self._name if self._name is not None else namedef
		
		fwrite.write("{2}{0}: {1} +({3})\n".format(lvl, name, "  "*(lvl), len(self._children)))
		for child in self._children:
			child.dumptree(fwrite=fwrite)

class GazeboMaterialFile:
	def __init__(self, filename):
		self._filename = filename
		self._imports = [] # TODO
		self._parsed = False
		self._root = GazeboItem(None)

	def getFilename(self):
		return self._filename

	def _parse(self, content):
		gz.ParserElement.setDefaultWhitespaceChars(' \t')
		singleline_comment = "//" + gz.restOfLine
		multiline_comment  = gz.cStyleComment
		comments = gz.MatchFirst([singleline_comment, multiline_comment])

		real    = gz.Combine(gz.Optional(gz.oneOf("+ -")) + gz.Optional(gz.Word(gz.nums), default="0") +"." + gz.Word(gz.nums)).setName("real")
		integer = gz.Combine(gz.Optional(gz.oneOf("+ -")) + gz.Word(gz.nums)).setName("integer")
		nums    = real | integer

		words  = gz.Word(gz.alphas)
		string = gz.dblQuotedString()
		item_type = gz.Word(gz.alphas+"_")

		extensions = (gz.oneOf(["frag", "glsl", "jpeg", "jpg", "png", "vert"]))
		_filename = gz.ZeroOrMore(gz.Word(gz.alphanums+"_.") + '.') + gz.Word(gz.alphanums+"_")
		_filepath = gz.ZeroOrMore(gz.Word(gz.alphanums+"_.") + "/")
		filename = gz.Combine(_filename + '.' + extensions)
		filepath = gz.Combine(gz.Optional("/")+_filepath)
		fileany  = gz.Combine(filepath + filename)

		importall = gz.Literal("*")
		importheader = gz.Literal("import").setResultsName('itemtype') + \
				(importall | gz.Combine(gz.delimitedList(item_type, delim=",", combine=True))).setResultsName('imports') + \
				gz.Literal("from").suppress() + (string | words).setResultsName('from')

		lineend = gz.OneOrMore(gz.LineEnd()).suppress()
		oplineend = gz.Optional(lineend)

		blockstart,  blockend = gz.Literal("{").suppress(),  gz.Literal("}").suppress()

		blockname =  gz.Combine(gz.Optional(gz.Word(gz.alphas+"_") + gz.Literal("/")) + gz.Word(gz.alphanums+"_"))
		blockoption  = item_type.setResultsName('itemtype') + (gz.OneOrMore(fileany | nums | item_type)).setResultsName('arguments') + lineend
		blockinherit = gz.Literal(":").suppress() + blockname
		blockheader  = item_type.setResultsName('itemtype') + gz.Optional(blockname).setResultsName('blockname') + \
					gz.Group(gz.Optional(gz.OneOrMore(item_type))).setResultsName('arguments') + \
					gz.Group(gz.Optional(blockinherit)).setResultsName('inheritance')

		blockinner = gz.Forward()
		blockinner << gz.Group(item_type.setResultsName('itemtype') + gz.Optional(blockname).setResultsName('blockname') + gz.ZeroOrMore(blockname).setResultsName('arguments') + oplineend + \
					blockstart + oplineend + \
					gz.ZeroOrMore(blockinner ^ gz.Group(blockoption)).setResultsName('blockbody') + \
					oplineend + blockend) + oplineend
	
		block = gz.Group(blockheader + oplineend + \
					blockstart + oplineend + \
					gz.ZeroOrMore(blockinner ^ gz.Group(blockoption)).setResultsName('blockbody') + \
					oplineend + blockend) + oplineend
	
		allitems = gz.ZeroOrMore(gz.Group(importheader) + lineend) + gz.ZeroOrMore(block) + oplineend
		allitems.ignore(comments)


		def makeBlock(token, level=0):
			tkeys = token.keys()
			if 'itemtype' in tkeys:
				item = GazeboItem(token['itemtype'])
			else:
				raise Exception("Cannot found itemtype in {0}".format(token))

			if 'blockname' in tkeys:
				item._setName(token['blockname'])
				#item.addArgument(token['blockname'])
			if 'arguments' in tkeys:
				for xarg in token['arguments']:
					item.addArgument(xarg)
			if 'inheritance' in tkeys:
				for xarg in token['inheritance']:
					item.addInheritance(xarg)

			if 'blockbody' in tkeys:
				for child in token['blockbody']:
					if type(child) != str:
						item.addChild(makeBlock(child, level=level+1))
					else:
						raise Exception("Failured while parsing blockbody", child)
			
			return item
			
		for tokens,start,end in allitems.scanString(content):
			for t in tokens:
				self._root.addChild(makeBlock(t))
		self._parsed = True

	def _do_inherit(self):
		if not self._parsed:
			raise Exception("Inheritance processing should be run after parser complete")

		def makechildmap(node):
			childmap = {}
			for child in node._children:
				if child.name() is None:
					continue
				if child.type() not in childmap:
					childmap[child.type()] = {}
				childmap[child.type()][child.name()] = child
			return childmap
		childmap = makechildmap(self._root)

		def makeblocks(node):
			blocks = {}
			for child in node._children:
				bid = child.type()
				if child.name() is not None:
					bid += ":"+child.name()
				if bid in blocks:
					raise Exception("Item is exists {0}".format(bid))
				blocks[bid] = child
			return blocks
		def inherit(nodeto, nodefrom, copy=False):
			# TODO: make copy of inherited properties
			bt, bf = makeblocks(nodeto), makeblocks(nodefrom)
			for bid, fchild in bf.items():
				if bid not in bt: # Item full inheritance
					nodeto._addChild(fchild)
				else: # Partial interitance
					fchildren = nodefrom.findAll('{0}'.format(fchild.type()))
					tchildren = nodeto.findAll('{0}'.format(fchild.type()))
					tchk = {}
					for tf in tchildren:
						tchk["%s:%s"%(tf.type(), tf.name())] = tf
					for fc in fchildren:
						uid = "%s:%s"%(fc.type(), fc.name())
						if uid in tchk:
							inherit(tchk[uid], fc)
							continue
						else:
							nodeto._addChild(fchild)

		for typeName, cm in childmap.items():
			for name, child in cm.items():
				inheritance = []
				if not child._inheritance:
					continue
				child._inheritance[:]
				for inh in child._inheritance[:]:
					if cm.get(inh) is not None:
						childinh = cm.get(inh)
						inheritance.append(cm.get(inh))
					else:
						raise Exception("{0} type {1} inherits from undeclared {2}".format(name, typeName, inh))
				child._inheritance = inheritance
				for inh in inheritance:
					if child not in inh._inherits_link:
						inh._inherits_link.append(child)
					inherit(child, inh)

	def parse(self, filename=None):
		if filename is None:
			return self.parse(self._filename)
		else:
			if filename == self._filename and self._parsed:
				return
		self._filename = filename
		with open(filename, "r") as f:
			content = f.read()
		self._parse(content)
		self._do_inherit()

	def find(self, query):
		self.parse()
		def split_query(query):
			level = 0
			items = []
			idx, idy = 0, 0
			has_string = False
			for c in query:
				if level == 0 and c == '.':
					items.append(query[idx:idy])
					idx, idy = idy+1, idy
				elif c == '[' or (not has_string and c == '"'):
					if (not has_string and c == '"'):
						has_string = True
					level+=1
				elif c == ']' or (has_string and c == '"'):
					if (has_string and c == '"'):
						has_string = False
					level-=1
				idy+=1
			items.append(query[idx:])
			return items

		path = split_query(query)
		items = self._root.findAll(path.pop(0))
		while len(path):
			buf, p = [], path.pop(0)
			for it in items:
				buf += it.findAll(p)
			items = buf
		return items
					
	def getColor(self, name):
		self.parse()
		return self.find('material[name={0}].technique.pass.ambient'.format(name))
			

# Example content
_content = """
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


if __name__ == "__main__":
	gzMaterial = GazeboMaterialFile("/usr/share/gazebo-8/media/materials/scripts/gazebo.material");
	c1, c2 = gzMaterial.getColor('Gazebo/Grey')[0], gzMaterial.getColor('Gazebo/Gray')[0]
	print(str(" ".join(c1.args())))
