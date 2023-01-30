import sys

class GazeboMaterialItem(object):
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

	@property
	def level(self):
		return self._level

	@property
	def type(self):
		return self._type

	@property
	def parent(self):
		return self._parent

	def arg(self, idx):
		if idx < len(self._arguments):
			return self._arguments[idx]

	@property
	def args(self):
		return self._arguments

	@property
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
			if child.type == typeName and checkOpts(child):
				out.append(child)
		return out

	def __repr__(self):
		return 'GazeboMaterialItem<' + str(self) + '>'

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

# vim: ts=4 sw=4 noet
