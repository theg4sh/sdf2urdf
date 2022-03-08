import os
import sys
# gazebo's material syntax parsing
import pyparsing as gz

from GazeboMaterial.GazeboMaterialItem import *

def _build_tokenizer():
	gz.ParserElement.setDefaultWhitespaceChars(' \t')
	singleline_comment = "//" + gz.restOfLine
	multiline_comment  = gz.cStyleComment
	comments = gz.MatchFirst([singleline_comment, multiline_comment])

	real    = gz.Combine(gz.Optional(gz.oneOf("+ -")) + gz.Optional(gz.Word(gz.nums)) +"." + gz.Word(gz.nums)).setName("real")
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

	return allitems

def _parse_query(query):
	# FIXME: parse query with pyparsing aka local `gz`
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


class GazeboMaterialFile:
	def __init__(self, filename):
		self._filename = filename
		self._imports = [] # TODO
		self._parsed = False
		self._root = GazeboMaterialItem(None)
		self._tokenizer = _build_tokenizer()

	def getFilename(self):
		return self._filename

	def _parse(self, content):
		def makeBlock(token, level=0):
			tkeys = list(token.keys())
			if 'itemtype' in tkeys:
				item = GazeboMaterialItem(token['itemtype'])
			else:
				raise Exception(f"Cannot found itemtype in {oken}")

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

		for tokens,start,end in self._tokenizer.scanString(content):
			for t in tokens:
				self._root.addChild(makeBlock(t))
		self._parsed = True

	def _do_inherit(self):
		if not self._parsed:
			raise Exception("Inheritance processing should be run after parser complete")

		def makechildmap(node):
			childmap = {}
			for child in node._children:
				if child.name is None:
					continue
				if child.type not in childmap:
					childmap[child.type] = {}
				childmap[child.type][child.name] = child
			return childmap
		childmap = makechildmap(self._root)

		def makeblocks(node):
			blocks = {}
			for child in node._children:
				bid = child.type
				if child.name is not None:
					bid += ":"+child.name
				if bid in blocks:
					raise Exception(f"Item is exists {bid}")
				blocks[bid] = child
			return blocks

		def inherit(nodeto, nodefrom, copy=False):
			# TODO: make copy of inherited properties
			bt, bf = makeblocks(nodeto), makeblocks(nodefrom)
			for bid, fchild in bf.items():
				if bid not in bt: # Item full inheritance
					nodeto._addChild(fchild)
				else: # Partial interitance
					fchildren = nodefrom.findAll(f'{fchild.type}')
					tchildren = nodeto.findAll(f'{fchild.type}')
					tchk = {}
					for tf in tchildren:
						tchk[f"{tf.type}:{tf.name}"] = tf
					for fc in fchildren:
						uid = f"{fc.type}:{fc.name}"
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
						raise Exception("{name} type {typeName} inherits from undeclared {inh}")
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
		path = _parse_query(query)
		items = self._root.findAll(path.pop(0))
		while len(path):
			buf, p = [], path.pop(0)
			for it in items:
				buf += it.findAll(p)
			items = buf
		return items

	def getColor(self, name):
		return self.find('material[name={0}].technique.pass.ambient'.format(name))

# vim: ts=4 sw=4 noet
