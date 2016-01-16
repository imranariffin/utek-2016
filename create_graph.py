import networkx as nx
import shapefile as sp


def get_intersections_coordinates(filename):
	shapes = sp.Reader("./inputfiles/" + filename).shapes()
	intersections_coordinates = [shape.__geo_interface__ for shape in shapes]
	return intersections_coordinates

def parser(filename):
	print "parser"

def get_IntersectionsRecords():
	myshp = open('./inputfiles/CENTRELINE_INTERSECTION_WGS84.shp', 'rb')
	mydbf = open('./inputfiles/CENTRELINE_INTERSECTION_WGS84.dbf', 'rb')
	r = sp.Reader(shp=myshp, dbf=mydbf)
	return r
def get_StreetRecords():
	myshp = open('./inputfiles/CENTRELINE_WGS84.shp', 'rb')
	mydbf = open('./inputfiles/CENTRELINE_WGS84.dbf', 'rb')
	r = sp.Reader(shp=myshp, dbf=mydbf)
	return r

class Records(object):
	def __init__(self):
		self.IntersectionsRecords = get_IntersectionsRecords()
		self.StreetRecords = get_StreetRecords()
	def get_intersection_shapes(self):
		return self.IntersectionsRecords.shapes()
	def get_intersection_records(self):
		return self.IntersectionsRecords.records()
	def get_intersection_fields(self):
		return self.IntersectionsRecords.fields
	def get_points_by_id(self, id):
		assert(type(id)==type(int()))
		records = self.IntersectionsRecords.records()
		for record in records:
			if record[0] == id:
				# return shape.__geo_interface__['coordinates']
				return record
	def generate_intersections(self):
		self.intersections = {}
		records = self.IntersectionsRecords.records()
		for record in records:
			intersection = {}
			fields = self.IntersectionsRecords.fields[1:]
			i = 0
			INT_ID = fields[0][0]
			for field in fields:
				# print "field: " + str(field)
				intersection[field[0]] = record[i]
				i += 1
			self.intersections[intersection[INT_ID]] = intersection

	def generate_streets(self):

		valid_fcodes = {
			201200: "Major Arterial Road",
			201201:	"Major Arterial Road Ramp",
			201300:	"Minor Arterial Road",
			201301:	"Minor Arterial Road Ramp",
			201400:	"Collector Road",
			201401:	"Collector Road Ramp",
			201500:	"Local Road",
			201600:	"Other Road",
			201601:	"Other Ramp",
			201700:	"Laneways",
			201800: "Pending"
		}

		self.streets = {}
		records = self.StreetRecords.records()
		for record in records:
			street = {}
			fields = self.StreetRecords.fields[1:]
			i = 0
			GEO_ID = fields[0][0]
			for field in fields:
				# print "field: " + str(field)
				street[field[0]] = record[i]
				i += 1
			assert(type(street['FCODE'])==type(int()))
			if street["FCODE"] in valid_fcodes.keys():
				self.streets[street[GEO_ID]] = street

	def generate_graph(self):
		# TEST uncomment for real deal
		self.generate_intersections()
		self.generate_streets()

		self.G = nx.Graph()
		# add intersections
		# for INT_ID in self.intersections:
		# 	intersection = self.intersections[INT_ID]
		# 	self.G.add_node(intersection['INT_ID'])
		# 	# self.G.add_node(intersection)
		# add edges
		for GEO_ID in self.streets:
			edge = self.streets[GEO_ID]
			self.G.add_edge(edge['FNODE'], edge['TNODE'])

	def find_distance(self, from_node_id, to_node_id):
		from math import acos, cos, sin, pi

		# get coordinates
		from_coordinates = {
			"lat":self.intersections[from_node_id]['LATITUDE'], 
			"lon":self.intersections[from_node_id]['LONGITUDE']
		}
		to_coordinates = {
			"lat":self.intersections[to_node_id]['LATITUDE'], 
			"lon":self.intersections[from_node_id]['LONGITUDE']
		}

		# calculate
		R = 6371000 # Earth's radius
		diff_lon = abs(from_coordinates["lon"]-to_coordinates["lon"])
		central_angle = acos(\
			sin(from_coordinates["lat"])*sin(to_coordinates["lat"]) + \
			cos(from_coordinates["lat"])*cos(to_coordinates["lat"])*cos(diff_lon))
		distance = R*central_angle

		return distance
	
	def get_neighbors(self, node_id):
		"""
		return type: list of ints
		"""
		return nx.all_neighbors(self.G, node_id)

	def Djikstra(self, start):
		import heapq, sys
		distances = {start : 0}
		paths = {start : None}
		visited_set = set()
		previouses = {}

		# set to infinite the distance from start to every other node
		for node in self.G.nodes():
			if node != start:
				distances[node] = sys.maxint
				paths[node] = [node]

		unvisited_queue = [(distances[node], node) for node in self.G.nodes()]
		heapq.heapify(unvisited_queue)

		while len(unvisited_queue):

			curr = heapq.heappop(unvisited_queue)
			curr_node = curr[1]
			visited_set.add(curr_node)

			for next_node in self.get_neighbors(curr_node):
				if next_node in visited_set:
					continue
				new_distance = distances[curr_node] + self.find_distance(curr_node, next_node)

				if new_distance < distances[next_node]:
					distances[next_node] = new_distance
					# set previous
					previouses[next_node] = curr_node

			# 1. rebuild heap
			while len(unvisited_queue):
				heapq.heappop(unvisited_queue)
			# 2. Put all vertices not visited into the queue
			unvisited_queue = [(distances[node], node) for node in self.G.nodes() if not visited_set.__contains__(node)]
        	heapq.heapify(unvisited_queue)      
		# self.distances = dict()
		# self.distances[start] = distances
		# self.previouses = dict()
		# self.distances[start] = previouses
		return (distances, paths)
		# return True


def test_distances(filename):
	f = open(filename)
	lines = f.readlines()
	N = int(lines.pop(0))

	R = Records()
	R.generate_graph()
	for node in R.G.nodes():
		R.Djikstra(node)
		print "done Djikstra(%)"%node

	for i in [2*x for x in range(N)]:
		from_node_id = int(lines[i])
		to_node_id = int(lines[i+1])
		# print from_node_id, to_node_id
		print G.distances[from_node_id][to_node_id]

def test_djikstra():
	# TEST Djikstra with a small graph
	R = Records()
	R.streets = {
		1: {"GEO_ID" : 1, "FNODE" : 1, "TNODE" : 2},
		2: {"GEO_ID" : 2, "FNODE" : 1, "TNODE" : 3},
		3: {"GEO_ID" : 3, "FNODE" : 2, "TNODE" : 4},
		4: {"GEO_ID" : 4, "FNODE" : 3, "TNODE" : 4},
		5: {"GEO_ID" : 5, "FNODE" : 4, "TNODE" : 5},
		6: {"GEO_ID" : 6, "FNODE" : 5, "TNODE" : 1}
	}
	R.intersections = {
		1: {"INT_ID" : 1, "LONGITUDE" : 105, "LATITUDE" : 77},
		2: {"INT_ID" : 2, "LONGITUDE" : 100, "LATITUDE" : 75},
		3: {"INT_ID" : 3, "LONGITUDE" : 105, "LATITUDE" : 77},
		4: {"INT_ID" : 4, "LONGITUDE" : 100, "LATITUDE" : 76.9},
		5: {"INT_ID" : 5, "LONGITUDE" : 101, "LATITUDE" : 77.9},
		6: {"INT_ID" : 6, "LONGITUDE" : 102, "LATITUDE" : 79.9}
	}
	R.generate_graph()

	print R.G.nodes()
	print R.intersections
	print R.G.edges()
	print R.streets
	print "Djikstra:"
	# print R.Djikstra(1)
	distances = {}
	for node in R.G.nodes():
		distances[node] = R.Djikstra(node)
		print distances[node]

def populate_graph(filename):
	"""
	input type		: String
	output type		: networkx Graph

	given input file name return a graph that contains
	the edges in the input file
	"""
	input_file = open(filename)
	lines = input_file.readlines()

	n_nodes = lines.pop(0)
	n_nodes = int(n_nodes)

	steps = [2*i for i in range(n_nodes)]
	for i in steps:
		node1 = int(lines[i])
		node2 = int(lines[i+1])
		print str(node1) + ", " + str(node2)

	G = nx.Graph()
	# populate using edges
	for i in steps:
		node1 = int(lines[i])
		node2 = int(lines[i+1])
		G.add_edge(node1, node1)

	return G


if __name__=="__main__":

	# R = Records()
	# # generate graph
	# R.generate_graph()
	# print R.G.nodes()[:10]
	# print R.G.edges()[:10]
	# print R.G.size()

	# print R.find_distance(14024724, 14024725)

	# print R.Djikstra(14024724)

	# # draw graph
	# import matplotlib.pyplot as plt

	# test_djikstra()

	test_distances("input11_short")

	# nx.draw(R.G)
	# plt.show()