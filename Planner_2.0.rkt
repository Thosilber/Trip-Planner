#lang dssl2
let eight_principles = ["Know your rights.",
"Acknowledge your sources.",
"Protect your work.",
"Avoid suspicion.",
"Do your own work.",
"Never falsify a record or permit another person to do so.",
"Never fabricate data, citations, or experimental results.",
"Always tell the truth when discussing your work with your instructor."]

# Final project: Trip Planner

import cons
import sbox_hash
import 'project-lib/dictionaries.rkt'
import 'project-lib/graph.rkt'
import 'project-lib/binheap.rkt'
import 'project-lib/stack-queue.rkt'


### Basic Types ###

#  - Latitudes and longitudes are numbers:
let Lat?  = num?
let Lon?  = num?

#  - Point-of-interest categories and names are strings:
let Cat?  = str?
let Name? = str?

### Raw Entity Types ###

#  - Raw positions are 2-element vectors with a latitude and a longitude
let RawPos? = TupC[Lat?, Lon?]

#  - Raw road segments are 4-element vectors with the latitude and
#    longitude of their first endpoint, then the latitude and longitude
#    of their second endpoint
let RawSeg? = TupC[Lat?, Lon?, Lat?, Lon?]

#  - Raw points-of-interest are 4-element vectors with a latitude, a
#    longitude, a point-of-interest category, and a name
let RawPOI? = TupC[Lat?, Lon?, Cat?, Name?]

### Contract Helpers ###

# ListC[T] is a list of `T`s (linear time):
let ListC = Cons.ListC
# List of unspecified element type (constant time):
let List? = Cons.list?


interface TRIP_PLANNER:

    # Returns the positions of all the points-of-interest that belong to
    # the given category.
    def locate_all(
            self,
            dst_cat:  Cat?           # point-of-interest category
        )   ->        ListC[RawPos?] # positions of the POIs

    # Returns the shortest route, if any, from the given source position
    # to the point-of-interest with the given name.
    def plan_route(
            self,
            src_lat:  Lat?,          # starting latitude
            src_lon:  Lon?,          # starting longitude
            dst_name: Name?          # name of goal
        )   ->        ListC[RawPos?] # path to goal

    # Finds no more than `n` points-of-interest of the given category
    # nearest to the source position.
    def find_nearby(
            self,
            src_lat:  Lat?,          # starting latitude
            src_lon:  Lon?,          # starting longitude
            dst_cat:  Cat?,          # point-of-interest category
            n:        nat?           # maximum number of results
        )   ->        ListC[RawPOI?] # list of nearby POIs


        
        
#-------------------------------------------------------------------------------------------------------#
        
        
# useful structs
                
struct position:
    let lat: num?
    let long: num?
    
struct road_segment:
    let pos1
    let pos2 
    
struct poi:                 
    let lat
    let long                     
    let cat                 # the category (string)
    let name                # the name (string) 
    

    
    
    
# some helper procedures
    
def distance_euclidean(x1, y1, x2, y2):
    let distance = ((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2)).sqrt()
    return distance

def heap_sort[X](v: VecC[X], lt?: FunC[X, X, bool?]) -> NoneC:
    let counter = 0
    let h = BinHeap[X](v.len(), lt?)  
    for i in range(v.len()):
        h.insert(v[i])
    while h.len() > 0:
        v[counter] = h.find_min()
        h.remove_min()
        counter = counter + 1
    
        
# dijkstra's algorithm
def dijkstra(graph, start):
    

    let dist = [inf; graph.len()]
    let pred = [None; graph.len()]
    let done = [False; graph.len()]

    def shorter? (path1, path2):
        return dist[path1] < dist[path2]
        
    let todo = BinHeap(2*graph.len(), shorter?)
    dist[start] = 0
    todo.insert(start)


    while todo.len() > 0:
        let i = todo.find_min()
        todo.remove_min()
    
        if not done[i]:
            done[i] = True
            let current = graph.get_adjacent(i)
        
            while current is not None:
                let adj_vertex = current.data
                let weight = graph.get_edge(i, adj_vertex)
            
                if dist[i] + weight < dist[adj_vertex]:
                    dist[adj_vertex] = dist[i] + weight
                    pred[adj_vertex] = i
                    todo.insert(adj_vertex)
                
                current = current.next
            
        
    return [pred, dist]
    
    
#-------------------------------------------------------------------------------------------------------#
   
    
class TripPlanner (TRIP_PLANNER):
    let graph
    let nodeID_to_position
    let position_to_nodeID
    let name_to_POIstruct
    let roads
    let POIs
    let nodeID_to_POIstruct
    
    
#-------------------------------------------------------------------------------------------------------#    
    
    
# INITIALIZING
    
    def __init__(self, roads, POIs):
       
        self.roads = roads
       
        self.POIs = POIs
        
       
        
        
       
        let all_nodes = AssociationList()                                         # as some positions might have 2 or more POIs, I am making a dictionary
        let count = 0                                                             # for all positions so those can be the nodes of my graph (with no duplicates)
        
       
        for x in POIs:                                                            # for loop to check and and POI positions to association list
           let POI_location = position(x[0], x[1])                  
           
           if not all_nodes.mem?(POI_location):                                   # if not in list, put
               all_nodes.put(POI_location, count)
               count = count + 1
                
        
        for x in roads:                                                           # for loop to check and add roads to association list
            let road = road_segment(position(x[0], x[1]), position(x[2], x[3]))   # making a road with struct road segment that has initial position and final position
            
            if not all_nodes.mem?(road.pos1):                                     # if not in list, put
                all_nodes.put(road.pos1, count)
                count = count + 1
                
            if not all_nodes.mem?(road.pos2):                                     # if not in list, put
                all_nodes.put(road.pos2, count)
                count = count + 1
         
                
            let graph_size = all_nodes.len()                                      # size of graph is the length of the association list with all nodes (no duplicates)
            self.graph = WuGraph(graph_size)                                      # weighted undirected graph with all positions as nodes (will be updatedd later)
            
            
            
        
        self.nodeID_to_position = AssociationList() # maybe change to vector for better performance
            
        self.position_to_nodeID = HashTable(all_nodes.len() * 2, make_sbox_hash())
        
        self.name_to_POIstruct = HashTable(all_nodes.len(), make_sbox_hash())
       
        self.nodeID_to_POIstruct = vec(all_nodes.len())
        
        
      
        let count2 = 0
        
        
        # adding roads
        
        for x in roads:
            let road = road_segment(position(x[0], x[1]), position(x[2], x[3]))
            
            if not self.position_to_nodeID.mem?(road.pos1):
                self.position_to_nodeID.put(road.pos1, count2)
                self.nodeID_to_position.put(count2, road.pos1)
                count2 = count2 + 1
                
            if not self.position_to_nodeID.mem?(road.pos2):
                self.position_to_nodeID.put(road.pos2, count2)
                self.nodeID_to_position.put(count2, road.pos2)
                count2 = count2 + 1
                
            let weight = distance_euclidean(road.pos1.lat, road.pos1.long, road.pos2.lat, road.pos2.long)  # distance ==> weigth on weighted graph
            
            # making graph with positions
            self.graph.set_edge(self.position_to_nodeID.get(road.pos1), self.position_to_nodeID.get(road.pos2), weight)
         
                
                
        # adding POIs
            
        for x in POIs:
            let poi_pos = position(x[0], x[1])
            let POI = poi(x[0],x[1], x[2], x[3])
            
            if not self.position_to_nodeID.mem?(poi_pos):
                self.position_to_nodeID.put(poi_pos, count2)
                self.nodeID_to_position.put(count2, poi_pos)
                                
            self.name_to_POIstruct.put(POI.name, POI)
        
            
            self.nodeID_to_POIstruct[self.position_to_nodeID.get(poi_pos)] = cons(POI, self.nodeID_to_POIstruct[self.position_to_nodeID.get(poi_pos)])
      
                    
    def show_dicts(self):  # function to show dicts - used to test if all of them were correct
         return [self.name_to_POIstruct]
 
#-------------------------------------------------------------------------------------------------------#            
            
            
# LOCATE ALL
    def locate_all(self, poi_category):
       
        let pos_list = None                                                  
        let remove_dup_dict = HashTable(self.POIs.len(), make_sbox_hash())   # hash table to keep track of duplicates
        
        for p in self.POIs:
            if poi_category == p[2] and not remove_dup_dict.mem?([p[0], p[1]]):
                remove_dup_dict.put([p[0], p[1]], "this is useless")
                pos_list = cons([p[0], p[1]], pos_list)
                
        return pos_list
                
     
#-------------------------------------------------------------------------------------------------------#        
    
               
# PLAN ROUTE
                
    def plan_route(self, starting_lat, starting_long, poi_name):
        
        let path = None
        
        # "easy" case
        if not self.name_to_POIstruct.mem?(poi_name) or not self.position_to_nodeID.mem?(position(starting_lat, starting_long)):
            return path
        
        let start = self.position_to_nodeID.get(position(starting_lat, starting_long))
        
        let POI_destination = self.name_to_POIstruct.get(poi_name)
            
        let node_destination = self.position_to_nodeID.get(position(POI_destination.lat, POI_destination.long))
        
        let dijkstra_result = dijkstra(self.graph, start)         # running dijkstra to find
        let pred = dijkstra_result[0]                             # accessing dijkstra pred array
        
        if node_destination == start:
            path = cons([starting_lat, starting_long], path)
        
        if dijkstra_result[1][node_destination] == inf:
            return None
            
        while node_destination is not start:                      # while loop on the destination node going backwards (from destination to start)
            path = cons([self.nodeID_to_position.get(node_destination).lat, self.nodeID_to_position.get(node_destination).long], path)
            
            node_destination = pred[node_destination]
            
            if node_destination == start:                         # doing this if destination node is start (start has to be included in the raw vector)
                path = cons([starting_lat, starting_long], path)  
           
            if node_destination == None:
                return path
                
        return path
           

        
#-------------------------------------------------------------------------------------------------------#
        
        
# FIND NEARBY
           
    def find_nearby(self, starting_lat, starting_long, poi_category, n):
        
        let start = self.position_to_nodeID.get(position(starting_lat, starting_long))
        let same_cat = self.locate_all(poi_category)
        
        # easy case
        if same_cat is None:
            return None
        
        let dijsktra_result = dijkstra(self.graph, start)
        let dist = dijsktra_result[1]
        
        let nearby_list = None
        
        let v = vec(dist.len())  # creating new vector to sort the dijkstra
        for x in range (v.len()):
            v[x] = [x, dist[x]]  # where x is the node and dist[x] is the distance
            
        heap_sort(v, λ x, y: x[1] < y[1])  # sorting by smallest distances
        
        let i = 0
        
        for x in v:
            let id = x[0]
            let position = self.nodeID_to_position.get(id)
            
            if x[1] is not inf and position is not None:
                let poi_position = self.nodeID_to_POIstruct[id]
                
               # println (poi_position)
                
                if poi_position is not None:
                    while poi_position is not None:
                       # println (poi_position.data)
                        
                        if poi_position.data.cat == poi_category:
                            let latitude = poi_position.data.lat
                            let longitude = poi_position.data.long
                            let category = poi_position.data.cat
                            let name = poi_position.data.name
                            
                            nearby_list = cons([latitude, longitude, category, name], nearby_list)
                            
                            i = i + 1
                            
                        poi_position = poi_position.next
                        
                        if i == n:
                            return nearby_list
           
        return nearby_list
                        
#-------------------------------------------------------------------------------------------------------#


def my_first_example():
    return TripPlanner([[0,0, 0,1], [0,0, 1,0]],
                       [[0,0, "bar", "The Empty Bottle"],
                        [0,1, "food", "Pelmeni"]])

test 'My first locate_all test':
    assert my_first_example().locate_all("food") == \
        cons([0,1], None)

test 'My first plan_route test':
   assert my_first_example().plan_route(0, 0, "Pelmeni") == \
       cons([0,0], cons([0,1], None))

test 'My first find_nearby test':
   assert my_first_example().find_nearby(0, 0, "food", 1) == \
        cons([0,1, "food", "Pelmeni"], None)

def example_from_handout():
    return TripPlanner([[0,0, 1,0], [0,0, 0,1], [0,1, 1,1], [0,1, 0,2], [1,0, 1,1], [0,2, 1,2], [1,1, 1,2], [1,2, 1,3], [1,3, -0.2,3.3]],
                                [[0,0, "food", "Sandwiches"], [0,1, "food", "Pasta"], [1,1, "bank", "Local Credit Union"], [1,3, "bar", "Bar None"],
                                [1,3, "bar", "H Bar"], [-0.2, 3.3, "food", "Burritos"]])
    


test 'handout example':
    let h = TripPlanner([[0,0, 1,0], [0,0, 0,1], [0,1, 1,1], [0,1, 0,2], [1,0, 1,1], [0,2, 1,2], [1,1, 1,2], [1,2, 1,3], [1,3, -0.2,3.3]],
                                [[0,0, "food", "Sandwiches"], [0,1, "food", "Pasta"], [1,1, "bank", "Local Credit Union"], [1,3, "bar", "Bar None"],
                                [1,3, "bar", "H Bar"], [-0.2, 3.3, "food", "Burritos"]])
    
    assert h.locate_all("food") == cons([-0.2,3.3], cons([0,1], cons([0,0], None)))
    
    assert h.plan_route(0,0,"Sandwiches") == cons([0,0], None)
    assert h.plan_route(0,1,"Sandwiches") == cons([0,1],cons([0,0], None))
    
    assert h.find_nearby(1,3, "food", 1) == cons([-0.2, 3.3, "food", "Burritos"], None)  # fix for next submission --> when n > 1 raises an error
    assert h.find_nearby(0,2, "food", 2) == cons([0,0, "food", "Sandwiches"], cons([0,1, "food", "Pasta"], None))
    
    
    
test 'my test':
    let tp = TripPlanner(
      [[0, 0, 0, 1],
       [0, 1, 3, 0],
       [0, 1, 4, 0],
       [0, 1, 5, 0],
       [0, 1, 6, 0],
       [0, 0, 1, 1],
       [1, 1, 3, 0],
       [1, 1, 4, 0],
       [1, 1, 5, 0],
       [1, 1, 6, 0],
       [0, 0, 2, 1],
       [2, 1, 3, 0],
       [2, 1, 4, 0],
       [2, 1, 5, 0],
       [2, 1, 6, 0]],
      [[0, 0, 'blacksmith', "Revere's Silver Shop"],
       [6, 0, 'church', 'Old North Church']])
    let result = tp.plan_route(0, 0, 'Old North Church')
    assert Cons.to_vec(result) \
      == [[0, 0], [2, 1], [6, 0]]
      
test 'hey':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'barber', 'Judy']])
    let result = tp.plan_route(0, 0, 'Judy')
    assert Cons.to_vec(result) \
      == []
      
      
test 'tey':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony']])
    let result = tp.plan_route(0, 0, 'Judy')
    assert Cons.to_vec(result) \
      == []
      
  
test 'tey tey':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [4, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy']])
    let result = tp.find_nearby(0, 0, 'barber', 3)
    assert (Cons.to_vec(result)) \
      == [[5, 0, 'barber', 'Judy'], [3, 0, 'barber', 'Tony']]
      
test 'more from report':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [4, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy']])
    let result = tp.find_nearby(0, 0, 'food', 1)
    assert (Cons.to_vec(result)) \
      == []
      
test 'more from report pt2':
    let tp = TripPlanner(
      [[0, 0, 0, 9],
       [0, 9, 9, 9],
       [0, 0, 1, 1],
       [1, 1, 2, 2],
       [2, 2, 3, 3],
       [3, 3, 4, 4],
       [4, 4, 5, 5],
       [5, 5, 6, 6],
       [6, 6, 7, 7],
       [7, 7, 8, 8],
       [8, 8, 9, 9]],
      [[7, 7, 'haberdasher', 'Archit'],
       [8, 8, 'haberdasher', 'Braden'],
       [9, 9, 'haberdasher', 'Cem']])
    let result = tp.find_nearby(0, 0, 'haberdasher', 2)
    assert (Cons.to_vec(result)) \
      == [[8, 8, 'haberdasher', 'Braden'],[7, 7, 'haberdasher', 'Archit']]
      
          
test 'more from report pt4':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy'],
       [5, 0, 'bar', 'Pasta']])
    let result = tp.find_nearby(0, 0, 'barber', 2)
    assert (Cons.to_vec(result)) \
      == [[5, 0, 'barber', 'Judy'], [3, 0, 'barber', 'Tony']]
      
test 'more from report pt5':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy'],
       [5, 0, 'bar', 'Pasta']])
    let result = tp.find_nearby(0, 0, 'barber', 2)
    assert (Cons.to_vec(result)) \
      == [[5, 0, 'barber', 'Judy'], [3, 0, 'barber', 'Tony']]
      
      
#### ^^^ YOUR CODE HERE
