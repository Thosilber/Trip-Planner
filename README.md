# Trip-Planner
My CS214 class final project: A Trip Planner coded in DSSL2 using several data structures and algorithms


ADTs:
all_nodes (dictionary)
To keep track of all nodes (locations) in each place that a trip will be planned. Do this to have the size of all nodes (with no duplicates) and use it for the size of the graph and dictionaries.
Association List
Faster and more efficient than an array, for example

self.graph (WU graph)
Organize and connect all nodes, roads, and POIs in one system. Includes distances between nodes and connections between POIs.
Adjacency Matrix for Weighted Undirected Graph
Graph → effective way to represent connections and paths between items (POIs in this case). Undirected → roads go both ways. Weighted → some queries require finding the closest / shortest path. Adjacency Matrix → imported from project lib but also provides the existence of undirected roads and edges.

self.nodeID_to_position (dictionary)
When inputting a node, get the position that it refers to.
Association List
Faster and effective. Node = key, position = value.

self.position_to_nodeID (dictionary)
When inputting a position, get the node that it corresponds to.
Hash Table
Faster and effective. Can transform/hash a position struct into a key number.

self.name_to_POIstruct (dictionary)
When inputting a name, get the whole POI struct it corresponds to.
Hash Table
Faster and effective. Can transform/hash a name (string) into a key number.
Remove_dup_dict (dictionary)
Used on locate_all  to check (using .mem?) if the position of the POI has already been added to the list of positions that contain the POI of the same category.
Hash Table
Faster and effective. Could also use an association list or vector, but thought a Hash Table was more effective. 

Todo (priority queue)
Used on Dijkstra's Algorithm to keep track of which node should be relaxed next.
Binary Heap
Ensures an effective insert and delete. Also sorts the data which is necessary for Dijkstra's. Finally, it is more efficient than options such a sorted list or unsorted list.
 


Algorithms:

Heap_sort
Used for find_nearby. Sorts all nearby POIs with the same category by proximity to therefore return the "n" closest.
I was working with the dijkstra's output array of distances. Therefore, heap_sort could easily sort it by smallest distances.

Dijkstra's Algorithm
Finds the shortests paths between the nodes on the graph. Used for plan_route to find the shortest path between the start position and destination. Used for find_nearby to find the POIs that are near the starting position.
Chosen because it manages to find the shortest paths between the nodes on the graph on a considerably low complexity. Also, there were no negative distances (weights) which allowed Dijkstra's to be used.
