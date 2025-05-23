####################################################
# LSrouter.py
# Name:
# HUID:
#####################################################

from router import Router
from packet import Packet
import json
import heapq

class LSrouter(Router):
    """Link state routing protocol implementation.

    Add your own class fields and initialization code (e.g. to create forwarding table
    data structures). See the `Router` base class for docstrings of the methods to
    override.
    """

    def __init__(self, addr, heartbeat_time):
        Router.__init__(self, addr)  # Initialize base class - DO NOT REMOVE
        self.heartbeat_time = heartbeat_time
        self.last_time = 0
        
        # Initialize data structures for link-state routing
        self.links_state = {}  # Local link state of this router {port: (neighbor_addr, cost)}
        self.ls_database = {}  # Global link-state database {router_addr: {'links': {...}, 'seq': sequence_number}}
        self.forwarding_table = {}  # Forwarding table {dst_addr: next_hop_port}
        self.seq_num = 0  # Sequence number for link state messages
        self.neighbors = {}  # Mapping of port to neighbor address {port: addr}
        
        # Initialize own link state in the database
        self.ls_database[self.addr] = {'links': {}, 'seq': self.seq_num}

    def handle_packet(self, port, packet):
        """Process incoming packet."""
        if packet.is_traceroute:
            # This is a data packet - forward according to forwarding table
            if packet.dst_addr in self.forwarding_table:
                out_port = self.forwarding_table[packet.dst_addr]
                self.send(out_port, packet)
        else:
            # This is a routing packet - process link state information
            try:
                # Parse the packet content
                ls_data = json.loads(packet.content)
                router_addr = ls_data['router']
                links = ls_data['links']
                seq_num = ls_data['seq']
                
                # Check if we have newer information
                update_needed = False
                if router_addr not in self.ls_database:
                    update_needed = True
                elif seq_num > self.ls_database[router_addr]['seq']:
                    update_needed = True
                
                if update_needed:
                    # Update the link state database
                    self.ls_database[router_addr] = {
                        'links': links,
                        'seq': seq_num
                    }
                    
                    # Recalculate forwarding table
                    self.calculate_forwarding_table()
                    
                    # Forward the packet to all other neighbors
                    for p in self.neighbors:
                        if p != port:  # Don't send back to the source
                            self.send(p, packet)
            except Exception:
                # Handle any parsing errors silently
                pass

    def handle_new_link(self, port, endpoint, cost):
        """Handle new link."""
        # Update local link state information
        self.links_state[port] = (endpoint, cost)
        self.neighbors[port] = endpoint
        
        # Update own link state in the database
        self.ls_database[self.addr]['links'][endpoint] = cost
        
        # Increment sequence number
        self.seq_num += 1
        self.ls_database[self.addr]['seq'] = self.seq_num
        
        # Recalculate the forwarding table
        self.calculate_forwarding_table()
        
        # Broadcast link state to all neighbors
        self.broadcast_link_state()

    def handle_remove_link(self, port):
        """Handle removed link."""
        if port in self.links_state:
            # Get the endpoint address before removal
            endpoint, _ = self.links_state[port]
            
            # Remove from local link state information
            del self.links_state[port]
            
            if port in self.neighbors:
                del self.neighbors[port]
            
            # Update own link state in the database
            if endpoint in self.ls_database[self.addr]['links']:
                del self.ls_database[self.addr]['links'][endpoint]
            
            # Increment sequence number
            self.seq_num += 1
            self.ls_database[self.addr]['seq'] = self.seq_num
            
            # Recalculate the forwarding table
            self.calculate_forwarding_table()
            
            # Broadcast link state to all neighbors
            self.broadcast_link_state()

    def handle_time(self, time_ms):
        """Handle current time."""
        if time_ms - self.last_time >= self.heartbeat_time:
            self.last_time = time_ms
            # Broadcast link state periodically
            self.broadcast_link_state()

    def broadcast_link_state(self):
        """Broadcast this router's link state to all neighbors."""
        # Create the link state message
        ls_message = {
            'router': self.addr,
            'links': self.ls_database[self.addr]['links'],
            'seq': self.seq_num
        }
        
        # Convert to JSON string
        ls_content = json.dumps(ls_message)
        
        # Send to all neighbors
        for port in self.neighbors:
            # Create a new routing packet
            packet = Packet(Packet.ROUTING, self.addr, self.neighbors[port], ls_content)
            self.send(port, packet)

    def calculate_forwarding_table(self):
        """Calculate the forwarding table using Dijkstra's algorithm."""
        # Initialize forwarding table
        self.forwarding_table = {}
        
        # Create a graph from the link state database
        graph = {}
        for router, data in self.ls_database.items():
            if router not in graph:
                graph[router] = {}
            
            for neighbor, cost in data['links'].items():
                if neighbor not in graph:
                    graph[neighbor] = {}
                
                graph[router][neighbor] = cost
        
        # Run Dijkstra's algorithm from this router
        distances = {node: float('infinity') for node in graph}
        previous = {node: None for node in graph}
        distances[self.addr] = 0
        pq = [(0, self.addr)]
        
        while pq:
            current_distance, current_node = heapq.heappop(pq)
            
            if current_distance > distances[current_node]:
                continue
            
            if current_node not in graph:
                continue
                
            for neighbor, weight in graph[current_node].items():
                distance = current_distance + weight
                
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous[neighbor] = current_node
                    heapq.heappush(pq, (distance, neighbor))
        
        # Build the forwarding table from the shortest paths
        for dst in distances:
            if dst != self.addr:
                # Find next hop on path to destination
                next_hop = self.get_next_hop(previous, dst)
                if next_hop is not None:
                    # Add to forwarding table
                    self.forwarding_table[dst] = next_hop

    def get_next_hop(self, previous, destination):
        """Determine the next hop port to reach the destination."""
        # If destination is not reachable
        if destination not in previous or previous[destination] is None:
            return None
        
        # Start from destination and work backwards
        current = destination
        next_node = None
        
        # Find the first node directly connected to this router
        while previous[current] != self.addr:
            next_node = current
            current = previous[current]
            # If we hit a dead end, the route is not complete
            if current is None:
                return None
        
        # current should now be the node directly connected to this router
        # Find the port that connects to this node
        for port, (endpoint, _) in self.links_state.items():
            if endpoint == current:
                return port
                
        return None

    def __repr__(self):
        """Representation for debugging in the network visualizer."""
        return f"LSrouter(addr={self.addr}, links={self.links_state}, ft={self.forwarding_table})"
