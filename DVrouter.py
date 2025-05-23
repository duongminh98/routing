####################################################
# DVrouter.py
# Name:
# HUID:
#####################################################

from router import Router
from packet import Packet
import json

class DVrouter(Router):
    """Distance vector routing protocol implementation.

    Add your own class fields and initialization code (e.g. to create forwarding table
    data structures). See the `Router` base class for docstrings of the methods to
    override.
    """

    # Định nghĩa vô cùng cho thuật toán distance vector
    INFINITY = 16

    def __init__(self, addr, heartbeat_time):
        Router.__init__(self, addr)  # Initialize base class - DO NOT REMOVE
        self.heartbeat_time = heartbeat_time
        self.last_time = 0
        
        # Initialize data structures for distance-vector routing
        self.distance_vector = {}  # Distance vector {dst_addr: cost}
        self.forwarding_table = {}  # Forwarding table {dst_addr: next_hop_port}
        self.neighbors = {}  # Information about neighbors {port: (neighbor_addr, cost)}
        self.neighbor_dv = {}  # Distance vectors from neighbors {neighbor_addr: {dst_addr: cost}}
        
        # Initialize distance vector with this router at distance 0
        self.distance_vector[self.addr] = 0

    def handle_packet(self, port, packet):
        """Process incoming packet."""
        if packet.is_traceroute:
            # This is a data packet - forward according to forwarding table
            if packet.dst_addr in self.forwarding_table:
                out_port = self.forwarding_table[packet.dst_addr]
                self.send(out_port, packet)
        else:
            # This is a routing packet - process distance vector information
            try:
                # Parse received distance vector
                dv_data = json.loads(packet.content)
                neighbor_addr = dv_data['addr']
                neighbor_dv = dv_data['dv']
                
                # Store neighbor's distance vector
                self.neighbor_dv[neighbor_addr] = neighbor_dv
                
                # Recalculate own distance vector
                updated = self.update_distance_vector()
                
                # If our distance vector changed, broadcast it
                if updated:
                    self.broadcast_distance_vector()
                    
            except Exception:
                # Handle any parsing errors silently
                pass

    def handle_new_link(self, port, endpoint, cost):
        """Handle new link."""
        # Store information about the new neighbor
        self.neighbors[port] = (endpoint, cost)
        
        # Initialize distance directly to this neighbor
        self.distance_vector[endpoint] = cost
        
        # Add an empty distance vector for the neighbor if it doesn't exist yet
        if endpoint not in self.neighbor_dv:
            self.neighbor_dv[endpoint] = {}
            
        # Update distance vector and forwarding table
        self.update_distance_vector()
        
        # Broadcast our updated distance vector to neighbors
        self.broadcast_distance_vector()

    def handle_remove_link(self, port):
        """Handle removed link."""
        if port in self.neighbors:
            # Get the neighbor address before removing the link
            neighbor_addr, _ = self.neighbors[port]
            
            # Remove the neighbor
            del self.neighbors[port]
            
            # Remove the neighbor's distance vector
            if neighbor_addr in self.neighbor_dv:
                del self.neighbor_dv[neighbor_addr]
            
            # Mark all routes through this port as infinity
            destinations_to_update = []
            for dst, next_hop in self.forwarding_table.items():
                if next_hop == port:
                    destinations_to_update.append(dst)
            
            # Update distance vector for all affected destinations
            for dst in destinations_to_update:
                self.distance_vector[dst] = self.INFINITY
                
            # Remove routes that used this port
            self.forwarding_table = {dst: p for dst, p in self.forwarding_table.items() if p != port}
                
            # Update the distance vector and forwarding table
            self.update_distance_vector()
            
            # Broadcast our updated distance vector to neighbors
            self.broadcast_distance_vector()

    def handle_time(self, time_ms):
        """Handle current time."""
        if time_ms - self.last_time >= self.heartbeat_time:
            self.last_time = time_ms
            # Broadcast distance vector periodically
            self.broadcast_distance_vector()

    def update_distance_vector(self):
        """Update this router's distance vector based on neighbors' distance vectors.
        
        Returns:
            bool: True if the distance vector was updated, False otherwise
        """
        updated = False
        
        # Save a copy of the old forwarding table to check for changes
        old_forwarding_table = self.forwarding_table.copy()
        
        # For each destination in our distance vector and neighbors' distance vectors
        all_destinations = set(self.distance_vector.keys())
        for neighbor_addr, neighbor_dv in self.neighbor_dv.items():
            all_destinations.update(neighbor_dv.keys())
        
        # Make sure our own address is always in the distance vector
        all_destinations.add(self.addr)
        
        # First reset forwarding table, keeping only directly connected neighbors
        # (these are always accurate and should not be overridden)
        new_forwarding_table = {}
        for port, (neighbor_addr, cost) in self.neighbors.items():
            new_forwarding_table[neighbor_addr] = port
            
        # Update the distance vector for each destination
        for dst in all_destinations:
            if dst == self.addr:
                # Distance to self is always 0
                self.distance_vector[dst] = 0
                continue
            
            # If this is a neighbor, we already know the best route
            if dst in new_forwarding_table:
                continue
            
            # Current best known distance to destination
            current_dist = self.distance_vector.get(dst, self.INFINITY)
            
            # Find the minimum cost path through neighbors
            min_cost = self.INFINITY
            best_port = None
            
            for port, (neighbor_addr, link_cost) in self.neighbors.items():
                if neighbor_addr in self.neighbor_dv:
                    # Get the cost from neighbor to destination
                    neighbor_cost = self.neighbor_dv[neighbor_addr].get(dst, self.INFINITY)
                    
                    # Skip this route if it's infinity (unreachable)
                    if neighbor_cost >= self.INFINITY:
                        continue
                    
                    # Calculate total cost via this neighbor
                    total_cost = link_cost + neighbor_cost
                    
                    # Update if we found a better path
                    if total_cost < min_cost:
                        min_cost = total_cost
                        best_port = port
            
            # Update distance vector if we found a new path or cost
            if min_cost != current_dist:
                self.distance_vector[dst] = min_cost
                updated = True
            
            # Update forwarding table with best path
            if min_cost < self.INFINITY and best_port is not None:
                new_forwarding_table[dst] = best_port
        
        # Update forwarding table
        self.forwarding_table = new_forwarding_table
        
        # Return whether there were any changes
        if updated or old_forwarding_table != self.forwarding_table:
            return True
        return False

    def broadcast_distance_vector(self):
        """Broadcast this router's distance vector to all neighbors."""
        for port, (neighbor_addr, _) in self.neighbors.items():
            # Create a modified distance vector for this neighbor (split horizon with poison reverse)
            modified_dv = {}
        
            for dst, cost in self.distance_vector.items():
                # Apply split horizon with poison reverse:
                # If the route to dst goes through this neighbor,
                # report it as infinity (poison reverse)
                if dst in self.forwarding_table and self.forwarding_table[dst] == port:
                    modified_dv[dst] = self.INFINITY
                else:
                    modified_dv[dst] = cost
            
            # Create message with the modified distance vector
            dv_message = {
                'addr': self.addr,
                'dv': modified_dv
            }
            
            # Convert to JSON string
            dv_content = json.dumps(dv_message)
            
            # Create a new routing packet
            packet = Packet(Packet.ROUTING, self.addr, neighbor_addr, dv_content)
            self.send(port, packet)

    def __repr__(self):
        """Representation for debugging in the network visualizer."""
        return f"DVrouter(addr={self.addr}, dv={self.distance_vector}, ft={self.forwarding_table})"
