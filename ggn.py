import osmnx as ox
import networkx as nx
import matplotlib.pyplot as plt
import random

# --------------------
# 1. Download road network for Gurgaon
# --------------------
city = "Gurgaon, India"
G = ox.graph_from_place(city, network_type="drive")

# --------------------
# 2. Get list of nodes and pick points
# --------------------
nodes = list(G.nodes())

# Base station (A) — fixed node (pick central)
base_station = random.choice(nodes)

# Random emergency points
num_emergencies = 5
emergency_points = random.sample([n for n in nodes if n != base_station], num_emergencies)

# Assign letters A, B, C, D...
labels = {base_station: "A"}
for i, node in enumerate(emergency_points):
    labels[node] = chr(66 + i)  # 66 is 'B'

# --------------------
# 3. Compute shortest paths from A to all emergencies
# --------------------
paths = {}
for node in emergency_points:
    path = nx.shortest_path(G, base_station, node, weight="length")
    paths[node] = path

# --------------------
# 4. Plot map
# --------------------
fig, ax = ox.plot_graph(G, show=False, close=False, node_size=0, edge_color="#cccccc", bgcolor="white")

# Highlight base station and emergencies
for node, label in labels.items():
    x, y = G.nodes[node]['x'], G.nodes[node]['y']
    color = "red" if label == "A" else "blue"
    ax.scatter(x, y, c=color, s=80, zorder=5)
    ax.text(x, y, label, fontsize=10, color="black", zorder=6)

# Highlight paths
for node, path in paths.items():
    ox.plot_graph_route(G, path, route_color="orange", route_linewidth=2, ax=ax, show=False, close=False)

plt.show()