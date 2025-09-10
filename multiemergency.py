"""
multiemergency.py

Multi-emergency Dijkstra simulation with 2 ambulances on OSM road network.
Select up to 6 emergency locations by clicking on the map.
The program finds the best assignment of emergencies to 2 ambulances
to minimize total travel distance/time.

Dependencies:
    pip install osmnx networkx matplotlib
"""

import osmnx as ox
import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import heapq
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
import itertools
import numpy as np

# -----------------------
# CONFIG
# -----------------------
PLACE_NAME = "Karol Bagh, New Delhi, India"
NETWORK_DIST_METERS = 900   # radius (meters) around center point to load
ANIM_INTERVAL_MS = 25       # animation frame interval

VISIT_COLOR = "#FFD700"     # gold for visited
PATH_COLOR_1 = "#FF3333"    # red for ambulance 1
PATH_COLOR_2 = "#33A1FF"    # blue for ambulance 2
BASE_COLOR = "#00BFFF"      # base station color (deep sky blue)
TARGET_COLOR = "#FF4500"    # target color (orange red)
DEFAULT_NODE_COLOR = "#111111"  # node dots default (dark)
EDGE_COLOR = "#888888"      # edges color
NODE_SIZE = 10

MAX_EMERGENCIES = 6

# -----------------------
# DOWNLOAD / PREPARE GRAPH
# -----------------------
print("⏳ Geocoding place and downloading road network...")
center_point = ox.geocode(PLACE_NAME)
G = ox.graph_from_point(center_point, dist=NETWORK_DIST_METERS, network_type="drive", simplify=True)

# Undirected for shortest paths
G = G.to_undirected()

pos = {n: (data["x"], data["y"]) for n, data in G.nodes(data=True)}

base_node = ox.distance.nearest_nodes(G, center_point[1], center_point[0])
print(f"Base station node (A) chosen: {base_node}")

fig, ax = ox.plot_graph(G, show=False, close=False, node_size=0, edge_color=EDGE_COLOR, bgcolor="white")
fig.canvas.manager.set_window_title("Multi-Emergency Ambulance Dispatch")

node_list = list(G.nodes())
node_colors = [DEFAULT_NODE_COLOR for _ in node_list]
scat = ax.scatter([pos[n][0] for n in node_list], [pos[n][1] for n in node_list], s=NODE_SIZE, c=node_colors, zorder=3)

# Color base node
base_idx = node_list.index(base_node)
node_colors[base_idx] = BASE_COLOR
scat.set_color(node_colors)

ax.text(pos[base_node][0], pos[base_node][1], " A ", color="white",
        bbox=dict(facecolor=BASE_COLOR, edgecolor='none', pad=1), zorder=10, fontsize=8)

# Load ambulance icons (two different colors for distinction)
ambulance_img_1 = plt.imread("ambulance.png")
ambulance_icon_1 = OffsetImage(ambulance_img_1, zoom=0.05)
ambulance_img_2 = plt.imread("ambulance.png")
ambulance_icon_2 = OffsetImage(ambulance_img_2, zoom=0.05)

# -----------------------
# Variables
# -----------------------
emergency_nodes = []
node_color_map = {n: DEFAULT_NODE_COLOR for n in node_list}
node_color_map[base_node] = BASE_COLOR

path_line_1 = None
path_line_2 = None

ambulance_annot_1 = None
ambulance_annot_2 = None

route_paths = [[], []]         # full node paths for ambulance 1 and 2
route_positions = [[], []]     # interpolated positions for animation
route_step_indices = [0, 0]    # current animation step index per ambulance

distance_matrix = None         # distance matrix for TSP calculations
path_matrix = None             # shortest paths between points

# -----------------------
# Dijkstra shortest path helper
# -----------------------
def dijkstra_shortest_path(G, source, target):
    try:
        return nx.shortest_path(G, source=source, target=target, weight='length')
    except nx.NetworkXNoPath:
        return []

# -----------------------
# Compute all pairs shortest paths between base and emergencies
# -----------------------
def compute_distance_matrix():
    global distance_matrix, path_matrix
    points = [base_node] + emergency_nodes
    n = len(points)
    distance_matrix = np.zeros((n, n))
    path_matrix = [[[] for _ in range(n)] for _ in range(n)]
    for i in range(n):
        for j in range(n):
            if i == j:
                distance_matrix[i, j] = 0.0
                path_matrix[i][j] = [points[i]]
            else:
                path = dijkstra_shortest_path(G, points[i], points[j])
                path_matrix[i][j] = path
                dist = 0.0
                for k in range(len(path)-1):
                    edge_data = G.get_edge_data(path[k], path[k+1])
                    # edge_data can be dict of dicts if multigraph
                    if isinstance(edge_data, dict) and 0 in edge_data:
                        dist += edge_data[0].get('length', 1.0)
                    else:
                        dist += edge_data.get('length', 1.0)
                distance_matrix[i, j] = dist

# -----------------------
# Solve partitioning of emergencies to 2 ambulances to minimize total route
# (Simple brute force over partitions since max emergencies = 6)
# Each ambulance route is base_node -> assigned emergencies in some order -> base_node
# Here we only consider permutations of emergencies order and choose best total cost
# -----------------------
def find_best_partition_and_routes():
    global best_partition, best_routes

    n = len(emergency_nodes)
    points = [base_node] + emergency_nodes

    best_cost = float('inf')
    best_partition = None
    best_routes = None

    # Generate all partitions of emergencies into two subsets
    # Use bitmask from 0 to 2^n -1 to represent one subset, other is complement
    # For each subset, try permutations to find best route order
    emergencies_idx = list(range(1, n+1))  # indices in distance_matrix and points

    for mask in range(1 << n):
        subset1 = [emergencies_idx[i] for i in range(n) if (mask & (1 << i)) != 0]
        subset2 = [e for e in emergencies_idx if e not in subset1]

        # For both subsets, find best permutation route cost
        best_order_1, cost_1 = find_best_route_order(subset1)
        best_order_2, cost_2 = find_best_route_order(subset2)

        total_cost = cost_1 + cost_2

        if total_cost < best_cost:
            best_cost = total_cost
            best_partition = (subset1, subset2)
            best_routes = (best_order_1, best_order_2)

    print(f"Best total route cost: {best_cost:.1f}")
    print(f"Best partition: {best_partition}")
    print(f"Best routes: {best_routes}")

# -----------------------
# For a subset of emergency indices, find best route order starting at base (0)
# using brute force permutations. Returns best order tuple and cost.
# -----------------------
def find_best_route_order(subset):
    if len(subset) == 0:
        return (), 0.0
    best_cost = float('inf')
    best_order = None
    for perm in itertools.permutations(subset):
        cost = 0.0
        prev = 0  # base index
        for idx in perm:
            cost += distance_matrix[prev][idx]
            prev = idx
        cost += distance_matrix[prev][0]  # return to base (optional, can be skipped)
        if cost < best_cost:
            best_cost = cost
            best_order = perm
    return best_order, best_cost

# -----------------------
# Build full path of nodes from base and ordered emergency indices
# -----------------------
def build_full_path(route_order):
    if not route_order:
        return [base_node]
    points = [base_node] + emergency_nodes
    full_path = [base_node]
    for idx in route_order:
        # Add shortest path from last node to this emergency node (excluding last node to avoid duplicates)
        last_node = full_path[-1]
        path_segment = dijkstra_shortest_path(G, last_node, points[idx])
        if len(path_segment) > 1:
            full_path.extend(path_segment[1:])
    # Optionally return to base? For now no return to base in full path
    return full_path

# -----------------------
# Draw route lines on axis, return line object
# -----------------------
def draw_route_on_ax(path_nodes, color, old_line=None):
    if old_line is not None:
        try:
            old_line.remove()
        except Exception:
            pass
    xs = [pos[n][0] for n in path_nodes]
    ys = [pos[n][1] for n in path_nodes]
    line, = ax.plot(xs, ys, color=color, linewidth=3, alpha=0.8, zorder=8)
    fig.canvas.draw_idle()
    return line

# -----------------------
# Animation step: move ambulances along their routes
# -----------------------
def anim_step(frame):
    global route_step_indices, ambulance_annot_1, ambulance_annot_2

    artists = [scat]

    for i in range(2):
        if len(route_positions[i]) == 0:
            continue
        idx = route_step_indices[i]
        x, y = route_positions[i][idx]
        if i == 0:
            if ambulance_annot_1 is not None:
                ambulance_annot_1.xybox = (x, y)
            else:
                ambulance_annot_1 = AnnotationBbox(ambulance_icon_1, (x, y), frameon=False, zorder=12)
                ax.add_artist(ambulance_annot_1)
            artists.append(ambulance_annot_1)
        else:
            if ambulance_annot_2 is not None:
                ambulance_annot_2.xybox = (x, y)
            else:
                ambulance_annot_2 = AnnotationBbox(ambulance_icon_2, (x, y), frameon=False, zorder=12)
                ax.add_artist(ambulance_annot_2)
            artists.append(ambulance_annot_2)

        # Advance animation index but don't go out of bounds
        if idx < len(route_positions[i]) - 1:
            route_step_indices[i] += 1

    if path_line_1 is not None:
        artists.append(path_line_1)
    if path_line_2 is not None:
        artists.append(path_line_2)

    colors = [node_color_map[n] for n in node_list]
    scat.set_color(colors)
    artists.append(scat)

    return artists

# -----------------------
# Click handler to add emergency nodes
# -----------------------
def on_click(event):
    if event.inaxes != ax:
        return
    if event.xdata is None or event.ydata is None:
        return
    clicked_x, clicked_y = event.xdata, event.ydata
    try:
        nearest = ox.distance.nearest_nodes(G, clicked_x, clicked_y)
    except Exception as e:
        print("nearest_nodes error:", e)
        return
    if nearest == base_node:
        print("Clicked base station — choose another node for emergency.")
        return
    if nearest in emergency_nodes:
        print("Already selected this emergency node.")
        return
    if len(emergency_nodes) >= MAX_EMERGENCIES:
        print("Maximum 6 emergencies selected.")
        return

    emergency_nodes.append(nearest)
    node_color_map[nearest] = TARGET_COLOR
    scat.set_color([node_color_map[n] for n in node_list])
    print(f"Selected emergency node #{len(emergency_nodes)}: {nearest}")

# -----------------------
# Dispatch button pressed - compute and animate routes
# -----------------------
def dispatch(event=None):
    global path_line_1, path_line_2, ambulance_annot_1, ambulance_annot_2
    global route_paths, route_positions, route_step_indices

    if len(emergency_nodes) == 0:
        print("No emergencies selected!")
        return

    print("Dispatching ambulances to emergencies...")
    compute_distance_matrix()
    find_best_partition_and_routes()

    # Build full paths
    path1 = build_full_path(best_routes[0]) if best_routes[0] else [base_node]
    path2 = build_full_path(best_routes[1]) if best_routes[1] else [base_node]

    print(f"Ambulance 1 full path nodes: {path1}")
    print(f"Ambulance 2 full path nodes: {path2}")

    # Remove old lines and ambulance icons
    if path_line_1 is not None:
        path_line_1.remove()
    if path_line_2 is not None:
        path_line_2.remove()
    if ambulance_annot_1 is not None:
        ambulance_annot_1.remove()
        ambulance_annot_1 = None
    if ambulance_annot_2 is not None:
        ambulance_annot_2.remove()
        ambulance_annot_2 = None

    # Draw route lines only if length > 1
    if len(path1) > 1:
        path_line_1 = draw_route_on_ax(path1, PATH_COLOR_1, path_line_1)
    else:
        path_line_1 = None
    if len(path2) > 1:
        path_line_2 = draw_route_on_ax(path2, PATH_COLOR_2, path_line_2)
    else:
        path_line_2 = None

    route_paths = [path1, path2]

    # Prepare animation positions for each ambulance
    route_positions = []
    for route_path in route_paths:
        pos_list = []
        for i in range(len(route_path) - 1):
            start_node = route_path[i]
            end_node = route_path[i+1]
            idx_start = [base_node] + emergency_nodes
            idx_start = idx_start.index(start_node)
            idx_end = [base_node] + emergency_nodes
            idx_end = idx_end.index(end_node)
            intermediate_nodes = path_matrix[idx_start][idx_end]
            for n in intermediate_nodes[:-1]:
                pos_list.append(pos[n])
        # Add last node position
        pos_list.append(pos[route_path[-1]])
        route_positions.append(pos_list)

    route_step_indices = [0, 0]

    # Place ambulance icons statically at base if no route to animate
    if len(path1) == 1:
        x, y = pos[path1[0]]
        ambulance_annot_1 = AnnotationBbox(ambulance_icon_1, (x, y), frameon=False, zorder=12)
        ax.add_artist(ambulance_annot_1)
    if len(path2) == 1:
        x, y = pos[path2[0]]
        ambulance_annot_2 = AnnotationBbox(ambulance_icon_2, (x, y), frameon=False, zorder=12)
        ax.add_artist(ambulance_annot_2)

# -----------------------
# Button for dispatch
# -----------------------
from matplotlib.widgets import Button
button_ax = fig.add_axes([0.82, 0.01, 0.15, 0.05])
button = Button(button_ax, "Dispatch Ambulances")
button.on_clicked(dispatch)

# -----------------------
# Connect click event
# -----------------------
cid = fig.canvas.mpl_connect("button_press_event", on_click)

# -----------------------
# Start animation
# -----------------------
anim = animation.FuncAnimation(fig, anim_step, interval=ANIM_INTERVAL_MS, blit=False)

print(f"Ready — select up to {MAX_EMERGENCIES} emergencies by clicking on map, then click 'Dispatch Ambulances'.")
plt.show()