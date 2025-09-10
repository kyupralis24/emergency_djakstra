"""
interactive_dijkstra_osm_ambulance_static.py

Interactive Dijkstra animation on a real road network (Karol Bagh, New Delhi).
Click anywhere on the map to choose an emergency node (nearest intersection).
The base station is fixed (A). The algorithm is animated showing visited nodes
and the final shortest path. Once path is found, an ambulance icon is placed
statically at the start of the path.

Dependencies:
    pip install osmnx networkx matplotlib
"""

import osmnx as ox
import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import heapq
from matplotlib.offsetbox import OffsetImage, AnnotationBbox

# -----------------------
# CONFIG
# -----------------------
PLACE_NAME = "Sonipat, Haryana, India"
NETWORK_DIST_METERS = 900   # radius (meters) around center point to load
ANIM_INTERVAL_MS = 25       # animation frame interval
VISIT_COLOR = "#FFD700"     # gold for visited
PATH_COLOR = "#FF3333"      # red for final path
BASE_COLOR = "#00BFFF"      # base station color (deep sky blue)
TARGET_COLOR = "#FF4500"    # target color (orange red)
DEFAULT_NODE_COLOR = "#111111"  # node dots default (dark)
EDGE_COLOR = "#888888"      # edges color
NODE_SIZE = 10

# -----------------------
# DOWNLOAD / PREPARE GRAPH
# -----------------------
print("⏳ Geocoding place and downloading road network (may take a few seconds)...")
center_point = ox.geocode(PLACE_NAME)
G = ox.graph_from_point(center_point, dist=NETWORK_DIST_METERS, network_type="drive", simplify=True)

# convert to undirected (for Dijkstra on an undirected road network)
G = G.to_undirected()

# get node coordinates (lon, lat) for plotting
pos = {n: (data["x"], data["y"]) for n, data in G.nodes(data=True)}

# pick a fixed base station node near the center (A)
base_node = ox.distance.nearest_nodes(G, center_point[1], center_point[0])
print(f"Base station node (A) chosen: {base_node}")

# prepare matplotlib figure with OSMnx default street plot
fig, ax = ox.plot_graph(G, show=False, close=False, node_size=0, edge_color=EDGE_COLOR, bgcolor="white")
fig.canvas.manager.set_window_title("Interactive Emergency Planner — Dijkstra (click to pick emergency)")

# scatter for nodes
node_xs = [pos[n][0] for n in G.nodes()]
node_ys = [pos[n][1] for n in G.nodes()]
node_list = list(G.nodes())
node_colors = [DEFAULT_NODE_COLOR for _ in node_list]
scat = ax.scatter(node_xs, node_ys, s=NODE_SIZE, c=node_colors, zorder=3)

# highlight base station
base_idx = node_list.index(base_node)
node_colors[base_idx] = BASE_COLOR
scat.set_color(node_colors)

# label base node "A"
ax.text(pos[base_node][0], pos[base_node][1], " A ", color="white",
        bbox=dict(facecolor=BASE_COLOR, edgecolor='none', pad=1), zorder=10, fontsize=8)

# -----------------------
# Dijkstra generator for animation
# -----------------------
def dijkstra_generator(G, start, target):
    dist = {n: float("inf") for n in G.nodes()}
    prev = {}
    dist[start] = 0.0
    pq = [(0.0, start)]
    seen = set()
    while pq:
        d, u = heapq.heappop(pq)
        if u in seen:
            continue
        seen.add(u)
        yield ("visit", u)

        if u == target:
            path = []
            cur = target
            while cur in prev:
                path.append(cur)
                cur = prev[cur]
            path.append(start)
            path.reverse()
            yield ("done", path)
            return

        for v in G.neighbors(u):
            data = G.get_edge_data(u, v, default={})
            if isinstance(data, dict) and len(data) > 0 and 0 in data:
                lengths = []
                for key in data:
                    edge_data = data[key]
                    if 'length' in edge_data:
                        lengths.append(edge_data['length'])
                w = min(lengths) if lengths else 1.0
            else:
                w = data.get('length', 1.0)

            nd = d + w
            if nd < dist[v]:
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))
                yield ("relax", v)
    yield ("no_path", None)


# -----------------------
# Globals for animation
# -----------------------
node_color_map = {n: DEFAULT_NODE_COLOR for n in node_list}
node_color_map[base_node] = BASE_COLOR

path_line = None
dijkstra_gen = None
ambulance_annot = None
current_target_node = None

# Load ambulance icon image
ambulance_img = plt.imread("ambulance.png")
ambulance_icon = OffsetImage(ambulance_img, zoom=0.05)  # adjust zoom for size

# -----------------------
# Draw path overlay (bold line)
# -----------------------
def draw_path_on_ax(path_nodes):
    global path_line
    if path_line is not None:
        try:
            path_line.remove()
        except Exception:
            pass
    xs = [pos[n][0] for n in path_nodes]
    ys = [pos[n][1] for n in path_nodes]
    path_line, = ax.plot(xs, ys, color=PATH_COLOR, linewidth=3.5, alpha=0.9, zorder=8)
    # annotate ETA (sum of lengths)
    total = 0.0
    for i in range(len(path_nodes)-1):
        u = path_nodes[i]
        v = path_nodes[i+1]
        data = G.get_edge_data(u, v, default={})
        if isinstance(data, dict) and len(data) > 0 and 0 in data:
            lengths = []
            for key in data:
                lengths.append(data[key].get('length', 0.0))
            weight = min(lengths) if lengths else 0.0
        else:
            weight = data.get('length', 0.0)
        total += weight
    x_last, y_last = pos[path_nodes[-1]]
    ax.text(x_last, y_last, f" ETA: {total:.0f} m", color=PATH_COLOR, fontsize=8, zorder=9,
            bbox=dict(facecolor="white", alpha=0.7, edgecolor='none', pad=1))
    fig.canvas.draw_idle()


# -----------------------
# Animation update step
# -----------------------
def anim_step(frame):
    global dijkstra_gen, ambulance_annot

    artists = [scat]

    if dijkstra_gen is not None:
        try:
            for _ in range(6):
                ev, payload = next(dijkstra_gen)
                if ev == "visit":
                    node_color_map[payload] = VISIT_COLOR
                elif ev == "relax":
                    if node_color_map[payload] == DEFAULT_NODE_COLOR:
                        node_color_map[payload] = "#FFC87C"
                elif ev == "done":
                    path = payload
                    for n in path:
                        node_color_map[n] = PATH_COLOR
                    draw_path_on_ax(path)
                    dijkstra_gen = None

                    # Place ambulance icon statically at start of path
                    if ambulance_annot is not None:
                        ambulance_annot.remove()
                    x, y = pos[path[0]]
                    ambulance_annot = AnnotationBbox(ambulance_icon, (x, y), frameon=False, zorder=12)
                    ax.add_artist(ambulance_annot)

                    artists.append(path_line)
                    artists.append(ambulance_annot)
                    break
                elif ev == "no_path":
                    print("No path found to that node.")
                    dijkstra_gen = None
                    break
        except StopIteration:
            dijkstra_gen = None

    colors = [node_color_map[n] for n in node_list]
    scat.set_color(colors)
    artists.append(scat)

    if path_line is not None:
        artists.append(path_line)

    return artists

# -----------------------
# Start dijkstra on new target node
# -----------------------
def start_dijkstra_to(target_node):
    global dijkstra_gen, node_color_map, ambulance_annot, current_target_node

    current_target_node = target_node
    # reset colors (keep base color)
    for n in node_list:
        node_color_map[n] = DEFAULT_NODE_COLOR
    node_color_map[base_node] = BASE_COLOR
    node_color_map[target_node] = TARGET_COLOR
    if ambulance_annot is not None:
        ambulance_annot.remove()
        ambulance_annot = None
    scat.set_color([node_color_map[n] for n in node_list])
    dijkstra_gen = dijkstra_generator(G, base_node, target_node)

# -----------------------
# Click handler
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
    print(f"Selected emergency node: {nearest}")
    start_dijkstra_to(nearest)

# -----------------------
# Connect events and start animation
# -----------------------
cid = fig.canvas.mpl_connect("button_press_event", on_click)
anim = animation.FuncAnimation(fig, anim_step, interval=ANIM_INTERVAL_MS, blit=False)

print("Ready — click on the map to select an emergency site (nearest intersection will be used).")
plt.show()