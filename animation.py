import osmnx as ox
import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import heapq
from matplotlib.offsetbox import OffsetImage, AnnotationBbox

# -----------------------
# CONFIG
# -----------------------
PLACE_NAME = "Karol Bagh, New Delhi, India"
NETWORK_DIST_METERS = 900
ANIM_INTERVAL_MS = 50  # animation frame interval (fast enough)
VISIT_COLOR = "#FFD700"
PATH_COLOR = "#FF3333"
BASE_COLOR = "#00BFFF"
TARGET_COLOR = "#FF4500"
DEFAULT_NODE_COLOR = "#111111"
EDGE_COLOR = "#888888"
NODE_SIZE = 10

# -----------------------
# LOAD GRAPH
# -----------------------
print("⏳ Geocoding place and downloading road network (may take a few seconds)...")
center_point = ox.geocode(PLACE_NAME)
G = ox.graph_from_point(center_point, dist=NETWORK_DIST_METERS, network_type="drive", simplify=True)
G = G.to_undirected()
pos = {n: (data["x"], data["y"]) for n, data in G.nodes(data=True)}

base_node = ox.distance.nearest_nodes(G, center_point[1], center_point[0])
print(f"Base station node (A) chosen: {base_node}")

fig, ax = ox.plot_graph(G, show=False, close=False, node_size=0, edge_color=EDGE_COLOR, bgcolor="white")
fig.canvas.manager.set_window_title("Interactive Emergency Planner — Dijkstra + Ambulance")

node_list = list(G.nodes())
node_colors = [DEFAULT_NODE_COLOR for _ in node_list]
node_xs = [pos[n][0] for n in node_list]
node_ys = [pos[n][1] for n in node_list]

scat = ax.scatter(node_xs, node_ys, s=NODE_SIZE, c=node_colors, zorder=3)

base_idx = node_list.index(base_node)
node_colors[base_idx] = BASE_COLOR
scat.set_color(node_colors)

ax.text(pos[base_node][0], pos[base_node][1], " A ", color="white",
        bbox=dict(facecolor=BASE_COLOR, edgecolor='none', pad=1), zorder=10, fontsize=8)

# -----------------------
# DIJKSTRA GENERATOR
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
# GLOBALS for animation
# -----------------------
node_color_map = {n: DEFAULT_NODE_COLOR for n in node_list}
node_color_map[base_node] = BASE_COLOR

path_line = None
dijkstra_gen = None
ambulance_annot = None
current_target_node = None
path_nodes = []
ambulance_index = 0
ambulance_moving = False
frames_since_path_found = 0
AMBULANCE_MOVE_FRAMES = 12  # frames to wait before moving ambulance to next node

# Load ambulance image
ambulance_img = plt.imread("ambulance.png")
ambulance_icon = OffsetImage(ambulance_img, zoom=0.05)

# -----------------------
# DRAW PATH
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
    fig.canvas.draw_idle()

# -----------------------
# ANIMATION STEP
# -----------------------
def anim_step(frame):
    global dijkstra_gen, ambulance_annot, path_nodes, ambulance_index
    global ambulance_moving, frames_since_path_found

    artists = [scat]

    if dijkstra_gen is not None:
        # Still running dijkstra animation
        try:
            for _ in range(6):
                ev, payload = next(dijkstra_gen)
                if ev == "visit":
                    node_color_map[payload] = VISIT_COLOR
                elif ev == "relax":
                    if node_color_map[payload] == DEFAULT_NODE_COLOR:
                        node_color_map[payload] = "#FFC87C"
                elif ev == "done":
                    path_nodes[:] = payload
                    for n in path_nodes:
                        node_color_map[n] = PATH_COLOR
                    draw_path_on_ax(path_nodes)
                    dijkstra_gen = None
                    ambulance_index = 0
                    frames_since_path_found = 0
                    ambulance_moving = True

                    # Place ambulance at start node
                    if ambulance_annot is not None:
                        ambulance_annot.remove()
                    x, y = pos[path_nodes[0]]
                    ambulance_annot = AnnotationBbox(ambulance_icon, (x, y), frameon=False, zorder=12)
                    ax.add_artist(ambulance_annot)
                    break
                elif ev == "no_path":
                    print("No path found.")
                    dijkstra_gen = None
                    ambulance_moving = False
                    break
        except StopIteration:
            dijkstra_gen = None

    elif ambulance_moving:
        # Move ambulance node-by-node every N frames
        frames_since_path_found += 1
        if frames_since_path_found >= AMBULANCE_MOVE_FRAMES:
            frames_since_path_found = 0
            ambulance_index += 1
            if ambulance_index >= len(path_nodes):
                ambulance_moving = False  # reached end
            else:
                node = path_nodes[ambulance_index]
                x, y = pos[node]
                ambulance_annot.xy = (x, y)

    colors = [node_color_map[n] for n in node_list]
    scat.set_color(colors)
    artists.append(scat)
    if path_line is not None:
        artists.append(path_line)
    if ambulance_annot is not None:
        artists.append(ambulance_annot)

    return artists

# -----------------------
# Start Dijkstra
# -----------------------
def start_dijkstra_to(target_node):
    global dijkstra_gen, node_color_map, ambulance_annot, current_target_node
    global path_nodes, ambulance_index, ambulance_moving, frames_since_path_found

    current_target_node = target_node

    # reset node colors
    for n in node_list:
        node_color_map[n] = DEFAULT_NODE_COLOR
    node_color_map[base_node] = BASE_COLOR
    node_color_map[target_node] = TARGET_COLOR

    if ambulance_annot is not None:
        ambulance_annot.remove()
        ambulance_annot = None

    scat.set_color([node_color_map[n] for n in node_list])
    dijkstra_gen = dijkstra_generator(G, base_node, target_node)

    # reset ambulance vars
    path_nodes = []
    ambulance_index = 0
    ambulance_moving = False
    frames_since_path_found = 0

# -----------------------
# Mouse click handler
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