import osmnx as ox
import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import heapq
import numpy as np

# -----------------------
# CONFIG
# -----------------------
CENTER_POINT = (28.6139, 77.2090)  # New Delhi lat, lon
NETWORK_DIST_METERS = 900          # radius in meters
ANIM_INTERVAL_MS = 30              # animation frame interval in ms

# Colors
VISIT_COLOR = "#FFC300"       # bright gold for visited
FRONTIER_COLOR = "#FFB347"   # lighter orange for frontier (relax)
PATH_COLOR = "#FF4500"        # vivid orange-red for final path
BASE_COLOR = "#1E90FF"        # dodger blue for base station
TARGET_COLOR = "#FF6347"      # tomato red for target node
DEFAULT_NODE_COLOR = "#555555"  # medium gray for default nodes
EDGE_COLOR = "#BBBBBB"         # light gray for edges
PATH_EDGE_COLOR = "#FF6347"   # match PATH_COLOR for edges on path

NODE_BASE_SIZE = 10
NODE_VISITED_SIZE = 35
NODE_PATH_SIZE = 50
NODE_BASE_TARGET_SIZE = 70

GLOW_ALPHA = 0.4  # alpha for glow effect

# -----------------------
# DOWNLOAD / PREPARE GRAPH
# -----------------------
print("⏳ Downloading road network around New Delhi (may take a few seconds)...")
G = ox.graph_from_point(CENTER_POINT, dist=NETWORK_DIST_METERS, network_type="drive", simplify=True)
G = G.to_undirected()

pos = {n: (data["x"], data["y"]) for n, data in G.nodes(data=True)}

base_node = ox.distance.nearest_nodes(G, CENTER_POINT[1], CENTER_POINT[0])
print(f"Base station node (A) chosen: {base_node}")

fig, ax = ox.plot_graph(G, show=False, close=False,
                       node_size=0, edge_color=EDGE_COLOR, bgcolor="white")
fig.canvas.manager.set_window_title("Interactive Emergency Planner — Enhanced Dijkstra")

node_list = list(G.nodes())
node_xs = [pos[n][0] for n in node_list]
node_ys = [pos[n][1] for n in node_list]

# Initialize node colors and sizes
node_color_map = {n: DEFAULT_NODE_COLOR for n in node_list}
node_size_map = {n: NODE_BASE_SIZE for n in node_list}
node_color_map[base_node] = BASE_COLOR
node_size_map[base_node] = NODE_BASE_TARGET_SIZE

scat = ax.scatter(node_xs, node_ys,
                  s=[node_size_map[n] for n in node_list],
                  c=[node_color_map[n] for n in node_list],
                  edgecolors='black', linewidth=0.4,
                  zorder=3)

# Text labels for base and target nodes
base_text = ax.text(pos[base_node][0], pos[base_node][1], " A ",
                    color="white", fontsize=10, fontweight='bold',
                    bbox=dict(facecolor=BASE_COLOR, edgecolor='none', pad=3),
                    zorder=10)

target_node = None
target_text = None

dijkstra_gen = None
path_line = None
path_edge_lines = []

visited_order = []

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
                lengths = [data[key].get('length', 1.0) for key in data]
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

def refresh_scatter():
    scat.set_color([node_color_map[n] for n in node_list])
    scat.set_sizes([node_size_map[n] for n in node_list])
    fig.canvas.draw_idle()

def draw_path_edges(path_nodes):
    global path_edge_lines
    # Remove old path edges
    for line in path_edge_lines:
        try:
            line.remove()
        except Exception:
            pass
    path_edge_lines.clear()
    # Draw new bold glowing edges with alpha pulse effect
    for i in range(len(path_nodes)-1):
        u = path_nodes[i]
        v = path_nodes[i+1]
        xs = [pos[u][0], pos[v][0]]
        ys = [pos[u][1], pos[v][1]]
        line, = ax.plot(xs, ys, color=PATH_EDGE_COLOR, linewidth=4, alpha=GLOW_ALPHA, zorder=7)
        path_edge_lines.append(line)

def animate_path_edges():
    # Pulse glow effect on path edges by changing alpha
    if not path_edge_lines:
        return
    alphas = np.abs(np.sin(animate_path_edges.phase))
    for line in path_edge_lines:
        line.set_alpha(GLOW_ALPHA + 0.3 * alphas)
    animate_path_edges.phase += 0.1
    fig.canvas.draw_idle()
animate_path_edges.phase = 0.0

def start_dijkstra_to(target):
    global dijkstra_gen, visited_order, target_node, target_text, path_line
    target_node = target
    # Reset nodes
    for n in node_list:
        node_color_map[n] = DEFAULT_NODE_COLOR
        node_size_map[n] = NODE_BASE_SIZE
    node_color_map[base_node] = BASE_COLOR
    node_size_map[base_node] = NODE_BASE_TARGET_SIZE
    node_color_map[target_node] = TARGET_COLOR
    node_size_map[target_node] = NODE_BASE_TARGET_SIZE
    refresh_scatter()

    # Remove previous target label
    global target_text
    if target_text is not None:
        try:
            target_text.remove()
        except Exception:
            pass
    target_text = ax.text(pos[target_node][0], pos[target_node][1], " B ",
                          color="white", fontsize=10, fontweight='bold',
                          bbox=dict(facecolor=TARGET_COLOR, edgecolor='none', pad=3),
                          zorder=10)

    # Remove previous path line and edges
    global path_line
    if path_line is not None:
        try:
            path_line.remove()
        except Exception:
            pass
    for line in path_edge_lines:
        try:
            line.remove()
        except Exception:
            pass
    path_edge_lines.clear()

    dijkstra_gen = dijkstra_generator(G, base_node, target_node)
    visited_order.clear()

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

def anim_step(frame):
    global dijkstra_gen, visited_order, path_line

    # Animate glowing edges if path exists
    animate_path_edges()

    if dijkstra_gen is None:
        return scat,

    try:
        for _ in range(6):
            ev, payload = next(dijkstra_gen)
            if ev == "visit":
                visited_order.append(payload)
                node_color_map[payload] = VISIT_COLOR
                node_size_map[payload] = NODE_VISITED_SIZE
            elif ev == "relax":
                if node_color_map[payload] == DEFAULT_NODE_COLOR:
                    node_color_map[payload] = FRONTIER_COLOR
                    node_size_map[payload] = NODE_VISITED_SIZE * 0.7
            elif ev == "done":
                path = payload
                for n in path:
                    node_color_map[n] = PATH_COLOR
                    node_size_map[n] = NODE_PATH_SIZE
                # Draw bold glowing path line
                xs = [pos[n][0] for n in path]
                ys = [pos[n][1] for n in path]
                if path_line is not None:
                    try:
                        path_line.remove()
                    except Exception:
                        pass
                path_line, = ax.plot(xs, ys, color=PATH_COLOR, linewidth=3.5, alpha=1.0, zorder=8)
                draw_path_edges(path)
                dijkstra_gen = None
                print("Path found:", path)
                break
            elif ev == "no_path":
                print("No path found to that node.")
                dijkstra_gen = None
                break
    except StopIteration:
        dijkstra_gen = None

    refresh_scatter()
    return scat,

cid = fig.canvas.mpl_connect("button_press_event", on_click)
anim = animation.FuncAnimation(fig, anim_step, interval=ANIM_INTERVAL_MS, blit=False)

print("Ready — click on the map to select an emergency site (nearest intersection will be used).")
plt.show()