import osmnx as ox
import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import heapq

# -----------------------
# CONFIG
# -----------------------
CENTER_POINT = (28.6139, 77.2090)  # New Delhi lat, lon
NETWORK_DIST_METERS = 900          # radius around center point to load graph
ANIM_INTERVAL_MS = 25              # animation frame interval in ms

VISIT_COLOR = "#FFD700"    # gold for visited
PATH_COLOR = "#FF3333"     # red for final path
BASE_COLOR = "#00BFFF"     # base station color (deep sky blue)
TARGET_COLOR = "#FF4500"   # target color (orange red)
DEFAULT_NODE_COLOR = "#111111"  # default node color (dark)
EDGE_COLOR = "#888888"     # edges color
NODE_SIZE = 10

# -----------------------
# DOWNLOAD / PREPARE GRAPH
# -----------------------
print("⏳ Downloading road network around New Delhi (may take a few seconds)...")
G = ox.graph_from_point(CENTER_POINT, dist=NETWORK_DIST_METERS, network_type="drive", simplify=True)

# convert to undirected for Dijkstra
G = G.to_undirected()

# get node coordinates for plotting
pos = {n: (data["x"], data["y"]) for n, data in G.nodes(data=True)}

# fixed base station node near center (A)
base_node = ox.distance.nearest_nodes(G, CENTER_POINT[1], CENTER_POINT[0])
print(f"Base station node (A) chosen: {base_node}")

# prepare matplotlib figure
fig, ax = ox.plot_graph(G, show=False, close=False, node_size=0, edge_color=EDGE_COLOR, bgcolor="white")
fig.canvas.manager.set_window_title("Interactive Emergency Planner — Dijkstra (click to pick emergency)")

# scatter plot for nodes (small dots)
node_list = list(G.nodes())
node_xs = [pos[n][0] for n in node_list]
node_ys = [pos[n][1] for n in node_list]
node_colors = [DEFAULT_NODE_COLOR for _ in node_list]

scat = ax.scatter(node_xs, node_ys, s=NODE_SIZE, c=node_colors, zorder=3)

# highlight base station
base_idx = node_list.index(base_node)
node_colors[base_idx] = BASE_COLOR
scat.set_color(node_colors)

# label base node "A"
ax.text(pos[base_node][0], pos[base_node][1], " A ", color="white",
        bbox=dict(facecolor=BASE_COLOR, edgecolor='none', pad=1), zorder=10, fontsize=8)

# animation state
dijkstra_gen = None
anim = None
current_target_node = None
visited_order = []

# Dijkstra generator
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

# node color map for animation
node_color_map = {n: DEFAULT_NODE_COLOR for n in node_list}
node_color_map[base_node] = BASE_COLOR

def refresh_scatter():
    colors = [node_color_map[n] for n in node_list]
    scat.set_color(colors)
    fig.canvas.draw_idle()

def start_dijkstra_to(target_node):
    global dijkstra_gen, visited_order, current_target_node
    current_target_node = target_node
    for n in node_list:
        node_color_map[n] = DEFAULT_NODE_COLOR
    node_color_map[base_node] = BASE_COLOR
    node_color_map[target_node] = TARGET_COLOR
    refresh_scatter()
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

path_line = None
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
    total = 0.0
    for i in range(len(path_nodes)-1):
        u = path_nodes[i]
        v = path_nodes[i+1]
        data = G.get_edge_data(u, v, default={})
        if isinstance(data, dict) and len(data) > 0 and 0 in data:
            lengths = [data[key].get('length', 0.0) for key in data]
            weight = min(lengths) if lengths else 0.0
        else:
            weight = data.get('length', 0.0)
        total += weight
    x_last, y_last = pos[path_nodes[-1]]
    ax.text(x_last, y_last, f" ETA: {total:.0f} m", color=PATH_COLOR, fontsize=8, zorder=9,
            bbox=dict(facecolor="white", alpha=0.7, edgecolor='none', pad=1))
    fig.canvas.draw_idle()

def anim_step(frames):
    global dijkstra_gen, visited_order
    if dijkstra_gen is None:
        return scat,

    try:
        for _ in range(6):
            ev, payload = next(dijkstra_gen)
            if ev == "visit":
                visited_order.append(payload)
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