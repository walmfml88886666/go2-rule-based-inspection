"""Generate editable draw.io figure masters from published metric datasets.

Export the masters with the official draw.io Desktop CLI; see docs/visual-comparison.md.
"""

from pathlib import Path
import json, base64, xml.etree.ElementTree as ET
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from tools.point_cloud import read_xyz, map_view

R = Path(__file__).resolve().parents[1]
A = R / "assets"
D = R / "docs"
BG = "#0b1928"
FG = "#edf4fa"
MUT = "#a4bbcc"
MINT = "#54d7bc"
AMBER = "#f5b560"
BLUE = "#3986d7"
ORANGE = "#ce6a21"


def master(name, w, h, bg):
    root = ET.Element("mxfile", host="app.diagrams.net")
    dia = ET.SubElement(root, "diagram", name=name, id=name)
    model = ET.SubElement(
        dia,
        "mxGraphModel",
        dx=str(w),
        dy=str(h),
        page="1",
        pageWidth=str(w),
        pageHeight=str(h),
        background=bg,
    )
    graph = ET.SubElement(model, "root")
    ET.SubElement(graph, "mxCell", id="0")
    ET.SubElement(graph, "mxCell", id="1", parent="0")
    return root, graph


root, g = master("system-overview", 1600, 1080, BG)
n = 1


def box(
    g,
    x,
    y,
    w,
    h,
    text="",
    fill="none",
    stroke="none",
    size=26,
    color=FG,
    bold=False,
    extra="",
):
    global n
    n += 1
    style = f"rounded=0;whiteSpace=wrap;html=0;fillColor={fill};strokeColor={stroke};fontColor={color};fontSize={size};fontFamily=Helvetica;align=left;verticalAlign=middle;spacingLeft=12;fontStyle={1 if bold else 0};{extra}"
    c = ET.SubElement(
        g, "mxCell", id=str(n), value=text, style=style, vertex="1", parent="1"
    )
    ET.SubElement(
        c,
        "mxGeometry",
        x=str(x),
        y=str(y),
        width=str(w),
        height=str(h),
        attrib={"as": "geometry"},
    )
    return str(n)


def edge(g, x1, y1, x2, y2, color=MINT, dashed=False):
    global n
    n += 1
    c = ET.SubElement(
        g,
        "mxCell",
        id=str(n),
        edge="1",
        parent="1",
        style=f"endArrow=block;endFill=1;strokeWidth=2;strokeColor={color};dashed={int(dashed)};",
    )
    geo = ET.SubElement(c, "mxGeometry", relative="1", attrib={"as": "geometry"})
    ET.SubElement(geo, "mxPoint", x=str(x1), y=str(y1), attrib={"as": "sourcePoint"})
    ET.SubElement(geo, "mxPoint", x=str(x2), y=str(y2), attrib={"as": "targetPoint"})


box(
    g,
    30,
    24,
    1520,
    62,
    "From measured LiDAR maps to traceable Go2 inspection",
    size=43,
    bold=True,
)
box(
    g,
    32,
    89,
    1510,
    44,
    "A  System workflow · Operator-supervised planning and field execution",
    size=30,
    color=FG,
    bold=True,
)
labels = [
    ("01  SENSE", "XT16 + Go2 IMU", "LiDAR scans · inertial data"),
    ("02  MAP", "LIO-SAM", "3D point cloud · map poses"),
    ("03  CONSTRAIN", "Dual-layer grid", "Navigation · line of sight"),
    ("04  PLAN", "Dijkstra GridPath", "Reachability · ordered route"),
    ("05  EXECUTE", "Go2 EDU patrol", "Localize · walk · capture"),
]
for i, (cap, title, body) in enumerate(labels):
    x = 42 + i * 307
    box(g, x, 159, 280, 150, fill="#142e42", stroke="#35536a")
    box(g, x + 6, 166, 268, 36, cap, size=20, color=MINT, bold=True)
    box(g, x + 6, 205, 268, 43, title, size=29, bold=True)
    box(g, x + 6, 253, 268, 41, body, size=20, color=MUT)
    if i < 4:
        edge(g, x + 281, 235, x + 304, 235)
box(
    g,
    420,
    323,
    600,
    46,
    "Target labels → observation-pose checks",
    size=22,
    color=AMBER,
)
edge(g, 805, 324, 805, 310, AMBER, True)
box(
    g,
    42,
    382,
    1508,
    57,
    "FIELD EVIDENCE:     Recorded trajectory   /   Inspection photographs   /   Timestamped events",
    fill="#122638",
    stroke="#35536a",
    size=25,
)
edge(g, 1408, 310, 1408, 381)
for i, scene in enumerate(("indoor", "outdoor")):
    cloud = read_xyz(R / "datasets" / f"{scene}.pcd")
    mission = json.loads((R / "datasets" / f"{scene}-route.json").read_text())
    route = np.asarray(mission["planned_xy_m"])
    stops = np.asarray(
        [a["observe_xyz_m"][:2] for a in mission["stops"] if a["type"] == "inspection"]
    )
    if scene == "indoor":
        cloud = cloud[:, [1, 0, 2]] * [1, -1, 1]
        route = route[:, [1, 0]] * [1, -1]
        stops = stops[:, [1, 0]] * [1, -1]
    limits = np.percentile(cloud[:, 2], [5, 95])
    bounds = (route.min(0) - 4, route.max(0) + 4)
    pixels, project, scale = map_view(cloud, bounds, (728, 430), limits)
    im = Image.fromarray(pixels)
    dr = ImageDraw.Draw(im)
    dr.rectangle((10, 382, 130, 429), fill=BG)
    p = [tuple(a) for a in project(route)]
    dr.line(p, fill=BG, width=11)
    dr.line(p, fill=MINT, width=5)
    for x, y in project(stops):
        dr.ellipse((x - 5, y - 5, x + 5, y + 5), fill=AMBER, outline=BG, width=2)
    ft = ImageFont.truetype("DejaVuSans.ttf", 19)
    dr.line((20, 393, 20 + scale * 5, 393), fill=FG, width=3)
    dr.text((20, 399), "5 m", font=ft, fill=FG, stroke_width=1, stroke_fill=BG)
    path = A / f"{scene}-overview-panel.png"
    im.save(path)
    x = 42 + i * 780
    box(
        g,
        x,
        467,
        728,
        49,
        f'{"B" if i==0 else "C"}  {scene.capitalize()} · {len(stops)} inspection stops',
        size=32,
        bold=True,
    )
    data = base64.b64encode(path.read_bytes()).decode()
    box(
        g,
        x,
        530,
        728,
        430,
        extra=f"shape=image;imageAspect=0;image=data:image/png,{data};",
    )
    box(g, x, 530, 728, 430, stroke="#35536a")
    box(
        g,
        x,
        967,
        728,
        30,
        f"Height color: map Z, {limits[0]:.1f} to {limits[1]:.1f} m (5th–95th percentile)",
        size=20,
        color="#c7d7e3",
    )
box(g, 42, 1017, 620, 43, "2D PLAN VIEW · Height-colored LiDAR", size=25, color=FG)
box(g, 680, 1037, 65, 4, fill=MINT)
box(g, 750, 1017, 330, 43, "Planned route", size=25, color=FG)
box(g, 1100, 1031, 17, 17, fill=AMBER, extra="shape=ellipse;")
box(g, 1120, 1017, 430, 43, "Observation poses", size=25, color=FG)
ET.indent(root)
ET.ElementTree(root).write(
    D / "system-overview.drawio", encoding="utf-8", xml_declaration=True
)
# Principle: aligned input columns and explicitly corresponding output masks.
root, g = master("navigation-sight-principle", 1600, 1030, "#ffffff")
n = 1
FG = "#172d46"
MUT = "#536b80"
box(
    g,
    35,
    20,
    1510,
    65,
    "One measured point cloud, two independent obstacle tests",
    size=41,
    bold=True,
    color=FG,
)
box(
    g,
    37,
    88,
    1510,
    46,
    "Separate body clearance from higher occlusions before selecting routes and observation poses",
    size=24,
    color=MUT,
)
box(
    g,
    35,
    160,
    825,
    49,
    "A  SIDE ELEVATION · height above local ground",
    size=27,
    bold=True,
    color=FG,
)
base = 760
scale = 210
left = 185
right = 850
for a, b, fill in [(0.16, 0.6296, "#e4f0fc"), (0.9, 2.2, "#fff0dd")]:
    box(g, left, base - b * scale, right - left, (b - a) * scale, fill=fill)
for z in [0, 0.16, 0.6296, 0.9, 2.2]:
    y = base - z * scale
    box(
        g,
        38,
        y - 18,
        140,
        36,
        f"{z:.2f} m" if z in [0.9, 2.2] else f"{z:g} m",
        size=22,
        color=MUT,
        extra="align=right;",
    )
    box(g, left, y, right - left, 1, fill="#b8c8d6")
box(g, left, base, right - left, 4, fill="#50677b")
box(g, 210, 779, 630, 36, "Estimated local ground ĝc", size=23, color=FG)
# Three schematic objects, not fabricated sensor samples. Colors mark height-band membership.
for x, h, w in [(300, 0.4, 78), (520, 1.55, 98), (748, 2.05, 72)]:
    if x == 520:
        box(
            g,
            x - 45,
            base - h * scale,
            w,
            0.2 * scale,
            fill="#e4d2bc",
            stroke="#997b56",
        )
    else:
        box(
            g,
            x - w / 2,
            base - h * scale,
            w,
            h * scale,
            fill="#e8edf1",
            stroke="#8c9eac",
        )
    for z in (
        [0.26, 0.37] if x == 300 else [1.4, 1.5] if x == 520 else [0.3, 0.46, 1.2, 1.7]
    ):
        box(
            g,
            x - 7,
            base - z * scale - 7,
            14,
            14,
            fill=BLUE if z < 0.63 else ORANGE,
            extra="ellipse;shape=ellipse;",
        )
    box(
        g,
        x - 85,
        231,
        170,
        53,
        {300: "Low barrier", 520: "High obstruction", 748: "Tall obstacle"}[x],
        size=22,
        color=FG,
        bold=True,
        extra="align=center;",
    )
    box(
        g,
        x - 25,
        826,
        50,
        40,
        {300: "①", 520: "②", 748: "③"}[x],
        size=25,
        color=FG,
        bold=True,
        extra="align=center;",
    )
box(
    g,
    220,
    884,
    605,
    60,
    "Illustrative returns · heights drawn to scale\nEach column represents one XY cell",
    size=21,
    color=MUT,
)
# Arrow into categorical cells gives direct correspondence without conflating viewing cells and rays.
edge(g, 870, 530, 925, 530, "#8397a9")
box(g, 940, 160, 615, 49, "B  OUTPUT · matching XY cells", size=27, color=FG, bold=True)
for y, title, band, colors in [
    (
        242,
        "NAVIGATION MASK",
        "0.16 < relative height < 0.6296 m",
        [BLUE, "#ffffff", BLUE],
    ),
    (
        558,
        "LINE-OF-SIGHT MASK",
        "0.90 < relative height < 2.20 m",
        ["#ffffff", ORANGE, ORANGE],
    ),
]:
    box(
        g,
        940,
        y,
        615,
        46,
        title,
        size=26,
        color=BLUE if y == 242 else ORANGE,
        bold=True,
    )
    box(g, 940, y + 49, 615, 39, band, size=23, color=FG)
    for i, c in enumerate(colors):
        x = 978 + i * 173
        box(g, x, y + 104, 125, 125, fill=c, stroke="#a3b3c1", extra="align=center;")
        box(
            g,
            x,
            y + 232,
            135,
            37,
            ["① Barrier", "② High", "③ Tall"][i],
            size=20,
            color=FG,
            extra="align=center;",
        )
    box(
        g,
        950,
        y + 269,
        600,
        30,
        "Filled: ≥2 returns · Unfilled: <2 returns",
        size=21,
        color=MUT,
    )
box(
    g,
    940,
    886,
    615,
    91,
    "① Triggers navigation only\n② Triggers line-of-sight only\n③ Triggers both masks",
    size=23,
    color=FG,
)
box(
    g,
    35,
    990,
    1515,
    35,
    "Grid pitch: 0.15 m · Each band is tested separately · Unfilled cells pass only that band’s return-count test",
    size=22,
    color=MUT,
)
ET.indent(root)
ET.ElementTree(root).write(
    D / "navigation-sight-principle.drawio", encoding="utf-8", xml_declaration=True
)
print("Created editable overview and principle masters")
