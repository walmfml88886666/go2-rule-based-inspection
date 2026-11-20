"""Generate editable draw.io figure masters from published metric datasets.

Export the masters with the official draw.io Desktop CLI; see docs/visual-comparison.md.
"""

from pathlib import Path
import base64
import io
import json
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from tools.point_cloud import read_xyz, map_view

R = Path(__file__).resolve().parents[1]
D = R / "docs"
BG = "#f5f7f5"
FG = "#122827"
MUT = "#4f6562"
MINT = "#007c72"
AMBER = "#b35a28"


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


root, g = master("system-overview", 1600, 850, BG)
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


def save(name):
    ET.indent(root)
    ET.ElementTree(root).write(
        D / f"{name}.drawio", encoding="utf-8", xml_declaration=True
    )


def label(x, y, w, h, value, *, size=28, color=FG, bold=False, fill="none"):
    box(g, x, y, w, h, value, size=size, color=color, bold=bold, fill=fill)


label(28, 16, 1544, 68, "From LiDAR survey to inspection route", size=54, bold=True)
label(30, 92, 1540, 46, "", fill="#e5eeea")
steps = [
    ("Scan & map", "XT16 + IMU → LIO-SAM"),
    ("Check view poses", "Clearance + line of sight"),
    ("Plan the route", "Dijkstra grid search"),
    ("Walk & inspect", "Go2 EDU → photos and logs"),
]
for i, (heading, detail) in enumerate(steps):
    x = 43 + 384 * i
    label(x, 91, 338, 44, heading, size=31, bold=True, color=MINT)
    label(x, 137, 355, 42, detail, size=26, color=MUT)
    if i < 3:
        edge(g, x + 336, 114, x + 367, 114, MINT)
label(
    43,
    184,
    1490,
    34,
    "Operator annotations feed the observation-pose checks.",
    size=24,
    color=MUT,
)

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
    pixels, project, scale = map_view(
        cloud,
        (route.min(0) - 2, route.max(0) + 2),
        (746, 455),
        limits,
        palette=((189, 206, 211), (101, 131, 146), (43, 65, 86)),
        background=(236, 242, 240),
    )
    im = Image.fromarray(pixels)
    draw = ImageDraw.Draw(im)
    draw.line(list(map(tuple, project(route))), fill="#ffffff", width=10)
    draw.line(list(map(tuple, project(route))), fill=MINT, width=5)
    for x, y in project(stops):
        draw.ellipse((x - 5, y - 5, x + 5, y + 5), fill=AMBER, outline="white", width=2)
    draw.rectangle((10, 399, 116, 451), fill=BG)
    draw.line((20, 410, 20 + scale * 5, 410), fill=FG, width=3)
    draw.text((20, 415), "5 m", font=ImageFont.truetype("DejaVuSans.ttf", 25), fill=FG)
    buffer = io.BytesIO()
    im.save(buffer, format="PNG")
    data = base64.b64encode(buffer.getvalue()).decode()
    x = 30 + i * 794
    label(
        x,
        229,
        740,
        58,
        f"{scene.title()} · {len(stops)} planned stops",
        size=39,
        bold=True,
    )
    box(
        g,
        x,
        294,
        746,
        455,
        extra=f"shape=image;imageAspect=0;image=data:image/png,{data};",
    )
    label(
        x,
        752,
        746,
        36,
        f"Map height {limits[0]:.1f} to {limits[1]:.1f} m · light to dark",
        size=24,
        color=MUT,
    )

box(g, 43, 814, 63, 5, fill=MINT)
label(108, 796, 245, 42, "Planned route", size=26)
box(g, 379, 805, 20, 20, fill=AMBER, extra="shape=ellipse;")
label(405, 796, 370, 42, "Observation pose", size=26)
save("system-overview")

root, g = master("navigation-sight-principle", 1600, 814, BG)
n = 1
label(29, 18, 1540, 72, "Which returns block motion or sight?", size=54, bold=True)
label(
    32,
    91,
    1540,
    45,
    "Count returns independently in two height bands within each 0.15 m grid cell.",
    size=31,
    color=MUT,
)
floor = 535
scale = 150
nav = "#006b82"
sight = "#994524"
for lo, hi, color in (
    (0.16, 0.6296, "#dceff3"),
    (0.90, 2.20, "#f7e9e0"),
):
    label(240, floor - hi * scale, 1310, (hi - lo) * scale, "", fill=color)
for h, tick in [
    (0, "0"),
    (0.16, "0.16"),
    (0.6296, "0.6296"),
    (0.90, "0.90"),
    (2.20, "2.20"),
]:
    y = floor - h * scale
    label(32, y - (10 if h == 0 else 18), 200, 36, tick + " m", size=26, color=MUT)
    box(g, 238, y, 1315, 2, fill="#9bafab")
box(g, 238, floor, 1315, 4, fill=FG)
label(27, 552, 385, 40, "Estimated local ground", size=26, bold=True)
scenarios = [
    ("Low barrier", 490, [(0.0, 0.40)], [0.25, 0.35], (2, 0)),
    ("Overhead obstruction", 911, [(1.24, 1.56)], [1.31, 1.47], (0, 2)),
    ("Tall obstacle", 1322, [(0, 1.97)], [0.31, 0.53, 1.25, 1.72], (2, 2)),
]
for title, x, spans, returns, counts in scenarios:
    box(g, x - 196, 143, 392, 49, title, size=30, bold=True, extra="align=center;")
    for lo, hi in spans:
        box(
            g,
            x - 47,
            floor - hi * scale,
            94,
            (hi - lo) * scale,
            fill="#c1cdc9",
            stroke="#687d77",
        )
    for z in returns:
        box(
            g,
            x - 8,
            floor - z * scale - 8,
            16,
            16,
            fill=nav if z < 0.63 else sight,
            extra="shape=ellipse;",
        )
    for j, (count, color, tint) in enumerate(
        zip(counts, (nav, sight), ("#dceff3", "#f7e9e0"))
    ):
        box(
            g,
            x - 181,
            611 + 61 * j,
            362,
            49,
            f"{count} returns · " + ("flagged" if count >= 2 else "below threshold"),
            size=26,
            bold=count >= 2,
            color=color,
            fill=tint,
            extra="align=center;",
        )
label(27, 611, 276, 49, "Navigation", size=28, color=nav, bold=True)
label(27, 672, 276, 49, "Line of sight", size=28, color=sight, bold=True)
label(
    27,
    747,
    1533,
    44,
    "Illustrative returns  ·  Strict height bounds  ·  A band flags a cell at 2 or more returns",
    size=27,
    color=MUT,
)
save("navigation-sight-principle")
