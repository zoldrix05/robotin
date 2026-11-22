"""Build the SDF world in metres, radians and kilograms."""

from argparse import ArgumentParser
from math import atan2, cos, dist, hypot, pi, radians, sin, tan
from pathlib import Path
from xml.etree import ElementTree as ET

from PIL import Image, ImageDraw, ImageFont


def element(parent, tag, value=None, **attributes):
    child = ET.SubElement(parent, tag, attributes)
    if value is not None:
        child.text = str(value)
    return child


def surface(link, name, shape, dimensions, pose, color, friction=0.25, collision=True,
            transparency=None, visible=True):
    size_tag = SHAPE_SIZE[shape]
    if visible:
        visual = element(link, "visual", name=f"{name}_visual")
        element(visual, "pose", pose)
        element(element(element(visual, "geometry"), shape), size_tag, dimensions)
        paint(visual, color)
        if transparency is not None:
            element(visual, "transparency", transparency)
    if collision:
        solid = element(link, "collision", name=name)
        element(solid, "pose", pose)
        element(element(element(solid, "geometry"), shape), size_tag, dimensions)
        ode = element(element(element(solid, "surface"), "friction"), "ode")
        element(ode, "mu", friction)
        element(ode, "mu2", friction)


def paint(visual, color):
    material = element(visual, "material")
    element(material, "diffuse", color)
    element(material, "ambient", color)


def mesh(link, name, path, color):
    visual = element(link, "visual", name=name)
    element(element(element(visual, "geometry"), "mesh"), "uri", path.resolve().as_uri())
    paint(visual, color)


def static_link(world, name):
    model = element(world, "model", name=name)
    element(model, "static", "true")
    return element(model, "link", name="body")


def inertia(link, mass, diagonal, pose=None):
    body = element(link, "inertial")
    if pose is not None:
        element(body, "pose", pose)
    element(body, "mass", mass)
    tensor = element(body, "inertia")
    for name, value in zip(("ixx", "iyy", "izz"), diagonal, strict=True):
        element(tensor, name, value)


def hinge(model, name, parent, child, axis, limit=None):
    joint = element(model, "joint", name=name, type="revolute")
    element(joint, "parent", parent)
    element(joint, "child", child)
    direction = element(joint, "axis")
    element(direction, "xyz", axis)
    if limit is not None:
        bounds = element(direction, "limit")
        element(bounds, "lower", -limit)
        element(bounds, "upper", limit)
        element(bounds, "effort", 20)
        element(bounds, "velocity", 7)


def anchor(model, link):
    joint = element(model, "joint", name="anchor", type="fixed")
    element(joint, "parent", "world")
    element(joint, "child", link)


def drive_link(model, name, pose, mass, moment):
    link = element(model, "link", name=name)
    element(link, "pose", pose)
    inertia(link, mass, (moment, moment, moment))
    return link


parser = ArgumentParser()
parser.add_argument("world", type=Path)
parser.add_argument("--camera", action="store_true")
parser.add_argument("--count", type=int, choices=(1, 2, 3), default=3)
parser.add_argument("--speed", type=float, choices=(60.0, 100.0, 120.0), default=100.0)
parser.add_argument("--diameter", type=float, default=64.0)
parser.add_argument("--mass", type=float, default=.25)
parser.add_argument("--fixed-angle", type=float, choices=(-43.0, 0.0, 43.0))
args = parser.parse_args()
if not 55 <= args.diameter <= 95 or not .18 <= args.mass <= .32:
    parser.error("El diámetro debe estar entre 55 y 95 mm y la masa entre 0,18 y 0,32 kg")

SHAPE_SIZE = {"box": "size", "ellipsoid": "radii"}
sdf = ET.Element("sdf", version="1.10")
world = element(sdf, "world", name="robotin")
physics = element(world, "physics", name="physics", type="ignored")
element(physics, "max_step_size", 0.002)
element(physics, "real_time_factor", 1.0)
for library, system in (
    ("physics", "Physics"),
    ("user-commands", "UserCommands"),
    ("scene-broadcaster", "SceneBroadcaster"),
):
    element(world, "plugin", filename=f"gz-sim-{library}-system", name=f"gz::sim::systems::{system}")
plant = element(world, "plugin", filename="librobotin_plant.so", name="robotin::Plant")
element(plant, "count", args.count)
element(plant, "belt_speed", args.speed / 1000)
if args.fixed_angle is not None:
    element(plant, "fixed_angle", radians(args.fixed_angle))
sun = element(world, "light", name="overhead", type="directional")
element(sun, "pose", "0 0 2 0 0 0")
element(sun, "diffuse", "0.8 0.8 0.8 1")
element(sun, "direction", "-0.2 0.3 -1")

floor = static_link(world, "floor")
surface(floor, "floor", "box", "4 4 .04", "0.4 0 -0.021 0 0 0", "0.24 0.28 0.33 1")
steel = ".24 .34 .41 1"
orange = ".95 .49 .15 1"
dark = ".09 .13 .16 1"
belt_model = element(world, "model", name="conveyor")
belt = drive_link(belt_model, "frame", "0 0 0 0 0 0", 3, .03)
anchor(belt_model, "frame")
surface(belt, "belt", "box", ".50 .15 .005", ".25 0 .3975 0 0 0", dark, 0.03)
surface(belt, "return_belt", "box", ".45 .15 .004", ".25 0 .348 0 0 0", dark, collision=False)
surface(belt, "bed", "box", ".45 .14 .030", ".25 0 .370 0 0 0", steel, collision=False)
if args.count == 3:
    surface(belt, "loading_tray", "box", ".17 .15 .007", "-.085 0 .3965 0 0 0", steel, .03)
    for y in (-.077, .077):
        surface(belt, f"loading_edge_{y}", "box", ".17 .006 .022",
                f"-.085 {y} .405 0 0 0", steel, collision=False)
    surface(belt, "loading_bracket", "box", ".022 .128 .14", "-.065 0 .325 0 .6 0",
            steel, collision=False)
for sign in (-1, 1):
    y = sign * .084
    surface(belt, f"frame_{sign}", "box", ".48 .014 .025",
            f".25 {y} .358 0 0 0", steel, collision=False)
    surface(belt, f"crossbar_{sign}", "box", ".44 .016 .018",
            f".25 {y} .145 0 0 0", steel, collision=False)
    for x in (.039, .46):
        surface(belt, f"leg_{sign}_{x}", "box", ".020 .020 .347",
                f"{x} {y} .174 0 0 0", steel, collision=False)
        surface(belt, f"foot_{sign}_{x}", "box", ".042 .042 .012",
                f"{x} {y} .007 0 0 0", dark, collision=False)


def cylinder(link, name, radius, length, pose, color):
    visual = element(link, "visual", name=name)
    element(visual, "pose", pose)
    geometry = element(element(visual, "geometry"), "cylinder")
    element(geometry, "radius", radius)
    element(geometry, "length", length)
    material = element(visual, "material")
    element(material, "diffuse", color)
    element(material, "ambient", color)


for name, x in (("drive_roller", .025), ("return_roller", .475)):
    roller_link = f"{name}_body"
    roller = drive_link(belt_model, roller_link, f"{x} 0 .375 0 0 0", .15, .00008)
    hinge(belt_model, name, "frame", roller_link, "0 1 0")
    cylinder(roller, "roller", .025, .150, f"0 0 0 {pi / 2} 0 0", dark)
    cylinder(roller, "shaft", .004, .20, f"0 0 0 {pi / 2} 0 0", steel)
    surface(roller, "rotation_mark", "box", ".045 .002 .004", "0 -.091 0 0 0 0",
            orange, collision=False)
    for sign in (-1, 1):
        cylinder(belt, f"bearing_{name}_{sign}", .012, .010,
                 f"{x} {sign * .081} .375 {pi / 2} 0 0", steel)
surface(belt, "gearbox", "box", ".044 .026 .044", ".025 -.102 .375 0 0 0", steel, collision=False)
cylinder(belt, "gearmotor_12v", .024, .064, f".025 -.148 .375 {pi / 2} 0 0", orange)
cylinder(belt, "motor_end_cap", .023, .006, f".025 -.182 .375 {pi / 2} 0 0", dark)
for i in range(7):
    model = element(world, "model", name=f"belt_cleat_{i}")
    cleat_poses = (
        ".025 0 .403 0 0 0", ".179 0 .403 0 0 0", ".332 0 .403 0 0 0",
        ".486 0 .401 0 .39 0", ".398 0 .347 0 3.14159 0",
        ".244 0 .347 0 3.14159 0", ".090 0 .347 0 3.14159 0",
    )
    element(model, "pose", cleat_poses[i])
    cleat = drive_link(model, "body", "0 0 0 0 0 0", .004, .000001)
    element(cleat, "gravity", "false")
    surface(cleat, "eva", "box", ".010 .125 .004", "0 0 0 0 0 0",
            ".29 .41 .45 1", collision=False)

tunnel = static_link(world, "optical_tunnel")
surface(tunnel, "back_panel", "box", ".192 .006 .24", ".25 .091 .518 0 0 0", dark, collision=False)
cover = static_link(world, "inspection_cover")
surface(cover, "inspection_panel", "box", ".192 .005 .225", ".25 -.091 .525 0 0 0",
        ".21 .35 .40 1", collision=False, transparency=.74)
surface(cover, "roof", "box", ".204 .196 .014", ".25 0 .652 0 0 0", dark, collision=False)
for y in (-.094, .094):
    surface(cover, f"roof_edge_{y}", "box", ".204 .006 .010",
            f".25 {y} .641 0 0 0", steel, collision=False)
for x in (.155, .345):
    for y in (-.092, .092):
        surface(tunnel, f"post_{x}_{y}", "box", ".013 .012 .244",
                f"{x} {y} .521 0 0 0", steel, collision=False)
for y in (-.073, .073):
    surface(tunnel, f"diffuser_{y}", "box", ".160 .010 .006",
            f".250 {y} .635 0 0 0", "1 .93 .77 1", collision=False)
    led = element(tunnel, "light", name=f"led_{y}", type="point")
    element(led, "pose", f".250 {y} .629 0 0 0")
    element(led, "visualize", "false")
    element(led, "diffuse", "0.8 0.76 0.68 1")
    element(led, "cast_shadows", "false")
    attenuation = element(led, "attenuation")
    element(attenuation, "range", .6)
    element(attenuation, "constant", .9)
    element(attenuation, "linear", .8)
for y in (-.087, .087):
    surface(tunnel, f"ir_housing_{y}", "box", ".014 .018 .024",
            f".185 {y} .442 0 0 0", orange, collision=False)
beam = ".003 .167 .002"
surface(tunnel, "ir_reference", "box", beam, ".185 0 .442 0 0 0",
        "1 .18 .06 1", collision=False, transparency=.45)
surface(tunnel, "camera_board", "box", ".029 .027 .004",
        ".25 0 .639 0 0 0", ".05 .37 .27 1", collision=False)
cylinder(tunnel, "camera_lens", .009, .009, ".25 0 .632 0 0 0", dark)

if args.camera:
    sensors = element(world, "plugin", filename="gz-sim-sensors-system", name="gz::sim::systems::Sensors")
    element(sensors, "render_engine", "ogre2")
    imager = element(tunnel, "sensor", name="overhead_camera", type="camera")
    element(imager, "pose", f".25 0 .626 0 {pi / 2} 0")
    element(imager, "topic", "/robotin/camera/image")
    element(imager, "always_on", "true")
    element(imager, "visualize", "false")
    element(imager, "update_rate", 20)
    optics = element(imager, "camera")
    element(optics, "horizontal_fov", 1.02 if args.diameter > 80 else .90)
    image = element(optics, "image")
    element(image, "width", 320)
    element(image, "height", 320)
    element(image, "format", "R8G8B8")
    clip = element(optics, "clip")
    element(clip, "near", .015)
    element(clip, "far", 1.0)

diverter = element(world, "model", name="diverter")
element(diverter, "pose", ".5 0 .365 0 0 0")
ramp = element(diverter, "link", name="chute")
base = drive_link(diverter, "base", "0 0 0 0 0 0", 2, .01)
inertia(ramp, .27, (.0006, .0025, .0030), ".09 0 -.05 0 0 0")
anchor(diverter, "base")
hinge(diverter, "yaw", "base", "chute", "0 0 1", .79)
surface(base, "support_column", "box", ".025 .025 .208",
        "0 0 -.261 0 0 0", steel, collision=False)
surface(base, "mounting_plate", "box", ".105 .14 .009",
        "0 -.03 -.095 0 0 0", steel, collision=False)
cylinder(base, "bearing_608", .011, .007, "0 0 -.084 0 0 0", ".68 .72 .73 1")
cylinder(ramp, "pivot_shaft", .004, .082, "0 0 -.044 0 0 0", dark)

# Parallel 40 mm horns give a 1:1 yaw transfer with an 80 mm connecting rod.
servo_y, horn = -.08, .040
surface(base, "mg996r_housing", "box", ".043 .021 .042",
        f"0 {servo_y} -.080 0 0 0", dark, collision=False)
surface(base, "servo_mount", "box", ".055 .028 .004",
        f"0 {servo_y} -.099 0 0 0", orange, collision=False)
arm = drive_link(diverter, "servo_arm", f"0 {servo_y} -.054 0 0 0", .025, .00003)
hinge(diverter, "servo_shaft", "base", "servo_arm", "0 0 1", .80)
cylinder(arm, "servo_gear", .010, .006, "0 0 0 0 0 0", orange)
surface(arm, "servo_horn", "box", ".042 .010 .004",
        ".020 0 0 0 0 0", orange, collision=False)
surface(ramp, "pivot_lever", "box", ".042 .009 .004",
        ".020 0 -.054 0 0 0", orange, collision=False)
rod = drive_link(diverter, "rod", f"{horn} {servo_y} -.051 0 0 0", .014, .00002)
hinge(diverter, "rod_hinge", "servo_arm", "rod", "0 0 1")
cylinder(rod, "steel_rod", .0022, .080, f"0 .040 0 {pi / 2} 0 0", ".71 .74 .76 1")
for y in (0, .080):
    cylinder(rod, f"rod_pin_{y}", .004, .009, f"0 {y} 0 0 0 0", orange)

# The bottom reaches under the belt. Trim only the highest strips at the inlet
# so the bowl can turn without hitting the upper belt surface.
slope = radians(28)
length = .260
overlap = .010
inner_radius = .051
thickness = .004
segment = pi / 12
channel_mesh = args.world.with_suffix(".obj")
def inlet(theta):
    fraction = max(0.0, (abs(theta) - radians(60)) / radians(18))
    return min(1.0, fraction) * (.035 + overlap) / cos(slope)


def flare(distance):
    return 1 + .32 * max(0.0, 1 - distance / .14)


def channel_point(distance, theta, radius):
    y = radius * sin(theta) * flare(distance)
    height = inner_radius - radius * cos(theta)
    return (
        distance * cos(slope) - overlap + height * sin(slope),
        y,
        -distance * sin(slope) + height * cos(slope),
    )


def write_channel(path):
    # Continuous inner and outer surfaces joined at the exposed lips.
    slices = 48
    rings = 25
    palette = path.with_suffix(".mtl")
    palette.write_text("newmtl chute\nKa 0.20 0.26 0.28\nKd 0.39 0.55 0.57\n"
                       "Ks 0.12 0.15 0.15\nNs 36\n", encoding="ascii")
    with path.open("w", encoding="ascii") as stream:
        stream.write(f"mtllib {palette.name}\nusemtl chute\n")
        for wall in (inner_radius, inner_radius + thickness):
            for section in range(rings):
                progress = section / (rings - 1)
                for i in range(slices + 1):
                    theta = i * pi / slices - pi / 2
                    start = inlet(theta)
                    distance = start + (length - start) * progress
                    x, y, z = channel_point(distance, theta, wall)
                    stream.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")
        for direction in (1, -1):
            for _ in range(rings):
                for i in range(slices + 1):
                    theta = i * pi / slices - pi / 2
                    nx, ny, nz = (sin(slope) * cos(theta), -sin(theta), cos(slope) * cos(theta))
                    stream.write(f"vn {direction * nx:.6f} {direction * ny:.6f} "
                                 f"{direction * nz:.6f}\n")
        def face(*indices):
            stream.write("f " + " ".join(f"{i}//{i}" for i in indices) + "\n")

        stride = slices + 1
        outer = rings * stride
        stream.write("s 1\n")
        for section in range(rings - 1):
            for i in range(slices):
                a = section * stride + i + 1
                b = a + stride
                face(a, b, b + 1, a + 1)
                face(a + outer, a + 1 + outer, b + 1 + outer, b + outer)
        stream.write("s off\n")
        for i in range(slices):
            a = i + 1
            b = (rings - 1) * stride + a
            face(a, a + 1, a + 1 + outer, a + outer)
            face(b, b + outer, b + 1 + outer, b + 1)
        for section in range(rings - 1):
            a = section * stride + 1
            b = a + stride
            face(a, a + outer, b + outer, b)
            face(a + slices, b + slices, b + slices + outer, a + slices + outer)


write_channel(channel_mesh)
mesh(ramp, "lined_chute", channel_mesh, ".66 .75 .75 1")
for i in range(12):
    theta = (i + .5) * segment - pi / 2
    sections = (inlet(theta), .14, length)
    for part, (start, end) in enumerate(zip(sections, sections[1:])):
        start_point = channel_point(start, theta, inner_radius + thickness / 2)
        end_point = channel_point(end, theta, inner_radius + thickness / 2)
        x, y, z = ((a + b) / 2 for a, b in zip(start_point, end_point, strict=True))
        dx, dy, dz = (b - a for a, b in zip(start_point, end_point, strict=True))
        yaw = atan2(dy, dx)
        pitch = atan2(-dz, hypot(dx, dy))
        width = 2 * inner_radius * tan(segment / 2) * flare(start) + .0013
        section_length = dist(start_point, end_point) + .002
        surface(ramp, f"channel_{i}_{part}", "box", f"{section_length} {width} {thickness}",
                f"{x} {y} {z} {theta} {pitch} {yaw}", "", .15, visible=False)

bins = static_link(world, "collector")


def bin_sign(link, label, text, y):
    plate = args.world.with_name(f"{label}_letters.obj")
    palette = plate.with_suffix(".mtl")
    palette.write_text("newmtl letters\nKa 0.015 0.026 0.032\nKd 0.025 0.048 0.058\n",
                       encoding="ascii")
    image = Image.new("1", (96, 26))
    draw = ImageDraw.Draw(image)
    draw.text((48, 13), text.upper(), anchor="mm", fill=1,
              font=ImageFont.load_default(size=16))
    pixels = image.load()
    with plate.open("w", encoding="ascii") as stream:
        stream.write(f"mtllib {palette.name}\nusemtl letters\nvn 0 0 1\n")
        index = 1
        for row in range(image.height):
            x = -.019 + row * .00145
            start = None
            for column in range(image.width + 1):
                filled = column < image.width and pixels[column, row]
                if filled and start is None:
                    start = column
                if not filled and start is not None:
                    lower, upper = ((edge - 48) * .00125 for edge in (start, column))
                    for px, py in ((x + .00145, lower), (x + .00145, upper),
                                   (x, upper), (x, lower)):
                        stream.write(f"v {px:.6f} {py:.6f} 0\n")
                    stream.write("f " + " ".join(f"{v}//1" for v in range(index, index + 4)) + "\n")
                    index += 4
                    start = None
    visual = element(link, "visual", name=f"{label}_sign")
    element(visual, "pose", f"1.038 {y} .283 0 0 0")
    element(element(element(visual, "geometry"), "mesh"), "uri", plate.resolve().as_uri())
    element(visual, "cast_shadows", "false")
    paint(visual, ".035 .055 .065 1")


for label, text, y, color, sign in (
    ("storage", "Almacén", 0.15, ".24 .58 .32 1", ".65 .85 .73 1"),
    ("consumption", "Consumo", 0.0, ".75 .59 .22 1", ".92 .80 .49 1"),
    ("discard", "Descarte", -0.15, ".66 .28 .28 1", ".89 .66 .62 1"),
):
    surface(bins, label, "box", ".45 .15 .02", f".820 {y} .148 0 0 0", color, 0.6)
    surface(bins, f"{label}_label_mount", "box", ".053 .134 .006", f"1.038 {y} .276 0 0 0",
            sign, collision=False)
    bin_sign(bins, label, text, y)
for y in (-0.075, 0.075):
    surface(bins, f"partition_{y}", "box", ".43 .008 .067",
            f".827 {y} .1915 0 0 0", ".44 .48 .54 1", 0.6)
for y in (-0.225, 0.225):
    direction = 1 if y > 0 else -1
    surface(bins, f"inlet_floor_{y}", "box", ".196 .030 .020",
            f".693 {y + .015 * direction} .148 0 0 0", ".32 .41 .46 1", 0.6)
    # Widen the outside of the inlet until the fruit has cleared the moving outlet.
    surface(bins, f"outer_inlet_{y}", "box", ".196 .008 .034",
            f".693 {y + .030 * direction} .175 0 0 0", ".32 .41 .46 1", 0.6)
    surface(bins, f"outer_wall_{y}", "box", ".254 .008 .13",
            f".918 {y} .223 0 0 0", ".32 .41 .46 1", 0.6)
surface(bins, "end_stop", "box", ".012 .45 .12", "1.039 0 .215 0 0 0", ".44 .48 .54 1", 0.6)

fruit_colors = (".31 .48 .12 1", ".22 .15 .14 1", ".32 .29 .15 1")
radius = args.diameter / 2000
length_radius = radius * 1.4375
for i in range(args.count):
    x = .175 - i * .150
    model = element(world, "model", name=f"fruit_{i}")
    element(model, "pose", f"{x} 0 {.401 + radius} 0 0 0")
    body = element(model, "link", name="body")
    longitudinal = args.mass * (radius ** 2 + radius ** 2) / 5
    transverse = args.mass * (length_radius ** 2 + radius ** 2) / 5
    inertia(body, args.mass, (longitudinal, transverse, transverse))
    surface(body, "avocado", "ellipsoid", f"{length_radius} {radius} {radius}", "0 0 0 0 0 0",
            fruit_colors[i], .25)

ET.indent(sdf)
ET.ElementTree(sdf).write(args.world, encoding="utf-8", xml_declaration=True)
