"""Scene-derived post clearances and screen-projected tabletop rulers."""
import numpy as np
from PIL import ImageDraw, ImageFont


COLORS = {'X': '#ff8b8b', 'Y': '#83ed9a'}


def post_clearances(model, data, plate=None):
    """Signed axis clearances to the combined tabletop or one plate's edges.

    Read the actual rendered geometry, rather than the saved configuration.
    With plate=None, internal seams between the four tables are excluded.
    Negative means the post extends past that edge; these are horizontal,
    surface-to-edge offsets.
    """
    tables = ([model.geom(f'table_{row}_{column}')
               for row in range(2) for column in range(2)] if plate is None else
              [model.geom(f'table_{plate // 2}_{plate % 2}')])
    low = np.min([data.geom(table.id).xpos - table.size for table in tables], axis=0)
    high = np.max([data.geom(table.id).xpos + table.size for table in tables], axis=0)
    post = model.geom('mount_post')
    center = data.geom(post.id).xpos
    radius = float(post.size[0])
    rulers = []
    for axis in range(2):
        for sign, edge in ((-1, low[axis]), (1, high[axis])):
            start = np.array([center[0], center[1], high[2] + .006])
            start[axis] += sign * radius
            end = start.copy()
            end[axis] = edge
            edge_name = ('+' if sign > 0 else '-') + 'XY'[axis]
            rulers.append({'edge': edge_name,
                           'label': (f'P{plate + 1} ' if plate is not None else '') + edge_name,
                           'color': '#d6a6ff' if plate is not None else COLORS['XY'[axis]],
                           'distance': float(sign * (end[axis] - start[axis])),
                           'start': start, 'end': end})
    return rulers


def ruler_groups(model, data):
    """Outer edges, four numbered plate edge sets, and four adjacent gaps."""
    groups = [('Outer table edges', post_clearances(model, data))]
    for plate in range(4):
        groups.append((f'Plate {plate + 1} edges', post_clearances(model, data, plate)))
    gaps = []
    # Plate numbering: P1 (-X,-Y), P2 (+X,-Y), P3 (-X,+Y), P4 (+X,+Y).
    for a, b, axis in ((0, 1, 0), (2, 3, 0), (0, 2, 1), (1, 3, 1)):
        first = model.geom(f'table_{a // 2}_{a % 2}')
        second = model.geom(f'table_{b // 2}_{b % 2}')
        start = data.geom(first.id).xpos.copy()
        end = data.geom(second.id).xpos.copy()
        start[axis] += first.size[axis]
        end[axis] -= second.size[axis]
        start[2] = end[2] = max(data.geom(first.id).xpos[2] + first.size[2],
                              data.geom(second.id).xpos[2] + second.size[2]) + .006
        gaps.append({'edge': '+' + 'XY'[axis], 'label': f'P{a + 1}-P{b + 1}',
                     'color': '#6bdcff', 'distance': float(end[axis] - start[axis]),
                     'start': start, 'end': end})
    groups.append(('Plate gaps', gaps))
    return groups


def project(point, scene, width, height):
    """Project through MuJoCo's central render camera, in image pixels."""
    camera = scene.camera[0]
    position = (np.asarray(scene.camera[0].pos) + scene.camera[1].pos) / 2
    offset = np.asarray(point) - position
    forward, up = np.asarray(camera.forward), np.asarray(camera.up)
    right = np.cross(forward, up)
    right /= np.linalg.norm(right)
    depth = float(offset @ forward)
    if depth <= camera.frustum_near:
        return None
    scale = 1 if camera.orthographic else camera.frustum_near / depth
    span = camera.frustum_top - camera.frustum_bottom
    x = float(offset @ right) * scale - camera.frustum_center
    y = float(offset @ up) * scale
    return (width / 2 + x * height / span,
            (camera.frustum_top - y) * height / span)


def draw_post_clearances(frame, scene, rulers, title='Post to outer table edges', panel_y=12):
    """Draw annotation-only rulers; physics and sensor images stay untouched."""
    draw = ImageDraw.Draw(frame)
    for ruler in rulers:
        start = project(ruler['start'], scene, frame.width, frame.height)
        end = project(ruler['end'], scene, frame.width, frame.height)
        if start is None or end is None:
            continue
        # Avoid sending huge, offscreen coordinates to Pillow for distant edits.
        if not all(-frame.width < x < 2 * frame.width and
                   -frame.height < y < 2 * frame.height for x, y in (start, end)):
            continue
        color = '#ffb454' if ruler['distance'] < 0 else ruler.get('color', COLORS[ruler['edge'][1]])
        draw.line((start, end), fill=color, width=2)
        direction = np.asarray(end) - start
        length = np.linalg.norm(direction)
        normal = np.array([-direction[1], direction[0]]) / length if length > .1 else np.array([0, 1])
        for point in (start, end):
            a, b = np.asarray(point) - 5 * normal, np.asarray(point) + 5 * normal
            draw.line((tuple(a), tuple(b)), fill=color, width=2)
        midpoint = (np.asarray(start) + end) / 2 + 10 * normal
        label = f"{ruler.get('label', ruler['edge'])} {ruler['distance'] * 100:.2f} cm"
        text_width, text_height = ImageFont.load_default().getmask(label).size
        x, y = midpoint - np.array([text_width, text_height]) / 2
        draw.rectangle((x - 3, y - 3, x + text_width + 3, y + text_height + 3), fill='#10161e')
        draw.text((x, y), label, fill=color)
    draw.rounded_rectangle((12, panel_y, 340, panel_y + 73), radius=8, fill='#10161e')
    draw.text((22, panel_y + 8), title + ' (cm)', fill='white')
    for index, ruler in enumerate(rulers):
        color = '#ffb454' if ruler['distance'] < 0 else ruler.get('color', COLORS[ruler['edge'][1]])
        x, y = 22 + (index % 2) * 158, panel_y + 30 + (index // 2) * 20
        draw.text((x, y), f"{ruler.get('label', ruler['edge'])}: {ruler['distance'] * 100:.2f}", fill=color)
    return frame


def draw_ruler_groups(frame, scene, groups, mask):
    """Each browser chooses its own ruler groups via a six-bit mask."""
    selected = [(title, rulers) for bit, (title, rulers) in enumerate(groups) if mask & (1 << bit)]
    for index, (title, rulers) in enumerate(selected):
        draw_post_clearances(frame, scene, rulers, title, 12 + index * 79)
    if selected:
        draw = ImageDraw.Draw(frame)
        y = 12 + len(selected) * 79
        draw.rectangle((12, y, 340, y + 25), fill='#10161e')
        draw.text((22, y + 6), 'Horizontal; negative = beyond edge / overlap', fill='#b4c9e5')
    return frame
