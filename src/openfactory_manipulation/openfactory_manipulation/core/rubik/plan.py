"""Which flips and turns make the cube show its six faces, and which solve it (D-026).

The cell can turn only the face that is down, and bring another face down only by a flip:
one of a few quarter turns about a horizontal axis (`kinds`), after turning the held cube
`yaw` quarter turns about the vertical. A plan is a list of operations:

    ('flip', kind, yaw)      the cube is flipped
    ('turn', k)              the gripper makes k quarter turns (-1, 1 or 2)

All of it is search over the 24 orientations of the cube.
"""

import heapq
import itertools

from . import orientation as ori
from . import rotations as rot
from .cube import parse_move
from .view import up_direction


def kind_rotations(kinds):
    """{name: rotation of the nest frame a flip kind applies to the cube} from the `kinds` of
    rubik.yaml: a quarter turn about `finger_axis` that takes the downward direction to
    `tool_axis`."""
    rotations = {}
    for name, spec in kinds.items():
        finger = tuple(int(round(v)) for v in spec['finger_axis'])
        tool = tuple(int(round(v)) for v in spec['tool_axis'])
        rotations[name] = next(
            r for r in (rot.quarter_turn(finger, 1), rot.quarter_turn(finger, -1))
            if rot.mat_vec(r, ori.DOWN) == tool)
    return rotations


def operation_rotation(operation, kinds):
    """Rotation of the nest frame an operation applies to the cube frame."""
    if operation[0] == 'turn':
        return rot.quarter_turn(ori.UP, operation[1])
    return rot.mat_mul(kinds[operation[1]], rot.quarter_turn(ori.UP, operation[2]))


def flip_options(kinds, costs):
    """[(operation, rotation of the nest frame, cost)] for every flip the cell can make.

    `kinds`: {name: rotation the flip applies to the cube}; `costs`: seconds of a flip and of
    the extra turn in the air (`flip`, `yaw`).
    """
    options = []
    for kind in sorted(kinds):
        for yaw in range(4):
            rotation = rot.mat_mul(kinds[kind], rot.quarter_turn(ori.UP, yaw))
            options.append((('flip', kind, yaw), rotation,
                            costs['flip'] + (costs['yaw'] if yaw else 0.0)))
    return options


def _shortest(options):
    """{rotation: (cost, [operations])}: the cheapest flips whose product is each of the 24
    rotations (applied in order: the first operation first)."""
    best = {rot.IDENTITY: (0.0, [])}
    counter = itertools.count()
    queue = [(0.0, next(counter), rot.IDENTITY, [])]
    while queue:
        cost, _, done, ops = heapq.heappop(queue)
        if best[done][0] < cost:
            continue
        for op, rotation, step in options:
            after = rot.mat_mul(rotation, done)
            if after not in best or cost + step < best[after][0] - 1e-9:
                best[after] = (cost + step, ops + [op])
                heapq.heappush(queue, (cost + step, next(counter), after, ops + [op]))
    return best


def scan_plan(options):
    """The cheapest flips after which every face of the cube has been up once, whatever its
    orientation at the start (the first view is taken before any flip)."""
    start = (rot.IDENTITY, frozenset([up_direction(rot.IDENTITY)]))
    best = {start: 0.0}
    counter = itertools.count()
    queue = [(0.0, next(counter), start, [])]
    while queue:
        cost, _, (done, seen), ops = heapq.heappop(queue)
        if len(seen) == 6:
            return ops
        if best[(done, seen)] < cost:
            continue
        for op, rotation, step in options:
            after = rot.mat_mul(rotation, done)
            key = (after, seen | {up_direction(after)})
            if key not in best or cost + step < best[key] - 1e-9:
                best[key] = cost + step
                heapq.heappush(queue, (cost + step, next(counter), key, ops + [op]))
    raise ValueError('the flips of the cell cannot bring every face up')


def solve_plan(moves, orientation, options, turn_cost=0.0):
    """Operations that make `moves` (such as ["R", "U2", "F'"]) on a cube seated in
    `orientation`, with the fewest seconds of flips. Returns (operations, cost)."""
    routes = _shortest(options)
    if len(routes) != 24:
        raise ValueError('the flips of the cell cannot reach every orientation')
    # states: the orientation the cube is in when a move is made. `layer[o]` is the best
    # (cost, operations) that ends with the last move made in orientation o.
    layer = {orientation: (0.0, [])}
    after_turn = {orientation: orientation}
    for move in moves:
        face, turns = parse_move(move)
        k = ori.gripper_turns(turns)
        nxt, nxt_after = {}, {}
        for target in ori.with_face_down(face):
            for last, (cost, ops) in layer.items():
                start = after_turn[last]
                route_cost, route = routes[rot.mat_mul(target, rot.transpose(start))]
                total = cost + route_cost + turn_cost
                if target not in nxt or total < nxt[target][0] - 1e-9:
                    nxt[target] = (total, ops + route + [('turn', k)])
                    nxt_after[target] = ori.after_turn(target, k)
        layer, after_turn = nxt, nxt_after
    cost, ops = min(layer.values(), key=lambda item: item[0])
    return ops, cost


def apply(state, orientation, operation, kinds):
    """(state, orientation) the logical model expects after one operation."""
    from .cube import apply_move

    if operation[0] == 'turn':
        return (apply_move(state, ori.turn_move(orientation, operation[1])),
                ori.after_turn(orientation, operation[1]))
    return state, ori.after_flip(orientation, kinds[operation[1]], operation[2])
