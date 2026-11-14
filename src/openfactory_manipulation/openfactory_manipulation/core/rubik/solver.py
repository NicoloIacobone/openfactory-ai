"""Solution of a cube state with Kociemba's two-phase algorithm.

The `kociemba` library (GPLv2, pip) is imported only inside `solve`: the rest of the package is
used and tested without it. In the VM it lives in a venv of its own (D-025).
"""

from .cube import SOLVED, apply_moves, check_solvable, parse_moves


def solve(state):
    """Moves that take `state` to the solved cube; an empty list if it is solved already.

    Raises ValueError if the state is not valid or cannot be solved.
    """
    import kociemba

    check_solvable(state)
    if state == SOLVED:
        # kociemba would return a sequence that leaves the cube as it is
        return []
    try:
        moves = parse_moves(kociemba.solve(state))
    except ValueError as exc:
        raise ValueError(f'unsolvable state: {exc}') from exc
    if apply_moves(state, moves) != SOLVED:
        raise ValueError('the solution found does not solve the cube')
    return moves
