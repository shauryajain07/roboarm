"""Cell suggestions for the operator. board = {cell: None | ("us"|"them", block)}."""
from __future__ import annotations

from poses import BLOCK_POINTS, CELLS, LINES

CENTRE, CORNERS = "r2c2", ("r1c1", "r1c3", "r3c1", "r3c3")


def _owner(board, c):
    v = board.get(c)
    return v[0] if v else None


def _completing(board, who):
    """Empty cells that would give `who` three in a line."""
    out = []
    for line in LINES:
        owners = [_owner(board, c) for c in line]
        if owners.count(who) == 2 and owners.count(None) == 1:
            out.append(line[owners.index(None)])
    return out


def suggest_place(board, placeable=CELLS):
    """Return (cell, reason) or (None, reason)."""
    empty = [c for c in CELLS if board.get(c) is None and c in placeable]
    if not empty:
        return None, "no empty reachable cell"
    for c in _completing(board, "us"):
        if c in empty:
            return c, "GREAT VICTORY - completes our line"
    for c in _completing(board, "them"):
        if c in empty:
            return c, "BLOCK their line"
    # cell that sits on the most lines still open for us
    def score(c):
        s = 0
        for line in LINES:
            if c in line and all(_owner(board, x) in (None, "us") for x in line):
                s += 1 + sum(_owner(board, x) == "us" for x in line)
        return s + (0.5 if c == CENTRE else 0.2 if c in CORNERS else 0)
    best = max(empty, key=score)
    return best, "builds the most open lines"


def suggest_attack(board):
    """Opponent block worth knocking out: breaks their near-win first, then most value."""
    theirs = [c for c in CELLS if _owner(board, c) == "them"]
    if not theirs:
        return None, "no opponent blocks"
    for line in LINES:
        owners = [_owner(board, c) for c in line]
        if owners.count("them") == 2 and owners.count(None) == 1:
            # remove the one that is on the most of their lines
            cands = [c for c in line if _owner(board, c) == "them"]
            return max(cands, key=lambda c: sum(c in l for l in LINES)), "breaks their 2-in-a-line"
    best =max(theirs, key=lambda c: BLOCK_POINTS.get(board[c][1], 0))
    return best, f"highest value ({board[best][1]})"


def score(board, who):
    return sum(BLOCK_POINTS.get(v[1], 0) for v in board.values() if v and v[0] == who)


def winner_line(board, who):
    for line in LINES:
        if all(_owner(board, c) == who for c in line):
            return line
    return None
