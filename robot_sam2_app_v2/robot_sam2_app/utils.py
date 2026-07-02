from __future__ import annotations

import re


# Separators that introduce the place target, longest/most-specific first.
_PLACE_SPLITS = (
    " and place it ", " then place it ", " and place ", " then place ",
    " place it ", " and put it ", " then put it ", " and put ", " put it ",
    " and drop it ", " drop it ", " and drop ",
)
_PICK_VERB = re.compile(r"^\s*(pick up|pick|grab|take|get)\s+", re.IGNORECASE)
_PLACE_PREP = re.compile(r"^\s*(place|put|drop|in|on|into|onto|at|inside)\s+", re.IGNORECASE)


def _strip_article(s: str) -> str:
    return s.strip().rstrip(".")


def parse_pick_place_command(text: str) -> tuple[str, str | None]:
    """Parse a natural-language command into (pick_query, place_query|None).

    "pick the red cup and place it in the box" -> ("the red cup", "the box")
    "pick up the marker and put it on the plate" -> ("the marker", "the plate")
    "pick the cup" -> ("the cup", None)
    """
    raw = (text or "").strip()
    low = raw.lower()

    split_at = None
    sep_len = 0
    for sep in _PLACE_SPLITS:
        idx = low.find(sep)
        if idx != -1 and (split_at is None or idx < split_at):
            split_at, sep_len = idx, len(sep)

    if split_at is None:
        pick = _PICK_VERB.sub("", raw, count=1)
        return _strip_article(pick), None

    pick_part = raw[:split_at]
    place_part = raw[split_at + sep_len:]
    # Strip a leading place preposition the separator may not have eaten (e.g. "in").
    place_part = _PLACE_PREP.sub("", place_part, count=1)
    pick = _PICK_VERB.sub("", pick_part, count=1)
    return _strip_article(pick), _strip_article(place_part) or None


def clamp(value: float, lower: float, upper: float):
    return lower if value < lower else upper if value > upper else value


def step_toward(current: int, target: int, limit: int) -> int:
    diff = target - current
    step = max(-limit, min(limit, diff))
    return int(current + step)


def normalize_class_name(name) -> str:
    return " ".join(str(name).lower().replace("_", " ").replace("-", " ").split())


def count_fingers(hand_landmarks) -> int:
    lm = hand_landmarks.landmark
    count = 0
    if lm[8].y < lm[5].y:
        count += 1
    if lm[12].y < lm[9].y:
        count += 1
    if lm[16].y < lm[13].y:
        count += 1
    if lm[20].y < lm[17].y:
        count += 1
    return count

