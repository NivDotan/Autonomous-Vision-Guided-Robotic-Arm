from __future__ import annotations


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

