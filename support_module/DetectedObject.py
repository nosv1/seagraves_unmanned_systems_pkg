from __future__ import annotations

from math import degrees, pi
from typing import Tuple

from .LidarPoint import LidarPoint

class DetectedObject:
    def __init__(self, significant_points: list[LidarPoint]) -> None:
        self.significant_points: list[LidarPoint] = significant_points

    @property
    def width(self) -> float:
        return self.significant_points[0].point.distance_to(
            self.significant_points[-1].point)

    @property
    def distance(self) -> float:
        return min([sp.distance for sp in self.significant_points])

def detect_object(
    points: list[LidarPoint], 
    starting_index: int, 
    distnace_threshold: float, 
    angle_threshold: float,
    max_skip_points: int) -> Tuple[DetectedObject, int]:
    """ returns detected object and next starting index """

    def is_significant_difference(point_1: LidarPoint, point_2: LidarPoint):
        distance_diff: float = point_1.distance - point_2.distance
        angle_diff: float = min(
            abs(point_1.angle - point_2.angle),
            abs(point_1.angle - (point_2.angle + 2*pi)))
        significant_distance: bool = distance_diff > distnace_threshold
        significant_angle: bool = degrees(angle_diff) > angle_threshold
        # return significant_distance or significant_angle
        return significant_distance

    detected_object = DetectedObject(significant_points=[])
    significant_index: int = None
    if starting_index != 0:
        detected_object.significant_points.append(points[starting_index])

    if starting_index == 0:
        # look right
        for i in range(starting_index - 1, -len(points), -1):
            if is_significant_difference(points[i], points[i+1]):
                detected_object.significant_points.append(points[i+1])
                break

    # look left
    for i in range(starting_index + 1, len(points)):
        if is_significant_difference(points[i], points[i-1]):
            detected_object.significant_points.append(points[i-1])
            significant_index = i
            break

        elif i - starting_index > max_skip_points:
            detected_object.significant_points.append(points[i])
            significant_index = i + 1 if i < len(points) - 1 else 0
            break

    return detected_object, significant_index

def detect_objects(
    points: list[LidarPoint], 
    *,
    distance_threshold: float = 0.3, 
    angle_threshold: float = 2.0,
    max_skip_points: int = 10) -> list[DetectedObject]:
    """ angle threshold is in degrees """
    detected_objects: list[DetectedObject] = []

    if len(points) <= 1:
        detected_objects.append(DetectedObject(significant_points=points))

    starting_indexes = {0}
    queue = [0]
    while queue:
        detected_object, significant_index = detect_object(
            points, queue[0], 
            distance_threshold, angle_threshold, max_skip_points)
        queue.pop(0)
        
        if significant_index and significant_index not in starting_indexes:
            queue.append(significant_index)
            starting_indexes.add(significant_index)
            detected_objects.append(detected_object)
            

    return detected_objects