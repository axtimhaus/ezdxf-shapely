from collections.abc import Iterable

import shapely.geometry as sg
from shapely import ops

DXF_UNIT_CODES = {0: None, 1: "in", 2: "ft", 3: "mi", 4: "mm", 5: "cm", 6: "m", 7: "km", 10: "yd", 14: "dm"}

__all__ = [
    "coerce_line_ends",
    "polygonize",
]


def coerce_line_ends(geoms: Iterable[sg.LineString], distance: float = 1e-8) -> list[sg.LineString]:
    """
    Coerce nearby line ends to the exact same point.

    :param geoms: iterable of line strings to operate on
    :param distance: maximum distance to move line ends during coercion

    :returns: the line strings with coerced ends (fresh instances)
    """

    geoms = list(geoms)

    for i in range(len(geoms)):
        ls1 = geoms[i]
        fp_1 = sg.Point(ls1.coords[0])  # startpoint
        lp_1 = sg.Point(ls1.coords[-1])  # endpoint

        for j in range(i + 1, len(geoms)):
            ls2 = geoms[j]
            fp_2 = sg.Point(ls2.coords[0])
            lp_2 = sg.Point(ls2.coords[-1])
            if fp_1.distance(fp_2) < distance and fp_1.distance(fp_2) != 0:
                geoms[j] = sg.LineString([ls1.coords[0]] + ls2.coords[1:])
            if fp_1.distance(lp_2) < distance and fp_1.distance(lp_2) != 0:
                geoms[j] = sg.LineString(ls2.coords[:-1] + [ls1.coords[0]])
            if lp_1.distance(fp_2) < distance and lp_1.distance(fp_2) != 0:
                geoms[j] = sg.LineString([ls1.coords[-1]] + ls2.coords[1:])
            if lp_1.distance(lp_2) < distance and lp_1.distance(lp_2) != 0:
                geoms[j] = sg.LineString(ls2.coords[:-1] + [ls1.coords[-1]])

    return geoms


def polygonize(
    geoms: Iterable[sg.LineString], coerce_ends=True, coercion_distance=1e-8, simplify=True
) -> list[sg.Polygon]:
    """
    Create polygons from the given line strings.
    Optionally, coerce the line ends before polygonization and simplify the result after.

    :param geoms: iterable of line strings to use for polygonization
    :param coerce_ends: whether to coerce the line ends before polygonization
    :param coercion_distance: maximum distance to move line ends during coercion
    :param simplify: whether to simplify the resulting polygons

    :returns: a list of created polygons
    """
    polygons = []

    if coerce_ends:
        geoms = coerce_line_ends(geoms, coercion_distance)

    polygons = list(ops.polygonize(geoms))

    if polygons and simplify:
        polygons = [p.simplify(0) for p in polygons]

    return polygons
