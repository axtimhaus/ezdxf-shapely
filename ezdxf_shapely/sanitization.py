from collections.abc import Iterable

import shapely.geometry as sg
from shapely import ops

DXF_UNIT_CODES = {0: None, 1: "in", 2: "ft", 3: "mi", 4: "mm", 5: "cm", 6: "m", 7: "km", 10: "yd", 14: "dm"}

__all__ = [
    "coerce_line_ends",
    "polygonize",
]


def coerce_line_ends(geoms: Iterable[sg.LineString], zip_length: float = 1e-6) -> list[sg.LineString]:
    """
    Zip tries to reconcile not-quite-matching LineString start and end points.
    Point < zip_length apart will be equated.
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
            if fp_1.distance(fp_2) < zip_length and fp_1.distance(fp_2) != 0:
                geoms[j] = sg.LineString([ls1.coords[0]] + ls2.coords[1:])
            if fp_1.distance(lp_2) < zip_length and fp_1.distance(lp_2) != 0:
                geoms[j] = sg.LineString(ls2.coords[:-1] + [ls1.coords[0]])
            if lp_1.distance(fp_2) < zip_length and lp_1.distance(fp_2) != 0:
                geoms[j] = sg.LineString([ls1.coords[-1]] + ls2.coords[1:])
            if lp_1.distance(lp_2) < zip_length and lp_1.distance(lp_2) != 0:
                geoms[j] = sg.LineString(ls2.coords[:-1] + [ls1.coords[-1]])

    return geoms


def polygonize(
    geoms: Iterable[sg.LineString], simplify=True, force_zip=False, zip_length=0.000001, retry_with_zip=True
) -> list[sg.Polygon]:
    polygons = []

    if not force_zip:
        result, dangles, cuts, invalids = ops.polygonize_full(geoms)
        polygons = list(result.geoms)

    if force_zip or (not polygons and retry_with_zip):
        geoms = coerce_line_ends(geoms, zip_length)
        result, dangles, cuts, invalids = ops.polygonize_full(geoms)
        polygons = list(result.geoms)

    if polygons and simplify:
        for i in range(len(polygons)):
            polygons[i] = polygons[i].simplify(0)

    return polygons
