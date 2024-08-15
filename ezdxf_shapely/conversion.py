import math
from collections.abc import Iterable

import ezdxf
import numpy as np
import shapely.geometry as sg
from ezdxf import entities
from geomdl import NURBS

from ezdxf_shapely import utils

__all__ = [
    "convert_2d_polyline",
    "convert_2d_spline",
    "convert_all",
    "convert_all_generator",
    "convert_arc",
    "convert_line",
    "convert_lwpolyline",
]


def convert_2d_polyline(polyline: entities.Polyline, degrees_per_segment: float = 1) -> sg.LineString:
    xy = []

    for i, v1 in enumerate(polyline.vertices):
        xy.append([v1.dxf.location.x, v1.dxf.location.y])
        if v1.dxf.bulge and v1.dxf.bulge != 0:
            if i + 1 == len(polyline.vertices):
                if polyline.is_closed:
                    v2 = polyline.vertices[0]
                else:
                    break
            else:
                v2 = polyline.vertices[i + 1]

            p1 = [v1.dxf.location.x, v1.dxf.location.y]
            p2 = [v2.dxf.location.x, v2.dxf.location.y]

            pts = utils.arc_points_from_bulge(p1, p2, v1.dxf.bulge, degrees_per_segment)
            pts = pts[1:-1]

            xy.extend(pts)

    return sg.LinearRing(xy) if polyline.is_closed else sg.LineString(xy)


def convert_lwpolyline(polyline: entities.LWPolyline, degrees_per_segment: float = 1) -> sg.LineString:
    """
    lwpolyline is a lightweight polyline (cf POLYLINE)
    This function equiv to _convert_2d_polyline
    """

    xy = []

    points = polyline.get_points()

    for i, v1 in enumerate(points):
        xy.append([v1[0], v1[1]])

        if (
            len(v1) == 4 and v1[4] != 0
        ):  # lwpolygon.points() returns tuple x,y,s,e,b. s and e are start and end width (irrelevant)
            if i + 1 == len(points):
                if polyline.closed:
                    v2 = points[0]
                else:
                    break
            else:
                v2 = points[i + 1]

            p1 = [v1[0], v1[1]]
            p2 = [v2[0], v2[1]]

            pts = utils.arc_points_from_bulge(p1, p2, v1[4], degrees_per_segment)
            pts = pts[1:-1]

            xy.extend(pts)

    return sg.LinearRing(xy) if polyline.is_closed else sg.LineString(xy)


def convert_2d_spline(spline: entities.Spline, delta=0.1) -> sg.LineString:
    """
    Uses geomdl module to create intermediate b-spline from dxf spline.
    This is then sampled as a linestring since shapely does not support splines.
    """

    curve = NURBS.Curve()
    curve.degree = spline.dxf.degree
    curve.ctrlpts = spline.control_points

    curve.weights = [1] * spline.control_point_count()  # spline.weights
    # curve.weights = spline.weights + [1] * np.array(spline.control_point_count()- len(spline.weights))
    curve.knotvector = spline.knots

    curve.delta = delta  # TODO: sampling - this could get out of hand depending on model dims and scale

    # TODO: conditional delta: min length, n and check for straight lines

    xyz = np.array(curve.evalpts)

    # discard z data
    xy = [x[:-1] for x in xyz]

    return sg.LineString(xy)


def convert_line(line: entities.Line) -> sg.LineString:
    return sg.LineString([(line.dxf.start.x, line.dxf.start.y), (line.dxf.end.x, line.dxf.end.y)])


def convert_arc(arc: entities.Arc, degrees_per_segment: float = 1) -> sg.LineString:
    """
    shapely does not do arcs, so we make it into an n-lined polyline.
    modified from here: https://stackoverflow.com/questions/30762329/how-to-create-polygons-with-arcs-in-shapely-or-a-better-library
    """
    start_angle = math.radians(arc.dxf.start_angle)
    end_angle = math.radians(arc.dxf.end_angle)
    if start_angle > end_angle:
        end_angle += 2 * math.pi

    pts = utils.arc_points(
        start_angle, end_angle, arc.dxf.radius, [arc.dxf.center.x, arc.dxf.center.y], degrees_per_segment
    )

    return sg.LineString(pts)


def convert_all_generator(
    dxf_entities: Iterable[entities.DXFGraphic], spline_delta=0.1, degrees_per_segment: float = 1
) -> Iterable[sg.LineString]:
    for e in dxf_entities:
        match e:
            case entities.Spline() as s if e.dxf.flags >= ezdxf.lldxf.const.PLANAR_SPLINE:
                yield convert_2d_spline(s, delta=spline_delta)
            case entities.Polyline() as pl if pl.get_mode() == "AcDb2dPolyline":
                yield convert_2d_polyline(pl, degrees_per_segment=degrees_per_segment)
            case entities.LWPolyline() as pl:
                yield convert_lwpolyline(pl, degrees_per_segment=degrees_per_segment)
            case entities.Line() as l:
                yield convert_line(l)
            case entities.Arc() as a:
                yield convert_arc(a, degrees_per_segment=degrees_per_segment)
            case _:
                msg = f"Conversion of entity type {type(e)} not supported."
                raise TypeError(msg)


def convert_all(
    dxf_entities: Iterable[entities.DXFGraphic], spline_delta=0.1, degrees_per_segment: float = 1
) -> list[sg.LineString]:
    return list(convert_all_generator(dxf_entities, spline_delta, degrees_per_segment))
