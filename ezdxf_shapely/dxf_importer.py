import logging
import math

import numpy as np
from geomdl import NURBS
import ezdxf
import ezdxf.entities as entities

import shapely.geometry as sg
from shapely import ops

from . import geometry
from . import utils

DXF_UNIT_CODES = {0: None, 1: "in", 2: "ft", 3: "mi", 4: "mm", 5: "cm", 6: "m", 7: "km", 10: "yd", 14: "dm"}


class DxfImporter:
    """
    Base abstract class. All cad importers should subclass this class.
    Imports CAD geometry into self.geometry
    """

    def __init__(self, filename: str):
        self.filename = filename
        self.units: str = None  # unitless
        self.geometry: list[sg.base.BaseGeometry] = []
        self.polygons: list[sg.Polygon] = []
        self._already_zipped = False

    def zip(self, zip_length: float = 0.000001):
        """
        Zip tries to reconcile not-quite-matching LineString start and end points.
        Point < zip_length apart will be equated.
        """

        zip = 0
        for i in range(len(self.geometry)):
            ls1 = self.geometry[i]
            fp_1 = sg.Point(ls1.coords[0])  # startpoint
            lp_1 = sg.Point(ls1.coords[-1])  # endpoint

            for j in range(i + 1, len(self.geometry)):
                ls2 = self.geometry[j]
                fp_2 = sg.Point(ls2.coords[0])
                lp_2 = sg.Point(ls2.coords[-1])
                if fp_1.distance(fp_2) < zip_length and fp_1.distance(fp_2) != 0:
                    self.geometry[j] = sg.LineString([ls1.coords[0]] + ls2.coords[1:])
                    zip += 1
                if fp_1.distance(lp_2) < zip_length and fp_1.distance(lp_2) != 0:
                    self.geometry[j] = sg.LineString(ls2.coords[:-1] + [ls1.coords[0]])
                    zip += 1
                if lp_1.distance(fp_2) < zip_length and lp_1.distance(fp_2) != 0:
                    self.geometry[j] = sg.LineString([ls1.coords[-1]] + ls2.coords[1:])
                    zip += 1
                if lp_1.distance(lp_2) < zip_length and lp_1.distance(lp_2) != 0:
                    self.geometry[j] = sg.LineString(ls2.coords[:-1] + [ls1.coords[-1]])
                    zip += 1
        self._already_zipped = True
        logging.info(f"Zipped {zip} points")

    def polygonize(self, simplify=True, force_zip=False, zip_length=0.000001, retry_with_zip=True):
        if not self.geometry:
            raise DxfImporterError("Cannot run polygonize() since no geometry yet. Have you run process()?")

        if not force_zip:
            result, dangles, cuts, invalids = ops.polygonize_full(self.geometry)
            self.polygons = list(result.geoms)

        if force_zip or (not self.polygons and not self._already_zipped and retry_with_zip):
            self.zip(zip_length)

            result, dangles, cuts, invalids = ops.polygonize_full(self.geometry)
            self.polygons = list(result.geoms)

        if self.polygons:
            if simplify:
                for i, p in enumerate(self.polygons):
                    self.polygons[i] = self.polygons[i].simplify(0)
        else:
            logging.error("Unable to from any closed polygons.")

        return result, dangles, cuts, invalids

    def cleanup(self, simplify=True, zip_length=0.000001, retry_with_zip=True) -> str:
        logging.info("cleanup is depreciated BTW. Use the polygonize function instead")
        self.polygonize(simplify, zip_length, retry_with_zip)
        return "done"

    def bounds(self) -> tuple[float]:
        """
        Returns, as (xmin,ymin,xmax,ymax) tuple, the bounding box which envelopes
        this importer's geometry
        """
        for g in self.geometry:
            b = g.bounds
            pass

    def _get_attribs(self, e: entities.DXFEntity):
        """ """
        attribs = geometry._GeometryAttribs()
        attribs["layer"] = e.dxf.layer

        return attribs

    def _process_2d_polyline(self, polyline: entities.Polyline, degrees_per_segment: float = 1):
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

        if polyline.is_closed:
            pl = sg.LinearRing(xy)
        else:
            pl = sg.LineString(xy)
        self.geometry.append(pl)

    def _process_lwpolyline(self, polyline: entities.LWPolyline, degrees_per_segment: float = 1):
        """
        lwpolyline is a lightweight polyline (cf POLYLINE)
        This function equiv to _process_2d_polyline
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

        if polyline.closed:
            pl = sg.LinearRing(xy)
        else:
            pl = sg.LineString(xy)
        self.geometry.append(pl)

    def _process_2d_spline(self, spline: entities.Spline, delta=0.1):
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

        curve.delta = delta  # TODO sampling - this could get out of hand depending on model dims and scale

        # TODO conditional delta: min length, n and check for straight lines

        xyz = np.array(curve.evalpts)

        # discard z data
        xy = list([x[:-1] for x in xyz])

        pl = sg.LineString(xy)
        # geometry.patch_geometry_with_attribs(pl,self._get_attribs(spline))

        self.geometry.append(pl)

    def _process_line(self, line: entities.Line):
        l = sg.LineString([(line.dxf.start.x, line.dxf.start.y), (line.dxf.end.x, line.dxf.end.y)])

        if l.length > 0:
            self.geometry.append(l)

    def _process_arc(self, arc: entities.Arc, degrees_per_segment: float = 1):
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

        arc = sg.LineString(pts)

        self.geometry.append(arc)

    def process(self, spline_delta=0.1, degrees_per_segment: float = 1):
        """
        Args:
            spline_delta (float, optional): Splines are not supported in shapely, so they are approximated as polylines. Defaults to 0.1
            degrees_per_segment (float, optional):
        Returns:
            str: report on geometry processed
        """

        sdoc = ezdxf.readfile(self.filename)

        if "$INSUNITS" in sdoc.header:
            try:
                u = int(sdoc.header["$INSUNITS"])
                if u in DXF_UNIT_CODES:
                    self.units = DXF_UNIT_CODES[u]
            except ValueError:
                logging.error("Casting to int error")

        ents = sdoc.modelspace().query("CIRCLE LINE ARC POLYLINE ELLIPSE SPLINE SHAPE LWPOLYLINE")

        n_splines = n_polylines = n_lines = n_arcs = n_not_implemented = n_lwpolylines = 0
        for e in ents:
            if isinstance(e, entities.Spline) and e.dxf.flags >= ezdxf.lldxf.const.PLANAR_SPLINE:
                self._process_2d_spline(e, delta=spline_delta)
                n_splines += 1
            elif isinstance(e, entities.Polyline):
                if e.get_mode() == "AcDb2dPolyline":
                    self._process_2d_polyline(e, degrees_per_segment=degrees_per_segment)
                    n_polylines += 1
                else:
                    logging.warning(f"Importing of DXF type {type(e)} is not implemented yet.")
                    logging.warning("Raise issue at https://github.com/aegis1980/cad-to-shapely/issues")
                    n_not_implemented += 1
            elif isinstance(e, entities.LWPolyline):
                self._process_lwpolyline(e, degrees_per_segment=degrees_per_segment)
                n_lwpolylines += 1
            elif isinstance(e, entities.Line):
                self._process_line(e)
                n_lines += 1

            elif isinstance(e, entities.Arc):
                self._process_arc(e, degrees_per_segment=degrees_per_segment)
                n_arcs += 1
            else:
                logging.warning(f"Importing of DXF type {type(e)} is not implemented yet.")
                logging.warning("Raise issue at https://github.com/aegis1980/cad-to-shapely/issues")
                n_not_implemented += 1

        return f"Found {n_polylines} polylines, {n_splines} splines, {n_lines} lines, {n_arcs} arcs. Could not process {n_not_implemented} entities."


class DxfImporterError(Exception):
    pass
