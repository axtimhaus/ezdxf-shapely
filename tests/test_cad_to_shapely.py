from pathlib import Path

import ezdxf

from ezdxf_shapely import convert_all, polygonize, utils

TESTS_DIR = Path(__file__).parent


def area_check(filename, area, tol=0.1):
    dxf_filepath = TESTS_DIR / filename
    dxf_doc = ezdxf.readfile(dxf_filepath)
    polygons = polygonize(convert_all(dxf_doc.modelspace().entities_in_redraw_order()))

    section = utils.find_holes(polygons)
    return abs(section.area - area) < tol


def test_complex_holes_section():
    dxf_filepath = TESTS_DIR / "section_holes_complex.dxf"
    dxf_doc = ezdxf.readfile(dxf_filepath)
    polygons = polygonize(convert_all(dxf_doc.modelspace().entities_in_redraw_order()))

    polygon_with_holes = utils.find_holes(polygons)
    assert len(polygon_with_holes.interiors) == 2


def test_simplelines_from_solidworks():
    dxf_filepath = TESTS_DIR / "simplelines_from_solidworks.dxf"
    dxf_doc = ezdxf.readfile(dxf_filepath)
    polygons = polygonize(convert_all(dxf_doc.modelspace().entities_in_redraw_order()))

    assert len(polygons) == 1


def test_dxf_r14_lines_and_arcs():
    dxf_filepath = TESTS_DIR / "200ub22_R12dxf_linesandarcs.dxf"
    dxf_doc = ezdxf.readfile(dxf_filepath)
    polygons = polygonize(convert_all(dxf_doc.modelspace().entities_in_redraw_order()))

    assert len(polygons) == 1


def test_hollow_section_from_steelweb_dot_info():
    assert area_check("200x100x6.dxf", 3373.593)


def test_aluminium_extrusion1():
    area = 884.2  # mm2 calculated in Rhino 3d
    assert area_check("test1.dxf", area)


def test_aluminium_extrusion2():
    area = 1266.0661  # mm2 calculated in Rhino 3d
    assert area_check("test2.dxf", area)


def test_aluminium_extrusion3():
    area = 681.9  # mm2 calculated in Rhino 3d
    assert area_check("test3.dxf", area)


def test_tophat_lwpolyline():
    area = 789.552272
    assert area_check("tophat_autocadlite.dxf", area)
