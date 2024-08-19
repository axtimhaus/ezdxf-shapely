import numpy as np
import pytest
import shapely.geometry as sg
from shapely import affinity
import ezdxf_shapely


@pytest.mark.parametrize(
    "geom",
    [
        sg.Point(1, 1).buffer(1),
        [sg.Point(1, 1).buffer(1), sg.Point(-1, 1).buffer(2)],
    ],
)
def test_centralize(geom):
    result = ezdxf_shapely.centralize(geom)

    for g in result:
        assert np.all(np.isclose(g.centroid.xy, 0, 1e-8))


@pytest.mark.parametrize(
    "geoms",
    [
        [sg.LineString([(0, 0), (1, 1)]), sg.LineString([(1, 1), (2, 2)])],
        [sg.LineString([(0, 0), (1, 1)]), affinity.translate(sg.LineString([(1, 1), (2, 2)]), 1e-9)],
    ],
)
def test_line_merge(geoms):
    result = ezdxf_shapely.line_merge(geoms)

    assert not isinstance(result, sg.MultiLineString)
