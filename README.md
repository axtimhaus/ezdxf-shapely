# EZDXF to Shapely Converter

Convert geometric entities loaded vom DXF files using [`ezdxf`](https://ezdxf.readthedocs.io/) to [`shapely`](https://pypi.org/project/Shapely/) geometry objects.

First use `ezdxf` to load the DXF entities from file and filter them using `query` or similar.

```python
import ezdxf

dxf_doc = ezdxf.readfile("your_file.dxf")
entities = dxf_doc.modelspace().query("*[layer==0]")
```

Then use `ezdxf-shapely` to convert the entities and refine the results.

```python
import ezdxf_shapely

geoms = ezdxf_shapely.convert_all(entities)
polygons = ezdxf_shapely.polygonize(geoms) # optionally merge lines to polygons
```

## License

Licensed under the terms of the [MIT License](LICENSE)
