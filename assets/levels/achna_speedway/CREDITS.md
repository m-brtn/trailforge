# Achna Speedway: data sources

The level is built from real data by `tools/levels/blender/make_achna_speedway.py`; the raw files are in `source/`
(fetched by `tools/levels/fetch_osm_track.py`, sha256 in `source/sources.lock.json`).

## OpenStreetMap

`source/osm.osm`: OpenStreetMap API 0.6 `map` response for the venue (bounding box 33.7925, 35.0205, 33.8100,
35.0315), fetched 2026-10-26. The circuit, drag strip, short course connector, paddock access, venue fence and the
farmland around it come from it (ways 734690065, 734690066, 734691195, 734690068 and the `landuse=farmland` ways).

© OpenStreetMap contributors. The data is available under the Open Database License (ODbL) 1.0:
https://www.openstreetmap.org/copyright. The level files derived from it (`heightmap.pgm`, `surfaces.pgm`,
`layout.json`) are a Derivative Database under the same licence.

## Copernicus DEM

`source/dem.json`: heights cropped from the Copernicus GLO-30 DEM tile
`Copernicus_DSM_COG_10_N35_00_E033_00_DEM` (AWS Open Data registry, https://copernicus-dem-30m.s3.amazonaws.com/).

© DLR e.V. 2010-2014 and © Airbus Defence and Space GmbH 2014-2018 provided under COPERNICUS by the European Union
and ESA; all rights reserved. Used under the Copernicus DEM licence (free use with this attribution).

## Not from data

Track width (12 m; published 12 to 15 m), driving direction, curbs, painted lines, run-offs, barriers, the pit lane,
all buildings and the vegetation are placed by the generator; none of them is mapped in OpenStreetMap.
