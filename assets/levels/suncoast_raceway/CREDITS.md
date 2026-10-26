# Suncoast Raceway: data sources

The level is built from real data by `tools/levels/blender/make_suncoast_raceway.py`; the raw files are in
`tools/levels/data/suncoast_raceway/` (fetched by `tools/levels/fetch_osm_track.py`, sha256 in `sources.lock.json`
there; not packaged with the game). Suncoast Raceway is a fictional name: the layout is traced from OpenStreetMap data
of a real circuit.

## OpenStreetMap

Layout traced from OpenStreetMap data of a real circuit; © OpenStreetMap contributors, ODbL. `osm.osm` is the
OpenStreetMap API 0.6 `map` response for the venue, fetched 2026-10-26 (bounding box and the way ids of the circuit,
drag strip, short course connector, paddock access and venue fence in `sources.lock.json`). The asphalt ways, the
fence and the farmland around them come from it.

© OpenStreetMap contributors. The data is available under the Open Database License (ODbL) 1.0:
https://www.openstreetmap.org/copyright. The level files derived from it (`heightmap.pgm`, `surfaces.pgm`,
`layout.json`) are a Derivative Database under the same licence.

## Copernicus DEM

`dem.json`: heights cropped from the Copernicus GLO-30 DEM tile
`Copernicus_DSM_COG_10_N35_00_E033_00_DEM` (AWS Open Data registry, https://copernicus-dem-30m.s3.amazonaws.com/).

© DLR e.V. 2010-2014 and © Airbus Defence and Space GmbH 2014-2018 provided under COPERNICUS by the European Union
and ESA; all rights reserved. Used under the Copernicus DEM licence (free use with this attribution).

## Not from data

Track width (12 m; published 12 to 15 m), driving direction, curbs, painted lines, run-offs, barriers, the pit lane,
all buildings and the vegetation are placed by the generator; none of them is mapped in OpenStreetMap.
