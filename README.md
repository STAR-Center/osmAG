# osmAG LIB

C++ Library of osmAG.

osmAG (open street map Area Graph) is a hierarchical semantic topological map representation. This is a C++ library based on osmAG, that provides osmAG parsing, saving, visualization, path planning, some ROS plugins and automatic osmAG generation from CAD, 2D grid maps, and 3D point clouds.

## osmAG processing
If you want to read and edit your osm-ag and do path-planning stuff, please use this package.
Now we just use the only one main function in osmAG/data_load_save.cpp

$ cd osmAG

$ mkdir build

$ cmake ..

$ make

$./data_process
