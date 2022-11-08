# Panoptic Mapping Utils
Utility tools and scripts around **panoptic_mapping**. Playing and creating datasets and similar.

## Mesh Saver

Run the mesh saver to save the same mesh that is visualised in RVIZ as a `.ply` file:

```
<node pkg="panoptic_mapping_utils"
      type="mesh_saver"
      name="mesh_saver"
      output="screen"
      required="true">
  <param name="topic" value="/panoptic_mapper/visualization/submaps/mesh" />
  <param name="output path" value="/tmp/mesh.ply" />
</node>
```
