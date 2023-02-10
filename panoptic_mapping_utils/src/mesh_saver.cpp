#include "panoptic_mapping_utils/mesh_saver.h"

namespace panoptic_mapping {

MeshSaver::MeshSaver(const ros::NodeHandle& nh) : nh_(nh) { setupRos(); }
void MeshSaver::setupRos() {
  mesh_sub_ =
      nh_.subscribe(nh_.param<std::string>(
                        "topic", "/panoptic_mapper/visualization/submaps/mesh"),
                    10, &MeshSaver::gotMeshCallback, this);
}

void MeshSaver::gotMeshCallback(const voxblox_msgs::MultiMesh& msg) {
  voxblox::Mesh full_mesh;
  bool first = true;

  for (const voxblox_msgs::MeshBlock& mesh_block : msg.mesh.mesh_blocks) {
    const voxblox::BlockIndex index(mesh_block.index[0], mesh_block.index[1],
                                    mesh_block.index[2]);

    if (mesh_block.x.size() == 0) {
      continue;
    }

    size_t vertex_index = 0u;
    voxblox::Mesh mesh;
    mesh.vertices.reserve(mesh_block.x.size());
    mesh.indices.reserve(mesh_block.x.size());

    // translate vertex data from message to voxblox mesh
    for (size_t i = 0; i < mesh_block.x.size(); ++i) {
      // Each vertex is given as its distance from the blocks origin in units of
      // (2*block_size), see mesh_vis.h for the slightly convoluted
      // justification of the 2.
      constexpr float point_conv_factor =
          2.0f / std::numeric_limits<uint16_t>::max();
      const float mesh_x =
          (static_cast<float>(mesh_block.x[i]) * point_conv_factor +
           static_cast<float>(index[0])) *
          msg.mesh.block_edge_length;
      const float mesh_y =
          (static_cast<float>(mesh_block.y[i]) * point_conv_factor +
           static_cast<float>(index[1])) *
          msg.mesh.block_edge_length;
      const float mesh_z =
          (static_cast<float>(mesh_block.z[i]) * point_conv_factor +
           static_cast<float>(index[2])) *
          msg.mesh.block_edge_length;

      mesh.indices.push_back(vertex_index++);
      mesh.vertices.emplace_back(mesh_x, mesh_y, mesh_z);
    }

    // calculate normals
    mesh.normals.reserve(mesh.vertices.size());
    for (size_t i = 0; i < mesh.vertices.size(); i += 3) {
      const voxblox::Point dir0 = mesh.vertices[i] - mesh.vertices[i + 1];
      const voxblox::Point dir1 = mesh.vertices[i] - mesh.vertices[i + 2];
      const voxblox::Point normal = dir0.cross(dir1).normalized();

      mesh.normals.push_back(normal);
      mesh.normals.push_back(normal);
      mesh.normals.push_back(normal);
    }

    // add color information
    mesh.colors.reserve(mesh.vertices.size());
    const bool has_color = mesh_block.x.size() == mesh_block.r.size();
    for (size_t i = 0; i < mesh_block.x.size(); ++i) {
      voxblox::Color color;
      if (has_color) {
        color.r = mesh_block.r[i];
        color.g = mesh_block.g[i];
        color.b = mesh_block.b[i];

      } else {
        // reconstruct normals coloring
        color.r = std::numeric_limits<uint8_t>::max() *
                  (mesh.normals[i].x() * 0.5f + 0.5f);
        color.g = std::numeric_limits<uint8_t>::max() *
                  (mesh.normals[i].y() * 0.5f + 0.5f);
        color.b = std::numeric_limits<uint8_t>::max() *
                  (mesh.normals[i].z() * 0.5f + 0.5f);
      }
      color.a = 1.0;
      mesh.colors.push_back(color);
    }

    // connect mesh

    if (first) {
      voxblox::createConnectedMesh(mesh, &full_mesh);
      first = false;
    } else {
      voxblox::Mesh connected_mesh;
      voxblox::createConnectedMesh(mesh, &connected_mesh);
      full_mesh.concatenateMesh(connected_mesh);
    }
  }
  // save mesh
  std::string output_path =
      nh_.param<std::string>("output_path", "/tmp/map_mesh.ply");
  voxblox::outputMeshAsPly(output_path, full_mesh);
  std::cout << "mesh saved to " << output_path << std::endl;
}
}  // namespace panoptic_mapping
