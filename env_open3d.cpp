#include <fcl/fcl.h>
#include <urdf_parser/urdf_parser.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <open3d/Open3D.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <map>
#include <sstream>
#include <cmath>
#include <thread>
#include <chrono>

using namespace fcl;
using namespace std;

using BV = OBBRSSf;
using CollisionShape = std::shared_ptr<BVHModel<BV>>;

// -------------------- SHAPE GENERATORS ------------------------
CollisionShape createBox(float x, float y, float z) {
    auto box = std::make_shared<BVHModel<BV>>();
    std::vector<Vector3f> points = {
        {-x/2, -y/2, -z/2}, {x/2, -y/2, -z/2}, {x/2, y/2, -z/2}, {-x/2, y/2, -z/2},
        {-x/2, -y/2, z/2}, {x/2, -y/2, z/2}, {x/2, y/2, z/2}, {-x/2, y/2, z/2}
    };
    std::vector<Triangle> triangles = {
        {0,1,2}, {0,2,3}, {4,7,6}, {4,6,5},
        {0,4,5}, {0,5,1}, {1,5,6}, {1,6,2},
        {2,6,7}, {2,7,3}, {3,7,4}, {3,4,0}
    };
    box->beginModel();
    box->addSubModel(points, triangles);
    box->endModel();
    return box;
}

CollisionShape createCylinder(float radius, float length, int segments = 20) {
    auto cyl = std::make_shared<BVHModel<BV>>();
    std::vector<Vector3f> points;
    std::vector<Triangle> triangles;
    float half = length / 2.0f;

    // Create vertices for top and bottom circles
    for (int i = 0; i < segments; ++i) {
        float theta = (2 * M_PI * i) / segments;
        float x = radius * cos(theta);
        float y = radius * sin(theta);
        points.emplace_back(x, y, -half);  // bottom circle
        points.emplace_back(x, y, half);   // top circle
    }

    // Add center points for caps
    Vector3f bottomCenter(0, 0, -half);
    Vector3f topCenter(0, 0, half);
    int bottomCenterIdx = points.size();
    int topCenterIdx = points.size() + 1;
    points.push_back(bottomCenter);
    points.push_back(topCenter);

    // Create side triangles
    for (int i = 0; i < segments; ++i) {
        int next = (i + 1) % segments;
        int bottom1 = i * 2;
        int top1 = i * 2 + 1;
        int bottom2 = next * 2;
        int top2 = next * 2 + 1;

        // Two triangles per side face
        triangles.emplace_back(bottom1, bottom2, top1);
        triangles.emplace_back(top1, bottom2, top2);
    }

    // Create bottom cap triangles
    for (int i = 0; i < segments; ++i) {
        int next = (i + 1) % segments;
        int bottom1 = i * 2;
        int bottom2 = next * 2;
        triangles.emplace_back(bottomCenterIdx, bottom2, bottom1);
    }

    // Create top cap triangles
    for (int i = 0; i < segments; ++i) {
        int next = (i + 1) % segments;
        int top1 = i * 2 + 1;
        int top2 = next * 2 + 1;
        triangles.emplace_back(topCenterIdx, top1, top2);
    }

    cyl->beginModel();
    cyl->addSubModel(points, triangles);
    cyl->endModel();
    return cyl;
}

CollisionShape createSphere(float radius, int rings = 10, int sectors = 20) {
    auto sphere = std::make_shared<BVHModel<BV>>();
    std::vector<Vector3f> points;
    std::vector<Triangle> triangles;

    // Generate sphere vertices
    for (int r = 0; r <= rings; ++r) {
        float theta = M_PI * r / rings;
        float z = radius * cos(theta);
        float xy = radius * sin(theta);

        for (int s = 0; s <= sectors; ++s) {
            float phi = 2 * M_PI * s / sectors;
            float x = xy * cos(phi);
            float y = xy * sin(phi);
            points.emplace_back(x, y, z);
        }
    }

    // Generate sphere triangles
    for (int r = 0; r < rings; ++r) {
        for (int s = 0; s < sectors; ++s) {
            int cur = r * (sectors + 1) + s;
            int next = (r + 1) * (sectors + 1) + s;

            triangles.emplace_back(cur, next, cur + 1);
            triangles.emplace_back(next, next + 1, cur + 1);
        }
    }

    sphere->beginModel();
    sphere->addSubModel(points, triangles);
    sphere->endModel();
    return sphere;
}

// ------------------ Mesh Loader -----------------------
CollisionShape loadMeshAsBVH(const std::string& path, float scale = 1.0f) {
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(path, 
        aiProcess_Triangulate | 
        aiProcess_JoinIdenticalVertices |
        aiProcess_GenNormals);
    
    if (!scene || !scene->HasMeshes()) {
        std::cerr << "Assimp failed to load mesh: " << path << std::endl;
        return nullptr;
    }

    auto model = std::make_shared<BVHModel<BV>>();
    std::vector<Vector3f> vertices;
    std::vector<Triangle> triangles;
    int vertex_offset = 0;

    for (unsigned int m = 0; m < scene->mNumMeshes; ++m) {
        const aiMesh* mesh = scene->mMeshes[m];

        // Add vertices
        for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
            aiVector3D v = mesh->mVertices[i];
            vertices.emplace_back(scale * v.x, scale * v.y, scale * v.z);
        }

        // Add triangles
        for (unsigned int i = 0; i < mesh->mNumFaces; ++i) {
            const aiFace& face = mesh->mFaces[i];
            if (face.mNumIndices != 3) continue; // Skip non-triangular faces
            triangles.emplace_back(
                face.mIndices[0] + vertex_offset,
                face.mIndices[1] + vertex_offset,
                face.mIndices[2] + vertex_offset
            );
        }

        vertex_offset += mesh->mNumVertices;
    }

    if (vertices.empty() || triangles.empty()) {
        std::cerr << "No valid mesh data found in: " << path << std::endl;
        return nullptr;
    }

    model->beginModel();
    model->addSubModel(vertices, triangles);
    model->endModel();
    return model;
}

Eigen::Isometry3f getFullLinkTransform(const urdf::LinkConstSharedPtr& link) {
    if (!link) return Eigen::Isometry3f::Identity();

    if (!link->parent_joint) return Eigen::Isometry3f::Identity();

    const urdf::Pose& pose = link->parent_joint->parent_to_joint_origin_transform;

    Eigen::Isometry3f tf = Eigen::Isometry3f::Identity();
    tf.translation() << pose.position.x, pose.position.y, pose.position.z;

    Eigen::Quaternionf q(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
    tf.linear() = q.normalized().toRotationMatrix();

    // Recursively accumulate transform
    return getFullLinkTransform(link->getParent()) * tf;
}

// -------------------- URDF Parser ------------------------
std::map<std::string, std::shared_ptr<CollisionObjectf>> parseURDFToFCL(const std::string& urdf_path) {
    std::ifstream urdf_file(urdf_path);
    if (!urdf_file.is_open()) {
        std::cerr << "URDF file not found: " << urdf_path << std::endl;
        return {};
    }

    std::stringstream buffer;
    buffer << urdf_file.rdbuf();
    std::string urdf_string = buffer.str();

    auto robot = urdf::parseURDF(urdf_string);
    if (!robot) {
        std::cerr << "Failed to parse URDF" << std::endl;
        return {};
    }

    std::map<std::string, std::shared_ptr<CollisionObjectf>> link_objects;

    for (const auto& [name, link] : robot->links_) {
        if (!link->collision || !link->collision->geometry) {
            std::cout << "Skipping link " << name << " (no collision geometry)" << std::endl;
            continue;
        }

        auto geom = link->collision->geometry;
        CollisionShape shape;

        try {
            if (geom->type == urdf::Geometry::BOX) {
                auto box = std::dynamic_pointer_cast<urdf::Box>(geom);
                shape = createBox(box->dim.x, box->dim.y, box->dim.z);
                std::cout << "Created box for link: " << name << std::endl;
            } 
            else if (geom->type == urdf::Geometry::CYLINDER) {
                auto cyl = std::dynamic_pointer_cast<urdf::Cylinder>(geom);
                shape = createCylinder(cyl->radius, cyl->length);
                std::cout << "Created cylinder for link: " << name << std::endl;
            } 
            else if (geom->type == urdf::Geometry::SPHERE) {
                auto sph = std::dynamic_pointer_cast<urdf::Sphere>(geom);
                shape = createSphere(sph->radius);
                std::cout << "Created sphere for link: " << name << std::endl;
            } 
            else if (geom->type == urdf::Geometry::MESH) {
                auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(geom);
                std::string mesh_path = mesh->filename;
                if (mesh_path.find("package://") == 0) {
                    std::string pkg_root = "/home/vansh/intern-ardee/src/";
                    mesh_path = pkg_root + mesh_path.substr(10);  // remove 'package://'
                }
                float scale_factor = (mesh->scale.x + mesh->scale.y + mesh->scale.z) / 3.0f;
                shape = loadMeshAsBVH(mesh_path, scale_factor);
                std::cout << "Created mesh for link: " << name << std::endl;
            } 
            else {
                std::cout << "Unsupported geometry type for link: " << name << std::endl;
                continue;
            }

            // Get full TF from root -> this link
            Eigen::Isometry3f full_tf = getFullLinkTransform(link);

            // Get local collision origin offset
            auto& local_origin = link->collision->origin;
            Eigen::Isometry3f local_tf = Eigen::Isometry3f::Identity();
            local_tf.translation() << local_origin.position.x, local_origin.position.y, local_origin.position.z;
            Eigen::Quaternionf q(local_origin.rotation.w, local_origin.rotation.x, local_origin.rotation.y, local_origin.rotation.z);
            local_tf.linear() = q.normalized().toRotationMatrix();

            // Combine both
            Eigen::Isometry3f final_tf = full_tf * local_tf;

            Transform3f tf;
            tf.linear() = final_tf.linear().cast<float>();
            tf.translation() = final_tf.translation().cast<float>();

            auto collision_obj = std::make_shared<CollisionObjectf>(shape, tf);
            collision_obj->computeAABB();
            link_objects[name] = collision_obj;

            std::cout << "[TF] Link: " << name 
                      << " | Translation: " << tf.translation().transpose() << std::endl;

        } catch (const std::exception& e) {
            std::cerr << "Error processing link " << name << ": " << e.what() << std::endl;
            continue;
        }
    }

    return link_objects;
}

// ------------------ Convert to Open3D ---------------------
std::shared_ptr<open3d::geometry::TriangleMesh> FCLToOpen3DMesh(
    const CollisionShape& shape,
    const Transform3f& tf,
    const Eigen::Vector3d& color
) {
    if (!shape) {
        std::cerr << "Null shape provided to FCLToOpen3DMesh" << std::endl;
        return nullptr;
    }

    // Cast to BVHModel
    auto bvh = std::dynamic_pointer_cast<const BVHModel<BV>>(shape);
    if (!bvh) {
        std::cerr << "Failed to cast to BVHModel" << std::endl;
        return nullptr;
    }

    auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();

    std::cout << "Converting mesh: " << bvh->num_vertices << " vertices, " << bvh->num_tris << " triangles" << std::endl;

    // Convert vertices
    mesh->vertices_.reserve(bvh->num_vertices);
    for (int i = 0; i < bvh->num_vertices; ++i) {
        const Vector3f& v = bvh->vertices[i];
        Vector3f v_tf = tf * v;  // Apply transformation
        mesh->vertices_.emplace_back(static_cast<double>(v_tf[0]), 
                                   static_cast<double>(v_tf[1]), 
                                   static_cast<double>(v_tf[2]));
    }

    // Convert triangles
    mesh->triangles_.reserve(bvh->num_tris);
    for (int i = 0; i < bvh->num_tris; ++i) {
        const Triangle& tri = bvh->tri_indices[i];
        // Validate triangle indices
        if (tri[0] < bvh->num_vertices && tri[1] < bvh->num_vertices && tri[2] < bvh->num_vertices) {
            mesh->triangles_.emplace_back(Eigen::Vector3i(tri[0], tri[1], tri[2]));
        } else {
            std::cerr << "Invalid triangle indices: " << tri[0] << ", " << tri[1] << ", " << tri[2] << std::endl;
        }
    }

    if (mesh->vertices_.empty() || mesh->triangles_.empty()) {
        std::cerr << "Empty mesh generated - vertices: " << mesh->vertices_.size() 
                  << ", triangles: " << mesh->triangles_.size() << std::endl;
        return nullptr;
    }

    std::cout << "Created Open3D mesh: " << mesh->vertices_.size() << " vertices, " 
              << mesh->triangles_.size() << " triangles" << std::endl;

    // Paint the mesh with the specified color
    mesh->PaintUniformColor(color);
    
    // Compute normals for proper lighting
    mesh->ComputeVertexNormals();
    
    // Ensure the mesh is valid
    if (!mesh->HasVertices() || !mesh->HasTriangles()) {
        std::cerr << "Generated mesh is invalid" << std::endl;
        return nullptr;
    }

    return mesh;
}

// Helper function to print transform 
void printTransform(const std::string& name, const Transform3f& tf) {
    std::cout << name << " Transform:" << std::endl;
    std::cout << "  Translation: " << tf.translation().transpose() << std::endl;
    std::cout << "  Rotation:\n" << tf.linear() << std::endl;
}

// -------------------- Main ------------------------
int main() {
    std::string urdf_path = "/home/vansh/intern-ardee/src/fcl-cpp/urdf/PXA-100.urdf";
    
    std::cout << "Parsing URDF file: " << urdf_path << std::endl;
    auto models = parseURDFToFCL(urdf_path);
    
    if (models.empty()) {
        std::cerr << "No models found in URDF file" << std::endl;
        return 1;
    }

    std::cout << "Found " << models.size() << " collision objects" << std::endl;

    // Create moving sphere - make it larger for easier collision detection
    auto sphere_geom = createSphere(0.1f);  // Increased size from 0.2f
    auto moving_sphere = std::make_shared<CollisionObjectf>(sphere_geom);

    // Initialize Open3D visualizer
    open3d::visualization::Visualizer vis;
    if (!vis.CreateVisualizerWindow("FCL + Open3D Viewer", 1280, 720)) {
        std::cerr << "Failed to create visualizer window" << std::endl;
        return 1;
    }

    // Set up camera and rendering options
    auto& view_control = vis.GetViewControl();
    auto& render_option = vis.GetRenderOption();
    render_option.mesh_show_wireframe_ = false;
    render_option.mesh_show_back_face_ = true;
    render_option.light_on_ = true;
    
    // Set initial camera position
    view_control.SetZoom(0.3);
    view_control.SetLookat(Eigen::Vector3d(0, 0, 0));
    view_control.SetUp(Eigen::Vector3d(0, 0, 1));
    view_control.SetFront(Eigen::Vector3d(1, 1, 1));

    float z = -2.0f;
    bool running = true;
    bool first_frame = true;
    std::shared_ptr<open3d::geometry::TriangleMesh> moving_sphere_mesh = nullptr;
    
    // Create mapping between models and their meshes for easier access
    std::map<std::string, std::shared_ptr<open3d::geometry::TriangleMesh>> mesh_map;
    
    // First, add all static geometries once
    for (const auto& [name, obj] : models) {
        auto obj_shape = std::dynamic_pointer_cast<BVHModel<BV>>(
            std::const_pointer_cast<CollisionGeometryf>(obj->collisionGeometry())
        );
        
        if (obj_shape) {
            auto mesh = FCLToOpen3DMesh(obj_shape, obj->getTransform(), Eigen::Vector3d(0.7, 0.7, 0.7));
            if (mesh) {
                std::cout << "Adding static mesh for: " << name << std::endl;
                mesh_map[name] = mesh;
                vis.AddGeometry(mesh, false);
                
                // Print some debug info about the mesh bounds
                auto bbox = mesh->GetAxisAlignedBoundingBox();
                std::cout << "  Mesh " << name << " bounds: min=" << bbox.GetMinBound().transpose() 
                          << " max=" << bbox.GetMaxBound().transpose() << std::endl;
            }
        }
    }

    // Create coordinate frame for reference
    auto coord_frame = open3d::geometry::TriangleMesh::CreateCoordinateFrame(0.1);
    vis.AddGeometry(coord_frame, false);

    std::cout << "Added " << mesh_map.size() << " static meshes to visualizer" << std::endl;

    // Animation parameters
    float speed = 0.03f;  // Slower movement for better observation
    int collision_count = 0;

    while (running) {
        // Update sphere position
        z += speed;
        if (z > 2.0f) z = -2.0f;

        // Create transform for moving sphere
        Transform3f tf;
        tf.translation() = Vector3f(0, 0, z);
        tf.linear() = Matrix3f::Identity();
        moving_sphere->setTransform(tf);
        moving_sphere->computeAABB();

        // Debug: Print sphere position
        if (static_cast<int>(z * 100) % 50 == 0) {  // Print every 0.5 units
            std::cout << "Sphere position: " << tf.translation().transpose() << std::endl;
        }

        // Reset collision counter
        collision_count = 0;

        // Check collisions and update colors
        for (const auto& [name, obj] : models) {
            bool collides = false;
            try {
                CollisionRequestf req;
                CollisionResultf res;
                
                // Enable contact computation for more detailed results
                req.enable_contact = true;
                req.num_max_contacts = 1;
                
                int collision_result = collide(moving_sphere.get(), obj.get(), req, res);
                
                if (collision_result > 0 && res.isCollision()) {
                    std::cout << "COLLISION DETECTED with: " << name << std::endl;
                    std::cout << "  Number of contacts: " << res.numContacts() << std::endl;
                    collides = true;
                    collision_count++;
                    
                    // Print collision details
                    if (res.numContacts() > 0) {
                        auto contact = res.getContact(0);
                        std::cout << "  Contact point: " << contact.pos.transpose() << std::endl;
                        std::cout << "  Penetration depth: " << contact.penetration_depth << std::endl;
                    }
                }
            }
            catch (const std::exception& e) {
                std::cerr << "Error during collision check with " << name << ": " << e.what() << std::endl;
            }

            // Update mesh color if it exists in the map
            if (mesh_map.find(name) != mesh_map.end()) {
                auto color = collides ? Eigen::Vector3d(1, 0, 0) : Eigen::Vector3d(0.7, 0.7, 0.7);
                mesh_map[name]->PaintUniformColor(color);
                vis.UpdateGeometry(mesh_map[name]);  // Important: Update the geometry in visualizer
            }
        }

        // Create/update moving sphere mesh
        if (!moving_sphere_mesh) {
            moving_sphere_mesh = FCLToOpen3DMesh(sphere_geom, tf, Eigen::Vector3d(0, 0.6, 1));
            if (moving_sphere_mesh) {
                vis.AddGeometry(moving_sphere_mesh, false);
                std::cout << "Added moving sphere mesh" << std::endl;
            }
        } else {
            // Update sphere position by recreating vertices
            moving_sphere_mesh->vertices_.clear();
            auto bvh = std::dynamic_pointer_cast<const BVHModel<BV>>(sphere_geom);
            if (bvh) {
                for (int i = 0; i < bvh->num_vertices; ++i) {
                    const Vector3f& v = bvh->vertices[i];
                    Vector3f v_tf = tf * v;
                    moving_sphere_mesh->vertices_.emplace_back(static_cast<double>(v_tf[0]),
                                                            static_cast<double>(v_tf[1]),
                                                            static_cast<double>(v_tf[2]));
                }
                moving_sphere_mesh->ComputeVertexNormals();
                vis.UpdateGeometry(moving_sphere_mesh);
            }
        }

        // Print collision summary
        if (collision_count > 0) {
            std::cout << "Total collisions this frame: " << collision_count << std::endl;
        }

        if (first_frame) {
            // Auto-fit the view to show all geometry
            // view_control.FitInGeometry(coord_frame->GetAxisAlignedBoundingBox());
            auto full_bbox = coord_frame->GetAxisAlignedBoundingBox();
            for (const auto& [_, mesh] : mesh_map) {
                full_bbox += mesh->GetAxisAlignedBoundingBox();
            }
            if (moving_sphere_mesh) {
                full_bbox += moving_sphere_mesh->GetAxisAlignedBoundingBox();
            }
            view_control.FitInGeometry(full_bbox);
            first_frame = false;
        }

        // Update visualization
        if (!vis.PollEvents()) {
            running = false;
        }
        vis.UpdateRender();

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    vis.DestroyVisualizerWindow();
    return 0;
}