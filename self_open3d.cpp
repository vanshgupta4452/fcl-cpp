#include <fcl/fcl.h>
#include <urdf_parser/urdf_parser.h>
#include <open3d/Open3D.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <map>
#include <sstream>
#include <random>

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
        {0,1,2}, {0,2,3}, {4,5,6}, {4,6,7},
        {0,1,5}, {0,5,4}, {1,2,6}, {1,6,5},
        {2,3,7}, {2,7,6}, {3,0,4}, {3,4,7}
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

    // Side surface
    for (int i = 0; i < segments; ++i) {
        float theta1 = (2 * M_PI * i) / segments;
        float theta2 = (2 * M_PI * (i + 1)) / segments;
        Vector3f p1(radius * cos(theta1), radius * sin(theta1), -half);
        Vector3f p2(radius * cos(theta2), radius * sin(theta2), -half);
        Vector3f p3(radius * cos(theta2), radius * sin(theta2), half);
        Vector3f p4(radius * cos(theta1), radius * sin(theta1), half);

        int base = points.size();
        points.push_back(p1);
        points.push_back(p2);
        points.push_back(p3);
        points.push_back(p4);

        triangles.emplace_back(base + 0, base + 1, base + 2);
        triangles.emplace_back(base + 0, base + 2, base + 3);
    }

    // Top and bottom caps
    Vector3f topCenter(0, 0, half), bottomCenter(0, 0, -half);
    int topIndex = points.size(); points.push_back(topCenter);
    int bottomIndex = points.size(); points.push_back(bottomCenter);

    for (int i = 0; i < segments; ++i) {
        float theta1 = (2 * M_PI * i) / segments;
        float theta2 = (2 * M_PI * (i + 1)) / segments;
        Vector3f pt1(radius * cos(theta1), radius * sin(theta1), half);
        Vector3f pt2(radius * cos(theta2), radius * sin(theta2), half);
        Vector3f pb1(radius * cos(theta1), radius * sin(theta1), -half);
        Vector3f pb2(radius * cos(theta2), radius * sin(theta2), -half);

        int id = points.size();
        points.push_back(pt1);
        points.push_back(pt2);
        points.push_back(pb1);
        points.push_back(pb2);

        triangles.emplace_back(topIndex, id, id + 1);
        triangles.emplace_back(bottomIndex, id + 2, id + 3);
    }

    cyl->beginModel();
    cyl->addSubModel(points, triangles);
    cyl->endModel();
    return cyl;
}

CollisionShape createSphere(float radius, int segments = 20) {
    auto sphere = std::make_shared<BVHModel<BV>>();
    std::vector<Vector3f> points;
    std::vector<Triangle> triangles;

    // Generate vertices using spherical coordinates
    for (int i = 0; i <= segments; ++i) {
        float phi = M_PI * i / segments;  // Latitude angle
        for (int j = 0; j <= segments; ++j) {
            float theta = 2 * M_PI * j / segments;  // Longitude angle
            
            float x = radius * sin(phi) * cos(theta);
            float y = radius * sin(phi) * sin(theta);
            float z = radius * cos(phi);
            
            points.emplace_back(x, y, z);
        }
    }

    // Generate triangles
    for (int i = 0; i < segments; ++i) {
        for (int j = 0; j < segments; ++j) {
            int p1 = i * (segments + 1) + j;
            int p2 = p1 + segments + 1;
            int p3 = p1 + 1;
            int p4 = p2 + 1;

            if (i != 0) {  // Not the north pole
                triangles.emplace_back(p1, p2, p3);
            }
            if (i != segments - 1) {  // Not the south pole
                triangles.emplace_back(p3, p2, p4);
            }
        }
    }

    sphere->beginModel();
    sphere->addSubModel(points, triangles);
    sphere->endModel();
    return sphere;
}

CollisionShape createMeshFromFile(const std::string& mesh_path, const urdf::Vector3& scale = {1.0, 1.0, 1.0}) {
    std::cout << " Loading mesh file: " << mesh_path << std::endl;
    
    // Clean up the path - remove package:// prefix if present
    std::string file_path = mesh_path;
    if (file_path.find("package://") == 0) {
        file_path = file_path.substr(10); // Remove "package://"
        std::cout << "   🔧 Cleaned path: " << file_path << std::endl;
    }
    
    // Try different mesh loading approaches
    auto mesh = std::make_shared<BVHModel<BV>>();
    std::vector<Vector3f> points;
    std::vector<Triangle> triangles;
    
    try {
        // Try loading with Open3D first
        auto open3d_mesh = open3d::io::CreateMeshFromFile(file_path);
        if (open3d_mesh && !open3d_mesh->vertices_.empty()) {
            std::cout << "   Successfully loaded mesh with Open3D" << std::endl;
            std::cout << "  Vertices: " << open3d_mesh->vertices_.size() 
                      << ", Triangles: " << open3d_mesh->triangles_.size() << std::endl;
            
            // Convert Open3D mesh to FCL format
            for (const auto& vertex : open3d_mesh->vertices_) {
                points.emplace_back(
                    vertex.x() * scale.x,
                    vertex.y() * scale.y,
                    vertex.z() * scale.z
                );
            }
            
            for (const auto& triangle : open3d_mesh->triangles_) {
                triangles.emplace_back(triangle.x(), triangle.y(), triangle.z());
            }
            
            if (scale.x != 1.0 || scale.y != 1.0 || scale.z != 1.0) {
                std::cout << "   📏 Applied scale: " << scale.x << " x " << scale.y << " x " << scale.z << std::endl;
            }
            
        } else {
            throw std::runtime_error("Failed to load with Open3D");
        }
        
    } catch (const std::exception& e) {
        std::cout << " Failed to load mesh file (" << e.what() << "), using default box" << std::endl;
        
        // Create a default box as fallback
        float size = 0.1f;
        points = {
            {-size/2, -size/2, -size/2}, {size/2, -size/2, -size/2}, 
            {size/2, size/2, -size/2}, {-size/2, size/2, -size/2},
            {-size/2, -size/2, size/2}, {size/2, -size/2, size/2}, 
            {size/2, size/2, size/2}, {-size/2, size/2, size/2}
        };
        triangles = {
            {0,1,2}, {0,2,3}, {4,5,6}, {4,6,7},
            {0,1,5}, {0,5,4}, {1,2,6}, {1,6,5},
            {2,3,7}, {2,7,6}, {3,0,4}, {3,4,7}
        };
    }
    
    if (!points.empty() && !triangles.empty()) {
        mesh->beginModel();
        mesh->addSubModel(points, triangles);
        mesh->endModel();
        return mesh;
    }
    
    std::cout << " ailed to create mesh, returning nullptr" << std::endl;
    return nullptr;
}

// -------------------- OPEN3D CONVERSION ------------------------
std::shared_ptr<open3d::geometry::TriangleMesh> fclToOpen3D(
    const CollisionShape& shape, 
    const Transform3f& transform,
    const Eigen::Vector3d& color = Eigen::Vector3d(0.7, 0.7, 0.7)) {
    
    auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    
    // Extract vertices from FCL BVH model
    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector3i> triangles;
    
    if (shape && shape->num_vertices > 0) {
        // Get vertices from FCL model
        for (int i = 0; i < shape->num_vertices; ++i) {
            Vector3f v = shape->vertices[i];
            // Apply transformation
            Vector3f transformed = transform * v;
            vertices.emplace_back(transformed.x(), transformed.y(), transformed.z());
        }
        
        // Get triangles from FCL model
        for (int i = 0; i < shape->num_tris; ++i) {
            Triangle tri = shape->tri_indices[i];
            triangles.emplace_back(tri[0], tri[1], tri[2]);
        }
    }
    
    mesh->vertices_ = vertices;
    mesh->triangles_ = triangles;
    mesh->ComputeVertexNormals();
    mesh->PaintUniformColor(color);
    
    return mesh;
}

// Helper function to safely extract shape from collision object
CollisionShape extractShape(const std::shared_ptr<CollisionObjectf>& collision_obj) {
    auto geom = collision_obj->collisionGeometry();
    auto bvh = std::dynamic_pointer_cast<const BVHModel<BV>>(geom);
    if (bvh) {
        // Create a non-const copy of the BVH model
        auto shape = std::make_shared<BVHModel<BV>>(*bvh);
        return shape;
    }
    return nullptr;
}

// -------------------- URDF PARSER ------------------------
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
        std::cerr << "Failed to parse URDF file!" << std::endl;
        return {};
    }

    std::map<std::string, std::shared_ptr<CollisionObjectf>> link_objects;

    std::cout << "\n🔍 Analyzing URDF links..." << std::endl;
    for (const auto& [name, link] : robot->links_) {
        std::cout << "\n📦 Processing link: " << name << std::endl;
        
        if (!link->collision) {
            std::cout << " No collision geometry defined" << std::endl;
            continue;
        }
        
        if (!link->collision->geometry) {
            std::cout << " No geometry defined in collision" << std::endl;
            continue;
        }

        auto geom = link->collision->geometry;
        CollisionShape shape;

       
        // Handle different geometry types
        if (geom->type == urdf::Geometry::BOX) {
            auto box = std::dynamic_pointer_cast<urdf::Box>(geom);
            std::cout << "Box dimensions: " << box->dim.x << " x " << box->dim.y << " x " << box->dim.z << std::endl;
            shape = createBox(box->dim.x, box->dim.y, box->dim.z);
            
        } else if (geom->type == urdf::Geometry::CYLINDER) {
            auto cyl = std::dynamic_pointer_cast<urdf::Cylinder>(geom);
            std::cout << "Cylinder: radius=" << cyl->radius << ", length=" << cyl->length << std::endl;
            shape = createCylinder(cyl->radius, cyl->length);
            
        } else if (geom->type == urdf::Geometry::SPHERE) {
            auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(geom);
            std::cout << " Sphere radius: " << sphere->radius << std::endl;
            shape = createSphere(sphere->radius);
            
        } else if (geom->type == urdf::Geometry::MESH) {
            auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(geom);
            std::cout << "Mesh file: " << mesh->filename << std::endl;
            if (mesh->scale.x != 1.0 || mesh->scale.y != 1.0 || mesh->scale.z != 1.0) {
                std::cout << " Mesh scale: " << mesh->scale.x << " x " << mesh->scale.y << " x " << mesh->scale.z << std::endl;
            }
            shape = createMeshFromFile(mesh->filename, mesh->scale);
            
        } else {
            std::cout << "Unsupported geometry type!" << std::endl;
            continue;
        }

        if (!shape) {
            std::cout << "  Failed to create collision shape!" << std::endl;
            continue;
        }

        // Get transform information
        auto& origin = link->collision->origin;
        Vector3f trans(origin.position.x, origin.position.y, origin.position.z);
        std::cout << " Position: (" << trans.x() << ", " << trans.y() << ", " << trans.z() << ")" << std::endl;
        std::cout << "Rotation: (" << origin.rotation.x << ", " << origin.rotation.y << ", " << origin.rotation.z << ")" << std::endl;

        Matrix3f rot = (AngleAxisf(origin.rotation.z, Vector3f::UnitZ()) *
                        AngleAxisf(origin.rotation.y, Vector3f::UnitY()) *
                        AngleAxisf(origin.rotation.x, Vector3f::UnitX())).toRotationMatrix();

        Transform3f tf;
        tf.linear() = rot;
        tf.translation() = trans;

        link_objects[name] = std::make_shared<CollisionObjectf>(shape, tf);
        std::cout << "   Successfully created collision object" << std::endl;
    }

    std::cout << "\n Summary: " << link_objects.size() << " collision objects created from " 
              << robot->links_.size() << " total links" << std::endl;

    return link_objects;
}

// -------------------- COLLISION VISUALIZATION ------------------------
void visualizeCollisions(const std::map<std::string, std::shared_ptr<CollisionObjectf>>& models) {
    auto vis = std::make_shared<open3d::visualization::Visualizer>();
    vis->CreateVisualizerWindow("Collision Detection Visualization", 1200, 800);
    
    std::vector<std::pair<std::string, std::string>> colliding_pairs;
    
    std::cout << "\n🔍 Performing collision detection..." << std::endl;
    
    // Check all pairs for collisions
    for (auto it1 = models.begin(); it1 != models.end(); ++it1) {
        for (auto it2 = std::next(it1); it2 != models.end(); ++it2) {
            CollisionRequestf request;
            CollisionResultf result;
            
            collide(it1->second.get(), it2->second.get(), request, result);
            
            if (result.isCollision()) {
                colliding_pairs.emplace_back(it1->first, it2->first);
                std::cout << "Collision detected: " << it1->first << " ↔ " << it2->first << std::endl;
            }
        }
    }
    
    if (colliding_pairs.empty()) {
        std::cout << "No collisions detected!" << std::endl;
    } else {
        std::cout << "Total collisions found: " << colliding_pairs.size() << std::endl;
    }
    
    // Visualize all objects with collision-based coloring
    for (const auto& [link_name, collision_obj] : models) {
        auto shape = extractShape(collision_obj);
        if (!shape) {
            std::cerr << "Failed to extract shape from link: " << link_name << std::endl;
            continue;
        }
        
        auto transform = collision_obj->getTransform();
        
        // Color based on collision status
        Eigen::Vector3d color;
        bool is_colliding = false;
        for (const auto& pair : colliding_pairs) {
            if (pair.first == link_name || pair.second == link_name) {
                is_colliding = true;
                break;
            }
        }
        
        color = is_colliding ? Eigen::Vector3d(1.0, 0.2, 0.2) : Eigen::Vector3d(0.2, 0.8, 0.2);
        
        auto mesh = fclToOpen3D(shape, transform, color);
        if (mesh && !mesh->vertices_.empty()) {
            vis->AddGeometry(mesh);
            std::cout << "Added link: " << link_name << " (" 
                      << (is_colliding ? "COLLISION" : "SAFE") << ")" << std::endl;
        }
    }
    
    // Add coordinate frame
    auto coord_frame = open3d::geometry::TriangleMesh::CreateCoordinateFrame(0.5);
    vis->AddGeometry(coord_frame);
    
    // Set up camera and rendering options
    auto& view_control = vis->GetViewControl();
    view_control.SetZoom(0.5);
    
    auto& render_option = vis->GetRenderOption();
    render_option.show_coordinate_frame_ = true;
    render_option.background_color_ = Eigen::Vector3d(0.05, 0.05, 0.05);
    render_option.mesh_show_wireframe_ = false;
    render_option.mesh_show_back_face_ = true;
    

    
    // Run the visualizer
    vis->Run();
    vis->DestroyVisualizerWindow();
}

// -------------------- MAIN ------------------------
int main() {
    std::string urdf_path = "/home/vansh/intern-ardee/src/fcl-cpp/urdf2/mr_robot.urdf";
    
    std::cout << "Starting URDF Collision Detection and Visualization..." << std::endl;
    std::cout << "Loading URDF file: " << urdf_path << std::endl;
    
    auto models = parseURDFToFCL(urdf_path);
    
    if (models.empty()) {
        std::cerr << " No models loaded from URDF file!" << std::endl;
        return -1;
    }
    
    std::cout << "Successfully loaded " << models.size() << " links from URDF" << std::endl;
    
    for (const auto& [link, model] : models) {
        std::cout << " Parsed link: " << link << std::endl;
    }
    
    // Launch collision detection visualization
    std::cout << "\n Launching collision detection visualization..." << std::endl;
    visualizeCollisions(models);
    
    return 0;
}