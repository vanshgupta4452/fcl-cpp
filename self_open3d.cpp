// -------------------- HEADERS ------------------------
#include <fcl/fcl.h>
#include <urdf_parser/urdf_parser.h>
#include <open3d/Open3D.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <map>
#include <set>
#include <sstream>
#include <random>
#include <algorithm>

using namespace fcl;
using namespace std;

using BV = OBBRSSf;
using CollisionShape = std::shared_ptr<BVHModel<BV>>;

// Forward declaration
std::set<std::pair<std::string, std::string>> getAdjacentLinkPairs(const urdf::ModelInterfaceSharedPtr& robot);

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

    for (int i = 0; i <= segments; ++i) {
        float phi = M_PI * i / segments;
        for (int j = 0; j <= segments; ++j) {
            float theta = 2 * M_PI * j / segments;
            float x = radius * sin(phi) * cos(theta);
            float y = radius * sin(phi) * sin(theta);
            float z = radius * cos(phi);
            points.emplace_back(x, y, z);
        }
    }

    for (int i = 0; i < segments; ++i) {
        for (int j = 0; j < segments; ++j) {
            int p1 = i * (segments + 1) + j;
            int p2 = p1 + segments + 1;
            int p3 = p1 + 1;
            int p4 = p2 + 1;

            if (i != 0)
                triangles.emplace_back(p1, p2, p3);
            if (i != segments - 1)
                triangles.emplace_back(p3, p2, p4);
        }
    }

    sphere->beginModel();
    sphere->addSubModel(points, triangles);
    sphere->endModel();
    return sphere;
}

CollisionShape createMeshFromFile(const std::string& mesh_path, const urdf::Vector3& scale = {1.0, 1.0, 1.0}) {
    std::string file_path = mesh_path;
    if (file_path.find("package://") == 0)
        file_path = file_path.substr(10);

    auto mesh = std::make_shared<BVHModel<BV>>();
    std::vector<Vector3f> points;
    std::vector<Triangle> triangles;

    try {
        auto open3d_mesh = open3d::io::CreateMeshFromFile(file_path);
        if (open3d_mesh && !open3d_mesh->vertices_.empty()) {
            for (const auto& vertex : open3d_mesh->vertices_)
                points.emplace_back(vertex.x() * scale.x, vertex.y() * scale.y, vertex.z() * scale.z);

            for (const auto& triangle : open3d_mesh->triangles_)
                triangles.emplace_back(triangle.x(), triangle.y(), triangle.z());
        } else {
            throw std::runtime_error("Open3D load failed");
        }
    } catch (...) {
        float s = 0.1f;
        points = {
            {-s, -s, -s}, {s, -s, -s}, {s, s, -s}, {-s, s, -s},
            {-s, -s, s}, {s, -s, s}, {s, s, s}, {-s, s, s}
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

    std::cerr << "Failed to create mesh, returning nullptr" << std::endl;
    return nullptr;
}

std::set<std::pair<std::string, std::string>> getAdjacentLinkPairs(const urdf::ModelInterfaceSharedPtr& robot) {
    std::set<std::pair<std::string, std::string>> adjacent;
    for (const auto& [name, link] : robot->links_) {
        if (link->parent_joint) {
            std::string parent = link->parent_joint->parent_link_name;
            std::string child = link->name;
            if (parent < child) adjacent.emplace(parent, child);
            else adjacent.emplace(child, parent);
        }
    }
    return adjacent;
}

// -------------------- UTILITY FUNCTIONS ------------------------
std::shared_ptr<open3d::geometry::TriangleMesh> fclToOpen3D(
    const CollisionShape& shape,
    const Transform3f& transform,
    const Eigen::Vector3d& color = Eigen::Vector3d(0.7, 0.7, 0.7)) {

    auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector3i> triangles;

    if (shape && shape->num_vertices > 0) {
        for (int i = 0; i < shape->num_vertices; ++i) {
            Vector3f v = transform * shape->vertices[i];
            vertices.emplace_back(v.x(), v.y(), v.z());
        }

        for (int i = 0; i < shape->num_tris; ++i) {
            Triangle t = shape->tri_indices[i];
            triangles.emplace_back(t[0], t[1], t[2]);
        }
    }

    mesh->vertices_ = vertices;
    mesh->triangles_ = triangles;
    mesh->ComputeVertexNormals();
    mesh->PaintUniformColor(color);

    return mesh;
}

CollisionShape extractShape(const std::shared_ptr<CollisionObjectf>& obj) {
    auto geom = obj->collisionGeometry();
    auto bvh = std::dynamic_pointer_cast<const BVHModel<BV>>(geom);
    return bvh ? std::make_shared<BVHModel<BV>>(*bvh) : nullptr;
}

Eigen::Isometry3f getFullLinkTransform(const urdf::LinkConstSharedPtr& link) {
    if (!link || !link->parent_joint)
        return Eigen::Isometry3f::Identity();

    const urdf::Pose& pose = link->parent_joint->parent_to_joint_origin_transform;
    Eigen::Isometry3f tf = Eigen::Isometry3f::Identity();
    tf.translation() << pose.position.x, pose.position.y, pose.position.z;
    Eigen::Quaternionf q(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
    tf.linear() = q.normalized().toRotationMatrix();

    return getFullLinkTransform(link->getParent()) * tf;
}

// -------------------- URDF PARSER ------------------------
std::map<std::string, std::shared_ptr<CollisionObjectf>> parseURDFToFCL(
    const std::string& urdf_path, 
    urdf::ModelInterfaceSharedPtr& robot) {
    
    std::ifstream urdf_file(urdf_path);
    if (!urdf_file.is_open()) {
        std::cerr << "URDF file not found: " << urdf_path << std::endl;
        return {};
    }

    std::stringstream buffer;
    buffer << urdf_file.rdbuf();
    robot = urdf::parseURDF(buffer.str());
    if (!robot) {
        std::cerr << "Failed to parse URDF" << std::endl;
        return {};
    }

    std::map<std::string, std::shared_ptr<CollisionObjectf>> objects;

    for (const auto& [name, link] : robot->links_) {
        if (!link->collision || !link->collision->geometry)
            continue;

        auto geom = link->collision->geometry;
        CollisionShape shape;

        try {
            if (geom->type == urdf::Geometry::BOX) {
                auto g = std::dynamic_pointer_cast<urdf::Box>(geom);
                shape = createBox(g->dim.x, g->dim.y, g->dim.z);
            } else if (geom->type == urdf::Geometry::CYLINDER) {
                auto g = std::dynamic_pointer_cast<urdf::Cylinder>(geom);
                shape = createCylinder(g->radius, g->length);
            } else if (geom->type == urdf::Geometry::SPHERE) {
                auto g = std::dynamic_pointer_cast<urdf::Sphere>(geom);
                shape = createSphere(g->radius);
            } else if (geom->type == urdf::Geometry::MESH) {
                auto g = std::dynamic_pointer_cast<urdf::Mesh>(geom);
                std::string path = g->filename;
                if (path.find("package://") == 0)
                    path = "/home/vansh/intern-ardee/src/" + path.substr(10);
                urdf::Vector3 scale(g->scale.x, g->scale.y, g->scale.z);
                shape = createMeshFromFile(path, scale);
            } else {
                continue;
            }

            if (!shape) {
                std::cerr << "Failed to create shape for link: " << name << std::endl;
                continue;
            }

            Eigen::Isometry3f tf = getFullLinkTransform(link);
            Eigen::Isometry3f local_tf = Eigen::Isometry3f::Identity();
            auto& origin = link->collision->origin;
            local_tf.translation() << origin.position.x, origin.position.y, origin.position.z;
            Eigen::Quaternionf q(origin.rotation.w, origin.rotation.x, origin.rotation.y, origin.rotation.z);
            local_tf.linear() = q.normalized().toRotationMatrix();

            Transform3f final_tf;
            Eigen::Isometry3f combined_tf = tf * local_tf;
            final_tf.linear() = combined_tf.linear().cast<float>();
            final_tf.translation() = combined_tf.translation().cast<float>();

            auto obj = std::make_shared<CollisionObjectf>(shape, final_tf);
            obj->computeAABB();
            objects[name] = obj;
        } catch (...) {
            std::cerr << "Error loading link: " << name << std::endl;
        }
    }

    return objects;
}

// -------------------- VISUALIZATION ------------------------
void visualizeCollisions(
    const std::map<std::string, std::shared_ptr<CollisionObjectf>>& models,
    const std::set<std::pair<std::string, std::string>>& adjacent_pairs) {
    
    auto vis = std::make_shared<open3d::visualization::Visualizer>();
    vis->CreateVisualizerWindow("Collision Visualization", 1200, 800);

    std::vector<std::pair<std::string, std::string>> collisions;
    for (auto it1 = models.begin(); it1 != models.end(); ++it1) {
        for (auto it2 = std::next(it1); it2 != models.end(); ++it2) {
            std::string a = it1->first;
            std::string b = it2->first;

            // Skip adjacent links
            auto key = (a < b) ? std::make_pair(a, b) : std::make_pair(b, a);
            if (adjacent_pairs.count(key)) continue;
            
            CollisionRequestf req;
            CollisionResultf res;
            collide(it1->second.get(), it2->second.get(), req, res);
            if (res.isCollision())
                collisions.emplace_back(it1->first, it2->first);
        }
    }

    std::cout << "Found " << collisions.size() << " collisions:" << std::endl;
    for (const auto& collision : collisions) {
        std::cout << "  " << collision.first << " <-> " << collision.second << std::endl;
    }

    for (const auto& [name, obj] : models) {
        auto shape = extractShape(obj);
        if (!shape) continue;

        bool collides = std::any_of(collisions.begin(), collisions.end(),
            [&](const auto& pair) { return pair.first == name || pair.second == name; });

        auto color = collides ? Eigen::Vector3d(1.0, 0.0, 0.0) : Eigen::Vector3d(0.0, 1.0, 0.0);
        auto mesh = fclToOpen3D(shape, obj->getTransform(), color);
        vis->AddGeometry(mesh);
    }

    vis->AddGeometry(open3d::geometry::TriangleMesh::CreateCoordinateFrame(0.5));
    vis->Run();
    vis->DestroyVisualizerWindow();
}

// -------------------- MAIN ------------------------
int main() {
    std::string urdf_path = "/home/vansh/intern-ardee/src/fcl-cpp/urdf/PXA-100.urdf";
    
    urdf::ModelInterfaceSharedPtr robot;
    auto models = parseURDFToFCL(urdf_path, robot);
    
    if (models.empty()) {
        std::cerr << "No collision models loaded!" << std::endl;
        return -1;
    }

    auto adjacent_pairs = getAdjacentLinkPairs(robot);
    
    std::cout << "Loaded " << models.size() << " collision models" << std::endl;
    std::cout << "Found " << adjacent_pairs.size() << " adjacent link pairs" << std::endl;
    
    visualizeCollisions(models, adjacent_pairs);
    
    return 0;
}