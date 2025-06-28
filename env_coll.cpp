#include <fcl/fcl.h>
#include <urdf_parser/urdf_parser.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

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

    for (int i = 0; i < segments; ++i) {
        float theta = (2 * M_PI * i) / segments;
        float x = radius * cos(theta);
        float y = radius * sin(theta);
        points.emplace_back(x, y, -half);
        points.emplace_back(x, y, half);
    }

    Vector3f bottomCenter(0, 0, -half);
    Vector3f topCenter(0, 0, half);
    int bottomCenterIdx = points.size();
    int topCenterIdx = points.size() + 1;
    points.push_back(bottomCenter);
    points.push_back(topCenter);

    for (int i = 0; i < segments; ++i) {
        int next = (i + 1) % segments;
        int bottom1 = i * 2;
        int top1 = i * 2 + 1;
        int bottom2 = next * 2;
        int top2 = next * 2 + 1;

        triangles.emplace_back(bottom1, bottom2, top1);
        triangles.emplace_back(top1, bottom2, top2);
    }

    for (int i = 0; i < segments; ++i) {
        int next = (i + 1) % segments;
        int bottom1 = i * 2;
        int bottom2 = next * 2;
        triangles.emplace_back(bottomCenterIdx, bottom2, bottom1);
    }

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
    const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);
    
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

        for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
            aiVector3D v = mesh->mVertices[i];
            vertices.emplace_back(scale * v.x, scale * v.y, scale * v.z);
        }

        for (unsigned int i = 0; i < mesh->mNumFaces; ++i) {
            const aiFace& face = mesh->mFaces[i];
            if (face.mNumIndices != 3) continue;
            triangles.emplace_back(
                face.mIndices[0] + vertex_offset,
                face.mIndices[1] + vertex_offset,
                face.mIndices[2] + vertex_offset
            );
        }

        vertex_offset += mesh->mNumVertices;
    }

    model->beginModel();
    model->addSubModel(vertices, triangles);
    model->endModel();
    return model;
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
            continue;
        }

        auto geom = link->collision->geometry;
        CollisionShape shape;

        if (geom->type == urdf::Geometry::BOX) {
            auto box = std::dynamic_pointer_cast<urdf::Box>(geom);
            shape = createBox(box->dim.x, box->dim.y, box->dim.z);
        } else if (geom->type == urdf::Geometry::CYLINDER) {
            auto cyl = std::dynamic_pointer_cast<urdf::Cylinder>(geom);
            shape = createCylinder(cyl->radius, cyl->length);
        } else if (geom->type == urdf::Geometry::SPHERE) {
            auto sph = std::dynamic_pointer_cast<urdf::Sphere>(geom);
            shape = createSphere(sph->radius);
        } else if (geom->type == urdf::Geometry::MESH) {
            auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(geom);
            shape = loadMeshAsBVH(mesh->filename, mesh->scale.x); // Add scaling support
        } else {
            continue;
        }

        auto& origin = link->collision->origin;
        Vector3f trans(origin.position.x, origin.position.y, origin.position.z);
        Quaternionf quat(origin.rotation.w, origin.rotation.x, origin.rotation.y, origin.rotation.z);
        Matrix3f rot = quat.toRotationMatrix();

        Transform3f tf;
        tf.linear() = rot;
        tf.translation() = trans;

        auto collision_obj = std::make_shared<CollisionObjectf>(shape, tf);
        collision_obj->computeAABB();
        link_objects[name] = collision_obj;
    }

    return link_objects;
}

// -------------------- Main ------------------------
int main() {
    std::string urdf_path = "/home/vansh/intern-ardee/src/fcl-cpp/urdf2/mr_robot.urdf";
    auto models = parseURDFToFCL(urdf_path);
std::cerr << "";
    if (models.empty()) {
        std::cerr << "No models found\n";
        return 1;
    }

    std::cout << "Loaded " << models.size() << " collision links from URDF\n";

    auto sphere_geom = createSphere(0.2f);
    auto moving_sphere = std::make_shared<CollisionObjectf>(sphere_geom);

    float z = -2.0f;

    while (true) {
        z += 0.05f;
        if (z > 2.0f) z = -2.0f;

        Transform3f tf;
        tf.translation() = Vector3f(0, 0, z);
        tf.linear() = Matrix3f::Identity();
        moving_sphere->setTransform(tf);
        moving_sphere->computeAABB();

        for (const auto& [name, obj] : models) {
            CollisionRequestf req;
            CollisionResultf res;
            if (collide(moving_sphere.get(), obj.get(), req, res) && res.isCollision()) {
                std::cout << "💥 COLLISION with: " << name << "\n";
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    return 0;
}