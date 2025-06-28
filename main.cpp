#include <fcl/fcl.h>
#include <urdf_parser/urdf_parser.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <map>
#include <sstream>

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
    std::map<std::string, std::shared_ptr<CollisionObjectf>> link_objects;

    for (const auto& [name, link] : robot->links_) {
        if (!link->collision || !link->collision->geometry) continue;

        auto geom = link->collision->geometry;
        CollisionShape shape;

        if (geom->type == urdf::Geometry::BOX) {
            auto box = std::dynamic_pointer_cast<urdf::Box>(geom);
            shape = createBox(box->dim.x, box->dim.y, box->dim.z);
        } else if (geom->type == urdf::Geometry::CYLINDER) {
            auto cyl = std::dynamic_pointer_cast<urdf::Cylinder>(geom);
            shape = createCylinder(cyl->radius, cyl->length);
        } else {
            std::cerr << "Unsupported geometry in link: " << name << std::endl;
            continue;
        }

        auto& origin = link->collision->origin;
        Vector3f trans(origin.position.x, origin.position.y, origin.position.z);

        Matrix3f rot = (AngleAxisf(origin.rotation.z, Vector3f::UnitZ()) *
                        AngleAxisf(origin.rotation.y, Vector3f::UnitY()) *
                        AngleAxisf(origin.rotation.x, Vector3f::UnitX())).toRotationMatrix();

        Transform3f tf;
        tf.linear() = rot;
        tf.translation() = trans;

        link_objects[name] = std::make_shared<CollisionObjectf>(shape, tf);
    }

    return link_objects;
}

// -------------------- MAIN ------------------------
int main() {
    std::string urdf_path = "/home/vansh/intern-ardee/src/fcl-cpp/urdf2/mr_robot.urdf";
    auto models = parseURDFToFCL(urdf_path);

    for (const auto& [link, model] : models) {
        std::cout << "Parsed link: " << link << ", object id: " << model->getNodeType() << std::endl;
    }

    // Collision check example (between first 2 links)
    if (models.size() >= 2) {
        auto it = models.begin();
        auto obj1 = *it++;
        auto obj2 = *it;

        CollisionRequestf request;
        CollisionResultf result;

        collide(obj1.second.get(), obj2.second.get(), request, result);
        if (result.isCollision())
            std::cout << " Collision Detected between [" << obj1.first << "] and [" << obj2.first << "]\n";
        else
            std::cout << " No Collision between [" << obj1.first << "] and [" << obj2.first << "]\n";
    }

    return 0;
}