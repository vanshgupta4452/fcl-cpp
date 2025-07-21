#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <memory>
#include <thread>
#include <iomanip>

// KDL includes
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

// Track-IK include
#include <trac_ik/trac_ik.hpp>

using namespace KDL;

class NativeTrackIKNode {
private:
    // Core components
    std::string urdf_string_;
    std::unique_ptr<TRAC_IK::TRAC_IK> tracik_solver_;
    std::unique_ptr<ChainFkSolverPos_recursive> fk_solver_;
    
    // Robot model
    KDL::Chain kdl_chain_;
    std::vector<std::string> joint_names_;
    std::vector<std::pair<double, double>> joint_limits_;
    
    // State variables
    JntArray current_joint_positions_;
    JntArray target_joint_positions_;
    JntArray joint_velocities_;
    Vector current_target_position_;
    
    // Control parameters
    bool first_solution_received_;
    bool target_reached_;
    double position_tolerance_;
    double joint_velocity_limit_;
    double convergence_threshold_;
    
    // Test targets
    std::vector<Vector> test_targets_;
    size_t current_target_index_;
    
    // Performance tracking
    int successful_solutions_;
    int failed_solutions_;
    double average_solve_time_;
    
    urdf::Model robot_model_;
    
    // Timing
    std::chrono::steady_clock::time_point last_update_;
    std::chrono::steady_clock::time_point last_target_switch_;

    std::vector<KDL::JntArray> path;

    
public:
    NativeTrackIKNode() : 
        first_solution_received_(false),
        target_reached_(true),
        position_tolerance_(0.01),    
        joint_velocity_limit_(1.0),   
        convergence_threshold_(0.001),
        current_target_index_(0),
        successful_solutions_(0),
        failed_solutions_(0),
        average_solve_time_(0.0) {

        // Initialize robot model
        if (!loadRobotModel()) {
            std::cerr << "Failed to load robot model" << std::endl;
            return;
        }
        robot_model_.initString(urdf_string_);

        // Initialize solvers
        if (!initializeSolvers()) {
            std::cerr << "Failed to initialize solvers" << std::endl;
            return;
        }

        // Initialize test targets
        initializeTestTargets();
        
        // Test basic functionality
        testSolverCapabilities();

        // Initialize timing
        last_update_ = std::chrono::steady_clock::now();
        last_target_switch_ = std::chrono::steady_clock::now();

        std::cout << "Native Track-IK Node initialized successfully (Position-only mode)" << std::endl;
    }

    void run() {
        std::cout << "Starting main loop. Press Ctrl+C to exit." << std::endl;
        
        while (true) {
            update();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

private:
    bool loadRobotModel() {
        // Load URDF file - UPDATE THIS PATH TO YOUR URDF FILE
        std::string urdf_path = "/home/vansh/intern-ardee/src/fcl-ros2/ajgar_description/urdf/arm.urdf";
        
        std::ifstream urdf_file(urdf_path);
        if (!urdf_file.is_open()) {
            std::cerr << "Failed to open URDF file: " << urdf_path << std::endl;
            return false;
        }
        
        std::stringstream buffer;
        buffer << urdf_file.rdbuf();
        urdf_string_ = buffer.str();
        urdf_file.close();
        
        // Parse URDF
        if (!robot_model_.initString(urdf_string_)) {
            std::cerr << "Failed to parse URDF" << std::endl;
            return false;
        }
        
        // Build KDL tree
        KDL::Tree kdl_tree;
        if (!kdl_parser::treeFromUrdfModel(robot_model_, kdl_tree)) {
            std::cerr << "Failed to convert URDF to KDL Tree" << std::endl;
            return false;
        }
        
        // Extract chain
        std::string base_link = "base_link";
        std::string tip_link = "end__1";
        
        if (!kdl_tree.getChain(base_link, tip_link, kdl_chain_)) {
            std::cerr << "Failed to extract chain from " << base_link << " to " << tip_link << std::endl;
            return false;
        }
        
        // Extract joint names
        joint_names_.clear();
        for (unsigned int i = 0; i < kdl_chain_.getNrOfSegments(); ++i) {
            KDL::Segment segment = kdl_chain_.getSegment(i);
            if (segment.getJoint().getType() != KDL::Joint::None) {
                joint_names_.push_back(segment.getJoint().getName());
            }
        }
        
        // Fallback joint names if extraction fails
        if (joint_names_.empty()) {
            joint_names_ = {
                "base_joint",
                "shoulder_joint",
                "arm_joint",
                "forearm_joint",
                "end_joint",
                "suction_joint"
            };
        }
        
        std::cout << "Successfully loaded robot model with " << kdl_chain_.getNrOfJoints() << " joints" << std::endl;
        
        return true;
    }
    
    bool initializeSolvers() {
        // Initialize Forward Kinematics solver
        fk_solver_ = std::make_unique<ChainFkSolverPos_recursive>(kdl_chain_);
        
        // Initialize Track-IK solver
        std::string base_link = "base_link";
        std::string tip_link = "end__1";
        
        double timeout = 0.1;
        double eps = 5e-3;
        
        std::cout << "Initializing Track-IK with timeout=" << timeout << ", eps=" << eps << std::endl;
        
        tracik_solver_ = std::make_unique<TRAC_IK::TRAC_IK>(
            base_link, tip_link, urdf_string_, timeout, eps, TRAC_IK::Speed);
        
        // Verify Track-IK initialization
        KDL::Chain trac_ik_chain;
        if (!tracik_solver_->getKDLChain(trac_ik_chain)) {
            std::cerr << "Failed to get KDL chain from Track-IK" << std::endl;
            return false;
        }

        KDL::Chain trac_chain;
        tracik_solver_->getKDLChain(trac_chain);
        std::cout << "TRAC-IK Joint Order:" << std::endl;
        for (unsigned int i = 0; i < trac_chain.getNrOfSegments(); ++i) {
            if (trac_chain.getSegment(i).getJoint().getType() != KDL::Joint::None)
                std::cout << "  " << trac_chain.getSegment(i).getJoint().getName() << std::endl;
        }
        
        // Initialize joint arrays
        unsigned int nj = kdl_chain_.getNrOfJoints();
        current_joint_positions_ = JntArray(nj);
        target_joint_positions_ = JntArray(nj);
        joint_velocities_ = JntArray(nj);
        
        // Extract joint limits
        joint_limits_.clear();
        for (const auto& name : joint_names_) {
            auto joint = robot_model_.getJoint(name);
            if (!joint) {
                std::cout << "WARNING: Joint '" << name << "' not found in URDF!" << std::endl;
            }
            else if (joint->type != urdf::Joint::CONTINUOUS && joint->limits) {
                if (joint->limits->upper == joint->limits->lower) {
                    std::cout << "WARNING: Joint '" << name << "' has identical upper and lower limits!" << std::endl;
                }
                joint_limits_.emplace_back(joint->limits->lower, joint->limits->upper);
            } else {
                joint_limits_.emplace_back(-M_PI, M_PI);
            }
        }
        
        if (joint_limits_.size() != kdl_chain_.getNrOfJoints()) {
            std::cerr << "Joint limits size (" << joint_limits_.size() << ") doesn't match number of joints in KDL chain (" << kdl_chain_.getNrOfJoints() << ")" << std::endl;
        }

        // Initialize joints to mid-range
        for (unsigned int i = 0; i < nj; ++i) {
            if (i < joint_limits_.size()) {
                current_joint_positions_(i) = (joint_limits_[i].first + joint_limits_[i].second) / 2.0;
            } else {
                current_joint_positions_(i) = 0.0;
            }
            joint_velocities_(i) = 0.0;
        }
        target_joint_positions_ = current_joint_positions_;
        
        // Set initial target position
        Frame initial_frame;
        if (fk_solver_->JntToCart(current_joint_positions_, initial_frame) >= 0) {
            current_target_position_ = initial_frame.p;
            std::cout << "Initial EE position: [" << std::fixed << std::setprecision(3) 
                     << current_target_position_.x() << ", " << current_target_position_.y() 
                     << ", " << current_target_position_.z() << "]" << std::endl;
        } else {
            std::cerr << "Failed to compute initial forward kinematics" << std::endl;
            return false;
        }
        
        std::cout << "All solvers initialized successfully" << std::endl;
        return true;
    }
    
    void initializeTestTargets() {
        Frame current_frame;
        if (fk_solver_->JntToCart(current_joint_positions_, current_frame) >= 0) {
            Vector base_pos = current_frame.p;
            
            std::cout << "Current EE position: [" << std::fixed << std::setprecision(3) 
                     << base_pos.x() << ", " << base_pos.y() << ", " << base_pos.z() << "]" << std::endl;
            
            test_targets_ = {
                Vector(0.0, 0.0, 0.5),
                Vector(0.0, -0.2, 0.5),
                Vector(-0.2, 0.2, 0.5),
            };
            
            // Test all targets and keep only reachable ones
            std::vector<Vector> reachable_targets;
            for (size_t i = 0; i < test_targets_.size(); ++i) {
                const auto& target = test_targets_[i];
                std::cout << "Testing target " << i << ": [" << std::fixed << std::setprecision(3) 
                         << target.x() << ", " << target.y() << ", " << target.z() << "]" << std::endl;
                
                if (isTargetReachable(target)) {
                    JntArray test_result(kdl_chain_.getNrOfJoints());
                    if (solveIKPositionOnly(target, current_joint_positions_, test_result)) {
                        reachable_targets.push_back(target);
                        std::cout << "✓ Target " << i << " is reachable" << std::endl;
                    } else {
                        std::cout << "✗ Target " << i << " failed IK test" << std::endl;
                    }
                } else {
                    std::cout << "✗ Target " << i << " failed reachability test" << std::endl;
                }
            }
            
            test_targets_ = reachable_targets;
            std::cout << "Initialized " << test_targets_.size() << " reachable test targets" << std::endl;
        } else {
            std::cerr << "Failed to get current EE position for test target initialization" << std::endl;
        }
    }
    
    bool isTargetReachable(const Vector& target) {
        double distance = target.Norm();
        std::cout << "Target distance from base: " << std::fixed << std::setprecision(3) << distance << std::endl;

        if (distance < 0.01) {
            std::cout << "Target too close to origin: " << distance << " m" << std::endl;
            return false;
        }
        
        if (std::abs(target.z()) < 0.001) {
            std::cout << "Target too close to base plane: z=" << target.z() << std::endl;
            return false;
        }
        
        return true;
    }
    
    void testSolverCapabilities() {
        std::cout << "Testing solver capabilities..." << std::endl;
        
        Frame current_frame;
        if (fk_solver_->JntToCart(current_joint_positions_, current_frame) >= 0) {
            JntArray test_result(kdl_chain_.getNrOfJoints());
            
            auto start_time = std::chrono::high_resolution_clock::now();
            bool success = solveIKPositionOnly(current_frame.p, current_joint_positions_, test_result);
            auto end_time = std::chrono::high_resolution_clock::now();
            
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            
            if (success) {
                std::cout << "✓ Self-test passed (" << std::fixed << std::setprecision(3) 
                         << duration.count() / 1000.0 << " ms)" << std::endl;
            } else {
                std::cerr << "✗ Self-test failed" << std::endl;
            }
        }
    }


     void interpolateJointsWithFK(const KDL::JntArray& q_start, const KDL::JntArray& q_goal, int steps,
                             std::vector<KDL::JntArray>& path,
                             KDL::ChainFkSolverPos_recursive& fk_solver) {
            if (q_start.rows() != q_goal.rows()) {
                std::cerr << "q_start and q_goal must have same dimensions" << std::endl;
                return;
            }

            path.clear();
            for (int i = 0; i <= steps; ++i) {
                double alpha = static_cast<double>(i) / steps;
                KDL::JntArray q_interp(q_start.rows());
                for (unsigned int j = 0; j < q_start.rows(); ++j) {
                    q_interp(j) = (1 - alpha) * q_start(j) + alpha * q_goal(j);
                }

                path.push_back(q_interp);

                // Print joint values
                std::cout << "Step " << i << " - Joints: ";
                for (unsigned int j = 0; j < q_interp.rows(); ++j) {
                    std::cout << q_interp(j) << " ";
                }

                // Compute FK for this step
                KDL::Frame ee_pose;
                if (fk_solver.JntToCart(q_interp, ee_pose) >= 0) {
                    double x = ee_pose.p.x();
                    double y = ee_pose.p.y();
                    double z = ee_pose.p.z();
                    double roll, pitch, yaw;
                    ee_pose.M.GetRPY(roll, pitch, yaw);
                    
                    std::cout << " | End-effector XYZ: (" << x << ", " << y << ", " << z << ")";
                    std::cout << " | RPY: (" << roll << ", " << pitch << ", " << yaw << ")";
                } else {
                    std::cout << " | FK failed!";
                }

                std::cout << std::endl;
            }
        }


    
    // Predict end-effector position after joint interpolation
    Vector predictEndEffectorPosition(const JntArray& interpolated_joints) {
        Frame predicted_frame;
        if (fk_solver_->JntToCart(interpolated_joints, predicted_frame) >= 0) {
            return predicted_frame.p;
        }
        std::cout<<"ikfailed predict endfactor position"<<std::endl;
        return Vector(0,0,0); // Return zero vector if FK fails
    }
    
    // Check if waypoint is reachable and safe
    bool isWaypointValid(const Vector& waypoint) {
        // Check distance constraints
        double distance = waypoint.Norm();
        if (distance < 0.01 || distance > 1.0) {
            return false;
        }
        
        // Check if IK solution exists
        JntArray temp_solution(kdl_chain_.getNrOfJoints());
        return solveIKPositionOnly(waypoint, current_joint_positions_, temp_solution);
    }
    

    
    bool solveIKPositionOnly(const Vector& target_pos, const JntArray& q_init, JntArray& q_result) {
            double distance_from_origin = target_pos.Norm();
            std::cout << "Target position: [" << std::fixed << std::setprecision(3) 
                    << target_pos.x() << ", " << target_pos.y() << ", " << target_pos.z() 
                    << "], distance: " << distance_from_origin << std::endl;

            if (distance_from_origin < 0.01) {
                std::cerr << "Target too close to origin: " << distance_from_origin << " m" << std::endl;
                return false;
            }

            if (distance_from_origin > 1.0) {
                std::cerr << "Target too far from origin: " << distance_from_origin << " m" << std::endl;
                return false;
            }

            Frame target_frame;
            target_frame.p = target_pos;

            q_result = JntArray(kdl_chain_.getNrOfJoints());

            auto start_time = std::chrono::high_resolution_clock::now();
            int result = tracik_solver_->CartToJnt(q_init, target_frame, q_result);
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

            if (result >= 0) {
                successful_solutions_++;
                average_solve_time_ = (average_solve_time_ * (successful_solutions_ - 1) +
                                    duration.count() / 1000.0) / successful_solutions_;

                // ✅ Now interpolate from q_init to q_result
                int steps = 5;
                interpolateJointsWithFK(q_init, q_result, steps, path, *fk_solver_);
                // Print interpolated steps
                

                Frame verify_frame;
                if (fk_solver_->JntToCart(q_result, verify_frame) >= 0) {
                    Vector pos_error = target_pos - verify_frame.p;
                    double error_norm = pos_error.Norm();

                    std::cout << "Solution found! Position error: " << std::fixed << std::setprecision(6) 
                            << error_norm << " m" << std::endl;

                    if (error_norm > position_tolerance_) {
                        std::cout << "WARNING: Large position error: " << error_norm << " m" << std::endl;
                    }
                }

                return true;
            } else {
                failed_solutions_++;

                std::cerr << "Track-IK failed for target [" << std::fixed << std::setprecision(3) 
                        << target_pos.x() << ", " << target_pos.y() << ", " << target_pos.z() << "]" << std::endl;

                switch (result) {
                    case -1:
                        std::cerr << "Track-IK failed: Timeout occurred" << std::endl;
                        break;
                    case -2:
                        std::cerr << "Track-IK failed: No solution within tolerance" << std::endl;
                        break;
                    case -3:
                        std::cerr << "Track-IK failed: Invalid inputs" << std::endl;
                        break;
                    default:
                        std::cerr << "Track-IK failed: Unknown error code " << result << std::endl;
                        break;
                }

                return false;
            }
        }


    void waypoint_checker() {
        for (size_t i = 0; i < path.size(); ++i) {
            bool all_joints_match = false;

            for (unsigned int j = 0; j < current_joint_positions_.rows(); ++j) {
                if (path[i](j) == current_joint_positions_(j)) {  // add tolerance
                    all_joints_match = true;
                    break;
                }
            }

            if (all_joints_match) {
                std::cout << "✅ Waypoint " << i << " reached" << std::endl;
            }
        }
    }   

    
    void update() {
        auto now = std::chrono::steady_clock::now();
        
        // Smooth joint interpolation
        bool joints_moving = false;
        for (unsigned int i = 0; i < current_joint_positions_.rows(); ++i) {
        
            double delta = target_joint_positions_(i) - current_joint_positions_(i);
            
            if (std::abs(delta) > convergence_threshold_) {
                double max_step = joint_velocity_limit_ * 0.1; // 100ms timestep
                double step = std::min(std::abs(delta), max_step);
                
                current_joint_positions_(i) += (delta > 0 ? 1 : -1) * step;
                joint_velocities_(i) = (delta > 0 ? 1 : -1) * step / 0.1;

                joints_moving = true;
            } else {
                current_joint_positions_(i) = target_joint_positions_(i);
                joint_velocities_(i) = 0.0;
            }
        }
        
        
        waypoint_checker();
        // Print current and target joint angles
        printJointAngles();
        
        // Check if target is reached
        if (!joints_moving && !target_reached_) {
            target_reached_ = true;
            std::cout << "Target reached!" << std::endl;
        }
        
        // Auto-cycle through test targets
        if (target_reached_ && !test_targets_.empty()) {
            if (std::chrono::duration_cast<std::chrono::seconds>(now - last_target_switch_).count() > 3) {
                // Switch to next target
                current_target_index_ = (current_target_index_ + 1) % test_targets_.size();
                current_target_position_ = test_targets_[current_target_index_];
                
                // Solve IK for new target
                JntArray q_result(kdl_chain_.getNrOfJoints());
                if (solveIKPositionOnly(current_target_position_, current_joint_positions_, q_result)) {
                    target_joint_positions_ = q_result;
                    target_reached_ = false;
                    std::cout << "Switching to target " << current_target_index_ << std::endl;
                } else {
                    std::cout << "WARNING: Failed to solve IK for target " << current_target_index_ << std::endl;
                }
                
                last_target_switch_ = now;
            }
        }
        
        // Print performance statistics occasionally
        static int counter = 0;
        if (++counter % 50 == 0) { // Every 5 seconds
            std::cout << "Performance: Success=" << successful_solutions_ 
                     << ", Failed=" << failed_solutions_ 
                     << ", Avg solve time=" << std::fixed << std::setprecision(3) 
                     << average_solve_time_ << " ms" << std::endl;
        }
    }
    
    void printJointAngles() {
        std::cout << "\n=== Joint Angles ===" << std::endl;
        std::cout << std::left << std::setw(15) << "Joint Name" 
                  << std::setw(15) << "Current (rad)" 
                  << std::setw(15) << "Target (rad)" 
                  << std::endl;
        std::cout << std::string(75, '-') << std::endl;
        bool all_reached=true;
        std::vector<double> joint_errors;

        double error;
        
        for (unsigned int i = 0; i < current_joint_positions_.rows(); ++i) {
            std::string joint_name = (i < joint_names_.size()) ? joint_names_[i] : ("joint_" + std::to_string(i));
            
            double current_rad = current_joint_positions_(i);
            double target_rad = target_joint_positions_(i);
            double current_deg = current_rad * 180.0 / M_PI;
            double target_deg = target_rad * 180.0 / M_PI;
            
            joint_errors.push_back(error);
            
            if (std::abs(current_rad - target_rad) > 1e-3)  // 🔍 threshold check
            all_reached = false;

            
            
            std::cout << std::left << std::setw(15) << joint_name
                      << std::setw(15) << std::fixed << std::setprecision(3) << current_rad
                      << std::setw(15) << std::fixed << std::setprecision(3) << target_rad<< std::endl;;
                    //   << std::setw(15) << std::fixed << std::setprecision(1) << current_deg
                    //   << std::setw(15) << std::fixed << std::setprecision(1) << target_deg << std::endl;



        }


        for (size_t i = 0; i < path.size(); ++i) {
                    std::cout << "Step " << i << ": ";
                    for (unsigned int j = 0; j < path[i].rows(); ++j) {
                        std::cout << std::fixed << std::setprecision(3) << path[i](j) << " ";
                    }
                    std::cout << std::endl;
                }

        
        if (all_reached) {
            std::cout << "\n✅ Reached Target Joint Configuration!" << std::endl;
            std::cout << "\nJoint-wise Error (target - current) in radians:\n";
            
            for (size_t i = 0; i < joint_errors.size(); ++i) {
                std::string joint_name = (i < joint_names_.size()) ? joint_names_[i] : ("joint_" + std::to_string(i));
                std::cout << std::setw(2) << i << ". " << std::setw(15) << joint_name
                        << ": " << std::fixed << std::setprecision(6) << joint_errors[i] << " rad" << std::endl;
            }

            std::cout << std::flush; 
        }

      
        
        // Print current end-effector position
        Frame current_frame;
        if (fk_solver_->JntToCart(current_joint_positions_, current_frame) >= 0) {
            std::cout << "\nCurrent EE Position: [" << std::fixed << std::setprecision(3) 
                     << current_frame.p.x() << ", " << current_frame.p.y() << ", " << current_frame.p.z() << "]" << std::endl;
        }
        
        std::cout << "Target Position: [" << std::fixed << std::setprecision(3) 
                 << current_target_position_.x() << ", " << current_target_position_.y() << ", " << current_target_position_.z() << "]" << std::endl;
        std::cout << std::string(75, '=') << std::endl;
    }
};

int main() {
    std::cout << "Starting Native Track-IK Node (Position-only mode)" << std::endl;
    
    try {
        NativeTrackIKNode node;
        node.run();
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}