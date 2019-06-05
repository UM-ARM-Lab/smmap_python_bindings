#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <iostream>
#include <functional>
#include <random>

#include <arc_utilities/timing.hpp>
#include <arc_utilities/math_helpers.hpp>
#include <smmap_utilities/grippers.h>
#include <smmap/task_framework.h>

namespace py = pybind11;
using namespace Eigen;
using namespace EigenHelpers;
using namespace smmap;

class SmmapWrapper
{
public:
    SmmapWrapper()
    {}

    ~SmmapWrapper()
    {
        execute_thread_.join();
    }

    void execute(
            const std::function<void(const unsigned long, const unsigned long)>& reset_random_seeds_fn,
            const std::function<void()>& lock_env_fn,
            const std::function<void()>& unlock_env_fn,
            const std::function<std::vector<VectorXd>()>& get_robot_joint_info_fn,
            const std::function<void(const VectorXd& configuration)> set_active_dof_values_fn,
            const std::function<VectorMatrix4d()> get_ee_poses_fn,
            const std::function<MatrixXd()> get_grippers_jacobian_fn,
            const std::function<std::vector<Vector3d>()> get_collision_points_of_interest_fn,
            const std::function<std::vector<MatrixXd>()> get_collision_points_of_interest_jacobians_fn,
            const std::function<bool()> full_robot_collision_check_fn,
            const std::function<std::vector<VectorXd> (const std::vector<std::string>& gripper_names, const VectorMatrix4d& target_poses, const double max_gripper_distance)>& close_ik_solutions_fn,
            const std::function<std::pair<bool, VectorXd>(const VectorXd& starting_config, const std::vector<std::string>& gripper_names, const VectorMatrix4d& target_poses)> general_ik_solution_fn,
            const std::function<bool(const std::vector<VectorXd>& path)> test_path_for_collision_fn)
    {
        execute_thread_ = std::thread(
                    &SmmapWrapper::execute_impl,
                    this,
                    reset_random_seeds_fn,
                    lock_env_fn,
                    unlock_env_fn,
                    get_robot_joint_info_fn,
                    set_active_dof_values_fn,
                    get_ee_poses_fn,
                    get_grippers_jacobian_fn,
                    get_collision_points_of_interest_fn,
                    get_collision_points_of_interest_jacobians_fn,
                    full_robot_collision_check_fn,
                    close_ik_solutions_fn,
                    general_ik_solution_fn,
                    test_path_for_collision_fn);
    }

    std::vector<double> timingTests(
            const std::function<void()>& lock_env_fn,
            const std::function<void()>& unlock_env_fn,
            const std::function<std::vector<VectorXd>()>& get_robot_joint_info_fn,
            const std::function<void(const VectorXd& configuration)> set_active_dof_values_fn,
            const std::function<VectorMatrix4d()> get_ee_poses_fn,
            const std::function<MatrixXd()> get_grippers_jacobian_fn,
            const std::function<std::vector<Vector3d>()> get_collision_points_of_interest_fn,
            const std::function<std::vector<MatrixXd>()> get_collision_points_of_interest_jacobians_fn,
            const std::function<bool()> full_robot_collision_check_fn,
            const std::function<std::vector<VectorXd> (const std::vector<std::string>& gripper_names, const VectorMatrix4d& target_poses, const double max_gripper_distance)>& close_ik_solutions_fn,
            const std::function<std::pair<bool, VectorXd>(const VectorXd& starting_config, const std::vector<std::string>& gripper_names, const VectorMatrix4d& target_poses)> general_ik_solution_fn,
            const std::function<bool(const std::vector<VectorXd>& path)> test_path_for_collision_fn)
    {
        (void)close_ik_solutions_fn;
        (void)general_ik_solution_fn;
        (void)test_path_for_collision_fn;
        (void)get_robot_joint_info_fn;
        py::gil_scoped_release release;

        VectorXd lower_limits(14);
        lower_limits << -169.9, -119.9, -169.9, -119.9, -169.9, -119.9, -174.9,
                        -169.9, -119.9, -169.9, -119.9, -169.9, -119.9, -174.9;
        lower_limits *= M_PI / 180.0;

        VectorXd upper_limits(14);
        upper_limits << 169.9, 119.9, 169.9, 119.9, 169.9, 119.9, 174.9,
                        169.9, 119.9, 169.9, 119.9, 169.9, 119.9, 174.9;
        upper_limits *= M_PI / 180.0;

        lock_env_fn();
        arc_utilities::Stopwatch stopwatch;
        std::mt19937_64 generator;
        std::uniform_real_distribution<double> uniform_unit_distribution;

        const auto sample = [&] ()
        {
            VectorXd rand_sample(14);
            for (ssize_t idx = 0; idx < 14; ++idx)
            {
                rand_sample(idx) = Interpolate(lower_limits(idx), upper_limits(idx), uniform_unit_distribution(generator));
            }
            return rand_sample;
        };

        std::cout << "Testing ee poses time: " << std::flush;
        stopwatch(arc_utilities::StopwatchControl::RESET);
        for (int idx = 0; idx < 100000; ++idx)
        {
            set_active_dof_values_fn(sample());
            get_ee_poses_fn();
        }
        const double ee_poses_time = stopwatch(arc_utilities::StopwatchControl::READ);
        std::cout << ee_poses_time << std::endl;

        std::cout << "Testing ee grippers jacobian time: " << std::flush;
        stopwatch(arc_utilities::StopwatchControl::RESET);
        for (int idx = 0; idx < 100000; ++idx)
        {
            set_active_dof_values_fn(sample());
            get_grippers_jacobian_fn();
        }
        const double ee_grippers_jacobian_time = stopwatch(arc_utilities::StopwatchControl::READ);
        std::cout << ee_grippers_jacobian_time << std::endl;

        std::cout << "Testing collision POI time: " << std::flush;
        stopwatch(arc_utilities::StopwatchControl::RESET);
        for (int idx = 0; idx < 100000; ++idx)
        {
            set_active_dof_values_fn(sample());
            get_collision_points_of_interest_fn();
        }
        const double collision_points_of_interest_time = stopwatch(arc_utilities::StopwatchControl::READ);
        std::cout << collision_points_of_interest_time << std::endl;

        std::cout << "Testing collision POI jacobian time: " << std::flush;
        stopwatch(arc_utilities::StopwatchControl::RESET);
        for (int idx = 0; idx < 100000; ++idx)
        {
            set_active_dof_values_fn(sample());
            get_collision_points_of_interest_jacobians_fn();
        }
        const double collision_points_of_interest_jacobians_time = stopwatch(arc_utilities::StopwatchControl::READ);
        std::cout << collision_points_of_interest_jacobians_time << std::endl;

        std::cout << "Testing collision check time: " << std::flush;
        stopwatch(arc_utilities::StopwatchControl::RESET);
        for (int idx = 0; idx < 100000; ++idx)
        {
            set_active_dof_values_fn(sample());
            full_robot_collision_check_fn();
        }
        const double collision_check_time = stopwatch(arc_utilities::StopwatchControl::READ);
        std::cout << collision_check_time << std::endl;

        unlock_env_fn();

        return {ee_poses_time, ee_grippers_jacobian_time, collision_points_of_interest_time, collision_points_of_interest_jacobians_time, collision_check_time};
    }

private:

    std::thread execute_thread_;

    // These need to be created in a seperate thread to avoid GIL problems, so use "lazy" initialization
    void execute_impl(
            const std::function<void(const unsigned long, const unsigned long)>& reset_random_seeds_fn,
            const std::function<void()>& lock_env_fn,
            const std::function<void()>& unlock_env_fn,
            const std::function<std::vector<VectorXd>()>& get_robot_joint_info_fn,
            const std::function<void(const VectorXd& configuration)> set_active_dof_values_fn,
            const std::function<VectorMatrix4d()> get_ee_poses_fn,
            const std::function<MatrixXd()> get_grippers_jacobian_fn,
            const std::function<std::vector<Vector3d>()> get_collision_points_of_interest_fn,
            const std::function<std::vector<MatrixXd>()> get_collision_points_of_interest_jacobians_fn,
            const std::function<bool()> full_robot_collision_check_fn,
            const std::function<std::vector<VectorXd> (const std::vector<std::string>& gripper_names, const VectorMatrix4d& target_poses, const double max_gripper_distance)>& close_ik_solutions_fn,
            const std::function<std::pair<bool, VectorXd>(const VectorXd& starting_config, const std::vector<std::string>& gripper_names, const VectorMatrix4d& target_poses)> general_ik_solution_fn,
            const std::function<bool(const std::vector<VectorXd>& path)> test_path_for_collision_fn) const
    {
        assert(ros::isInitialized());

        auto nh = std::make_shared<ros::NodeHandle>();
        auto ph = std::make_shared<ros::NodeHandle>("smmap_planner_node");

        const auto get_ee_poses_with_conversion_fn = [&get_ee_poses_fn] ()
        {
            const auto as_matrix4d = get_ee_poses_fn();
            AllGrippersSinglePose as_isometry(as_matrix4d.size());
            for (size_t idx = 0; idx < as_matrix4d.size(); ++idx)
            {
                as_isometry[idx] = as_matrix4d[idx];
            }
            return as_isometry;
        };

        const auto close_ik_solutions_with_conversion_fn = [&close_ik_solutions_fn] (
                const std::vector<std::string>& gripper_names, const VectorIsometry3d& target_poses, const double max_gripper_distance)
        {
            VectorMatrix4d target_poses_as_matrices(target_poses.size());
            for (size_t idx = 0; idx < target_poses.size(); ++idx)
            {
                target_poses_as_matrices[idx] = target_poses[idx].matrix();
            }
            return close_ik_solutions_fn(gripper_names, target_poses_as_matrices, max_gripper_distance);
        };

        const auto general_ik_solution_with_conversion_fn = [&general_ik_solution_fn] (
                const VectorXd& starting_config,
                const std::vector<std::string>& gripper_names,
                const AllGrippersSinglePose& target_poses)
        {
            VectorMatrix4d target_poses_as_matrices(target_poses.size());
            for (size_t idx = 0; idx < target_poses.size(); ++idx)
            {
                target_poses_as_matrices[idx] = target_poses[idx].matrix();
            }
            return general_ik_solution_fn(starting_config, gripper_names, target_poses_as_matrices);
        };

        ROS_INFO("Creating utility objects");
        auto robot = std::make_shared<RobotInterface>(nh, ph);
        robot->setCallbackFunctions(
                    reset_random_seeds_fn,
                    lock_env_fn,
                    unlock_env_fn,
                    get_robot_joint_info_fn,
                    set_active_dof_values_fn,
                    get_ee_poses_with_conversion_fn,
                    get_grippers_jacobian_fn,
                    get_collision_points_of_interest_fn,
                    get_collision_points_of_interest_jacobians_fn,
                    full_robot_collision_check_fn,
                    close_ik_solutions_with_conversion_fn,
                    general_ik_solution_with_conversion_fn,
                    test_path_for_collision_fn);
        auto vis = std::make_shared<Visualizer>(nh, ph, true);
        auto task_specification = TaskSpecification::MakeTaskSpecification(nh, ph, vis);

        ROS_INFO("Creating and executing planner");
        TaskFramework framework(nh, ph, robot, vis, task_specification);
        framework.execute();

        ROS_INFO("Disposing planner...");
    }
};

PYBIND11_MODULE(smmap_python_bindings, m)
{
    m.doc() = "SMMAP Library Plugin";

    py::class_<SmmapWrapper>(m, "SmmapWrapper")
        .def(py::init<>())
        .def("execute", &SmmapWrapper::execute)
        .def("timingTests", &SmmapWrapper::timingTests);
}


/// Reference:
// python -c "import sys; print '\n'.join(sys.path); sys.path.append('/home/dmcconac/Dropbox/catkin_ws/devel/lib'); import smmap_python_bindings as smmap; smmap.foo()"
