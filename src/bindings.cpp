#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <iostream>
#include <functional>

#include <smmap/planner.h>
#include <smmap/grippers.hpp>

namespace py = pybind11;

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
            std::function<EigenHelpers::VectorMatrix4d(const Eigen::VectorXd& configuration)> get_ee_poses_fn,
            std::function<Eigen::MatrixXd(const Eigen::VectorXd& configuration)> get_grippers_jacobian_fn,
            std::function<std::vector<Eigen::Vector3d>(const Eigen::VectorXd& configuration)> get_collision_points_of_interest_fn,
            std::function<std::vector<Eigen::MatrixXd>(const Eigen::VectorXd& configuration)> get_collision_points_of_interest_jacobians_fn,
            std::function<bool(const Eigen::VectorXd& configuration)> full_robot_collision_check_fn)
    {
        execute_thread_ = std::thread(
                    &SmmapWrapper::execute_impl,
                    this,
                    get_ee_poses_fn,
                    get_grippers_jacobian_fn,
                    get_collision_points_of_interest_fn,
                    get_collision_points_of_interest_jacobians_fn,
                    full_robot_collision_check_fn);
    }

private:

    std::thread execute_thread_;

    // These need to be created in a seperate thread to avoid GIL problems, so use "lazy" initialization
    void execute_impl(
            std::function<EigenHelpers::VectorMatrix4d(const Eigen::VectorXd& configuration)> get_ee_poses_fn,
            std::function<Eigen::MatrixXd(const Eigen::VectorXd& configuration)> get_grippers_jacobian_fn,
            std::function<std::vector<Eigen::Vector3d>(const Eigen::VectorXd& configuration)> get_collision_points_of_interest_fn,
            std::function<std::vector<Eigen::MatrixXd>(const Eigen::VectorXd& configuration)> get_collision_points_of_interest_jacobians_fn,
            std::function<bool(const Eigen::VectorXd& configuration)> full_robot_collision_check_fn) const
    {
        assert(ros::isInitialized());

        ros::NodeHandle nh;
        ros::NodeHandle ph("smmap_planner_node");

        const auto get_ee_poses_with_conversion_fn = [&get_ee_poses_fn] (const Eigen::VectorXd& configuration)
        {
            const auto as_matrix4d = get_ee_poses_fn(configuration);
            smmap::AllGrippersSinglePose as_isometry(as_matrix4d.size());
            for (size_t idx = 0; idx < as_matrix4d.size(); ++idx)
            {
                as_isometry[idx] = as_matrix4d[idx];
            }
            return as_isometry;
        };


        ROS_INFO("Creating utility objects");
        smmap::RobotInterface::Ptr robot = std::make_shared<smmap::RobotInterface>(nh, ph);
        robot->setCallbackFunctions(
                    get_ee_poses_with_conversion_fn,
                    get_grippers_jacobian_fn,
                    get_collision_points_of_interest_fn,
                    get_collision_points_of_interest_jacobians_fn,
                    full_robot_collision_check_fn);
        smmap_utilities::Visualizer::Ptr vis = std::make_shared<smmap_utilities::Visualizer>(nh, ph);
        smmap::TaskSpecification::Ptr task_specification(smmap::TaskSpecification::MakeTaskSpecification(nh, ph, vis));

        ROS_INFO("Creating and executing planner");
        smmap::Planner planner(nh, ph, robot, vis, task_specification);
        planner.execute();

        ROS_INFO("Disposing planner...");
    }
};

PYBIND11_MODULE(smmap_python_bindings, m)
{
    m.doc() = "SMMAP Library Plugin";

    py::class_<SmmapWrapper>(m, "SmmapWrapper")
        .def(py::init<>())
        .def("execute", &SmmapWrapper::execute);
}


/// Reference:
// python -c "import sys; print '\n'.join(sys.path); sys.path.append('/home/dmcconac/Dropbox/catkin_ws/devel/lib'); import smmap_python_bindings as smmap; smmap.foo()"
