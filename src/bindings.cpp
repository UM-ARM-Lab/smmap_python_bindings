#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <iostream>
#include <functional>

#include "smmap/planner.h"

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

    void execute()
    {
        execute_thread_ = std::thread(&SmmapWrapper::execute_impl, this);
    }

private:

    std::thread execute_thread_;

    // These need to be created in a seperate thread to avoid GIL problems, so use "lazy" initialization
    void execute_impl() const
    {
        assert(ros::isInitialized());

        ros::NodeHandle nh;
        ros::NodeHandle ph("smmap_planner_node");

        ROS_INFO("Creating utility objects");
        smmap::RobotInterface robot(nh);
        smmap_utilities::Visualizer::Ptr vis = std::make_shared<smmap_utilities::Visualizer>(nh, ph);
        smmap::TaskSpecification::Ptr task_specification(smmap::TaskSpecification::MakeTaskSpecification(nh, ph, vis));

        ROS_INFO("Creating and executing planner");
        smmap::Planner planner(nh, ph, robot, vis, task_specification);
        planner.execute();

        ROS_INFO("Disposing planner...");
    }
};

//using MatrixXdR = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

//void SetJacobian(const Eigen::Ref<MatrixXdR>& jacobian)
//void SetJacobian(const Eigen::Ref<Eigen::MatrixXd>& jacobian)
void SetJacobian(const Eigen::MatrixXd& jacobian)
{
//    std::cout << "Ind: " << manipulator_ind << std::endl;
    std::cout << "Jacobian:\n" << jacobian << std::endl;
}

//static std::function<Eigen::MatrixXd()> global_callback;

void SetJacobianCallbackFunction(const std::function<Eigen::MatrixXd(int)>& callback)
{
    const Eigen::MatrixXd jacobian = callback(1);
    std::cout << jacobian << std::endl;
}

PYBIND11_MODULE(smmap_python_bindings, m)
{
    m.doc() = "SMMAP Library Plugin";

    py::class_<SmmapWrapper>(m, "SmmapWrapper")
        .def(py::init<>())
        .def("execute", &SmmapWrapper::execute);

    m.def("SetJacobian", &SetJacobian, "Sets the Jacobian for the indicted manipulator");

    m.def("SetJacobianCallbackFunction", &SetJacobianCallbackFunction, "Function pointer test");

//    m.def("cholesky1", [](Eigen::Ref<MatrixXdR> x) -> Eigen::MatrixXd { return x.llt().matrixL(); });
//    m.def("cholesky2", [](const Eigen::Ref<const MatrixXdR> &x) -> Eigen::MatrixXd { return x.llt().matrixL(); });
}


/// Reference:
// python -c "import sys; print '\n'.join(sys.path); sys.path.append('/home/dmcconac/Dropbox/catkin_ws/devel/lib'); import smmap_python_bindings as smmap; smmap.foo()"
