#include <pybind11/pybind11.h>

#include <iostream>

#include "smmap/planner.h"

namespace py = pybind11;

void foo()
{
    std::cout << "Hello world\n";
}

class SmmapWrapper
{
public:
    SmmapWrapper()
        : nh_()
        , ph_("smmap_planner_node")
        , robot_(nh_)
        , vis_(nh_, ph_)
        , task_specification_(smmap::TaskSpecification::MakeTaskSpecification(nh_, ph_))
        , planner_(robot_, vis_, task_specification_)
    {}

    void execute()
    {
        planner_.execute();
    }

private:

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    smmap::RobotInterface robot_;
    smmap_utilities::Visualizer vis_;

    smmap::TaskSpecification::Ptr task_specification_;
    smmap::Planner planner_;
};

PYBIND11_MODULE(smmap_python_bindings, m)
{
    m.doc() = "SMMAP Library Plugin";

    m.def("foo", &foo, "Random hello world test.");

    py::class_<SmmapWrapper>(m, "SmmapWrapper")
        .def(py::init<>())
        .def("execute", &SmmapWrapper::execute);
}
