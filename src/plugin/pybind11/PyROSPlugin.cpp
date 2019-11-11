/*!
  @author Shin'ichiro Nakaoka
*/

#include "../BodyPublisherItem.h"
#include <cnoid/PyUtil>
#include <cnoid/PyBase>

using namespace cnoid;
namespace py = pybind11;

PYBIND11_MODULE(ROSPlugin, m)
{
    m.doc() = "Choreonoid ROSPlugin module";

    py::module::import("cnoid.Base");
    py::module::import("cnoid.BodyPlugin");

    py::class_<BodyPublisherItem, BodyPublisherItemPtr, ControllerItem>(m, "BodyPublisherItem")
        .def(py::init<>())
        ;
}


