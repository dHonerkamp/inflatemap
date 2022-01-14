#include <inflatemap/mycostmap.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(pybindings, m) {
    m.def("get_inflated_map", &my_costmap::getInflatedMapPython, "getInflatedMap");

#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}
