#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "LISxMDL.h"

namespace py = pybind11;

PYBIND11_MODULE(pylisxmdl, m) {
    m.doc() = "Python bindings for LISxMDL";

    // Enums
    py::enum_<LISxMDL::ODR>(m, "ODR")
        .value("Hz_0_625", LISxMDL::ODR::Hz_0_625)
        .value("Hz_1_25",  LISxMDL::ODR::Hz_1_25)
        .value("Hz_2_5",   LISxMDL::ODR::Hz_2_5)
        .value("Hz_5",     LISxMDL::ODR::Hz_5)
        .value("Hz_10",    LISxMDL::ODR::Hz_10)
        .value("Hz_20",    LISxMDL::ODR::Hz_20)
        .value("Hz_40",    LISxMDL::ODR::Hz_40)
        .value("Hz_80",    LISxMDL::ODR::Hz_80)
        .value("Hz_155",   LISxMDL::ODR::Hz_155)
        .value("Hz_300",   LISxMDL::ODR::Hz_300)
        .value("Hz_560",   LISxMDL::ODR::Hz_560)
        .value("Hz_1000",  LISxMDL::ODR::Hz_1000)
        .export_values();

    py::enum_<LISxMDL::FullScale>(m, "FullScale")
        .value("Gauss_4",  LISxMDL::FullScale::Gauss_4)
        .value("Gauss_8",  LISxMDL::FullScale::Gauss_8)
        .value("Gauss_12", LISxMDL::FullScale::Gauss_12)
        .value("Gauss_16", LISxMDL::FullScale::Gauss_16)
        .export_values();

    // Class
    py::class_<LISxMDL>(m, "pylisxmdl")
        .def(py::init<LISxMDL::FullScale, LISxMDL::ODR>(),
             py::arg("range") = LISxMDL::FullScale::Gauss_16,
             py::arg("odr")   = LISxMDL::ODR::Hz_80)

        .def("read_raw", [](LISxMDL &self) {
            int16_t mx, my, mz;
            if (self.readRaw(mx, my, mz))
                return py::make_tuple(mx, my, mz);
            throw std::runtime_error("Failed to read raw data");
        })
        .def("read_gauss", [](LISxMDL &self) {
            double mx, my, mz;
            if (self.read_gauss(mx, my, mz))
                return py::make_tuple(mx, my, mz);
            throw std::runtime_error("Failed to read scaled data");
        })
        .def("data_ready", &LISxMDL::dataReady);
}
/*
First start up...
    python3 -m venv .venv
    source .venv/bin/activate
    pip3 install pybind11

Compilation

g++ -O2 -Wall -shared -std=c++14 -fPIC \
    $(python3 -m pybind11 --includes) \
    pyLISxMDL.cpp -o pylisxmdl$(python3-config --extension-suffix) \
    -lpigpio -lrt

*/