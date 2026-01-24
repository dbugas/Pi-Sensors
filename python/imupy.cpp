#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include "IMU.h"

namespace py = pybind11;

/* ---------------- Quaternion helper ---------------- */

struct Quaternion {
    double w, x, y, z;

    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(double w_, double x_, double y_, double z_)
        : w(w_), x(x_), y(y_), z(z_) {}

    static Quaternion from_quat_data(const QuatData &qd) {
        return {qd.q[0], qd.q[1], qd.q[2], qd.q[3]};
    }
};

/* ---------------- Python module ---------------- */

PYBIND11_MODULE(imupy, m) {
    m.doc() = "Python bindings for 10DOF IMU";

    /* -------- Quaternion -------- */

    py::class_<Quaternion>(m, "Quaternion")
        .def(py::init<>())
        .def(py::init<double, double, double, double>())
        .def_readwrite("w", &Quaternion::w)
        .def_readwrite("x", &Quaternion::x)
        .def_readwrite("y", &Quaternion::y)
        .def_readwrite("z", &Quaternion::z)
        .def("to_tuple", [](const Quaternion &q) {
            return py::make_tuple(q.w, q.x, q.y, q.z);
        })
        .def("to_numpy", [](const Quaternion &q) {
            py::array_t<double> arr(4);
            auto *ptr = arr.mutable_data();
            ptr[0] = q.w;
            ptr[1] = q.x;
            ptr[2] = q.y;
            ptr[3] = q.z;
            return arr;
        })
        .def("__repr__", [](const Quaternion &q) {
            return py::str("Quaternion(w={}, x={}, y={}, z={})")
                   .format(q.w, q.x, q.y, q.z);
        });

    /* -------- IMU Performance Mode -------- */

    py::enum_<IMU::PerformanceMode>(m, "PerformanceMode")
        .value("Ultra",  IMU::PerformanceMode::Ultra)
        .value("High",   IMU::PerformanceMode::High)
        .value("Medium", IMU::PerformanceMode::Medium)
        .value("Low",    IMU::PerformanceMode::Low)
        .export_values();

    /* -------- IMU Class -------- */

    py::class_<IMU>(m, "IMU")
        .def(py::init<IMU::PerformanceMode, bool, bool>(),
             py::arg("PerformanceMode") = IMU::PerformanceMode::High,
             py::arg("Use_Mag") = true,
             py::arg("Use_Barometer") = true)

        .def("start_sensor_thread", [](IMU &self) {
            py::gil_scoped_release release;
            self.start_sensor_thread();
        })
        .def("stop_sensor_threads", [](IMU &self) {
            py::gil_scoped_release release;
            self.stop_sensor_threads();
        })

        .def("update_quat_thread", [](IMU &self) {
            py::gil_scoped_release release;
            self.update_quat_thread();
        })

        .def("Accel_raw", [](IMU &self) {
            const AccelData* a = self.latest_accel();
            return py::make_tuple(a->x, a->y, a->z);
        })

        .def("Gyro_raw", [](IMU &self) {
            const GyroData* g = self.latest_gyro();
            return py::make_tuple(g->x, g->y, g->z);
        })

        .def("Mag_raw", [](IMU &self) {
            const MagData* m = self.latest_mag();
            return py::make_tuple(m->x, m->y, m->z);
        })

        .def("Baro_raw", [](IMU &self) {
            const BaroData* b = self.latest_baro();
            return py::make_tuple(
                b->pressure_Pa,
                b->temperature_C,
                b->altitude_m
            );
        })

        .def("GetQuat", [](IMU &self) {
            QuatData q;
            self.get_latest_quat_and_consume(q);
            return Quaternion::from_quat_data(q);
        })
        .def("update_quat", [](IMU &self) {
            self.update_quat();
        })
        .def("update_mag_raw", [](IMU &self) {
            self.update_mag_raw();
        })
        .def("update_imu", [](IMU &self) {
            self.update_imu();
        });
}
