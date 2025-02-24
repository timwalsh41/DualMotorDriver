#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include "PidController.h"
#include "IEncoder.h"

namespace py = pybind11;

class PyEncoder : public IEncoder {
public:
    using IEncoder::IEncoder;

    void update_count(bool channel_A, bool channel_B) override {
        PYBIND11_OVERLOAD_PURE(void, IEncoder, update_count, channel_A, channel_B);
    }

    int32_t update_delta() override {
        PYBIND11_OVERLOAD_PURE(int32_t, IEncoder, update_delta);
    }

    void reset_count() override {
        PYBIND11_OVERLOAD_PURE(void, IEncoder, reset_count);
    }

    int64_t get_count() const override {
        PYBIND11_OVERLOAD_PURE(int64_t, IEncoder, get_count);
    }

    int64_t get_last_count() const override {
        PYBIND11_OVERLOAD_PURE(int64_t, IEncoder, get_last_count);
    }

    void set_count(int32_t count) override {
        PYBIND11_OVERLOAD_PURE(void, IEncoder, set_count, count);
    }
};

PYBIND11_MODULE(PidControllerWrapper, m) {
    py::enum_<ControlMode>(m, "ControlMode")
        .value("OPEN_LOOP", ControlMode::OPEN_LOOP)
        .value("CLOSED_LOOP", ControlMode::CLOSED_LOOP);

    py::class_<IEncoder, PyEncoder>(m, "IEncoder")
        .def(py::init<>())
        .def("update_count", &IEncoder::update_count)
        .def("update_delta", &IEncoder::update_delta)
        .def("reset_count", &IEncoder::reset_count)
        .def("get_count", &IEncoder::get_count)
        .def("get_last_count", &IEncoder::get_last_count)
        .def("set_count", &IEncoder::set_count);

    py::class_<PidController>(m, "PidController")
        .def(py::init<float, float, float, float, IEncoder*>())
        .def("run_controller", &PidController::run_controller)
        .def("get_current_speed_cps", &PidController::get_current_speed_cps)
        .def("set_target_speed", &PidController::set_target_speed)
        .def("set_mode", &PidController::set_mode)
        .def("get_kP", &PidController::get_kP)
        .def("get_kI", &PidController::get_kI)
        .def("get_kD", &PidController::get_kD)
        .def("get_kFF", &PidController::get_kFF)
        .def("get_target_speed_cps", &PidController::get_target_speed_cps)
        .def("get_accumulated_error", &PidController::get_accumulated_error)
        .def("set_kP", &PidController::set_kP)
        .def("set_kI", &PidController::set_kI)
        .def("set_kD", &PidController::set_kD)
        .def("set_kFF", &PidController::set_kFF);
}
