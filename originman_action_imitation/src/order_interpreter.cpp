#include "order_interpreter.hpp"
#include <stdexcept>

OrderInterpreter::OrderInterpreter() {
    Py_Initialize();
    if (!initialize_python_modules()) {
        throw std::runtime_error("Failed to initialize Python modules");
    }
}

OrderInterpreter::~OrderInterpreter() {
    cleanup_python_objects();
    Py_Finalize();
}

void OrderInterpreter::control_serial_servo(const std::string& order, int sleep_time) {
    PyObject* pArgs = PyTuple_Pack(1, PyUnicode_FromString(order.c_str()));
    if (!pArgs || !PyObject_CallObject(pFunc_run_action_group_, pArgs)) {
        PyErr_Print();
    }
    Py_XDECREF(pArgs);
    if (sleep_time > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
    }
}

void OrderInterpreter::control_serial_servo(int servo_id, int pulse, int use_time) {
    PyObject* pArgs = PyTuple_Pack(3, PyLong_FromLong(servo_id), PyLong_FromLong(pulse), PyLong_FromLong(use_time));
    if (!pArgs || !PyObject_CallObject(pFunc_set_bus_servo_pulse_, pArgs)) {
        PyErr_Print();
    }
    execute_sleep(use_time);
    Py_XDECREF(pArgs);
}

void OrderInterpreter::control_pwm_servo(int servo_id, int pulse, int use_time) {
    PyObject* pArgs = PyTuple_Pack(3, PyLong_FromLong(servo_id), PyLong_FromLong(pulse), PyLong_FromLong(use_time));
    if (!pArgs || !PyObject_CallObject(pFunc_set_pwm_servo_pulse_, pArgs)) {
        PyErr_Print();
    }
    execute_sleep(use_time);
    Py_XDECREF(pArgs);
}

bool OrderInterpreter::initialize_python_modules() {
    pModule_action_group_ = PyImport_ImportModule("hiwonder.ActionGroupControl");
    pFunc_run_action_group_ = PyObject_GetAttrString(pModule_action_group_, "runActionGroup");
    pModule_controller_ = PyImport_ImportModule("hiwonder.Controller");
    pModule_rrc_ = PyImport_ImportModule("hiwonder.ros_robot_controller_sdk");
    pFunc_controller_ = PyObject_GetAttrString(pModule_controller_, "Controller");
    pClass_board_ = PyObject_GetAttrString(pModule_rrc_, "Board");
    pInstance_board_ = PyObject_CallObject(pClass_board_, nullptr);
    pArgs_ = PyTuple_Pack(1, pInstance_board_);
    pInstance_ = PyObject_CallObject(pFunc_controller_, pArgs_);
    pFunc_set_pwm_servo_pulse_ = PyObject_GetAttrString(pInstance_, "set_pwm_servo_pulse");
    pFunc_set_bus_servo_pulse_ = PyObject_GetAttrString(pInstance_, "set_bus_servo_pulse");
    pTimeModule_ = PyImport_ImportModule("time");
    pSleepFunc_ = PyObject_GetAttrString(pTimeModule_, "sleep");

    return pModule_action_group_ && pFunc_run_action_group_ && PyCallable_Check(pFunc_run_action_group_) &&
           pModule_controller_ && pModule_rrc_ && pFunc_controller_ && pClass_board_ &&
           pInstance_board_ && pArgs_ && pInstance_ && pFunc_set_pwm_servo_pulse_ &&
           pFunc_set_bus_servo_pulse_ && pTimeModule_ && pSleepFunc_ && PyCallable_Check(pSleepFunc_);
}

void OrderInterpreter::execute_sleep(int use_time) {
    PyObject* pArgs_sleep = PyTuple_Pack(1, PyFloat_FromDouble(use_time / 1000.0));
    if (!pArgs_sleep || !PyObject_CallObject(pSleepFunc_, pArgs_sleep)) {
        PyErr_Print();
    }
    Py_XDECREF(pArgs_sleep);
}

void OrderInterpreter::cleanup_python_objects() {
    Py_XDECREF(pFunc_run_action_group_); Py_XDECREF(pModule_action_group_);
    Py_XDECREF(pModule_controller_); Py_XDECREF(pModule_rrc_);
    Py_XDECREF(pFunc_controller_); Py_XDECREF(pClass_board_);
    Py_XDECREF(pInstance_board_); Py_XDECREF(pArgs_); Py_XDECREF(pInstance_);
    Py_XDECREF(pFunc_set_pwm_servo_pulse_); Py_XDECREF(pFunc_set_bus_servo_pulse_);
    Py_XDECREF(pSleepFunc_); Py_XDECREF(pTimeModule_);
}