#ifndef ORDER_INTERPRETER_HPP
#define ORDER_INTERPRETER_HPP

#include <python3.10/Python.h>
#include <string>
#include <thread>
#include <chrono>

class OrderInterpreter {
public:
    OrderInterpreter();
    ~OrderInterpreter();

    void control_serial_servo(const std::string& order, int sleep_time = 0);
    void control_serial_servo(int servo_id, int pulse, int use_time);
    void control_pwm_servo(int servo_id, int pulse, int use_time);

private:
    bool initialize_python_modules();
    void execute_sleep(int use_time);
    void cleanup_python_objects();

    PyObject* pModule_action_group_ = nullptr;
    PyObject* pFunc_run_action_group_ = nullptr;
    PyObject* pModule_controller_ = nullptr;
    PyObject* pModule_rrc_ = nullptr;
    PyObject* pFunc_controller_ = nullptr;
    PyObject* pClass_board_ = nullptr;
    PyObject* pInstance_board_ = nullptr;
    PyObject* pArgs_ = nullptr;
    PyObject* pInstance_ = nullptr;
    PyObject* pFunc_set_pwm_servo_pulse_ = nullptr;
    PyObject* pFunc_set_bus_servo_pulse_ = nullptr;
    PyObject* pTimeModule_ = nullptr;
    PyObject* pSleepFunc_ = nullptr;
};

#endif // ORDER_INTERPRETER_HPP