#include "MotorController.h"
#include "SerialInterface.h"

class MockController : public IMotorController
{
    public:
        MockController() {}
        ~MockController() {}

        void enable_drive() {}
        void disable_drive() {}
        void set_mode(ControlMode mode) {}
        void set_target_speed(MotorChannel channel, float speed_cps) {}
        void clear_encoders() {}
        const SystemStatus& get_status() const {return m_system_status;}
    private:
        SystemStatus m_system_status;
};

MockController controller;

void serial_test_setup()
{

}

void serial_test_teardown()
{

}

void test_serial_type()
{
    SerialInterface interface(&controller);
}

