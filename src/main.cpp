#include "mpc_controller.h"

int main(int argc, char const *argv[])
{
    std::unique_ptr<shibo::controller::MPC_controller> mpc_controller = std::make_unique<shibo::controller::MPC_controller>();
    mpc_controller->init();

    while (true)
    {
        mpc_controller->compute_mpc();
    }
    

    return 0;
}
