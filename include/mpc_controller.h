# include <math.h>
# include <iomanip>
# include <memory>
# include <string>

# include "eigen3/Eigen/Core"
# include "common.h"
# include "mpc_osqp.h"

namespace shibo{
    namespace controller{
        class MPC_controller{
            public:
                MPC_controller();
                ~MPC_controller();

                void init();
            private:
        };
    }
}