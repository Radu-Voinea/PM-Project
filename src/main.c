#include "esp_log.h"

#if defined(BUILD_CAR)
#include "car.h"
#elif defined(BUILD_REMOTE)
#include "remote.h"
#else
#error "Define BUILD_CAR or BUILD_REMOTE via build_flags"
#endif

void app_main(void)
{
#if defined(BUILD_CAR)
    car_main();
#else
    remote_main();
#endif
}
