find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
find_path(WIRINGPI_INCLUDE_DIRS NAMES wiringPi.h)
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(WiringPi DEFAULT_MSG WIRINGPI_LIBRARIES WIRINGPI_INCLUDE_DIRS)
