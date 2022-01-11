/*----------------------------------------------------------------------------*/
/* ADIS16448 RoboRIO Driver (c) by Juan Chong
/*
/* The ADIS16448 RoboRIO Driver is licensed under a
/* Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
/*
/* You should have received a copy of the license along with this
/* work. If not, see <http://creativecommons.org/licenses/by-nc-sa/4.0/>.                             */
/*----------------------------------------------------------------------------*/

#include <hal/HAL.h>

#include "gtest/gtest.h"

int main(int argc, char** argv) {
    HAL_Initialize(500, 0);
    ::testing::InitGoogleTest(&argc, argv);
    int ret = RUN_ALL_TESTS();
    return ret;
}
