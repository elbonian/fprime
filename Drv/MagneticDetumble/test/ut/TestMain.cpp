#include "Tester.hpp"
#include <gtest/gtest.h>
#include <Fw/Test/UnitTest.hpp>

// Instantiate a global Os::Log logger if needed by F Prime test infrastructure
// #include <Os/Log.hpp>
// Os::Log logger;


TEST(MagneticDetumbleTest, DirectDipoleCommand) {
    Drv::Tester tester;
    tester.test_directDipoleCommand();
}

TEST(MagneticDetumbleTest, SetDipoleCmdInPort) {
    Drv::Tester tester;
    tester.test_setDipoleCmdInPort();
}

TEST(MagneticDetumbleTest, BdotConfigurationCommands) {
    Drv::Tester tester;
    tester.test_bdotConfigurationCommands();
}

TEST(MagneticDetumbleTest, BdotAlgorithmNominal) {
    Drv::Tester tester;
    tester.test_bdotAlgorithm_nominal();
}

TEST(MagneticDetumbleTest, BdotAlgorithmStaleMagField) {
    Drv::Tester tester;
    tester.test_bdotAlgorithm_staleMagField();
}

TEST(MagneticDetumbleTest, BdotAlgorithmStaleAngVel) {
    Drv::Tester tester;
    tester.test_bdotAlgorithm_staleAngVel();
}

TEST(MagneticDetumbleTest, BdotAlgorithmZeroMagField) {
    Drv::Tester tester;
    tester.test_bdotAlgorithm_zeroMagField();
}

TEST(MagneticDetumbleTest, BdotAlgorithmInvalidDt) {
    Drv::Tester tester;
    tester.test_bdotAlgorithm_invalidDt();
}

TEST(MagneticDetumbleTest, CoilSaturation) {
    Drv::Tester tester;
    tester.test_coilSaturation();
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  // Optional: Fw::Test::UnitTest::setLogger(&logger);
  return RUN_ALL_TESTS();
}
