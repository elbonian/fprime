#include "Tester.hpp"
#include <gtest/gtest.h>

TEST(MagneticDetumbleTest, Initialization) {
    Svc::Tester tester;
    tester.testInitialization();
}

TEST(MagneticDetumbleTest, GainCalculation) {
    Svc::Tester tester;
    tester.testGainCalculation();
}

TEST(MagneticDetumbleTest, NominalCalculation) {
    Svc::Tester tester;
    tester.testNominalCalculation();
}

TEST(MagneticDetumbleTest, ZeroMagField) {
    Svc::Tester tester;
    tester.testZeroMagField();
}

TEST(MagneticDetumbleTest, Saturation) {
    Svc::Tester tester;
    tester.testSaturation();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
