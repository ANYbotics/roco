/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Main test file
 */

#include <gtest/gtest.h>

#include <roco/controllers/controllers.hpp>
#include <roco/model/CommandInterface.hpp>
#include <roco/model/StateInterface.hpp>
#include <roco/time/Time.hpp>
#include <roco/time/TimeStd.hpp>
#include <roco/workers/Worker.hpp>
#include <roco/workers/WorkerEvent.hpp>
#include <roco/workers/WorkerEventInterface.hpp>
#include <roco/workers/WorkerEventStd.hpp>
#include <roco/workers/WorkerHandle.hpp>
#include <roco/workers/WorkerInterface.hpp>
#include <roco/workers/WorkerOptions.hpp>

using ::testing::InitGoogleTest;

/* RUN TESTS */
int main(int argc, char** argv) {
  InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
