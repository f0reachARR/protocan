#include <gtest/gtest.h>

#include "protocan/pdo_manager.hpp"

using namespace protocan;

TEST(PdoManager, AllocateHighPriority)
{
  PdoManager mgr;

  // priority 0-2 → High band (0x001–0x0FF)
  auto id = mgr.allocate(0);
  ASSERT_TRUE(id.has_value());
  EXPECT_GE(*id, kPdoHighPriorityMin);
  EXPECT_LE(*id, kPdoHighPriorityMax);
}

TEST(PdoManager, AllocateMidPriority)
{
  PdoManager mgr;

  auto id = mgr.allocate(3);
  ASSERT_TRUE(id.has_value());
  EXPECT_GE(*id, kPdoMidPriorityMin);
  EXPECT_LE(*id, kPdoMidPriorityMax);
}

TEST(PdoManager, AllocateLowPriority)
{
  PdoManager mgr;

  auto id = mgr.allocate(7);
  ASSERT_TRUE(id.has_value());
  EXPECT_GE(*id, kPdoLowPriorityMin);
  EXPECT_LE(*id, kPdoLowPriorityMax);
}

TEST(PdoManager, ReleaseAndReallocate)
{
  PdoManager mgr;

  auto id1 = mgr.allocate(0);
  ASSERT_TRUE(id1.has_value());

  mgr.release(*id1);

  auto id2 = mgr.allocate(0);
  ASSERT_TRUE(id2.has_value());
  EXPECT_EQ(*id1, *id2);  // 解放後に同じ ID が再割り当てされる
}

TEST(PdoManager, SetAndGetMapping)
{
  PdoManager mgr;

  PdoMapping mapping;
  mapping.pdo_id = 0x181;
  mapping.direction = PdoCfgDirection::TX;
  mapping.period_ms = 10;
  mapping.total_size = 8;
  mapping.entries.push_back({1, 0, 0, 0, 0, 4});
  mapping.entries.push_back({1, 0, 0, 1, 4, 4});

  mgr.set_mapping(mapping);

  auto result = mgr.get_mapping(0x181);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->pdo_id, 0x181);
  EXPECT_EQ(result->entries.size(), 2u);
  EXPECT_EQ(result->total_size, 8);
}

TEST(PdoManager, RemoveMapping)
{
  PdoManager mgr;

  PdoMapping mapping;
  mapping.pdo_id = 0x200;
  mapping.direction = PdoCfgDirection::RX;
  mgr.set_mapping(mapping);

  EXPECT_TRUE(mgr.get_mapping(0x200).has_value());

  mgr.remove_mapping(0x200);
  EXPECT_FALSE(mgr.get_mapping(0x200).has_value());
}

TEST(PdoManager, Reset)
{
  PdoManager mgr;

  PdoMapping mapping;
  mapping.pdo_id = 0x100;
  mgr.set_mapping(mapping);

  mgr.reset();

  EXPECT_TRUE(mgr.mappings().empty());
}
