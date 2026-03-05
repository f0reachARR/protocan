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

TEST(PdoManager, GenerateOptimalMappings)
{
  PdoManager mgr;
  uint8_t device_id = 5;

  ParsedDescriptor desc;
  desc.schema_hash = 0x1234;

  // Topic 0: TX, periodic=true(100ms), priority=2 (High), 2 fields (4B, 4B)
  ParsedTopic t0;
  t0.index = 0;
  t0.is_tx = true;
  t0.periodic = true;
  t0.priority = 2;
  t0.message.fields.push_back({"f1", 5, 0, 4, ""});
  t0.message.fields.push_back({"f2", 5, 4, 4, ""});
  desc.topics.push_back(t0);

  // Topic 1: TX, periodic=true(100ms), priority=2 (High), 1 field (8B)
  ParsedTopic t1;
  t1.index = 1;
  t1.is_tx = true;
  t1.periodic = true;
  t1.priority = 2;
  t1.message.fields.push_back({"f3", 9, 0, 8, ""});
  desc.topics.push_back(t1);

  // Topic 2: RX, periodic=false(0ms), priority=5 (Low), 1 field (2B)
  ParsedTopic t2;
  t2.index = 2;
  t2.is_tx = false;
  t2.periodic = false;
  t2.priority = 5;
  t2.message.fields.push_back({"f4", 3, 0, 2, ""});
  desc.topics.push_back(t2);

  std::vector<NodeConfig> configs;
  configs.push_back({10, &desc});  // local_node_id = 10

  auto mappings = mgr.generate_optimal_mappings(device_id, configs);

  // We expect 2 mappings: one for TX (High, 100ms) and one for RX (Low, 0ms)
  ASSERT_EQ(mappings.size(), 2u);

  // 最初のマッピングは TX グループのパッキング
  // Topic 0と1が同じグループに入り、4B+4B+8B = 16B になるはず
  const auto & m_tx = mappings[0];
  EXPECT_EQ(m_tx.direction, PdoCfgDirection::TX);
  EXPECT_EQ(m_tx.period_ms, 100);
  EXPECT_EQ(m_tx.total_size, 16);
  ASSERT_EQ(m_tx.entries.size(), 3u);

  EXPECT_EQ(m_tx.entries[0].topic_index, 0);
  EXPECT_EQ(m_tx.entries[0].size, 4);
  EXPECT_EQ(m_tx.entries[0].offset, 0);

  EXPECT_EQ(m_tx.entries[1].topic_index, 0);
  EXPECT_EQ(m_tx.entries[1].size, 4);
  EXPECT_EQ(m_tx.entries[1].offset, 4);

  EXPECT_EQ(m_tx.entries[2].topic_index, 1);
  EXPECT_EQ(m_tx.entries[2].size, 8);
  EXPECT_EQ(m_tx.entries[2].offset, 8);

  // 2番目のマッピングは RX グループのパッキング
  const auto & m_rx = mappings[1];
  EXPECT_EQ(m_rx.direction, PdoCfgDirection::RX);
  EXPECT_EQ(m_rx.period_ms, 0);
  EXPECT_EQ(m_rx.total_size, 2);
  ASSERT_EQ(m_rx.entries.size(), 1u);

  EXPECT_EQ(m_rx.entries[0].topic_index, 2);
  EXPECT_EQ(m_rx.entries[0].size, 2);
  EXPECT_EQ(m_rx.entries[0].offset, 0);
}
