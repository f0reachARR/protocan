#include "protocan/pdo_manager.hpp"

namespace protocan
{

std::optional<uint16_t> PdoManager::allocate(uint32_t priority)
{
  PdoPriority band = priority_to_band(priority);

  switch (band) {
    case PdoPriority::HIGH:
      return find_free_id(kPdoHighPriorityMin, kPdoHighPriorityMax);
    case PdoPriority::MID:
      return find_free_id(kPdoMidPriorityMin, kPdoMidPriorityMax);
    case PdoPriority::LOW:
      return find_free_id(kPdoLowPriorityMin, kPdoLowPriorityMax);
    default:
      return find_free_id(kPdoLowPriorityMin, kPdoLowPriorityMax);
  }
}

void PdoManager::release(uint16_t pdo_id)
{
  allocated_.erase(pdo_id);
  mappings_.erase(pdo_id);
}

void PdoManager::set_mapping(const PdoMapping & mapping)
{
  mappings_[mapping.pdo_id] = mapping;
  allocated_[mapping.pdo_id] = true;
}

std::optional<PdoMapping> PdoManager::get_mapping(uint16_t pdo_id) const
{
  auto it = mappings_.find(pdo_id);
  if (it == mappings_.end()) {
    return std::nullopt;
  }
  return it->second;
}

void PdoManager::remove_mapping(uint16_t pdo_id)
{
  mappings_.erase(pdo_id);
  allocated_.erase(pdo_id);
}

void PdoManager::reset()
{
  mappings_.clear();
  allocated_.clear();
}

std::optional<uint16_t> PdoManager::find_free_id(uint16_t min_id, uint16_t max_id) const
{
  for (uint16_t id = min_id; id <= max_id; ++id) {
    if (allocated_.find(id) == allocated_.end()) {
      return id;
    }
  }
  return std::nullopt;
}

}  // namespace protocan
