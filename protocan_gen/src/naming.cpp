#include "naming.hpp"

#include <cctype>
#include <string>

std::string camel_to_snake(const std::string & name)
{
  std::string result;
  result.reserve(name.size() + 8);

  for (size_t i = 0; i < name.size(); ++i) {
    char c = name[i];
    if (std::isupper(static_cast<unsigned char>(c))) {
      if (i > 0) {
        bool prev_lower = std::islower(static_cast<unsigned char>(name[i - 1]));
        bool prev_upper = std::isupper(static_cast<unsigned char>(name[i - 1]));
        bool next_lower =
          (i + 1 < name.size()) && std::islower(static_cast<unsigned char>(name[i + 1]));
        if (prev_lower || (prev_upper && next_lower)) {
          result += '_';
        }
      }
      result += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    } else {
      result += c;
    }
  }

  // Compress consecutive underscores
  std::string compressed;
  compressed.reserve(result.size());
  bool prev_under = false;
  for (char c : result) {
    if (c == '_') {
      if (!prev_under) compressed += c;
      prev_under = true;
    } else {
      compressed += c;
      prev_under = false;
    }
  }

  return compressed;
}
