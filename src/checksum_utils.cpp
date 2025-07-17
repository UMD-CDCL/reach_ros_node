#include "checksum_utils.hpp"
#include <sstream>
#include <iomanip>

bool check_nmea_checksum(const std::string &sentence) {
  auto star = sentence.find('*');
  if (star == std::string::npos || star + 3 > sentence.size()) return false;
  unsigned char cs = 0;
  for (size_t i = 1; i < star; ++i)
    cs ^= static_cast<unsigned char>(sentence[i]);
  std::stringstream ss;
  ss << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << cs;
  return ss.str() == sentence.substr(star + 1, 2);
}

std::string compute_nmea_checksum(const std::string &sentence) {
  unsigned char cs = 0;
  for (size_t i = 1; i < sentence.size() && sentence[i] != '*'; ++i)
    cs ^= static_cast<unsigned char>(sentence[i]);
  std::stringstream ss;
  ss << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << cs;
  return ss.str();
}
