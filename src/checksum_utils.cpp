#include "checksum_utils.hpp"
#include <cctype>
#include <sstream>
#include <iomanip>
#include <algorithm>

bool check_nmea_checksum(const std::string &sentence) {
  // find the '*' separator
  auto star = sentence.find('*');
  if (star == std::string::npos) return false;

  // extract and clean the transmitted checksum (everything after '*')
  std::string transmitted = sentence.substr(star + 1);
  // strip CR, LF, spaces:
  while (!transmitted.empty() && std::isspace(transmitted.back())) {
    transmitted.pop_back();
  }
  // must be exactly two hex digits
  if (transmitted.size() != 2 ||
      !std::isxdigit(transmitted[0]) ||
      !std::isxdigit(transmitted[1])) {
    return false;
  }

  // compute the XOR over everything between '$' (excluded) and '*' (excluded)
  unsigned char cs = 0;
  // start at i=1 to skip the '$'
  for (size_t i = 1; i < star; ++i) {
    cs ^= static_cast<unsigned char>(sentence[i]);
  }

  // format computed checksum as two uppercase hex digits
  std::ostringstream oss;
  oss << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
      << static_cast<int>(cs);
  std::string comp = oss.str();

  // compare
  return comp == transmitted;
}

std::string compute_nmea_checksum(const std::string &sentence) {
  // same loop as above, but over the entire string until '*' or end
  unsigned char cs = 0;
  for (size_t i = 1; i < sentence.size() && sentence[i] != '*'; ++i) {
    cs ^= static_cast<unsigned char>(sentence[i]);
  }
  std::ostringstream oss;
  oss << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
      << static_cast<int>(cs);
  return oss.str();
}