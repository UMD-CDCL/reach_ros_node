#ifndef REACH_SERIAL_CPP__CHECKSUM_UTILS_HPP_
#define REACH_SERIAL_CPP__CHECKSUM_UTILS_HPP_

#include <string>

bool check_nmea_checksum(const std::string &sentence);
std::string compute_nmea_checksum(const std::string &sentence);

#endif  // REACH_SERIAL_CPP__CHECKSUM_UTILS_HPP_