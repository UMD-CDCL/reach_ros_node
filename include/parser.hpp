#ifndef REACH_SERIAL_CPP__PARSER_HPP_
#define REACH_SERIAL_CPP__PARSER_HPP_

#include <string>
#include <vector>

struct ParsedSentence {
  std::string type;
  std::vector<std::string> fields;
};

ParsedSentence parse_nmea_sentence(const std::string &sentence);

#endif  // REACH_SERIAL_CPP__PARSER_HPP_