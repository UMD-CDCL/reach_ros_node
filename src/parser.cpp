#include <regex>
#include <sstream>
#include <iostream>
#include "parser.hpp"

ParsedSentence parse_nmea_sentence(const std::string &sentence) {
  ParsedSentence out;

  // first extract the clean NMEA part from received sentence
  // basic NMEA format: starts with $GP/GA/GN/GL, ends with *XX
  static const std::regex nmea_extract_pattern(R"(\$(GP|GA|GN|GL)[A-Z]{3}(,[^,]*)*\*[0-9A-Fa-f]{2})");

  std::sregex_iterator it(sentence.begin(), sentence.end(), nmea_extract_pattern);
  std::sregex_iterator end;

  std::string sentence_cleaned;

  sentence_cleaned = (*it).str();  // gets the first regex match

  // strip checksum suffix
  auto star = sentence_cleaned.find('*');
  std::string core = sentence_cleaned.substr(0, star);

  // std::cout << "core: " << core << std::endl;

  // split on commas
  std::vector<std::string> fields;
  std::stringstream ss(core);
  std::string token;
  while (std::getline(ss, token, ',')) {
    fields.push_back(token);
  }
  if (fields.empty()) {
    return out;
  }

  // extract and remove the header (e.g. "$GPGGA")
  const std::string &header = fields[0];
  if (header.size() < 6) {
    return out;
  }
  // sentence type is the last 3 chars of header
  out.type = header.substr(3);
  fields.erase(fields.begin());

  // the rest are the data fields
  out.fields = std::move(fields);
  return out;
}

