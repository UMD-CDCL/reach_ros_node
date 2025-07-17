#include <regex>
#include <sstream>
#include "parser.hpp"

ParsedSentence parse_nmea_sentence(const std::string &sentence) {
  ParsedSentence out;
  // 1) Basic NMEA format: starts with $GP/GA/GN/GL, ends with *XX
  static const std::regex pattern(R"(^\$(?:GP|GA|GN|GL)[^*]*\*[0-9A-Fa-f]{2}$)");
  if (!std::regex_match(sentence, pattern)) {
    return out;
  }

  // 2) Strip checksum suffix
  auto star = sentence.find('*');
  std::string core = sentence.substr(0, star);

  // 3) Split on commas
  std::vector<std::string> fields;
  std::stringstream ss(core);
  std::string token;
  while (std::getline(ss, token, ',')) {
    fields.push_back(token);
  }
  if (fields.empty()) {
    return out;
  }

  // 4) Extract and remove the header (e.g. "$GPGGA")
  const std::string &header = fields[0];
  if (header.size() < 6) {
    return out;
  }
  // Sentence type is the last 3 chars of header
  out.type = header.substr(3);
  fields.erase(fields.begin());

  // 5) The rest are the data fields
  out.fields = std::move(fields);
  return out;
}

