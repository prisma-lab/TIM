#pragma once

#include <cctype>
#include <stdexcept>
#include <string>
#include <vector>

namespace primitive_manager
{
struct Command
{
  std::string text;
  std::string name;
  std::vector<std::string> args;
};

// Parse SEED's atom or functor syntax, preserving nested arguments and quoted
// strings. Reject malformed input before it reaches a plugin.
inline Command parse_command(const std::string & input)
{
  Command result;
  char quote = 0;
  bool escaped = false;
  if (input.size() > 4096) {throw std::invalid_argument("Command is too long");}
  for (unsigned char ch : input) {
    if (quote) {
      result.text += ch;
      if (escaped) {escaped = false;}
      else if (ch == '\\') {escaped = true;}
      else if (ch == quote) {quote = 0;}
    } else if (ch == '\'' || ch == '"') {
      quote = ch;
      result.text += ch;
    } else if (!std::isspace(ch)) {result.text += ch;}
  }
  if (quote || result.text.empty()) {throw std::invalid_argument("Empty or unterminated command");}
  const auto opening = result.text.find('(');
  result.name = result.text.substr(0, opening);
  if (result.name.empty() || !(std::isalpha(static_cast<unsigned char>(result.name[0])) ||
    result.name[0] == '_')) {throw std::invalid_argument("Invalid command name");}
  for (unsigned char ch : result.name) {
    if (!std::isalnum(ch) && ch != '_') {throw std::invalid_argument("Invalid command name");}
  }
  if (opening == std::string::npos) {return result;}
  if (result.text.back() != ')') {throw std::invalid_argument("Unclosed command arguments");}
  std::string argument;
  std::vector<char> brackets;
  for (size_t i = opening + 1; i + 1 < result.text.size(); ++i) {
    char ch = result.text[i];
    if (quote) {
      argument += ch;
      if (escaped) {escaped = false;}
      else if (ch == '\\') {escaped = true;}
      else if (ch == quote) {quote = 0;}
      continue;
    }
    if (ch == '\'' || ch == '"') {quote = ch;}
    else if (ch == '(' || ch == '[') {brackets.push_back(ch);}
    else if (ch == ')' || ch == ']') {
      if (brackets.empty() || brackets.back() != (ch == ')' ? '(' : '[')) {
        throw std::invalid_argument("Mismatched command brackets");
      }
      brackets.pop_back();
    } else if (ch == ',' && brackets.empty()) {
      if (argument.empty()) {throw std::invalid_argument("Empty argument");}
      result.args.push_back(argument);
      argument.clear();
      continue;
    }
    argument += ch;
  }
  if (!brackets.empty() || quote) {throw std::invalid_argument("Unclosed argument");}
  if (!argument.empty()) {result.args.push_back(argument);}
  else if (!result.args.empty()) {throw std::invalid_argument("Empty final argument");}
  if (result.args.empty()) {result.text = result.name;}
  return result;
}
}  // namespace primitive_manager
