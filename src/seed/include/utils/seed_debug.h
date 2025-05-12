#include <iostream>

//text colors using ansi syntax... for linux terminal!
namespace ansi{
/*       foreground background
black        30         40
red          31         41
green        32         42
yellow       33         43
blue         34         44
magenta      35         45
cyan         36         46
white        37         47
*/

//default style
const std::string end = "\033[0m";
//foreground colors
const std::string red     = "\033[31m";
const std::string green   = "\033[32m";
const std::string yellow  = "\033[33m";
const std::string blue    = "\033[34m";
const std::string magenta = "\033[35m";
const std::string cyan    = "\033[36m";
const std::string white   = "\033[37m";

//background colors
const std::string back_red       = "\033[41m";
const std::string back_green     = "\033[42m";
const std::string back_yellow    = "\033[43m";
const std::string back_blue      = "\033[44m";
const std::string back_magenta   = "\033[45m";
const std::string back_cyan      = "\033[46m";
const std::string back_white     = "\033[47m";
}

//convert string to number considering the std::locale::classic()
//  NOTE: the decimal are always separated by "." independently from the current system
//        this should be more reliable than atof and stod
inline double ston(std::string str){
    float number = 0.0f;
    std::istringstream istr(str);
    istr.imbue(std::locale::classic());
    istr >> number;
    return number;
}

//replace all instances of the substring "str" from the string "from" to the string "to"
inline void replaceAll(std::string& str, const std::string& from, const std::string& to) {
    if(from.empty())
        return;
    size_t start_pos = 0;
    while((start_pos = str.find(from, start_pos)) != std::string::npos) {
        str.replace(start_pos, from.length(), to);
        start_pos += to.length(); // In case 'to' contains 'from', like replacing 'x' with 'yx'
    }
}


//round a double to a specific number of digits
inline double roundec(double num_to_round, int numDigits){
    return round(num_to_round * powf(10, numDigits)) / powf(10, numDigits);
}

inline std::string stringToUpper(std::string s) {
    for (unsigned int l = 0; l < s.length(); l++) {
        s[l] = toupper(s[l]);
    }
    return s;
}