#include "Parser.h"

bool Parser::loadFile(const std::string& filename) {
    std::ifstream infile(filename);
    if (!infile.is_open()) return false;

    std::string line;
    std::string currentKey;
    std::string currentValue;
    bool inMultiline = false;

    while (std::getline(infile, line)) {
        auto commentPos = line.find('#');
        if (commentPos != std::string::npos)
            line = line.substr(0, commentPos);

        line = trim(line);
        if (line.empty()) continue;

        if (!inMultiline) {
            auto eqPos = line.find('=');
            if (eqPos == std::string::npos) continue;

            currentKey = trim(line.substr(0, eqPos));
            currentValue = trim(line.substr(eqPos + 1));

            if (currentValue.find('[') != std::string::npos &&
                currentValue.find(']') == std::string::npos) {
                // Starts a multiline value
                inMultiline = true;
                currentValue += " ";
            }
            else {
                // Single-line assignment
                data_[currentKey] = currentValue;
                currentKey.clear();
                currentValue.clear();
            }
        }
        else {
            currentValue += trim(line) + " ";
            if (line.find(']') != std::string::npos) {
                data_[currentKey] = currentValue;
                inMultiline = false;
                currentKey.clear();
                currentValue.clear();
            }
        }
    }

    return true;
}


bool Parser::get(const std::string& key, int& value) const {
    auto it = data_.find(key);
    if (it == data_.end()) return false;

    std::istringstream iss(it->second);
    return (iss >> value) ? true : false;
}


bool Parser::get(const std::string& key, double& value) const {
    auto it = data_.find(key);
    if (it == data_.end()) return false;

    std::istringstream iss(it->second);
    return (iss >> value) ? true : false;
}

bool Parser::get(const std::string& key, float& value) const {
    auto it = data_.find(key);
    if (it == data_.end()) return false;

    std::istringstream iss(it->second);
    return (iss >> value) ? true : false;
}

bool Parser::get(const std::string& key, bool& value) const {
    auto it = data_.find(key);
    if (it == data_.end()) return false;
    std::string val = it->second;
    std::transform(val.begin(), val.end(), val.begin(), ::tolower);
    value = (val == "true" || val == "1");
    return true;
}

bool Parser::get(const std::string& key, std::string& value) const {
    auto it = data_.find(key);
    if (it == data_.end()) return false;
    value = it->second;
    return true;
}

std::string Parser::trim(const std::string& s) {
    auto start = s.find_first_not_of(" \t\r\n");
    auto end = s.find_last_not_of(" \t\r\n");
    return (start == std::string::npos) ? "" : s.substr(start, end - start + 1);
}