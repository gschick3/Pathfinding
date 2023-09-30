#include "CSV.h"
#include <fstream>
#include <sstream>

std::vector<std::vector<std::string>> CSV::read(std::string path, char delimiter) {
    std::ifstream in;
    in.open(path);

    std::vector<std::vector<std::string>> data;
    std::vector<std::string> row;
    std::string line, word;

    while (!in.eof()) {
        row.clear();

        std::getline(in, line);

        std::stringstream ss(line);

        while (std::getline(ss, word, delimiter))
            row.push_back(word);
        data.push_back(row);
    }

    return data;
}