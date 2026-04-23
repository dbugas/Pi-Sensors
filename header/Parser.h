#pragma once

#include <string>
#include <unordered_map>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <Eigen/Dense>
#include <vector>
#include <regex>
#include <iostream>
#include <stdexcept>

class Parser {
    // This class is used to parse a configuration file and load variables into the program.
    // It supports various data types including integers, doubles, floats, booleans, strings, 1D/2D vectors, and Eigen matrices.
    // A 3x3 matrix can be specified in a format like: A = [1 2 3; 4 5 6; 7 8 9], A = [1, 2, 3; 4, 5, 6; 7, 8, 9].
    // While vectors can be specified as: v = [1 2 3], v = [1, 2, 3], v = 1 2 3, or v = 1, 2, 3, or a column vector v = 1; 2; 3.
    // The configuration file can contain single-line and multi-line values, with support for comments denoted by #.
	// Note, Eigen::vectorXd is a column vector that cannot be transposed, so if a single row is provided, it will be transposed to fit the column vector format.
public:
    Parser(const std::string filename) {
        loadFile(filename);
    }
    bool loadFile(const std::string& filename);

    template <typename T>
    void loadVar(const std::string& key, T& var) const {
        if (!get(key, var)) {
            std::cout << "Problem loading variable = " << key << std::endl;
            throw std::runtime_error("Variable Not Found!");
        }
        //std::cout << "variable " << key << " loaded = \n"<< var << std::endl;
    }


private:
    std::unordered_map<std::string, std::string> data_;
    static std::string trim(const std::string& s);

    bool get(const std::string& key, int& value) const;
    bool get(const std::string& key, double& value) const;
    bool get(const std::string& key, float& value) const;
    bool get(const std::string& key, bool& value) const;
    bool get(const std::string& key, std::string& value) const;

    template<typename Derived>
    bool get(const std::string& key, Eigen::MatrixBase<Derived>& value) const {
        using Scalar = typename Derived::Scalar;

        auto it = data_.find(key);
        if (it == data_.end()) return false;

        std::string originalStr = it->second;   // keep original for error messages
        std::string str = originalStr;

        // Remove outer brackets
        str.erase(std::remove_if(str.begin(), str.end(), [](char c) { return c == '[' || c == ']'; }), str.end());

        std::vector<std::vector<Scalar>> rows;
        std::stringstream ss(str);
        std::string rowStr;

        while (std::getline(ss, rowStr, ';')) {
            std::vector<Scalar> row;
            std::stringstream rowStream(rowStr);
            std::string valStr;

            while (std::getline(rowStream, valStr, ',')) {
                std::stringstream valStream(valStr);
                std::string number;
                while (valStream >> number) {
                    try {
                        row.push_back(static_cast<Scalar>(std::stod(number)));
                    }
                    catch (...) {
                        return false;
                    }
                }
            }
            if (!row.empty())
                rows.push_back(row);
        }

        if (rows.empty()) return false;

        size_t numInputRows = rows.size();
        size_t numInputCols = rows[0].size();

        for (const auto& r : rows)
            if (r.size() != numInputCols) return false;   // non-rectangular

        // ==================== STRICT VECTOR TYPE CHECKING ====================
        constexpr bool TargetIsColVector = (Derived::ColsAtCompileTime == 1 && Derived::RowsAtCompileTime != 1);
        constexpr bool TargetIsRowVector = (Derived::RowsAtCompileTime == 1 && Derived::ColsAtCompileTime != 1);
        constexpr bool isColDynamic = Derived::ColsAtCompileTime == Eigen::Dynamic;
        constexpr bool isRowDynamic = Derived::RowsAtCompileTime == Eigen::Dynamic;

        bool hasSemicolon = (originalStr.find(';') != std::string::npos);

        // Detect what the user actually wrote
        bool writtenAsRow = (numInputRows == 1 && numInputCols > 1 && !hasSemicolon);
        bool writtenAsColumn = (numInputCols == 1 && numInputRows > 1 && hasSemicolon);

        if (TargetIsColVector) {
            if (writtenAsRow) {
                throw std::runtime_error("Config error: Variable '" + key +
                    "' is declared as VectorXd (column vector) in code, "
                    "but written as horizontal row in config file. "
                    "Use semicolons for column vectors.");
            }
        }
        else if (TargetIsRowVector) {
            if (writtenAsColumn) {
                throw std::runtime_error("Config error: Variable '" + key +
                    "' is declared as RowVectorXd (row vector) in code, "
                    "but written as vertical column in config file. "
                    "Remove semicolons or write horizontally for row vectors.");
            }
        }

        // Auto-transpose only when it makes sense and is safe
        size_t finalRows = numInputRows;
        size_t finalCols = numInputCols;

        if (TargetIsColVector && numInputRows == 1 && numInputCols > 1) {
            finalRows = numInputCols;
            finalCols = 1;
        }
        else if (TargetIsRowVector && numInputCols == 1 && numInputRows > 1) {
            finalRows = 1;
            finalCols = numInputRows;
        }

        // Validate dimensions for fixed-size matrices
        if (!isRowDynamic && finalRows != static_cast<size_t>(Derived::RowsAtCompileTime)) return false;
        if (!isColDynamic && finalCols != static_cast<size_t>(Derived::ColsAtCompileTime)) return false;

        // Fill the Eigen object
        if constexpr (isRowDynamic || isColDynamic) {
            auto& val = const_cast<Eigen::MatrixBase<Derived>&>(value).derived();
            val.resize(finalRows, finalCols);
            for (size_t i = 0; i < finalRows; ++i) {
                for (size_t j = 0; j < finalCols; ++j) {
                    val(i, j) = (finalRows == numInputRows) ? rows[i][j]: rows[j][0];  // transposed case
                }
            }
        }
        else {
            for (size_t i = 0; i < finalRows; ++i)
                for (size_t j = 0; j < finalCols; ++j)
                    value(i, j) = (finalRows == numInputRows) ? rows[i][j] : rows[j][0];
        }

        return true;
    }

    template <typename T>
    bool get(const std::string& key, std::vector<std::vector<T>>& value) const {
        auto it = data_.find(key);
        if (it == data_.end()) return false;

        std::string str = it->second;

        // Remove outer brackets if present
        str.erase(std::remove_if(str.begin(), str.end(), [](char c) { return c == '[' || c == ']'; }), str.end());

        std::vector<std::vector<T>> rows;
        std::stringstream ss(str);
        std::string rowStr;

        while (std::getline(ss, rowStr, ';')) {
            std::vector<T> row;
            std::stringstream rowStream(rowStr);
            std::string valStr;

            while (std::getline(rowStream, valStr, ',')) {
                std::stringstream valStream(valStr);
                std::string item;
                while (valStream >> item) {
                    if constexpr (std::is_same_v<T, std::string>) {
                        row.push_back(item);                    // string case
                    }
                    else {
                        try {
                            row.push_back(static_cast<T>(std::stod(item)));
                        }
                        catch (...) {
                            return false;
                        }
                    }
                }
            }
            if (!row.empty()) {
                rows.push_back(std::move(row));
            }
        }

        if (rows.empty()) return false;

        value = std::move(rows);
        return true;
    }

    template <typename T>
    bool get(const std::string& key, std::vector<T>& value) const {
        auto it = data_.find(key);
        if (it == data_.end()) return false;

        std::string str = it->second;
        str.erase(std::remove_if(str.begin(), str.end(), [](char c) { return c == '[' || c == ']'; }), str.end());

        value.clear();
        std::stringstream ss(str);
        std::string token;

        while (std::getline(ss, token, ',')) {
            std::stringstream valStream(token);
            std::string item;
            while (valStream >> item) {
                if constexpr (std::is_same_v<T, std::string>) {
                    value.push_back(item);                    // string case
                }
                else {
                    try {
                        value.push_back(static_cast<T>(std::stod(item)));
                    }
                    catch (...) {
                        return false;
                    }
                }
            }
        }
        return !value.empty();
    }
};
