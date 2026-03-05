
#ifndef DASTS2_VERSION9_C_UTILS_H
#define DASTS2_VERSION9_C_UTILS_H

#include <iostream>
#include <vector>
#include <numeric> // std::iota
#include <algorithm>
// #include <glob.h> // glob(), globfree()
#include "regex"
#include "filesystem"

class Utils
{
public:
    template <typename T>
    static std::vector<size_t> sortIndices(const std::vector<T> &v, bool reversed = false)
    {

        // initialize original index locations
        std::vector<size_t> idx(v.size());
        iota(idx.begin(), idx.end(), 0);

        // sort indexes based on comparing values in v
        // using std::stable_sort instead of std::sort
        // to avoid unnecessary index re-orderings
        // when v contains elements of equal values
        stable_sort(idx.begin(), idx.end(),
                    [&v, &reversed](size_t i1, size_t i2)
                    {
                        if (reversed)
                        {
                            return v[i1] > v[i2];
                        }
                        return v[i1] < v[i2];
                    });

        return idx;
    }

    template <typename T>
    static std::vector<T> slice(std::vector<T> const &v, int m, int n)
    {
        // m: start index
        // n: end index (include)
        auto first = v.begin() + m;
        auto last = v.begin() + n + 1;

        std::vector<T> vec(first, last);
        return vec;
    }

    static std::vector<std::string> glob(const std::string &path, std::string name)
    {
        using namespace std;
        vector<string> ret;

        name = regex_replace(name, regex("\\."), "\\.");
        name = regex_replace(name, regex("\\*"), ".+?");
        name = path + name;
        std::regex pattern(name);
        for (const auto &entry : std::filesystem::directory_iterator(path))
        {
            std::string p = entry.path().string();
            if (std::regex_match(p.begin(), p.end(), pattern))
            {
                ret.push_back(p);
            }
        }
        return ret;
    }
};

#endif // DOOR2DOOR2_UTILS_H
