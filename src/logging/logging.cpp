#include "logging.h"
#include "defines.h"

#include <algorithm>
#include <sstream>

using namespace std;

namespace logging
{
// trim from start (in place)
static inline void ltrim(string &s) {
    s.erase(s.begin(), find_if(s.begin(), s.end(), [](int ch) {
        return !isspace(ch);
    }));
}

// trim from end (in place)
static inline void rtrim(string &s) {
    s.erase(find_if(s.rbegin(), s.rend(), [](int ch) {
        return !isspace(ch);
    }).base(), s.end());
}

// trim from both ends (in place)
static inline void trim(string &s) {
    ltrim(s);
    rtrim(s);
}

vector<string> split_csv_line(string line) {
    stringstream liness(line);
    vector<string> line_data;
    while (!liness.eof()) {
        string tmp;
        getline(liness,tmp, ';');
        line_data.push_back(tmp);
    }
    return line_data;
}

map<string, int> read_head_map(string heads) {
    stringstream heads_ss(heads);
    int cnt=0;

    map<string, int> res;
    while (!heads_ss.eof()) {
        string tmp;
        getline(heads_ss,tmp, ';');
        trim(tmp);
        auto replay_str_pos =tmp.find("_replay");
        if (replay_str_pos!= string::npos)
            tmp = tmp.substr(0,replay_str_pos);
        auto insect_str_pos =tmp.find("_insect");
        if (insect_str_pos!= string::npos)
            tmp = tmp.substr(0,insect_str_pos);
        map<const string, int>::value_type head(tmp,cnt);
        res.insert(head);
        cnt++;
    }
    return res;
}
}
