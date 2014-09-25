#ifndef ACTION_SERVER_UUID_
#define ACTION_SERVER_UUID_

#include <string>

namespace act
{

class UUID
{

public:

    bool operator<(const UUID& other) const { return id_ < other.id_; }

    bool operator==(const UUID& other) const { return id_ == other.id_; }

    // serialize vector to stream
    friend std::ostream& operator<< (std::ostream& out, const UUID& id) {
        out << id.id_;
        return out;
    }

    static UUID generate();

private:

    std::string id_;

};

}

#endif
