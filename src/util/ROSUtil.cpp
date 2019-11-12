#include "ROSUtil.h"
#include <vector>

using namespace std;
using namespace cnoid;

namespace {

vector<char*> rosargs;

}

std::vector<char*>& cnoid::rosInitArguments()
{
    return rosargs;
}
