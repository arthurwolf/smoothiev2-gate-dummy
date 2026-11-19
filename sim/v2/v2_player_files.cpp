#include "v2_player_files.h"
#include <cerrno>
#include <cstdlib>
#include <limits.h>
#include <string>

namespace {
std::string sd_root;

std::string resolve(const char *path)
{
    if (!path) return {};
    const std::string name(path);
    if (name.compare(0, 4, "/sd/") != 0) return name;
    if (sd_root.empty()) return {};
    char resolved[PATH_MAX];
    if (!realpath((sd_root + name.substr(3)).c_str(), resolved)) return {};
    const std::string absolute(resolved);
    if (sd_root != "/" && (absolute.compare(0, sd_root.size(), sd_root) != 0 ||
                           absolute.size() <= sd_root.size() || absolute[sd_root.size()] != '/')) return {};
    return absolute;
}
}

bool sim_v2_set_sd_root(const char *path)
{
    char resolved[PATH_MAX];
    struct stat info;
    if (!path || !realpath(path, resolved) || stat(resolved, &info) || !S_ISDIR(info.st_mode)) return false;
    sd_root = resolved;
    return true;
}

bool sim_v2_sd_root_ready() { return !sd_root.empty(); }

FILE *sim_v2_player_fopen(const char *path, const char *mode)
{
    const auto resolved = resolve(path);
    if (resolved.empty()) { errno = ENOENT; return nullptr; }
    return std::fopen(resolved.c_str(), mode);
}

int sim_v2_player_stat(const char *path, struct stat *result)
{
    const auto resolved = resolve(path);
    if (resolved.empty()) { errno = ENOENT; return -1; }
    return stat(resolved.c_str(), result);
}
