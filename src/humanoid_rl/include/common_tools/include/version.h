#ifndef VERSION_H
#define VERSION_H

#define HUMANOID_RL_MAJOR 0

#define HUMANOID_RL_MINOR 0

#define HUMANOID_RL_PATCH 0

#define HUMANOID_RL_BUILD 1

#define HUMANOID_RL_VERSION_STRING "0.0.0.1"

#ifdef __cplusplus
extern "C" {
#endif
//获取主版本号
static inline int get_version_major() { return HUMANOID_RL_MAJOR; }
//获取次版本号
static inline int get_version_minor() { return HUMANOID_RL_MINOR; }
//获取修订号
static inline int get_version_patch() { return HUMANOID_RL_PATCH; }
//获取构建号
static inline int get_version_build() { return HUMANOID_RL_BUILD; }
//获取版本号
static inline const char *get_version_string() { return HUMANOID_RL_VERSION_STRING; }

#ifdef __cplusplus
}
#endif
#endif // VERSION_H
