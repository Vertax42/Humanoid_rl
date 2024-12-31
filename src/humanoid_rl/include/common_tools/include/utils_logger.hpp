#ifndef UTILS_LOGGER_HPP
#define UTILS_LOGGER_HPP

#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <ostream>
#include <sstream>
#include <stdarg.h>
#include <stdio.h>
#include <string>
#include <thread>

// #ifdef __CUDACC__
// #include <cuda_runtime.h>
// #define CUDA_AVAILABLE true  
// #else
// #define CUDA_AVAILABLE false
// #endif

#include <utils_colorprint.hpp>
#include <utils_os_compatible.hpp>
#include <utils_timer.hpp>

/*
 * Logger utilities
 * Author: XinCheng Yang
 */

#define FILE_LOGGER_VERSION "V1.7"
#define FILE_LOGGER_VERSION_INFO "Add more tools, print out the hardware information in printf_program."

#ifndef printf_line
#define printf_line std::cout << __FILE__ << " " << __LINE__ << std::endl;
#endif

#define LOG_FILE_LINE(x)                                                                                               \
    *((x).get_ostream()) << __FILE__ << "   " << __LINE__ << endl;                                                     \
    (x).get_ostream()->flush();

#define LOG_FILE_LINE_AB(a, b)                                                                                         \
    *((a).get_ostream(b)) << __FILE__ << "   " << __LINE__ << endl;                                                    \
    (a).get_ostream()->flush();

#define LOG_FUNCTION_LINE(x)                                                                                           \
    *((x).get_ostream()) << __FUNCTION__ << "   " << __LINE__ << endl;                                                 \
    (x).get_ostream()->flush();

#define LOG_FUNCTION_LINE_AB(a, b)                                                                                     \
    *((a).get_ostream(b)) << __FUNCTION__ << "   " << __LINE__ << endl;                                                \
    (x).get_ostream()->flush();

#define ADD_SCREEN_PRINTF_OUT_METHOD                                                                                   \
    int m_if_verbose_screen_printf = 1;                                                                                \
    char *m_sceen_output_cstr = new char[10000]();                                                                     \
    std::string m_sceen_output_string = std::string(m_sceen_output_cstr);                                              \
    std::shared_ptr<std::stringstream> m_sceen_output_stringstream                                                     \
        = std::make_shared<std::stringstream>(m_sceen_output_string);                                                  \
    inline void screen_printf(const char *fmt, ...)                                                                    \
    {                                                                                                                  \
        if(m_if_verbose_screen_printf) return;                                                                         \
        va_list ap;                                                                                                    \
        va_start(ap, fmt);                                                                                             \
        if(common_tools::vasprintf(&m_sceen_output_cstr, fmt, ap) == 0)                                                \
        {                                                                                                              \
            return;                                                                                                    \
        }                                                                                                              \
        std::printf("%s", m_sceen_output_cstr);                                                                        \
    }                                                                                                                  \
    std::ostream *sreen_outstream()                                                                                    \
    {                                                                                                                  \
        if(m_if_verbose_screen_printf)                                                                                 \
        {                                                                                                              \
            return m_sceen_output_stringstream.get();                                                                  \
        } else                                                                                                         \
        {                                                                                                              \
            return &std::cout;                                                                                         \
        }                                                                                                              \
    }

#define screen_out (*sreen_outstream())
#define ENABLE_SCREEN_PRINTF                                                                                           \
    common_tools::Variable_restore_point<int> val_restore(&m_if_verbose_screen_printf,                                 \
                                                          0); // Enable printf in this scope.
#define DISABLE_SCREEN_PRINTF                                                                                          \
    common_tools::Variable_restore_point<int> val_restore(&m_if_verbose_screen_printf,                                 \
                                                          1); // Disable printf in this scope.

namespace common_tools // Commond tools
{
using namespace std;

inline void printf_software_version(const std::string &s)
{
    std::cout << ANSI_COLOR_GREEN_BOLD;
    std::cout << std::endl;
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cout << "Here is the software environment infos: " << std::endl;
    std::cout << "Software name            : " << ANSI_COLOR_RED_BOLD << s << ANSI_COLOR_GREEN_BOLD << std::endl;

#ifdef CIRCLE_DETECTION_MAJOR // Has Circle detection
    std::cout << "Circle_Detection version : " << CIRCLE_DETECTION_MAJOR << "." << CIRCLE_DETECTION_MINOR << "."
              << CIRCLE_DETECTION_PATCH << "." << CIRCLE_DETECTION_BUILD << std::endl;
#endif

// #ifdef __GNUC__ &&__GNUC_MINOR__ &&__GNUC_PATCHLEVEL__
#ifdef __GNUC_PATCHLEVEL__
    std::cout << "GCC version              : " << __GNUC__ << "." << __GNUC_MINOR__ << "." << __GNUC_PATCHLEVEL__
              << std::endl;
    ;
#endif

#ifdef QT_VERSION // Has Qt
    std::cout << "Qt version               : " << (QT_VERSION >> 16) << "." << ((QT_VERSION >> 8) & 0xFF) << "."
              << (QT_VERSION & 0xFF) << std::endl;
#endif
#ifdef BOOST_VERSION // Has Boost
    std::cout << "Boost version            : " << BOOST_VERSION / 100000 << "." << BOOST_VERSION / 100 % 1000 << "."
              << BOOST_VERSION % 100 << std::endl;
#endif

// #ifdef EIGEN_WORLD_VERSION &&EIGEN_MAJOR_VERSION &&EIGEN_MINOR_VERSION // Has Eigen
#ifdef EIGEN_MINOR_VERSION // Has Eigen
    std::cout << "Eigen version            : " << EIGEN_WORLD_VERSION << "." << EIGEN_MAJOR_VERSION << "."
              << EIGEN_MINOR_VERSION << std::endl;
#endif

#ifdef VTK_MAJOR_VERSION
    std::cout << "VTK version              : " << VTK_MAJOR_VERSION << "." << VTK_MINOR_VERSION << "."
              << VTK_BUILD_VERSION << std::endl;
#endif

// #ifdef PCL_MAJOR_VERSION && PCL_MINOR_VERSION && PCL_REVISION_VERSION // Has PCL
#ifdef PCL_VERSION
    std::cout << "PCL version              : " << PCL_MAJOR_VERSION << "." << PCL_MINOR_VERSION << "."
              << PCL_REVISION_VERSION << std::endl;
#endif

// #ifdef CV_MAJOR_VERSION &&CV_MINOR_VERSION &&CV_VERSION_REVISION // Has openCV
#ifdef CV_VERSION_REVISION // Has openCV
    std::cout << "OpenCV version           : " << CV_MAJOR_VERSION << "." << CV_MINOR_VERSION << "."
              << CV_VERSION_REVISION << std::endl;
#endif

// #ifdef CERES_VERSION_MAJOR &&CERES_VERSION_MINOR &&CERES_VERSION_PATCH // Has Ceres-solver
#ifdef CERES_VERSION_PATCH // Has Ceres-solver
    std::cout << "Ceres-solver version     : " << CERES_VERSION_MAJOR << "." << CERES_VERSION_MINOR << "."
              << CERES_VERSION_PATCH << std::endl;
#endif

// #ifdef CGAL_MAJOR_VERSION &&CGAL_MINOR_VERSION &&CGAL_BUGFIX_VERSION // didn't take effect...., any bugs here?
#ifdef CGAL_BUGFIX_VERSION // didn't take effect...., any bugs here?
    std::cout << "CGAL version             : " << CGAL_MAJOR_VERSION << "." << CGAL_MINOR_VERSION << "."
              << CGAL_BUGFIX_VERSION << std::endl;
#endif
#ifdef CGAL_VERSION // didn't take effect...., any bugs here?
    std::cout << "CGAL version             : " << CGAL_VERSION << std::endl;
#endif
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << ANSI_COLOR_RESET << std::endl;
}

template <typename T>
struct Variable_restore_point {
    T *m_targer_variable;
    T m_initial_value;

    Variable_restore_point(T *variable_ptr)
    {
        m_targer_variable = variable_ptr;
        m_initial_value = *m_targer_variable;
    }

    Variable_restore_point(T *variable_ptr, const T temp_val)
    {
        m_targer_variable = variable_ptr;
        m_initial_value = *m_targer_variable;
        *m_targer_variable = temp_val;
    }

    ~Variable_restore_point() { *m_targer_variable = m_initial_value; }
};

inline bool if_file_exist(const std::string &name)
{
    // Copy from:
    // https://stackoverflow.com/questions/12774207/fastest-way-to-check-if-a-file-exist-using-standard-c-c11-c
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
};

class File_logger {
public:
    std::map<string, std::ostream *> m_map_file_os;
    char m_temp_char[10000];
    string m_temp_string;
    std::shared_ptr<std::stringstream> m_temp_stringstream;
    string m_save_dir_name = string("./");
    std::shared_ptr<std::mutex> m_mutex_log;
    int m_if_verbose = 0;
    void release()
    {
        for(auto it = m_map_file_os.begin(); it != m_map_file_os.end(); it++)
        {
            it->second->flush();
            (it->second) = NULL;
            delete it->second;
        }
        m_map_file_os.clear();
    };

    ~File_logger() { release(); };

    void set_log_dir(string _dir_name)
    {
        release();
        m_save_dir_name = _dir_name;
        create_dir(m_save_dir_name);
        // mkdir(m_save_dir_name.c_str(), 0775);
    }

    File_logger(string _dir_name = string("/home/ziv/data/"))
    {
        m_mutex_log = std::make_shared<std::mutex>();
        set_log_dir(_dir_name);
        m_map_file_os.insert(std::pair<string, std::ostream *>("screen", &std::cout));
        m_temp_string.reserve(1e4);
        m_temp_stringstream = std::make_shared<std::stringstream>(m_temp_string);
    }

    string version()
    {
        std::stringstream ss;
        ss << "===== This is version of File_logger =====" << endl;
        ss << "Version      : " << FILE_LOGGER_VERSION << endl;
        ss << "Version info : " << FILE_LOGGER_VERSION_INFO << endl;
        ss << "Complie date : " << __DATE__ << "  " << __TIME__ << endl;
        ss << "=====           End                  =====" << endl;
        return string(ss.str());
    }

    void init(std::string _file_name, std::string prefix_name = string("log"), int mode = std::ios::out)
    {
        std::ofstream *ofs = new std::ofstream();
        sprintf(m_temp_char, "%s/%s_%s", m_save_dir_name.c_str(), prefix_name.c_str(), _file_name.c_str());
        ofs->open(m_temp_char, ios::out);

        if(ofs->is_open())
        {
            // cout << "Open " << _file_name << " successful." << endl;
            m_map_file_os.insert(std::pair<string, std::ostream *>(prefix_name, ofs));
        } else
        {
            cout << "Fail to open " << _file_name << endl;
            m_map_file_os.insert(std::pair<string, std::ostream *>(prefix_name, &std::cout));
        }
    };

    std::ostream *get_ostream(std::string prefix_name = string("log"))
    {
        if(m_if_verbose) return m_temp_stringstream.get();
        auto it = m_map_file_os.find(prefix_name);

        if(it != m_map_file_os.end())
        {
            return (it->second);
        } else // if no exit, create a new one.
        {
            init("tempadd.txt", prefix_name);
            return get_ostream(prefix_name);
        }
    }

    int printf(const char *fmt, ...)
    {
        if(m_if_verbose) return 0;
        std::unique_lock<std::mutex> lock(*m_mutex_log);
#ifdef _WIN32
        va_list ap;
        char *result = 0;
        va_start(ap, fmt);
        if(vasprintf(&result, fmt, ap) == 0)
        {
            return 0;
        }
        // cout << "Fmt = " << fmt <<endl;
        // cout << "Args" = << m_args <<endl;m_temp_char
        m_temp_string = string(result);
        // cout << m_temp_string;
        *(get_ostream()) << m_temp_string;
        (get_ostream("log"))->flush();
        return m_temp_string.length();
// return 0;
#else
        va_list ap;
        char *result = 0;
        va_start(ap, fmt);
        if(vasprintf(&result, fmt, ap) == 0)
        {
            return 0;
        }
        // cout << "Fmt = " << fmt <<endl;
        // cout << "Args" = << m_args <<endl;m_temp_char
        m_temp_string = string(result);
        // cout << m_temp_string;
        *(get_ostream()) << m_temp_string;
        (get_ostream("log"))->flush();
        return m_temp_string.length();
#endif
    }
};
}; // namespace common_tools

#ifdef _MSC_VER
#include <Shlobj.h>
#ifndef _USE_WINSDKOS
#define _USE_WINSDKOS
#include <VersionHelpers.h>
#endif
#else
#include <sys/utsname.h>
#ifdef __APPLE__
#include <sys/sysctl.h>
#else
#include <sys/sysinfo.h>
#endif
#include <pwd.h>
#include <unistd.h>
#endif

#ifndef MAX_PATH
#define MAX_PATH 260
#endif

namespace common_tools {
using namespace std;

// Manage setting/removing bit flags
template <typename TYPE>
class TFlags {
public:
    typedef TYPE Type;

public:
    inline TFlags() : flags(0) {}
    inline TFlags(const TFlags &rhs) : flags(rhs.flags) {}
    inline TFlags(Type f) : flags(f) {}
    inline bool isSet(Type aFlag) const { return (flags & aFlag) == aFlag; }
    inline bool isSet(Type aFlag, Type nF) const { return (flags & (aFlag | nF)) == aFlag; }
    inline bool isSetExclusive(Type aFlag) const { return flags == aFlag; }
    inline bool isAnySet(Type aFlag) const { return (flags & aFlag) != 0; }
    inline bool isAnySet(Type aFlag, Type nF) const
    {
        const Type m(flags & (aFlag | nF));
        return m != 0 && (m & nF) == 0;
    }
    inline bool isAnySetExclusive(Type aFlag) const { return (flags & aFlag) != 0 && (flags & ~aFlag) == 0; }
    inline void set(Type aFlag, bool bSet)
    {
        if(bSet)
            set(aFlag);
        else
            unset(aFlag);
    }
    inline void set(Type aFlag) { flags |= aFlag; }
    inline void unset(Type aFlag) { flags &= ~aFlag; }
    inline void flip(Type aFlag) { flags ^= aFlag; }
    inline void operator=(TFlags rhs) { flags = rhs.flags; }
    inline operator Type() const { return flags; }
    inline operator Type &() { return flags; }

protected:
    Type flags;
};
typedef class TFlags<uint32_t> Flags;

typedef struct CPUINFO_ARM_TYP {
    bool bNEON;           // NEON SIMD support
    bool bVFP;            // Vector Floating Point support
    bool bSVE;            // Scalable Vector Extension support
    uint8_t implementer;  // ARM implementer code (e.g., 0x41 for ARM Ltd.)
    uint8_t variant;      // Variant of the CPU core
    uint8_t architecture; // Architecture version (e.g., ARMv8.2)
    uint16_t part_number; // Part number of the CPU
    uint8_t revision;     // Revision number of the CPU
    char vendor[13];      // CPU vendor name (e.g., "ARM")
    char name[49];        // CPU model name (e.g., "Cortex-A76")
} CPUINFO_ARM;

typedef struct CPUINFO_TYP {
    bool bSSE;       // Streaming SIMD Extensions
    bool bSSE2;      // Streaming SIMD Extensions 2
    bool bSSE3;      // Streaming SIMD Extensions 3
    bool bSSE41;     // Streaming SIMD Extensions 4.1
    bool bSSE42;     // Streaming SIMD Extensions 4.2
    bool bAVX;       // Advanced Vector Extensions
    bool bFMA;       // Fused Multiply锟紸dd
    bool b3DNOW;     // 3DNow! (vendor independent)
    bool b3DNOWEX;   // 3DNow! (AMD specific extensions)
    bool bMMX;       // MMX support
    bool bMMXEX;     // MMX (AMD specific extensions)
    bool bEXT;       // extended features available
    char vendor[13]; // vendor name
    char name[49];   // CPU name
} CPUINFO;

using String = std::string;
// F U N C T I O N S ///////////////////////////////////////////////

// Flags   InitCPU();
// CPUINFO GetCPUInfo();
// bool    OSSupportsSSE();
// bool    OSSupportsAVX();

#define PATH_SEPARATOR '/'
#define PATH_SEPARATOR_STR "/"
#define REVERSE_PATH_SEPARATOR '\\'

static String &trimUnifySlash(String &path)
{
    String::size_type start = 1;
    while((start = path.find(PATH_SEPARATOR, start)) != String::npos)
        if(path[start - 1] == PATH_SEPARATOR)
            path.erase(start, 1);
        else
            ++start;
    return path;
}
static String &ensureUnifySlash(String &path)
{
    String::size_type start = 0;
    while((start = path.find(REVERSE_PATH_SEPARATOR, start)) != String::npos)
        path[start] = PATH_SEPARATOR;
    return trimUnifySlash(path);
}

static String formatBytes(int64_t aBytes)
{
    char buff[100];
    if(aBytes < (int64_t)1024)
    {
        // return String::FormatString( "%dB", ( uint32_t ) aBytes & 0xffffffff );
        sprintf(buff, "%dB", (uint32_t)aBytes & 0xffffffff);
    } else if(aBytes < (int64_t)1024 * 1024)
    {
        // return String::FormatString( "%.02fKB", ( double ) aBytes / ( 1024.0 ) );
        sprintf(buff, "%.02fKB", (double)aBytes / (1024.0));
    } else if(aBytes < (int64_t)1024 * 1024 * 1024)
    {
        // return String::FormatString( "%.02fMB", ( double ) aBytes / ( 1024.0 * 1024.0 ) );
        sprintf(buff, "%.02fMB", (double)aBytes / (1024.0 * 1024.0));
    } else if(aBytes < (int64_t)1024 * 1024 * 1024 * 1024)
    {
        // return String::FormatString( "%.02fGB", ( double ) aBytes / ( 1024.0 * 1024.0 * 1024.0 ) );
        sprintf(buff, "%.02fGB", (double)aBytes / (1024.0 * 1024.0 * 1024.0));
    } else
    {
        // return String::FormatString( "%.02fTB", ( double ) aBytes / ( 1024.0 * 1024.0 * 1024.0 * 1024.0 ) );
        sprintf(buff, "%.02fTB", (double)aBytes / (1024.0 * 1024.0 * 1024.0 * 1024.0));
    }
    std::string res_string = buff;
    return res_string;
}

inline String get_home_folder()
{
#ifdef _MSC_VER
    TCHAR homedir[MAX_PATH];
    if(SHGetSpecialFolderPath(0, homedir, CSIDL_PROFILE, TRUE) != TRUE) return String();
#else
    const char *homedir;
    if((homedir = getenv("HOME")) == NULL) homedir = getpwuid(getuid())->pw_dir;
#endif // _MSC_VER
    String dir(String(homedir) + PATH_SEPARATOR);
    return ensureUnifySlash(dir);
}

inline String get_application_folder()
{
#ifdef _MSC_VER
    TCHAR appdir[MAX_PATH];
    if(SHGetSpecialFolderPath(0, appdir, CSIDL_APPDATA, TRUE) != TRUE) return String();
    String dir(String(appdir) + PATH_SEPARATOR);
#else
    const char *homedir;
    if((homedir = getenv("HOME")) == NULL) homedir = getpwuid(getuid())->pw_dir;
    String dir(String(homedir) + PATH_SEPARATOR + String(".config") + PATH_SEPARATOR);
#endif // _MSC_VER
    return ensureUnifySlash(dir);
}

#ifndef __LOGGER_HPP__
#define __LOGGER_HPP__
#include "utils_colorprint.hpp"
#include "utils_os_compatible.hpp"
#include "utils_timer.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <ostream>
#include <sstream>
#include <stdarg.h> //need for such like printf(...)
#include <stdio.h>
#include <string>
#include <thread>
// #define FILE_LOGGER_VERSION      "V1.0"
// #define FILE_LOGGER_VERSION_INFO "First version"

//#define FILE_LOGGER_VERSION "V1.1"
//#define FILE_LOGGER_VERSION_INFO "Add macro, make logger more easy to call"

//#define FILE_LOGGER_VERSION "V1.2"
//#define FILE_LOGGER_VERSION_INFO "Compatible with windows."

//#define FILE_LOGGER_VERSION "V1.3"
//#define FILE_LOGGER_VERSION_INFO "Support verbose"

// #define FILE_LOGGER_VERSION "V1.4"
// #define FILE_LOGGER_VERSION_INFO "Add macro code block, enable turn off the printf quickly"

// #define FILE_LOGGER_VERSION "V1.5"
// #define FILE_LOGGER_VERSION_INFO "Add variable restore block, enable turn off/on screen prinf in local block scope."

// #define FILE_LOGGER_VERSION "V1.6"
// #define FILE_LOGGER_VERSION_INFO "Fix discontruction bugs."

#define FILE_LOGGER_VERSION "V1.7"
#define FILE_LOGGER_VERSION_INFO "Add more tools, print out the hardware information in printf_program."

#ifndef printf_line
#define printf_line std::cout << __FILE__ << " " << __LINE__ << std::endl;
#endif

#define printf_program(a)                                                                                              \
    std::cout << ANSI_COLOR_YELLOW_BOLD;                                                                               \
    std::cout << "=============================================================" << std::endl;                         \
    std::cout << "App name   : " << ANSI_COLOR_RED_BOLD << a << ANSI_COLOR_YELLOW_BOLD << std::endl;                   \
    std::cout << "Build date : " << __DATE__ << "  " << __TIME__ << std::endl;                                         \
    std::cout << "CPU infos  : " << common_tools::get_cpu_info() << std::endl;                                         \
    std::cout << "RAM infos  : " << common_tools::get_RAM_info() << std::endl;                                         \
    std::cout << "OS  infos  : " << common_tools::get_OS_info() << std::endl;                                          \
    std::cout << "Home dir   : " << common_tools::get_home_folder() << std::endl;                                      \
    std::cout << "Current dir: " << common_tools::get_current_folder() << std::endl;                                   \
    std::cout << "Date mow   : " << common_tools::get_current_date_time_str() << std::endl;                            \
    std::cout << "=============================================================" << ANSI_COLOR_RESET << std::endl;


#define LOG_FILE_LINE(x)                                                                                               \
    *((x).get_ostream()) << __FILE__ << "   " << __LINE__ << endl;                                                     \
    (x).get_ostream()->flush();

#define LOG_FILE_LINE_AB(a, b)                                                                                         \
    *((a).get_ostream(b)) << __FILE__ << "   " << __LINE__ << endl;                                                    \
    (a).get_ostream()->flush();

#define LOG_FUNCTION_LINE(x)                                                                                           \
    *((x).get_ostream()) << __FUNCTION__ << "   " << __LINE__ << endl;                                                 \
    (x).get_ostream()->flush();

#define LOG_FUNCTION_LINE_AB(a, b)                                                                                     \
    *((a).get_ostream(b)) << __FUNCTION__ << "   " << __LINE__ << endl;                                                \
    (x).get_ostream()->flush();

#define ADD_SCREEN_PRINTF_OUT_METHOD                                                                                   \
    int m_if_verbose_screen_printf = 1;                                                                                \
    char *m_sceen_output_cstr = new char[10000]();                                                                     \
    std::string m_sceen_output_string = std::string(m_sceen_output_cstr);                                              \
    std::shared_ptr<std::stringstream> m_sceen_output_stringstream                                                     \
        = std::make_shared<std::stringstream>(m_sceen_output_string);                                                  \
    inline void screen_printf(const char *fmt, ...)                                                                    \
    {                                                                                                                  \
        if(m_if_verbose_screen_printf) return;                                                                         \
        va_list ap;                                                                                                    \
        va_start(ap, fmt);                                                                                             \
        if(common_tools::vasprintf(&m_sceen_output_cstr, fmt, ap) == 0)                                                \
        {                                                                                                              \
            return;                                                                                                    \
        }                                                                                                              \
        std::printf("%s", m_sceen_output_cstr);                                                                        \
    }                                                                                                                  \
    std::ostream *sreen_outstream()                                                                                    \
    {                                                                                                                  \
        if(m_if_verbose_screen_printf)                                                                                 \
        {                                                                                                              \
            return m_sceen_output_stringstream.get();                                                                  \
        } else                                                                                                         \
        {                                                                                                              \
            return &std::cout;                                                                                         \
        }                                                                                                              \
    }

#define screen_out (*sreen_outstream())
#define ENABLE_SCREEN_PRINTF                                                                                           \
    common_tools::Variable_restore_point<int> val_restore(&m_if_verbose_screen_printf,                                 \
                                                          0); // Enable printf in this scope.
#define DISABLE_SCREEN_PRINTF                                                                                          \
    common_tools::Variable_restore_point<int> val_restore(&m_if_verbose_screen_printf,                                 \
                                                          1); // Disable printf in this scope.

namespace common_tools // Commond tools
{
using namespace std;

template <typename T>
struct Variable_restore_point {
    T *m_targer_variable;
    T m_initial_value;

    Variable_restore_point(T *variable_ptr)
    {
        m_targer_variable = variable_ptr;
        m_initial_value = *m_targer_variable;
    }

    Variable_restore_point(T *variable_ptr, const T temp_val)
    {
        m_targer_variable = variable_ptr;
        m_initial_value = *m_targer_variable;
        *m_targer_variable = temp_val;
    }

    ~Variable_restore_point() { *m_targer_variable = m_initial_value; }
};

inline bool if_file_exist(const std::string &name)
{
    // Copy from:
    // https://stackoverflow.com/questions/12774207/fastest-way-to-check-if-a-file-exist-using-standard-c-c11-c
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
};

class File_logger {
public:
    std::map<string, std::ostream *> m_map_file_os;
    char m_temp_char[10000];
    string m_temp_string;
    std::shared_ptr<std::stringstream> m_temp_stringstream;
    string m_save_dir_name = string("./");
    std::shared_ptr<std::mutex> m_mutex_log;
    int m_if_verbose = 0;
    void release()
    {
        for(auto it = m_map_file_os.begin(); it != m_map_file_os.end(); it++)
        {
            it->second->flush();
            (it->second) = NULL;
            delete it->second;
        }
        m_map_file_os.clear();
    };

    ~File_logger() { release(); };

    void set_log_dir(string _dir_name)
    {
        release();
        m_save_dir_name = _dir_name;
        create_dir(m_save_dir_name);
        // mkdir(m_save_dir_name.c_str(), 0775);
    }

    File_logger(string _dir_name = string("/home/ziv/data/"))
    {
        m_mutex_log = std::make_shared<std::mutex>();
        set_log_dir(_dir_name);
        m_map_file_os.insert(std::pair<string, std::ostream *>("screen", &std::cout));
        m_temp_string.reserve(1e4);
        m_temp_stringstream = std::make_shared<std::stringstream>(m_temp_string);
    }

    string version()
    {
        std::stringstream ss;
        ss << "===== This is version of File_logger =====" << endl;
        ss << "Version      : " << FILE_LOGGER_VERSION << endl;
        ss << "Version info : " << FILE_LOGGER_VERSION_INFO << endl;
        ss << "Complie date : " << __DATE__ << "  " << __TIME__ << endl;
        ss << "=====           End                  =====" << endl;
        return string(ss.str());
    }

    void init(std::string _file_name, std::string prefix_name = string("log"), int mode = std::ios::out)
    {
        std::ofstream *ofs = new std::ofstream();
        sprintf(m_temp_char, "%s/%s_%s", m_save_dir_name.c_str(), prefix_name.c_str(), _file_name.c_str());
        ofs->open(m_temp_char, ios::out);

        if(ofs->is_open())
        {
            // cout << "Open " << _file_name << " successful." << endl;
            m_map_file_os.insert(std::pair<string, std::ostream *>(prefix_name, ofs));
        } else
        {
            cout << "Fail to open " << _file_name << endl;
            m_map_file_os.insert(std::pair<string, std::ostream *>(prefix_name, &std::cout));
        }
    };

    std::ostream *get_ostream(std::string prefix_name = string("log"))
    {
        if(m_if_verbose) return m_temp_stringstream.get();
        auto it = m_map_file_os.find(prefix_name);

        if(it != m_map_file_os.end())
        {
            return (it->second);
        } else // if no exit, create a new one.
        {
            init("tempadd.txt", prefix_name);
            return get_ostream(prefix_name);
        }
    }

    int printf(const char *fmt, ...)
    {
        if(m_if_verbose) return 0;
        std::unique_lock<std::mutex> lock(*m_mutex_log);
#ifdef _WIN32
        va_list ap;
        char *result = 0;
        va_start(ap, fmt);
        if(vasprintf(&result, fmt, ap) == 0)
        {
            return 0;
        }
        // cout << "Fmt = " << fmt <<endl;
        // cout << "Args" = << m_args <<endl;m_temp_char
        m_temp_string = string(result);
        // cout << m_temp_string;
        *(get_ostream()) << m_temp_string;
        (get_ostream("log"))->flush();
        return m_temp_string.length();
// return 0;
#else
        va_list ap;
        char *result = 0;
        va_start(ap, fmt);
        if(vasprintf(&result, fmt, ap) == 0)
        {
            return 0;
        }
        // cout << "Fmt = " << fmt <<endl;
        // cout << "Args" = << m_args <<endl;m_temp_char
        m_temp_string = string(result);
        // cout << m_temp_string;
        *(get_ostream()) << m_temp_string;
        (get_ostream("log"))->flush();
        return m_temp_string.length();
#endif
    }
};
}; // namespace common_tools

#ifdef _MSC_VER
#include <Shlobj.h>
#ifndef _USE_WINSDKOS
#define _USE_WINSDKOS
#include <VersionHelpers.h>
#endif
#else
#include <sys/utsname.h>
#ifdef __APPLE__
#include <sys/sysctl.h>
#else
#include <sys/sysinfo.h>
#endif
#include <pwd.h>
#include <unistd.h>
#endif

#ifndef MAX_PATH
#define MAX_PATH 260
#endif

namespace common_tools {
using namespace std;

// Manage setting/removing bit flags
template <typename TYPE>
class TFlags {
public:
    typedef TYPE Type;

public:
    inline TFlags() : flags(0) {}
    inline TFlags(const TFlags &rhs) : flags(rhs.flags) {}
    inline TFlags(Type f) : flags(f) {}
    inline bool isSet(Type aFlag) const { return (flags & aFlag) == aFlag; }
    inline bool isSet(Type aFlag, Type nF) const { return (flags & (aFlag | nF)) == aFlag; }
    inline bool isSetExclusive(Type aFlag) const { return flags == aFlag; }
    inline bool isAnySet(Type aFlag) const { return (flags & aFlag) != 0; }
    inline bool isAnySet(Type aFlag, Type nF) const
    {
        const Type m(flags & (aFlag | nF));
        return m != 0 && (m & nF) == 0;
    }
    inline bool isAnySetExclusive(Type aFlag) const { return (flags & aFlag) != 0 && (flags & ~aFlag) == 0; }
    inline void set(Type aFlag, bool bSet)
    {
        if(bSet)
            set(aFlag);
        else
            unset(aFlag);
    }
    inline void set(Type aFlag) { flags |= aFlag; }
    inline void unset(Type aFlag) { flags &= ~aFlag; }
    inline void flip(Type aFlag) { flags ^= aFlag; }
    inline void operator=(TFlags rhs) { flags = rhs.flags; }
    inline operator Type() const { return flags; }
    inline operator Type &() { return flags; }

protected:
    Type flags;
};
typedef class TFlags<uint32_t> Flags;

typedef struct CPUINFO_TYP {
    bool bSSE;       // Streaming SIMD Extensions
    bool bSSE2;      // Streaming SIMD Extensions 2
    bool bSSE3;      // Streaming SIMD Extensions 3
    bool bSSE41;     // Streaming SIMD Extensions 4.1
    bool bSSE42;     // Streaming SIMD Extensions 4.2
    bool bAVX;       // Advanced Vector Extensions
    bool bFMA;       // Fused Multiply锟紸dd
    bool b3DNOW;     // 3DNow! (vendor independent)
    bool b3DNOWEX;   // 3DNow! (AMD specific extensions)
    bool bMMX;       // MMX support
    bool bMMXEX;     // MMX (AMD specific extensions)
    bool bEXT;       // extended features available
    char vendor[13]; // vendor name
    char name[49];   // CPU name
} CPUINFO;
using String = std::string;
// F U N C T I O N S ///////////////////////////////////////////////

// Flags   InitCPU();
// CPUINFO GetCPUInfo();
// bool    OSSupportsSSE();
// bool    OSSupportsAVX();

#define PATH_SEPARATOR '/'
#define PATH_SEPARATOR_STR "/"
#define REVERSE_PATH_SEPARATOR '\\'

static String &trimUnifySlash(String &path)
{
    String::size_type start = 1;
    while((start = path.find(PATH_SEPARATOR, start)) != String::npos)
        if(path[start - 1] == PATH_SEPARATOR)
            path.erase(start, 1);
        else
            ++start;
    return path;
}
static String &ensureUnifySlash(String &path)
{
    String::size_type start = 0;
    while((start = path.find(REVERSE_PATH_SEPARATOR, start)) != String::npos)
        path[start] = PATH_SEPARATOR;
    return trimUnifySlash(path);
}

static String formatBytes(int64_t aBytes)
{
    char buff[100];
    if(aBytes < (int64_t)1024)
    {
        // return String::FormatString( "%dB", ( uint32_t ) aBytes & 0xffffffff );
        sprintf(buff, "%dB", (uint32_t)aBytes & 0xffffffff);
    } else if(aBytes < (int64_t)1024 * 1024)
    {
        // return String::FormatString( "%.02fKB", ( double ) aBytes / ( 1024.0 ) );
        sprintf(buff, "%.02fKB", (double)aBytes / (1024.0));
    } else if(aBytes < (int64_t)1024 * 1024 * 1024)
    {
        // return String::FormatString( "%.02fMB", ( double ) aBytes / ( 1024.0 * 1024.0 ) );
        sprintf(buff, "%.02fMB", (double)aBytes / (1024.0 * 1024.0));
    } else if(aBytes < (int64_t)1024 * 1024 * 1024 * 1024)
    {
        // return String::FormatString( "%.02fGB", ( double ) aBytes / ( 1024.0 * 1024.0 * 1024.0 ) );
        sprintf(buff, "%.02fGB", (double)aBytes / (1024.0 * 1024.0 * 1024.0));
    } else
    {
        // return String::FormatString( "%.02fTB", ( double ) aBytes / ( 1024.0 * 1024.0 * 1024.0 * 1024.0 ) );
        sprintf(buff, "%.02fTB", (double)aBytes / (1024.0 * 1024.0 * 1024.0 * 1024.0));
    }
    std::string res_string = buff;
    return res_string;
}

inline String get_home_folder()
{
#ifdef _MSC_VER
    TCHAR homedir[MAX_PATH];
    if(SHGetSpecialFolderPath(0, homedir, CSIDL_PROFILE, TRUE) != TRUE) return String();
#else
    const char *homedir;
    if((homedir = getenv("HOME")) == NULL) homedir = getpwuid(getuid())->pw_dir;
#endif // _MSC_VER
    String dir(String(homedir) + PATH_SEPARATOR);
    return ensureUnifySlash(dir);
}

inline String get_application_folder()
{
#ifdef _MSC_VER
    TCHAR appdir[MAX_PATH];
    if(SHGetSpecialFolderPath(0, appdir, CSIDL_APPDATA, TRUE) != TRUE) return String();
    String dir(String(appdir) + PATH_SEPARATOR);
#else
    const char *homedir;
    if((homedir = getenv("HOME")) == NULL) homedir = getpwuid(getuid())->pw_dir;
    String dir(String(homedir) + PATH_SEPARATOR + String(".config") + PATH_SEPARATOR);
#endif // _MSC_VER
    return ensureUnifySlash(dir);
}

inline String get_current_folder()
{
    char pathname[MAX_PATH + 1];
#ifdef _MSC_VER
    if(!GetCurrentDirectory(MAX_PATH, pathname))
#else  // _MSC_VER
    if(!getcwd(pathname, MAX_PATH))
#endif // _MSC_VER
        return String();
    String dir(String(pathname) + PATH_SEPARATOR);
    return ensureUnifySlash(dir);
}

#ifdef _MSC_VER
#include <intrin.h>
inline void CPUID(int CPUInfo[4], int level) { __cpuid(CPUInfo, level); }
#elif defined(__x86_64__)
#include <cpuid.h>
inline void CPUID(int CPUInfo[4], int level)
{
    unsigned *p((unsigned *)CPUInfo);
    __get_cpuid((unsigned &)level, p + 0, p + 1, p + 2, p + 3);
}
#elif defined(__aarch64__)

// 内联函数：读取 MIDR_EL1 系统寄存器
inline uint64_t read_midr_el1()
{
    uint64_t midr_el1;
    asm volatile("mrs %0, MIDR_EL1" : "=r"(midr_el1));
    return midr_el1;
}

// 内联函数：读取 ID_AA64PFR0_EL1 系统寄存器
inline uint64_t read_id_aa64pfr0_el1()
{
    uint64_t id_aa64pfr0_el1;
    asm volatile("mrs %0, ID_AA64PFR0_EL1" : "=r"(id_aa64pfr0_el1));
    return id_aa64pfr0_el1;
}

// 内联函数：填充 CPUINFO_ARM 结构体
inline void populate_arm_cpu_info(CPUINFO_ARM &cpu_info)
{
    uint64_t midr_el1 = read_midr_el1();
    uint64_t id_aa64pfr0_el1 = read_id_aa64pfr0_el1();

    // 解析 MIDR_EL1 的字段
    cpu_info.implementer = (midr_el1 >> 24) & 0xFF; // [31:24]
    cpu_info.variant = (midr_el1 >> 20) & 0xF;      // [23:20]
    cpu_info.architecture = (midr_el1 >> 16) & 0xF; // [19:16]
    cpu_info.part_number = (midr_el1 >> 4) & 0xFFF; // [15:4]
    cpu_info.revision = midr_el1 & 0xF;             // [3:0]

    switch(cpu_info.implementer)
    {
    case 0x41:
        strncpy(cpu_info.vendor, "ARM", sizeof(cpu_info.vendor));
        break;
    case 0x42:
        strncpy(cpu_info.vendor, "Broadcom", sizeof(cpu_info.vendor));
        break;
    case 0x43:
        strncpy(cpu_info.vendor, "Cavium", sizeof(cpu_info.vendor));
        break;
    case 0x44:
        strncpy(cpu_info.vendor, "Digital Equipment Corporation", sizeof(cpu_info.vendor));
        break;
    case 0x46:
        strncpy(cpu_info.vendor, "Fujitsu", sizeof(cpu_info.vendor));
        break;
    case 0x49:
        strncpy(cpu_info.vendor, "Infineon Technologies", sizeof(cpu_info.vendor));
        break;
    case 0x4D:
        strncpy(cpu_info.vendor, "Motorola", sizeof(cpu_info.vendor));
        break;
    case 0x4E:
        strncpy(cpu_info.vendor, "NVIDIA", sizeof(cpu_info.vendor));
        break;
    case 0x50:
        strncpy(cpu_info.vendor, "Applied Micro", sizeof(cpu_info.vendor));
        break;
    case 0x51:
        strncpy(cpu_info.vendor, "Qualcomm", sizeof(cpu_info.vendor));
        break;
    case 0x56:
        strncpy(cpu_info.vendor, "Marvell", sizeof(cpu_info.vendor));
        break;
    case 0x69:
        strncpy(cpu_info.vendor, "Intel", sizeof(cpu_info.vendor));
        break;
    default:
        strncpy(cpu_info.vendor, "ImpUnknown", sizeof(cpu_info.vendor));
        break;
    }

    // 根据部件号设置 CPU 名称（需要查表）
    switch(cpu_info.part_number)
    {
    case 0x810:
        strncpy(cpu_info.name, "ARM810", sizeof(cpu_info.name));
        break;
    case 0xc05:
        strncpy(cpu_info.name, "Cortex-A5", sizeof(cpu_info.name));
        break;
    case 0xC07:
        strncpy(cpu_info.name, "Cortex-A7", sizeof(cpu_info.name));
        break;
    case 0xc08:
        strncpy(cpu_info.name, "Cortex-A8", sizeof(cpu_info.name));
        break;
    case 0xC09:
        strncpy(cpu_info.name, "Cortex-A9", sizeof(cpu_info.name));
        break;
    case 0xC0d:
        strncpy(cpu_info.name, "Cortex-A17", sizeof(cpu_info.name));
        break;
    case 0xC0f:
        strncpy(cpu_info.name, "Cortex-A15", sizeof(cpu_info.name));
        break;
    case 0xC20:
        strncpy(cpu_info.name, "Cortex-M0", sizeof(cpu_info.name));
        break;
    case 0xC23:
        strncpy(cpu_info.name, "Cortex-M3", sizeof(cpu_info.name));
        break;
    case 0xC24:
        strncpy(cpu_info.name, "Cortex-M4", sizeof(cpu_info.name));
        break;
    case 0xC27:
        strncpy(cpu_info.name, "Cortex-M7", sizeof(cpu_info.name));
        break;
    case 0xC60:
        strncpy(cpu_info.name, "Cortex-R0", sizeof(cpu_info.name));
        break;
    case 0xD01:
        strncpy(cpu_info.name, "Cortex-A32", sizeof(cpu_info.name));
        break;
    case 0xD03:
        strncpy(cpu_info.name, "Cortex-A53", sizeof(cpu_info.name));
        break;
    case 0xD04:
        strncpy(cpu_info.name, "Cortex-A35", sizeof(cpu_info.name));
        break;
    case 0xD05:
        strncpy(cpu_info.name, "Cortex-A55", sizeof(cpu_info.name));
        break;
    case 0xD07:
        strncpy(cpu_info.name, "Cortex-A57", sizeof(cpu_info.name));
        break;
    case 0xD08:
        strncpy(cpu_info.name, "Cortex-A72", sizeof(cpu_info.name));
        break;
    case 0xD09:
        strncpy(cpu_info.name, "Cortex-A73", sizeof(cpu_info.name));
        break;
    case 0xD0A:
        strncpy(cpu_info.name, "Cortex-A75", sizeof(cpu_info.name));
        break;
    case 0xD0D:
        strncpy(cpu_info.name, "Cortex-A77", sizeof(cpu_info.name));
        break;
    case 0xD13:
        strncpy(cpu_info.name, "Cortex-R52", sizeof(cpu_info.name));
        break;
    case 0xD20:
        strncpy(cpu_info.name, "Cortex-M23", sizeof(cpu_info.name));
        break;
    case 0xD42:
        strncpy(cpu_info.name, "Cortex-A78AE", sizeof(cpu_info.name));
        break;
    case 0xD4A:
        strncpy(cpu_info.name, "Cortex-X2", sizeof(cpu_info.name));
        break;
    case 0xD4B:
        strncpy(cpu_info.name, "Cortex-A710", sizeof(cpu_info.name));
        break;
    case 0xD4C:
        strncpy(cpu_info.name, "Cortex-A510", sizeof(cpu_info.name));
        break;
    case 0xD40:
        strncpy(cpu_info.name, "Neoverse-N1", sizeof(cpu_info.name));
        break;
    case 0xD49:
        strncpy(cpu_info.name, "Neoverse-V1", sizeof(cpu_info.name));
        break;
    case 0xD4D:
        strncpy(cpu_info.name, "Neoverse-N2", sizeof(cpu_info.name));
        break;
    case 0xD4E:
        strncpy(cpu_info.name, "Neoverse-V2", sizeof(cpu_info.name));
        break;
    default:
        strncpy(cpu_info.name, "Unknown", sizeof(cpu_info.name));
        break;
    }

    // 检查扩展功能支持
    cpu_info.bNEON = true;                                // ARMv8-A 默认支持 NEON
    cpu_info.bVFP = true;                                 // ARMv8-A 默认支持 VFP
    cpu_info.bSVE = ((id_aa64pfr0_el1 >> 32) & 0xF) == 1; // SVE 位 [35:32]
}

inline std::string get_arm_cpu_info()
{
    CPUINFO_ARM cpu_info = {};
    populate_arm_cpu_info(cpu_info);

    std::string cpu = cpu_info.name; // CPU 名称

    // 添加实现者信息
    cpu += " (" + std::string(cpu_info.vendor) + ")";

    // 添加架构版本
    cpu += " Arch v" + std::to_string(cpu_info.architecture);

    // 添加扩展特性
    if(cpu_info.bSVE) cpu += " SVE";
    if(cpu_info.bNEON) cpu += " NEON";
    if(cpu_info.bVFP) cpu += " VFP";

    return cpu;
}
#else
#error "Unsupported platform for CPUID or CPU information retrieval"
#endif

#ifdef __x86_64__
inline CPUINFO GetCPUInfo_()
{
    CPUINFO info;

    // set all values to 0 (false)
    memset(&info, 0, sizeof(CPUINFO));

    int CPUInfo[4];

    // CPUID with an InfoType argument of 0 returns the number of
    // valid Ids in CPUInfo[0] and the CPU identification string in
    // the other three array elements. The CPU identification string is
    // not in linear order. The code below arranges the information
    // in a human readable form.
    CPUID(CPUInfo, 0);
    *((int *)info.vendor) = CPUInfo[1];
    *((int *)(info.vendor + 4)) = CPUInfo[3];
    *((int *)(info.vendor + 8)) = CPUInfo[2];

    // Interpret CPU feature information.
    CPUID(CPUInfo, 1);
    info.bMMX = (CPUInfo[3] & 0x800000) != 0;            // test bit 23 for MMX
    info.bSSE = (CPUInfo[3] & 0x2000000) != 0;           // test bit 25 for SSE
    info.bSSE2 = (CPUInfo[3] & 0x4000000) != 0;          // test bit 26 for SSE2
    info.bSSE3 = (CPUInfo[2] & 0x1) != 0;                // test bit 0 for SSE3
    info.bSSE41 = (CPUInfo[2] & 0x80000) != 0;           // test bit 19 for SSE4.1
    info.bSSE42 = (CPUInfo[2] & 0x100000) != 0;          // test bit 20 for SSE4.2
    info.bAVX = (CPUInfo[2] & 0x18000000) == 0x18000000; // test bits 28,27 for AVX
    info.bFMA = (CPUInfo[2] & 0x18001000) == 0x18001000; // test bits 28,27,12 for FMA

    // EAX=0x80000000 => CPUID returns extended features
    CPUID(CPUInfo, 0x80000000);
    const unsigned nExIds = CPUInfo[0];
    info.bEXT = (nExIds >= 0x80000000);

    // must be greater than 0x80000004 to support CPU name
    if(nExIds > 0x80000004)
    {
        size_t idx(0);
        CPUID(CPUInfo, 0x80000002); // CPUID returns CPU name part1
        while(((uint8_t *)CPUInfo)[idx] == ' ')
            ++idx;
        memcpy(info.name, (uint8_t *)CPUInfo + idx, sizeof(CPUInfo) - idx);
        idx = sizeof(CPUInfo) - idx;

        CPUID(CPUInfo, 0x80000003); // CPUID returns CPU name part2
        memcpy(info.name + idx, CPUInfo, sizeof(CPUInfo));
        idx += 16;

        CPUID(CPUInfo, 0x80000004); // CPUID returns CPU name part3
        memcpy(info.name + idx, CPUInfo, sizeof(CPUInfo));
    }

    if((strncmp(info.vendor, "AuthenticAMD", 12) == 0) && info.bEXT)
    {                                                   // AMD
        CPUID(CPUInfo, 0x80000001);                     // CPUID will copy ext. feat. bits to EDX and cpu type to EAX
        info.b3DNOWEX = (CPUInfo[3] & 0x40000000) != 0; // indicates AMD extended 3DNow+!
        info.bMMXEX = (CPUInfo[3] & 0x400000) != 0;     // indicates AMD extended MMX
    }

    return info;
}
#endif


inline String get_cpu_info()
{
#ifndef __aarch64__
    const CPUINFO info(GetCPUInfo_());
    String cpu(info.name[0] == 0 ? info.vendor : info.name);
#else
    CPUINFO_ARM cpu_info = {};
    populate_arm_cpu_info(cpu_info);

    std::string cpu = cpu_info.name; // CPU 名称

    // 添加实现者信息
    cpu += " (" + std::string(cpu_info.vendor) + ")";

    // 添加架构版本
    cpu += " Arch v" + std::to_string(cpu_info.architecture);

    // 添加扩展特性
    if(cpu_info.bSVE) cpu += " SVE";
    if(cpu_info.bNEON) cpu += " NEON";
    if(cpu_info.bVFP) cpu += " VFP";
#endif
    return cpu;
}

/*----------------------------------------------------------------*/

inline String get_RAM_info()
{
#if defined(_MSC_VER)

#ifdef _WIN64
    MEMORYSTATUSEX memoryStatus;
    memset(&memoryStatus, sizeof(MEMORYSTATUSEX), 0);
    memoryStatus.dwLength = sizeof(memoryStatus);
    ::GlobalMemoryStatusEx(&memoryStatus);
    const size_t nTotalPhys((size_t)memoryStatus.ullTotalPhys);
    const size_t nTotalVirtual((size_t)memoryStatus.ullTotalVirtual);
#else
    MEMORYSTATUS memoryStatus;
    memset(&memoryStatus, sizeof(MEMORYSTATUS), 0);
    memoryStatus.dwLength = sizeof(MEMORYSTATUS);
    ::GlobalMemoryStatus(&memoryStatus);
    const size_t nTotalPhys((size_t)memoryStatus.dwTotalPhys);
    const size_t nTotalVirtual((size_t)memoryStatus.dwTotalVirtual);
#endif

#elif defined(__APPLE__)

    int mib[2] = { CTL_HW, HW_MEMSIZE };
    const unsigned namelen = sizeof(mib) / sizeof(mib[0]);
    size_t len = sizeof(size_t);
    size_t nTotalPhys;
    sysctl(mib, namelen, &nTotalPhys, &len, NULL, 0);
    const size_t nTotalVirtual(nTotalPhys);

#else // __GNUC__

    struct sysinfo info;
    sysinfo(&info);
    const size_t nTotalPhys((size_t)info.totalram);
    const size_t nTotalVirtual((size_t)info.totalswap);

#endif // _MSC_VER
    return formatBytes(nTotalPhys) + " Physical Memory " + formatBytes(nTotalVirtual) + " Virtual Memory";
}
/*----------------------------------------------------------------*/

inline String get_OS_info()
{
#ifdef _MSC_VER

    String os;
#ifdef _USE_WINSDKOS
#ifndef _WIN32_WINNT_WIN10
#define _WIN32_WINNT_WIN10 0x0A00
    if(IsWindowsVersionOrGreater(HIBYTE(_WIN32_WINNT_WIN10), LOBYTE(_WIN32_WINNT_WIN10), 0))
#else
    if(IsWindows10OrGreater())
#endif
        os = _T("Windows 10+");
    else if(IsWindows8Point1OrGreater())
        os = _T("Windows 8.1");
    else if(IsWindows8OrGreater())
        os = _T("Windows 8");
    else if(IsWindows7SP1OrGreater())
        os = _T("Windows 7 (SP1)");
    else if(IsWindows7OrGreater())
        os = _T("Windows 7");
    else if(IsWindowsVistaSP2OrGreater())
        os = _T("Windows Vista (SP2)");
    else if(IsWindowsVistaSP1OrGreater())
        os = _T("Windows Vista (SP1)");
    else if(IsWindowsVistaOrGreater())
        os = _T("Windows Vista");
    else if(IsWindowsXPSP3OrGreater())
        os = _T("Windows XP (SP3)");
    else if(IsWindowsXPSP2OrGreater())
        os = _T("Windows XP (SP2)");
    else if(IsWindowsXPSP1OrGreater())
        os = _T("Windows XP (SP1)");
    else if(IsWindowsXPOrGreater())
        os = _T("Windows XP");
    else
        os = _T("Windows (unknown version)");
#else
    OSVERSIONINFOEX ver;
    memset(&ver, 0, sizeof(OSVERSIONINFOEX));
    ver.dwOSVersionInfoSize = sizeof(OSVERSIONINFOEX);

    if(!GetVersionEx((OSVERSIONINFO *)&ver))
    {
        ver.dwOSVersionInfoSize = sizeof(OSVERSIONINFO);
        if(!GetVersionEx((OSVERSIONINFO *)&ver))
        {
            return "Windows (unknown version)";
        }
    }

    if(ver.dwPlatformId != VER_PLATFORM_WIN32_NT)
    {
        os = "Win9x/ME";
    } else
    {
        switch(ver.dwMajorVersion)
        {
        case 4:
            os = "WinNT4";
            break;

        case 5:
            switch(ver.dwMinorVersion)
            {
            case 0:
                os = "Win2000";
                break;
            case 1:
                os = "WinXP";
                break;
            case 2:
                os = "Win2003";
                break;
            default:
                os = "Unknown WinNT5";
            }
            break;

        case 6:
            switch(ver.dwMinorVersion)
            {
            case 0:
                os = (ver.wProductType == VER_NT_WORKSTATION ? "WinVista" : "Win2008");
                break;
            case 1:
                os = (ver.wProductType == VER_NT_WORKSTATION ? "Win7" : "Win2008R2");
                break;
            case 2:
                os = (ver.wProductType == VER_NT_WORKSTATION ? "Win8" : "Win2012");
                break;
            case 3:
                os = (ver.wProductType == VER_NT_WORKSTATION ? "Win8.1" : "Win2012R2");
                break;
            case 4:
                os = "Win10";
                break;
            default:
                os = "Unknown WinNT6";
            }
            break;

        default:
            os = "Windows (version unknown)";
        }
        if(ver.wProductType & VER_NT_WORKSTATION)
            os += " Pro";
        else if(ver.wProductType & VER_NT_SERVER)
            os += " Server";
        else if(ver.wProductType & VER_NT_DOMAIN_CONTROLLER)
            os += " DC";
    }

    if(ver.wServicePackMajor != 0)
    {
        os += " (SP";
        os += String::ToString(ver.wServicePackMajor);
        if(ver.wServicePackMinor != 0)
        {
            os += '.';
            os += String::ToString(ver.wServicePackMinor);
        }
        os += ")";
    }
#endif

#ifdef _WIN64
    os += " x64";
#else
    typedef BOOL(WINAPI * LPFN_ISWOW64PROCESS)(HANDLE, PBOOL);
    const LPFN_ISWOW64PROCESS fnIsWow64Process
        = (LPFN_ISWOW64PROCESS)GetProcAddress(GetModuleHandle("kernel32"), "IsWow64Process");
    BOOL bIsWow64 = FALSE;
    if(fnIsWow64Process && fnIsWow64Process(GetCurrentProcess(), &bIsWow64) && bIsWow64) os += " x64";
#endif

    return os;

#else // _MSC_VER

    utsname n;
    if(uname(&n) != 0) return "linux (unknown version)";
    return String(n.sysname) + " " + String(n.release) + " (" + String(n.machine) + ")";

#endif // _MSC_VER
}
/*----------------------------------------------------------------*/
};

#endif

inline String get_current_folder()
{
    char pathname[MAX_PATH + 1];
#ifdef _MSC_VER
    if(!GetCurrentDirectory(MAX_PATH, pathname))
#else  // _MSC_VER
    if(!getcwd(pathname, MAX_PATH))
#endif // _MSC_VER
        return String();
    String dir(String(pathname) + PATH_SEPARATOR);
    return ensureUnifySlash(dir);
}

#ifdef _MSC_VER
#include <intrin.h>
inline void CPUID(int CPUInfo[4], int level) { __cpuid(CPUInfo, level); }
#elif defined(__x86_64__)
#include <cpuid.h>
inline void CPUID(int CPUInfo[4], int level)
{
    unsigned *p((unsigned *)CPUInfo);
    __get_cpuid((unsigned &)level, p + 0, p + 1, p + 2, p + 3);
}
#elif defined(__aarch64__)

// 内联函数：读取 MIDR_EL1 系统寄存器
inline uint64_t read_midr_el1()
{
    uint64_t midr_el1;
    asm volatile("mrs %0, MIDR_EL1" : "=r"(midr_el1));
    return midr_el1;
}

// 内联函数：读取 ID_AA64PFR0_EL1 系统寄存器
inline uint64_t read_id_aa64pfr0_el1()
{
    uint64_t id_aa64pfr0_el1;
    asm volatile("mrs %0, ID_AA64PFR0_EL1" : "=r"(id_aa64pfr0_el1));
    return id_aa64pfr0_el1;
}

inline void populate_arm_cpu_info(CPUINFO_ARM &cpu_info)
{
    uint64_t midr_el1 = read_midr_el1();
    uint64_t id_aa64pfr0_el1 = read_id_aa64pfr0_el1();

    // 解析 MIDR_EL1 的字段
    cpu_info.implementer = (midr_el1 >> 24) & 0xFF; // [31:24]
    cpu_info.variant = (midr_el1 >> 20) & 0xF;      // [23:20]
    cpu_info.architecture = (midr_el1 >> 16) & 0xF; // [19:16]
    cpu_info.part_number = (midr_el1 >> 4) & 0xFFF; // [15:4]
    cpu_info.revision = midr_el1 & 0xF;             // [3:0]

    switch(cpu_info.implementer)
    {
    case 0x41:
        strncpy(cpu_info.vendor, "ARM", sizeof(cpu_info.vendor));
        break;
    case 0x42:
        strncpy(cpu_info.vendor, "Broadcom", sizeof(cpu_info.vendor));
        break;
    case 0x43:
        strncpy(cpu_info.vendor, "Cavium", sizeof(cpu_info.vendor));
        break;
    case 0x44:
        strncpy(cpu_info.vendor, "Digital Equipment Corporation", sizeof(cpu_info.vendor));
        break;
    case 0x46:
        strncpy(cpu_info.vendor, "Fujitsu", sizeof(cpu_info.vendor));
        break;
    case 0x49:
        strncpy(cpu_info.vendor, "Infineon Technologies", sizeof(cpu_info.vendor));
        break;
    case 0x4D:
        strncpy(cpu_info.vendor, "Motorola", sizeof(cpu_info.vendor));
        break;
    case 0x4E:
        strncpy(cpu_info.vendor, "NVIDIA", sizeof(cpu_info.vendor));
        break;
    case 0x50:
        strncpy(cpu_info.vendor, "Applied Micro", sizeof(cpu_info.vendor));
        break;
    case 0x51:
        strncpy(cpu_info.vendor, "Qualcomm", sizeof(cpu_info.vendor));
        break;
    case 0x56:
        strncpy(cpu_info.vendor, "Marvell", sizeof(cpu_info.vendor));
        break;
    case 0x69:
        strncpy(cpu_info.vendor, "Intel", sizeof(cpu_info.vendor));
        break;
    default:
        strncpy(cpu_info.vendor, "ImpUnknown", sizeof(cpu_info.vendor));
        break;
    }

    // 根据部件号设置 CPU 名称（需要查表）
    switch(cpu_info.part_number)
    {
    case 0x810:
        strncpy(cpu_info.name, "ARM810", sizeof(cpu_info.name));
        break;
    case 0xc05:
        strncpy(cpu_info.name, "Cortex-A5", sizeof(cpu_info.name));
        break;
    case 0xC07:
        strncpy(cpu_info.name, "Cortex-A7", sizeof(cpu_info.name));
        break;
    case 0xc08:
        strncpy(cpu_info.name, "Cortex-A8", sizeof(cpu_info.name));
        break;
    case 0xC09:
        strncpy(cpu_info.name, "Cortex-A9", sizeof(cpu_info.name));
        break;
    case 0xC0d:
        strncpy(cpu_info.name, "Cortex-A17", sizeof(cpu_info.name));
        break;
    case 0xC0f:
        strncpy(cpu_info.name, "Cortex-A15", sizeof(cpu_info.name));
        break;
    case 0xC20:
        strncpy(cpu_info.name, "Cortex-M0", sizeof(cpu_info.name));
        break;
    case 0xC23:
        strncpy(cpu_info.name, "Cortex-M3", sizeof(cpu_info.name));
        break;
    case 0xC24:
        strncpy(cpu_info.name, "Cortex-M4", sizeof(cpu_info.name));
        break;
    case 0xC27:
        strncpy(cpu_info.name, "Cortex-M7", sizeof(cpu_info.name));
        break;
    case 0xC60:
        strncpy(cpu_info.name, "Cortex-R0", sizeof(cpu_info.name));
        break;
    case 0xD01:
        strncpy(cpu_info.name, "Cortex-A32", sizeof(cpu_info.name));
        break;
    case 0xD03:
        strncpy(cpu_info.name, "Cortex-A53", sizeof(cpu_info.name));
        break;
    case 0xD04:
        strncpy(cpu_info.name, "Cortex-A35", sizeof(cpu_info.name));
        break;
    case 0xD05:
        strncpy(cpu_info.name, "Cortex-A55", sizeof(cpu_info.name));
        break;
    case 0xD07:
        strncpy(cpu_info.name, "Cortex-A57", sizeof(cpu_info.name));
        break;
    case 0xD08:
        strncpy(cpu_info.name, "Cortex-A72", sizeof(cpu_info.name));
        break;
    case 0xD09:
        strncpy(cpu_info.name, "Cortex-A73", sizeof(cpu_info.name));
        break;
    case 0xD0A:
        strncpy(cpu_info.name, "Cortex-A75", sizeof(cpu_info.name));
        break;
    case 0xD0D:
        strncpy(cpu_info.name, "Cortex-A77", sizeof(cpu_info.name));
        break;
    case 0xD13:
        strncpy(cpu_info.name, "Cortex-R52", sizeof(cpu_info.name));
        break;
    case 0xD20:
        strncpy(cpu_info.name, "Cortex-M23", sizeof(cpu_info.name));
        break;
    case 0xD42:
        strncpy(cpu_info.name, "Cortex-A78AE", sizeof(cpu_info.name));
        break;
    case 0xD4A:
        strncpy(cpu_info.name, "Cortex-X2", sizeof(cpu_info.name));
        break;
    case 0xD4B:
        strncpy(cpu_info.name, "Cortex-A710", sizeof(cpu_info.name));
        break;
    case 0xD4C:
        strncpy(cpu_info.name, "Cortex-A510", sizeof(cpu_info.name));
        break;
    case 0xD40:
        strncpy(cpu_info.name, "Neoverse-N1", sizeof(cpu_info.name));
        break;
    case 0xD49:
        strncpy(cpu_info.name, "Neoverse-V1", sizeof(cpu_info.name));
        break;
    case 0xD4D:
        strncpy(cpu_info.name, "Neoverse-N2", sizeof(cpu_info.name));
        break;
    case 0xD4E:
        strncpy(cpu_info.name, "Neoverse-V2", sizeof(cpu_info.name));
        break;
    default:
        strncpy(cpu_info.name, "Unknown", sizeof(cpu_info.name));
        break;
    }

    // 检查扩展功能支持
    cpu_info.bNEON = true;                                // ARMv8-A 默认支持 NEON
    cpu_info.bVFP = true;                                 // ARMv8-A 默认支持 VFP
    cpu_info.bSVE = ((id_aa64pfr0_el1 >> 32) & 0xF) == 1; // SVE 位 [35:32]
}

inline std::string get_arm_cpu_info()
{
    CPUINFO_ARM cpu_info = {};
    populate_arm_cpu_info(cpu_info);

    std::string cpu = cpu_info.name; // CPU 名称

    // 添加实现者信息
    cpu += " (" + std::string(cpu_info.vendor) + ")";

    // 添加架构版本
    cpu += " Arch v" + std::to_string(cpu_info.architecture);

    // 添加扩展特性
    if(cpu_info.bSVE) cpu += " SVE";
    if(cpu_info.bNEON) cpu += " NEON";
    if(cpu_info.bVFP) cpu += " VFP";

    return cpu;
}
#else
#error "Unsupported platform for CPUID or CPU information retrieval"
#endif

#ifdef __x86_64__
inline CPUINFO GetCPUInfo_()
{
    CPUINFO info;

    // set all values to 0 (false)
    memset(&info, 0, sizeof(CPUINFO));

    int CPUInfo[4];

    // CPUID with an InfoType argument of 0 returns the number of
    // valid Ids in CPUInfo[0] and the CPU identification string in
    // the other three array elements. The CPU identification string is
    // not in linear order. The code below arranges the information
    // in a human readable form.
    CPUID(CPUInfo, 0);
    *((int *)info.vendor) = CPUInfo[1];
    *((int *)(info.vendor + 4)) = CPUInfo[3];
    *((int *)(info.vendor + 8)) = CPUInfo[2];

    // Interpret CPU feature information.
    CPUID(CPUInfo, 1);
    info.bMMX = (CPUInfo[3] & 0x800000) != 0;            // test bit 23 for MMX
    info.bSSE = (CPUInfo[3] & 0x2000000) != 0;           // test bit 25 for SSE
    info.bSSE2 = (CPUInfo[3] & 0x4000000) != 0;          // test bit 26 for SSE2
    info.bSSE3 = (CPUInfo[2] & 0x1) != 0;                // test bit 0 for SSE3
    info.bSSE41 = (CPUInfo[2] & 0x80000) != 0;           // test bit 19 for SSE4.1
    info.bSSE42 = (CPUInfo[2] & 0x100000) != 0;          // test bit 20 for SSE4.2
    info.bAVX = (CPUInfo[2] & 0x18000000) == 0x18000000; // test bits 28,27 for AVX
    info.bFMA = (CPUInfo[2] & 0x18001000) == 0x18001000; // test bits 28,27,12 for FMA

    // EAX=0x80000000 => CPUID returns extended features
    CPUID(CPUInfo, 0x80000000);
    const unsigned nExIds = CPUInfo[0];
    info.bEXT = (nExIds >= 0x80000000);

    // must be greater than 0x80000004 to support CPU name
    if(nExIds > 0x80000004)
    {
        size_t idx(0);
        CPUID(CPUInfo, 0x80000002); // CPUID returns CPU name part1
        while(((uint8_t *)CPUInfo)[idx] == ' ')
            ++idx;
        memcpy(info.name, (uint8_t *)CPUInfo + idx, sizeof(CPUInfo) - idx);
        idx = sizeof(CPUInfo) - idx;

        CPUID(CPUInfo, 0x80000003); // CPUID returns CPU name part2
        memcpy(info.name + idx, CPUInfo, sizeof(CPUInfo));
        idx += 16;

        CPUID(CPUInfo, 0x80000004); // CPUID returns CPU name part3
        memcpy(info.name + idx, CPUInfo, sizeof(CPUInfo));
    }

    if((strncmp(info.vendor, "AuthenticAMD", 12) == 0) && info.bEXT)
    {                                                   // AMD
        CPUID(CPUInfo, 0x80000001);                     // CPUID will copy ext. feat. bits to EDX and cpu type to EAX
        info.b3DNOWEX = (CPUInfo[3] & 0x40000000) != 0; // indicates AMD extended 3DNow+!
        info.bMMXEX = (CPUInfo[3] & 0x400000) != 0;     // indicates AMD extended MMX
    }

    return info;
}
#endif


inline String get_cpu_info()
{
#ifndef __aarch64__
    const CPUINFO info(GetCPUInfo_());
    String cpu(info.name[0] == 0 ? info.vendor : info.name);
#else
    CPUINFO_ARM cpu_info = {};
    populate_arm_cpu_info(cpu_info);

    std::string cpu = cpu_info.name; // CPU 名称

    // 添加实现者信息
    cpu += " (" + std::string(cpu_info.vendor) + ")";

    // 添加架构版本
    cpu += " Arch v" + std::to_string(cpu_info.architecture);

    // 添加扩展特性
    if(cpu_info.bSVE) cpu += " SVE";
    if(cpu_info.bNEON) cpu += " NEON";
    if(cpu_info.bVFP) cpu += " VFP";
#endif
    return cpu;
}

/*----------------------------------------------------------------*/

inline String get_RAM_info()
{
#if defined(_MSC_VER)

#ifdef _WIN64
    MEMORYSTATUSEX memoryStatus;
    memset(&memoryStatus, sizeof(MEMORYSTATUSEX), 0);
    memoryStatus.dwLength = sizeof(memoryStatus);
    ::GlobalMemoryStatusEx(&memoryStatus);
    const size_t nTotalPhys((size_t)memoryStatus.ullTotalPhys);
    const size_t nTotalVirtual((size_t)memoryStatus.ullTotalVirtual);
#else
    MEMORYSTATUS memoryStatus;
    memset(&memoryStatus, sizeof(MEMORYSTATUS), 0);
    memoryStatus.dwLength = sizeof(MEMORYSTATUS);
    ::GlobalMemoryStatus(&memoryStatus);
    const size_t nTotalPhys((size_t)memoryStatus.dwTotalPhys);
    const size_t nTotalVirtual((size_t)memoryStatus.dwTotalVirtual);
#endif

#elif defined(__APPLE__)

    int mib[2] = { CTL_HW, HW_MEMSIZE };
    const unsigned namelen = sizeof(mib) / sizeof(mib[0]);
    size_t len = sizeof(size_t);
    size_t nTotalPhys;
    sysctl(mib, namelen, &nTotalPhys, &len, NULL, 0);
    const size_t nTotalVirtual(nTotalPhys);

#else // __GNUC__

    struct sysinfo info;
    sysinfo(&info);
    const size_t nTotalPhys((size_t)info.totalram);
    const size_t nTotalVirtual((size_t)info.totalswap);

#endif // _MSC_VER
    return formatBytes(nTotalPhys) + " Physical Memory " + formatBytes(nTotalVirtual) + " Virtual Memory";
}

inline double get_total_phy_RAM_size_in_MB()
{
    struct sysinfo info;
    sysinfo(&info);
    return ((double)((size_t)info.totalram) / 1024.0 / 1024.0);
}

inline double get_total_RAM_size_in_MB()
{
    struct sysinfo info;
    sysinfo(&info);
    return ((double)((size_t)info.totalram + info.totalswap) / 1024.0 / 1024.0);
}

inline double get_total_phy_RAM_size_in_GB() { return ((double)get_total_phy_RAM_size_in_MB() / 1024.0); }

inline double get_total_RAM_size_in_GB() { return ((double)get_total_RAM_size_in_MB() / 1024.0); }


inline String get_OS_info()
{
#ifdef _MSC_VER

    String os;
#ifdef _USE_WINSDKOS
#ifndef _WIN32_WINNT_WIN10
#define _WIN32_WINNT_WIN10 0x0A00
    if(IsWindowsVersionOrGreater(HIBYTE(_WIN32_WINNT_WIN10), LOBYTE(_WIN32_WINNT_WIN10), 0))
#else
    if(IsWindows10OrGreater())
#endif
        os = _T("Windows 10+");
    else if(IsWindows8Point1OrGreater())
        os = _T("Windows 8.1");
    else if(IsWindows8OrGreater())
        os = _T("Windows 8");
    else if(IsWindows7SP1OrGreater())
        os = _T("Windows 7 (SP1)");
    else if(IsWindows7OrGreater())
        os = _T("Windows 7");
    else if(IsWindowsVistaSP2OrGreater())
        os = _T("Windows Vista (SP2)");
    else if(IsWindowsVistaSP1OrGreater())
        os = _T("Windows Vista (SP1)");
    else if(IsWindowsVistaOrGreater())
        os = _T("Windows Vista");
    else if(IsWindowsXPSP3OrGreater())
        os = _T("Windows XP (SP3)");
    else if(IsWindowsXPSP2OrGreater())
        os = _T("Windows XP (SP2)");
    else if(IsWindowsXPSP1OrGreater())
        os = _T("Windows XP (SP1)");
    else if(IsWindowsXPOrGreater())
        os = _T("Windows XP");
    else
        os = _T("Windows (unknown version)");
#else
    OSVERSIONINFOEX ver;
    memset(&ver, 0, sizeof(OSVERSIONINFOEX));
    ver.dwOSVersionInfoSize = sizeof(OSVERSIONINFOEX);

    if(!GetVersionEx((OSVERSIONINFO *)&ver))
    {
        ver.dwOSVersionInfoSize = sizeof(OSVERSIONINFO);
        if(!GetVersionEx((OSVERSIONINFO *)&ver))
        {
            return "Windows (unknown version)";
        }
    }

    if(ver.dwPlatformId != VER_PLATFORM_WIN32_NT)
    {
        os = "Win9x/ME";
    } else
    {
        switch(ver.dwMajorVersion)
        {
        case 4:
            os = "WinNT4";
            break;

        case 5:
            switch(ver.dwMinorVersion)
            {
            case 0:
                os = "Win2000";
                break;
            case 1:
                os = "WinXP";
                break;
            case 2:
                os = "Win2003";
                break;
            default:
                os = "Unknown WinNT5";
            }
            break;

        case 6:
            switch(ver.dwMinorVersion)
            {
            case 0:
                os = (ver.wProductType == VER_NT_WORKSTATION ? "WinVista" : "Win2008");
                break;
            case 1:
                os = (ver.wProductType == VER_NT_WORKSTATION ? "Win7" : "Win2008R2");
                break;
            case 2:
                os = (ver.wProductType == VER_NT_WORKSTATION ? "Win8" : "Win2012");
                break;
            case 3:
                os = (ver.wProductType == VER_NT_WORKSTATION ? "Win8.1" : "Win2012R2");
                break;
            case 4:
                os = "Win10";
                break;
            default:
                os = "Unknown WinNT6";
            }
            break;

        default:
            os = "Windows (version unknown)";
        }
        if(ver.wProductType & VER_NT_WORKSTATION)
            os += " Pro";
        else if(ver.wProductType & VER_NT_SERVER)
            os += " Server";
        else if(ver.wProductType & VER_NT_DOMAIN_CONTROLLER)
            os += " DC";
    }

    if(ver.wServicePackMajor != 0)
    {
        os += " (SP";
        os += String::ToString(ver.wServicePackMajor);
        if(ver.wServicePackMinor != 0)
        {
            os += '.';
            os += String::ToString(ver.wServicePackMinor);
        }
        os += ")";
    }
#endif

#ifdef _WIN64
    os += " x64";
#else
    typedef BOOL(WINAPI * LPFN_ISWOW64PROCESS)(HANDLE, PBOOL);
    const LPFN_ISWOW64PROCESS fnIsWow64Process
        = (LPFN_ISWOW64PROCESS)GetProcAddress(GetModuleHandle("kernel32"), "IsWow64Process");
    BOOL bIsWow64 = FALSE;
    if(fnIsWow64Process && fnIsWow64Process(GetCurrentProcess(), &bIsWow64) && bIsWow64) os += " x64";
#endif

    return os;

#else // _MSC_VER

    utsname n;
    if(uname(&n) != 0) return "linux (unknown version)";
    return String(n.sysname) + " " + String(n.release) + " (" + String(n.machine) + ")";

#endif // _MSC_VER
}
/*----------------------------------------------------------------*/
};


#endif // UTILS_LOGGER_HPP
