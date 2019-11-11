#ifndef CNOID_ROSPLUGIN_EXPORTDECL_H_INCLUDED
# define CNOID_ROSPLUGIN_EXPORTDECL_H_INCLUDED

# if defined _WIN32 || defined __CYGWIN__
#  define CNOID_ROSPLUGIN_DLLIMPORT __declspec(dllimport)
#  define CNOID_ROSPLUGIN_DLLEXPORT __declspec(dllexport)
#  define CNOID_ROSPLUGIN_DLLLOCAL
# else
#  if __GNUC__ >= 4
#   define CNOID_ROSPLUGIN_DLLIMPORT __attribute__ ((visibility("default")))
#   define CNOID_ROSPLUGIN_DLLEXPORT __attribute__ ((visibility("default")))
#   define CNOID_ROSPLUGIN_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define CNOID_ROSPLUGIN_DLLIMPORT
#   define CNOID_ROSPLUGIN_DLLEXPORT
#   define CNOID_ROSPLUGIN_DLLLOCAL
#  endif
# endif

# ifdef CNOID_ROSPLUGIN_STATIC
#  define CNOID_ROSPLUGIN_DLLAPI
#  define CNOID_ROSPLUGIN_LOCAL
# else
#  ifdef CnoidROSPlugin_EXPORTS
#   define CNOID_ROSPLUGIN_DLLAPI CNOID_ROSPLUGIN_DLLEXPORT
#  else
#   define CNOID_ROSPLUGIN_DLLAPI CNOID_ROSPLUGIN_DLLIMPORT
#  endif
#  define CNOID_ROSPLUGIN_LOCAL CNOID_ROSPLUGIN_DLLLOCAL
# endif

#endif

#ifdef CNOID_EXPORT
# undef CNOID_EXPORT
#endif
#define CNOID_EXPORT CNOID_ROSPLUGIN_DLLAPI
