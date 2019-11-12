#ifndef CNOID_ROSUTIL_EXPORTDECL_H
# define CNOID_ROSUTIL_EXPORTDECL_H

# if defined _WIN32 || defined __CYGWIN__
#  define CNOID_ROSUTIL_DLLIMPORT __declspec(dllimport)
#  define CNOID_ROSUTIL_DLLEXPORT __declspec(dllexport)
#  define CNOID_ROSUTIL_DLLLOCAL
# else
#  if __GNUC__ >= 4
#   define CNOID_ROSUTIL_DLLIMPORT __attribute__ ((visibility("default")))
#   define CNOID_ROSUTIL_DLLEXPORT __attribute__ ((visibility("default")))
#   define CNOID_ROSUTIL_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define CNOID_ROSUTIL_DLLIMPORT
#   define CNOID_ROSUTIL_DLLEXPORT
#   define CNOID_ROSUTIL_DLLLOCAL
#  endif
# endif

# ifdef CNOID_ROSUTIL_STATIC
#  define CNOID_ROSUTIL_DLLAPI
#  define CNOID_ROSUTIL_LOCAL
# else
#  ifdef CnoidROSUtil_EXPORTS
#   define CNOID_ROSUTIL_DLLAPI CNOID_ROSUTIL_DLLEXPORT
#  else
#   define CNOID_ROSUTIL_DLLAPI CNOID_ROSUTIL_DLLIMPORT
#  endif
#  define CNOID_ROSUTIL_LOCAL CNOID_ROSUTIL_DLLLOCAL
# endif

#endif

#ifdef CNOID_EXPORT
# undef CNOID_EXPORT
#endif
#define CNOID_EXPORT CNOID_ROSUTIL_DLLAPI
