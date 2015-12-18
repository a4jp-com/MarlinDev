// @section info

#if ENABLED(USE_AUTOMATIC_VERSIONING)
  #include "_Version.h"
#else
  #include "Default_Version.h"
#endif

// User-specified version info of this build to display in [Pronterface, etc] terminal window during
// startup. Implementation of an idea by Prof Braino to inform user that any changes made to this
// build by the user have been successfully uploaded into firmware.
#ifndef STRING_CONFIG_H_AUTHOR
  #define STRING_CONFIG_H_AUTHOR "(none, default config)" // Who made the changes.
#endif
//#define NO_BOOTSCREEN // To suppress the bootscreen even if a display is present
#if DISABLED(NO_BOOTSCREEN)
  #define SHOW_BOOTSCREEN
#endif
#ifndef STRING_SPLASH_LINE1
  #define STRING_SPLASH_LINE1 SHORT_BUILD_VERSION // will be shown during bootup in line 1
#endif
//#define STRING_SPLASH_LINE2 STRING_DISTRIBUTION_DATE // will be shown during bootup in line 2
