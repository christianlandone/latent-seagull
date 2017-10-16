#
_XDCBUILDCOUNT = 
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = C:/ti/simplelink_cc13x0_sdk_1_40_00_10/source;C:/ti/simplelink_cc13x0_sdk_1_40_00_10/kernel/tirtos/packages;C:/ti/ccs720/ccsv7/ccs_base;C:/Users/chris/Desktop/Proj/Commesh/software/latent-seagull/tirtos_builds_CC1350STK_release_ccs/.config
override XDCROOT = C:/ti/ccs720/xdctools_3_50_02_20_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = C:/ti/simplelink_cc13x0_sdk_1_40_00_10/source;C:/ti/simplelink_cc13x0_sdk_1_40_00_10/kernel/tirtos/packages;C:/ti/ccs720/ccsv7/ccs_base;C:/Users/chris/Desktop/Proj/Commesh/software/latent-seagull/tirtos_builds_CC1350STK_release_ccs/.config;C:/ti/ccs720/xdctools_3_50_02_20_core/packages;..
HOSTOS = Windows
endif
