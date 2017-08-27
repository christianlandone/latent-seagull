################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
build-1297958319:
	@$(MAKE) -Onone -f subdir_rules.mk build-1297958319-inproc

build-1297958319-inproc: ../release.cfg
	@echo 'Building file: $<'
	@echo 'Invoking: XDCtools'
	"C:/ti/ccs720/xdctools_3_50_02_20_core/xs" --xdcpath="C:/ti/simplelink_cc13x0_sdk_1_40_00_10/source;C:/ti/simplelink_cc13x0_sdk_1_40_00_10/kernel/tirtos/packages;C:/ti/ccs720/ccsv7/ccs_base;" xdc.tools.configuro -o configPkg -t ti.targets.arm.elf.M3 -p ti.platforms.simplelink:CC1350F128 -r release -c "C:/ti/ccs720/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS" --compileOptions " -DDeviceFamily_CC13X0 " "$<"
	@echo 'Finished building: $<'
	@echo ' '

configPkg/linker.cmd: build-1297958319 ../release.cfg
configPkg/compiler.opt: build-1297958319
configPkg/: build-1297958319


