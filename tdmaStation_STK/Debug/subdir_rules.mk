################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
CC1350STK.obj: ../CC1350STK.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccs720/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS/bin/armcl" -mv7M3 --code_state=16 --float_support=vfplib -me --include_path="C:/Users/chris/Desktop/Proj/Commesh/software/latent-seagull/tdmaStation_STK" --include_path="C:/ti/simplelink_cc13x0_sdk_1_40_00_10/kernel/tirtos/packages/ti/sysbios/posix" --include_path="//EPICURUS/Users/chris/Desktop/Proj/Commesh/software/latent-seagull/tdmaBTS_LPD/smartrf_settings" --include_path="C:/ti/ccs720/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS/include" --define=DeviceFamily_CC13X0 -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="CC1350STK.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

StateMachine.obj: ../StateMachine.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccs720/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS/bin/armcl" -mv7M3 --code_state=16 --float_support=vfplib -me --include_path="C:/Users/chris/Desktop/Proj/Commesh/software/latent-seagull/tdmaStation_STK" --include_path="C:/ti/simplelink_cc13x0_sdk_1_40_00_10/kernel/tirtos/packages/ti/sysbios/posix" --include_path="//EPICURUS/Users/chris/Desktop/Proj/Commesh/software/latent-seagull/tdmaBTS_LPD/smartrf_settings" --include_path="C:/ti/ccs720/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS/include" --define=DeviceFamily_CC13X0 -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="StateMachine.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

ccfg.obj: ../ccfg.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccs720/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS/bin/armcl" -mv7M3 --code_state=16 --float_support=vfplib -me --include_path="C:/Users/chris/Desktop/Proj/Commesh/software/latent-seagull/tdmaStation_STK" --include_path="C:/ti/simplelink_cc13x0_sdk_1_40_00_10/kernel/tirtos/packages/ti/sysbios/posix" --include_path="//EPICURUS/Users/chris/Desktop/Proj/Commesh/software/latent-seagull/tdmaBTS_LPD/smartrf_settings" --include_path="C:/ti/ccs720/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS/include" --define=DeviceFamily_CC13X0 -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="ccfg.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

tdmaStation.obj: ../tdmaStation.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccs720/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS/bin/armcl" -mv7M3 --code_state=16 --float_support=vfplib -me --include_path="C:/Users/chris/Desktop/Proj/Commesh/software/latent-seagull/tdmaStation_STK" --include_path="C:/ti/simplelink_cc13x0_sdk_1_40_00_10/kernel/tirtos/packages/ti/sysbios/posix" --include_path="//EPICURUS/Users/chris/Desktop/Proj/Commesh/software/latent-seagull/tdmaBTS_LPD/smartrf_settings" --include_path="C:/ti/ccs720/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS/include" --define=DeviceFamily_CC13X0 -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="tdmaStation.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


