################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
Clock.obj: C:/Dropbox/TIteachingProject/RSLK_1_1/inc/Clock.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccs901/ccs/tools/compiler/ti-cgt-arm_18.12.1.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="C:/ti/ccs901/ccs/ccs_base/arm/include" --include_path="C:/ti/ccs901/ccs/ccs_base/arm/include/CMSIS" --include_path="C:/Dropbox/TIteachingProject/RSLK_1_1/PeriodicTimerA0Ints" --include_path="C:/ti/ccs901/ccs/tools/compiler/ti-cgt-arm_18.12.1.LTS/include" --advice:power=all --define=__MSP432P401R__ --define=ccs -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

CortexM.obj: C:/Dropbox/TIteachingProject/RSLK_1_1/inc/CortexM.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccs901/ccs/tools/compiler/ti-cgt-arm_18.12.1.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="C:/ti/ccs901/ccs/ccs_base/arm/include" --include_path="C:/ti/ccs901/ccs/ccs_base/arm/include/CMSIS" --include_path="C:/Dropbox/TIteachingProject/RSLK_1_1/PeriodicTimerA0Ints" --include_path="C:/ti/ccs901/ccs/tools/compiler/ti-cgt-arm_18.12.1.LTS/include" --advice:power=all --define=__MSP432P401R__ --define=ccs -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

LaunchPad.obj: C:/Dropbox/TIteachingProject/RSLK_1_1/inc/LaunchPad.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccs901/ccs/tools/compiler/ti-cgt-arm_18.12.1.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="C:/ti/ccs901/ccs/ccs_base/arm/include" --include_path="C:/ti/ccs901/ccs/ccs_base/arm/include/CMSIS" --include_path="C:/Dropbox/TIteachingProject/RSLK_1_1/PeriodicTimerA0Ints" --include_path="C:/ti/ccs901/ccs/tools/compiler/ti-cgt-arm_18.12.1.LTS/include" --advice:power=all --define=__MSP432P401R__ --define=ccs -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccs901/ccs/tools/compiler/ti-cgt-arm_18.12.1.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="C:/ti/ccs901/ccs/ccs_base/arm/include" --include_path="C:/ti/ccs901/ccs/ccs_base/arm/include/CMSIS" --include_path="C:/Dropbox/TIteachingProject/RSLK_1_1/PeriodicTimerA0Ints" --include_path="C:/ti/ccs901/ccs/tools/compiler/ti-cgt-arm_18.12.1.LTS/include" --advice:power=all --define=__MSP432P401R__ --define=ccs -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

TimerA0.obj: C:/Dropbox/TIteachingProject/RSLK_1_1/inc/TimerA0.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccs901/ccs/tools/compiler/ti-cgt-arm_18.12.1.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="C:/ti/ccs901/ccs/ccs_base/arm/include" --include_path="C:/ti/ccs901/ccs/ccs_base/arm/include/CMSIS" --include_path="C:/Dropbox/TIteachingProject/RSLK_1_1/PeriodicTimerA0Ints" --include_path="C:/ti/ccs901/ccs/tools/compiler/ti-cgt-arm_18.12.1.LTS/include" --advice:power=all --define=__MSP432P401R__ --define=ccs -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


