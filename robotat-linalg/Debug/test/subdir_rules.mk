################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
test/%.obj: ../test/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"/opt/ti/ccs1030/ccs/tools/compiler/ti-cgt-arm_20.2.5.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="/media/Win10/Users/Daniel/UVG/Ciclo10/Diseño de Ingenieria 2/codigo/robotat-linalg" --include_path="/media/Win10/Users/Daniel/UVG/Ciclo10/Diseño de Ingenieria 2/codigo/robotat-linalg/src" --include_path="/opt/ti/ccs1030/ccs/tools/compiler/ti-cgt-arm_20.2.5.LTS/include" --define=ccs="ccs" --define=PART_TM4C123GH6PM -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="test/$(basename $(<F)).d_raw" --obj_directory="test" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: "$<"'
	@echo ' '


