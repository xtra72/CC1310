################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
build-1317695535:
	@$(MAKE) --no-print-directory -Onone -f subdir_rules.mk build-1317695535-inproc

build-1317695535-inproc: ../release.cfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: XDCtools'
	"C:/ti/xdctools_3_51_02_21_core/xs" --xdcpath="C:/ti/simplelink_cc13x0_sdk_3_10_00_11/source;C:/ti/simplelink_cc13x0_sdk_3_10_00_11/kernel/tirtos/packages;" xdc.tools.configuro -o configPkg -t ti.targets.arm.elf.M3 -p ti.platforms.simplelink:CC1310F128 -r release -c "C:/ti/ccs901/ccs/tools/compiler/ti-cgt-arm_18.12.1.LTS" --compileOptions " -DDeviceFamily_CC13X0 " "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

configPkg/linker.cmd: build-1317695535 ../release.cfg
configPkg/compiler.opt: build-1317695535
configPkg/: build-1317695535


