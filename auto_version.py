# Shamelessly stolen from Awbmilne's answer to this question 
# https://stackoverflow.com/questions/56923895/auto-increment-build-number-using-platformio#56943572

import subprocess

Import("env")

def get_firmware_specifier_build_flag():
	#Uses only annotated tags
	#ret = subprocess.run(["git", "describe"], stdout=subprocess.PIPE, text=True)
	#Uses any tags
	ret = subprocess.run(["git", "describe", "--tags"], stdout=subprocess.PIPE, text=True)
	build_version = ret.stdout.strip()
	build_flag = "-D AUTO_VERSION=\\\"" + build_version + "\\\""
	print ("Firmware Revision: " + build_version)
	return (build_flag)

env.Append(
	BUILD_FLAGS=[get_firmware_specifier_build_flag()]
)
