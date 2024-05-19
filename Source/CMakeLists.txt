#plugin build file
cmake_minimum_required(VERSION 3.5.0)

#include common rules
include(../PluginRules.cmake)

#add sources, not including OpenEphysLib.cpp
add_sources(${PLUGIN_NAME}
	devices/rhythm-api/okFrontPanelDLL.h
	devices/rhythm-api/rhd2000datablock.cpp
	devices/rhythm-api/rhd2000datablock.h
	devices/rhythm-api/rhd2000evalboard.cpp
	devices/rhythm-api/rhd2000evalboard.h
	devices/rhythm-api/rhd2000registers.cpp
	devices/rhythm-api/rhd2000registers.h
	devices/oni/rhythm-api/okFrontPanelDLL.h
	devices/oni/rhythm-api/rhd2000ONIdatablock.cpp
	devices/oni/rhythm-api/rhd2000ONIdatablock.h
	devices/oni/rhythm-api/rhd2000ONIboard.cpp
	devices/oni/rhythm-api/rhd2000ONIboard.h
	devices/oni/rhythm-api/rhd2000ONIregisters.cpp
	devices/oni/rhythm-api/rhd2000ONIregisters.h
	devices/AcquisitionBoard.h
	devices/oni/AcqBoardONI.h
	devices/oni/AcqBoardONI.cpp
	devices/opalkelly/AcqBoardOpalKelly.h
	devices/opalkelly/AcqBoardOpalKelly.cpp
	devices/simulated/AcqBoardSim.h
	devices/simulated/AcqBoardSim.cpp
	devices/Headstage.h
	devices/oni/HeadstageONI.h
	devices/oni/HeadstageONI.cpp
	devices/opalkelly/HeadstageOpalKelly.h
	devices/opalkelly/HeadstageOpalKelly.cpp
	devices/simulated/HeadstageSim.h
	devices/simulated/HeadstageSim.cpp
	devices/ImpedanceMeter.h
	devices/oni/ImpedanceMeterONI.h
	devices/oni/ImpedanceMeterONI.cpp
	devices/opalkelly/ImpedanceMeterOpalKelly.h
	devices/opalkelly/ImpedanceMeterOpalKelly.cpp
	devices/simulated/ImpedanceMeterSim.h
	devices/simulated/ImpedanceMeterSim.cpp
	UI/ChannelComponent.h
	UI/ChannelComponent.cpp
	UI/ChannelList.h
	UI/ChannelList.cpp
	UI/ChannelCanvas.h
	UI/ChannelCanvas.cpp
	DeviceThread.cpp
	DeviceThread.h
	DeviceEditor.cpp
	DeviceEditor.h
	AcqBoardOutput.h
	AcqBoardOutput.cpp
	AcqBoardOutputEditor.h
	AcqBoardOutputEditor.cpp
	)

#optional: create IDE groups
plugin_create_filters()
