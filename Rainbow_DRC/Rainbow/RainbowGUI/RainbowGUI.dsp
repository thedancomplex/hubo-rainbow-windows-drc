# Microsoft Developer Studio Project File - Name="RainbowGUI" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Application" 0x0101

CFG=RAINBOWGUI - WIN32 RELEASE
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "RainbowGUI.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "RainbowGUI.mak" CFG="RAINBOWGUI - WIN32 RELEASE"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "RainbowGUI - Win32 Release" (based on "Win32 (x86) Application")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
MTL=midl.exe
RSC=rc.exe
# PROP BASE Use_MFC 6
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 6
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "Release"
# PROP Intermediate_Dir "Release"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MD /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_WINDOWS" /D "_AFXDLL" /Yu"stdafx.h" /FD /c
# ADD CPP /nologo /MD /W3 /GX /O2 /I "C:\Program Files\IntervalZero\RTX\include" /D "WIN32" /D "NDEBUG" /D "_WINDOWS" /D "_AFXDLL" /D "_MBCS" /FR /Yu"stdafx.h" /FD /c
# ADD BASE MTL /nologo /D "NDEBUG" /mktyplib203 /win32
# ADD MTL /nologo /D "NDEBUG" /mktyplib203 /win32
# ADD BASE RSC /l 0x409 /d "NDEBUG" /d "_AFXDLL"
# ADD RSC /l 0x409 /d "NDEBUG" /d "_AFXDLL"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 /nologo /subsystem:windows /machine:I386
# ADD LINK32 rtapi_w32.lib winmm.lib /nologo /subsystem:windows /machine:I386 /libpath:"C:\Program Files\IntervalZero\RTX\lib"
# Begin Target

# Name "RainbowGUI - Win32 Release"
# Begin Group "Source Files"

# PROP Default_Filter "cpp;c;cxx;rc;def;r;odl;idl;hpj;bat"
# Begin Source File

SOURCE=.\BipedDlg.cpp
# End Source File
# Begin Source File

SOURCE=.\CommThread.cpp
# End Source File
# Begin Source File

SOURCE=.\DemoTest2Dlg.cpp
# End Source File
# Begin Source File

SOURCE=.\DemoTestDlg.cpp
# End Source File
# Begin Source File

SOURCE=.\DrcQuadDlg.cpp
# End Source File
# Begin Source File

SOURCE=.\DRCtestDlg.cpp
# End Source File
# Begin Source File

SOURCE=.\EncoderDlg.cpp
# End Source File
# Begin Source File

SOURCE=.\InitHUBO2Dlg.cpp
# End Source File
# Begin Source File

SOURCE=.\JMCDlg.cpp
# End Source File
# Begin Source File

SOURCE=.\JointsDlg.cpp
# End Source File
# Begin Source File

SOURCE=.\LStatusDlg.cpp
# End Source File
# Begin Source File

SOURCE=.\PresetDlg.cpp
# End Source File
# Begin Source File

SOURCE=.\RainbowGUI.cpp
# End Source File
# Begin Source File

SOURCE=.\RainbowGUI.rc
# End Source File
# Begin Source File

SOURCE=.\RainbowGUIDlg.cpp
# End Source File
# Begin Source File

SOURCE=.\SensorDlg.cpp
# End Source File
# Begin Source File

SOURCE=.\StdAfx.cpp
# ADD CPP /Yc"stdafx.h"
# End Source File
# Begin Source File

SOURCE=.\UserDlg.cpp
# End Source File
# Begin Source File

SOURCE=.\UStatusDlg.cpp
# End Source File
# Begin Source File

SOURCE=.\WalkingDlg.cpp
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter "h;hpp;hxx;hm;inl"
# Begin Source File

SOURCE=.\BipedDlg.h
# End Source File
# Begin Source File

SOURCE=.\CommThread.h
# End Source File
# Begin Source File

SOURCE=.\DemoTest2Dlg.h
# End Source File
# Begin Source File

SOURCE=.\DemoTestDlg.h
# End Source File
# Begin Source File

SOURCE=.\DrcQuadDlg.h
# End Source File
# Begin Source File

SOURCE=.\DRCtestDlg.h
# End Source File
# Begin Source File

SOURCE=.\EncoderDlg.h
# End Source File
# Begin Source File

SOURCE=.\InitHUBO2Dlg.h
# End Source File
# Begin Source File

SOURCE=.\JMCDlg.h
# End Source File
# Begin Source File

SOURCE=.\JointsDlg.h
# End Source File
# Begin Source File

SOURCE=.\LStatusDlg.h
# End Source File
# Begin Source File

SOURCE=.\PresetDlg.h
# End Source File
# Begin Source File

SOURCE=.\RainbowGUI.h
# End Source File
# Begin Source File

SOURCE=.\RainbowGUIDlg.h
# End Source File
# Begin Source File

SOURCE=.\Resource.h
# End Source File
# Begin Source File

SOURCE=.\SensorDlg.h
# End Source File
# Begin Source File

SOURCE=.\StdAfx.h
# End Source File
# Begin Source File

SOURCE=.\UserDlg.h
# End Source File
# Begin Source File

SOURCE=.\UStatusDlg.h
# End Source File
# Begin Source File

SOURCE=.\WalkingDlg.h
# End Source File
# End Group
# Begin Group "Resource Files"

# PROP Default_Filter "ico;cur;bmp;dlg;rc2;rct;bin;rgs;gif;jpg;jpeg;jpe"
# Begin Source File

SOURCE=.\res\RainbowGUI.ico
# End Source File
# Begin Source File

SOURCE=.\res\RainbowGUI.rc2
# End Source File
# End Group
# Begin Source File

SOURCE=.\ReadMe.txt
# End Source File
# End Target
# End Project
