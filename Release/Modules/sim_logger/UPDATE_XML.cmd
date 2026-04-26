@echo off

rem This cmd script will update your %APPDATA%\Microsoft\FSX\exe.xml and/or dll.xml, using the source files in this folder

rem Usage: update_fsx_xml <exe.xml|dll.xml> program name

rem For exe.xml, update will CREATE a new file if one doesn't exist, using new_exe.xml
rem or update will UPDATE the contents with the <Launch.Addon> section from existing_exe.xml

.\install\update_fsx_xml exe.xml sim_logger.exe

rem Similarly for dll.xml:

rem .\install\update_fsx_xml dll.xml b21_vario.dll

pause

