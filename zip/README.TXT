*************************************
*       Sim_logger version 1.18     *
*************************************

Author: Ian Forster-Lewis
Homepage: http://carrier.csi.cam.ac.uk/forsterlewis/soaring/sim/fsx/dev/sim_logger/

This is the FSX add-on saves IGC flight log files in any
folder you specify.  These can be viewed in a variety of log analysis programs,
including:

* FSZweverView2 (download from www.virtualsoaring.org)

* IGCview ( http://carrier.csi.cam.ac.uk/forsterlewis/soaring/igcview/igcview.html )
    clunky, but web based and can display/race multiple logs

* Google Earth (trace converted by http://www.gpsvisualizer.com )

* http://www.everytrail.com
   e.g. http://www.everytrail.com/view_trip.php?trip_id=24425

* http://ywtw.de/igcsimen.html
     animated Google Earth tool that 'flies' your trace within Google Earth.

This add-on REPLACES sim_probe if you have that installed, and you should have CumulusX v1.5 onwards.

CumulusX! is available at http://www.luerkens.homepage.t-online.de/peter/

***************************************************************************

INSTALLATION

  (1) INSTALL THE VISUAL C++ 2008 RUNTIME LIBRARY (vcredist_x86.exe)
      Download and install the file from Microsoft:

      http://www.microsoft.com/DOWNLOADS/details.aspx?FamilyID=9b2da534-3e03-4391-8a4d-074b9f2bc1bf&displaylang=en

      A copy of the file vcredist_x86.exe is included in this zip file if for some reason you have a problem
      with the above link.

  (2) INSTALL THE SIM_LOGGER.EXE FILE:
      From the Windows 'start' menu, open "My Computer" and navigate through the folders to
      "C:\Program Files\Microsoft Games\Microsoft Flight Simulator X"

      Open the sim_logger.zip file (i.e. the file containing this readme)
      with Windows Explorer

      drag the 'Modules' folder
      into your base FSX program folder, i.e. you should end up with
      "C:\Program Files\Microsoft Games\Microsoft Flight Simulator X\Modules\sim_logger\sim_logger.exe"

  (3) ADD SIM_LOGGER.EXE TO YOUR FSX AUTOSTART FILE "EXE.XML":

      If this is the *first* time you've ever added an add-on module to FSX, you may not
      have an "exe.xml" file at all, in which case you will have to create one using 
      Notepad.  Otherwise you can use Notepad to simply edit it. A sample "exe.xml" file
      is at the end of this readme.

         The "exe.xml" file is at the following locations:

            on XP:
   
      "<Drive>:\Documents and Settings\<username>\Application Data\Microsoft\FSX\exe.xml"


            on Vista:

      "<Drive>:\Users\<username>\AppData\Roaming\Microsoft\FSX\exe.xml"

      (or on either system, use Start->Run "%APPDATA%\microsoft\fsx")
      
      Note the "Application Data" or "AppData" folder can be 'hidden' by default in Windows and
      in Windows Explorer you might have to go into Tools->Folder Options for the
      "<Drive>:\Documents and Settings\<username>" folder and select "show hidden files and folders".

      You need to add a "<Launch.Addon>" section for sim_logger.exe, which you can cut-and-paste
      from the sample below.  For completeness I've included the entire file including the entries
      for CumulusX! and sim_logger.  Yours may have *additional* entries if you have other addons
      *or* if you have FSX Deluxe/Professional you will have an entry for the Object Placement Tool.

      The sim_probe entry will look something like the following. Note the 'CommandLine' entry is 
      *new* for this version of sim_probe, and you can specify *any* folder for saving your IGC
      log files.  Note the use of quotes and double-backslashes, particularly the "\\" at the *end*
      of the log folder name. Note the use of quotes and double-backslashes, particularly the 
      "\\" at the *end* of the log folder name. There. I hope you're paying attention.

      <Launch.Addon>
        <Name>sim_logger</Name>
        <Disabled>False</Disabled>
        <ManualLoad>False</ManualLoad>
        <Path>Modules\sim_logger\sim_logger.exe</Path>
        <CommandLine>log="Modules\\sim_logger\\logs\\"</CommandLine>
      </Launch.Addon>


  (4) Start an FSX flight, and you should see a new "Add-ons".."Sim_logger" menu item.

  (5) The IGC files will be saved as <tail number>_<flight name>_<date>_<time>.igc in the folder of your choice.
      It is worth going into the aircraft selection options and setting the tail number of the
      DG808S to your competition number.  I'm B21.

Good luck.

Ian Forster-Lewis


****************************************************************************************************
******************         SAMPLE EXE.XML FILE       ***********************************************
****************************************************************************************************

<?xml version="1.0" encoding="Windows-1252"?>

<SimBase.Document Type="Launch" version="1,0">
  <Descr>Launch</Descr>
  <Filename>exe.xml</Filename>
  <Launch.ManualLoad>False</Launch.ManualLoad>
  
  <Launch.Addon>
    <Disabled>False</Disabled>
    <ManualLoad>False</ManualLoad>
    <Name>CumulusX!</Name>
    <Path>Modules\CumulusX!\CumulusX.exe</Path>
    <CommandLine></CommandLine>
    <NewConsole>True</NewConsole>
  </Launch.Addon>
  
  <Launch.Addon>
    <Disabled>False</Disabled>
    <ManualLoad>False</ManualLoad>
    <Name>sim_logger</Name>
    <Path>Modules\sim_logger\sim_logger.exe</Path>
    <CommandLine>log="Modules\\sim_logger\\logs\\"</CommandLine>
  </Launch.Addon>
</SimBase.Document>


