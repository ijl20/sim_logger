//------------------------------------------------------------------------------
//						sim_logger
//  FSX IGC-standard logger
//  
//  Description:
//              reads the aircraft lat/long/alt and timestamp and writes an IGC-format log file
//
//              Written by Ian Forster-Lewis www.forsterlewis.com
//------------------------------------------------------------------------------

#include <windows.h>
#include <tchar.h>
#include <stdio.h>
#include <stdlib.h>
#include <strsafe.h>
#include <math.h>
#include <time.h>
#include <sys/types.h>
#include <sys/timeb.h>
#include <io.h>
#include <shlobj.h>

#include "SimConnect.h"

// sim_logger version 
double version = 2.66;

//********************************************************************************
//********************   VERSION HISTORY          ********************************
//********************************************************************************
// 2.21  * pitch fixed
//       * clean_string and menus now international
// 2.20  * completed interp code
// 2.19  * interp code (not finished)
// 2.18  * set igc file timestamp to load of flt file
// 2.16  * language ini file
// 2.15  * pitch updates
//       * default_aircraft in sim_logger.ini
//       * AI aircraft retry to default if 1st create fails
// 2.14  * disable FSX thermals
// 2.12  * load sim_logger.ini
// 2.11  * remove AI objects at end of tracklog
//       * set atc_id with user competition id
// 2.10  * menus - detail, disable/enable
// 2.09  * improving menus - added tracklog info
// 2.07  * improving menus
// 2.06  * added bank slew
//       * now loads tracklog files from FLT folder
//       * initial implementation of basic FSX menus
// 2.05  * implemented 'following' slew algorithm fpor heading with the object moving
//         towards the ReplayPoint predicted 4 seconds ahead
// 2.03  * developing slew movement for ai objects - al function embedded
// 2.02  * moving the AI object using slew mode, to calibrate slew rate
// 2.00  * reads test file ai.igc and replays SimProbe object from seatac

// 1.19: * autosave on "plane crash"
//       * detect load of "Previous flight.FLT" or FLT file not found and
//         do not overwrite FLT/WX pathnames and checksums
//       * put ATC_ID into pilot name instead of 'not recorded', also ATC_TYPE

//********************************************************************************

char sim_connect_string[100]; // set to "Sim_connect vX.XX"

// 'debug' parameters from command line that control various levels of diagnostic output
bool debug_info = false; // default 'debug' level - prints out startup, lift, errors
bool debug = false; // prints out lift factors as sim_probe runs
bool debug_calls = false; // greater level of debug - procedure calls
bool debug_events = false; // greater level of debug - events

bool menu_show_text = false; // boolean to decide whether to display debug text in FSX window

const int MAXBUF = 1000; // max length of an IGC file line or a filename
const int MAXC = 20; // max number of C records collectable from PLN file
const int IGC_TICK_COUNT = 4; // log every 4 seconds
const INT32 IGC_MAX_RECORDS = 40000; // log a maximum of this many 'B' records.
//debug - this const set short for testing
const INT32 IGC_MIN_RECORDS = 150; // don't auto-save an IGC file if it is short
const INT32 IGC_MIN_FLIGHT_SECS_TO_LANDING = 80; // don't trigger a log save on landing unless
                                                 // airborne for at least 80 seconds
// Language strings
char lang_save[MAXBUF] = "SAVE tracklog file now";
char lang_files[MAXBUF] = "Manage tracklogs for current flight...";
//char lang_folders[MAXBUF] = "View tracklog folders...";
char lang_restart[MAXBUF] = "RESTART igc tracklog from here";
char lang_replay_title[MAXBUF] = "Sim_logger replay";
char lang_replay_tracklog_title[MAXBUF] = "Sim_logger replay -- Tracklog Info";
char lang_tracklog_select[MAXBUF] = "Select file to manage:";
char lang_next_tracklogs[MAXBUF] = "Next tracklogs in this folder...";
char lang_previous_tracklogs[MAXBUF] = "Previous tracklogs in this folder...";
char lang_detail_checksums[MAXBUF] = "DETAIL checksums for this tracklog...";
char lang_disable_tracklog[MAXBUF] = "DISABLE this tracklog from sim_logger replay";
char lang_enable_tracklog[MAXBUF] = "ENABLE this tracklog for sim_logger replay";
char lang_delete_tracklog[MAXBUF] = "DELETE this tracklog";
char lang_cancel[MAXBUF] = "Cancel...";
char lang_return[MAXBUF] = "Return...";
char lang_blank_line[MAXBUF] = "--";
char lang_checksum_ok[MAXBUF] = "The sim_logger checksum is OK";
char lang_checksum_not_found[MAXBUF] = "No checksum ('G') record in this tracklog";
char lang_checksum_too_short[MAXBUF] = "Sim_logger could not verify this tracklog";
char lang_checksum_failed[MAXBUF] = "Sim_logger could not verify this tracklog";
char lang_checksum_file_error[MAXBUF] = "File i/o error reading this tracklog";
char lang_no_replay[MAXBUF] = " (NO REPLAY)";
char lang_reset[132] = "Tracklog has been reset to start from here...";
char lang_weather[MAXBUF] = "[WEATHER] ";
char lang_flight[MAXBUF] = "[FLIGHT] ";
char lang_aircraft[MAXBUF] = "[AIRCRAFT] ";
char lang_enable_replay[MAXBUF] = "ENABLE replay of tracklogs";
char lang_disable_replay[MAXBUF] = "ENABLE replay of tracklogs";

// INI file variables - defaults in load_ini()
wchar_t ini_log_directory[MAXBUF];
bool ini_enable_replay = true;
wchar_t ini_pilot_name[MAXBUF];
wchar_t ini_aircraft_id[MAXBUF];
bool ini_disable_fsx_thermals = true;
wchar_t ini_default_aircraft[MAXBUF];
wchar_t ini_language[MAXBUF];
double ini_pitch_offset; // pitch adjustment to apply to AI aircraft
double ini_pitch_min; // max low-speed pitch in radians (negative)
double ini_pitch_max; // max high-speed pitch in radians (positive) 
double ini_pitch_v_zero; // speed in m/s for pitch=0;


// these are the strings used to 'DISABLE' and 'DELETE' tracklogs
// the string is inserted before the '.igc' e.g. 'myfile[X].igc'
const char tracklog_disable_string[] = "[X]";
const char tracklog_delete_string[] = "[XX]";
const wchar_t tracklog_skip_string[] = L"[X";

// on startup, sim_logger checks for the ThermalDescriptions.xml file
bool fsx_thermals_enabled = false;

// full pathnames to each of the files
wchar_t FSXBASE[MAXBUF]; // pathname to FSX base folder (current dir when sim_logger is loaded)
char flt_pathname[MAXBUF] = "";
char air_pathname[MAXBUF] = ""; 
char pln_pathname[MAXBUF] = ""; // we don't have to checksum the .PLN file
char wx_pathname[MAXBUF] = "";
char cmx_pathname[MAXBUF] = ""; // path to CumulusX config file
char cfg_pathname[MAXBUF] = ""; // path to aircraft.cfg config file - same folder as AIR
char xml_pathname[MAXBUF] = ""; // path to mission XML file - same folder as FLT
wchar_t MYDOCS[MAXBUF]; // pathname to user "My Documents" folder
wchar_t FSXFILES[MAXBUF]; // path to "Flight Simulator X Files" folder

// shorter names for the files (folder/filename)
char flt_name[MAXBUF] = ""; 
char air_name[MAXBUF] = ""; 
char pln_name[MAXBUF] = "";
char wx_name[MAXBUF] = "";
char cmx_name[MAXBUF] = "";
char cfg_name[MAXBUF] = "";
char xml_name[MAXBUF] = "";

// directories parsed from file load pathnames
wchar_t flt_directory[MAXBUF] = L""; 

// length of checksum string
const int CHKSUM_CHARS = 6;

// printable checksum strings
char chksum[CHKSUM_CHARS+1]     = "000000";
char chksum_flt[CHKSUM_CHARS+1] = "000000";
char chksum_air[CHKSUM_CHARS+1] = "000000";
char chksum_wx[CHKSUM_CHARS+1]  = "000000";
char chksum_cmx[CHKSUM_CHARS+1] = "000000";
char chksum_cfg[CHKSUM_CHARS+1] = "000000"; // checksum for aircraft.cfg
char chksum_xml[CHKSUM_CHARS+1] = "000000"; // checksum for mission xml file
char chksum_all[CHKSUM_CHARS+1] = "000000"; // combined checksum for aircraft.cfg

// PLN data
int c_wp_count = 0; // count of C waypoints (#C records = this + 3)
char c_landing[MAXBUF];
char c[MAXC][MAXBUF];

// aircraft strings
char ATC_ID[MAXBUF];
char ATC_TYPE[MAXBUF];
char TITLE[MAXBUF];

// .FLT file load timestamp YYYY-MM-DD_HHMM
wchar_t flt_load_time[MAXBUF] = L"";

// CumulusX code - set to non-zero if CX is locked
DWORD cx_code = 0;

// Weather code - set to non-zero if Wx is unchanged by user after WX file load
DWORD wx_code = 0;

// ThermalDescriptions.xml code - set to non-zero if file removed
DWORD therm_code = 0;

int     quit = 0;
HANDLE  hSimConnect = NULL;

static enum EVENT_ID {
    EVENT_SIM_START,
	EVENT_FLIGHT,
	EVENT_AIRCRAFT, // get event when aircraft loaded
	EVENT_FLIGHTPLAN,     // get event when flightplan loaded
	EVENT_WEATHER,        // get event when weather mode changed
	EVENT_MISSIONCOMPLETED,
	EVENT_CRASHED,        // pilot has just crashed his aircraft
// events for the FSX menus put up by logger under add-ons
    EVENT_LOGGER_MENU,
    EVENT_REPLAY_MENU,
	EVENT_MENU_FOLDERS,
	EVENT_MENU_FOLDERS_SELECTED0, // two events for folders text menu
	EVENT_MENU_FOLDERS_SELECTED1,
	EVENT_MENU_TRACKLOGS,
	EVENT_MENU_TRACKLOGS_SELECTED0, // we have two of these event and toggle
	EVENT_MENU_TRACKLOGS_SELECTED1, // between then to avoid FSX menu remove
    EVENT_MENU_TRACKLOG_INFO,
    EVENT_MENU_TRACKLOG_DETAIL,
	EVENT_MENU_WRITE_LOG,
	EVENT_MENU_RESTART, // reset the tracklog to 0 points, but keep other data
	EVENT_MENU_TEXT,
// ai control events
	EVENT_FREEZE_LATLONG,
	EVENT_FREEZE_ALTITUDE,
	EVENT_FREEZE_ATTITUDE,	
	EVENT_OBJECT_REMOVED,	
	EVENT_SLEW_ON,	
    EVENT_AXIS_SLEW_AHEAD_SET,
    EVENT_AXIS_SLEW_ALT_SET,
    EVENT_AXIS_SLEW_HEADING_SET,
    EVENT_AXIS_SLEW_BANK_SET,
    EVENT_AXIS_SLEW_PITCH_SET,
    // debug
    EVENT_SLEW_ALTIT_UP_SLOW,
	// gear events
	EVENT_GEAR_UP,
	EVENT_GEAR_DOWN,

// the following are keystroke events useed in testing
    EVENT_Z,
    EVENT_X,
    EVENT_C,
    EVENT_V,
// Cx custom event
	EVENT_CX_CODE,
};

static enum DATA_REQUEST_ID {
    REQUEST_USER_POS,
	REQUEST_AI_RELEASE,
	REQUEST_AI_CREATE = 0x00100000,
	REQUEST_AI_POS = 0x00200000, // offsets create a range of enums
	REQUEST_AI_REMOVE = 0x00300000,
	REQUEST_STARTUP_DATA = 0x00400000,
	REQUEST_AIRCRAFT_DATA,
};

// GROUP_ID and INPUT_ID are used for keystroke events in testing
static enum GROUP_ID {
    GROUP_ZX,
	GROUP_MENU
};

static enum INPUT_ID {
    INPUT_ZX
};

static enum DEFINITION_ID {
    DEFINITION_USER_POS,
    DEFINITION_AI_POS,
	DEFINITION_AI_MOVE,
    DEFINITION_AI_SET_DATA,
	DEFINITION_STARTUP,
	DEFINITION_AIRCRAFT,
};

struct UserStruct {
    double latitude;
    double longitude;
    double altitude; // meters
	INT32  sim_on_ground;
	INT32  zulu_time; // seconds
	INT32  rpm; // engine revs per min
};

// FSX FLIGHT DATA NEEDED FOR IGC LOG, e.g. startup time, aircraft info
struct StartupStruct {
	INT32 start_time;
	INT32 zulu_day;
	INT32 zulu_month;
	INT32 zulu_year;
};

struct AircraftStruct { // variable length strings ATC ID, ATC TYPE, TITLE
	char strings[1];
};

// create var to hold user plane position
UserStruct user_pos;

// startup_data holds the data picked up from FSX at the start of each flight
StartupStruct startup_data;

//*******************************************************************************

// this very useful function converts in(UNICODE) to s(ascii) and strips odd chars
void clean_string(char *s, wchar_t *in) {
	unsigned int i = 0;
    //const wchar_t *PLN_WCHARS = L"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz.<>, []-*()_+\\/üÜä";
	while (i<wcslen(in)) {
        //if (wcschr(PLN_WCHARS,in[i])==NULL) 
		//	s[i] = ' ';
		//else
			s[i] = char(in[i]);
		i++;
    }
	s[i] = '\0';
}


//*******************************************************************************
//****************************  INI FILE ****************************************
//*******************************************************************************

wchar_t INI_SUB_PATH[] = L"\\Modules\\sim_logger\\sim_logger.ini";
// LANG_SUB_PATH has ini_language added plus .ini
wchar_t LANG_SUB_PATH[] = L"\\Modules\\sim_logger\\lang_";

wchar_t ini_default[MAXBUF] = L"";

wchar_t INI_APP_NAME[] = L"sim_logger";

void load_ini() {
	wchar_t ini_path[MAXBUF];
	wchar_t buf[MAXBUF];
	DWORD length;
	float float_buf;
	// build full sim_logger.ini path
	wcscpy_s(ini_path, MAXBUF, FSXBASE);
	wcscat_s(ini_path, MAXBUF, INI_SUB_PATH);
	// log_directory
	length = GetPrivateProfileString(INI_APP_NAME,
										L"log_directory",
										ini_default,
										buf,
										MAXBUF,
										ini_path);
	wcscpy_s(ini_log_directory, MAXBUF, buf);
	// if (debug) wprintf(L"INI: log_directory = \"%s\"\n", ini_log_directory);

	// enable_replay
	length = GetPrivateProfileString(INI_APP_NAME,
										L"enable_replay",
										ini_default,
										buf,
										MAXBUF,
										ini_path);
	if (_wcsicmp(buf, L"")==0) ini_enable_replay = true;
	else if (_wcsicmp(buf, L"false")==0) ini_enable_replay = false;
	else if (_wcsicmp(buf, L"0")==0) ini_enable_replay = false;
	else ini_enable_replay = true;
	//if (debug) printf("INI: enable_replay = %s\n", (ini_enable_replay) ? "true":"false");

	// disable_fsx_thermals
	length = GetPrivateProfileString(INI_APP_NAME,
										L"disable_fsx_thermals",
										ini_default,
										buf,
										MAXBUF,
										ini_path);
	if (_wcsicmp(buf, L"")==0) ini_disable_fsx_thermals = true;
	else if (_wcsicmp(buf, L"false")==0) ini_disable_fsx_thermals = false;
	else if (_wcsicmp(buf, L"0")==0) ini_disable_fsx_thermals = false;
	else ini_disable_fsx_thermals = true;
	//if (debug) printf("INI: disable_fsx_thermals = %s\n", (ini_disable_fsx_thermals) ? "true":"false");

	// pilot_name
	length = GetPrivateProfileString(INI_APP_NAME,
										L"pilot_name",
										ini_default,
										buf,
										MAXBUF,
										ini_path);
	wcscpy_s(ini_pilot_name, MAXBUF, buf);
	// if (debug) wprintf(L"INI: pilot_name = \"%s\"\n", ini_pilot_name);

	// aircraft_id
	length = GetPrivateProfileString(INI_APP_NAME,
										L"aircraft_id",
										ini_default,
										buf,
										MAXBUF,
										ini_path);
	wcscpy_s(ini_aircraft_id, MAXBUF, buf);
	// if (debug) wprintf(L"INI: aircraft_id = \"%s\"\n", ini_aircraft_id);

	// default_aircraft
	length = GetPrivateProfileString(INI_APP_NAME,
										L"default_aircraft",
										L"DG808S",
										buf,
										MAXBUF,
										ini_path);
	wcscpy_s(ini_default_aircraft, MAXBUF, buf);
	// if (debug) wprintf(L"INI: default_aircraft = \"%s\"\n", ini_default_aircraft);

	// language
	length = GetPrivateProfileString(INI_APP_NAME,
										L"language",
										L"en",
										buf,
										MAXBUF,
										ini_path);
	wcscpy_s(ini_language, MAXBUF, buf);
	// if (debug) wprintf(L"INI: language = \"%s\"\n", ini_language);

	// PITCH adjustments
	length = GetPrivateProfileString(INI_APP_NAME,
										L"pitch_offset",
										ini_default,
										buf,
										MAXBUF,
										ini_path);
	float_buf = 0; // default pitch_offset = 0
	swscanf_s(buf,L"%f",&float_buf);
	ini_pitch_offset = float_buf;

	length = GetPrivateProfileString(INI_APP_NAME,
										L"pitch_min",
										ini_default,
										buf,
										MAXBUF,
										ini_path);
	float_buf = (float)-0.3; // default pitch_min = -0.3
	swscanf_s(buf,L"%f",&float_buf);
	ini_pitch_min = float_buf;

	length = GetPrivateProfileString(INI_APP_NAME,
										L"pitch_max",
										ini_default,
										buf,
										MAXBUF,
										ini_path);
	float_buf = (float)0.1; // default pitch_max = 0.1
	swscanf_s(buf,L"%f",&float_buf);
	ini_pitch_max = float_buf;

	length = GetPrivateProfileString(INI_APP_NAME,
										L"pitch_v_zero",
										ini_default,
										buf,
										MAXBUF,
										ini_path);
	float_buf = 30;
	swscanf_s(buf,L"%f",&float_buf);
	ini_pitch_v_zero = float_buf;


	if (debug) wprintf(L"INI: pitch_offset = %f, pitch_min=%f,pitch_max=%f, pitch_v_zero=%f\n",
		ini_pitch_offset, ini_pitch_min, ini_pitch_max, ini_pitch_v_zero);
}

void ini_write_string(wchar_t *key, wchar_t *value) {
	wchar_t ini_path[MAXBUF];
	BOOL success;
	// build full sim_logger.ini path
	wcscpy_s(ini_path, MAXBUF, FSXBASE);
	wcscat_s(ini_path, MAXBUF, INI_SUB_PATH);

	// enable_replay
	success = WritePrivateProfileString(INI_APP_NAME,
										key,
										value,
										ini_path);
	//if (debug) printf("INI: enable_replay = %s\n", (ini_enable_replay) ? "true":"false");

}

void get_lang_string(char *key, char *value, char* def, wchar_t *wpath) {
	wchar_t wbuf[MAXBUF];
    wchar_t wkey[MAXBUF];
    wchar_t wdef[MAXBUF];
    size_t wlen;
    				// convert key to unicode
	mbstowcs_s(&wlen,wkey,MAXBUF,key,MAXBUF);
    				// convert default to unicode
	mbstowcs_s(&wlen,wdef,MAXBUF,def,MAXBUF);

	DWORD length = GetPrivateProfileString(INI_APP_NAME,
										wkey,
										wdef,
										wbuf,
										MAXBUF,
										wpath);
	clean_string(value, wbuf);
	//if (debug) printf("LANG: %s = \"%s\"\n", key, value);
}

void load_lang() {
	wchar_t ini_path[MAXBUF];

	// build full sim_logger.ini path
	wcscpy_s(ini_path, MAXBUF, FSXBASE);
	wcscat_s(ini_path, MAXBUF, LANG_SUB_PATH);
	wcscat_s(ini_path, MAXBUF, ini_language);
	wcscat_s(ini_path, MAXBUF, L".ini");

    get_lang_string("save",lang_save,
                    "SAVE tracklog file now", ini_path);
    get_lang_string("files",lang_files,
                    "Manage tracklogs for current flight...", ini_path);
    get_lang_string("restart",lang_restart,
                    "RESTART igc tracklog from here", ini_path);
    get_lang_string("replay_title",lang_replay_title,
                    "Sim_logger replay", ini_path);
    get_lang_string("replay_tracklog_title",lang_replay_tracklog_title,
                    "Sim_logger replay -- Tracklog Info", ini_path);
    get_lang_string("tracklog_select",lang_tracklog_select,
                    "Select file to manage:", ini_path);
    get_lang_string("next_tracklogs",lang_next_tracklogs,
                    "Next tracklogs in this folder...", ini_path);
    get_lang_string("previous_tracklogs",lang_previous_tracklogs,
                    "Previous tracklogs in this folder...", ini_path);
    get_lang_string("detail_checksums",lang_detail_checksums,
                    "DETAIL checksums for this tracklog...", ini_path);
    get_lang_string("disable_tracklog",lang_disable_tracklog,
                    "DISABLE this tracklog from sim_logger replay", ini_path);
    get_lang_string("enable_tracklog",lang_enable_tracklog,
                    "ENABLE this tracklog for sim_logger replay", ini_path);
    get_lang_string("delete_tracklog",lang_delete_tracklog,
                    "DELETE this tracklog", ini_path);
    get_lang_string("cancel",lang_cancel,
                    "Cancel...", ini_path);
    get_lang_string("return",lang_return,
                    "Return...", ini_path);
    get_lang_string("checksum_ok",lang_checksum_ok,
                    "The sim_logger checksum is OK", ini_path);
    get_lang_string("checksum_not_found",lang_checksum_not_found,
                    "No checksum ('G') record in this tracklog", ini_path);
    get_lang_string("checksum_too_short",lang_checksum_too_short,
                    "Sim_logger could not verify this tracklog", ini_path);
    get_lang_string("checksum_failed",lang_checksum_failed,
                    "Sim_logger could not verify this tracklog", ini_path);
    get_lang_string("checksum_file_error",lang_checksum_file_error,
                    "File i/o error reading this tracklog", ini_path);
    get_lang_string("no_replay",lang_no_replay,
                    " (NO REPLAY)", ini_path);
    get_lang_string("reset",lang_reset,
                    "Tracklog has been reset to start from here...", ini_path);
    get_lang_string("weather",lang_weather,
                    "[WEATHER] ", ini_path);
    get_lang_string("flight",lang_flight,
                    "[FLIGHT] ", ini_path);
    get_lang_string("aircraft",lang_aircraft,
                    "[AIRCRAFT] ", ini_path);
    get_lang_string("enable_replay",lang_enable_replay,
                    "ENABLE replay of tracklogs", ini_path);
    get_lang_string("disable_replay",lang_disable_replay,
                    "DISABLE replay of tracklogs", ini_path);

}

//*******************************************************************************
//****************************  DISABLE FSX THERMALS ****************************

wchar_t ThermalDescriptionsXML[] = L"ThermalDescriptions.xml";
wchar_t ThermalDescriptionsXML_disabled[] = L"ThermalDescriptions[X].xml";

bool disable_fsx_thermals() {
	SetCurrentDirectory(FSXBASE);
	// see if the file actually exists
	if(_waccess_s(ThermalDescriptionsXML, 0) == 0) {
		// file exists - rename it
		if (debug) wprintf(L"ThermalDescriptions.xml file found\n");
		if (ini_disable_fsx_thermals) return true;
		int rc = _wrename(ThermalDescriptionsXML, ThermalDescriptionsXML_disabled);
		if (debug) {
			if (rc==0) wprintf(L"%s renamed to %s\n", ThermalDescriptionsXML, ThermalDescriptionsXML_disabled);
			else {
				perror("Rename error");
				printf("Error renaming ThermalDescriptions.xml\n");
			}
		}
		if (rc==0) return false;
	} else {
		if (debug) wprintf(L"ThermalDescriptions.xml not found\n");
		return false;
	}
	return false;
}

//*******************************************************************************
//********************** CHECKSUM CALCULATION ***********************************
//*******************************************************************************

// return codes from chksum_igc_file()
static enum CHKSUM_RESULT {
    CHKSUM_OK,
	CHKSUM_NOT_FOUND,
	CHKSUM_TOO_SHORT,
	CHKSUM_BAD,
	CHKSUM_FILE_ERROR,
};

// number of characters to include in checksum
const int CHK_CHARS = 63; // number of chars in chk_source and chk_map
// list of CHK_CHARS characters to be mapped into checksum (other chars ignored)
char *chk_source = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ.abcdefghijklmnopqrstuvwxyz";
// map table for char->int 0..(CHK_CHARS-1)
int chk_map[CHK_CHARS]    = { 14,46,51,8,26,2,32,39,29,
							 37,4,44,20,61,22,58,16,25,
							 60,13,31,53,11,50,6,38,41,
							 23,56,17,1,19,45,10,28,15,
							 36,9,57,12,49,33,3,24,30,
							 62,47,5,43,0,27,52,34,55,
							 21,54,59,18,48,35,40,7,42};

// modulo value for chksum_index which increments through the file
const int CHKSUM_MAX_INDEX = 1987;

struct ChksumData {
	int index;
	int num[CHKSUM_CHARS];
};

// incrementally update checksum given current char c
void incr_chksum(ChksumData *chk_data, char c) {
	unsigned int c_pos = 0;
	// convert c to int via chk_source array
	while (c_pos<CHK_CHARS && chk_source[c_pos]!=c) c_pos++;
	// if c not found then simply return (only need checksum valid chars)
	if (c_pos==CHK_CHARS) return;

	// now c_pos is index of c in char_source, get mapped number
	int map_num = chk_map[(c_pos + chk_data->index) % CHK_CHARS];
	for (int i=0; i<CHKSUM_CHARS; i++) {
		chk_data->num[i] = chk_map[(chk_data->num[i]+map_num+i) % CHK_CHARS];
	}
	// Increment checksum_index
	chk_data->index = (chk_data->index + 1) % CHKSUM_MAX_INDEX;
}

// update chksum_num based on input string s
void chksum_string(ChksumData *chk_data, char *s) {
	for (unsigned int i=0; i<strlen(s); i++) 
		incr_chksum(chk_data, s[i]);
}

// update chksum_num based on BINARY input string s
void chksum_binary(ChksumData *chk_data, char *s, int count) {
	for (int i=0; i<count; i++) 
		incr_chksum(chk_data, s[i]);
}

// convert chk_data.num[] into string chksum
void chksum_to_string(char chksum[CHKSUM_CHARS+1], ChksumData chk_data) {
	for (int i=0; i<CHKSUM_CHARS;i++) 
		chksum[i] = chk_source[chk_data.num[i] % 36];
}

void chksum_reset(ChksumData *chk_data) {
	chk_data->index = 1;
	for (int i=0; i<CHKSUM_CHARS;i++) chk_data->num[i]=i;
}

CHKSUM_RESULT chksum_binary_file(char chksum[CHKSUM_CHARS+1], char *filepath) {
	FILE *f;
	errno_t err;
	char buf[MAXBUF];
	int read_count; // number of chars read
	// calculated checksum as sequence of ints 0..CHK_CHARS
	ChksumData chk_data;
	
	chksum_reset(&chk_data);

	if( (err = fopen_s(&f, filepath, "rb")) != 0 ) {
        strcpy_s(chksum, CHKSUM_CHARS+1, "000000");
		return CHKSUM_FILE_ERROR;
	}
	while (!feof(f)) {
		read_count = fread(buf, sizeof(char),sizeof(buf),f);
		chksum_binary(&chk_data, buf, read_count);
	}
	chksum_to_string(chksum, chk_data);
	fclose(f);
	return CHKSUM_OK;
}

// starts_bracket returns -1 if string doesn't have '[' as first non-space char
// otherwise it returns the index of the '[' char
int starts_bracket(char *s) {
    unsigned int i = 0;
    //debug
    //printf("starts_bracket %s\n",s);
    while (i<10 && i<strlen(s)) {
        if (s[i]=='[') return i;        // found '[' => return
        else if (s[i]!=' ') return -1;  // string starts with non-'[' 
        i++;
    }
    return -1;                      
}

bool perf_match(char *line_buf) {
    const int PERF_COUNT = 11;
    char *strings[PERF_COUNT];
    int length[PERF_COUNT]; // number of chars into string to detect
    int pos;
    int i = 0;

    strings[0]="[airplane_geometry]";
    length[0] = 5;
    strings[1]="[flaps.";
    length[1] = 4;
    strings[2]="[flight_tuning]";
    length[2] = 4;
    strings[3]="[water_ballast_system]";
    length[3] = 3;
    strings[4]="[weight_and_balance]";
    length[4] = 3;
    strings[5]="[generalenginedata]";
    length[5] = 11;
    strings[6]="[jet_engine]";
    length[6] = 4;
    strings[7]="[piston_engine]";
    length[7] = 4;
    strings[8]="[propeller]";
    length[8] = 4;
    strings[9]="[turbineenginedata]";
    length[9] = 6;
    strings[10]="[turboprop_engine]";
    length[10] = 6;

    pos = starts_bracket(line_buf);
    //debug
    //printf(" (pos=%+d) ",pos);
    if (pos<0) return false;
    while (i<PERF_COUNT) {
        if ( strncmp(line_buf+pos,strings[i],length[i]) == 0 ) {
            //debug
            //printf("PERF_MATCH %s\n", strings[i]);
            return true;
        }
        i++;
    }
    return false;
}

// chksum_cfg_file calculates a checksum for aircraft.cfg file only including sections
// of the file that affect performance
CHKSUM_RESULT chksum_cfg_file(char chksum[CHKSUM_CHARS+1], char *filepath) {
	FILE *f;
	errno_t err;
	char line_buf[MAXBUF];
    bool in_perf_section = false;

	// calculated checksum as sequence of ints 0..CHK_CHARS
	ChksumData chk_data;
	
	chksum_reset(&chk_data);

	if( (err = fopen_s(&f, filepath, "r")) != 0 ) {
        strcpy_s(chksum, CHKSUM_CHARS+1, "000000");
		return CHKSUM_FILE_ERROR;
	}
    while (fgets(line_buf, MAXBUF, f)!=NULL) {
        //debug
        //if (in_perf_section) printf("    PERF ");
        //else printf("NON-PERF ");
        //printf(">%s",line_buf);
		if (!in_perf_section) in_perf_section = perf_match(line_buf);
        else {
            if (starts_bracket(line_buf)<0)
                chksum_string(&chk_data, line_buf);
            else in_perf_section = perf_match(line_buf);
        }
    }
	chksum_to_string(chksum, chk_data);
	fclose(f);
	return CHKSUM_OK;
}

// This routine is used to *check* the checksum at the end of an IGC file
// the checksum will be stored in the final 'G' record.
// Only alphanumeric characters before the 'G' record contribute to the checksum.
CHKSUM_RESULT chksum_igc_file(char chksum[CHKSUM_CHARS+1], char *filepath) {
	FILE *f;
	errno_t err;
	char line_buf[MAXBUF];
	// calculated checksum as sequence of ints 0..CHK_CHARS
	ChksumData chk_data;
	
	chksum_reset(&chk_data);

    // open file
	if( (err = fopen_s(&f, filepath, "r")) != 0 ) {
		return CHKSUM_FILE_ERROR;
	}
	while (fgets(line_buf, MAXBUF, f)!=NULL) {
		if (line_buf[0]=='G') break;
		//if (strncmp(line_buf,"L FSX GENERAL", 13)==0) printf("%s",line_buf+6);
		chksum_string(&chk_data, line_buf);
	}
    // close file
   	fclose(f);

	if (line_buf[0]!='G') {
			return CHKSUM_NOT_FOUND;
	}

	if (strlen(line_buf)<CHKSUM_CHARS+1) {
			return CHKSUM_TOO_SHORT;
	}
	chksum_to_string(chksum, chk_data);
	for (int i=0; i<CHKSUM_CHARS; i++) {
		if (chksum[i]!=line_buf[i+1]) {
			return CHKSUM_BAD;
		}
	}
	return CHKSUM_OK;
}

CHKSUM_RESULT check_file(char *pfilepath) {
	char chksum[CHKSUM_CHARS+1] = "000000";
	return chksum_igc_file(chksum, pfilepath);
}

// this routine produces a general checksum for the
// FLT, WX, CMX, AIR, aircraft.cfg files
// so if this is correct the user does not have to look at the 
// individual checksums
CHKSUM_RESULT chksum_chksum(char chksum[CHKSUM_CHARS+1]) {
	ChksumData chk_data;

	chksum_reset(&chk_data);

	chksum_string(&chk_data,chksum_flt);
	chksum_string(&chk_data,chksum_air);
	chksum_string(&chk_data,chksum_wx);
	chksum_string(&chk_data,chksum_cmx);
	chksum_string(&chk_data,chksum_cfg);
	chksum_string(&chk_data,chksum_xml);
	if (cx_code==0) 
		chksum_string(&chk_data, "CX UNLOCKED");
	else
		chksum_string(&chk_data, "CX LOCKED");

	if (wx_code==0) 
		chksum_string(&chk_data, "WX UNLOCKED");
	else
		chksum_string(&chk_data, "WX LOCKED");

    if (therm_code==0)
        chksum_string(&chk_data, "THERM FILE PRESENT");
    else
        chksum_string(&chk_data, "NO THERM FILE");

	chksum_to_string(chksum, chk_data);
	return CHKSUM_OK;
}

//*******************************************************************
//*******************************************************************
//******************    PARSE THE PLN FILE **************************
//*******************************************************************

// utility function - copy 'n' chars from src to dest
void cpy(char *dest, int max, char *src, int n) {
    if (n>max) return;
    for (int i=0;i<n;i++) dest[i]=src[i];
    return;
}

CHKSUM_RESULT pln_to_c(char *filepath) {
	FILE *f;
	errno_t err;
	const char LLA_FORMAT[] = "%c %d° %d' %f\", %c %d° %d' %f";

	char line_buf[MAXBUF];
    char s[MAXBUF]; // general buffer
	wchar_t in_buf[MAXCHAR*4]; // line input buffer

    char *ptr_value; // temporary pointer into read line from PLN
    char *ptr_end; // temp pointer to end of value
    int value_length; // length of value up to following '<' char

    char lat_NS = ' ';
    int lat_degs = 0;
    int lat_mins = 0;
    float lat_secs = 0;
    char long_NS = ' ';
    int long_degs = 0;
    int long_mins = 0;
    float long_secs = 0;
    char comma = ','; // temp placeholder for sscanf_s

    //debug
    //printf("parsing %s\n",filepath);

    c_wp_count = 0;
    strcpy_s(c[0],MAXBUF,"");
    strcpy_s(c[1],MAXBUF,"");
    strcpy_s(c_landing,MAXBUF,"");

	if( (err = fopen_s(&f, filepath, "r, ccs=UNICODE")) != 0 ) {
		return CHKSUM_FILE_ERROR;
	}
	// use current PC time for declaration date in C header record
	__time64_t ltime;
	struct tm today;
    _time64(&ltime);
    _localtime64_s( &today, &ltime );
	strftime(s, MAXBUF, "C%d%m%y%H%M%S000000000100NO TASK", &today );
	strcpy_s(c[0],MAXBUF,s);

    while (fgetws(in_buf, MAXBUF, f)!=NULL) {
		//debug
		//wprintf_s(L"- %s",in_buf);
        clean_string(line_buf, in_buf);
        //debug
        //printf("c %s\n",line_buf);
		//continue;
        // now test for each record we want from PLN

        ptr_value = strstr(line_buf,"<Title>");
        if (ptr_value!=NULL) {
            ptr_end = strchr(ptr_value+1, '<');
            if (ptr_end!=NULL) {
                value_length = (int)(ptr_end-ptr_value-7);
                strncpy_s(s,MAXBUF,ptr_value+7,value_length);
                strcpy_s(c[0]+25,MAXBUF-25,s);
				strcat_s(c[0]+25,MAXBUF,"\n");
                //debug
                //printf("Title \"%s\"\n", c[0]+25);
            }
            continue;
        }
        ptr_value = strstr(line_buf,"<DepartureName>");
        if (ptr_value!=NULL) {
            ptr_end = strchr(ptr_value+1, '<');
            if (ptr_end!=NULL) {
                value_length = (int)(ptr_end-ptr_value-15);
                strncpy_s(s,MAXBUF,ptr_value+15,value_length);
                strcpy_s(c[1]+18,MAXBUF-18,s);
				strcat_s(c[1]+18,MAXBUF,"\n");
                //debug
                //printf("DepartureName \"%s\"\n", c[1]+18);
            }
            continue;
        }
        ptr_value = strstr(line_buf,"<DestinationName>");
        if (ptr_value!=NULL) {
            ptr_end = strchr(ptr_value+1, '<');
            if (ptr_end!=NULL) {
                value_length = (int)(ptr_end-ptr_value-17);
                strncpy_s(s,MAXBUF,ptr_value+17,value_length);
                strcpy_s(c_landing+18,MAXBUF-18,s);
				strcat_s(c_landing+18,MAXBUF,"\n");
                //debug
                //printf("DestinationName \"%s\"\n", c_landing+18);
            }
            continue;
        }
        ptr_value = strstr(line_buf,"<DepartureLLA>");
        if (ptr_value!=NULL) {
            ptr_end = strchr(ptr_value+1, '<');
            if (ptr_end!=NULL) {
                sscanf_s(ptr_value+14, LLA_FORMAT,
                    &lat_NS,1, &lat_degs, &lat_mins, &lat_secs, 
                    &long_NS,1, &long_degs, &long_mins, &long_secs);

                lat_secs = lat_secs / 60 * 1000;
                long_secs = long_secs / 60 * 1000;
                sprintf_s(s,MAXBUF,"C%02.2d%02.2d%03.0f%c%03.3d%02.2d%03.0f%c",
                    lat_degs, lat_mins, lat_secs, lat_NS, 
                    long_degs, long_mins, long_secs, long_NS );
                cpy(c[1],MAXBUF,s,18);
                //debug                C  DD    MM   MMM   N   DDD   MM    MMM  E
                //printf("DepartureLLA \"C %02.2d %02.2d %03.0f %c %03.3d %02.2d %03.0f %c\"\n",
                //    lat_degs, lat_mins, lat_secs, lat_NS, 
                //    long_degs, long_mins, long_secs, long_NS );
            }
            continue;
        }
        ptr_value = strstr(line_buf,"<DestinationLLA>");
        if (ptr_value!=NULL) {
            ptr_end = strchr(ptr_value+1, '<');
            if (ptr_end!=NULL) {
                sscanf_s(ptr_value+16, "%c %d° %d' %f\", %c %d° %d' %f",
                    &lat_NS,1, &lat_degs, &lat_mins, &lat_secs,
                    &long_NS,1, &long_degs, &long_mins, &long_secs);

                lat_secs = lat_secs / 60 * 1000;
                long_secs = long_secs / 60 * 1000;
                sprintf_s(s,MAXBUF,"C%02.2d%02.2d%03.0f%c%03.3d%02.2d%03.0f%c",
                    lat_degs, lat_mins, lat_secs, lat_NS, 
                    long_degs, long_mins, long_secs, long_NS );
                cpy(c_landing,MAXBUF,s,18);
                //debug                C  DD    MM   MMM   N   DDD   MM    MMM  E
                //printf("DestinationLLA \"C %02.2d %02.2d %03.0f %c %03.3d %02.2d %03.0f %c\"\n",
                //    lat_degs, lat_mins, lat_secs, lat_NS, 
                //    long_degs, long_mins, long_secs, long_NS );
            }
            continue;
        }
        ptr_value = strstr(line_buf,"<ATCWaypoint ");
        if (ptr_value!=NULL) {
            c_wp_count++;
            ptr_end = strchr(ptr_value, '>');
            if (ptr_end!=NULL) {
                value_length = (int)(ptr_end-ptr_value-18);
                strncpy_s(s,MAXBUF,ptr_value+17,value_length);
                strcpy_s(c[c_wp_count+1]+18,MAXBUF-18,s);
				strcat_s(c[c_wp_count+1]+18,MAXBUF,"\n");
                //debug
                //printf("Waypoint \"%s\"\n", c[c_wp_count+1]+18);
            }
            continue;
        }
        ptr_value = strstr(line_buf,"<WorldPosition>");
        if (ptr_value!=NULL) {
            ptr_end = strchr(ptr_value+1, '<');
            if (ptr_end!=NULL) {
                sscanf_s(ptr_value+15, "%c %d° %d' %f\", %c %d° %d' %f",
                    &lat_NS,1, &lat_degs, &lat_mins, &lat_secs,
                    &long_NS,1, &long_degs, &long_mins, &long_secs);

                lat_secs = lat_secs / 60 * 1000;
                long_secs = long_secs / 60 * 1000;
                sprintf_s(s,MAXBUF,"C%02.2d%02.2d%03.0f%c%03.3d%02.2d%03.0f%c",
                    lat_degs, lat_mins, lat_secs, lat_NS, 
                    long_degs, long_mins, long_secs, long_NS );
                cpy(c[c_wp_count+1],MAXBUF,s,18);
                //debug                C  DD    MM   MMM   N   DDD   MM    MMM  E
                //printf("WorldPosition \"C %02.2d %02.2d %03.0f %c %03.3d %02.2d %03.0f %c\"\n",
                //    lat_degs, lat_mins, lat_secs, lat_NS, 
                //    long_degs, long_mins, long_secs, long_NS );
            }
            continue;
        }
    }
	fclose(f);
	//debug
	//printf("\nParsing PLN completed.\n");
	// inject turnpoint count (waypoints - 2) into c[0] record
	if (c_wp_count>2) {
		sprintf_s(s, MAXBUF,"%02.2d",c_wp_count-2);
		cpy(c[0]+23,MAXBUF-23,s,2);
	} else cpy(c[0]+23,MAXBUF-23,"00",2);
	//return CHKSUM_OK;
    //debug
	if (debug) {
		printf("first C record: %s",c[0]);
		printf("departure:      %s",c[1]);
		for (int i=0; i<c_wp_count; i++) printf("WP:             %s",c[i+2]);
		printf("landing:        %s",c_landing);
	}
	return CHKSUM_OK;

}


//*******************************************************************
//**************** find short filename      *************************

// given path = "C:\foo\bah\myfile.flt"
// then name =  "bah\myfile.txt"

void path_to_name(char name[MAXBUF], char path[MAXBUF]) {
	int i;
	bool found1 = false; // flag to say we've found 1st '\' char
	bool found2 = false;

	// first check if file is there
    if(_access_s(path, 0) != 0) {
        strcpy_s(name, MAXBUF,"not found");
        return;
	}

	// ok we've found the file, so lets abbreviate the name
	i = strlen(path);
	while (i>0 && !found2) {
		if (path[--i]=='\\') { 
			if (!found1) found1 = true;
			else found2 = true;
		}
	}	
	strcpy_s(name, MAXBUF,path+i+1);
}

// convert a full file path to a path to the directory
void path_to_directory(wchar_t dir[MAXBUF], char *path) {
	char s[MAXBUF];
	size_t wlen;

	if (debug) printf("path_to_dir: %s\n",path);
	// first check if file is there
    if(_access_s(path, 0) != 0) {
        wcscpy_s(dir, MAXBUF,L"");
		if (debug) printf("path_to_dir: WTF?? file %s not found\n",path);
        return;
	}
	// find last '\'
	char *ptr_last_slash = strrchr(path, '\\');
	if (ptr_last_slash==NULL) {
        wcscpy_s(dir, MAXBUF,L"");
		if (debug) printf("path_to_dir: No folders in path %s",path);
        return;
	}
	errno_t err = strncpy_s( s, MAXBUF , path, ptr_last_slash - path);
				// convert to unicode
	mbstowcs_s(&wlen,dir,MAXBUF,s,MAXBUF);
}

//*******************************************************************
// igc file logger vars
//*******************************************************************

int igc_tick_counter = 0; // variable to keep track of how many ticks we've counted 0..IGC_TICK_COUNT
INT32 igc_record_count = 0; // count of how many 'B' records we've recorded

INT32 igc_takeoff_time; // note time of last "SIM ON GROUND"->!(SIM ON GROUND) transition
INT32 igc_prev_on_ground = 0;

// flag to confirm a user file save - so we don't auto-save over it
bool igc_saved = false;

// struct of data in an IGC 'B' record
struct igc_b {
	INT32 zulu_time;
	double latitude;
	double longitude;
	double altitude;
    double rpm;
};

// array to hold all the 'B' records
igc_b igc_pos[IGC_MAX_RECORDS];

//**********************************************************************************
//**********************************************************************************
//******* IGC FILE ROUTINES                                                 ********
//**********************************************************************************
//**********************************************************************************


void igc_reset_log() {
	//c_wp_count = 0;
	igc_record_count = 0;
    igc_saved = false;
}

void get_aircraft_data() {
    HRESULT hr;
    // set data request
    hr = SimConnect_RequestDataOnSimObject(hSimConnect, 
                                            REQUEST_AIRCRAFT_DATA, 
                                            DEFINITION_AIRCRAFT, 
                                            SIMCONNECT_OBJECT_ID_USER,
                                            SIMCONNECT_PERIOD_ONCE); 

}

// this routine will be called each time:
//  * sim start
//  * change of aircraft
//  * change of Wx

void get_startup_data() {
    HRESULT hr;
    // set data request
    hr = SimConnect_RequestDataOnSimObject(hSimConnect, 
                                            REQUEST_STARTUP_DATA, 
                                            DEFINITION_STARTUP, 
                                            SIMCONNECT_OBJECT_ID_USER,
                                            SIMCONNECT_PERIOD_ONCE); 

	// now get aircraft data
	get_aircraft_data();
}

void get_user_pos_updates() {
    HRESULT hr;
    if (debug_calls) printf(" ..entering get_user_pos_updates()..");
	// set data request
	hr = SimConnect_RequestDataOnSimObject(hSimConnect, 
											REQUEST_USER_POS, 
											DEFINITION_USER_POS, 
											SIMCONNECT_OBJECT_ID_USER,
											SIMCONNECT_PERIOD_SECOND); 
    if (debug_calls) printf(" ..leaving get_user_pos_updates()..\n");
}

void igc_log_point(UserStruct p) {
	if (igc_record_count<IGC_MAX_RECORDS) {
		if (igc_record_count==0 || p.zulu_time!=igc_pos[igc_record_count-1].zulu_time) {
			igc_pos[igc_record_count].latitude = p.latitude;
			igc_pos[igc_record_count].longitude = p.longitude;
			igc_pos[igc_record_count].altitude = p.altitude;
			igc_pos[igc_record_count].zulu_time =p.zulu_time;
			igc_pos[igc_record_count].rpm =p.rpm;
			igc_record_count++;
		}
	}
}

void igc_restart() {
    igc_reset_log();
	HRESULT hr = SimConnect_Text(hSimConnect, 
								SIMCONNECT_TEXT_TYPE_PRINT_GREEN, 
								6.0, 
								EVENT_MENU_TEXT, //sizeof("TESTING"), "TESTING"); 
								sizeof(lang_reset), 
								lang_reset);
}

// save flt load time string YYYY-MM-DD_HHMM
void igc_get_flight_load_time() {
	__time64_t ltime;
	struct tm today;
    _time64(&ltime);
    _localtime64_s( &today, &ltime );
	wcsftime(flt_load_time, MAXBUF, L"%Y-%m-%d_%H%M", &today );
}

void igc_write_file(wchar_t *reason) {
	FILE *f;
	char buf[MAXBUF];
	char s[MAXBUF]; // buffer to how igc records before writing to file
	wchar_t fn[MAXBUF];
	//wchar_t wflt_pathname[MAXBUF]; // unicode flt_pathname
	wchar_t wflight_filename[MAXBUF]; // unicode flight_filename
	char s2[MAXBUF]; // another general text buffer
	wchar_t ws[MAXBUF]; // general unicode buffer
	size_t wlen; // length of a wchar buffer
	errno_t err;
	ChksumData chk_data;
	char chksum[CHKSUM_CHARS+1] = "000000";

    // flag file as saved by user
    if (wcscmp(reason,L"")==0) {
        igc_saved = true;
    }

	if (false && debug) {
		printf("flt_pathname=%s\n", flt_pathname);
		printf("chksum_flt=%s\n\n", chksum_flt);
		printf("air_pathname=%s\n", air_pathname);
		printf("chksum_air=%s\n\n", chksum_air);
		printf("pln_pathname=%s (no checksum)\n\n", pln_pathname);
		printf("wx_pathname=%s\n", wx_pathname);
		printf("chksum_wx=%s\n\n", chksum_wx);
		printf("cmx_pathname=%s\n", cmx_pathname);
		printf("chksum_cmx=%s\n\n", chksum_cmx);
		printf("cfg_pathname=%s\n", cfg_pathname);
		printf("chksum_cfg=%s\n\n", chksum_cfg);
	}

	// path_to_name(flt_name, flt_pathname); done on FLT load event
	path_to_name(air_name, air_pathname);
	path_to_name(pln_name, pln_pathname);
	path_to_name(wx_name, wx_pathname);
	path_to_name(cmx_name, cmx_pathname);
	path_to_name(cfg_name, cfg_pathname);
	path_to_name(xml_name, xml_pathname);
    
	// flight_filename is the FLT filename without the .FLT
	// i.e. "c:\abc\def\my flight.FLT" -> "my flight"
	char *flight_fn1 = strrchr(flt_pathname, '\\');
	if (flight_fn1==NULL) flight_fn1 = flt_pathname;
	else flight_fn1++;
	char *dot = strrchr(flight_fn1,'.');
	char flight_filename[MAXBUF];
	if (dot!=NULL) {
		//strncpy(flight_fn2, flight_fn1, dot-flight_fn1+1);
		errno_t err = strncpy_s( flight_filename, _countof(flight_filename), flight_fn1, dot-flight_fn1);
	} else {
		errno_t err = strcpy_s( flight_filename, flight_fn1);
	}

	// create folder if needed and build log filename as fn
	if (wcslen(ini_log_directory)==0) {
		if (debug) printf("Using FLT directory to save IGC tracklog\n");
		wcscpy_s(fn, MAXBUF, flt_directory);
		wcscat_s(fn, MAXBUF, L"\\");
	} else {
		if (debug) printf("Using INI folder to save IGC tracklog\n");
		wcscpy_s(fn, MAXBUF, ini_log_directory);
	}
	// convert flight_filename to unicode
	mbstowcs_s(&wlen,wflight_filename,flight_filename,MAXBUF);

	wcscat_s(fn, MAXBUF, wflight_filename);
	if (CreateDirectory(fn,NULL)==0 && GetLastError()!=ERROR_ALREADY_EXISTS) {
		if (debug) wprintf(L"ERROR: couldn't create folder \"%s\"\n", fn);
		return;
	}
	if (debug) wprintf(L"Saving tracklog to folder \"%s\"\n", fn);
	wcscat_s(fn, MAXBUF, L"\\");
	// pick up aircraft id from ini file else use ATC_ID
	if (wcscmp(ini_aircraft_id, L"")==0) {
		// convert ATC_ID to unicode
		mbstowcs_s(&wlen,ws,ATC_ID,MAXBUF);
		wcscat_s(fn, MAXBUF, ws);
	} else {
		wcscat_s(fn ,MAXBUF, ini_aircraft_id);
	}
	wcscat_s(fn, L"_");
	wcscat_s(fn, wflight_filename);
	wcscat_s(fn, L"_");
    wcscat_s(fn, flt_load_time);
	if (wcslen(reason)>1) {
		wcscat_s(fn, MAXBUF, L"(");
		wcscat_s(fn, MAXBUF, reason);
		wcscat_s(fn, MAXBUF, L")");
	}
	wcscat_s(fn, MAXBUF, L".igc");
	//strcat_s(fn, "\"");

	// debug
	if (debug) wprintf(L"\nWriting IGC file: %s\n",fn);

	if( (err = _wfopen_s(&f, fn, L"w")) != 0 ) {
		char error_text[200];
		
		sprintf_s(error_text, 
				sizeof(error_text), 
				"ERROR igc_logger v%.2f could NOT save IGC tracklog file", 
				version);

		HRESULT hr = SimConnect_Text(hSimConnect, 
									SIMCONNECT_TEXT_TYPE_SCROLL_RED, 
									15.0, 
									EVENT_MENU_TEXT,
									sizeof(error_text), 
									error_text);
		return;
	} else {
		chksum_reset(&chk_data);
		// ok we've opened the log file - lets write all the data to it
		sprintf_s(s,MAXBUF,         "AXXX sim_logger v%.2f\n", version); // manufacturer
		chksum_string(&chk_data, s); fprintf(f, s);

		sprintf_s(s,MAXBUF,		   "HFDTE%02.2d%02.2d%02.2d\n", startup_data.zulu_day,     // date
														startup_data.zulu_month,
														startup_data.zulu_year % 1000);
		chksum_string(&chk_data, s); fprintf(f, s);

		sprintf_s(s,MAXBUF,         "HFFXA035\n");                        // gps accuracy
		chksum_string(&chk_data, s); fprintf(f, s);

		strcpy_s(s, MAXBUF, "HFPLTPILOTINCHARGE: ");
		if (wcscmp(ini_pilot_name, L"")==0) {
			strcat_s(s,MAXBUF, "pilot ");
			strcat_s(s,MAXBUF, ATC_ID);
		} else {
			clean_string(s2, ini_pilot_name);
			strcat_s(s,MAXBUF, s2);
		}
		strcat_s(s,MAXBUF, "\n");
		chksum_string(&chk_data, s); fprintf(f, s);

		sprintf_s(s,MAXBUF,         "HFCM2CREW2: not recorded\n");
		chksum_string(&chk_data, s); fprintf(f, s);

		sprintf_s(s,MAXBUF,         "HFGTYGLIDERTYPE:%s\n", TITLE);
		chksum_string(&chk_data, s); fprintf(f, s);

		strcpy_s(s, MAXBUF, "HFGIDGLIDERID:");
		if (wcscmp(ini_aircraft_id, L"")==0) {
			strcat_s(s,MAXBUF, ATC_ID);
		} else {
			clean_string(s2, ini_aircraft_id);
			strcat_s(s,MAXBUF, s2);
		}
		strcat_s(s,MAXBUF, "\n");
		chksum_string(&chk_data, s); fprintf(f, s);

		sprintf_s(s,MAXBUF,         "HFDTM100GPSDATUM: WGS-1984\n");
		chksum_string(&chk_data, s); fprintf(f, s);

		sprintf_s(s,MAXBUF,         "HFRFWFIRMWAREVERSION: %.2f\n", version);
		chksum_string(&chk_data, s); fprintf(f, s);

		sprintf_s(s,MAXBUF,         "HFRHWHARDWAREVERSION: 2009\n");
		chksum_string(&chk_data, s); fprintf(f, s);

		sprintf_s(s,MAXBUF,         "HFFTYFRTYPE: sim_logger by Ian Forster-Lewis\n");
		chksum_string(&chk_data, s); fprintf(f, s);

		sprintf_s(s,MAXBUF,         "HFGPSGPS:Microsoft Flight Simulator\n");
		chksum_string(&chk_data, s); fprintf(f, s);

		sprintf_s(s,MAXBUF,         "HFPRSPRESSALTSENSOR: Microsoft Flight Simulator\n");
		chksum_string(&chk_data, s); fprintf(f, s);

		strcpy_s(s, MAXBUF, "HFCIDCOMPETITIONID:");
		if (wcscmp(ini_aircraft_id, L"")==0) {
			strcat_s(s,MAXBUF, ATC_ID);
		} else {
			clean_string(s2, ini_aircraft_id);
			strcat_s(s,MAXBUF, s2);
		}
		strcat_s(s,MAXBUF, "\n");
		chksum_string(&chk_data, s); fprintf(f, s);

		sprintf_s(s,MAXBUF,         "HFCCLCOMPETITIONCLASS: %s\n", ATC_TYPE);
		chksum_string(&chk_data, s); fprintf(f, s);

							// extension record to say info at end of 'B' recs
							// FXA = fix accuracy
							// SIU = satellites in use
							// ENL = engine noise level 000-999
		sprintf_s(s,MAXBUF,         "I023638FXA3941ENL\n"); 
		chksum_string(&chk_data, s); fprintf(f, s);

		// Task (C) records
		if (c_wp_count>1) {
			chksum_string(&chk_data, c[0]); fprintf(f, c[0]);
			chksum_string(&chk_data, c[1]); fprintf(f, c[1]);
			for (int i=0; i<c_wp_count; i++) {
				chksum_string(&chk_data, c[i+2]); fprintf(f, c[i+2]);
			}
			chksum_string(&chk_data, c_landing); fprintf(f, c_landing);
		}

		// FSX Comment (L) records
        clean_string(s2, flt_load_time);
        sprintf_s(s,MAXBUF,		   "L FSX user PC time            %s\n", s2);
		chksum_string(&chk_data, s); fprintf(f, s);

        sprintf_s(s,MAXBUF,		   "L FSX FLT checksum            %s (%s)\n", chksum_flt, flt_name);
		chksum_string(&chk_data, s); fprintf(f, s);

		//sprintf_s(s,MAXBUF,		   "L FSX PLN filename %s\n", pln_pathname);
		//chksum_string(&chk_data, s); fprintf(f, s);
		//sprintf_s(s,MAXBUF,		   "L FSX PLN checksum %s\n", chksum_pln);
		//chksum_string(&chk_data, s); fprintf(f, s);

		//sprintf_s(s,MAXBUF,		   "L FSX WX filename %s\n", wx_pathname);
		//chksum_string(&chk_data, s); fprintf(f, s);
		sprintf_s(s,MAXBUF,		   "L FSX WX checksum             %s (%s)\n", chksum_wx, wx_name);
		chksum_string(&chk_data, s); fprintf(f, s);

		//sprintf_s(s,MAXBUF,		   "L FSX CMX filename %s\n", cmx_pathname);
		//chksum_string(&chk_data, s); fprintf(f, s);
		sprintf_s(s,MAXBUF,		   "L FSX CMX checksum            %s (%s)\n", chksum_cmx, cmx_name);
		chksum_string(&chk_data, s); fprintf(f, s);

		sprintf_s(s,MAXBUF,		   "L FSX mission checksum        %s (%s)\n", chksum_xml, xml_name);
		chksum_string(&chk_data, s); fprintf(f, s);

		sprintf_s(s,MAXBUF,		   "L FSX aircraft.cfg checksum   %s (%s)\n", chksum_cfg, cfg_name);
		chksum_string(&chk_data, s); fprintf(f, s);

		//sprintf_s(s,MAXBUF,		   "L FSX AIR filename %s\n", air_pathname);
		//chksum_string(&chk_data, s); fprintf(f, s);
		sprintf_s(s,MAXBUF,		   "L FSX AIR checksum            %s (%s)\n", chksum_air, air_name);
		chksum_string(&chk_data, s); fprintf(f, s);

		// write CumulusX status locked/unlocked
		if (cx_code==0)
			sprintf_s(s,MAXBUF,		   "L FSX CumulusX status:        UNLOCKED\n");
		else
			sprintf_s(s,MAXBUF,		   "L FSX CumulusX status:        LOCKED OK\n");
		chksum_string(&chk_data, s); fprintf(f, s);

		// write wx status (unlocked if user has entered weather menu
		// after a WX file load
		if (wx_code==0)
            sprintf_s(s,MAXBUF,		   "L FSX WX status:              UNLOCKED\n");
		else
            sprintf_s(s,MAXBUF,		   "L FSX WX status:              LOCKED OK\n");
		chksum_string(&chk_data, s); fprintf(f, s);

		// ThermalDescriptions.xml entry
		if (fsx_thermals_enabled)
			sprintf_s(s,MAXBUF,		   "L FSX ThermalDescriptions.xml STILL BEING USED\n");
		else
			sprintf_s(s,MAXBUF,		   "L FSX ThermalDescriptions.xml REMOVED OK\n");
		chksum_string(&chk_data, s); fprintf(f, s);

		// now calculate a value for the GENERAL CHECKSUM
		chksum_chksum(chksum_all);
		sprintf_s(s,MAXBUF,		   "L FSX GENERAL CHECKSUM            %s  <---- CHECK THIS FIRST\n", chksum_all);
		chksum_string(&chk_data, s); fprintf(f, s);

		// now do the 'B' location records
		for (INT32 i=0; i<igc_record_count; i++) {
			int hours = igc_pos[i].zulu_time / 3600;
			int minutes = (igc_pos[i].zulu_time - hours * 3600 ) / 60;
			int secs = igc_pos[i].zulu_time % 60;
			char NS = (igc_pos[i].latitude>0.0) ? 'N' : 'S';
			char EW = (igc_pos[i].longitude>0.0) ? 'E' : 'W';
			double abs_latitude = fabs(igc_pos[i].latitude);
			double abs_longitude = fabs(igc_pos[i].longitude);
			int lat_DD = int(abs_latitude);
			int lat_MM = int( (abs_latitude - float(lat_DD)) * 60.0);
			int lat_mmm = int( (abs_latitude - float(lat_DD) - (float(lat_MM) / 60.0)) * 60000.0);
			int long_DDD = int(abs_longitude);
			int long_MM = int((abs_longitude - float(long_DDD)) * 60.0);
			int long_mmm = int((abs_longitude - float(long_DDD) - (float(long_MM) / 60.0)) * 60000.0);
			int altitude = int(igc_pos[i].altitude);
            int FXA = 27;
            int ENL = (int(igc_pos[i].rpm)>9990)? 999 : int(igc_pos[i].rpm) / 10;

//			sprintf_s(s,MAXBUF,     "B %02.2d %02.2d %02.2d %02.2d %02.2d %03.3d %c %03.3d %02.2d %03.3d %c A %05.5d %05.5d 000\n",
			sprintf_s(s,MAXBUF,     "B%02.2d%02.2d%02.2d%02.2d%02.2d%03.3d%c%03.3d%02.2d%03.3d%cA%05.5d%05.5d%03.3d%03.3d\n",
				    hours, minutes, secs,
					lat_DD, lat_MM, lat_mmm, NS,
					long_DDD, long_MM, long_mmm, EW,
					altitude, altitude, FXA, ENL);
			chksum_string(&chk_data, s); fprintf(f, s);
		}
		chksum_to_string(chksum, chk_data);
		fprintf(f,         "G%s\n",chksum);

		fclose(f);

		char file_write_text[250];
		
		clean_string(buf, fn);
		sprintf_s(file_write_text, 
				sizeof(file_write_text), 
				"igc_logger v%.2f wrote %s", 
				version, 
				buf);

		HRESULT hr = SimConnect_Text(hSimConnect, 
									SIMCONNECT_TEXT_TYPE_PRINT_GREEN, 
									6.0, 
									EVENT_MENU_TEXT, //sizeof("TESTING"), "TESTING"); 
									sizeof(file_write_text), 
									file_write_text);
	}
}

// write out the IGC file if it hasn't already been saved by the user
void flush_igc(wchar_t *reason) {
    if (!igc_saved && igc_record_count>IGC_MIN_RECORDS) {
				igc_write_file(reason);
	}
}

// this routine used to trigger an igc_file_write, but not used now
void igc_ground_check(INT32 on_ground, INT32 zulu_time) {
	// test for start of flight
	if (igc_record_count<2) {
		// if at start of flight then set up initial 'on ground' status
		igc_prev_on_ground = on_ground;
	} else
	// test for takeoff
	if (igc_prev_on_ground != 0 && on_ground == 0) {
		igc_prev_on_ground = 0; // remember current state is NOT on ground
		igc_takeoff_time = zulu_time; // record current time
		if (debug) printf("\nTakeoff detected\n"); 
	} else 
	// test for landing		
	if (igc_prev_on_ground == 0 && on_ground != 0 && // just landed
	       (zulu_time - igc_takeoff_time)>IGC_MIN_FLIGHT_SECS_TO_LANDING) { 
			   // AND was airborn long enough
  		if (debug) printf("\nLanding detected\n"); 
		igc_prev_on_ground = 1;
	} else {
		igc_prev_on_ground = on_ground;
	}
}

//*******************************************************************************
//*******************************************************************************
//*******************************************************************************
//**********                AI REPLAY code                ***********************
//*******************************************************************************
//*******************************************************************************

//*******************************************************************************
// AI DATA
const int MAX_AI = 200;

//const int AI_RETRY_TIME = 3; // after this delay from start, will check AI is loaded
// bool ai_retried = false; // flag to say re-create of AI has been tried...
const int MAX_AI_RETRIES = 2; // number of times to retry ai on failure
int ai_created_or_failed = 0;
int ai_retry_count = 0;
bool ai_failed = false; // flag set to true when any object create fail detected
int ai_count = 0; // count of IGC files loaded

struct ReplayPoint {
    double latitude;  // (PLANE LATITUDE, degrees) north positive
    double longitude; // (PLANE LONGITUDE, degrees) east positive
    double altitude;  // (PLANE ALTITUDE, Meters) 
	double pitch;     // (PLANE PITCH DEGREES, radians)
	double bank;      // (PLANE BANK DEGREES, radians)
	double heading;   // (PLANE HEADING DEGREES TRUE, radians)
	INT32 zulu_time;   // (ZULU TIME, seconds) seconds since midnight UTC
	double speed;     // forward speed, meters per second
};

// block of data locating ai object
struct AIStruct {
    double latitude; // deg N +ve
    double longitude; // deg E +ve
	double altitude; // m
    double pitch; // PLANE PITCH DEGREES, Radians
	double bank; // PLANE BANK DEGREES, Radians
	double heading; // PLANE HEADING DEGREES TRUE, Radians
	double altitude_agl;
	INT32  sim_on_ground;
};

// block of data to move an ai object
struct AIMoveStruct {
    double latitude; // deg N +ve
    double longitude; // deg E +ve
	double altitude; // m
    double pitch; // PLANE PITCH DEGREES, Radians
	double bank; // PLANE BANK DEGREES, Radians
	double heading; // PLANE HEADING DEGREES TRUE, Radians
};

// struct to hold the atc_id to be set on AI aircraft
struct AiSetDataStruct {
	char atc_id[32];
};

// here's the structure that holds the replay records for all loaded flights
ReplayPoint replay[MAX_AI][IGC_MAX_RECORDS];

struct AIInfo {
    int logpoint_count; // count of logpoints in this tracklog
	int next_logpoint; // cursor
    SIMCONNECT_OBJECT_ID id;
    bool created; // set to true when FSX says this object created OK
    bool removed; // set to true when zulu_time goes beyond last trackpoint
    bool default_tried; // set to true when a create with default a/c has been tried
	char title[MAXBUF];
	char atc_id[MAXBUF];
	INT32 gear_up_disable_timeout; // zulu time after which we can raise the gear
	bool gear_up; // gear up status
	//debug
	double alt_offset; // if we detect SIM ON GROUND we can calibrate IGC alt data
};

AIInfo ai_info[MAX_AI];

char *ai_model="DG808S"; // sim_logger SimProbe or DG808S ...

// flag to suppress PROBE ID exceptions (missing probe errors) while probes are re-created

bool suppress_object_id_exceptions = false;

double zulu_clock = 0.0; // this is sim_logger's version of FSX 'ZULU TIME'
double zulu_offset = 0.0; // zulu_time is system_time+zulu_offset

//debug timer var to reduce update rate
double last_update = 0; // timestamp of last update
DWORD  slew_rate = 0; // debug slew rate for calibration
double test_alt_offset = 0; // adjustment (m) to altitude on file load for testing
double test_lat_offset = 0; // adjustment (deg) to latitude on file load for testing
double test_lon_offset = 0; // adjustment (deg) to longitude on file load for testing
int test_time_offset = 0; // adjustment (s) to time on file load for testing

// END OF AI DATA
//*******************************************************************************

const double M_PI = 4.0*atan(1.0); // pi
const double EARTH_RAD = 6366710.0; // earth's radius in meters

//**********************************************************************************
// now we have a number of functions to do lat/long calculations to work
// out the lat/long needed for each probe.

// convert radians at the center of the earth to meters on the surface
inline double rad2m(double rad)
{
    return EARTH_RAD * rad;
}

// convert metres on earths surface to radians subtended at the centre
inline double m2rad(double distance)
{
    return distance / EARTH_RAD;
}

// convert degrees to radians
inline double deg2rad(double deg)
{
    return deg * (M_PI / 180.0);
}

// convert radians to degrees
inline double rad2deg(double rad)
{
    return rad * (180.0 / M_PI);
}

// convert meters to feet (for initposition)
inline double m2ft(double m) {
    return m * 3.2808399;
}

// bearing (radians) from point 1 to point 2
inline double bearing(double lat1d, double lon1d, double lat2d, double lon2d) {
	//double y = -sin(lon1-lon2) * cos(lat2);
	//double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2)* cos(lon1-lon2);
	//double bearing_radians = atan2(y, x);
	//return fmod(rad2deg(bearing_radians)+360,360);
	double lat1 = deg2rad(lat1d);
	double lon1 = deg2rad(lon1d);
	double lat2 = deg2rad(lat2d);
	double lon2 = deg2rad(lon2d);
	return fmod(atan2(sin(lon2-lon1)*cos(lat2),
                      cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1))+2*M_PI,
                        2*M_PI);
}

// distance (m) on earth's surface from point 1 to point 2
double distance(double lat1, double lon1, double lat2, double lon2) {
    double lat1r = deg2rad(lat1);
    double lon1r = deg2rad(lon1);
    double lat2r = deg2rad(lat2);
    double lon2r = deg2rad(lon2);
    return acos(sin(lat1r) * sin(lat2r)+cos(lat1r) * cos(lat2r) * cos(lon2r-lon1r)) * EARTH_RAD;
}

// gives average bearing of middle point between three points
// which gives a 'target' heading for the ai object to be at when it arrives at middle point
double target_heading(double lat1, double lon1, double lat2, double lon2, double lat3, double lon3) {
    double avg_lat1 = (lat1 + lat2) / 2;
    double avg_lon1 = (lon1 + lon2) / 2;
    double avg_lat2 = (lat2 + lat3) / 2;
    double avg_lon2 = (lon2 + lon3) / 2;
        
    return bearing(avg_lat1, avg_lon1, avg_lat2, avg_lon2);
}

// +ve/-ve difference between two headings
double heading_delta(double desired, double current) {
    double angle;
    angle = fmod(desired - current + 2 * M_PI, 2 * M_PI);
    return (angle > M_PI) ? angle - 2 * M_PI : angle;
}

//*********************************************************************************************
// interp code - inject more ReplayPoints where IGC file timestep > 5  seconds
//*********************************************************************************************

// distance_and_bearing(...) returns a new lat/long a distance and bearing from lat1,lon1.
// lat, longs in degrees, rbearng in radians, distance in meters
ReplayPoint distance_and_bearing(ReplayPoint p, double distance, double rbearing) {
	double rlat1, rlong1, rdistance, rlat2, rlong2;
	ReplayPoint r;
	rlat1 = deg2rad(p.latitude);
	rlong1 = deg2rad(p.longitude);
	rdistance = m2rad(distance);
	rlat2 = asin(sin(rlat1)*cos(rdistance)+cos(rlat1)*sin(rdistance)*cos(rbearing));
	if (cos(rlat2)==0) {
        rlong2 = rlong1;      // endpoint a pole
	}
	else {
		rlong2 = fmod((rlong1+asin(sin(rbearing)*sin(rdistance)/cos(rlat2))+M_PI),(2*M_PI))-M_PI;
	}
	r.latitude = rad2deg(rlat2);
	r.longitude = rad2deg(rlong2);
	return r;
}

// interp returns a RelayPoint step_time after point p1
// with time/lat/lon/alt only
ReplayPoint interp(ReplayPoint p0, ReplayPoint p1, ReplayPoint p2, ReplayPoint p3, INT32 step_time) {
    
    double correction_coefficient = 0.17;
	INT32 time_delta = p2.zulu_time - p1.zulu_time;

    //bearings in radians
    double bearing0 = bearing(p0.latitude, p0.longitude, p1.latitude, p1.longitude);
    double bearing1 = bearing(p1.latitude, p1.longitude, p2.latitude, p2.longitude);
    double bearing2 = bearing(p2.latitude, p2.longitude, p3.latitude, p3.longitude);
    
    double bearing_delta1 = heading_delta(bearing0, bearing1);
    double bearing_delta2 = heading_delta(bearing1, bearing2);
    
    double total_turn1 = bearing_delta1 + bearing_delta2;
    
    double heading_correction1 = correction_coefficient * total_turn1;
    
    double distance1 = distance(p1.latitude, p1.longitude, p2.latitude, p2.longitude);
    
    double speed1 = distance1 / time_delta;
    
    double new_heading1 = bearing1 + heading_correction1;
    double new_heading_delta1 = heading_delta(bearing1, new_heading1);
    double speed_correction1 = (1 + (1 - 2 / M_PI) * abs(new_heading_delta1));
    double distance_to_interp1 = speed1 * step_time * speed_correction1;
    
    ReplayPoint r = distance_and_bearing(p1, distance_to_interp1, new_heading1);
	r.zulu_time = p1.zulu_time + step_time;
	r.altitude = p1.altitude + ((double)step_time/(double)time_delta)*(p2.altitude-p1.altitude);
	return r;
}
//*********************************************************************************************
// slew calibration functions

DWORD slew_rotation_to_rate(double rotation) { // rotation in radians / second
    // +ve rotate to port, port wing down, nose down
    // rotation rad/s = rate ^ 2 / 11240000

    return (rotation<0) ? (DWORD) -sqrt(-rotation * 11240000) : (DWORD) sqrt(rotation * 11240000);
}

DWORD slew_ahead_to_rate(double speed) { // speed in meters per second
    // +ve forwards
    // speed m/s = rate ^ 2 / 45678
    return (speed<0) ? (DWORD) -sqrt(-speed * 45678) : (DWORD) sqrt(speed * 45678);
}

DWORD slew_alt_to_rate(double sink) {
    // +ve downwards
    return (sink<0)? (DWORD) -sqrt( -sink * 3084000 ) : (DWORD) sqrt( sink * 3084000 );
}

//*********************************************************************************************
// which heading should object be at to approach on correct target heading
double desired_heading(double bearing_to_wp, double target_heading) {
    double heading;
    double coefficient;
    
    coefficient = 0.5;
    
    heading = bearing_to_wp - coefficient * heading_delta(target_heading, bearing_to_wp);
    heading = heading + 2 * M_PI;
    
    return fmod(heading, 2 * M_PI);
}

//*********************************************************************************************
// calculate appropriate pitch based on speed and predict point
// +ve pitch is nose DOWN
double desired_pitch(double alt_delta, double dist, double time) {
    double zdist; // 3d diagonal distance for proper speed
    double speed;
    double slope_pitch;
    double speed_pitch;
    double pitch;

    //double coefficient;
    // check for safe values - worst case we can always give a pitch of zero
    if (time<0.1) return 0;
    if (dist<0.1) return 0;
    //coefficient = 0.35;
    
    zdist = sqrt(pow(alt_delta,2) + pow(dist,2));
    speed = zdist / time;

    // get pitch values due to (i) slope and (ii) speed and combine them
    slope_pitch = -atan(alt_delta / dist);

	//const double PITCH_MIN = -0.3; // pitch at min speed i.e. max pitch nose UP
	//const double PITCH_MAX = 0.18; // pitch at max speed

	double C = -2 * ini_pitch_min / M_PI;
	double X = tan(ini_pitch_max / C);

	speed_pitch = C * atan(X*(1-ini_pitch_v_zero/speed)) + ini_pitch_offset;
    //speed_pitch = coefficient * atan ( 900 / pow(speed,2) - 1 );

    pitch = slope_pitch + speed_pitch;
    pitch = min(pitch, 1.5);
    pitch = max(pitch, -1.5);
    return pitch;
}

// what rate should object change heading at
DWORD slew_turn_rate(double bearing_to_wp, double current_heading, double target_heading) {
    double desired;
    double coefficient;
    
    coefficient = 0.65;

    desired = desired_heading(bearing_to_wp, target_heading);
    
    // note minus in front of coefficient - +ve turn reduces heading!
    return slew_rotation_to_rate(-coefficient * heading_delta(desired, current_heading));
}

// what rate to set ahead slew should object move to arrive at correct time
DWORD slew_ahead_rate(double lat1, double lon1, double lat2, double lon2, double time_to_go) {
    double speed = distance(lat1, lon1, lat2, lon2) / time_to_go;
    return slew_ahead_to_rate(speed);
}

//*********************************************************************************************

// synchronise the sim_logger clock to FSX 'ZULU TIME'
void zulu_clock_sync(INT32 zulu_time) {
	// zulu_time is definitive time from FSX
	// debug
	__timeb64 time_buffer;
	_ftime64_s(&time_buffer);

	double system_time = (double)time_buffer.time + (double)time_buffer.millitm/1000;
	// if internal clock has drifted by 4 seconds then adjust
	if (abs((double)zulu_time - (system_time + zulu_offset))>4) {
		if (debug) printf("\nAdjusting clock to fsx=%d, was=%.2f\n",zulu_time, system_time+zulu_offset);
		zulu_offset = zulu_time - system_time;
		//if (debug) printf("New offset %.2f\n",zulu_offset);
	}
	zulu_clock = system_time + zulu_offset;
}

void remove_ai(int ai_index)
{
    HRESULT hr;
    if (ai_info[ai_index].created) {
	    if (debug) printf("remove_ai(%d)..", ai_index);
		ai_info[ai_index].created = false;
        ai_info[ai_index].removed = true;
		hr = SimConnect_AIRemoveObject(hSimConnect, ai_info[ai_index].id, (UINT)REQUEST_AI_REMOVE+ai_index);
	}
}

// reset the loaded AI igc files
void reset_ai() {
	for (int i=0; i<ai_count; i++) {
		remove_ai(i);
		ai_info[i].created = false;
        ai_info[i].removed = false;
        ai_info[i].default_tried = false;
		ai_info[i].logpoint_count = 0;
		ai_info[i].alt_offset = 0;
		ai_info[i].gear_up_disable_timeout = 0;
		ai_info[i].gear_up = false;
	}
	ai_count = 0;
    ai_created_or_failed = 0;
    ai_failed = false;
    ai_retry_count = 0;
}

void move_ai(int ai_index, ReplayPoint r) {
	HRESULT hr;
	AIMoveStruct ai_move_data;
	ai_move_data.latitude = r.latitude;
	ai_move_data.longitude = r.longitude;
	ai_move_data.altitude = r.altitude;
	ai_move_data.pitch = r.pitch;
	ai_move_data.bank = r.bank;
	ai_move_data.heading = r.heading;

	if (debug) printf("Moving ai(%d) to %2.5f,%3.5f\n", ai_index, r.latitude, r.longitude);
	// set LLAPBH
	hr = SimConnect_SetDataOnSimObject(hSimConnect,
										DEFINITION_AI_MOVE,
										ai_info[ai_index].id,
										0, 0, sizeof(ai_move_data), &ai_move_data);
	// send slew command to stop
	hr = SimConnect_TransmitClientEvent(hSimConnect,
						ai_info[ai_index].id,
						EVENT_AXIS_SLEW_AHEAD_SET,
						0, // zero ahead rate => stop
						SIMCONNECT_GROUP_PRIORITY_HIGHEST,
						SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY);
}

void create_ai(int ai_index)
{
    if (debug) printf("Creating AI(%d) %s\n", ai_index, ai_info[ai_index].title);
    HRESULT hr;

	SIMCONNECT_DATA_INITPOSITION ai_init;
    
    //ai_init.Altitude   = replay[ai_index][0].altitude;  // Altitude of Sea-tac is 433 feet
    //debug - added 40 feet for seatac test
    ai_init.Altitude   = m2ft(replay[ai_index][0].altitude)+10; // feet Altitude of Sea-tac is 433 feet
    ai_init.Latitude   = replay[ai_index][0].latitude;    // Degrees Convert from 47 25.90 N
    ai_init.Longitude  = replay[ai_index][0].longitude;   // Degrees Convert from 122 18.48 W
    ai_init.Pitch      = rad2deg(replay[ai_index][0].pitch);       // Degrees
    ai_init.Bank       = rad2deg(replay[ai_index][0].bank);        // Degrees
    ai_init.Heading    = rad2deg(replay[ai_index][0].heading);     // Degrees
    ai_init.OnGround   = 0;                               // 1=OnGround, 0 = airborne
    ai_init.Airspeed   = 0;                               // Knots
    
	// now create ai object
    if (!ai_info[ai_index].created) hr = SimConnect_AICreateSimulatedObject(hSimConnect, 
                                            ai_info[ai_index].title, 
                                            ai_init, 
                                            (UINT)REQUEST_AI_CREATE+ai_index);
    //if (debug) printf("create_ai %s\n", (hr==S_OK) ? "OK" : "FAIL");
}

// retry_ai() gets called if there is an CREATE_OBJECT_FAILED exception
void retry_ai() {
    char buf[MAXBUF]; // general buffer
    // check retry count and quit if at max
    if (ai_retry_count++==MAX_AI_RETRIES) return;
    if (debug) printf("Retrying AI creates, ai_count=%d\n", ai_count);
    // check each ai object to see if it is not created and not removed
    for (int ai_index=0; ai_index<ai_count; ai_index++) {
        if (!ai_info[ai_index].created &&
            !ai_info[ai_index].removed &&
            !ai_info[ai_index].default_tried) {
            ai_created_or_failed--;
            // have another try, this time with the default aircraft
            clean_string(buf, ini_default_aircraft);
            //if (debug) printf("Retrying AI(%d:created=%d,removed=%d,tried=%d:%s) with %s\n",
            //                    ai_index,
            //                    (ai_info[ai_index].created) ? 1 : 0,
            //                    (ai_info[ai_index].removed) ? 1 : 0,
            //                    (ai_info[ai_index].default_tried) ? 1 : 0,
            //                    ai_info[ai_index].title,
            //                    buf);
            strcpy_s(ai_info[ai_index].title, MAXBUF, buf);
            ai_info[ai_index].default_tried = true;
            create_ai(ai_index);
        }
    }
}

// increment the created_or_failed count, and trigger a retry if necessary
void incr_ai_created_or_failed() {
    ai_created_or_failed++;
    //if (debug) printf("incr_ai_created_or_failed: %d\n",ai_created_or_failed);
    if (ai_created_or_failed==ai_count && ai_failed) retry_ai();
}


//*****************************************************************************************
// init_ai(i) transmits the event to 'freeze' the altitude & attitude of ai[i]
void init_ai(int ai_index) {
	HRESULT hr;
	//hr = SimConnect_AIReleaseControl(hSimConnect, probe_id[i], request_probe_release[i]);
	//hr = SimConnect_TransmitClientEvent(hSimConnect,
	//									ai_id[i],
	//									EVENT_FREEZE_ALTITUDE,
	//									1, // set freeze value to 1
	//									SIMCONNECT_GROUP_PRIORITY_HIGHEST,
	//									SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY);
	//hr = SimConnect_TransmitClientEvent(hSimConnect,
	//									ai_id[i],
	//									EVENT_FREEZE_ATTITUDE,
	//									1, // set freeze value to 1
	//									SIMCONNECT_GROUP_PRIORITY_HIGHEST,
	//									SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY);
	//hr = SimConnect_TransmitClientEvent(hSimConnect,
	//									ai_id[i],
	//									EVENT_FREEZE_LATLONG,
	//									1, // set freeze value to 1
	//									SIMCONNECT_GROUP_PRIORITY_HIGHEST,
	//									SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY);
    // now try slew mode
    // debug
    if (debug) printf("Slew On(%d) ", ai_index);
	hr = SimConnect_TransmitClientEvent(hSimConnect,
										ai_info[ai_index].id,
										EVENT_SLEW_ON,
										1, // set slew value to 1
										SIMCONNECT_GROUP_PRIORITY_HIGHEST,
										SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY);
}

//*****************************************************************************************
// transmit GEAR_UP and GEAR_DOWN events
void ai_gear(int ai_index, int current_target, AIStruct pos) {
	HRESULT hr;
	const double LANDING_SPEED = 10; // m/s
	const INT32 LANDING_LOOKAHEAD = 100; // (seconds) look ahead 2 minutes
	const double GEAR_UP_HEIGHT = 40; // meters
	ReplayPoint *r = replay[ai_index];

	INT32 current_time = r[current_target].zulu_time;
	if (debug) printf("ai_gear(%d) @ %d, gear=%s, speed=%.2f, agl=%.2f timeout=",
		ai_index,
		current_time, 
		(ai_info[ai_index].gear_up)?"UP":"DOWN",
		(float) r[current_target].speed,
		(float) pos.altitude_agl
		);
	if ( current_time < ai_info[ai_index].gear_up_disable_timeout) {
		if (debug) printf("%d\n", current_time);
		return;
	}
	if (debug) printf("\n");

	int i = current_target;
	while (ai_info[ai_index].gear_up && 
				r[i].zulu_time < current_time + LANDING_LOOKAHEAD) {
		if (debug) printf("%.0f,",r[i].speed);
		if (r[i].speed < LANDING_SPEED ) {
			ai_info[ai_index].gear_up = false;
			ai_info[ai_index].gear_up_disable_timeout = current_time + LANDING_LOOKAHEAD;
			if (debug) printf(" sending GEAR_DOWN to ai(%d) until %d\n", ai_index, ai_info[ai_index].gear_up_disable_timeout);
			hr = SimConnect_TransmitClientEvent(hSimConnect,
												ai_info[ai_index].id,
												EVENT_GEAR_DOWN,
												0,
												SIMCONNECT_GROUP_PRIORITY_HIGHEST,
												SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY);
			return;
		}
		if (++i == ai_info[ai_index].logpoint_count) return;
	}

	if (!ai_info[ai_index].gear_up && pos.altitude_agl > GEAR_UP_HEIGHT) {
		if (debug) printf("\nsending GEAR_UP to ai(%d)\n", ai_index);
		ai_info[ai_index].gear_up = true;
		hr = SimConnect_TransmitClientEvent(hSimConnect,
										ai_info[ai_index].id,
										EVENT_GEAR_UP,
										0,
										SIMCONNECT_GROUP_PRIORITY_HIGHEST,
										SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY);
	}
}

// request pos updates for ai object
void get_ai_pos_updates(int ai_index) {
    HRESULT hr;
    //if (debug) printf(" requesting pos updates for ai %d\n",ai_index);
	// set data request
	hr = SimConnect_RequestDataOnSimObject(hSimConnect,
                                            //debug must make this request enum id unique
											(UINT)REQUEST_AI_POS+ai_index, 
											DEFINITION_AI_POS, 
											ai_info[ai_index].id,
											SIMCONNECT_PERIOD_SECOND); 
}

//*****************************************************************************************
//***********************        update_ai()   ********************************************
//*****************************************************************************************
//********************** This is where we do the predict-point following stuff ************

// update the positions of the ai object i
// called each time the actual ai position is returned from FSX
void update_ai(int ai_index, AIStruct pos) {
    const double PREDICT_PERIOD = 4; // predict replay position 4 seconds ahead
	const double AI_WARP_TIME = 30; // if current AI point is 30 seconds old, then MOVE not SLEW
    HRESULT hr;
    int i = 1;
    ReplayPoint *r = replay[ai_index]; // the array of ReplayPoints for current tracklog
    ReplayPoint predict_point; // a ReplayPoint for the predicted position

    bool found = false;
    // scan the loaded IGC file until you find current time position
    //debug - this could be more efficient if we assume monotonic time
    while (i<ai_info[ai_index].logpoint_count-2) {
	    if (zulu_clock>r[i].zulu_time) i++;
	    else {
		    found = true;
		    break;
	    }
    }
    // now r[i] is first ReplayPoint AFTER current sim zulu_clock
	if (!found) {
		remove_ai(ai_index);
		return;
	}

	// test to see if zulu_time of current AI position is so old we should MOVE not SLEW
	if (zulu_clock - r[ai_info[ai_index].next_logpoint].zulu_time > AI_WARP_TIME) {
		if (debug) printf("zulu_clock: %.1f, next point: %d(%d), current: %d(%d)\n",
			zulu_clock, i, r[i].zulu_time, ai_info[ai_index].next_logpoint, r[ai_info[ai_index].next_logpoint].zulu_time);
		move_ai(ai_index, r[i]);
		ai_info[ai_index].next_logpoint = i;
		return;
	}

	// OK, the next tracklog position is not too far away, so we'll aim for predict point
	ai_info[ai_index].next_logpoint = i;

	// now search forwards again for the NEXT point after the predict_point
    found = false;
    // PREDICT where the object would be in 4 seconds time
    double predict_time = zulu_clock + PREDICT_PERIOD;
    int j = i;
    while (j<ai_info[ai_index].logpoint_count-2) {
        if (predict_time>r[j].zulu_time) j++;
        else {
	        found = true;
	        break;
        }
    }
    if (found) { // i.e. we have also found the predict point
        // now r[j] is first ReplayPoint AFTER predict_time
	    // progress is fraction of forward progress beyond found replay point
	    double progress = (predict_time - r[j-1].zulu_time)/(r[j].zulu_time - r[j-1].zulu_time);
	    progress = max(progress,0); // don't extrapolate *before* r[j-1]
	    predict_point.latitude = r[j-1].latitude + progress * (r[j].latitude - r[j-1].latitude);
	    predict_point.longitude = r[j-1].longitude + progress * (r[j].longitude - r[j-1].longitude);
		// include alt_offset in alt calc
	    predict_point.altitude = r[j-1].altitude + progress * (r[j].altitude - r[j-1].altitude) + ai_info[ai_index].alt_offset;
	    predict_point.heading = bearing(r[j-1].latitude, r[j-1].longitude,r[j].latitude, r[j].longitude);
		if (pos.sim_on_ground) {
			// ON GROUND, so we can calibrate the IGC file alts with an offset
			// temporarily disabled while I think about the issues...
			//ai_info[ai_index].alt_offset = pos.altitude - r[j-1].altitude;
			//if (debug) printf("%s alt_offset %.1f\n",ai_info[ai_index].atc_id, ai_info[ai_index].alt_offset);
			predict_point.pitch = 0;
			predict_point.bank = 0;
		} else {
			predict_point.bank = r[j-1].bank + progress * (r[j].bank - r[j-1].bank);
			predict_point.pitch = r[j-1].pitch + progress * (r[j].pitch - r[j-1].pitch);
		}
        // now calculate steering deltas based on predict point
        double bearing_to_wp = bearing(pos.latitude, pos.longitude,
                                       predict_point.latitude, predict_point.longitude);

        double desired = desired_heading(bearing_to_wp, predict_point.heading);

        double delta = heading_delta(desired, pos.heading);

        DWORD heading_rate = slew_turn_rate(bearing_to_wp, pos.heading, predict_point.heading);

        DWORD ahead_rate = slew_ahead_rate(pos.latitude, pos.longitude,
                                       predict_point.latitude, predict_point.longitude,
                                       PREDICT_PERIOD);

        DWORD bank_rate = slew_rotation_to_rate((predict_point.bank - pos.bank) / PREDICT_PERIOD);

        DWORD pitch_rate = slew_rotation_to_rate((predict_point.pitch - pos.pitch) / PREDICT_PERIOD);

        DWORD alt_rate = slew_alt_to_rate((pos.altitude - predict_point.altitude) / PREDICT_PERIOD);

        //debug - print lat longs for excel analysis
        // time,lat,lon,alt,pitch,bank,heading,ahead rate, alt rate, pitch rate, bank rate, heading rate
        if (false && debug) printf("%5.2f,%2.5f,%3.5f,%5.0f,%+2.3f,%+2.3f,%+2.3f,%+5d,%+5d,%+5d,%+5d,%+5d",
                            zulu_clock,
                            pos.latitude,
                            pos.longitude,
                            pos.altitude,
                            pos.pitch,
                            pos.bank,
                            pos.heading,
                            ahead_rate,
							alt_rate,
							pitch_rate,
							bank_rate,
							heading_rate
                            );
        // target time,lat,lon,alt,pitch,bank,heading,||
        if (false && debug) printf(",target:,%d,%2.5f,%3.5f,%5.0f,%+2.5f,%+2.5f,%+2.5f\n",
                            r[i].zulu_time,
                            r[i].latitude,
                            r[i].longitude,
                            r[i].altitude,
                            r[i].pitch,
                            r[i].bank,
                            r[i].heading );
        
		// send the actual slew adjustments
        hr = SimConnect_TransmitClientEvent(hSimConnect,
						    ai_info[ai_index].id,
						    EVENT_AXIS_SLEW_AHEAD_SET,
						    ahead_rate,
						    SIMCONNECT_GROUP_PRIORITY_HIGHEST,
						    SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY);

        hr = SimConnect_TransmitClientEvent(hSimConnect,
						    ai_info[ai_index].id,
						    EVENT_AXIS_SLEW_HEADING_SET,
						    heading_rate,
						    SIMCONNECT_GROUP_PRIORITY_HIGHEST,
						    SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY);

        hr = SimConnect_TransmitClientEvent(hSimConnect,
						    ai_info[ai_index].id,
						    EVENT_AXIS_SLEW_ALT_SET,
						    alt_rate,
						    SIMCONNECT_GROUP_PRIORITY_HIGHEST,
						    SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY);

        hr = SimConnect_TransmitClientEvent(hSimConnect,
						    ai_info[ai_index].id,
						    EVENT_AXIS_SLEW_BANK_SET,
						    bank_rate,
						    SIMCONNECT_GROUP_PRIORITY_HIGHEST,
						    SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY);

        hr = SimConnect_TransmitClientEvent(hSimConnect,
						    ai_info[ai_index].id,
						    EVENT_AXIS_SLEW_PITCH_SET,
						    pitch_rate,
						    SIMCONNECT_GROUP_PRIORITY_HIGHEST,
						    SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY);
		// send gear up/down as necessary
		//ai_gear(ai_index, i, pos);
	}
}

bool text_char(char m) {
	return (m>='a' && m<='z') || (m>='A' && m<='Z') || (m>='0' && m<='9') || m==' ' || m=='_' || m=='-';
}


// pick out a record from the igc file

bool get_igc_record(char *out_s, char *rec, char *key) {
	int len = strlen(key);
	char s[MAXBUF];
	for (int m=0;m<len;m++) s[m] = rec[m];
	s[len] = '\0';
	if (_stricmp(s,key)==0) {
		int m = 0;
		// skip leading spaces in value
		while (rec[m+len]==' ') m++;
		// if record is blank just return rather than overwrite value
		if (rec[m+len]=='\0') return true;
		while (text_char(rec[m+len])) {
			out_s[m] = rec[m+len];
			m++;
		}
		out_s[m] = '\0';
		if (debug) printf("%s%s...", key, out_s);
		return true;
	}
	return false;
}

// calculate appropriate pitch/bank/heading/speed values for replaypoint[i]
void ai_update_pbhs(ReplayPoint p[], int i) {
	// pitch & speed
	if (i==0) { p[i].pitch = 0; p[i].speed = 0; }
	else {
		double dist = distance(p[i-1].latitude,
								      p[i-1].longitude,
								      p[i].latitude,
								      p[i].longitude);
		p[i].pitch = desired_pitch(p[i].altitude-p[i-1].altitude,
                                        dist,
                                        p[i].zulu_time-p[i-1].zulu_time);
		p[i].speed = dist / (p[i].zulu_time-p[i-1].zulu_time);
		if (debug) printf("%d %4.1f %3.2f\n", p[i].zulu_time, p[i].altitude, p[i].speed);
	}

    // heading
    if (i==1) p[0].heading = bearing(p[0].latitude, p[0].longitude,
                                     p[1].latitude, p[1].longitude);
	else if (i>1) {
		p[i-1].heading = target_heading(p[i-2].latitude, p[i-2].longitude,
                                     p[i-1].latitude, p[i-1].longitude,
                                     p[i].latitude, p[i].longitude);
		//if (debug) printf("updating p[%2d-%d] (%2.5f,%2.5f) p[%2d-%d].heading=%.3f\n",
		//	i, p[i].zulu_time,p[i].latitude, p[i].longitude,i-1,p[i-1].zulu_time,p[i-1].heading);
	}

	// bank
	if (i<2) p[i].bank = 0;
	else {
		double this_bearing = bearing(p[i-1].latitude,
								      p[i-1].longitude,
								      p[i].latitude,
								      p[i].longitude);
		double prev_bearing = bearing(p[i-2].latitude,
								      p[i-2].longitude,
								      p[i-1].latitude,
								      p[i-1].longitude);
		double bearing_delta = fmod(this_bearing + 2*M_PI - prev_bearing, 2*M_PI);
		if (bearing_delta>M_PI) bearing_delta = bearing_delta - 2*M_PI;
		double turn_radians_per_second = bearing_delta / (p[i].zulu_time - p[i-1].zulu_time);
		p[i].bank = -turn_radians_per_second * 4;
        p[i].bank = min(p[i].bank, 1.5);
        p[i].bank = max(p[i].bank, -1.5);
	}

}
// load an IGC file into the replay buffer

int load_igc_file(int ai_index, wchar_t path[MAXBUF]) {
	// see if the .IGC file actually exists
	if(_waccess_s(path, 0) != 0) {
		// file not found
		if (debug) wprintf(L"IGC file not found: %s\n", path);
		return -1;
	} else {
		// file exists
		FILE *f;
		errno_t err;
		char line_buf[MAXBUF];
		char s[MAXBUF];
		int i = 0; // record counter
		int j = 0; // general counter
		ReplayPoint *p = replay[ai_index];

		if( (err = _wfopen_s(&f, path, L"r")) != 0 ) {
			return -1;
		}

		// initialise the aircraft title to ini_default_aircraft
        // but this will get overwritten if there's a glider type in the IGC file
        clean_string(line_buf, ini_default_aircraft);
		strcpy_s(ai_info[ai_index].title, line_buf);
		// initialise ATC_ID
		strcpy_s(ai_info[ai_index].atc_id, MAXBUF, "XXXX");

		while (fgets(line_buf, MAXBUF, f)!=NULL) {
			if (get_igc_record(ai_info[ai_index].title,line_buf,"HFGTYGLIDERTYPE:"))
				continue;
			if (get_igc_record(ai_info[ai_index].atc_id,line_buf,"HFCIDCOMPETITIONID:"))
				continue;
			if (get_igc_record(ai_info[ai_index].atc_id,line_buf,"HFGIDGLIDERID:"))
				continue;
			if (get_igc_record(ai_info[ai_index].atc_id,line_buf,"LCU::HPGIDGLIDERID:"))
				continue;
			if (get_igc_record(ai_info[ai_index].atc_id,line_buf,"LCU::HPCIDCOMPETITIONID:"))
				continue;
			if (get_igc_record(ai_info[ai_index].atc_id,line_buf,"HFCID Competition ID    :"))
				continue;

			if (line_buf[0]!='B') continue;
			//if (debug) printf("input: %s",line_buf);
			j = 0;
			s[j++] = line_buf[1]; // HH
			s[j++] = line_buf[2];
			s[j++] = ' ';
			s[j++] = line_buf[3]; // MM
			s[j++] = line_buf[4];
			s[j++] = ' ';
			s[j++] = line_buf[5]; // SS
			s[j++] = line_buf[6];
			s[j++] = ' ';
			s[j++] = line_buf[7]; // DD latitude
			s[j++] = line_buf[8];
			s[j++] = ' ';
			s[j++] = line_buf[9]; // MM
			s[j++] = line_buf[10];
			s[j++] = '.';
			s[j++] = line_buf[11]; // mmm
			s[j++] = line_buf[12];
			s[j++] = line_buf[13];
			s[j++] = ' ';
			s[j++] = line_buf[14]; // N/S (N positive)
			//s[j++] = ' ';
			s[j++] = line_buf[15]; // DDD longitude
			s[j++] = line_buf[16];
			s[j++] = line_buf[17];
			s[j++] = ' ';
			s[j++] = line_buf[18]; // MM
			s[j++] = line_buf[19];
			s[j++] = '.';
			s[j++] = line_buf[20]; // mmm
			s[j++] = line_buf[21];
			s[j++] = line_buf[22];
			s[j++] = ' ';
			s[j++] = line_buf[23]; // E/W (E positive)
			//s[j++] = ' ';
			s[j++] = line_buf[25]; // alt
			s[j++] = line_buf[26];
			s[j++] = line_buf[27];
			s[j++] = line_buf[28]; 
			s[j++] = line_buf[29];
			s[j++] = '\0';
			// if (debug) printf("Parsed to: %s\n",s);
			// now parse s into floats using sscanf
			int hours,mins,secs, d_lat, d_long, alt;
			float m_lat,m_long, angle;
			char ns, ew; // north/south, east/west
			sscanf_s(s, "%d %d %d %d %f %c %d %f %c %d",
						&hours,&mins,&secs,&d_lat, &m_lat, &ns, 1, &d_long, &m_long, &ew, 1, &alt);

			//if (debug) {
			//	printf("result:    %d,%d,%d,%d,%f,'%c',%d,%f,'%c',%d\n\n",
			//			hours,mins,secs,d_lat, m_lat, ns, d_long, m_long, ew, alt);
			//}
			// latitude
			angle = (d_lat + m_lat/60);
			if (ns=='S')  angle = -angle;
			p[i].latitude = angle + test_lat_offset;

			// longitude
			angle = (d_long + m_long/60);
			if (ew=='W')  angle = -angle;
			p[i].longitude = angle + test_lon_offset;

			// altitude
			p[i].altitude = alt + test_alt_offset;

			// time
			p[i].zulu_time = 3600*hours+60*mins+secs+test_time_offset;

			// now we can insert some interpolated points if needed
			const int IGC_GAP_TIME = 12; // do NOT try and interpolate a gap >12 seconds
			const int IGC_DELTA_MAX = 5; // if gap >5 seconds then insert interpolated point
			const int IGC_INTERP_TIMESTEP = 3; // interpolate at 3 second intervals
			if (i>2 && (p[i-1].zulu_time-p[i-2].zulu_time)<= IGC_GAP_TIME) {
				while ((p[i-1].zulu_time-p[i-2].zulu_time)>IGC_DELTA_MAX) {
					// calc an interp point after p[i-2]
					ReplayPoint p_interp = interp(p[i-3],p[i-2],p[i-1],p[i],IGC_INTERP_TIMESTEP);
					// shuffle up points to allow insert
					p[i+1] = p[i];
					p[i] = p[i-1];
					p[i-1] = p_interp; // this will be the next point we interpolate from
					//ai_update_pbh(p, i-1); // update pitch/bank/heading of interp point
					i = i+1;
				}
			}
			i++;
		}
		fclose(f);

		// now update all the pitch/bank/heading values
		for (int x=0; x<i; x++) ai_update_pbhs(p,x);

		// and now do some fix up of headings for low speed stuff
		bool valid_heading = false; // set to true when we have a reasonable speed to trust heading
		for (int x=i-2;x>=0;x--) {
			//check speed > 3m/s
			const int MIN_HEADING_SPEED = 3;
			if (distance(p[x].latitude,p[x].longitude,p[x+1].latitude,p[x+1].longitude)/(p[x+1].zulu_time-p[x].zulu_time) > MIN_HEADING_SPEED) {
				valid_heading = true;
				continue;
			}
			// here we must be < 3m/s, so if we have a valid heading, copy it
			p[x].pitch = 0;
			p[x].bank = 0;
			if (valid_heading) p[x].heading = p[x+1].heading;
		}

		if (debug) {
			wprintf(L"\n",path);
            //debug print IGC file
			//for (int x=0; x<i; x++)
			//	printf("%04d,time,%d,lat,%2.5f,lon,%3.5f,alt,%5.0f,pitch,%.3f,bank,%.3f,heading,%.3f\n",
			//		x,
			//		p[x].zulu_time,
			//		p[x].latitude,
			//		p[x].longitude,
			//		p[x].altitude,
			//		p[x].pitch,
			//		p[x].bank,
			//		p[x].heading);
		}
		ai_info[ai_index].logpoint_count = i;
		ai_info[ai_index].next_logpoint = 0;
		ai_info[ai_index].alt_offset = 0;
		ai_info[ai_index].created = false;
		ai_info[ai_index].default_tried = false;
		ai_info[ai_index].gear_up = false;
		ai_info[ai_index].gear_up_disable_timeout = 0;
		return 0;
	}
}

// load all IGC files from a folder
void load_igc_files(char *folder) {
	wchar_t wfolder[MAXBUF];
	size_t wlen; // length of unicode folder name
	if (debug) printf("Loading IGC files...\n");
	// see if the .FLT file actually exists
	if(_access_s(folder, 0) != 0) {
		// folder not found
        if (debug) printf("IGC folder not found: %s\n", folder);
		return;
	}
	// convert to unicode
	mbstowcs_s(&wlen,wfolder,folder,MAXBUF);
	// folder exists
	SetCurrentDirectory(wfolder);
	// iterate through the files
	WIN32_FIND_DATA next_file;
	HANDLE h;
	h = FindFirstFile(L"*.igc", &next_file);
	if (h == INVALID_HANDLE_VALUE) {
		if (debug) printf("No IGC files found in folder\n");
		return; // didn't even find 1 file in that folder
	}
	do {
        // skip files that contain "[X]"
        if (wcsstr(next_file.cFileName,tracklog_skip_string)!=NULL) {
            if (debug) wprintf(L"Skipping tracklog %s\n", next_file.cFileName);
            continue;
        }
		if (debug) wprintf(L"Loading file %s...", next_file.cFileName);
		if (load_igc_file(ai_count, next_file.cFileName)==0) {
			create_ai(ai_count);
			ai_count++;
		}
	} while (FindNextFile(h,&next_file));
	FindClose(h);
}

//**********************************************************************************
//******************* PROCESS THE FILE LOAD MESSAGES     ***************************
//**********************************************************************************
//**********************************************************************************
void process_aircraft_load_msg(char *filepath) {
	char *c_pointer; // pointer to last '.' in FLT pathname

	if (debug) printf("\n[ EVENT_AIRCRAFT ]: %s\n", filepath);
	// reset the IGC record count and start a new log
    flush_igc(L"auto-save");
	igc_reset_log();
	// copy filename into flight_pathname global
	strcpy_s(air_pathname, filepath);
	// build the cfg_pathname global for aircraft.cfg
	strcpy_s(cfg_pathname, filepath);
	c_pointer = strrchr(cfg_pathname, '\\');
	if (c_pointer!=NULL) {
        strcpy_s(c_pointer+1,30,"aircraft.cfg");
	}
	// calculate checksum for AIR and aircraft.cfg file
	chksum_binary_file(chksum_air, air_pathname);
	chksum_cfg_file(chksum_cfg, cfg_pathname);
	get_startup_data();
}

void process_plan_load_msg(char *filepath) {
	if (debug) printf("\n[ EVENT_FLIGHTPLAN ]: %s\n", filepath);
	// reset the IGC record count and start a new log
    flush_igc(L"auto-save");
	igc_reset_log();
	// copy filename into flight_pathname global
	strcpy_s(pln_pathname, filepath);
	pln_to_c(pln_pathname);
}

// process EVENT_FLIGHT
void process_flt_load_msg(char *flt_filepath) {
	char *c_pointer; // pointer to last '.' in FLT pathname
	char buf[MAXBUF]; // general buffer

    // note time of this flt load
    igc_get_flight_load_time();

    if (debug) {
        wprintf(L"\n[ EVENT_FLIGHT %s]:",flt_load_time);
        printf(" %s\n", flt_filepath);
    }
	// reset the IGC record count and start a new log
    flush_igc(L"auto-save");
	igc_reset_log();
	reset_ai();

	// see if the .FLT file actually exists
	if(_access_s(flt_filepath, 0) != 0) {
		// file not found
		if (debug) printf("FLT file not found\n");
		return;
	} else {
		// file exists
		// check to see if it is 'Previous flight'
		char buf[MAXBUF];
		path_to_name(buf, flt_filepath);
		if (_stricmp(buf,"FSX\\Previous flight.FLT")==0) {
			if (debug) printf("Previous flight loaded\n");
			// loaded Previous flight.FLT
			return;
		}
	}
	// OK we have a kosher .FLT file, so process it

	// save its directory
	path_to_directory(flt_directory, flt_filepath);
	if (debug) wprintf(L"FLT directory: %s\n", flt_directory);

	// copy filename into flt_pathname global
	strcpy_s(flt_pathname, flt_filepath);
	path_to_name(flt_name, flt_pathname);
	// 
	// build the wx_pathname global
	strcpy_s(wx_pathname, flt_filepath);
	c_pointer = strrchr(wx_pathname, '.');
	if (c_pointer!=NULL) {
		c_pointer[1] = 'W';
		c_pointer[2] = 'X';
		c_pointer[3] = '\0';
	}
	// build the cmx_pathname global
	strcpy_s(cmx_pathname, flt_filepath);
	c_pointer = strrchr(cmx_pathname, '.');
	if (c_pointer!=NULL) {
		c_pointer[1] = 'C';
		c_pointer[2] = 'M';
		c_pointer[3] = 'X';
		c_pointer[4] = '\0';
	}
	// build the xml_pathname global for mission xml file
	strcpy_s(xml_pathname, flt_filepath);
	c_pointer = strrchr(xml_pathname, '.');
	if (c_pointer!=NULL) {
		c_pointer[1] = 'X';
		c_pointer[2] = 'M';
		c_pointer[3] = 'L';
		c_pointer[4] = '\0';
	}
	// calculate checksum for FLT, WX, CMX, XML files
	chksum_binary_file(chksum_flt, flt_pathname);
	if (chksum_binary_file(chksum_wx, wx_pathname)==CHKSUM_OK)
		wx_code = 1;
	chksum_binary_file(chksum_cmx, cmx_pathname);
	chksum_binary_file(chksum_xml, xml_pathname);
	get_startup_data();
	// now setup AI objects
	strcpy_s(buf, flt_filepath);
	buf[strlen(buf)-4] = '\0';

	// now load the AI aircraft if replay is enabled
	if (ini_enable_replay) load_igc_files(buf);
}

//*********************************************************************************************
//*********************************************************************************************
// ************************************   MENUS   *********************************************
//*********************************************************************************************
//*********************************************************************************************

char menu_info[12][MAXBUF]; // buffer to hold each menu text item (12 total)
                            // title, prompt, menu 1..9, menu 0.
                            //  0       1          2  10     11 

wchar_t menu_list_entries[MAX_AI][MAXBUF]; // array of filenames
int menu_list_count = 0; // count of IGC files found for menu
int menu_list_index = 0; // index of file at top of current menu

static enum MENU { // list of menu types, e.g. for 'return'
    MENU_NONE, // nothing to do
    MENU_TRACKLOGS,
    MENU_TRACKLOG_INFO,
};

//MENU_CODE menu_status = MENU_PENDING_NONE;

EVENT_ID menu_event = EVENT_MENU_TRACKLOGS_SELECTED1;

MENU menu_back = MENU_NONE; // state variable to hold menu for user 'Return..'

// globals to be used in tracklog info menu and checksum detail menu
char menu_tracklog_date[MAXBUF];
char menu_tracklog_starttime[MAXBUF];
char menu_tracklog_endtime[MAXBUF];
char menu_tracklog_pilot[MAXBUF];
char menu_tracklog_id[MAXBUF];
char menu_tracklog_aircraft[MAXBUF];
long int menu_tracklog_duration;
CHKSUM_RESULT menu_tracklog_g_status;
char menu_tracklog_general_checksum[MAXBUF];
char menu_tracklog_flt_checksum[MAXBUF];
char menu_tracklog_wx_checksum[MAXBUF];
char menu_tracklog_cmx_checksum[MAXBUF];
char menu_tracklog_mission_checksum[MAXBUF];
char menu_tracklog_cfg_checksum[MAXBUF];
char menu_tracklog_air_checksum[MAXBUF];
char menu_tracklog_cx_status[MAXBUF];
char menu_tracklog_wx_status[MAXBUF];
char menu_tracklog_thermals_status[MAXBUF];


//*** CREATE ADD-ON MENU
void create_addon_menu() {
    HRESULT hr;
	hr = SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_LOGGER_MENU);
	//hr = SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_REPLAY_MENU);
	hr = SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_MENU_WRITE_LOG);
	hr = SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_MENU_TRACKLOGS);
	hr = SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_MENU_FOLDERS);
	hr = SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_MENU_RESTART);
	// Add sim_logger menu items
	hr = SimConnect_MenuAddItem(hSimConnect, sim_connect_string, EVENT_LOGGER_MENU, 0);
	//hr = SimConnect_MenuAddItem(hSimConnect, "Sim_logger replay", EVENT_REPLAY_MENU, 0);
	hr = SimConnect_MenuAddSubItem(hSimConnect, EVENT_LOGGER_MENU, lang_save, EVENT_MENU_WRITE_LOG, 0);
	hr = SimConnect_MenuAddSubItem(hSimConnect, EVENT_LOGGER_MENU, lang_restart, EVENT_MENU_RESTART, 0);
	//hr = SimConnect_MenuAddSubItem(hSimConnect, EVENT_REPLAY_MENU, lang_files, EVENT_MENU_TRACKLOGS, 0);
	hr = SimConnect_MenuAddSubItem(hSimConnect, EVENT_LOGGER_MENU, lang_files, EVENT_MENU_TRACKLOGS, 0);
    // disable folder listing for now - will implement later...
	//hr = SimConnect_MenuAddSubItem(hSimConnect, EVENT_REPLAY_MENU, lang_folders, EVENT_MENU_FOLDERS, 0);
	// Sign up for the notifications
	hr = SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_MENU, EVENT_LOGGER_MENU);
	//hr = SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_MENU, EVENT_REPLAY_MENU);
	hr = SimConnect_SetNotificationGroupPriority(hSimConnect, GROUP_MENU, SIMCONNECT_GROUP_PRIORITY_HIGHEST);
}

// function to decide if tracklog is disabled
bool tracklog_disabled(char *filename) {
    return (strstr(filename,tracklog_disable_string)!=NULL);
}

// function to decide if tracklog is 'deleted'
bool tracklog_deleted(char *filename) {
    return (strstr(filename,tracklog_delete_string)!=NULL);
}

void menu_find_files() {
	WIN32_FIND_DATA next_file;
	HANDLE h;
	char s[MAXBUF]; // general buffer
    menu_list_count = 0;
	h = FindFirstFile(L"*.igc", &next_file);
	if (h == INVALID_HANDLE_VALUE) {
		if (debug) printf("No IGC files found in folder\n");
	} else {
		do {
			clean_string(s,next_file.cFileName);
			if (!tracklog_deleted(s)) {
				wcscpy_s(menu_list_entries[menu_list_count], MAXBUF, next_file.cFileName);
				menu_list_count++;
			}
		} while (FindNextFile(h,&next_file));
		FindClose(h);
	}

    menu_list_index = 0; // initialise the menu file index to 0 i.e. first page
}

//debug - this not used yet - correct folder not yet set
void menu_find_folders() {
	WIN32_FIND_DATA next_folder;
	HANDLE h;

    menu_list_count = 0;
	h = FindFirstFileEx(L"*.",FindExInfoStandard, &next_folder, FindExSearchLimitToDirectories, NULL, 0 );
	if (h == INVALID_HANDLE_VALUE) {
		if (debug) printf("No folders found\n");
	} else {
		do {
			wcscpy_s(menu_list_entries[menu_list_count], MAXBUF, next_folder.cFileName);
			menu_list_count++;
		} while (FindNextFile(h,&next_folder));
		FindClose(h);
	}
    menu_list_index = 0; // initialise the menu file index to 0 i.e. first page
}

void menu_list(char *menu_text) {
	char s[MAXBUF]; // general string buffer
	char *pc; // general character pointer
    int menu_index = 0;
    int list_index = menu_list_index;

	// now build the menu_text string, which has title/prompt/item1, etc with NULLS between
	pc = menu_text;
	
    strcpy_s(pc, MAXBUF, lang_replay_title);
	strcpy_s(menu_info[menu_index++], MAXBUF, lang_replay_title);
	pc += strlen(lang_replay_title)+1;

	strcpy_s(pc, MAXBUF, lang_tracklog_select);
	strcpy_s(menu_info[menu_index++], MAXBUF, lang_tracklog_select);
	pc += strlen(lang_tracklog_select)+1;

	//debug - also need to add other identifying info, e.g. length
    // put filenames in menu
    while (menu_index<9 && list_index<menu_list_count) {
		clean_string(s,menu_list_entries[list_index++]);
		strcpy_s(menu_info[menu_index++], MAXBUF, s);
        if (tracklog_disabled(s)) strcat_s(s, MAXBUF, lang_no_replay);
		strcpy_s(pc, MAXBUF, s);
		pc += strlen(s)+1;
	}
    // put blanks in menu to fill up rest of slots (until next, prev, cancel)
    while (menu_index<9) {
		strcpy_s(menu_info[menu_index++], MAXBUF, "");
		strcpy_s(pc, MAXBUF, lang_blank_line);
		pc += strlen(lang_blank_line)+1;
    }
    // either add 'next' to menu or print last filename, or blank if no more pages
    if (list_index<menu_list_count-1) { // if 2+ more files then 'next'
		strcpy_s(menu_info[menu_index++], MAXBUF, "NEXT");
        strcpy_s(pc, MAXBUF, lang_next_tracklogs);
		pc += strlen(lang_next_tracklogs)+1;
    } else if (list_index==menu_list_count-1) { // one more file then filename
		clean_string(s,menu_list_entries[list_index++]);
		strcpy_s(menu_info[menu_index++], MAXBUF, s);
        if (tracklog_disabled(s)) strcat_s(s, MAXBUF, lang_no_replay);
		strcpy_s(pc, MAXBUF, s);
		pc += strlen(s)+1;
    } else {                                // if no more files then 'empty'
		strcpy_s(menu_info[menu_index++], MAXBUF, lang_blank_line);
        strcpy_s(pc, MAXBUF, lang_blank_line);
		pc += strlen(lang_blank_line)+1;
    }
    // either add 'prev' or blank to menu
    if (menu_list_index!=0) {
		strcpy_s(menu_info[menu_index++], MAXBUF, lang_previous_tracklogs);
        strcpy_s(pc, MAXBUF, lang_previous_tracklogs);
		pc += strlen(lang_previous_tracklogs)+1;
    } else {
		strcpy_s(menu_info[menu_index++], MAXBUF, lang_blank_line);
		strcpy_s(pc, MAXBUF, lang_blank_line);
		pc += strlen(lang_blank_line)+1;
    }
    // finally add 'cancel' to menu
	strcpy_s(menu_info[menu_index++], MAXBUF, lang_cancel);
    strcpy_s(pc, MAXBUF, lang_cancel);
	pc += strlen(lang_cancel)+1;
    // and add final null
    *pc = '\0';
}

void menu_list_tracklogs() {
	char menu_text[12 * MAXBUF]; // buffer to hold menu display text
    menu_list(menu_text);
        // toggle menu event to avoid FSX auto-remove
    if (menu_event == EVENT_MENU_TRACKLOGS_SELECTED1)
        menu_event = EVENT_MENU_TRACKLOGS_SELECTED0;
    else
        menu_event = EVENT_MENU_TRACKLOGS_SELECTED1;

	HRESULT hr = SimConnect_Text(hSimConnect, 
					SIMCONNECT_TEXT_TYPE_MENU, 
					0, 
					menu_event, 
					sizeof(menu_text), 
					(void*)menu_text);
}

void menu_list_folders() {
	char menu_text[12 * MAXBUF]; // buffer to hold menu display text
    menu_list(menu_text);
        // toggle menu event to avoid FSX auto-remove
    if (menu_event == EVENT_MENU_FOLDERS_SELECTED1)
        menu_event = EVENT_MENU_FOLDERS_SELECTED0;
    else
        menu_event = EVENT_MENU_FOLDERS_SELECTED1;

	HRESULT hr = SimConnect_Text(hSimConnect, 
					SIMCONNECT_TEXT_TYPE_MENU, 
					0, 
					menu_event, 
					sizeof(menu_text), 
					(void*)menu_text);
}

void menu_tracklog_init() {
    strcpy_s(menu_tracklog_date,"(not in tracklog)");
    strcpy_s(menu_tracklog_starttime,"0");
    strcpy_s(menu_tracklog_endtime,"0");
    strcpy_s(menu_tracklog_pilot,"(not in tracklog)");
    strcpy_s(menu_tracklog_id,"(not in tracklog)");
    strcpy_s(menu_tracklog_aircraft,"(not in tracklog)");
    menu_tracklog_duration = 0;
    menu_tracklog_g_status = CHKSUM_FILE_ERROR;
    strcpy_s(menu_tracklog_general_checksum,"(not in tracklog)");
    strcpy_s(menu_tracklog_flt_checksum,"(not in tracklog)");
    strcpy_s(menu_tracklog_wx_checksum,"(not in tracklog)");
    strcpy_s(menu_tracklog_cmx_checksum,"(not in tracklog)");
    strcpy_s(menu_tracklog_mission_checksum,"(not in tracklog)");
    strcpy_s(menu_tracklog_cfg_checksum,"(not in tracklog)");
    strcpy_s(menu_tracklog_air_checksum,"(not in tracklog)");
    strcpy_s(menu_tracklog_cx_status,"(not in tracklog)");
    strcpy_s(menu_tracklog_wx_status,"(not in tracklog)");
    strcpy_s(menu_tracklog_thermals_status,"(not in tracklog)");
}

bool igc_read(char line_buf[], char tag[], char* dest) {
    char s[MAXBUF];
    int offset = strlen(tag);
	for (int m=0;m<offset;m++) s[m] = line_buf[m];
	s[offset] = '\0';
	if (strcmp(s,tag)==0) {
		int m = 0;
		while (text_char(line_buf[m+offset])) {
			dest[m] = line_buf[m+offset];
			m++;
		}
		dest[m] = '\0';
        return true;
	}
    return false;
}
// display a menu of information about a tracklog
void menu_tracklog(char *filename) {
	char menu_text[12 * MAXBUF]; // buffer to hold menu display text
	char s[MAXBUF]; // general string buffer
	char *pc; // general character pointer

    // initialise the info to be read/calculated from the tracklog
    menu_tracklog_init();

    // check that file exists - just return if not
	if(_access_s(filename, 0) != 0) {
		// file not found
		if (debug) printf("IGC file not found: %s\n", filename);
		return;
	}
	// file exists
	FILE *f;
	errno_t err;
	char line_buf[MAXBUF];
	int i = 0; // record counter
	int j = 0; // general counter

    // try opening it for reading - return if this fails
	if( (err = fopen_s(&f, filename, "r")) != 0 ) {
		return;
	}

	while (fgets(line_buf, MAXBUF, f)!=NULL) {
        // test for B record first, it's the most common...
        if (line_buf[0]=='B') {
            if (strcmp(menu_tracklog_starttime,"0")==0) {
	            if (debug) printf("Start B record: %s\n",line_buf);
                menu_tracklog_starttime[0] = line_buf[1];
                menu_tracklog_starttime[1] = line_buf[2];
                menu_tracklog_starttime[2] = ' ';
                menu_tracklog_starttime[3] = line_buf[3];
                menu_tracklog_starttime[4] = line_buf[4];
                menu_tracklog_starttime[5] = ' ';
                menu_tracklog_starttime[6] = line_buf[5];
                menu_tracklog_starttime[7] = line_buf[6];
                menu_tracklog_starttime[8] = '\0';
            }
            // now cache the end time
            menu_tracklog_endtime[0] = line_buf[1];
            menu_tracklog_endtime[1] = line_buf[2];
            menu_tracklog_endtime[2] = ' ';
            menu_tracklog_endtime[3] = line_buf[3];
            menu_tracklog_endtime[4] = line_buf[4];
            menu_tracklog_endtime[5] = ' ';
            menu_tracklog_endtime[6] = line_buf[5];
            menu_tracklog_endtime[7] = line_buf[6];
            menu_tracklog_endtime[8] = '\0';
            continue;
        }

		if (igc_read(line_buf, "HFDTE", menu_tracklog_date)) continue;
		if (igc_read(line_buf, "HFGTYGLIDERTYPE:", menu_tracklog_aircraft)) continue ;
		if (igc_read(line_buf, "HFCIDCOMPETITIONID:", menu_tracklog_id)) continue;
        if (igc_read(line_buf, "HFPLTPILOTINCHARGE:", menu_tracklog_pilot)) continue;
        if (igc_read(line_buf, "L FSX GENERAL CHECKSUM            ",
            menu_tracklog_general_checksum)) continue;
        if (igc_read(line_buf, "L FSX FLT checksum            ",
            menu_tracklog_flt_checksum)) continue;
        if (igc_read(line_buf, "L FSX WX checksum             ",
            menu_tracklog_wx_checksum)) continue;
        if (igc_read(line_buf, "L FSX CMX checksum            ",
            menu_tracklog_cmx_checksum)) continue;
        if (igc_read(line_buf, "L FSX mission checksum        ",
            menu_tracklog_mission_checksum)) continue;
        if (igc_read(line_buf, "L FSX aircraft.cfg checksum   ",
            menu_tracklog_cfg_checksum)) continue;
        if (igc_read(line_buf, "L FSX AIR checksum            ",
            menu_tracklog_air_checksum)) continue;
        if (igc_read(line_buf, "L FSX CumulusX status:        ",
            menu_tracklog_cx_status)) continue;
        if (igc_read(line_buf, "L FSX WX status:              ",
            menu_tracklog_wx_status)) continue;
        // small bugfix to accept '=' versions prior to 2.0
        if (igc_read(line_buf, "L FSX WX status=              ",
            menu_tracklog_wx_status)) continue;
        if (igc_read(line_buf, "L FSX ThermalDescriptions.xml ",
            menu_tracklog_thermals_status)) continue;

    }
	fclose(f);

	// now build the menu_text string, which has title/prompt/item1, etc with NULLS between
	pc = menu_text;
	
    // title
    strcpy_s(pc, MAXBUF, lang_replay_tracklog_title);
	pc += strlen(lang_replay_tracklog_title)+1;

    // prompt = filename
	strcpy_s(pc, MAXBUF, filename);
    strcpy_s(menu_info[1], MAXBUF, filename);
	pc += strlen(filename)+1;

    // Item #1: date/time
    // calculate duration
    int zulu_start_h, zulu_start_m, zulu_start_s;
    int zulu_end_h, zulu_end_m, zulu_end_s;;
    sscanf_s(menu_tracklog_starttime, "%d %d %d", &zulu_start_h, &zulu_start_m, &zulu_start_s);
    sscanf_s(menu_tracklog_endtime, "%d %d %d", &zulu_end_h, &zulu_end_m, &zulu_end_s);
    menu_tracklog_duration = (zulu_end_h*3600 + zulu_end_m*60 + zulu_end_s) - 
		                       (zulu_start_h*3600 + zulu_start_m*60 + zulu_start_s);
    int hours = (int) (menu_tracklog_duration/3600);
    int mins = (int)((menu_tracklog_duration - hours*3600)/60);
    int secs = menu_tracklog_duration % 60;
	sprintf_s(s, MAXBUF, "FSX date (DD/MM/YY): %c%c/%c%c/%c%c, time: %02d:%02d:%02d (Zulu), duration %02d:%02d:%02d",
		menu_tracklog_date[0], menu_tracklog_date[1], // DD
		menu_tracklog_date[2], menu_tracklog_date[3], // MM
		menu_tracklog_date[4], menu_tracklog_date[5], // YY
        zulu_start_h, zulu_start_m, zulu_start_s, hours, mins, secs);
	strcpy_s(pc, MAXBUF, s);
	pc += strlen(s)+1;

    // Item #2: aircraft, id, pilot
    sprintf_s(s, MAXBUF, "Aircraft/id/pilot:%s /%s /%s",
        menu_tracklog_aircraft, menu_tracklog_id, menu_tracklog_pilot);
	strcpy_s(pc, MAXBUF, s);
	pc += strlen(s)+1;

    // Item #3: validity of IGC file via G checksum
    menu_tracklog_g_status = check_file(filename);
    switch (menu_tracklog_g_status) {
        case CHKSUM_OK:
            strcpy_s(s, MAXBUF,lang_checksum_ok);
            break;
        case CHKSUM_NOT_FOUND:
            strcpy_s(s, MAXBUF,lang_checksum_not_found);
            break;
        case CHKSUM_TOO_SHORT:
            strcpy_s(s, MAXBUF,lang_checksum_too_short);
            break;
        case CHKSUM_BAD:
            strcpy_s(s, MAXBUF,lang_checksum_failed);
            break;
        case CHKSUM_FILE_ERROR:
            strcpy_s(s, MAXBUF,lang_checksum_file_error);
            break;
    }
	strcpy_s(pc, MAXBUF, s);
	pc += strlen(s)+1;

    // Item #4: aircraft, id, pilot
    sprintf_s(s, MAXBUF, "FSX overall checksum: %s", menu_tracklog_general_checksum);
	strcpy_s(pc, MAXBUF, s);
	pc += strlen(s)+1;

    // Item #5 and #6 filler with blanks
    for (int i=0; i<2;i++) {
		strcpy_s(pc, MAXBUF, lang_blank_line);
		pc += strlen(lang_blank_line)+1;
    }

    // Item #7 more checksum detail
    strcpy_s(pc, MAXBUF, lang_detail_checksums);
	pc += strlen(lang_detail_checksums)+1;

    // Item #8 remove from replay
    if (tracklog_disabled(filename)) {
        strcpy_s(pc, MAXBUF, lang_enable_tracklog);
	    pc += strlen(lang_enable_tracklog)+1;
    } else {
        strcpy_s(pc, MAXBUF, lang_disable_tracklog);
	    pc += strlen(lang_disable_tracklog)+1;
    }

    // Item #9 DELETE this file
    strcpy_s(pc, MAXBUF, lang_delete_tracklog);
	pc += strlen(lang_delete_tracklog)+1;

    // Item #0 finally add 'return' to menu
    strcpy_s(pc, MAXBUF, lang_return);
	pc += strlen(lang_return)+1;
    // and add final null
    *pc = '\0';

	HRESULT hr = SimConnect_Text(hSimConnect, 
					SIMCONNECT_TEXT_TYPE_MENU, 
					0, 
					EVENT_MENU_TRACKLOG_INFO, 
					sizeof(menu_text), 
					(void*)menu_text);

}

// display checksum detail for a tracklog
void menu_tracklog_detail() {
	char menu_text[12 * MAXBUF]; // buffer to hold menu display text
	char s[MAXBUF]; // general string buffer
	char *pc; // general character pointer

	// build the menu_text string, which has title/prompt/item1, etc with NULLS between
	pc = menu_text;
	
    // title
    strcpy_s(pc, MAXBUF, lang_replay_tracklog_title);
	pc += strlen(lang_replay_tracklog_title)+1;

    // prompt = filename
	strcpy_s(pc, MAXBUF, menu_info[1]);
	pc += strlen(menu_info[1])+1;

    // Item #1: WX locked/unlocked
    strcpy_s(pc, MAXBUF, lang_weather);
	pc += strlen(lang_weather);
    sprintf_s(s, MAXBUF, "FSX weather menu %s", menu_tracklog_wx_status);
	strcpy_s(pc, MAXBUF, s);
	pc += strlen(s)+1;

    // Item #2: CX locked/unlocked
    strcpy_s(pc, MAXBUF, lang_weather);
	pc += strlen(lang_weather);
    sprintf_s(s, MAXBUF, "CumulusX! settings %s", menu_tracklog_cx_status);
	strcpy_s(pc, MAXBUF, s);
	pc += strlen(s)+1;

    // Item #3: WX checksum
    strcpy_s(pc, MAXBUF, lang_weather);
	pc += strlen(lang_weather);
    sprintf_s(s, MAXBUF, "WX file checksum %s", menu_tracklog_wx_checksum);
	strcpy_s(pc, MAXBUF, s);
	pc += strlen(s)+1;

    // Item #4: CMX checksum
    strcpy_s(pc, MAXBUF, lang_weather);
	pc += strlen(lang_weather);
    sprintf_s(s, MAXBUF, "CMX file checksum %s", menu_tracklog_cmx_checksum);
	strcpy_s(pc, MAXBUF, s);
	pc += strlen(s)+1;

    // Item #5: ThermalDescriptions checksum
    strcpy_s(pc, MAXBUF, lang_weather);
	pc += strlen(lang_weather);
    sprintf_s(s, MAXBUF, "FSX Thermals %s", menu_tracklog_thermals_status);
	strcpy_s(pc, MAXBUF, s);
	pc += strlen(s)+1;

    // Item #6: CFG checksum
    strcpy_s(pc, MAXBUF, lang_aircraft);
	pc += strlen(lang_aircraft);
    sprintf_s(s, MAXBUF, "aircraft.cfg checksum %s", menu_tracklog_cfg_checksum);
	strcpy_s(pc, MAXBUF, s);
	pc += strlen(s)+1;

    // Item #7: AIR checksum
    strcpy_s(pc, MAXBUF, lang_aircraft);
	pc += strlen(lang_aircraft);
    sprintf_s(s, MAXBUF, "AIR file checksum %s", menu_tracklog_air_checksum);
	strcpy_s(pc, MAXBUF, s);
	pc += strlen(s)+1;

    // Item #8: FLT checksum
    strcpy_s(pc, MAXBUF, lang_flight);
	pc += strlen(lang_flight);
    sprintf_s(s, MAXBUF, "FLT file checksum %s", menu_tracklog_flt_checksum);
	strcpy_s(pc, MAXBUF, s);
	pc += strlen(s)+1;

    // Item #9: mission checksum
    strcpy_s(pc, MAXBUF, lang_flight);
	pc += strlen(lang_flight);
    sprintf_s(s, MAXBUF, "mission checksum %s", menu_tracklog_mission_checksum);
	strcpy_s(pc, MAXBUF, s);
	pc += strlen(s)+1;

    // Item #0 finally add 'return' to menu
    strcpy_s(pc, MAXBUF, lang_return);
	pc += strlen(lang_return)+1;
    // and add final null
    *pc = '\0';

	HRESULT hr = SimConnect_Text(hSimConnect, 
					SIMCONNECT_TEXT_TYPE_MENU, 
					0, 
					EVENT_MENU_TRACKLOG_DETAIL, 
					sizeof(menu_text), 
					(void*)menu_text);

}

void remove_menu() {
	static const char Empty1[] = "";
	SimConnect_Text(hSimConnect, 
					SIMCONNECT_TEXT_TYPE_MENU, 
					0, 
					menu_event, 
					sizeof(Empty1), 
					(void*)Empty1);
}

// this routine is called when the user clicks on a tracklog in the list
void menu_tracklog_select(SIMCONNECT_TEXT_RESULT result)
{
    switch(result)
    {
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_1:
        if (debug) printf("Item #1 Selected\n");
        menu_tracklog(menu_info[2]);
        return;
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_2:
        if (debug) printf("Item #2 Selected\n");
        menu_tracklog(menu_info[3]);
        return;
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_3:
        if (debug) printf("Item #3 Selected\n");
        menu_tracklog(menu_info[4]);
        return;
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_4:
        if (debug) printf("Item #4 Selected\n");
        menu_tracklog(menu_info[5]);
        return;
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_5:
        if (debug) printf("Item #5 Selected\n");
        menu_tracklog(menu_info[6]);
        return;
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_6:
        if (debug) printf("Item #6 Selected\n");
        menu_tracklog(menu_info[7]);
        return;
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_7:
        if (debug) printf("Item #7 Selected\n");
        menu_tracklog(menu_info[8]);
        return;
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_8:
        // next OR a filename
        if (debug) printf("Item #8 Selected\n");
        if (strcmp(menu_info[9],"NEXT")==0) {
            menu_list_index += 7;
            menu_list_tracklogs();
        } else {
            menu_tracklog(menu_info[8]);
        }
        return;
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_9:
        // PREVIOUS
        if (debug) printf("Item #9 Selected\n");
        menu_list_index -= 7;
        if (menu_list_index<0) menu_list_index = 0;
        menu_list_tracklogs();
        //remove_current_menu(); // this will trigger a menu reload
        return;
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_10:
        // CANCEL
        if (debug) printf("Item #0 Selected\n");
        return;
    case SIMCONNECT_TEXT_RESULT_DISPLAYED:
        if (debug) printf("Displayed\n");
        return;
    case SIMCONNECT_TEXT_RESULT_QUEUED:
        if (debug) printf("Queued\n");
        return;
    case SIMCONNECT_TEXT_RESULT_REMOVED:
        if (debug) printf("Removed from Queue\n");
        return;
    case SIMCONNECT_TEXT_RESULT_REPLACED:
        if (debug) printf("Replaced in Queue\n");
        return;
    case SIMCONNECT_TEXT_RESULT_TIMEOUT:
        if (debug) printf("Timeout\n");
        return;
    default:
        if (debug) printf("Unknown SIMCONNECT_TEXT_RESULT\n");
        return;
    }
}

// to disable, rename a file <oldname>[X].igc so sim_logger doesn't load it as AI
void menu_disable(char *old_filename) {
    char new_filename[MAXBUF];
    	// first check if file is there
    if(_access_s(old_filename, 0) != 0) {
        if (debug) printf("Tracklog error in menu_disable - \"%s\" not found", old_filename);
        return;
	}

    strcpy_s(new_filename, MAXBUF, old_filename);
    strcpy_s(new_filename+strlen(old_filename)-4, MAXBUF, tracklog_disable_string);
	strcat_s(new_filename, MAXBUF, ".igc");
    int rc = rename(old_filename, new_filename);
    if (debug) {
        if (rc==0) printf("Tracklog \"%s\" renamed to \"%s\"\n", old_filename, new_filename);
        else {
            perror("Rename error");
            printf("Tracklog rename error on \"%s\"\n", old_filename);
        }
    }
}

// to delete, rename a file <oldname>[XX].igc so sim_logger doesn't load it as AI or list it
void menu_delete(char *old_filename) {
    char new_filename[MAXBUF];
    	// first check if file is there
    if(_access_s(old_filename, 0) != 0) {
        if (debug) printf("Tracklog error in menu_disable - \"%s\" not found", old_filename);
        return;
	}

    strcpy_s(new_filename, MAXBUF, old_filename);
    strcpy_s(new_filename+strlen(old_filename)-4, MAXBUF, tracklog_delete_string);
	strcat_s(new_filename, MAXBUF, ".igc");
    int rc = rename(old_filename, new_filename);
    if (debug) {
        if (rc==0) printf("Tracklog \"%s\" renamed to \"%s\"\n", old_filename, new_filename);
        else {
            perror("Rename error");
            printf("Tracklog rename error on \"%s\"\n", old_filename);
        }
    }
}

// 'enable' for AI by removing [X]
void menu_enable(char *old_filename) {
    char new_filename[MAXBUF];
    	// first check if file is there
    if(_access_s(old_filename, 0) != 0) {
        if (debug) printf("Tracklog error in menu_enable - \"%s\" not found", old_filename);
        return;
	}

    strcpy_s(new_filename, MAXBUF, old_filename);
    strcpy_s(new_filename+strlen(old_filename)-strlen(tracklog_disable_string)-4, 
                MAXBUF, 
                old_filename+strlen(old_filename)-4);
    int rc = rename(old_filename, new_filename);
    if (debug) {
        if (rc==0) printf("Tracklog \"%s\" renamed to \"%s\"\n", old_filename, new_filename);
        else {
            perror("Rename error");
            printf("Tracklog rename error on \"%s\"\n", old_filename);
        }
    }
}

void menu_disable_toggle(char *filename) {
    if (tracklog_disabled(filename)) 
        menu_enable(filename);
    else
        menu_disable(filename);
}

// this routine is called when the user clicks somewhere on the tracklog_info menu
void menu_tracklog_info_select(SIMCONNECT_TEXT_RESULT result)
{
    switch(result)
    {
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_1:
        if (debug) printf("Item #1 Selected\n");
        menu_list_tracklogs();
        return;
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_2:
        if (debug) printf("Item #2 Selected\n");
        menu_list_tracklogs();
        return;
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_3:
        if (debug) printf("Item #3 Selected\n");
        menu_list_tracklogs();
        return;
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_4:
        if (debug) printf("Item #4 Selected\n");
        menu_list_tracklogs();
        return;
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_5:
        if (debug) printf("Item #5 Selected\n");
        menu_list_tracklogs();
        return;
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_6:
        if (debug) printf("Item #6 Selected\n");
        menu_list_tracklogs();
        return;
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_7:
        // CHECKSUM DETAIL
        if (debug) printf("Item #7 (DETAIL) Selected\n");
        menu_tracklog_detail();
        return;
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_8:
        // REMOVE FROM REPLAY OR RE-ENABLE
        if (debug) printf("Item #8 (REMOVE/REENABLE) Selected\n");
        // menu_info[1] contains filename of current tracklog...
        menu_disable_toggle(menu_info[1]);
        menu_find_files();
        menu_list_tracklogs();
        return;
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_9:
        // DELETE
        if (debug) printf("Item #9 (DELETE) Selected\n");
		menu_delete(menu_info[1]);
        menu_find_files();
        menu_list_tracklogs();
        return;
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_10:
        // RETURN
        if (debug) printf("Item #10 (RETURN) Selected\n");
        menu_list_tracklogs();
        return;
    case SIMCONNECT_TEXT_RESULT_DISPLAYED:
        if (debug) printf("Displayed\n");
        return;
    case SIMCONNECT_TEXT_RESULT_QUEUED:
        if (debug) printf("Queued\n");
        return;
    case SIMCONNECT_TEXT_RESULT_REMOVED:
        if (debug) printf("Removed from Queue\n");
        return;
    case SIMCONNECT_TEXT_RESULT_REPLACED:
        if (debug) printf("Replaced in Queue\n");
        return;
    case SIMCONNECT_TEXT_RESULT_TIMEOUT:
        if (debug) printf("Timeout\n");
        return;
    default:
        if (debug) printf("Unknown SIMCONNECT_TEXT_RESULT in menu_curent_select()\n");
        return;
    }
}

// this routine is called when the user clicks somewhere on the tracklog detail menu
void menu_tracklog_detail_select(SIMCONNECT_TEXT_RESULT result)
{
    switch(result)
    {
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_1:
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_2:
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_3:
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_4:
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_5:
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_6:
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_7:
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_8:
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_9:
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_10:
        // RETURN
        if (debug) printf("Item #10 (RETURN) Selected\n");
        menu_tracklog(menu_info[1]);
        return;
    case SIMCONNECT_TEXT_RESULT_DISPLAYED:
        if (debug) printf("Displayed\n");
        return;
    case SIMCONNECT_TEXT_RESULT_QUEUED:
        if (debug) printf("Queued\n");
        return;
    case SIMCONNECT_TEXT_RESULT_REMOVED:
        if (debug) printf("Removed from Queue\n");
        return;
    case SIMCONNECT_TEXT_RESULT_REPLACED:
        if (debug) printf("Replaced in Queue\n");
        return;
    case SIMCONNECT_TEXT_RESULT_TIMEOUT:
        if (debug) printf("Timeout\n");
        return;
    default:
        if (debug) printf("Unknown SIMCONNECT_TEXT_RESULT");
        return;
    }
}

void menu_folder_select(SIMCONNECT_TEXT_RESULT result)
{
    switch(result)
    {
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_1:
        if (debug) printf("Item #1 Selected\n");
        return;
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_2:
        if (debug) printf("Item #2 Selected\n");
        return;
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_3:
        if (debug) printf("Item #3 Selected\n");
        return;
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_4:
        if (debug) printf("Item #4 Selected\n");
        return;
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_5:
        if (debug) printf("Item #5 Selected\n");
        return;
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_6:
        if (debug) printf("Item #6 Selected\n");
        return;
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_7:
        if (debug) printf("Item #7 Selected\n");
        return;
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_8:
        // NEXT
        if (debug) printf("Item #8 (NEXT) Selected\n");
        //menu_status = MENU_TRACKLOGS_NEXT;
        menu_list_index += 7;
        menu_list_folders();
        // remove_current_menu(); // this will trigger a menu reload 
        return;
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_9:
        // PREVIOUS
        if (debug) printf("Item #9 (PREVIOUS) Selected\n");
        //menu_status = MENU_TRACKLOGS_PREVIOUS;
        menu_list_index -= 7;
        menu_list_folders();
        //remove_current_menu(); // this will trigger a menu reload
        return;
    case SIMCONNECT_TEXT_RESULT_MENU_SELECT_10:
        // CANCEL
        if (debug) printf("Item #10 (CANCEL) Selected\n");
        return;
    case SIMCONNECT_TEXT_RESULT_DISPLAYED:
        if (debug) printf("Displayed\n");
        return;
    case SIMCONNECT_TEXT_RESULT_QUEUED:
        if (debug) printf("Queued\n");
        return;
    case SIMCONNECT_TEXT_RESULT_REMOVED:
        if (debug) printf("Removed from Queue\n");
        //if (menu_status==MENU_TRACKLOGS_NEXT || 
        //       menu_status==MENU_TRACKLOGS_PREVIOUS) {
        //    menu_status = MENU_PENDING_NONE;
        //    menu_list_tracklogs();
        //}
        return;
    case SIMCONNECT_TEXT_RESULT_REPLACED:
        if (debug) printf("Replaced in Queue\n");
        return;
    case SIMCONNECT_TEXT_RESULT_TIMEOUT:
        if (debug) printf("Timeout\n");
        return;
    default:
        if (debug) printf("Unknown SIMCONNECT_TEXT_RESULT\n");
        return;
    }
}


//*********************************************************************************************
//********** this is the main message handling loop of logger, receiving messages from FS **
//*********************************************************************************************
void CALLBACK MyDispatchProcSO(SIMCONNECT_RECV* pData, DWORD cbData, void *pContext)
{   
    HRESULT hr;
    //printf("\nIn dispatch proc");

    switch(pData->dwID)
    {
        case SIMCONNECT_RECV_ID_EVENT:
        {
            SIMCONNECT_RECV_EVENT *evt = (SIMCONNECT_RECV_EVENT*)pData;

            switch(evt->uEventID)
            {
				case EVENT_MENU_WRITE_LOG:
					if (debug) printf(" [EVENT_MENU_WRITE_LOG]\n");
					igc_write_file(L"");
                    break;
					
				case EVENT_MENU_RESTART:
					if (debug) printf(" [EVENT_MENU_RESTART]\n");
					igc_restart();
                    break;
					
				case EVENT_MENU_TRACKLOGS:
					if (debug) printf(" [EVENT_MENU_TRACKLOGS] ");
                    menu_back = MENU_NONE;
                    menu_find_files();
					menu_list_tracklogs();
                    break;
					
				case EVENT_MENU_TRACKLOGS_SELECTED0:
				case EVENT_MENU_TRACKLOGS_SELECTED1:
					if (debug) printf(" [EVENT_MENU_TRACKLOGS_SELECTEDX] ");
					menu_tracklog_select( (SIMCONNECT_TEXT_RESULT) evt->dwData );
                    break;
					
				case EVENT_MENU_FOLDERS:
					if (debug) printf(" [EVENT_MENU_FOLDERS] ");
                    menu_find_folders();
                    menu_list_folders();
                    break;
					
				case EVENT_MENU_FOLDERS_SELECTED0:
				case EVENT_MENU_FOLDERS_SELECTED1:
					if (debug) printf(" [EVENT_MENU_FOLDERS_SELECTEDX] ");
					menu_folder_select( (SIMCONNECT_TEXT_RESULT) evt->dwData );
                    break;
					
				case EVENT_MENU_TRACKLOG_INFO:
					if (debug) printf(" [EVENT_MENU_TRACKLOG_INFO] ");
					menu_tracklog_info_select( (SIMCONNECT_TEXT_RESULT) evt->dwData );
                    break;
					
				case EVENT_MENU_TRACKLOG_DETAIL:
					if (debug) printf(" [EVENT_MENU_TRACKLOG_DETAIL] ");
					menu_tracklog_detail_select( (SIMCONNECT_TEXT_RESULT) evt->dwData );
                    break;
					
                case EVENT_MENU_TEXT:
					if (debug) printf(" [EVENT_MENU_TEXT] ");
                    break;

                case EVENT_SIM_START:
					if (debug) printf(" [EVENT_SIM_START]\n");
                    // Sim has started so turn the input events on
                    //hr = SimConnect_SetInputGroupState(hSimConnect, INPUT_ZX, SIMCONNECT_STATE_ON);

					// get startup data e.g. zulu time, aircraft info
					get_startup_data();
                    break;

                case EVENT_MISSIONCOMPLETED:
					if (debug) printf(" [EVENT_MISSIONCOMPLETED]\n");
					// always write an IGC file on mission completion
					igc_write_file(L"");
                    break;

                case EVENT_CRASHED:
					if (debug) printf(" [EVENT_CRASHED]\n");
					// always write an IGC file on flight crash
					flush_igc(L"plane crash");
                    igc_reset_log();
                    break;

                case EVENT_Z: // keystroke Z
                    //debug - test sending slew command to object 0
                    //slew_rate += 100;
                    //if (debug) printf("[EVENT_Z] - sending slew command %d\n", slew_rate);
                    //hr = SimConnect_TransmitClientEvent(hSimConnect,
					//				ai_info[0].id,
                    //                //SIMCONNECT_OBJECT_ID_USER,
					//				EVENT_AXIS_SLEW_AHEAD_SET,
					//				slew_rate, // 
					//				SIMCONNECT_GROUP_PRIORITY_HIGHEST,
					//				SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY);

                    break;

                case EVENT_X: // keystroke X
                    break;
                
				case EVENT_CX_CODE: // CumulusX reporting a UI unlock
					if (debug) printf(" [EVENT_CX_CODE]=%d\n",evt->dwData);
					cx_code = evt->dwData;
					break;

                default:
                    if (debug) printf("\nUnknown event: %d\n", evt->uEventID);
                    break;
            }
            break;
        }

        case SIMCONNECT_RECV_ID_ASSIGNED_OBJECT_ID:
        {
            SIMCONNECT_RECV_ASSIGNED_OBJECT_ID *pObjData = (SIMCONNECT_RECV_ASSIGNED_OBJECT_ID*)pData;
    
			// we come here after the create_ai()
            if (pObjData->dwRequestID >= (UINT)REQUEST_AI_CREATE &&
                pObjData->dwRequestID < (UINT)REQUEST_AI_CREATE+MAX_AI) {
           
				UINT ai_index = (UINT)pObjData->dwRequestID - (UINT)REQUEST_AI_CREATE;
				ai_info[ai_index].id = pObjData->dwObjectID;
				if (debug) printf(" [REQUEST_AI_CREATE(%d), dwObjectID=%d] ",ai_index,pObjData->dwObjectID);
				ai_info[ai_index].created = true;
                // send freeze events to ai object
				init_ai(ai_index);
				// set the ATC ID
				AiSetDataStruct ai_set_data;
				strcpy_s(ai_set_data.atc_id, 32, ai_info[ai_index].atc_id);
				if (debug) printf("ATC ID %s\n", ai_set_data.atc_id);
				hr = SimConnect_SetDataOnSimObject(hSimConnect,
													DEFINITION_AI_SET_DATA,
													ai_info[ai_index].id,
													0, 0, sizeof(ai_set_data), &ai_set_data);
				//if (debug) printf("Set AI %d ATC_ID to %s\n",ai_index, ai_set_data.atc_id);
				// request one-second position updates
                get_ai_pos_updates(ai_index);
                incr_ai_created_or_failed();
            } else {
                if (debug) printf("\nUnknown creation %d", pObjData->dwRequestID);
            }
            break;
        }

        case SIMCONNECT_RECV_ID_EVENT_WEATHER_MODE:
        {
            SIMCONNECT_RECV_EVENT *evt = (SIMCONNECT_RECV_EVENT*)pData;

            switch(evt->uEventID)
            {
				case EVENT_WEATHER: // User has changed weather
					if (debug) printf(" [EVENT_WEATHER]\n");
					wx_code = 0;
					break;

				default:
                    if (debug) printf("\nUnknown weather mode event: %d\n", evt->uEventID);
                    break;
			}
			break;
		}
        case SIMCONNECT_RECV_ID_SIMOBJECT_DATA:
        {
            SIMCONNECT_RECV_SIMOBJECT_DATA *pObjData = (SIMCONNECT_RECV_SIMOBJECT_DATA*) pData;

            if (pObjData->dwRequestID >= (UINT)REQUEST_AI_POS &&
                pObjData->dwRequestID < (UINT)REQUEST_AI_POS+MAX_AI) {
           
				UINT ai_index = (UINT)pObjData->dwRequestID - (UINT)REQUEST_AI_POS;
				// these events will come back once per second
				// from get_ai_pos_updates() call
                AIStruct *pU = (AIStruct*)&pObjData->dwData;
                AIStruct pos;
				pos.latitude = pU->latitude;
				pos.longitude = pU->longitude;
				pos.altitude = pU->altitude;
				pos.pitch = pU->pitch;
				pos.bank = pU->bank;
				pos.heading = pU->heading;
				pos.altitude_agl = pU->altitude_agl;
				pos.sim_on_ground = pU->sim_on_ground;
                // now send slew signals to update ai object
                //debug - ai_index should be proper range value
                update_ai(ai_index, pos);
                break;
            }
            
            switch(pObjData->dwRequestID)
            {

                case REQUEST_STARTUP_DATA:
                {
					// startup data will be requested at SIM START
					if (debug) printf(" [REQUEST_STARTUP_DATA] ");
                    StartupStruct *pU = (StartupStruct*)&pObjData->dwData;
					startup_data.start_time = pU->start_time;
					startup_data.zulu_day = pU->zulu_day;
					startup_data.zulu_month = pU->zulu_month;
					startup_data.zulu_year = pU->zulu_year;
                    zulu_clock_sync(pU->start_time);
					if (debug) printf("\nStartup data: Zulu time=%d-%d-%d@%d\n",
									  startup_data.zulu_year, startup_data.zulu_month,
									  startup_data.zulu_day, startup_data.start_time);
					// now we have the startup data, request user pos updates
					get_user_pos_updates();
                    break;
                }

                case REQUEST_AIRCRAFT_DATA:
                {
					// startup data will be requested at SIM START
					if (debug) printf(" [REQUEST_AIRCRAFT_DATA] ");
					AircraftStruct *pS = (AircraftStruct*)&pObjData->dwData;
                    char *pszATC_ID;
                    char *pszATC_TYPE;
                    char *pszTITLE;
                    DWORD cbATC_ID;
					DWORD cbATC_TYPE;
					DWORD cbTITLE;

					// Note how the third parameter is moved along the data received
                    if(SUCCEEDED(SimConnect_RetrieveString(pData, cbData, &pS->strings, &pszATC_ID, &cbATC_ID)) &&
                       SUCCEEDED(SimConnect_RetrieveString(pData, cbData, pszATC_ID+cbATC_ID, &pszATC_TYPE, &cbATC_TYPE)) &&
					   SUCCEEDED(SimConnect_RetrieveString(pData, cbData, pszATC_TYPE+cbATC_TYPE, &pszTITLE, &cbTITLE)))
                    {
                       if (debug) printf("\nATC_ID = \"%s\" ATC_TYPE = \"%s\" TITLE = \"%s\"\n",
                                pszATC_ID, pszATC_TYPE, pszTITLE );
       					strcpy_s(ATC_ID, pszATC_ID);
       					strcpy_s(ATC_TYPE, pszATC_TYPE);
       					strcpy_s(TITLE, pszTITLE);

                    } else
						if (debug) printf("\nCouldn't retrieve the aircraft strings.");
                    break;
                }

                case REQUEST_USER_POS:
				{
					// these events will come back once per second
					// from get_user_pos_updates() call
                    UserStruct *pU = (UserStruct*)&pObjData->dwData;
					user_pos.latitude = pU->latitude;
					user_pos.longitude = pU->longitude;
					user_pos.altitude = pU->altitude;
					user_pos.sim_on_ground = pU->sim_on_ground;
					user_pos.zulu_time = pU->zulu_time;
					user_pos.rpm = pU->rpm;

                    zulu_clock_sync(pU->zulu_time);

					// store position to igc log array on every nth tick
					if (++igc_tick_counter==IGC_TICK_COUNT) {
						//if (debug) printf("B(%d, %.2f) ",user_pos.zulu_time,zulu_clock);
						igc_log_point(user_pos);
						igc_tick_counter = 0;
					}
					if (debug_events && user_pos.sim_on_ground!=0) printf(" [REQUEST_USER_POS (%d)G] ", igc_record_count);
					if (debug_events && user_pos.sim_on_ground==0) printf(" [REQUEST_USER_POS (%d)A] ", igc_record_count);
					// process 'on ground' status and decide whether to write a log file
					igc_ground_check(user_pos.sim_on_ground, user_pos.zulu_time);
                    break;
                }

                default:
					if (debug_info || debug) printf("\nUnknown SIMCONNECT_RECV_ID_SIMOBJECT_DATA request %d", pObjData->dwRequestID);
                    break;

            }
            break;
        }

		case SIMCONNECT_RECV_ID_EVENT_OBJECT_ADDREMOVE:
        {
            SIMCONNECT_RECV_EVENT_OBJECT_ADDREMOVE *evt = (SIMCONNECT_RECV_EVENT_OBJECT_ADDREMOVE*)pData;
            
            switch(evt->uEventID)
            {
                case EVENT_OBJECT_REMOVED:
					for (int i=0; i<ai_count; i++) {
						if (evt->dwData == ai_info[i].id) {
							if (debug) printf("[EVENT_OBJECT_REMOVED ai object[%d] ]\n", i);
							ai_info[i].created = false;
							break;
						}
					}
                    break;

                //case EVENT_REMOVED_AIRCRAFT:
                //    printf("\nAI object removed: Type=%d, ObjectID=%d", evt->eObjType, evt->dwData);
                //    break;
				default:
					if (debug) printf("\n\n*Unrecognized SIMCONNECT_RECV_ID_EVENT_OBJECT_ADDREMOVE Type=%d, ObjectID=%d", evt->eObjType, evt->dwData);
					break;

            }
            break;
        }

        case SIMCONNECT_RECV_ID_EXCEPTION:
        {
            SIMCONNECT_RECV_EXCEPTION *except = (SIMCONNECT_RECV_EXCEPTION*)pData;
            switch(except->dwException)
            {
                case SIMCONNECT_EXCEPTION_CREATE_OBJECT_FAILED:
                    if (debug) printf("CREATE_OBJECT_FAILED EXCEPTION\n");
                    // increment the count and trigger a retry if needed
                    ai_failed = true;
                    incr_ai_created_or_failed();
                    break;

				default:
					if (debug_info || debug) printf("\n\nsim_logger EXCEPTION=%d  SendID=%d  Index=%d  cbData=%d\n", except->dwException, except->dwSendID, except->dwIndex, cbData);
					break;
            }
            break;
        }

        case SIMCONNECT_RECV_ID_OPEN:
        {
            SIMCONNECT_RECV_OPEN *open = (SIMCONNECT_RECV_OPEN*)pData;
			if (debug) printf("Connected to FSX Version %d.%d\n", open->dwApplicationVersionMajor, open->dwApplicationVersionMinor);
            break;
        }

		case SIMCONNECT_RECV_ID_EVENT_FILENAME:
        {
            SIMCONNECT_RECV_EVENT_FILENAME *evt = (SIMCONNECT_RECV_EVENT_FILENAME*)pData;
            switch(evt->uEventID)
            {
                case EVENT_FLIGHT:
					// FLT file loaded
                    process_flt_load_msg(evt->szFileName);
                    break;

                case EVENT_AIRCRAFT:
                    // AIR file loaded
                    process_aircraft_load_msg(evt->szFileName);
                    break;

                case EVENT_FLIGHTPLAN:
                    // PLN file loaded
                    process_plan_load_msg(evt->szFileName);
                    break;

                default:
		            if (debug_info || debug) printf("\nUnrecognized RECV_ID_EVENT_FILENAME Received:%d\n",evt->uEventID);
                   break;
            }
            break;
        }

        case SIMCONNECT_RECV_ID_QUIT:
        {
			// write the IGC file if there is one
            flush_igc(L"quit");
			// set flag to trigger a quit
            quit = 1;
            break;
        }

        default:
            if (debug_info || debug) printf("\nUnrecognized RECV_ID Received:%d\n",pData->dwID);
            break;
    }
}

void connectToSim()
{
    HRESULT hr;

	sprintf_s(sim_connect_string, 
				sizeof(sim_connect_string), 
				"Sim_logger v%.2f", 
				version);

    if (SUCCEEDED(SimConnect_Open(&hSimConnect, sim_connect_string, NULL, 0, 0, 0)))
    {
        //if (debug_info || debug) printf("SimConnect_Open succeeded\n", version);   
          
        // Create some private events
        //debug - don't need this key event
        hr = SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_Z);
        //hr = SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_X);

        // Sign up for notifications
        //debug
        hr = SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_ZX, EVENT_Z);
        //hr = SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_ZX, EVENT_X);

        // Link the private events to keyboard keys, and ensure the input events are off
        //debug - test on Z key not needed in prod
        hr = SimConnect_MapInputEventToClientEvent(hSimConnect, INPUT_ZX, "Z", EVENT_Z);
        //hr = SimConnect_MapInputEventToClientEvent(hSimConnect, INPUT_ZX, "X", EVENT_X);
        hr = SimConnect_SetInputGroupState(hSimConnect, INPUT_ZX, SIMCONNECT_STATE_ON);

		// display the sim_logger addon menu
		create_addon_menu();

        // DEFINITION_AIRCRAFT
        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_AIRCRAFT,
                                            "ATC ID", 
                                            NULL,
											SIMCONNECT_DATATYPE_STRINGV);

        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_AIRCRAFT,
                                            "ATC TYPE", 
                                            NULL,
											SIMCONNECT_DATATYPE_STRINGV);

        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_AIRCRAFT,
                                            "TITLE", 
                                            NULL,
											SIMCONNECT_DATATYPE_STRINGV);

		// DEFINITION_STARTUP
        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_STARTUP,
                                            "ZULU TIME", 
                                            "seconds",
											SIMCONNECT_DATATYPE_INT32);

        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_STARTUP,
                                            "ZULU DAY OF MONTH", 
                                            "number",
											SIMCONNECT_DATATYPE_INT32);

        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_STARTUP,
                                            "ZULU MONTH OF YEAR", 
                                            "number",
											SIMCONNECT_DATATYPE_INT32);

        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_STARTUP,
                                            "ZULU YEAR", 
                                            "number",
											SIMCONNECT_DATATYPE_INT32);

		// DEFINITION_USER_POS
        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_USER_POS,
                                            "Plane Latitude", 
                                            "degrees");

        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_USER_POS,
                                            "Plane Longitude", 
                                            "degrees");

        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_USER_POS,
                                            "PLANE ALTITUDE", 
                                            "meters");

        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_USER_POS,
                                            "SIM ON GROUND", 
                                            "bool",
											SIMCONNECT_DATATYPE_INT32);

        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_USER_POS,
                                            "ZULU TIME", 
                                            "seconds",
											SIMCONNECT_DATATYPE_INT32);

        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_USER_POS,
                                            "GENERAL ENG RPM:1", 
                                            "Rpm",
											SIMCONNECT_DATATYPE_INT32);

        // DEFINITION_AI_MOVE - move an ai object
		hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_AI_MOVE, 
                                            "PLANE LATITUDE", 
                                            "Degrees");

        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_AI_MOVE, 
                                            "PLANE LONGITUDE", 
                                            "Degrees");

        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_AI_MOVE, 
                                            "PLANE ALTITUDE", 
                                            "Meters");

        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_AI_MOVE, 
                                            "PLANE PITCH DEGREES", 
                                            "Radians");

        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_AI_MOVE, 
                                            "PLANE BANK DEGREES", 
                                            "Radians");

        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_AI_MOVE, 
                                            "PLANE HEADING DEGREES TRUE", 
                                            "Radians");

        // DEFINITION_AI_POS - position of ai object
		hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_AI_POS, 
                                            "PLANE LATITUDE", 
                                            "Degrees");

        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_AI_POS, 
                                            "PLANE LONGITUDE", 
                                            "Degrees");

        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_AI_POS, 
                                            "PLANE ALTITUDE", 
                                            "Meters");

        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_AI_POS, 
                                            "PLANE PITCH DEGREES", 
                                            "Radians");

        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_AI_POS, 
                                            "PLANE BANK DEGREES", 
                                            "Radians");

        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_AI_POS, 
                                            "PLANE HEADING DEGREES TRUE", 
                                            "Radians");

        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_AI_POS, 
                                            "PLANE ALT ABOVE GROUND", 
                                            "Meters");

        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_AI_POS,
                                            "SIM ON GROUND", 
                                            "bool",
											SIMCONNECT_DATATYPE_INT32);

        // DEFINITION_AI_SET_DATA - position of ai object
		hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_AI_SET_DATA, 
                                            "ATC ID",
											NULL,
                                            SIMCONNECT_DATATYPE_STRING32);

		// Listen for the CumulusX.ReportSessionCode event
		hr = SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_CX_CODE, "CumulusX.ReportSessionCode");
		hr = SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_ZX, EVENT_CX_CODE, false);
		hr = SimConnect_SetNotificationGroupPriority(hSimConnect, GROUP_ZX, SIMCONNECT_GROUP_PRIORITY_DEFAULT);

        // Listen for a simulation start event
        hr = SimConnect_SubscribeToSystemEvent(hSimConnect, EVENT_SIM_START, "SimStart");

        // Subscribe to the FlightLoaded event to detect flight start and end
        hr = SimConnect_SubscribeToSystemEvent(hSimConnect, EVENT_FLIGHT, "FlightLoaded");

        // Subscribe to the MissionCompleted event to detect flight end
        hr = SimConnect_SubscribeToSystemEvent(hSimConnect, EVENT_MISSIONCOMPLETED, "MissionCompleted");

        // Subscribe to the Crashed event to detect aircraft crash and save IGC file
        hr = SimConnect_SubscribeToSystemEvent(hSimConnect, EVENT_CRASHED, "Crashed");

        // Subscribe to the MissionCompleted event to detect flight end
        hr = SimConnect_SubscribeToSystemEvent(hSimConnect, EVENT_AIRCRAFT, "AircraftLoaded");

        // Subscribe to the MissionCompleted event to detect flight end
        hr = SimConnect_SubscribeToSystemEvent(hSimConnect, EVENT_FLIGHTPLAN, "FlightPlanActivated");

        // Subscribe to the MissionCompleted event to detect flight end
        hr = SimConnect_SubscribeToSystemEvent(hSimConnect, EVENT_WEATHER, "WeatherModeChanged");

		//  set the id for the freeze events so this client has full control of probe objects
		hr = SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_FREEZE_LATLONG, "FREEZE_LATITUDE_LONGITUDE_SET");
		hr = SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_FREEZE_ALTITUDE, "FREEZE_ALTITUDE_SET");
		hr = SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_FREEZE_ATTITUDE, "FREEZE_ATTITUDE_SET");

        // AI slew events
		hr = SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_SLEW_ON, "SLEW_ON");
        hr = SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AXIS_SLEW_AHEAD_SET,   "AXIS_SLEW_AHEAD_SET");
        hr = SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AXIS_SLEW_ALT_SET,     "AXIS_SLEW_ALT_SET");
        hr = SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AXIS_SLEW_HEADING_SET, "AXIS_SLEW_HEADING_SET");
        hr = SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AXIS_SLEW_BANK_SET,    "AXIS_SLEW_BANK_SET");
        hr = SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AXIS_SLEW_PITCH_SET,   "AXIS_SLEW_PITCH_SET");
        //debug
        hr = SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_SLEW_ALTIT_UP_SLOW,    "SLEW_ALTIT_UP_SLOW");
		//hr = SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AXIS_SLEW_AHEAD_SET, "AXIS_SLEW_AHEAD_SET");
		// ai gear events
        hr = SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_GEAR_UP,  "GEAR_UP");
        hr = SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_GEAR_DOWN,"GEAR_DOWN");

		// Now loop checking for messages until quit
		hr  = S_OK;
        while( hr == S_OK && 0 == quit )
        {
            hr = SimConnect_CallDispatch(hSimConnect, MyDispatchProcSO, NULL);
            Sleep(1);
        } 
		if (hr==S_OK) hr = SimConnect_Close(hSimConnect);
		else {
			if (debug) printf("Fail code from CallDispatch\n");
			// write the IGC file if there is one
			flush_igc(L"fsx crash");
		}

	} else {
	    if (debug) printf("Couldn't connect to FSX.. logger will exit now\n");
	}
}


//int __cdecl _tmain(int argc, _TCHAR* argv[])
int main(int argc, char* argv[])
{
	igc_reset_log();

	// set up command line arguments (debug mode)
	for (int i=1; i<argc; i++) {
		if (strcmp(argv[i],"debug")==0) {
			debug = true;
			debug_info = false;
		}
		else if (strcmp(argv[i],"info")==0)      debug_info = true; // will open console window
		else if (strcmp(argv[i],"calls")==0)     debug_calls = true;
		else if (strcmp(argv[i],"events")==0)    debug_events = true;
	}

	// debug
	//load_igc_file(0,L"interp_test.igc");
	//return 0;

	// set up FSXBASE path
    HKEY keyHandle;
	DWORD size1 = MAXBUF;
    DWORD Type;
    if( RegOpenKeyExW(    HKEY_LOCAL_MACHINE,
						L"SOFTWARE\\Microsoft\\Microsoft Games\\Flight Simulator\\10.0",
						0,
						KEY_QUERY_VALUE, 
						&keyHandle) == ERROR_SUCCESS) {
        size1=1023;
        RegQueryValueExW( keyHandle, 
							L"SetupPath", 
							NULL, 
							&Type,
							(LPBYTE)FSXBASE,
							&size1);
        if (debug) wprintf(L"FSXBASE from registry=%s\n",FSXBASE);
    }     
	else { 
		if (debug) wprintf(L"Registry data for FSX not found (so using start folder of sim_logger).\n");
		GetCurrentDirectory(MAXBUF, FSXBASE);
		wcscat_s(FSXBASE, MAXBUF, L"\\");
		if (debug) wprintf(L"FSXBASE from currentfolder=%s\n", FSXBASE);
	}	    
    RegCloseKey(keyHandle);


	// set up MYDOCS path
	HRESULT hr = SHGetFolderPath(NULL, CSIDL_PERSONAL, NULL,
                             SHGFP_TYPE_CURRENT, MYDOCS);
	wcscat_s(MYDOCS, L"\\");
	if (debug) wprintf(L"User documents=%s\n", MYDOCS);


	// set up FSXFILES path
	wcscpy_s(FSXFILES, MYDOCS); // start from my docs folder
	// find name (not path) of fsx files folder from FSXBASE\language.dll
	bool fsx_files_done = false;
	wchar_t dll_path[MAXBUF];
	wchar_t fsx_files[MAXBUF];
	wcscpy_s(dll_path,MAXBUF,FSXBASE);
	wcscat_s(dll_path,MAXBUF,L"language.dll");
	//if (debug) wprintf(L"Loading dll=%s\n",dll_path);
	HINSTANCE hInstLang = LoadLibraryW(dll_path);
	if (hInstLang)
	{ 
		//if (debug) wprintf(L"Loaded dll=%s\n", dll_path);
		if (LoadStringW(hInstLang, 36864, &fsx_files[0], 128)!=0) {
			if (debug) wprintf(L"FSX files folder (from language.dll)=%s\n",fsx_files);
			// append fsx_files to FSXFILES
			wcscat_s(FSXFILES, fsx_files);
			fsx_files_done = true;
		}
		FreeLibrary(hInstLang);
	}
	// if that didn't work, use English version...
	if (!fsx_files_done) {
		if (debug) wprintf(L"Failed to read FSXBASE\\language.dll, defaulting to English\n");
		wcscat_s(FSXFILES, L"Flight Simulator X Files");
	}
	wcscat_s(FSXFILES,L"\\");
	if (debug) wprintf(L"FSXFILES=%s\n",FSXFILES);

	//debug
	return 0;

	// load values from sim_logger.ini
	load_ini();

    // load language string
    load_lang();

	// debug
	//ini_write_string(L"enable_replay", L"true");
	//return 0;

	// disable FSX thermals if sim_logger.ini permits it
	fsx_thermals_enabled = disable_fsx_thermals();
	//return 0;

	if (argc==2 && !debug) {
		printf("\nChecking igc file checksum\n");
		Sleep(1000);printf(".");
		Sleep(1000);printf(".");
		Sleep(1000);printf(".\n");
		
		switch (check_file(argv[1]))
		{
			case CHKSUM_OK:
				printf("IGC file checks OK.\n");
				break;

			case CHKSUM_TOO_SHORT:
				printf("BAD CHECKSUM. This file contains a checksum but it is too short.\n");
				break;

			case CHKSUM_NOT_FOUND:
				printf("BAD CHECKSUM. This file does not contain a 'G' record.\n");
				break;

			case CHKSUM_BAD:
				printf("BAD CHECKSUM. 'G' record found but checksum is wrong.\n");
				break;

			case CHKSUM_FILE_ERROR:
				printf("FILE ERROR. Couldn't read the igc file \"%s\".\n", argv[1]);
				break;
		}

		return 0;
	}

	if (!debug && !debug_info) FreeConsole(); // kill console unless requested

	if (debug) {
		printf("Starting logger version %.2f in debug mode\n", version);
		if (debug_info) printf("+info");
		if (debug_calls) printf("+calls");
		if (debug_events) printf("+events");
	} else if (debug_info) {
		printf("Debug mode = debug_info\n");
	}

    connectToSim();
    return 0;
}