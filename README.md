Can anyone comment on the best way to move an AI object smoothly under the direct control of a simconnect client ? The issue has been around for a long time, fsrecorder seems to do pretty well so its possible, but I've seen lots of posts but no good answer. To some this post will seem a bit of a necro of the issue but I'm hoping experience has been gained since Russell's post.

I do not believe this is purely a function of update rates, and do not want to get into a debate about the latency of named pipes/udp/tcp etc. or why simconnect sucks.

Russell did a good job of posting this video in this other fsdeveloper post from last year, but I don't think he ever got a clear answer.

I do 100% understand the comments from folks that *if* you could update the AI position (lat/long/alt) 100 times a second *and* pass those updates to FSX with zero latency *and* ensure the updates are synchronised with the millisecond timestamp of the FSX frames *then* it would be smooth, but I don't think this is a viable technique, even though it seems the simplest.

My fundamental question is "is it possible to have the plane move smoothly between points relative to a shadowing user aircraft" even if the position updates are relatively infrequent from the simconnect client (e.g. even once every FOUR seconds). This would mean you would have be sending velocity or acceleration vectors to the AI object, not just simple lat/long/alt updates every visual frame. Like Russell, I'm *not* concerned at this stage with the jumps that may then occur every four seconds - it's the relative jitter during linear movement I want to address at this stage. I have no problem at all with the FREEZE and DISABLE SIM commands.

I think the basic answer needed is whether the AI aircraft can be given a position update including (e.g.) VELOCITY BODY Z and have that do what you'd expect until changed, or maybe the slew mode is helpful in a similar way.

So my requirement would be satisfied if I just converted all the IGC files into a single fs-recorder 'frc' file and used fs-recorder, but there are a couple of subtleties that might make it worth writing my own code - mainly the correct handling of the timestamps in the IGC files.

So there are a few possible methods:

* just 'freeze lat/long/alt' and set the lat/lon/alt of the AI plane every visual sim frame - this gives apparent 'jitter' if you watch the AI aircraft from a user aircraft flying alongside. I have this working but with jitter, interpolating between the IGC trackpoints within simconnect and sending each update via SetDataOnSimObject.

* using 'freeze' events or 'sim disable', initially set the lat/long/alt of the AI aircraft in the sky but set 'heading' and 'velocity body z' in the hope the aircraft then actually moves - I've tried this but currently have the a/c stationary between SetDataOnSimObject calls even though I am setting velocity body z. Still looking at this though.

* set the heading of the AI aircraft using SetDataOnSimObject and put the AI aircraft into 'slew' mode and send the appropriate set-forward-slew-speed event - haven't tried this yet but at least will have to 'calibrate' the meaning of the slew events.

* use the simconnect AI waypoint function to dynamically set waypoints in front of the AI aircraft - my concern is I'd have maybe 3000 waypoints per AI aircraft (that's how many trackpoints per IGC file) plus I'd like to see if I could efficiently move 100 AI aircraft. Given the tracklog file contains the real path of an aircraft so the physics is pre-computed, I don't think I'd want FSX to try and re-compute the flying physics for each small step in the log unless there was no other way. I've used AI objects with waypoints in one of my missions, so I get the concept, although there are a few challenges using gliders...

in summary, the following approach is looking promising, having been tested to show 100% smooth movement of the ai object relative to the user aircraft:

(1) put the AI object into slew mode by sending in the "SLEW ON" event

(2) alter the forward speed of the object by sending it the "AXIS_SLEW_AHEAD_SET" event, or rotate it in pitch, bank or heading with the similar AXIS_SLEW_.. events. The DWORD value in the TransmitClientEvent controls the *speed* of the movement, not the absolute position of the object...
Here's something new, i.e. not a repeat of the posts we lost in the server crash: the formulae to give you the desired speeds are:
-x-x-x-x- AHEAD (DWORD) slew_rate = sqrt( "speed meters per second" * 45678 )
-x-x-x-x- ROTATE (DWORD) slew_rate = sqrt( "rotate radians/second" * 11240000)
Note that the speed of rotation for a given slew_rate DWORD value is the *same* whether you're setting pitch, bank or heading, assuming you're using the corresponding TransmitClientEvent. A POSITIVE value for the slew_rate DWORD value rotates the object HEADING to PORT, gives BANK PORT WING DOWN, or PITCH NOSE DOWN.

(3) because you are only *nudging* the ai object in the direction you want it to go (i.e. you are controlling velocities not the absolute position) you have to use RequestDataOnSimObject for the AI object say once-per-second and get the lat/long/alt/pitch/bank/heading of the ai object, so you can compare that with the position you actually want the aircraft, and decide which slew value to give it. If it's not clear, you are sending AXIS_SLEW_.._SET at most one-per-second, adjusting the *speed* with which the object moves forwards or rotates in pitch, bank or heading. If your object were an airliner, you could adjust the slew rates a lot less frequently.

This approach has the potential to be smoother than the simplest move-the-aircraft-through-explicit-lat/long/alt positions (requires updates on a per-frame basis) and far more efficient than the use of 'real' FSX AI flying through waypoints (requires air physics computed for every aircraft). But I haven't completed the programming of a multi-aircraft replay to see how effective the slew technique is in actual FSX...

I've repeated your testing using the traffic toolbox and confirmed your findings for vertical speed, and I've reverse-engineered the formula behind the KEY_AXIS_SLEW_ALT_SET DWORD value. So now we have:

* Forward movement (m/s) : AXIS_SLEW_AHEAD_SET, DWORD = sqrt( "speed meters per second" * 45678 )

* rotation speed pitch, bank, heading (rad/s): DWORD rate = sqrt( "rotate radians/second" * 11240000), using
** AXIS_SLEW_PITCH_SET (+ve = nose down) (NO CHANGE TO ALTITUDE)
** AXIS_SLEW_BANK_SET (+ve = port wing down)
** AXIS_SLEW_HEADING_SET (+ve = turn to port) (CHANGES DIRECTION OF MOVEMENT)

* vertical speed (m/s): AXIS_SLEW_ALT_SET, DWORD = sqrt("vertical speed m/s" * 3084000)
** +ve value moves aircraft DOWN.
** tested for vertical speeds -0..15m/s, rate 0..10,000. Have NOT tested negative values.

Adjusting vertical speed ('ALT') does not seem to alter the forward speed ('AHEAD'). This is actually helpful if you are following timestamped waypoints as you can calculate the forward speed necessary to reach the next waypoint at the correct time without worrying about the extra length of a descending diagonal, and the required vertical speed can be computed independently. It just means the true airspeed is actually the hypotenuse of AHEAD and ALT m/s speeds (sqrt(AHEAD^2+ALT^2)). Also ALT being independent of pitch means you can adjust the pitch of the AI aircraft at low speeds without causing the aircraft to climb...

Update - I now have an AI object following smoothly along the path created by a 'real' FSX aircraft sampled once-per-four-seconds (that I call the *lead* aircraft), by updating the slew rates of the AI aircraft once-per-second.

The basic idea is as described earlier in this thread, but that still left a lot of programming to simply control the AI object by adjusting the slew rates. The basic technique is a "following" AI algorithm as illustrated here. I *predict* where the lead aircraft is going to be FOUR SECONDS AHEAD (the predicted_point), and adjust the slew rates of the AI aircraft to turn towards that point.

All motion of the AI object is by sending the slew AHEAD/HEADING/BANK/PITCH/ALT commands which adjusts the corresponding linear or rotational speed, and in between my per-second updates the AI aircraft continues with the previously set velocities, i.e. if it's turning it will continue to turn. Testing shows the movement has none of the jitter present when doing absolute position updates via simconnect.

Manuel - to your specific question - for my purposes I am *not* reading the speed of the lead aircraft and sending that as a "slew ahead" event to the AI object - I am computing the speed required to reach the predicted_point of the lead aircraft from the AI object current position at the correct time, and sending the "slew ahead" rate appropriate for that speed - this will be very similar to the lead aircraft speed but has the effect that the AI object is always automatically correcting its position by turning *and* accelerating. The altitude control is the same - I compare the current AI altitude with the altitude of the lead aircraft 'predicted_point', and send the appropriate slew ALT command that would adjust the altitude over the next four seconds. A small subtlety is this computation is actually performed every *second* as that's the frequency I get the AI position updates via simconnect.

So the general principle remains to monitor the position of the AI object once-per-second, and send it slew commands to adjust its forward or vertical speed or adjust its rotational speed to continually correct its direction towards a forward predicted point of the lead aircraft.

My interest is to replay 'IGC files', which are soaring GPS log files giving periodic samples of timestamped lat/long/alt. The technique could be used for multi-player if the lat/long/alt readings were piped over the network, but I'm not sure of a new requirement as that capability is already available in standard FSX. For what it's worth, I don't know what the non-Microsoft multi-player modules do - my guess is they try and control the exact position of the AI aircraft representing each user on a per-frame basis in the local client, but interpolate between infrequent updates across the network.

I've now flown with AI objects following a few different IGC tracklogs, but I haven't loaded up multiple tracklogs to see how the replay performance scales. I'm not far off from that though.

I have the AI object in 'slew' mode, by sending it the 'slew on' event, and am controlling the body variables specifically in that mode e.g. by sending AXIS_SLEW_AHEAD_SET, DWORD=1000. This starts the object moving smoothly forwards and it will continue with a constant velocity until the next update from simconnect, regardless of how long I delay before doing that. I've found 1-second updates work quite well to guide an AI object along a chosen path. Was there something you've seen in slew mode that suggests some other issue? I'm well aware of the issues in regular flight mode, which is what caused me to investigate slew mode in the first place.

At the moment I have an aircraft flying smoothly around SeaTac airport, following a track recorded from another aircraft. I'm about to scale up the test with maybe 100 aircraft following that same path (with small offsets) to see what the performance looks like...

you don't need (or want) to re-use request id's. The essential method is described by Beatle here.

You're just trying to ensure request id's are unique, with the confusing bit being they're defined in an 'enum' which seems to suggest each one has to be explicitly defined, but in fact you can use thousands (tweaked from Beatle):
Code:
```
enum Requests
{
    REQUEST1,
    REQUEST2,
    REQUEST_AI_POS =   0x00100000, // here's the example we're interested in
    REQUEST3 = 0x00200000, // this entry ensures a large range is free
}

UINT ai_index = 0;

// and then wherever you want a request id for a new aircraft, use
//  (UINT)REQUEST_AI_POS + ai_index

// and where you generally have a 'case' statement in your message
 processing procedure, *precede* that with

    if (dwRequestID >= (UINT)REQUEST_AI_POS &&
        dwRequestID < (UINT)REQUEST_AI_POS + 10000)
          // choose some max number of objects you will create at one time
    {
       UINT ai_index = (UINT)dwRequestID - (UINT)REQUEST_AI_POS;
        // do something relevant here
    }
```

I *create* the AI aircraft with some suitable init position - sound like you've got this bit ok

I send the 'SLEW ON' event to the AI object (and never send the SLEW OFF)

I RequestDataOnSimObject for time/lat/long/alt/pitch/bank/heading of the AI object once-per-second

Each time I get this data packet back for the AI object, I calculate where I want the AI aircraft to be four seconds later than the time (zulu time) returned in the data packet.

How you calculate this 'predict point' can vary, depending on your application - maybe it'll help if you explain what you're doing. E.g. you could have an AI object follow the user aircraft - the predict point would at its simplest be a lat/long/alt projection forwards from the user aircraft using its current position, heading, and speed. Or you could do something similar with the position of a user aircraft in multi-player. In *my* application I actually have a GPS-trace file showing the tracklog of a real aircraft, and it's easy for me to walk forwards through the file and pick out the "T+4 seconds" position of the aircraft. This 'predictive' algorithm is very similar to simply continously aiming at the user aircraft (e.g. if you wanted to follow that) but for various stability reasons it's actually a bit smoother if you actually aim to stay four seconds behind a point you're predicting the user aircraft will be in four seconds, if you can get you're head around that concept....

Given the *current* position and orientation of the AI aircraft (returned in the RequestDataOnSimObject data packet) and the *predict point* I want the AI aircraft to reach in 4 seconds, I can compute the SLEW_AXIS.. event values to send to the AI aircraft to have it arrive at that point at the right time. The AI aircraft doesn't necessarily ever arrive exactly at *that* point because I'll be sending more SLEW_AXIS events a second later aiming at a slightly updated predict point, so the AI aircraft behaves as expected, slewing around the sky in a fairly sensible way, with the 'predict point' always being dangled 4 seconds in front of it to aim at...

You can test the SLEW_AXIS.. events using Traffic Tool on the user aircraft (thanks Manuel). If you do this it helps if you update the shift-Z display values to get vertical climb rate and true heading etc.

Then you can hit kind of a milestone having a SLEW_AXIS_ALT_SET(1000) event cause the AI object to descend at a steady 63 feet-per-minute... and have the RequestDataOnSimObject pick up the AI object altitude once-per-second and display it in a console window or something. At this point you have just about everything you need to create a working navigating AI object that you can move around at will.

But as I mentioned in an earlier post, even though the principle is this simple (once you get it), there is still a lot of quality programming to do just to make sure the process actually works in practice. E.g. I had to calibrate the SLEW_AXIS.. values, and the 'turn-and-move-forward' algorithm of moving an object takes a while to get correct - it's certainly harder than the basic 'warping' of the AI object directly to the point in space that you want it. The good news is that each 'degree of freedom' is actually pretty independent, so you can get the code working for moving the aircraft in lat/long at a fixed altitude (this is the only tricky 'aiming' algorithm) and add in altitude changes afterwards. Pitch and bank don't alter the direction of motion of the aircraft so they're easy to program and are cosmetic really.

My particular requirement has location 'samples' that really can be quite a long way apart (eg 10 seconds), so it's necessary for me to smoothly interpolate between them - today I've been working out how to move the aircraft in a smooth 24-second turn when my lat/long/alt fixes might be at 10-second intervals (try it on a piece of paper....). So I'm dealing with some pretty detailed issues as the basic method is working very smoothly...

Another subtlety (as mentioned by Manuel earlier but I forgot about it) is that my source data (real GPS traces) sometimes have an altitude offset such that for an aircraft on the ground I am trying to tell FSX to move it *lower* to the height my GPS trace says (wrongly) it should be at. FSX works well in placing the AI aircraft on the ground (and not below it), and no lower, but my AI 'following' algorithm was going a bit nuts twisting the aircraft to try and reach the lower position. It was an easyish fix to detect SIM ON GROUND from the AI object and tweak the movement algorithm in that case (i.e. force pitch and bank to zero).

Also today I downloaded the tracklogs of 39 real aircraft from a gliding competition in the UK, and replayed them on my PC substituting 747's for the gliders, and had a cloud of 747's circling smoothly over Cambridge... ZERO framerate hit.

sim_logger is written as a single Visual C++ program of over 4000 lines, divided into the main sections:
Program startup and ini file handling.
FSX simconnect message handling, in particular recognising file loads of FLT, PLN, WX files and receiving the position updates from the user aircraft and the replaying AI objects
Writing the IGC file for the user aircraft. The checksum formula has been removed to keep it secure.
Reading the IGC files associated with the loaded flight, adding interpolated points if the sample rate is a bit low, and estimating values for pitch, bank and heading (these are not in the log).
The AI movement algorithm, updating the slew rates of the AI objects once-per-second.
The menu system using the FSX SimConnect_Text call

In particular the AI following algorithms, the IGC file 'interpolation' algorithm that injects extra trackpoints if the gap between samples is >5 seconds, and the estimation of pitch and bank from the lat/long/alt tracklog sample values are non-trivial.

Other significant parts of the simconnect module are more engineering than computer science, may help any developers of simconnect modules, e.g. finding and parsing the FLT file when the user loads one or simply subscribing to and processing position updates from the user aircraft or AI objects. Also processing an 'ini' file for the module, and using the 'SimConnect_Text' call to create a menu system.

