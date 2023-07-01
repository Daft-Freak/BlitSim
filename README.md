# BlitSim
(Couldn't come up with a name)

Demo: https://cupboard.daftgames.net/NotA32BlitEmulator/

## What does it do?
Runs unmodified .blit files by translating 32blit API calls to the 32blit-sdl equivalent (in most cases). Game code is run in an interpreter (copied from [DERP](https://github.com/Daft-Freak/DERP) and extended).

## Why?
Distraction from one of my 32blit-replacement projects. (The idea of making something *100% compatible*)

## Is it accurate?
Nope. No attempt is made at accurate timing in the interpreter, it just runs as fast as it can. Also the lack of the firmware sometimes allows games to get away with using more memory than they should.

There are also some bugs and missing features in the CPU interpreter. Tell me if you find an obvious one!

## Is it fast?
Because of the lack of actually emulating the hardware, it's faster than DERP at least.

## Can it be used for debugging?
Maybe if I ported the GDB server and fault handling from DERP.

## Does it run DOOM?
Yes.

## Can I have one?
Unlike the real hardware, yes... Assuming it builds, I've only tested Linux.

## How do I build it?

Standard [32blit-sdk build things](https://github.com/32blit/32blit-sdk/tree/master/docs#readme)...
```
cmake -B build -G Ninja -D32BLIT_DIR=/path/to/32blit-sdk
ninja -C build
```
(Okay, that wasn't a standard example)

## How do I use it?

`./BlitSim game.blit`, or put a copy of `launcher.blit` and some other .blits next to the executable and run it for the full 32blit experience. (Except the system menu)


## Need more speed?

Rebuild with `-DENABLE_SCREEN_SPEED_HACKS=1` to patch more functions to run outside the interpreter.