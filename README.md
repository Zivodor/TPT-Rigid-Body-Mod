Rigid Body Mod for The Powder Toy - v0.0.1
==========================================

**AI disclosure: the code in this mod (everything that isn't The Powder Toy itself or Box2D) was written with an AI assistant, Claude by Anthropic, under the direction of the mod's author.**

This is a mod of [The Powder Toy](https://powdertoy.co.uk) that adds rigid bodies: solids that move as one piece. Draw a box of RIGD and it falls, lands, tips over edges, slides down slopes, stacks and rolls, keeping its shape. Bodies float or sink in liquids according to their density.

It is not affiliated with or supported by the official Powder Toy team. Please report problems with the mod in its forum thread or here, not to the official developers.

Materials
---------

All are in the Solids menu. Densities are relative to water (1.0); liquids use roughly real densities too (oil 0.85, mercury 13.5, lava 3.1).

| Element | Material | Density | Behaviour |
| ------- | -------- | ------- | --------- |
| RIGD | Iron   | 7.8  | Sinks in water, floats on mercury. Conducts. |
| RWOD | Wood   | 0.6  | Floats on water and oil. Burns. |
| RRBR | Rubber | 1.1  | Very bouncy and grippy. Barely sinks in water. |
| RICE | Ice    | 0.92 | Almost frictionless. Floats in water, sinks in oil. Melts into water above 0C. |
| RSTN | Stone  | 2.6  | Rough. Sinks in water, floats on lava and mercury. Melts into lava. |
| RFOM | Foam   | 0.1  | Rides high on any liquid. Burns. |
| RGLD | Gold   | 19.3 | Sinks even in mercury. Conducts. |

How it works
------------

* Touching pixels of the same material form one body; different materials stay separate bodies. Pixels drawn onto a body join it, and cutting or melting a body splits it.
* Bodies are simulated with [Box2D](https://box2d.org) and drawn back onto the pixel grid every frame. Walls and other solids are obstacles; liquids and gases are pushed aside, with buoyancy and drag.
* A single pixel of rigid material isn't a body: it falls like powder. Molten rigid material is ordinary lava, and turns back into a rigid body when it sets.
* *Simulation Options > Rigid body liquid levelling* moves liquid under floating bodies so levels even out (TPT's liquids have no pressure of their own).

Known limitations
-----------------

* Powders are treated as solid ground: bodies rest on sand rather than sinking into it.
* Body velocities aren't saved; after loading a save, bodies start at rest (with their shapes intact).
* Rigid bodies packed tightly inside another body can lag slightly behind it while both are moving.

Building and testing
--------------------

Build as you would The Powder Toy (see the build instructions below); Box2D is included in `vendor/box2d`. The rigid body code is in `src/simulation/RigidBodies.cpp`. Headless tests:

```
meson setup build-release --buildtype=release
ninja -C build-release rigid-bodies-test
./build-release/rigid-bodies-test             # all scenarios
./build-release/rigid-bodies-test "floats"    # only scenarios whose names contain "floats"
```

The rest of this file is The Powder Toy's own README.

The Powder Toy - June 2026
==========================

Get the latest version [from the Powder Toy website](https://powdertoy.co.uk/Download.html). We're also on [Steam](https://store.steampowered.com/app/1148350/The_Powder_Toy/).

To use online features such as saving, you need to [register an account](https://powdertoy.co.uk/Register.html).
You can also visit [the official TPT forum](https://powdertoy.co.uk/Discussions/Categories/Index.html).

Have you ever wanted to blow something up? Or maybe you always dreamt of operating an atomic power plant? Do you have a will to develop your own CPU? The Powder Toy lets you to do all of these, and even more!

The Powder Toy is a free physics sandbox game, which simulates air pressure and velocity, heat, gravity and a countless number of interactions between different substances! The game provides you with various building materials, liquids, gases and electronic components which can be used to construct complex machines, guns, bombs, realistic terrains and almost anything else. You can then mine them and watch cool explosions, add intricate wirings, play with little stickmen or operate your machine. You can browse and play thousands of different saves made by the community or upload your own – we welcome your creations!

There is a Lua API – you can automate your work or even make plugins for the game. The Powder Toy is free and the source code is distributed under the GNU General Public License, so you can modify the game yourself or help with development.

Build instructions
===========================================================================

See the _Powder Toy Development Help_ section [on the main page of the wiki](https://powdertoy.co.uk/Wiki/W/Main_Page.html).

Special Thanks
===========================================================================

* Stanislaw K Skowronek - Designed the original Powder Toy
* Simon Robertshaw - Wrote the website, current server owner
* Skresanov Savely
* Pilihp64
* Catelite
* Victoria Hoyle
* Nathan Cousins
* jacksonmj
* Felix Wallin
* Lieuwe Mosch
* Anthony Boot
* Me4502
* MaksProg
* jacob1
* mniip
* LBPHacker

Libraries and other assets used
===========================================================================

* [bzip2](http://www.bzip.org/)
* [FFTW](http://fftw.org/)
* [JsonCpp](https://github.com/open-source-parsers/jsoncpp)
* [libcurl](https://curl.se/libcurl/)
* [libpng](http://www.libpng.org/pub/png/libpng.html)
* [Lua](https://www.lua.org/)
* [LuaJIT](https://luajit.org/)
* [Mallangche](https://github.com/JammPark/Mallangche)
* [mbedtls](https://www.trustedfirmware.org/projects/mbed-tls/)
* [SDL](https://libsdl.org/)
* [Box2D](https://box2d.org) (Rigid Body Mod only; MIT license, Copyright (c) 2022 Erin Catto, see [vendor/box2d/LICENSE](vendor/box2d/LICENSE))

Instructions
===========================================================================

Click on the elements with the mouse and draw in the field, like in MS Paint. The rest of the game is learning what happens next.

Controls
===========================================================================

| Key                     | Action                                                          |
| ----------------------- | --------------------------------------------------------------- |
| TAB                     | Switch between circle/square/triangle brush                     |
| Space                   | Pause                                                           |
| Q / Esc                 | Quit                                                            |
| Z                       | Zoom                                                            |
| S                       | Save stamp (use with Ctrl when STK2 is out)                     |
| L                       | Load last saved stamp                                           |
| K                       | Stamp library                                                   |
| 0-9                     | Set view mode                                                   |
| P / F2                  | Save screenshot as .png                                         |
| E                       | Bring up element search                                         |
| F                       | Pause and step to next frame                                    |
| G                       | Increase grid size                                              |
| Shift + G               | Decrease grid size                                              |
| H                       | Show/Hide HUD                                                   |
| Ctrl + H / F1           | Show intro text                                                 |
| D / F3                  | Debug mode (use with Ctrl when STK2 is out)                     |
| I                       | Invert Pressure and Velocity map                                |
| W                       | Cycle gravity modes (use with Ctrl when STK2 is out)            |
| Y                       | Cycle air modes                                                 |
| Ctrl + E                | Cycle edge modes                                                |
| B                       | Enter decoration editor menu                                    |
| Ctrl + B                | Toggle decorations on/off                                       |
| N                       | Toggle Newtonian Gravity on/off                                 |
| U                       | Toggle ambient heat on/off                                      |
| Ctrl + I                | Install powder toy, for loading saves/stamps by double clicking |
| Backtick                | Toggle console                                                  |
| =                       | Reset pressure and velocity map                                 |
| Ctrl + =                | Reset Electricity                                               |
| \[                      | Decrease brush size                                             |
| \]                      | Increase brush size                                             |
| Alt + \[                | Decrease brush size by 1                                        |
| Alt + \]                | Increase brush size by 1                                        |
| Ctrl + C/V/X            | Copy/Paste/Cut                                                  |
| Ctrl + Z                | Undo                                                            |
| Ctrl + Y                | Redo                                                            |
| Ctrl + Cursor drag      | Rectangle                                                       |
| Shift + Cursor drag     | Line                                                            |
| Middle click            | Sample element                                                  |
| Alt + Left click        | Sample element                                                  |
| Mouse scroll            | Change brush size                                               |
| Ctrl + Mouse scroll     | Change vertical brush size                                      |
| Shift + Mouse scroll    | Change horizontal brush size                                    |
| Shift + R               | Horizontal mirror for selected area when pasting stamps         |
| Ctrl + Shift + R        | Vertical mirror for selected area when pasting stamps           |
| R                       | Rotate selected area counterclockwise when pasting stamps       |
| F11                     | Toggle fullscreen                                               |

Command Line
---------------------------------------------------------------------------

| Command               | Description                                      | Example                                     |
| --------------------- | ------------------------------------------------ | --------------------------------------------|
| `scale:SIZE`          | Change window scale factor                       | `scale:2`                                   |
| `kiosk`               | Fullscreen mode                                  |                                             |
| `proxy:SERVER[:PORT]` | Proxy server to use                              | `proxy:wwwcache.lancs.ac.uk:8080`           |
| `open FILE`           | Opens the file as a stamp or game save           |                                             |
| `ddir DIRECTORY`      | Directory used for saving stamps and preferences |                                             |
| `ptsave:SAVEID`       | Open online save, used by ptsave: URLs           | `ptsave:2198`                               |
| `disable-network`     | Disables internet connections                    |                                             |
| `disable-bluescreen`  | Disable bluescreen handler                       |                                             |
| `redirect`            | Redirects output to stdout.txt / stderr.txt      |                                             |
| `console`             | Redirects output to a new console on Windows     |                                             |
| `cafile:CAFILE`       | Set certificate bundle path                      | `cafile:/etc/ssl/certs/ca-certificates.crt` |
| `capath:CAPATH`       | Set certificate directory path                   | `capath:/etc/ssl/certs`                     |
