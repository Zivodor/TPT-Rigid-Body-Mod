#include "simulation/ToolCommon.h"
#include "simulation/RigidBodies.h"
#include "gui/game/Brush.h"

static void performClick(SimTool *tool, Simulation *sim, const Brush &brush, ui::Point position);
static void performDrawLine(SimTool *tool, Simulation *sim, const Brush &brush, ui::Point position1, ui::Point position2, bool dragging);

// How fast a push sends off a body as dense as water, in pixels per frame:
// a click, and a line for each pixel of its length. Denser bodies go slower
// (gold about a twentieth as fast), lighter ones faster, up to the speed
// limit.
constexpr float pokeSpeed = 6.0f;
constexpr float pushSpeedPerPixel = 0.15f;

void SimTool::Tool_PUSH()
{
	Identifier = "RIGID_BODIES_TOOL_PUSH";
	Name = "PUSH";
	Colour = 0xD08030_rgb;
	Description = "Pushes rigid bodies. Click to shove, or draw a line to push along it.";
	PerformDraw = nullptr;
	PerformDrawRect = nullptr;
	PerformClick = &performClick;
	PerformDrawLine = &performDrawLine;
}

static std::vector<std::pair<int, int>> cellsUnder(const Brush &brush, ui::Point position)
{
	std::vector<std::pair<int, int>> cells;
	for (ui::Point off : brush)
	{
		cells.emplace_back(position.X + off.X, position.Y + off.Y);
	}
	return cells;
}

static void performClick(SimTool *tool, Simulation *sim, const Brush &brush, ui::Point position)
{
	sim->rigidBodies->Poke(*sim, cellsUnder(brush, position), { float(position.X), float(position.Y) }, pokeSpeed * tool->Strength);
}

static void performDrawLine(SimTool *tool, Simulation *sim, const Brush &brush, ui::Point position1, ui::Point position2, bool dragging)
{
	// Dragging the brush around does nothing; only a line drawn with the
	// line tool pushes, from where it starts.
	if (dragging)
	{
		return;
	}
	auto line = position2 - position1;
	sim->rigidBodies->Push(*sim, cellsUnder(brush, position1), { float(position1.X), float(position1.Y) }, { pushSpeedPerPixel * line.X, pushSpeedPerPixel * line.Y });
}
