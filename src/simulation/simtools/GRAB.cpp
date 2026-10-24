#include "simulation/ToolCommon.h"
#include "simulation/RigidBodies.h"
#include "gui/game/Brush.h"

static void performDraw(SimTool *tool, Simulation *sim, const Brush &brush, ui::Point position);
static void performDrawLine(SimTool *tool, Simulation *sim, const Brush &brush, ui::Point position1, ui::Point position2, bool dragging);
static void performClick(SimTool *tool, Simulation *sim, const Brush &brush, ui::Point position);

void SimTool::Tool_GRAB()
{
	Identifier = "RIGID_BODIES_TOOL_GRAB";
	Name = "GRAB";
	Colour = 0x80C040_rgb;
	Description = "Grabs rigid bodies. Click and drag to move them around, let go to throw them.";
	PerformDraw = &performDraw;
	PerformDrawLine = &performDrawLine;
	PerformDrawRect = nullptr;
	PerformClick = &performClick;
}

// Pressing the mouse button takes hold of the body under the brush, nearest
// the cursor.
static void performDraw(SimTool *tool, Simulation *sim, const Brush &brush, ui::Point position)
{
	std::vector<std::pair<int, int>> cells;
	for (ui::Point off : brush)
	{
		cells.emplace_back(position.X + off.X, position.Y + off.Y);
	}
	sim->rigidBodies->Grab(*sim, cells, { float(position.X), float(position.Y) });
}

// While the button is held, the game calls this every frame with where the
// cursor is.
static void performDrawLine(SimTool *tool, Simulation *sim, const Brush &brush, ui::Point position1, ui::Point position2, bool dragging)
{
	if (dragging)
	{
		sim->rigidBodies->MoveGrab({ float(position2.X), float(position2.Y) });
	}
}

// Letting go of the button lets go of the body.
static void performClick(SimTool *tool, Simulation *sim, const Brush &brush, ui::Point position)
{
	sim->rigidBodies->Release();
}
