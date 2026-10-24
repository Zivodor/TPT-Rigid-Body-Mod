#include "simulation/ElementCommon.h"

void Element::Element_RRBR()
{
	Identifier = "RIGID_BODIES_PT_RRBR";
	Name = "RRBR";
	Colour = 0x383838_rgb;
	MenuVisible = 1;
	MenuSection = SC_SOLIDS;
	Enabled = 1;

	// Movement is handled by RigidBodies, not by the particle update. The
	// element must stay a gravity-less solid or each pixel would move on its
	// own. Its density and other physics are in RigidBodies' material table.
	Advection = 0.0f;
	AirDrag = 0.00f * CFDS;
	AirLoss = 0.90f;
	Loss = 0.00f;
	Collision = 0.0f;
	Gravity = 0.0f;
	Diffusion = 0.00f;
	HotAir = 0.000f	* CFDS;
	Falldown = 0;

	Flammable = 5;
	Explosive = 0;
	Meltable = 0;
	Hardness = 10;

	// Heavier than any liquid or powder, so nothing displaces its pixels.
	Weight = 100;

	HeatConduct = 20;
	Description = "Rigid rubber. Very bouncy and grippy. Slightly denser than water.";

	Properties = TYPE_SOLID;

	LowPressure = IPL;
	LowPressureTransition = NT;
	HighPressure = IPH;
	HighPressureTransition = NT;
	LowTemperature = ITL;
	LowTemperatureTransition = NT;
	HighTemperature = ITH;
	HighTemperatureTransition = NT;
}
