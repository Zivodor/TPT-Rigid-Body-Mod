#include "simulation/ElementCommon.h"

void Element::Element_ELAS()
{
	Identifier = "RIGID_BODIES_PT_ELAS";
	Name = "ELAS";
	Colour = 0xD040A0_rgb;
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

	Flammable = 0;
	Explosive = 0;
	Meltable = 0;
	Hardness = 10;

	// Heavier than any liquid or powder, so nothing displaces its pixels.
	Weight = 100;

	HeatConduct = 20;
	Description = "Rigid elastic. Bounces back perfectly, losing nothing when it hits things. Grippy like rubber.";

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
