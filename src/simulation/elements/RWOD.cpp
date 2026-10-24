#include "simulation/ElementCommon.h"

void Element::Element_RWOD()
{
	Identifier = "RIGID_BODIES_PT_RWOD";
	Name = "RWOD";
	Colour = 0xA87C48_rgb;
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

	Flammable = 20;
	Explosive = 0;
	Meltable = 0;
	Hardness = 16;

	// Heavier than any liquid or powder, so nothing displaces its pixels.
	Weight = 100;

	HeatConduct = 164;
	Description = "Rigid wood. Light: floats on water and oil. Burns.";

	Properties = TYPE_SOLID|PROP_NEUTPENETRATE;

	LowPressure = IPL;
	LowPressureTransition = NT;
	HighPressure = IPH;
	HighPressureTransition = NT;
	LowTemperature = ITL;
	LowTemperatureTransition = NT;
	HighTemperature = 873.0f;
	HighTemperatureTransition = PT_FIRE;
}
