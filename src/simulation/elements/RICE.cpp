#include "simulation/ElementCommon.h"

void Element::Element_RICE()
{
	Identifier = "RIGID_BODIES_PT_RICE";
	Name = "RICE";
	Colour = 0xA0D8FF_rgb;
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
	Hardness = 20;

	// Heavier than any liquid or powder, so nothing displaces its pixels.
	Weight = 100;
	DefaultProperties.temp = R_TEMP - 40.0f + 273.15f;

	HeatConduct = 46;
	Description = "Rigid ice. Slippery. Floats on water, sinks in oil. Melts above 0C.";

	Properties = TYPE_SOLID|PROP_NEUTPASS;

	LowPressure = IPL;
	LowPressureTransition = NT;
	HighPressure = IPH;
	HighPressureTransition = NT;
	LowTemperature = ITL;
	LowTemperatureTransition = NT;
	HighTemperature = 273.15f;
	HighTemperatureTransition = PT_WATR;
}
