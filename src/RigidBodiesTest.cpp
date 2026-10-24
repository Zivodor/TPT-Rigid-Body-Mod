// Headless scenarios for RIGD rigid bodies. Run with:
//   meson test -C <builddir> rigid-bodies
// or run the rigid-bodies-test executable directly for timings and details.
#include "simulation/Simulation.h"
#include "simulation/SimulationData.h"
#include "simulation/ElementClasses.h"
#include "simulation/RigidBodies.h"
#include "client/GameSave.h"
#include "common/platform/Platform.h"
#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <cstdio>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace
{
	bool IsRigid(int type)
	{
		return type == PT_RIGD || type == PT_RWOD || type == PT_RRBR || type == PT_RICE ||
			type == PT_RSTN || type == PT_RFOM || type == PT_RGLD;
	}

	struct Bounds
	{
		int x0 = XRES, y0 = YRES, x1 = -1, y1 = -1, count = 0;
		int Width() const { return x1 - x0 + 1; }
		int Height() const { return y1 - y0 + 1; }
	};

	struct Scene
	{
		std::unique_ptr<Simulation> sim = Simulation::Factory();
		bool check = true; // verify the grid every frame (off for timing)

		Scene()
		{
			sim->SetEdgeMode(EDGE_SOLID);
		}

		// Inert, indestructible ground.
		void Solid(int x0, int y0, int x1, int y1)
		{
			for (int y = y0; y <= y1; y++)
			{
				for (int x = x0; x <= x1; x++)
				{
					sim->create_part(-1, x, y, PT_DMND);
				}
			}
		}

		void Rigid(int x0, int y0, int w, int h, int type = PT_RIGD)
		{
			for (int y = y0; y < y0 + h; y++)
			{
				for (int x = x0; x < x0 + w; x++)
				{
					sim->create_part(-1, x, y, type);
				}
			}
		}

		// A diamond-walled tank from x0 to x1 with its floor's top at
		// `floor`, filled to `depth` with a liquid, left to settle.
		void Tank(int x0, int x1, int floor, int depth, int liquid)
		{
			Solid(x0 - 3, floor, x1 + 3, floor + 3);
			Solid(x0 - 3, floor - depth - 40, x0 - 1, floor - 1);
			Solid(x1 + 1, floor - depth - 40, x1 + 3, floor - 1);
			for (int y = floor - depth; y < floor; y++)
			{
				for (int x = x0; x <= x1; x++)
				{
					sim->create_part(-1, x, y, liquid);
				}
			}
			Run(60);
		}

		int CountOf(int type) const
		{
			int n = 0;
			for (int i = 0; i < sim->parts.active; i++)
			{
				n += sim->parts[i].type == type;
			}
			return n;
		}

		// Top of the liquid just either side of a body (the median of a few
		// columns), which is what it floats in.
		int SurfaceBeside(const Bounds &b, int liquid) const
		{
			std::vector<int> tops;
			for (int x : { b.x0 - 6, b.x0 - 4, b.x0 - 2, b.x1 + 2, b.x1 + 4, b.x1 + 6 })
			{
				for (int y = 0; y < YRES; y++)
				{
					auto r = sim->pmap[y][x];
					if (r && TYP(r) == liquid)
					{
						tops.push_back(y);
						break;
					}
				}
			}
			std::sort(tops.begin(), tops.end());
			return tops.empty() ? YRES : tops[tops.size() / 2];
		}

		// Top of the liquid in the tank, by the walls where no body is.
		int Surface(int x0, int x1, int liquid) const
		{
			int top = YRES;
			for (int x : { x0, x0 + 1, x0 + 2, x1 - 2, x1 - 1, x1 })
			{
				for (int y = 0; y < YRES; y++)
				{
					auto r = sim->pmap[y][x];
					if (r && TYP(r) == liquid)
					{
						top = std::min(top, y);
						break;
					}
				}
			}
			return top;
		}

		void Circle(int cx, int cy, int r)
		{
			for (int y = cy - r; y <= cy + r; y++)
			{
				for (int x = cx - r; x <= cx + r; x++)
				{
					if ((x - cx) * (x - cx) + (y - cy) * (y - cy) <= r * r && !sim->pmap[y][x])
					{
						sim->create_part(-1, x, y, PT_RIGD);
					}
				}
			}
		}

		int Count() const
		{
			int n = 0;
			for (int i = 0; i < sim->parts.active; i++)
			{
				n += IsRigid(sim->parts[i].type);
			}
			return n;
		}

		// Every rigid particle must be in the particle map exactly where it is.
		void CheckGrid() const
		{
			for (int i = 0; i < sim->parts.active; i++)
			{
				auto &part = sim->parts[i];
				if (!IsRigid(part.type))
				{
					continue;
				}
				auto x = int(part.x + 0.5f), y = int(part.y + 0.5f);
				if (part.x != float(x) || part.y != float(y))
				{
					throw std::string("particle off grid");
				}
				if (ID(sim->pmap[y][x]) != i)
				{
					throw std::string("particle missing from pmap (overlap) at ") + std::to_string(x) + "," + std::to_string(y);
				}
			}
			// And no cell may claim a particle that isn't actually there.
			for (int y = 0; y < YRES; y++)
			{
				for (int x = 0; x < XRES; x++)
				{
					auto r = sim->pmap[y][x];
					if (!r)
					{
						continue;
					}
					auto &part = sim->parts[ID(r)];
					if (part.type != TYP(r) || int(part.x + 0.5f) != x || int(part.y + 0.5f) != y)
					{
						throw std::string("stale pmap entry at ") + std::to_string(x) + "," + std::to_string(y);
					}
				}
			}
		}

		void Run(int frames)
		{
			for (int f = 0; f < frames; f++)
			{
				// The rigid bodies run at the end of BeforeSim, so that's when
				// their pixels must be in order; the particle update after it
				// can turn other things (cooling molten gold, say) into rigid
				// material anywhere.
				sim->BeforeSim(true);
				if (check)
				{
					CheckGrid();
				}
				sim->UpdateParticles(0, NPART);
				sim->AfterSim();
			}
		}

		// Save the scene as it is now, if RIGID_BODIES_SAVES is set, to look
		// at with the render tool.
		void Snapshot(const std::string &label) const
		{
			if (auto dir = std::getenv("RIGID_BODIES_SAVES"))
			{
				auto data = sim->Save(true, RES.OriginRect())->Serialise().second;
				Platform::WriteFile(data, ByteString(dir) + "/" + ByteString(label) + ".cps");
			}
		}

		// Bounds of the rigid particles in the given rectangle (all, by default).
		Bounds Of(int x0 = 0, int y0 = 0, int x1 = XRES, int y1 = YRES) const
		{
			Bounds b;
			for (int i = 0; i < sim->parts.active; i++)
			{
				auto &part = sim->parts[i];
				auto x = int(part.x + 0.5f), y = int(part.y + 0.5f);
				if (!IsRigid(part.type) || x < x0 || x > x1 || y < y0 || y > y1)
				{
					continue;
				}
				b.x0 = std::min(b.x0, x);
				b.y0 = std::min(b.y0, y);
				b.x1 = std::max(b.x1, x);
				b.y1 = std::max(b.y1, y);
				b.count++;
			}
			return b;
		}

		// Bounds of each body, by body ID.
		std::map<int, Bounds> Bodies() const
		{
			std::map<int, Bounds> bodies;
			for (int i = 0; i < sim->parts.active; i++)
			{
				auto &part = sim->parts[i];
				if (!IsRigid(part.type))
				{
					continue;
				}
				auto &b = bodies[part.tmp2];
				auto x = int(part.x + 0.5f), y = int(part.y + 0.5f);
				b.x0 = std::min(b.x0, x);
				b.y0 = std::min(b.y0, y);
				b.x1 = std::max(b.x1, x);
				b.y1 = std::max(b.y1, y);
				b.count++;
			}
			return bodies;
		}

		// Small empty pockets (up to 8 cells) closed in by particles, with
		// at least half of their border rigid: holes in a body, or gaps
		// between a body and what it rests on.
		int Holes() const
		{
			int holes = 0;
			// 0: not looked at, 1: part of a big or open empty area, 2: a pocket
			std::vector<char> known(XRES * YRES, 0);
			std::vector<int> stamp(XRES * YRES, 0);
			int search = 0;
			for (int y = 1; y < YRES - 1; y++)
			{
				for (int x = 1; x < XRES - 1; x++)
				{
					if (sim->pmap[y][x] || known[y * XRES + x])
					{
						continue;
					}
					search++;
					std::vector<std::pair<int, int>> pocket{ { x, y } };
					stamp[y * XRES + x] = search;
					int border = 0, rigid = 0;
					bool open = false;
					for (size_t i = 0; i < pocket.size() && !open; i++)
					{
						for (auto [dx, dy] : { std::pair(0, -1), { -1, 0 }, { 1, 0 }, { 0, 1 } })
						{
							auto nx = pocket[i].first + dx, ny = pocket[i].second + dy;
							auto k = ny * XRES + nx;
							if (nx <= 0 || nx >= XRES - 1 || ny <= 0 || ny >= YRES - 1 || known[k] == 1)
							{
								open = true;
								break;
							}
							auto r = sim->pmap[ny][nx];
							if (r)
							{
								border++;
								rigid += IsRigid(TYP(r));
							}
							else if (stamp[k] != search)
							{
								stamp[k] = search;
								pocket.push_back({ nx, ny });
								if (pocket.size() > 8)
								{
									open = true;
									break;
								}
							}
						}
					}
					for (auto [px, py] : pocket)
					{
						known[py * XRES + px] = open ? 1 : 2;
					}
					if (!open && rigid * 2 >= border)
					{
						holes += int(pocket.size());
					}
				}
			}
			return holes;
		}

		std::vector<std::pair<int, int>> Positions() const
		{
			std::vector<std::pair<int, int>> positions;
			for (int i = 0; i < sim->parts.active; i++)
			{
				if (IsRigid(sim->parts[i].type))
				{
					positions.emplace_back(int(sim->parts[i].x), int(sim->parts[i].y));
				}
			}
			return positions;
		}
	};

	void Expect(bool condition, const std::string &message)
	{
		if (!condition)
		{
			throw message;
		}
	}

	std::string Describe(const Bounds &b)
	{
		char buf[100];
		snprintf(buf, sizeof(buf), "x %d..%d y %d..%d (%d px)", b.x0, b.x1, b.y0, b.y1, b.count);
		return buf;
	}

	constexpr int ground = 340; // top row of the floor in most scenes

	struct Scenario
	{
		const char *name;
		std::function<void(Scene &)> run;
	};

	const std::vector<Scenario> scenarios = {
		{ "flat box lands and stays level", [](Scene &s) {
			s.Solid(20, ground, 590, ground + 5);
			s.Rigid(100, 200, 30, 12);
			s.Run(400);
			auto b = s.Of();
			Expect(b.y1 == ground - 1, "not resting on the floor: " + Describe(b));
			Expect(b.Width() == 30 && b.Height() == 12, "box deformed or tilted: " + Describe(b));
		} },
		{ "tall tower stays upright", [](Scene &s) {
			s.Solid(20, ground, 590, ground + 5);
			s.Rigid(200, ground - 45, 8, 40);
			s.Run(400);
			auto b = s.Of();
			Expect(b.y1 == ground - 1 && b.Width() == 8 && b.Height() == 40, "tower fell or moved: " + Describe(b));
		} },
		{ "plank overhanging a ledge tips off", [](Scene &s) {
			s.Solid(20, 200, 150, 205);
			s.Solid(20, ground, 590, ground + 5);
			s.Rigid(135, 193, 50, 6); // only 16 of 50 px on the ledge
			s.Run(400);
			auto b = s.Of();
			Expect(b.y0 > 250, "plank still up: " + Describe(b));
			Expect(b.count == 300, "plank lost pixels: " + Describe(b));
		} },
		{ "plank mostly on a ledge stays", [](Scene &s) {
			s.Solid(20, 200, 150, 205);
			s.Solid(20, ground, 590, ground + 5);
			s.Rigid(110, 193, 50, 6); // 41 of 50 px on the ledge
			s.Run(400);
			auto b = s.Of();
			Expect(b.y1 == 199 && b.Height() == 6, "plank moved: " + Describe(b));
		} },
		{ "top-heavy upside-down L topples", [](Scene &s) {
			s.Solid(20, ground, 590, ground + 5);
			s.Rigid(150, ground - 40, 6, 40);
			s.Rigid(156, ground - 40, 30, 6);
			s.Run(500);
			auto b = s.Of();
			Expect(b.Height() < 39, "still standing: " + Describe(b));
		} },
		{ "three stacked boxes stay stacked", [](Scene &s) {
			s.Solid(20, ground, 590, ground + 5);
			s.Rigid(200, ground - 20, 40, 20);
			s.Rigid(206, ground - 41, 30, 20);
			s.Rigid(210, ground - 62, 20, 20);
			s.Run(600);
			auto bodies = s.Bodies();
			Expect(bodies.size() == 3, "expected 3 bodies, got " + std::to_string(bodies.size()));
			auto all = s.Of();
			Expect(all.y0 >= ground - 64 && all.x0 >= 198 && all.x1 <= 242, "stack collapsed: " + Describe(all));
		} },
		{ "box on box nudged sideways settles, doesn't slide off", [](Scene &s) {
			s.Solid(20, ground, 590, ground + 5);
			s.Rigid(200, ground - 20, 60, 20);
			s.Rigid(215, ground - 35, 20, 12);
			s.Run(60);
			// Shove the top box with a small sideways velocity by drawing it moving: give the
			// whole scene a brief custom sideways gravity instead.
			s.sim->gravityMode = GRAV_CUSTOM;
			s.sim->customGravityX = 0.15f;
			s.sim->customGravityY = 1.0f;
			s.Run(20);
			s.sim->gravityMode = GRAV_VERTICAL;
			s.Run(300);
			auto top = s.Of(0, 0, XRES, ground - 21);
			Expect(top.count == 240 && top.y1 == ground - 21, "top box fell off: " + Describe(top));
		} },
		{ "box slides down a 45 degree slope", [](Scene &s) {
			for (int x = 100; x <= 250; x++)
			{
				s.Solid(x, 300 - (250 - x), x, 300);
			}
			s.Solid(20, 301, 590, 306);
			s.Rigid(110, 120, 12, 12);
			s.Run(500);
			auto b = s.Of();
			Expect(b.x0 > 230, "box didn't travel down the slope: " + Describe(b));
		} },
		{ "pile of 25 boxes settles cleanly", [](Scene &s) {
			s.Solid(20, ground, 590, ground + 5);
			s.Solid(20, 200, 25, ground);
			s.Solid(585, 200, 590, ground);
			unsigned seed = 7;
			auto rnd = [&seed](int lo, int hi) {
				seed = seed * 1103515245u + 12345u;
				return lo + int((seed >> 16) % unsigned(hi - lo + 1));
			};
			for (int n = 0; n < 25; n++)
			{
				int w = rnd(4, 25), h = rnd(4, 25);
				int x = rnd(30, 575 - w), y = 10 + (n % 5) * 60 + rnd(0, 20);
				bool free = true;
				for (int yy = y - 1; yy <= y + h && free; yy++)
				{
					for (int xx = x - 1; xx <= x + w && free; xx++)
					{
						free = !s.sim->pmap[yy][xx];
					}
				}
				if (free)
				{
					s.Rigid(x, y, w, h);
				}
			}
			auto count = s.Count();
			s.Run(900);
			Expect(s.Count() == count, "pixels lost");
			auto before = s.Positions();
			s.Run(60);
			auto after = s.Positions();
			int moved = 0;
			for (size_t i = 0; i < before.size(); i++)
			{
				moved += before[i] != after[i];
			}
			Expect(moved < count / 50, std::to_string(moved) + " of " + std::to_string(count) + " px still moving");
			auto pulled = s.sim->rigidBodies->timings.pulled;
			Expect(pulled < 50, "bodies pulled back " + std::to_string(pulled) + " times; their pixels kept getting stuck");
		} },
		{ "circle dropped on a circle rolls off", [](Scene &s) {
			s.Solid(20, ground, 590, ground + 5);
			s.Circle(200, ground - 13, 12);
			s.Circle(202, ground - 60, 12);
			s.Run(600);
			auto bodies = s.Bodies();
			Expect(bodies.size() == 2, "expected 2 bodies, got " + std::to_string(bodies.size()));
			for (auto &[id, b] : bodies)
			{
				Expect(b.y1 == ground - 1, "a circle isn't on the floor: " + Describe(b));
			}
		} },
		{ "diamond dropped on its point falls onto a side", [](Scene &s) {
			s.Solid(20, ground, 590, ground + 5);
			// A square turned 45 degrees, very slightly lopsided.
			for (int y = -15; y <= 15; y++)
			{
				for (int x = -15; x <= 16; x++)
				{
					if (std::abs(x) + std::abs(y) <= 15 || (x == 16 && y == 0))
					{
						s.sim->create_part(-1, 200 + x, 250 + y, PT_RIGD);
					}
				}
			}
			s.Run(600);
			auto b = s.Of();
			Expect(b.Height() < 26, "still balanced on its point: " + Describe(b));
		} },
		{ "circles and triangles tumble into a heap without sticking", [](Scene &s) {
			s.Solid(20, ground, 590, ground + 5);
			s.Solid(20, 250, 25, ground);
			s.Solid(250, 250, 255, ground);
			for (int n = 0; n < 6; n++)
			{
				s.Circle(60 + n * 30, 100 + (n % 2) * 30, 9);
				for (int y = 0; y < 16; y++)
				{
					for (int x = -y; x <= y; x++)
					{
						s.sim->create_part(-1, 75 + n * 30 + x, 170 + (n % 3) * 25 + y, PT_RIGD);
					}
				}
			}
			s.Run(900);
			auto before = s.Positions();
			s.Run(60);
			auto after = s.Positions();
			int moved = 0;
			for (size_t i = 0; i < before.size(); i++)
			{
				moved += before[i] != after[i];
			}
			Expect(moved < int(before.size()) / 50, std::to_string(moved) + " px still moving");
			auto pulled = s.sim->rigidBodies->timings.pulled;
			Expect(pulled < 20, "bodies pulled back " + std::to_string(pulled) + " times; their pixels kept getting stuck");
		} },
		{ "mixed heap of circles, triangles, diamonds and boxes, void edges", [](Scene &s) {
			s.sim->SetEdgeMode(EDGE_VOID);
			s.Solid(20, 340, 589, 345);
			s.Solid(20, 200, 25, 339);
			s.Solid(584, 200, 589, 339);
			auto put = [&](int x, int y) {
				s.sim->create_part(-1, x, y, PT_RIGD);
			};
			for (int n = 0; n < 8; n++)
			{
				s.Circle(60 + n * 65, 60 + (n % 2) * 25, 10 + n % 3 * 3);
				for (int y = 0, size = 12 + n % 2 * 5; y <= size; y++)
				{
					for (int x = -y; x <= y; x++)
					{
						put(90 + n * 65 + x, 120 + (n % 3) * 20 + y);
					}
				}
				for (int y = -11; y <= 11; y++)
				{
					for (int x = -11; x <= 11; x++)
					{
						if (std::abs(x) + std::abs(y) <= 11)
						{
							put(70 + n * 65 + x, 200 + (n % 2) * 20 + y);
						}
					}
				}
				s.Rigid(50 + n * 65, 250, 14 + n % 3 * 6, 10);
			}
			auto count = s.Count();
			s.Run(900);
			Expect(s.Count() == count, "pixels lost");
		} },
		{ "U-shaped body lands on pillars close together", [](Scene &s) {
			// Rows with more than one run of cells, in a body and in the terrain.
			s.Solid(20, ground, 590, ground + 5);
			s.Solid(200, ground - 30, 203, ground - 1);
			s.Solid(208, ground - 30, 211, ground - 1);
			s.Rigid(190, 200, 6, 30);
			s.Rigid(216, 200, 6, 30);
			s.Rigid(190, 230, 32, 6);
			s.Run(400);
			auto b = s.Of();
			Expect(b.count == 6 * 30 * 2 + 32 * 6, "pixels lost: " + Describe(b));
			Expect(b.y1 <= ground - 1, "fell through: " + Describe(b));
		} },
		{ "big triangle on the floor has no holes", [](Scene &s) {
			s.Solid(20, ground, 590, ground + 5);
			for (int y = 0; y < 70; y++)
			{
				for (int x = -y; x <= y; x++)
				{
					s.sim->create_part(-1, 200 + x, 200 + y, PT_RIGD);
				}
			}
			s.Run(400);
			Expect(s.Holes() == 0, std::to_string(s.Holes()) + " holes");
		} },
		{ "big circles on the floor have no holes", [](Scene &s) {
			s.Solid(20, ground, 590, ground + 5);
			s.Circle(150, 200, 40);
			s.Circle(400, 150, 45);
			s.Run(500);
			Expect(s.Holes() == 0, std::to_string(s.Holes()) + " holes");
		} },
		{ "big triangle and circles on a wall floor have no holes", [](Scene &s) {
			s.sim->SetEdgeMode(EDGE_VOID);
			s.sim->CreateWallBox(40, 340, 580, 343, WL_WALL);
			for (int y = 0; y < 70; y++)
			{
				for (int x = -y; x <= y; x++)
				{
					s.sim->create_part(-1, 120 + x, 200 + y, PT_RIGD);
				}
			}
			s.Circle(300, 200, 40);
			s.Circle(480, 150, 45);
			s.Run(500);
			Expect(s.Holes() == 0, std::to_string(s.Holes()) + " holes");
		} },
		{ "wood floats in water, about 60% under", [](Scene &s) {
			s.Tank(150, 450, 330, 60, PT_WATR);
			s.Rigid(285, 150, 30, 14, PT_RWOD);
			s.Run(700);
			auto b = s.Of();
			auto surface = s.Surface(150, 450, PT_WATR);
			// Measured against the water right beside the wood: the water it
			// pushes aside mounds up around it a little, as it would.
			auto local = s.SurfaceBeside(b, PT_WATR);
			auto under = float(b.y1 + 1 - local) / b.Height();
			Expect(b.y1 < 320, "wood is on the bottom: " + Describe(b));
			Expect(under > 0.45f && under < 0.75f, "wood is " + std::to_string(int(under * 100)) + "% under (surface beside it at " + std::to_string(local) + ", by the walls at " + std::to_string(surface) + "): " + Describe(b));
		} },
		{ "iron and stone sink in water, water is conserved", [](Scene &s) {
			s.Tank(150, 450, 330, 60, PT_WATR);
			auto water = s.CountOf(PT_WATR);
			s.Rigid(200, 150, 30, 14, PT_RIGD);
			s.Rigid(350, 150, 30, 14, PT_RSTN);
			s.Run(700);
			for (auto &[id, b] : s.Bodies())
			{
				Expect(b.y1 == 329, "didn't sink to the bottom: " + Describe(b));
			}
			Expect(s.CountOf(PT_WATR) == water, "water went from " + std::to_string(water) + " to " + std::to_string(s.CountOf(PT_WATR)) + " (" + std::to_string(s.sim->rigidBodies->timings.fluidLost) + " had nowhere to go)");
		} },
		{ "iron and stone float on mercury, gold sinks", [](Scene &s) {
			s.Tank(150, 450, 330, 50, PT_MERC);
			auto mercury = s.CountOf(PT_MERC);
			s.Rigid(180, 150, 30, 14, PT_RIGD);
			s.Rigid(285, 150, 30, 14, PT_RSTN);
			s.Rigid(390, 150, 30, 14, PT_RGLD);
			s.Run(700);
			auto surface = s.Surface(150, 450, PT_MERC);
			auto iron = s.Of(170, 0, 230, YRES), stone = s.Of(265, 0, 335, YRES), gold = s.Of(370, 0, 440, YRES);
			Expect(iron.y1 < 320 && iron.y0 < surface, "iron isn't floating: " + Describe(iron) + " surface " + std::to_string(surface));
			Expect(stone.y1 < 320 && stone.y0 < surface, "stone isn't floating: " + Describe(stone));
			// Stone (2.6) floats much higher than iron (7.8) on mercury (13.5).
			Expect(stone.y1 < iron.y1, "stone should float higher than iron: stone " + Describe(stone) + ", iron " + Describe(iron) + ", surface " + std::to_string(surface));
			Expect(gold.y1 == 329, "gold didn't sink: " + Describe(gold));
			// Mercury grows and shrinks with temperature by itself, so count
			// only what the bodies lost.
			(void)mercury;
			Expect(s.sim->rigidBodies->timings.fluidLost == 0, std::to_string(s.sim->rigidBodies->timings.fluidLost) + " mercury particles had nowhere to go");
		} },
		{ "ice floats in water but sinks in oil; foam floats high on oil", [](Scene &s) {
			s.sim->legacy_enable = 1; // no heat, so the ice doesn't melt or freeze the water
			s.Tank(40, 280, 330, 60, PT_WATR);
			s.Tank(330, 570, 330, 60, PT_OIL);
			s.Rigid(145, 150, 30, 20, PT_RICE);
			s.Rigid(370, 150, 30, 20, PT_RICE);
			s.Rigid(500, 150, 30, 20, PT_RFOM);
			s.Run(800);
			auto iceInWater = s.Of(40, 0, 280, YRES), iceInOil = s.Of(330, 0, 450, YRES), foam = s.Of(460, 0, 570, YRES);
			auto water = s.Surface(40, 280, PT_WATR), oil = s.Surface(330, 570, PT_OIL);
			auto under = float(iceInWater.y1 + 1 - water) / iceInWater.Height();
			Expect(iceInWater.y1 < 320 && under > 0.75f, "ice in water should float low: " + std::to_string(int(under * 100)) + "% under, " + Describe(iceInWater));
			Expect(iceInOil.y1 == 329, "ice should sink in oil: " + Describe(iceInOil));
			auto foamUnder = float(foam.y1 + 1 - oil) / foam.Height();
			Expect(foamUnder < 0.3f, "foam should float high: " + std::to_string(int(foamUnder * 100)) + "% under, " + Describe(foam));
		} },
		{ "rubber bounces higher than stone", [](Scene &s) {
			s.Solid(20, ground, 590, ground + 5);
			s.Rigid(150, 150, 16, 16, PT_RRBR);
			s.Rigid(350, 150, 16, 16, PT_RSTN);
			int landed[2] = { 0, 0 }, peak[2] = { YRES, YRES };
			for (int f = 0; f < 200; f++)
			{
				s.Run(1);
				Bounds b[2] = { s.Of(100, 0, 250, YRES), s.Of(300, 0, 450, YRES) };
				for (int i = 0; i < 2; i++)
				{
					if (b[i].y1 == ground - 1)
					{
						landed[i] = 1;
					}
					else if (landed[i])
					{
						peak[i] = std::min(peak[i], b[i].y1);
					}
				}
			}
			Expect(ground - 1 - peak[0] > 15, "rubber barely bounced: " + std::to_string(ground - 1 - peak[0]) + " px");
			Expect(ground - 1 - peak[0] > 3 * std::max(0, ground - 1 - peak[1]), "rubber bounced " + std::to_string(ground - 1 - peak[0]) + " px, stone " + std::to_string(ground - 1 - peak[1]));
		} },
		{ "pushed sideways, ice slides far and rubber grips", [](Scene &s) {
			s.sim->legacy_enable = 1;
			s.Solid(20, ground, 590, ground + 5);
			s.Rigid(100, ground - 12, 12, 12, PT_RICE);
			s.Rigid(300, ground - 12, 12, 12, PT_RRBR);
			s.Run(30);
			auto ice0 = s.Of(0, 0, 200, YRES), rubber0 = s.Of(250, 0, 400, YRES);
			// A sideways push: tilt gravity for a moment.
			s.sim->gravityMode = GRAV_CUSTOM;
			s.sim->customGravityX = 0.4f;
			s.sim->customGravityY = 1.0f;
			for (int f = 0; f < 20; f++)
			{
				s.Run(1);
				float gx, gy;
				s.sim->GetGravityField(106, ground - 6, 1.0f, 1.0f, gx, gy);
				fprintf(stderr, "[%d x0=%d id=%d temp=%.0f type=%d] ", f, s.Of(0, 0, 250, YRES).x0, s.sim->parts[ID(s.sim->pmap[ground - 5][105])].tmp2, s.sim->parts[ID(s.sim->pmap[ground - 5][105])].temp, TYP(s.sim->pmap[ground - 5][105]));
			}
			fprintf(stderr, "\n");
			s.sim->gravityMode = GRAV_VERTICAL;
			s.Run(150);
			auto ice = s.Of(0, 0, 250, YRES), rubber = s.Of(250, 0, 590, YRES);
			Expect(ice.x0 - ice0.x0 > 15, "ice only slid " + std::to_string(ice.x0 - ice0.x0) + " px");
			Expect(std::abs(rubber.x0 - rubber0.x0) < 3, "rubber slid " + std::to_string(rubber.x0 - rubber0.x0) + " px");
		} },
		{ "different materials don't stick together", [](Scene &s) {
			s.Solid(20, ground, 590, ground + 5);
			s.Rigid(200, ground - 20, 40, 20, PT_RIGD);
			s.Rigid(210, ground - 32, 20, 12, PT_RWOD);
			s.Run(5);
			Expect(s.Bodies().size() == 2, "expected 2 bodies, got " + std::to_string(s.Bodies().size()));
			s.Run(200);
			Expect(s.Bodies().size() == 2, "they merged");
		} },
		{ "stone settles on mercury instead of bobbing", [](Scene &s) {
			s.Tank(150, 450, 330, 50, PT_MERC);
			s.Rigid(285, 150, 30, 14, PT_RSTN);
			s.Run(300);
			int lowest = 0, highest = YRES;
			for (int f = 0; f < 100; f++)
			{
				s.Run(1);
				auto b = s.Of();
				lowest = std::max(lowest, b.y1);
				highest = std::min(highest, b.y1);
			}
			Expect(lowest - highest <= 3, "still bobbing by " + std::to_string(lowest - highest) + " px");
		} },
		{ "hollow stone boat floats, solid stone sinks", [](Scene &s) {
			s.Tank(100, 500, 330, 60, PT_WATR);
			auto surface = s.Surface(100, 500, PT_WATR);
			// A U-shaped hull with 2 px walls, open at the top, set down just
			// above the water.
			s.Rigid(150, surface - 3, 60, 2, PT_RSTN);
			s.Rigid(150, surface - 21, 2, 18, PT_RSTN);
			s.Rigid(208, surface - 21, 2, 18, PT_RSTN);
			s.Rigid(380, surface - 11, 20, 10, PT_RSTN);
			s.Run(500);
			auto boat = s.Of(100, 0, 300, YRES), block = s.Of(320, 0, 500, YRES);
			Expect(boat.y1 < 320, "the boat sank: " + Describe(boat));
			Expect(block.y1 == 329, "the block didn't sink: " + Describe(block));
		} },
		{ "floating wood rests calmly instead of jittering", [](Scene &s) {
			s.Tank(150, 450, 330, 60, PT_WATR);
			s.Rigid(260, 200, 80, 10, PT_RWOD);
			s.Run(500);
			int lowest = 0, highest = YRES;
			// It may ride slow swells on TPT's uneven water, but its pixels
			// shouldn't flick back and forth from frame to frame.
			int moves = 0, last = s.Of().y1;
			for (int f = 0; f < 120; f++)
			{
				s.Run(1);
				auto b = s.Of();
				moves += b.y1 != last;
				last = b.y1;
				lowest = std::max(lowest, b.y1);
				highest = std::min(highest, b.y1);
			}
			Expect(lowest - highest <= 3, "bobbing by " + std::to_string(lowest - highest) + " px");
			Expect(moves <= 12, "moved " + std::to_string(moves) + " times in 120 frames");
		} },
		{ "water poured on one side levels out under a floating plank", [](Scene &s) {
			s.Tank(150, 450, 330, 50, PT_WATR);
			s.Rigid(190, 200, 220, 8, PT_RWOD);
			s.Run(300);
			// Pour a heap of water onto the left end.
			for (int y = 180; y < 200; y++)
			{
				for (int x = 152; x < 185; x++)
				{
					if (!s.sim->pmap[y][x])
					{
						s.sim->create_part(-1, x, y, PT_WATR);
					}
				}
			}
			s.Run(900);
			auto left = s.Surface(150, 160, PT_WATR), right = s.Surface(440, 450, PT_WATR);
			Expect(std::abs(left - right) <= 2, "water at " + std::to_string(left) + " on the left, " + std::to_string(right) + " on the right");
		} },
		{ "a sealed hollow wooden box stays dry inside", [](Scene &s) {
			s.Tank(150, 450, 330, 60, PT_WATR);
			// 40x30 box, 2 px walls.
			s.Rigid(280, 180, 40, 2, PT_RWOD);
			s.Rigid(280, 208, 40, 2, PT_RWOD);
			s.Rigid(280, 182, 2, 26, PT_RWOD);
			s.Rigid(318, 182, 2, 26, PT_RWOD);
			s.Run(600);
			Expect(s.Bodies().size() == 1, "expected the box to be one body");
			auto b = s.Of();
			// Water inside: cells strictly within the box's walls, judged along
			// each row between its leftmost and rightmost wood pixel.
			int inside = 0;
			for (int y = b.y0; y <= b.y1; y++)
			{
				int l = XRES, r = -1;
				for (int x = b.x0; x <= b.x1; x++)
				{
					auto q = s.sim->pmap[y][x];
					if (q && TYP(q) == PT_RWOD)
					{
						l = std::min(l, x);
						r = std::max(r, x);
					}
				}
				for (int x = l + 1; x < r; x++)
				{
					auto q = s.sim->pmap[y][x];
					inside += q && TYP(q) == PT_WATR;
				}
			}
			Expect(inside == 0, std::to_string(inside) + " water particles got inside");
			Expect(b.y1 < 320, "the box sank: " + Describe(b));
		} },
		{ "a lone pixel falls like powder instead of being a body", [](Scene &s) {
			s.Solid(20, ground, 590, ground + 5);
			s.sim->create_part(-1, 200, 100, PT_RIGD);
			s.Run(5);
			Expect(s.sim->rigidBodies->BodyCount() == 0, "it became a rigid body");
			s.Run(300);
			auto b = s.Of();
			Expect(b.count == 1 && b.y1 == ground - 1, "didn't land: " + Describe(b));
		} },
		{ "lone pixels sink or float by density", [](Scene &s) {
			s.sim->legacy_enable = 1;
			s.Tank(150, 450, 330, 60, PT_WATR);
			// One wood pixel at the bottom, one gold pixel at the top.
			auto wood = s.sim->create_part(-1, 200, 325, PT_WATR);
			s.sim->part_change_type(wood, 200, 325, PT_RWOD);
			auto gold = s.sim->create_part(-1, 300, 200, PT_RGLD);
			s.Run(400);
			auto surface = s.Surface(150, 450, PT_WATR);
			Expect(int(s.sim->parts[wood].y) <= surface + 1, "wood pixel didn't float up: y " + std::to_string(int(s.sim->parts[wood].y)) + ", surface " + std::to_string(surface));
			Expect(int(s.sim->parts[gold].y) == 329, "gold pixel didn't sink: y " + std::to_string(int(s.sim->parts[gold].y)));
		} },
		{ "dripping molten gold cools into grains, not hundreds of bodies", [](Scene &s) {
			s.Solid(20, ground, 590, ground + 5);
			// Separate drips of molten gold, just above its melting point,
			// spread out so they cool one by one.
			for (int y = 60; y < 180; y += 4)
			{
				for (int x = 150; x < 450; x += 9)
				{
					auto i = s.sim->create_part(-1, x + (y / 4) % 3, y, PT_LAVA);
					s.sim->parts[i].ctype = PT_RGLD;
					s.sim->parts[i].temp = 1340.0f;
					// As if it had melted off a body: it still carries that
					// body's tag.
					s.sim->parts[i].tmp2 = 1000 + (x / 9 + y / 4) % 7;
				}
			}
			s.sim->rigidBodies->timings = {};
			s.Run(600);
			auto gold = s.CountOf(PT_RGLD), lava = s.CountOf(PT_LAVA);
			auto bodies = s.sim->rigidBodies->BodyCount();
			auto &t = s.sim->rigidBodies->timings;
			auto ms = (t.gather + t.terrain + t.step + t.place) * 1000 / 600;
			fprintf(stderr, "gold %d, lava %d, bodies %d, %.2f ms/frame\n", gold, lava, bodies, ms);
			Expect(gold > 500, "only " + std::to_string(gold) + " drips solidified");
			Expect(bodies * 4 < gold, std::to_string(bodies) + " bodies for " + std::to_string(gold) + " gold pixels");
		} },
		{ "a gold block melting in hot lava stays cheap", [](Scene &s) {
			s.Solid(20, ground, 590, ground + 5);
			s.Solid(150, ground - 60, 152, ground - 1);
			s.Solid(450, ground - 60, 452, ground - 1);
			for (int y = ground - 25; y < ground; y++)
			{
				for (int x = 153; x < 450; x++)
				{
					auto i = s.sim->create_part(-1, x, y, PT_LAVA);
					s.sim->parts[i].ctype = PT_STNE;
					s.sim->parts[i].temp = 3000.0f;
				}
			}
			s.Rigid(250, 220, 80, 40, PT_RGLD);
			s.sim->rigidBodies->timings = {};
			double worst = 0;
			for (int f = 0; f < 600; f += 50)
			{
				auto &t = s.sim->rigidBodies->timings;
				auto before = t.gather + t.terrain + t.step + t.place;
				s.Run(50);
				auto spent = (t.gather + t.terrain + t.step + t.place - before) * 1000 / 50;
				worst = std::max(worst, spent);
				fprintf(stderr, "frame %d: gold %d, molten gold %d, bodies %d, %.2f ms/frame\n", f + 50, s.CountOf(PT_RGLD), [&] {
					int n = 0;
					for (int i = 0; i < s.sim->parts.active; i++)
					{
						n += s.sim->parts[i].type == PT_LAVA && s.sim->parts[i].ctype == PT_RGLD;
					}
					return n;
				}(), s.sim->rigidBodies->BodyCount(), spent);
			}
#ifdef __OPTIMIZE__
			auto limit = 3.0;
#else
			auto limit = 30.0; // unoptimised builds are around ten times slower
#endif
			Expect(worst < limit, "rigid bodies took up to " + std::to_string(worst) + " ms a frame while melting");
		} },
		{ "gold cargo stays inside a wooden ship, without gaps", [](Scene &s) {
			s.sim->legacy_enable = 1;
			s.Tank(60, 560, 330, 70, PT_WATR);
			// A closed hull with 5 px walls and a cabin at one end, the cabin
			// filled with gold, dropped in. It's top-heavy, so it heels over.
			auto outline = [&](int x0, int y0, int w, int h) {
				s.Rigid(x0, y0, w, 5, PT_RWOD);
				s.Rigid(x0, y0 + h - 5, w, 5, PT_RWOD);
				s.Rigid(x0, y0 + 5, 5, h - 10, PT_RWOD);
				s.Rigid(x0 + w - 5, y0 + 5, 5, h - 10, PT_RWOD);
			};
			outline(220, 200, 160, 40);
			outline(225, 165, 50, 35);
			s.Rigid(230, 170, 40, 30, PT_RGLD);
			s.Run(900);
			// Gold showing outside the ship: gold next to water or to empty
			// space that's open to the outside (as opposed to a gap inside).
			std::vector<char> open(XRES * YRES, 0);
			std::vector<int> queue{ CELL * XRES + CELL };
			open[queue[0]] = 1;
			for (size_t head = 0; head < queue.size(); head++)
			{
				auto x = queue[head] % XRES, y = queue[head] / XRES;
				for (auto [dx, dy] : { std::pair(0, -1), { -1, 0 }, { 1, 0 }, { 0, 1 } })
				{
					auto nx = x + dx, ny = y + dy;
					if (nx < 0 || nx >= XRES || ny < 0 || ny >= YRES || open[ny * XRES + nx])
					{
						continue;
					}
					auto r = s.sim->pmap[ny][nx];
					if (!r || TYP(r) == PT_WATR)
					{
						open[ny * XRES + nx] = 1;
						queue.push_back(ny * XRES + nx);
					}
				}
			}
			int exposed = 0, gaps = 0;
			auto isRigid = [&](int x, int y) {
				auto r = s.sim->pmap[y][x];
				return r && (TYP(r) == PT_RGLD || TYP(r) == PT_RWOD);
			};
			for (int y = 1; y < YRES - 1; y++)
			{
				for (int x = 1; x < XRES - 1; x++)
				{
					auto r = s.sim->pmap[y][x];
					if (r && TYP(r) == PT_RGLD)
					{
						for (auto [dx, dy] : { std::pair(0, -1), { -1, 0 }, { 1, 0 }, { 0, 1 } })
						{
							if (open[(y + dy) * XRES + x + dx])
							{
								exposed++;
								break;
							}
						}
					}
					// An empty cell shut in between gold and wood (the ship's own
					// hull is closed too, and notches in its inside wall don't count).
					auto gold = [&](int gx, int gy) {
						auto q = s.sim->pmap[gy][gx];
						return q && TYP(q) == PT_RGLD;
					};
					auto between = [&](int ax, int ay, int bx, int by) {
						return isRigid(ax, ay) && isRigid(bx, by) && (gold(ax, ay) || gold(bx, by));
					};
					if (!r && !open[y * XRES + x] && (between(x - 1, y, x + 1, y) || between(x, y - 1, x, y + 1)))
					{
						gaps++;
					}
				}
			}
			fprintf(stderr, "exposed gold %d, gaps %d, stuck %d\n", exposed, gaps, s.sim->rigidBodies->timings.stuck);
			Expect(exposed == 0, std::to_string(exposed) + " gold pixels showing outside the wood");
			Expect(gaps <= 2, std::to_string(gaps) + " gaps between gold and wood");
		} },
		{ "gold filling a wooden box sits in it cleanly", [](Scene &s) {
			s.sim->legacy_enable = 1;
			s.Tank(60, 560, 330, 90, PT_WATR);
			// Starts above the water and sinks to the bottom.
			s.Rigid(250, 140, 80, 6, PT_RWOD);
			s.Rigid(250, 204, 80, 6, PT_RWOD);
			s.Rigid(250, 146, 6, 58, PT_RWOD);
			s.Rigid(324, 146, 6, 58, PT_RWOD);
			s.Rigid(256, 146, 68, 58, PT_RGLD);
			s.Run(700);
			// Gaps: empty cells between gold and wood.
			int gaps = 0;
			auto at = [&](int x, int y) {
				auto q = s.sim->pmap[y][x];
				return q ? TYP(q) : 0;
			};
			for (int y = 100; y < 330; y++)
			{
				for (int x = 150; x < 450; x++)
				{
					if (at(x, y))
					{
						continue;
					}
					auto pair = [&](int a, int b) {
						return (a == PT_RGLD && b == PT_RWOD) || (a == PT_RWOD && b == PT_RGLD) || (a == PT_RGLD && b == PT_RGLD);
					};
					gaps += pair(at(x - 1, y), at(x + 1, y)) || pair(at(x, y - 1), at(x, y + 1));
				}
			}
			auto box = s.Of(150, 100, 450, 330);
			auto displaced = s.sim->rigidBodies->DisplacedPixels();
			fprintf(stderr, "box %s, gaps %d, displaced %d\n", Describe(box).c_str(), gaps, displaced);
			Expect(box.Width() == 80, "pixels sticking out of the box: " + Describe(box));
			Expect(gaps == 0, std::to_string(gaps) + " gaps between gold and wood");
			Expect(displaced == 0, std::to_string(displaced) + " pixels out of place");
		} },
		{ "gold drawn into a floating box sinks it cleanly", [](Scene &s) {
			s.sim->legacy_enable = 1;
			s.sim->CreateWallBox(56, 330, 564, 333, WL_WALL);
			s.Solid(57, 200, 59, 329);
			s.Solid(561, 200, 563, 329);
			for (int y = 240; y < 330; y++)
			{
				for (int x = 60; x <= 560; x++)
				{
					s.sim->create_part(-1, x, y, PT_WATR);
				}
			}
			s.Run(60);
			// An empty wooden box floats; then the inside is filled with gold.
			s.Rigid(250, 150, 80, 6, PT_RWOD);
			s.Rigid(250, 214, 80, 6, PT_RWOD);
			s.Rigid(250, 156, 6, 58, PT_RWOD);
			s.Rigid(324, 156, 6, 58, PT_RWOD);
			s.Run(400);
			auto box = s.Of();
			for (int y = box.y0; y <= box.y1; y++)
			{
				for (int x = box.x0; x <= box.x1; x++)
				{
					if (!s.sim->pmap[y][x])
					{
						// Only inside the box: between wood on the left and right.
						bool l = false, r = false;
						for (int xx = box.x0; xx < x; xx++) l |= s.sim->pmap[y][xx] && TYP(s.sim->pmap[y][xx]) == PT_RWOD;
						for (int xx = x + 1; xx <= box.x1; xx++) r |= s.sim->pmap[y][xx] && TYP(s.sim->pmap[y][xx]) == PT_RWOD;
						bool u = false, d = false;
						for (int yy = box.y0; yy < y; yy++) u |= s.sim->pmap[yy][x] && TYP(s.sim->pmap[yy][x]) == PT_RWOD;
						for (int yy = y + 1; yy <= box.y1; yy++) d |= s.sim->pmap[yy][x] && TYP(s.sim->pmap[yy][x]) == PT_RWOD;
						if (l && r && u && d)
						{
							s.sim->create_part(-1, x, y, PT_RGLD);
						}
					}
				}
			}
			s.Run(700);
			int gaps = 0;
			auto at = [&](int x, int y) {
				auto q = s.sim->pmap[y][x];
				return q ? TYP(q) : 0;
			};
			for (int y = 100; y < 330; y++)
			{
				for (int x = 150; x < 450; x++)
				{
					if (at(x, y))
					{
						continue;
					}
					auto pair = [&](int a, int b) {
						return (a == PT_RGLD && b == PT_RWOD) || (a == PT_RWOD && b == PT_RGLD) || (a == PT_RGLD && b == PT_RGLD);
					};
					gaps += pair(at(x - 1, y), at(x + 1, y)) || pair(at(x, y - 1), at(x, y + 1));
				}
			}
			auto all = s.Of(150, 100, 450, 330);
			int wx0 = XRES, wx1 = -1;
			for (int i = 0; i < s.sim->parts.active; i++)
			{
				auto &part = s.sim->parts[i];
				if (part.type == PT_RWOD)
				{
					wx0 = std::min(wx0, int(part.x));
					wx1 = std::max(wx1, int(part.x));
				}
			}
			auto displaced = s.sim->rigidBodies->DisplacedPixels();
			fprintf(stderr, "all %s, wood x %d..%d, gaps %d, displaced %d, stuck %d\n", Describe(all).c_str(), wx0, wx1, gaps, displaced, s.sim->rigidBodies->timings.stuck);
			Expect(wx1 - wx0 + 1 <= 81, "wood sticking out: x " + std::to_string(wx0) + ".." + std::to_string(wx1));
			Expect(gaps == 0, std::to_string(gaps) + " gaps between gold and wood");
			Expect(displaced == 0, std::to_string(displaced) + " pixels out of place");
		} },
		{ "a heeled-over ship doesn't dam the water", [](Scene &s) {
			s.sim->legacy_enable = 1;
			s.Tank(60, 560, 330, 70, PT_WATR);
			auto outline = [&](int x0, int y0, int w, int h) {
				s.Rigid(x0, y0, w, 5, PT_RWOD);
				s.Rigid(x0, y0 + h - 5, w, 5, PT_RWOD);
				s.Rigid(x0, y0 + 5, 5, h - 10, PT_RWOD);
				s.Rigid(x0 + w - 5, y0 + 5, 5, h - 10, PT_RWOD);
			};
			outline(220, 200, 160, 40);
			outline(225, 165, 50, 35);
			// Less gold than will sink it: it floats, heeled over.
			s.Rigid(230, 190, 20, 8, PT_RGLD);
			s.Run(1200);
			auto ship = s.Of();
			auto left = s.Surface(60, 70, PT_WATR), right = s.Surface(550, 560, PT_WATR);
			Expect(ship.y1 < 320, "the ship sank: " + Describe(ship));
			Expect(std::abs(left - right) <= 2, "water at " + std::to_string(left) + " on the left, " + std::to_string(right) + " on the right");
		} },
		{ "blocks falling into water stay rigid every frame", [](Scene &s) {
			s.sim->legacy_enable = 1;
			s.Tank(60, 560, 330, 110, PT_WATR);
			s.Rigid(250, 120, 50, 50, PT_RGLD);
			s.Run(50);
			s.Rigid(275, 60, 50, 50, PT_RGLD);
			int worst = 0;
			for (int f = 0; f < 300; f++)
			{
				s.Run(1);
				worst = std::max(worst, s.sim->rigidBodies->DisplacedPixels());
			}
			Expect(worst <= 6, "up to " + std::to_string(worst) + " pixels out of place in a frame");
		} },
		{ "a gold block dropped on another doesn't bounce", [](Scene &s) {
			s.Solid(20, ground, 590, ground + 5);
			s.Rigid(200, ground - 40, 60, 40, PT_RGLD);
			s.Run(30);
			s.Rigid(205, 150, 50, 40, PT_RGLD);
			int landed = 0, peak = YRES;
			for (int f = 0; f < 200; f++)
			{
				s.Run(1);
				auto top = s.Of(0, 0, XRES, ground - 41);
				if (top.y1 == ground - 41)
				{
					landed = 1;
				}
				else if (landed && top.count)
				{
					peak = std::min(peak, top.y1);
				}
			}
			Expect(landed, "never landed");
			Expect(peak >= ground - 42, "bounced " + std::to_string(ground - 41 - peak) + " px");
		} },
		{ "timing: a screen of water with 24 boxes in it", [](Scene &s) {
			s.check = false;
			auto frame = [&](Scene &scene, int frames) {
				auto start = std::chrono::steady_clock::now();
				scene.Run(frames);
				return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() * 1000 / frames;
			};
			auto fill = [](Scene &scene) {
				scene.sim->CreateWallBox(4, 374, 607, 379, WL_WALL);
				for (int y = 165; y < 374; y++)
				{
					for (int x = 5; x < 606; x++)
					{
						scene.sim->create_part(-1, x, y, PT_WATR);
					}
				}
				scene.Run(60);
			};
			fill(s);
			auto waterOnly = frame(s, 100);
			// The same boxes as plain iron, which TPT doesn't move: what the
			// particles cost TPT itself.
			Scene iron;
			iron.check = false;
			fill(iron);
			for (int i = 0; i < 24; i++)
			{
				iron.Rigid(20 + (i % 12) * 49, 30 + (i / 12) * 60, 36, 36, PT_IRON);
			}
			auto ironBoxes = frame(iron, 200);
			for (int i = 0; i < 24; i++)
			{
				s.Rigid(20 + (i % 12) * 49, 20 + (i / 12) * 60, 36, 36, PT_RGLD);
			}
			s.sim->rigidBodies->timings = {};
			auto falling = frame(s, 200);
			auto &t = s.sim->rigidBodies->timings;
			auto rigid = [&] { return (t.gather + t.terrain + t.step + t.place) * 5; };
			fprintf(stderr, "water only %.1f ms/frame; with 24 iron boxes %.1f; with 24 rigid gold boxes falling %.1f (rigid code %.2f)\n", waterOnly, ironBoxes, falling, rigid());
			s.sim->rigidBodies->timings = {};
			auto settled = frame(s, 200);
			fprintf(stderr, "settled %.1f ms/frame (rigid code %.2f: gather %.2f, terrain %.2f, step %.2f, place+levelling %.2f)\n", settled, rigid(), t.gather * 5, t.terrain * 5, t.step * 5, t.place * 5);
			// The same again with liquid levelling turned off.
			s.sim->rigidBodies->levelLiquids = false;
			s.sim->rigidBodies->timings = {};
			auto unlevelled = frame(s, 200);
			fprintf(stderr, "settled, levelling off: %.1f ms/frame (rigid code %.2f)\n", unlevelled, rigid());
		} },
		{ "frames: two gold blocks dropped into water (for looking at)", [](Scene &s) {
			s.sim->legacy_enable = 1;
			s.sim->CreateWallBox(56, 330, 564, 333, WL_WALL);
			s.Solid(57, 150, 59, 329);
			s.Solid(561, 150, 563, 329);
			for (int y = 220; y < 330; y++)
			{
				for (int x = 60; x <= 560; x++)
				{
					s.sim->create_part(-1, x, y, PT_WATR);
				}
			}
			s.Run(60);
			s.Rigid(250, 120, 50, 50, PT_RGLD);
			s.Run(50);
			s.Rigid(275, 60, 50, 50, PT_RGLD);
			for (int f = 0; f < 200; f += 4)
			{
				s.Run(4);
				char label[32];
				snprintf(label, sizeof(label), "drop-%03d", f + 4);
				s.Snapshot(label);
			}
		} },
		{ "showcase: every material in water and mercury (for looking at)", [](Scene &s) {
			s.sim->legacy_enable = 1;
			s.Tank(30, 300, 330, 70, PT_WATR);
			s.Tank(330, 580, 330, 50, PT_MERC);
			int types[] = { PT_RWOD, PT_RICE, PT_RFOM, PT_RRBR, PT_RIGD, PT_RSTN, PT_RGLD };
			for (int i = 0; i < 7; i++)
			{
				s.Rigid(40 + i * 37, 150 + (i % 2) * 20, 22, 12, types[i]);
				s.Rigid(340 + i * 34, 150 + (i % 2) * 20, 22, 12, types[i]);
			}
			s.Run(900);
		} },
		{ "held brush with a wobble makes one body", [](Scene &s) {
			s.Solid(20, ground, 590, ground + 5);
			for (int f = 0; f < 20; f++)
			{
				s.Circle(150 + f % 2, 100, 8);
				s.Run(1);
			}
			auto bodies = s.Bodies();
			Expect(bodies.size() == 1, "expected 1 body, got " + std::to_string(bodies.size()));
			s.Run(300);
			Expect(s.Bodies().size() == 1, "body broke up");
		} },
		{ "cutting a bridge splits it and both halves fall", [](Scene &s) {
			s.Solid(20, ground, 590, ground + 5);
			s.Solid(98, 250, 102, ground);
			s.Solid(158, 250, 162, ground);
			s.Rigid(95, 244, 71, 6);
			s.Run(30);
			for (int y = 0; y < YRES; y++)
			{
				auto r = s.sim->pmap[y][130];
				if (r && IsRigid(TYP(r)))
				{
					s.sim->kill_part(ID(r));
				}
			}
			s.Run(400);
			auto bodies = s.Bodies();
			Expect(bodies.size() == 2, "expected 2 bodies, got " + std::to_string(bodies.size()));
			for (auto &[id, b] : bodies)
			{
				Expect(b.y1 > 260, "a half didn't fall: " + Describe(b));
			}
		} },
		{ "box lands on a wall", [](Scene &s) {
			s.sim->CreateWallBox(100, 300, 300, 307, WL_WALL);
			s.Rigid(150, 200, 20, 20);
			s.Run(300);
			auto b = s.Of();
			Expect(b.y1 == 299 && b.Height() == 20, "box not resting on the wall: " + Describe(b));
		} },
		{ "baseline: same floor, no rigid bodies (for timing)", [](Scene &s) {
			s.Solid(20, ground, 590, ground + 5);
			s.Run(300);
		} },
		{ "huge body (30000 px): fall and rest (for timing)", [](Scene &s) {
			s.Solid(20, ground, 590, ground + 5);
			s.Rigid(150, 100, 200, 150);
			s.Run(300);
			auto b = s.Of();
			Expect(b.y1 == ground - 1 && b.count == 30000, "huge body: " + Describe(b));
		} },
		{ "huge circle painted with a held brush (for timing)", [](Scene &s) {
			s.Solid(20, ground, 590, ground + 5);
			for (int f = 0; f < 60; f++)
			{
				s.Circle(300 + f % 3, 150, 60 + f / 6);
				s.Run(1);
			}
			s.Run(240);
			Expect(s.Bodies().size() == 1, "expected 1 body, got " + std::to_string(s.Bodies().size()));
		} },
		{ "big body (3000 px) performance", [](Scene &s) {
			s.Solid(20, ground, 590, ground + 5);
			s.Rigid(150, 100, 100, 30);
			s.Run(300);
			auto b = s.Of();
			Expect(b.y1 == ground - 1 && b.count == 3000, "big body: " + Describe(b));
		} },
	};
}

int main(int argc, char *argv[])
{
	auto simulationData = std::make_unique<SimulationData>();
	int failed = 0;
	for (auto &scenario : scenarios)
	{
		if (argc > 1 && std::string(scenario.name).find(argv[1]) == std::string::npos)
		{
			continue;
		}
		Scene scene;
		auto start = std::chrono::steady_clock::now();
		std::string error;
		try
		{
			scenario.run(scene);
		}
		catch (const std::string &message)
		{
			error = message;
		}
		auto seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
		printf("%-55s %s (%.1fs)\n", scenario.name, error.empty() ? "ok" : ("FAIL: " + error).c_str(), seconds);
		// To look at the end of each scenario, set RIGID_BODIES_SAVES to a
		// directory and render the saves with the render tool.
		if (auto dir = std::getenv("RIGID_BODIES_SAVES"))
		{
			auto save = scene.sim->Save(true, RES.OriginRect());
			auto data = save->Serialise().second;
			if (!data.empty())
			{
				std::string name = scenario.name;
				for (auto &c : name)
				{
					c = std::isalnum((unsigned char)c) ? c : '-';
				}
				Platform::WriteFile(data, ByteString(dir) + "/" + ByteString(name) + ".cps");
			}
		}
		auto &t = scene.sim->rigidBodies->timings;
		printf("    rigid bodies: gather %.0f ms, terrain %.0f ms, step %.0f ms, place %.0f ms, stuck %d, pulled back %d, displaced at end %d\n", t.gather * 1000, t.terrain * 1000, t.step * 1000, t.place * 1000, t.stuck, t.pulled, scene.sim->rigidBodies->DisplacedPixels());
		printf("    placed %ld, pending %ld, search steps %ld\n", t.placed, t.pending, t.searchSteps);
		failed += !error.empty();
	}
	return failed ? 1 : 0;
}
